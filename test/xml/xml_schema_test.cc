// Copyright 2024 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <optional>
#include <sstream>
#include <string>
#include <thread>  // NOLINT(build/c++11)
#include <vector>

#include <gtest/gtest.h>
#include "absl/strings/str_format.h"
#include "src/xml/xml_native_reader.h"
#include "src/xml/xml_util.h"
#include "test/fixture.h"
#include "tinyxml2.h"

namespace mujoco {
namespace {

using XMLSchemaTest = MujocoTest;

static std::optional<std::string> IsValidSchemaFormat(
    std::vector<const char*> schema[], unsigned nrow) {
  if (schema[0][0][0] == '<' || schema[0][0][0] == '>') {
    return "expected element, found bracket";
  }

  // check entire schema for null pointers
  for (int i = 0; i < nrow; i++) {
    // base pointers
    if (!schema[i][0]) {
      return "null pointer found in row " + std::to_string(i);
    }

    // detect element
    if (schema[i][0][0] != '<' && schema[i][0][0] != '>') {
      // first 2 pointers required
      if (!schema[i][1]) {
        return absl::StrFormat(
            "expected element, found null pointers"
            "in row %d, element %s",
            i, schema[i][0]);
      }

      // check type
      if (schema[i][1][0] != '!' && schema[i][1][0] != '?' &&
          schema[i][1][0] != '*' && schema[i][1][0] != 'R') {
        return absl::StrFormat("invalid type in row %d, element %s", i,
                               schema[i][0]);
      }

      // attribute pointers
      for (int j = 0; j < schema[i].size() - 2; j++) {
        if (!schema[i][2 + j]) {
          return absl::StrFormat(
              "null attribute %d in"
              " row %d, element %s",
              j, i, schema[i][0]);
        }
      }
    }
  }

  // process sub-elements of complex element
  if (nrow > 1) {
    // check for bracketed block
    if (schema[1][0][0] != '<' || schema[nrow - 1][0][0] != '>') {
      return "expected brackets after complex element";
    }

    // parse block into simple and complex elements
    int start = 2;
    while (start < nrow - 1) {
      int end = start;

      // look for bracketed block at start + 1
      if (schema[start + 1][0][0] == '<') {
        // look for corresponding closing bracket
        int cnt = 0;
        while (end <= nrow - 1) {
          if (schema[end][0][0] == '<') {
            cnt++;
          } else if (schema[end][0][0] == '>') {
            cnt--;
            if (cnt == 0) {
              break;
            }
          }
          end++;
        }

        // closing bracket not found
        if (end > nrow - 1) {
          return "matching closing bracket not found";
        }
      }

      // recursively check for error
      auto error = IsValidSchemaFormat(schema + start, end - start + 1);
      if (error.has_value()) {
        return error;
      }

      // proceed with next subelement
      start = end + 1;
    }
  }
  return std::nullopt;
}

TEST_F(XMLSchemaTest, MjcfSchemaTest) {
  auto error = IsValidSchemaFormat(MJCF, nMJCF);
  auto error_msg = error.value_or("");

  EXPECT_EQ(error_msg, "");
  ASSERT_FALSE(error.has_value());
}

TEST_F(XMLSchemaTest, DeduplicatesAttributesAndHandlesNullElement) {
  std::vector<const char*> rows[] = {
      {"item", "!", "pos", "name", "pos"},
  };
  mjXSchema schema(rows, 1);

  std::stringstream out;
  schema.Print(out, 0);
  EXPECT_EQ(out.str(), "item (!)                      name pos \n");

  std::string error = "stale";
  EXPECT_EQ(schema.Check(nullptr, 0, error), nullptr);
  EXPECT_EQ(error, "");

  std::vector<const char*> recursive_rows[] = {
      {"item", "R", "name"},
  };
  mjXSchema recursive_schema(recursive_rows, 1);
  tinyxml2::XMLDocument ns_doc;
  ns_doc.Parse(R"(
  <item xmlns="http://example.com">
    <item xmlns="http://example.com"/>
  </item>
  )");
  EXPECT_NE(recursive_schema.Check(ns_doc.RootElement(), 0, error), nullptr);
  EXPECT_EQ(error, "unrecognized attribute: 'xmlns'");
}

TEST_F(XMLSchemaTest, ConcurrentSchemaCheck) {
  static constexpr char kValidXml[] = R"(
  <mujoco model="concurrent">
    <default>
      <default class="sub">
        <geom size="0.1"/>
      </default>
    </default>
    <worldbody>
      <body name="b1" pos="0 0 1">
        <joint type="hinge"/>
        <geom class="sub"/>
        <body name="b2" pos="0.2 0 0">
          <joint type="slide"/>
          <geom size="0.05"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  static constexpr char kInvalidAttrXml[] = R"(
  <mujoco>
    <worldbody>
      <geom unknown_attr="1"/>
    </worldbody>
  </mujoco>
  )";
  static constexpr char kDuplicateUniqueXml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <inertial mass="1"/>
        <inertial mass="2"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  const mjXSchema schema(MJCF, nMJCF, MJCF_constraints, nMJCF_constraints);
  ASSERT_EQ(schema.GetError(), "");

  constexpr int kNumThreads = 8;
  constexpr int kIters = 50;
  std::vector<int> failures(kNumThreads, 0);
  std::vector<std::thread> threads;
  threads.reserve(kNumThreads);
  for (int t = 0; t < kNumThreads; ++t) {
    threads.emplace_back([t, &schema, &failures]() {
      std::string err;
      for (int i = 0; i < kIters; ++i) {
        tinyxml2::XMLDocument valid_doc;
        valid_doc.Parse(kValidXml);
        if (schema.Check(valid_doc.RootElement(), 0, err) != nullptr ||
            !err.empty()) {
          failures[t]++;
        }

        tinyxml2::XMLDocument bad_attr_doc;
        bad_attr_doc.Parse(kInvalidAttrXml);
        if (schema.Check(bad_attr_doc.RootElement(), 0, err) == nullptr ||
            err != "unrecognized attribute: 'unknown_attr'") {
          failures[t]++;
        }

        tinyxml2::XMLDocument dup_doc;
        dup_doc.Parse(kDuplicateUniqueXml);
        if (schema.Check(dup_doc.RootElement(), 0, err) == nullptr ||
            err != "unique element 'inertial' found 2 times") {
          failures[t]++;
        }
      }
    });
  }
  for (std::thread& th : threads) {
    th.join();
  }
  for (int t = 0; t < kNumThreads; ++t) {
    EXPECT_EQ(failures[t], 0) << "thread=" << t;
  }
}

}  // namespace
}  // namespace mujoco
