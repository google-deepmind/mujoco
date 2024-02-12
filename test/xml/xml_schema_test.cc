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
#include <string>

#include <gtest/gtest.h>
#include "src/xml/xml_native_reader.h"
#include "src/xml/xml_util.h"
#include "test/fixture.h"
#include <absl/strings/numbers.h>
#include <absl/strings/str_format.h>

namespace mujoco {
namespace {


using XMLSchemaTest = MujocoTest;

static std::optional<std::string> IsValidSchemaFormat(
    const char* schema[][mjXATTRNUM], unsigned nrow) {
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
      // first 3 pointers required
      if (!schema[i][1] || !schema[i][2]) {
        return absl::StrFormat("expected element, found null pointers"
                               "in row %d, element %s", i, schema[i][0]);
      }

      // check type
      if (schema[i][1][0] != '!' && schema[i][1][0] != '?' &&
          schema[i][1][0] != '*' && schema[i][1][0] != 'R') {
        return absl::StrFormat("invalid type in row %d, element %s",
                               i, schema[i][0]);
      }

      // number of attributes
      int nattr = 0;
      if (!absl::SimpleAtoi(schema[i][2], &nattr)) {
        return absl::StrFormat("unparseable number of attributes in"
                               " row %d, element %s", i, schema[i][0]);
      } else if (nattr < 0 || nattr > mjXATTRNUM - 3) {
        return absl::StrFormat("invalid number of attributes in"
                               " row %d, element %s", i, schema[i][0]);
      }

      // attribute pointers
      for (int j = 0; j < nattr; j++) {
        if (!schema[i][3 + j]) {
          return absl::StrFormat("null attribute %d in"
                                 " row %d, element %s", j, i, schema[i][0]);
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

}  // namespace
}  // namespace mujoco
