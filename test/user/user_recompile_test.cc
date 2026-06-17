// Copyright 2026 DeepMind Technologies Limited
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

// Tests for recompiling multiple files.

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <filesystem>  // NOLINT
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/match.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include "src/xml/xml_api.h"
#include "src/xml/xml_numeric_format.h"
#include "test/compare_model.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;

std::vector<std::string> GetRecompileTestModels() {
  std::vector<std::string> models;
  std::string ext(".xml");
  for (const auto& path : {GetTestDataFilePath("."), GetModelPath(".")}) {
    for (const auto& p : std::filesystem::recursive_directory_iterator(path)) {
      if (p.path().extension() == ext) {
        std::string xml = p.path().string();
        if (absl::StrContains(xml, "malformed_") ||
            absl::StrContains(xml, "_fail") ||
            absl::StrContains(xml, "touch_grid") ||
            absl::StrContains(xml, "perf") || absl::StrContains(xml, "cow") ||
            // exclude conflict test assets (designed to fail compile)
            absl::StrContains(xml, "xml/testdata/parent_")) {
          continue;
        }
        models.push_back(xml);
      }
    }
  }
  return models;
}

class RecompileCompareTest : public MujocoTest,
                             public ::testing::WithParamInterface<std::string> {
 public:
};
TEST_P(RecompileCompareTest, RecompileCompare) {
  std::string xml = GetParam();
  std::string field = "";

  FullFloatPrecision increase_precision;

  // load spec
  std::array<char, 1000> err;
  mjSpec* s = mj_parseXML(xml.c_str(), 0, err.data(), err.size());

  if (!s) {
    GTEST_SKIP() << "Failed to load " << xml << ": " << err.data();
  }

  // copy spec
  mjSpec* s_copy = mj_copySpec(s);

  // compare signature
  EXPECT_EQ(s->element->signature, s_copy->element->signature) << xml;

  // compile twice and compare
  mjModel* m_old = mj_compile(s, nullptr);

  if (!m_old) {
    mj_deleteSpec(s);
    GTEST_SKIP() << "Failed to compile " << xml << ": " << mjs_getError(s);
  }

  mjModel* m_new = mj_compile(s, nullptr);
  mjModel* m_copy = mj_compile(s_copy, nullptr);

  // compare signature
  EXPECT_EQ(m_old->signature, m_new->signature) << xml;
  EXPECT_EQ(m_old->signature, m_copy->signature) << xml;

  ASSERT_THAT(m_new, NotNull())
      << "Failed to recompile " << xml << ": " << mjs_getError(s);
  ASSERT_THAT(m_copy, NotNull())
      << "Failed to compile " << xml << ": " << mjs_getError(s_copy);

  mjtNum tol = 0;

  EXPECT_LE(CompareModel(m_old, m_new, field), tol)
      << "Compiled and recompiled models are different!\n"
      << "Affected file " << xml << '\n'
      << "Different field: " << field << '\n';

  EXPECT_LE(CompareModel(m_old, m_copy, field), tol)
      << "Original and copied models are different!\n"
      << "Affected file " << xml << '\n'
      << "Different field: " << field << '\n';

  // copy to a new spec, compile and compare
  mjSpec* s_copy2 = mj_copySpec(s);
  mjModel* m_copy2 = mj_compile(s_copy2, nullptr);

  ASSERT_THAT(m_copy2, NotNull())
      << "Failed to compile " << xml << ": " << mjs_getError(s_copy2);

  EXPECT_LE(CompareModel(m_old, m_copy2, field), tol)
      << "Original and re-copied models are different!\n"
      << "Affected file " << xml << '\n'
      << "Different field: " << field << '\n';

  mj_deleteModel(m_new);
  mj_deleteModel(m_copy);
  mj_deleteModel(m_copy2);
  mj_deleteSpec(s_copy);
  mj_deleteSpec(s_copy2);
  mj_deleteSpec(s);
  mj_deleteModel(m_old);
}

INSTANTIATE_TEST_SUITE_P(
    AllModels, RecompileCompareTest,
    ::testing::ValuesIn(GetRecompileTestModels()),
    [](const ::testing::TestParamInfo<std::string>& info) {
      std::string name = std::filesystem::path(info.param).filename().string();
      std::replace_if(
          name.begin(), name.end(),
          [](char c) { return !std::isalnum(c); }, '_');
      return name + "_" + std::to_string(info.index);
    });

}  // namespace
}  // namespace mujoco
