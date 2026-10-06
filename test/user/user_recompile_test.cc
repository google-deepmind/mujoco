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
#include <sstream>
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
#include "test/compare_spec.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::IsEmpty;
using ::testing::NotNull;

std::vector<std::string> GetRecompileTestModels() {
  std::vector<std::string> models;
  std::string ext(".xml");
  for (const auto& path : {GetTestDataFilePath("."), GetModelPath(".")}) {
    for (const auto& p : std::filesystem::recursive_directory_iterator(path)) {
      if (p.path().extension() == ext) {
        // generic format, so patterns containing '/' also match on Windows
        std::string xml = p.path().generic_string();
        if (absl::StrContains(xml, "malformed_") ||
            absl::StrContains(xml, "_fail") ||
            absl::StrContains(xml, "touch_grid") ||
            absl::StrContains(xml, "perf") || absl::StrContains(xml, "cow") ||
#ifndef MJ_WITH_USD
            absl::StrContains(xml, "usd.xml") ||
#endif
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

// Differences between two specs, apart from those of a compilation completing
// keyframes: it sizes their vectors for the model, and adds keyframes up to
// the number set by nkey.
std::string WithoutKeyframeCompletion(const std::string& differences) {
  std::istringstream lines(differences);
  std::string line;
  std::string other;
  while (std::getline(lines, line)) {
    bool vector =
        absl::StartsWith(line, "key[") && absl::StrContains(line, " size: ");
    bool number = absl::StartsWith(line, "key: count: ");
    if (!vector && !number) other += line + '\n';
  }
  return other;
}

// The spec as it is saved, empty if it cannot be saved.
std::string SaveToString(const mjSpec* s) {
  std::array<char, 1000> err;
  int size = mj_saveXMLString(s, nullptr, 0, err.data(), err.size());
  if (size <= 0) return "";
  std::string xml(size + 1, '\0');
  mj_saveXMLString(s, xml.data(), size + 1, err.data(), err.size());
  xml.resize(size);
  return xml;
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

  // copy spec, the copy has what was authored
  mjSpec* s_copy = mj_copySpec(s);
  const int kAllDifferences = 100000;
  EXPECT_THAT(CompareSpec(s, s_copy, kAllDifferences), IsEmpty()) << xml;

  // an uncompiled spec has no signature
  EXPECT_EQ(s->element->signature, 0) << xml;
  EXPECT_EQ(s_copy->element->signature, 0) << xml;

  // compile twice and compare
  mjModel* m_old = mj_compile(s, nullptr);

  if (!m_old) {
    std::string error_message = mjs_getError(s);
    mj_deleteSpec(s_copy);
    mj_deleteSpec(s);
    GTEST_SKIP() << "Failed to compile " << xml << ": " << error_message;
  }

  // compiling leaves what was authored as it was, unless it restructures
  if (!s->compiler.fusestatic && !s->compiler.discardvisual) {
    EXPECT_THAT(
        WithoutKeyframeCompletion(CompareSpec(s_copy, s, kAllDifferences)),
        IsEmpty())
        << xml;
  }

  // the elements of a compiled spec hold what the model was given, so copying
  // the model back changes nothing in the spec, nor in what is saved
  std::string unchanged = SaveToString(s);
  mjSpec* s_before = mj_copySpec(s);
  EXPECT_EQ(mj_copyBack(s, m_old), 1) << xml << ": " << mjs_getError(s);
  EXPECT_THAT(CompareSpec(s_before, s, kAllDifferences), IsEmpty()) << xml;
  EXPECT_EQ(SaveToString(s), unchanged) << xml;

  // and so do the elements of a copy of the spec
  mjSpec* s_copied = mj_copySpec(s);
  EXPECT_EQ(mj_copyBack(s_copied, m_old), 1)
      << xml << ": " << mjs_getError(s_copied);
  EXPECT_THAT(CompareSpec(s_before, s_copied, kAllDifferences), IsEmpty())
      << xml;
  mj_deleteSpec(s_copied);
  mj_deleteSpec(s_before);

  // a spec which was compiled, and restructured if it asks for it, is left as
  // it is by the next compilation
  mjSpec* s_compiled = mj_copySpec(s);
  mjModel* m_new = mj_compile(s, nullptr);
  EXPECT_THAT(
      WithoutKeyframeCompletion(CompareSpec(s_compiled, s, kAllDifferences)),
      IsEmpty())
      << xml;
  mj_deleteSpec(s_compiled);
  mjModel* m_copy = mj_compile(s_copy, nullptr);

  // compare signature
  EXPECT_EQ(m_old->signature, m_new->signature) << xml;
  EXPECT_EQ(m_old->signature, m_copy->signature) << xml;

  // compiling refreshes the signature of the spec
  EXPECT_EQ(s->element->signature, m_new->signature) << xml;
  EXPECT_EQ(s_copy->element->signature, m_copy->signature) << xml;

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

  // a copy of a compiled spec holds what was authored in the original, and is
  // saved as the original is, also once the original is deleted
  std::string saved = SaveAndReadXml(s);
  mjSpec* s_copy3 = mj_copySpec(s);
  EXPECT_THAT(CompareSpec(s, s_copy3, kAllDifferences), IsEmpty()) << xml;

  mj_deleteModel(m_new);
  mj_deleteModel(m_copy);
  mj_deleteModel(m_copy2);
  mj_deleteSpec(s_copy);
  mj_deleteSpec(s_copy2);
  mj_deleteSpec(s);
  mj_deleteModel(m_old);

  EXPECT_EQ(SaveAndReadXml(s_copy3), saved)
      << "Original and copied specs are saved differently!\n"
      << "Affected file " << xml << '\n';
  mj_deleteSpec(s_copy3);
}

// A spec which is saved as it was written reads back as a spec which is saved
// the same and compiles to the same model, in both notations.
TEST_P(RecompileCompareTest, SavedAsWritten) {
  std::string xml = GetParam();
  std::array<char, 1000> err;
  mjSpec* s = mj_parseXML(xml.c_str(), 0, err.data(), err.size());
  if (!s) {
    GTEST_SKIP() << "Failed to load " << xml << ": " << err.data();
  }

  // a spec is saved as it was written before it is compiled, unless its
  // keyframes wait for a compilation to be laid out for the model
  s->compiler.savecompiled = 0;
  s->compiler.savecanonical = 0;
  std::string uncompiled = SaveToString(s);

  mjModel* m = mj_compile(s, nullptr);
  if (!m) {
    std::string error_message = mjs_getError(s);
    mj_deleteSpec(s);
    GTEST_SKIP() << "Failed to compile " << xml << ": " << error_message;
  }

  for (bool canonical : {false, true}) {
    s->compiler.savecanonical = canonical;
    std::string saved = SaveToString(s);
    ASSERT_FALSE(saved.empty()) << xml;

    // compiling changes what is saved only by completing keyframes and by
    // restructuring
    if (!canonical && !uncompiled.empty() && !m->nkey &&
        !s->compiler.fusestatic && !s->compiler.discardvisual) {
      EXPECT_EQ(uncompiled, saved) << xml;
    }

    // the saved file is read from where the model is, to find its assets
    mjSpec* r = mj_parseXMLString(saved.c_str(), 0, err.data(), err.size());
    ASSERT_THAT(r, NotNull()) << xml << ": " << err.data() << '\n' << saved;
    mjs_setString(r->modelfiledir, mjs_getString(s->modelfiledir));
    mjModel* m_saved = mj_compile(r, nullptr);
    ASSERT_THAT(m_saved, NotNull()) << xml << ": " << mjs_getError(r);

    std::string field = "";
    EXPECT_EQ(CompareModel(m, m_saved, field), 0)
        << "Model and model saved as written are different!\n"
        << "Affected file " << xml << '\n'
        << "Canonical notation: " << canonical << '\n'
        << "Different field: " << field << '\n';

    r->compiler.savecompiled = 0;
    r->compiler.savecanonical = canonical;
    EXPECT_EQ(SaveToString(r), saved) << xml;

    mj_deleteModel(m_saved);
    mj_deleteSpec(r);
  }

  mj_deleteModel(m);
  mj_deleteSpec(s);
}

INSTANTIATE_TEST_SUITE_P(
    AllModels, RecompileCompareTest,
    ::testing::ValuesIn(GetRecompileTestModels()),
    [](const ::testing::TestParamInfo<std::string>& info) {
      std::string name = std::filesystem::path(info.param).filename().string();
      std::replace_if(
          name.begin(), name.end(), [](char c) { return !std::isalnum(c); },
          '_');
      return name + "_" + std::to_string(info.index);
    });

}  // namespace
}  // namespace mujoco
