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

// Tests for loading and saving multiple files.

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <filesystem>  // NOLINT(build/c++17)
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/match.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/xml/xml_numeric_format.h"
#include "test/compare_model.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using XMLWriterTest = MujocoTest;

std::vector<std::string> GetWriteReadTestModels() {
  std::vector<std::string> models;
  std::string ext(".xml");
  for (const auto& path : {GetTestDataFilePath("."), GetModelPath(".")}) {
    for (const auto& p : std::filesystem::recursive_directory_iterator(path)) {
      if (p.path().extension() == ext) {
        std::string xml = p.path().string();
        if (  // if file is meant to fail, skip it
            absl::StrContains(xml, "malformed_") ||
            absl::StrContains(xml, "_fail") ||
            // exclude files that are too slow to load
            absl::StrContains(xml, "cow") || absl::StrContains(xml, "gmsh_") ||
            absl::StrContains(xml, "shark_") ||
            absl::StrContains(xml, "perf") ||
            // exclude files that fail the comparison test
            absl::StrContains(xml, "rfcamera") ||
            absl::StrContains(xml, "tactile") ||
            absl::StrContains(xml, "makemesh") ||
            absl::StrContains(xml, "many_dependencies") ||
            absl::StrContains(xml, "usd") ||
            absl::StrContains(xml, "torus_maxhull") ||
            absl::StrContains(xml, "fitmesh_") ||
            absl::StrContains(xml, "lengthrange") ||
            absl::StrContains(xml, "hfield_xml") ||
            absl::StrContains(xml, "fromto_convex") ||
            absl::StrContains(xml, "cube_skin") ||
            absl::StrContains(xml, "cube_3x3x3") ||
            // exclude files that fail since we do not save pinned flex nodes
            absl::StrContains(xml, "gripper_trilinear") ||
            absl::StrContains(xml, "strain") ||
            // exclude conflict tests (known option conflict warnings/errors)
            absl::StrContains(xml, "xml/testdata/parent_")) {
          continue;
        }
        models.push_back(xml);
      }
    }
  }
  return models;
}

class WriteReadCompareTest : public XMLWriterTest,
                             public ::testing::WithParamInterface<std::string> {
 public:
};
TEST_P(WriteReadCompareTest, WriteReadCompare) {
  std::string xml = GetParam();

  // full precision float printing
  FullFloatPrecision increase_precision;

  // load model
  std::array<char, 1000> error;
  mjSpec* s =
      mj_parseXML(xml.c_str(), nullptr, error.data(), error.size());
  if (!s) {
    GTEST_SKIP() << "Failed to load " << xml.c_str() << ": " << error.data();
  }

  mjModel* m = mj_compile(s, nullptr);
  if (!m) {
    mj_deleteSpec(s);
    GTEST_SKIP() << "Failed to compile " << xml.c_str() << ": " << error.data();
  }

  // make data
  mjData* d = mj_makeData(m);
  ASSERT_THAT(d, testing::NotNull()) << "Failed to create data\n";

  // save and load back
  auto abs_path = std::filesystem::path(xml);
  mjSpec* stemp = mj_parseXMLString(SaveAndReadXml(s).c_str(), 0, error.data(),
                                    error.size());
  ASSERT_THAT(stemp, NotNull())
      << "Failed to load " << xml.c_str() << ": " << error.data();
  mjs_setString(stemp->modelfiledir,
                abs_path.remove_filename().string().c_str());
  mjModel* mtemp = mj_compile(stemp, nullptr);

  ASSERT_THAT(mtemp, NotNull()) << error.data() << " from " << xml.c_str();

  mjtNum tol = 0;

  // for particularly sensitive models, relax the tolerance
  if (absl::StrContains(xml, "belt.xml") ||
      absl::StrContains(xml, "cable.xml")) {
    tol = 1e-13;
  }

  // compare and delete
  std::string field = "";
  mjtNum result = CompareModel(m, mtemp, field);
  EXPECT_LE(result, tol) << "Loaded and saved models are different!\n"
                         << "Affected file " << xml << '\n'
                         << "Different field: " << field << '\n';
  mj_deleteModel(mtemp);

  // check for stack memory leak
  mj_step(m, d);
  EXPECT_EQ(d->pstack, 0) << "mjData stack memory leak detected in " <<
      xml << '\n';

  // delete data
  mj_deleteData(d);

  // allocate buffer, save m into it
  size_t sz = mj_sizeModel(m);
  void* buffer = mju_malloc(sz);
  mj_saveModel(m, nullptr, buffer, sz);

  // make new VFS add buffer to it
  mjVFS* vfs = (mjVFS*)mju_malloc(sizeof(mjVFS));
  mj_defaultVFS(vfs);
  int failed = mj_addBufferVFS(vfs, "model.mjb", buffer, sz);
  EXPECT_EQ(failed, 0) << "Failed to add buffer to VFS";

  // load model from VFS
  mtemp = mj_loadModel("model.mjb", vfs);
  ASSERT_THAT(mtemp, NotNull());

  // compare with 0 tolerance
  field = "";
  result = CompareModel(m, mtemp, field);
  EXPECT_EQ(result, 0) << "Loaded and saved binary models are different!\n"
                       << "Affected file " << xml << '\n'
                       << "Different field: " << field << '\n';

  // clean up
  mj_deleteSpec(s);
  mj_deleteSpec(stemp);
  mj_deleteModel(m);
  mj_deleteModel(mtemp);
  mj_deleteVFS(vfs);
  mju_free(vfs);
  mju_free(buffer);
}

INSTANTIATE_TEST_SUITE_P(
    AllModels, WriteReadCompareTest,
    ::testing::ValuesIn(GetWriteReadTestModels()),
    [](const ::testing::TestParamInfo<std::string>& info) {
      std::string name = std::filesystem::path(info.param).filename().string();
      std::replace_if(
          name.begin(), name.end(),
          [](char c) { return !std::isalnum(c); }, '_');
      return name + "_" + std::to_string(info.index);
    });

}  // namespace
}  // namespace mujoco
