// Copyright 2021 DeepMind Technologies Limited
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

// Tests for user/user_api.cc.

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/match.h>
#include <mujoco/mujoco.h>
#include "src/user/user_api.h"
#include "src/xml/xml.h"
#include "src/xml/xml_numeric_format.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::NotNull;


// ----------------------------- test set/get  --------------------------------

TEST_F(MujocoTest, ReadWriteData) {
  mjSpec* spec = mjm_createSpec();
  mjmBody* world = mjm_findBody(spec, "world");
  mjmBody* body = mjm_addBody(world, 0);
  mjmSite* site = mjm_addSite(body, 0);

  {
    double vec[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    const char* str = "sitename";

    mjm_setString(site->name, str);
    mjm_setDouble(site->userdata, vec, 10);
  }

  EXPECT_THAT(mjm_getString(site->name), HasSubstr("sitename"));

  int nsize;
  const double* vec = mjm_getDouble(site->userdata, &nsize);
  for (int i = 0; i < nsize; ++i) {
    EXPECT_EQ(vec[i], i);
  }

  mjm_deleteSpec(spec);
}

// ------------------- test recompilation multiple files ----------------------
TEST_F(PluginTest, RecompileCompare) {
  // full precision float printing
  FullFloatPrecision increase_precision;

  // loop over all xml files in data
  std::vector<std::string> paths = {GetTestDataFilePath("."),
                                    GetModelPath(".")};
  std::string ext(".xml");
  for (auto const& path : paths) {
    for (auto &p : std::filesystem::recursive_directory_iterator(path)) {
      if (p.path().extension() == ext) {
        std::string xml = p.path().string();

        // if file is meant to fail, skip it
        if (absl::StrContains(p.path().string(), "malformed_") ||
            absl::StrContains(p.path().string(), "touch_grid") ||
            absl::StrContains(p.path().string(), "cow") ||
            absl::StrContains(p.path().string(), "discardvisual")) {
          continue;
        }

        // load model
        std::array<char, 1000> error;
        mjSpec* spec =
            mjParseXML(xml.c_str(), nullptr, error.data(), error.size());

        // compile twice
        mjModel* m_old = mjm_compile(spec, nullptr);
        mjModel* m_new = mjm_compile(spec, nullptr);

        ASSERT_THAT(m_old, NotNull())
            << "Failed to compile " << xml << ": " << error.data();
        ASSERT_THAT(m_new, NotNull())
            << "Failed to recompile " << xml << ": " << error.data();

        // compare and delete
        std::string field = "";
        mjtNum result = CompareModel(m_old, m_new, field);
        mjtNum tol = 0;
        EXPECT_LE(result, tol)
            << "Loaded and saved models are different!\n"
            << "Affected file " << p.path().string() << '\n'
            << "Different field: " << field << '\n';

        // delete models
        mjm_deleteSpec(spec);
        mj_deleteModel(m_old);
        mj_deleteModel(m_new);
      }
    }
  }
}

// ------------------- test cache with modified assets -------------------------
TEST_F(PluginTest, RecompileCompareCache) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh file="cube.obj"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="cube"/>
    </worldbody>
  </mujoco>)";

  static constexpr char cube1[] = R"(
  v -0.500000 -0.500000  0.500000
  v  0.500000 -0.500000  0.500000
  v -0.500000  0.500000  0.500000
  v  0.500000  0.500000  0.500000
  v -0.500000  0.500000 -0.500000
  v  0.500000  0.500000 -0.500000
  v -0.500000 -0.500000 -0.500000
  v  0.500000 -0.500000 -0.500000)";

  static constexpr char cube2[] = R"(
  v -1 -1  1
  v  1 -1  1
  v -1  1  1
  v  1  1  1
  v -1  1 -1
  v  1  1 -1
  v -1 -1 -1
  v  1 -1 -1)";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "cube.obj", cube1, sizeof(cube1));

  std::array<char, 1024> error;

  // load model once
  mjModel* m = LoadModelFromString(xml, error.data(), error.size(), vfs.get());
  EXPECT_EQ(m->mesh_vert[0], -0.5);
  mj_deleteModel(m);

  // update cube.obj, load again
  mj_deleteFileVFS(vfs.get(), "cube.obj");
  mj_addBufferVFS(vfs.get(), "cube.obj", cube2, sizeof(cube2));
  m = LoadModelFromString(xml, error.data(), error.size(), vfs.get());
  EXPECT_EQ(m->mesh_vert[0], -1);
  mj_deleteModel(m);

  mj_deleteVFS(vfs.get());
}

}  // namespace
}  // namespace mujoco
