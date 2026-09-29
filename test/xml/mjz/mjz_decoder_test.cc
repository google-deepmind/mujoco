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

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <string_view>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
// Disable unused function warnings for miniz.
#if defined(__GNUC__) || defined(__clang__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include <miniz.h>
#if defined(__GNUC__) || defined(__clang__)
  #pragma GCC diagnostic pop
#endif
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using MjzTest = MujocoTest;

using ::testing::IsNull;
using ::testing::Not;
using ::testing::NotNull;
using ::testing::StrEq;

TEST_F(MjzTest, ParseWithoutVFS) {
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");
  char err[1000] = "";
  mjSpec* spec = mj_parse(filepath.c_str(), "", nullptr, err, sizeof(err));
  EXPECT_THAT(spec, NotNull());
  EXPECT_THAT(err, StrEq(""));
  mjModel* model = mj_compile(spec, nullptr);
  EXPECT_THAT(model, NotNull()) << mjs_getError(spec);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

TEST_F(MjzTest, ParseMultipleTimesWithoutVFS) {
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");
  char err[1000] = "";
  mjSpec* spec1 = mj_parse(filepath.c_str(), "", nullptr, err, sizeof(err));
  EXPECT_THAT(spec1, NotNull());
  EXPECT_THAT(err, StrEq(""));

  mjSpec* spec2 = mj_parse(filepath.c_str(), "", nullptr, err, sizeof(err));
  EXPECT_THAT(spec2, NotNull());
  EXPECT_THAT(err, StrEq(""));

  mjModel* model1 = mj_compile(spec1, nullptr);
  EXPECT_THAT(model1, NotNull()) << mjs_getError(spec1);
  mjModel* model2 = mj_compile(spec2, nullptr);
  EXPECT_THAT(model2, NotNull()) << mjs_getError(spec2);

  mj_deleteModel(model1);
  mj_deleteModel(model2);
  mj_deleteSpec(spec1);
  mj_deleteSpec(spec2);
}

TEST_F(MjzTest, ParseWithVFS) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");
  char err[1000] = "";
  mjSpec* spec = mj_parse(filepath.c_str(), "", &vfs, err, sizeof(err));
  EXPECT_THAT(spec, NotNull());
  EXPECT_THAT(err, StrEq(""));
  mjModel* model = mj_compile(spec, &vfs);
  EXPECT_THAT(model, NotNull()) << mjs_getError(spec);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
  mj_deleteVFS(&vfs);
}

TEST_F(MjzTest, ParseMultipleTimesSameVFS) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  std::string filepath = GetTestDataFilePath("testdata/model.mjz");
  char err[1000] = "";
  mjSpec* spec1 = mj_parse(filepath.c_str(), "", &vfs, err, sizeof(err));
  EXPECT_THAT(spec1, NotNull());
  EXPECT_THAT(err, StrEq(""));

  mjSpec* spec2 = mj_parse(filepath.c_str(), "", &vfs, err, sizeof(err));
  EXPECT_THAT(spec2, NotNull());
  EXPECT_THAT(err, StrEq(""));

  mjModel* model1 = mj_compile(spec1, &vfs);
  EXPECT_THAT(model1, NotNull()) << mjs_getError(spec1);
  mjModel* model2 = mj_compile(spec2, &vfs);
  EXPECT_THAT(model2, NotNull()) << mjs_getError(spec2);

  mj_deleteModel(model1);
  mj_deleteModel(model2);
  mj_deleteSpec(spec1);
  mj_deleteSpec(spec2);
  mj_deleteVFS(&vfs);
}

TEST_F(MjzTest, InvalidPath) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  std::string filepath = GetTestDataFilePath("testdata/not_a_file.mjz");
  char err[1000] = "";
  mjSpec* spec = mj_parse(filepath.c_str(), "", nullptr, err, sizeof(err));
  EXPECT_THAT(spec, IsNull());
  EXPECT_THAT(err, Not(StrEq("")));
  mj_deleteVFS(&vfs);
}

// Older Windows builds wrote archive entry names with '\' separators.
TEST_F(MjzTest, ParseBackslashEntryNames) {
  static constexpr char xml[] = R"(
    <mujoco>
      <compiler meshdir="assets"/>
      <asset>
        <mesh name="tet" file="meshes/tet.obj"/>
      </asset>
      <worldbody>
        <geom type="mesh" mesh="tet"/>
      </worldbody>
    </mujoco>
  )";
  static constexpr char obj[] =
      "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
      "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";

  mz_zip_archive zip;
  std::memset(&zip, 0, sizeof(zip));
  ASSERT_TRUE(mz_zip_writer_init_heap(&zip, 0, 0));
  ASSERT_TRUE(mz_zip_writer_add_mem(&zip, "model.xml", xml, std::strlen(xml),
                                    MZ_DEFAULT_COMPRESSION));
  ASSERT_TRUE(mz_zip_writer_add_mem(&zip, "assets\\meshes\\tet.obj", obj,
                                    std::strlen(obj), MZ_DEFAULT_COMPRESSION));
  void* archive = nullptr;
  size_t archive_size = 0;
  ASSERT_TRUE(
      mz_zip_writer_finalize_heap_archive(&zip, &archive, &archive_size));
  mz_zip_writer_end(&zip);

  const std::string filepath = testing::TempDir() + "/backslash_entries.mjz";
  std::ofstream(filepath, std::ios::binary)
      .write(static_cast<const char*>(archive), archive_size);
  std::free(archive);

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  char err[1000] = "";
  mjSpec* spec = mj_parse(filepath.c_str(), "", &vfs, err, sizeof(err));
  ASSERT_THAT(spec, NotNull()) << err;
  mjModel* model = mj_compile(spec, &vfs);
  EXPECT_THAT(model, NotNull()) << mjs_getError(spec);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
  mj_deleteVFS(&vfs);
  std::remove(filepath.c_str());
}

}  // namespace
}  // namespace mujoco
