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

// Tests for src/xml/mjdb/mjdb_provider.cc.

#ifdef _WIN32
  #include <cstdlib>
#else
  #include <stdlib.h>
#endif
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>
#include <string_view>
#include <system_error>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

namespace mujoco {
namespace {

using ::testing::EndsWith;
using ::testing::Gt;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

constexpr char kRootDirEnvVar[] = "MJDB_ROOT_DIR";

// An object with one geom.
constexpr char kObjectXml[] = R"(
<mujoco>
  <worldbody>
    <body name="box">
      <geom type="box" size="0.1 0.2 0.3"/>
    </body>
  </worldbody>
</mujoco>
)";

// A scene that attaches version 3 of the object twice.
constexpr char kSceneXml[] = R"(
<mujoco>
  <asset>
    <model name="box" file="mjdb://object/origin/box?version=3"/>
  </asset>
  <worldbody>
    <body name="left_root" pos="0 1 0">
      <freejoint/>
      <attach model="box" body="box" prefix="left_"/>
    </body>
    <body name="right_root" pos="0 -1 0">
      <freejoint/>
      <attach model="box" body="box" prefix="right_"/>
    </body>
  </worldbody>
</mujoco>
)";

// Sets an environment variable, or unsets it if value is null.
void SetEnv(const char* name, const char* value) {
#ifdef _WIN32
  _putenv_s(name, value ? value : "");
#else
  if (value) {
    setenv(name, value, /*overwrite=*/1);
  } else {
    unsetenv(name);
  }
#endif
}

// The warnings that MuJoCo has issued since the start of the current test.
std::string& Warnings() {
  static std::string* warnings = new std::string;
  return *warnings;
}

// Each test has an empty root directory, which MJDB_ROOT_DIR names.
class MjdbProviderTest : public testing::Test {
 protected:
  void SetUp() override {
    const testing::TestInfo* test_info =
        testing::UnitTest::GetInstance()->current_test_info();
    test_dir_ = std::filesystem::path(testing::TempDir()) /
                "mjdb_provider_test" / test_info->name();
    std::filesystem::remove_all(test_dir_);
    std::filesystem::create_directories(RootDir());
    SetEnv(kRootDirEnvVar, RootDir().string().c_str());

    Warnings().clear();
    mju_user_warning =
        +[](const char* message) { Warnings().append(message).append("\n"); };
  }

  void TearDown() override {
    mj_freeLastXML();
    mju_user_warning = nullptr;
    SetEnv(kRootDirEnvVar, nullptr);
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  std::filesystem::path RootDir() const { return test_dir_ / "root"; }

  // A directory beside the root directory.
  std::filesystem::path OutsideDir() const { return test_dir_ / "outside"; }

  static void WriteFile(const std::filesystem::path& path,
                        std::string_view content) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary);
    file << content;
    ASSERT_TRUE(file.good()) << path;
  }

  // Returns the model that the MJCF file at a URI loads, or null if loading
  // fails, in which case error() says why. The caller owns the model.
  mjModel* Load(const char* uri) {
    char load_error[1024] = "";
    mjModel* model = mj_loadXML(uri, nullptr, load_error, sizeof(load_error));
    error_ = load_error;
    return model;
  }

  const std::string& error() const { return error_; }

 private:
  std::filesystem::path test_dir_;
  std::string error_;
};

TEST_F(MjdbProviderTest, ProviderIsRegistered) {
  const mjpResourceProvider* provider =
      mjp_findProvider("mjdb://object/origin/box");
  ASSERT_THAT(provider, NotNull());
  EXPECT_STREQ(provider->prefix, "mjdb");
}

TEST_F(MjdbProviderTest, LoadsPinnedVersion) {
  WriteFile(RootDir() / "object" / "origin" / "box" / "3", kObjectXml);

  mjModel* model = Load("mjdb://object/origin/box?version=3");
  ASSERT_THAT(model, NotNull()) << error();
  EXPECT_EQ(model->ngeom, 1);
  mj_deleteModel(model);
}

TEST_F(MjdbProviderTest, LoadsActiveVersionByDefault) {
  WriteFile(RootDir() / "object" / "origin" / "box" / "active", kObjectXml);

  mjModel* model = Load("mjdb://object/origin/box");
  ASSERT_THAT(model, NotNull()) << error();
  EXPECT_EQ(model->ngeom, 1);
  mj_deleteModel(model);
}

TEST_F(MjdbProviderTest, IgnoresOtherQueryParameters) {
  WriteFile(RootDir() / "object" / "origin" / "box" / "3", kObjectXml);

  mjModel* model = Load("mjdb://object/origin/box?a=b&version=3&c=d");
  ASSERT_THAT(model, NotNull()) << error();
  mj_deleteModel(model);
}

TEST_F(MjdbProviderTest, LoadsUidWithSeveralComponents) {
  WriteFile(RootDir() / "object" / "origin" / "group" / "box" / "3",
            kObjectXml);

  mjModel* model = Load("mjdb://object/origin/group/box?version=3");
  ASSERT_THAT(model, NotNull()) << error();
  mj_deleteModel(model);
}

TEST_F(MjdbProviderTest, LoadsSceneThatAttachesObjects) {
  WriteFile(RootDir() / "object" / "origin" / "box" / "3", kObjectXml);
  WriteFile(RootDir() / "scene" / "origin" / "room" / "1", kSceneXml);

  mjModel* model = Load("mjdb://scene/origin/room?version=1");
  ASSERT_THAT(model, NotNull()) << error();
  EXPECT_EQ(model->ngeom, 2);
  mj_deleteModel(model);
}

TEST_F(MjdbProviderTest, NamesResourceWithItsFormat) {
  WriteFile(RootDir() / "object" / "origin" / "box" / "3", kObjectXml);

  char open_error[1024] = "";
  mjResource* resource =
      mju_openResource("", "mjdb://object/origin/box?version=3", nullptr,
                       open_error, sizeof(open_error));
  ASSERT_THAT(resource, NotNull()) << open_error;
  EXPECT_THAT(resource->name, EndsWith("?version=3.xml"));

  const void* buffer = nullptr;
  const int size = mju_readResource(resource, &buffer);
  ASSERT_THAT(size, Gt(0));
  EXPECT_EQ(std::string_view(static_cast<const char*>(buffer), size),
            kObjectXml);
  mju_closeResource(resource);

  EXPECT_EQ(Warnings(), "");

  // The name of an opened resource resolves to the same file.
  mjModel* model = Load("mjdb://object/origin/box?version=3.xml");
  ASSERT_THAT(model, NotNull()) << error();
  mj_deleteModel(model);
}

TEST_F(MjdbProviderTest, NamesArchiveWithItsFormat) {
  // The signature of an empty zip archive, followed by the rest of its only
  // record.
  WriteFile(RootDir() / "object" / "origin" / "mug" / "1",
            std::string("PK\x05\x06", 4) + std::string(18, '\0'));

  char open_error[1024] = "";
  mjResource* resource =
      mju_openResource("", "mjdb://object/origin/mug?version=1", nullptr,
                       open_error, sizeof(open_error));
  ASSERT_THAT(resource, NotNull()) << open_error;
  EXPECT_THAT(resource->name, EndsWith("?version=1.mjz"));
  mju_closeResource(resource);
}

TEST_F(MjdbProviderTest, FailsWhenRootDirIsNotSet) {
  WriteFile(RootDir() / "object" / "origin" / "box" / "3", kObjectXml);
  SetEnv(kRootDirEnvVar, nullptr);

  EXPECT_THAT(Load("mjdb://object/origin/box?version=3"), IsNull());
  EXPECT_THAT(Warnings(), HasSubstr("MJDB_ROOT_DIR is not set"));
}

TEST_F(MjdbProviderTest, FailsForMissingAsset) {
  WriteFile(RootDir() / "object" / "origin" / "box" / "3", kObjectXml);

  EXPECT_THAT(Load("mjdb://object/origin/box?version=4"), IsNull());
  EXPECT_THAT(Warnings(), HasSubstr("cannot read"));
}

TEST_F(MjdbProviderTest, FailsForDirectory) {
  WriteFile(RootDir() / "object" / "origin" / "box" / "3" / "file", kObjectXml);

  EXPECT_THAT(Load("mjdb://object/origin/box?version=3"), IsNull());
  EXPECT_THAT(Warnings(), HasSubstr("cannot read"));
}

TEST_F(MjdbProviderTest, FailsForUnrecognizedFormat) {
  WriteFile(RootDir() / "object" / "origin" / "box" / "3", "not a model");

  EXPECT_THAT(Load("mjdb://object/origin/box?version=3"), IsNull());
  EXPECT_THAT(Warnings(), HasSubstr("neither an MJZ archive nor an XML file"));
}

TEST_F(MjdbProviderTest, FailsForUriWithTooFewComponents) {
  WriteFile(RootDir() / "object" / "origin" / "active", kObjectXml);

  EXPECT_THAT(Load("mjdb://object/origin"), IsNull());
  EXPECT_THAT(Warnings(), HasSubstr("invalid URI"));
}

TEST_F(MjdbProviderTest, DoesNotResolveOutsideRootDir) {
  WriteFile(OutsideDir() / "box" / "active", kObjectXml);
  WriteFile(OutsideDir() / "box" / "3", kObjectXml);

  // {root}/../outside/box/active
  EXPECT_THAT(Load("mjdb://../outside/box"), IsNull());
  EXPECT_THAT(Load("mjdb://../outside/box?version=3"), IsNull());
  EXPECT_THAT(Warnings(), HasSubstr("invalid URI"));
}

TEST_F(MjdbProviderTest, DoesNotResolveVersionWithSeparators) {
  WriteFile(RootDir() / "object" / "origin" / "box" / "3", kObjectXml);

  // {root}/object/origin/box/../box/3
  EXPECT_THAT(Load("mjdb://object/origin/box?version=../box/3"), IsNull());
  EXPECT_THAT(Load("mjdb://object/origin/box?version=.."), IsNull());
  EXPECT_THAT(Warnings(), HasSubstr("invalid URI"));
}

}  // namespace
}  // namespace mujoco
