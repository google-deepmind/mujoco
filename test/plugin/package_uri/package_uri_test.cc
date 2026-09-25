// Copyright 2026 PickNik Inc.
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

// Tests for the package:// resource provider plugin.

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

namespace fs = std::filesystem;

using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

static constexpr char kTetrahedronObj[] = R"(v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 1 2 3
f 1 4 2
f 1 3 4
f 2 4 3
)";

void SetEnv(const char* name, const std::string& value) {
#ifdef _WIN32
  _putenv_s(name, value.c_str());
#else
  setenv(name, value.c_str(), 1);
#endif
}

void WriteFile(const fs::path& path, const std::string& contents) {
  fs::create_directories(path.parent_path());
  std::ofstream(path, std::ios::binary) << contents;
}

// creates one package in a ROS 2 install space and one in a source tree
class PackageUriTest : public MujocoTest {
 protected:
  void SetUp() override {
    // one directory per test, since ctest runs the tests in parallel processes
    const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
    root_ = fs::path(::testing::TempDir()) / "package_uri_test" / info->name();
    fs::remove_all(root_);

    // ament: index marker plus share directory
    fs::path ament = root_ / "install";
    WriteFile(ament / "share/ament_index/resource_index/packages/ament_pkg",
              "");
    WriteFile(ament / "share/ament_pkg/meshes/tetrahedron.obj",
              kTetrahedronObj);

    // source tree: directory named after the package with a package.xml
    fs::path src = root_ / "src";
    WriteFile(src / "src_pkg/package.xml", "<package/>");
    WriteFile(src / "src_pkg/meshes/tetrahedron.obj", kTetrahedronObj);

    SetEnv("AMENT_PREFIX_PATH", ament.string());
    SetEnv("ROS_PACKAGE_PATH", src.string());
  }

  void TearDown() override { fs::remove_all(root_); }

  fs::path root_;
};

std::string MjcfWithMesh(const std::string& file) {
  return R"(
  <mujoco>
    <compiler meshdir="ignored_by_provider_uris"/>
    <asset>
      <mesh name="tetrahedron" file=")" +
         file + R"("/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="tetrahedron"/>
    </worldbody>
  </mujoco>
  )";
}

TEST_F(PackageUriTest, ResolvesAmentPackage) {
  std::string xml = MjcfWithMesh("package://ament_pkg/meshes/tetrahedron.obj");
  std::array<char, 1024> error;
  MjModelPtr model =
      LoadModelFromString(xml.c_str(), error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull()) << error.data();
  EXPECT_EQ(model->nmeshvert, 4);
}

TEST_F(PackageUriTest, ResolvesRosPackagePath) {
  std::string xml = MjcfWithMesh("package://src_pkg/meshes/tetrahedron.obj");
  std::array<char, 1024> error;
  MjModelPtr model =
      LoadModelFromString(xml.c_str(), error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull()) << error.data();
  EXPECT_EQ(model->nmeshvert, 4);
}

TEST_F(PackageUriTest, SchemeIsCaseInsensitive) {
  std::string xml = MjcfWithMesh("PACKAGE://ament_pkg/meshes/tetrahedron.obj");
  std::array<char, 1024> error;
  MjModelPtr model =
      LoadModelFromString(xml.c_str(), error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull()) << error.data();
}

TEST_F(PackageUriTest, ResolvesUrdfMesh) {
  static constexpr char urdf[] = R"(
  <robot name="">
  <link name="base">
    <collision>
      <geometry>
        <mesh filename="package://ament_pkg/meshes/tetrahedron.obj"/>
      </geometry>
    </collision>
  </link>
  </robot>
  )";
  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(urdf, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull()) << error.data();
  EXPECT_EQ(model->nmeshvert, 4);
}

TEST_F(PackageUriTest, UnknownPackageFails) {
  std::string xml =
      MjcfWithMesh("package://missing_pkg/meshes/tetrahedron.obj");
  std::array<char, 1024> error;
  MjModelPtr model =
      LoadModelFromString(xml.c_str(), error.data(), error.size());
  EXPECT_THAT(model.get(), IsNull());
  EXPECT_THAT(error.data(),
              HasSubstr("package://missing_pkg/meshes/tetrahedron.obj"));
}

TEST_F(PackageUriTest, MissingFileFails) {
  std::string xml = MjcfWithMesh("package://ament_pkg/meshes/missing.obj");
  std::array<char, 1024> error;
  MjModelPtr model =
      LoadModelFromString(xml.c_str(), error.data(), error.size());
  EXPECT_THAT(model.get(), IsNull());
  EXPECT_THAT(error.data(),
              HasSubstr("package://ament_pkg/meshes/missing.obj"));
}

TEST_F(PackageUriTest, ReadsResourceBytes) {
  std::array<char, 1024> error;
  mjResource* resource =
      mju_openResource("", "package://ament_pkg/meshes/tetrahedron.obj",
                       nullptr, error.data(), error.size());
  ASSERT_THAT(resource, NotNull()) << error.data();

  const void* buffer = nullptr;
  int nbytes = mju_readResource(resource, &buffer);
  EXPECT_EQ(std::string(static_cast<const char*>(buffer), nbytes),
            kTetrahedronObj);
  mju_closeResource(resource);
}

TEST_F(PackageUriTest, AbsolutePathIsRejected) {
  // the file exists on disk, but a URI must not escape its package
  std::string mesh =
      (root_ / "install/share/ament_pkg/meshes/tetrahedron.obj").string();
  std::string xml = MjcfWithMesh("package://ament_pkg/" + mesh);
  std::array<char, 1024> error;
  MjModelPtr model =
      LoadModelFromString(xml.c_str(), error.data(), error.size());
  EXPECT_THAT(model.get(), IsNull());
}

TEST_F(PackageUriTest, ParentTraversalIsRejected) {
  // the file exists next to the package, but a URI must not escape its package
  WriteFile(root_ / "install/share/outside.obj", kTetrahedronObj);
  std::array<char, 1024> error;
  mjResource* resource =
      mju_openResource("", "package://ament_pkg/../outside.obj", nullptr,
                       error.data(), error.size());
  EXPECT_THAT(resource, IsNull());
  if (resource) mju_closeResource(resource);

  // same through the compiler, which reduces paths before opening them
  std::string xml =
      MjcfWithMesh("package://ament_pkg/meshes/../../outside.obj");
  MjModelPtr model =
      LoadModelFromString(xml.c_str(), error.data(), error.size());
  EXPECT_THAT(model.get(), IsNull());
}

TEST_F(PackageUriTest, DotDotPackageIsRejected) {
  // ".." exists inside the ament index directory, but is not a package
  WriteFile(root_ / "install/outside.obj", kTetrahedronObj);
  std::array<char, 1024> error;
  mjResource* resource = mju_openResource("", "package://../outside.obj",
                                          nullptr, error.data(), error.size());
  EXPECT_THAT(resource, IsNull());
  if (resource) mju_closeResource(resource);
}

TEST_F(PackageUriTest, ProviderRejectsEscapingNames) {
  // the compiler reduces ".." before opening a resource, so call the provider
  // directly to check that it enforces containment on its own
  WriteFile(root_ / "install/share/outside.obj", kTetrahedronObj);
  WriteFile(root_ / "install/outside.obj", kTetrahedronObj);
  const mjpResourceProvider* provider =
      mjp_getResourceProvider("package://ament_pkg/meshes/tetrahedron.obj");
  ASSERT_THAT(provider, NotNull());

  for (const char* name : {
           "package://ament_pkg/../outside.obj",
           "package://ament_pkg/meshes/../../outside.obj",
           "package://../outside.obj",
           "package://./ament_pkg/meshes/tetrahedron.obj",
           "package://ament_pkg//meshes/tetrahedron.obj",
       }) {
    mjResource resource = {};
    resource.name = const_cast<char*>(name);
    EXPECT_EQ(provider->open(&resource), 0) << name;
    if (resource.data) provider->close(&resource);
  }

  // the same provider serves a name that stays inside the package
  mjResource resource = {};
  resource.name =
      const_cast<char*>("package://ament_pkg/meshes/tetrahedron.obj");
  ASSERT_EQ(provider->open(&resource), 1);
  provider->close(&resource);
}

TEST_F(PackageUriTest, PackageWithoutPathFails) {
  std::string xml = MjcfWithMesh("package://ament_pkg/");
  std::array<char, 1024> error;
  MjModelPtr model =
      LoadModelFromString(xml.c_str(), error.data(), error.size());
  EXPECT_THAT(model.get(), IsNull());
}

}  // namespace
}  // namespace mujoco
