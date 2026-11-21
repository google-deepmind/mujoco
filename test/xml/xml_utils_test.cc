// Copyright 2025 DeepMind Technologies Limited
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

#include <filesystem>  // NOLINT
#include <map>
#include <set>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

TEST_F(MujocoTest, GetXMLDependenciesTest) {
  static const std::vector<std::string> kModelPaths = {
      GetTestDataFilePath("xml/testdata/many_dependencies.xml"),
      GetTestDataFilePath("xml/testdata/parent_model.xml"),
      GetTestDataFilePath("xml/testdata/child.xml"),
      GetTestDataFilePath("xml/testdata/meshes/mesh1.obj"),
      GetTestDataFilePath("xml/testdata/meshes/flex.obj"),
      GetTestDataFilePath("xml/testdata/meshes/cube.skn"),
  };

  mjStringVec dependencies;
  mju_getXMLDependencies(kModelPaths[0].c_str(), &dependencies);
  std::set<std::string> dependency_set;
  for (const auto& dep : dependencies) {
    dependency_set.insert(std::filesystem::path(dep).generic_string());
  }
  std::set<std::string> expected_dependency_set;
  for (const auto& path : kModelPaths) {
    expected_dependency_set.insert(
        std::filesystem::path(path).generic_string());
  }
  EXPECT_EQ(dependency_set, expected_dependency_set);
}

// Custom resource provider that serves XML strings from an in-memory map,
// used to exercise the non-filesystem code path.
namespace memxml {
static const std::map<std::string, std::string>* g_files = nullptr;

int Open(mjResource* resource) {
  return g_files && g_files->count(resource->name) ? 1 : 0;
}
int Read(mjResource* resource, const void** buffer) {
  auto it = g_files->find(resource->name);
  if (it == g_files->end()) return -1;
  *buffer = it->second.data();
  return static_cast<int>(it->second.size());
}
void Close(mjResource* resource) {}
}  // namespace memxml

// Verifies mju_getXMLDependencies works against a non-filesystem resource
// provider (the case the WASM/HTTP build relies on).
TEST_F(MujocoTest, GetXMLDependenciesViaProvider) {
  const std::map<std::string, std::string> files = {
      {"memxml:/scene.xml",
       "<mujoco>\n"
       "  <include file=\"child.xml\"/>\n"
       "  <compiler meshdir=\"meshes\"/>\n"
       "  <asset>\n"
       "    <mesh name=\"m\" file=\"m.obj\"/>\n"
       "  </asset>\n"
       "</mujoco>"},
      {"memxml:/child.xml",
       "<mujoco>\n"
       "  <asset>\n"
       "    <texture name=\"t\" type=\"2d\" file=\"t.png\"/>\n"
       "  </asset>\n"
       "</mujoco>"},
  };
  memxml::g_files = &files;

  mjpResourceProvider provider = {
      .prefix = "memxml",
      .open = memxml::Open,
      .read = memxml::Read,
      .close = memxml::Close,
  };
  ASSERT_GT(mjp_registerProvider(&provider), 0);

  mjStringVec dependencies;
  mju_getXMLDependencies("memxml:/scene.xml", &dependencies);
  std::set<std::string> dep_set(dependencies.begin(), dependencies.end());

  EXPECT_THAT(dep_set, testing::UnorderedElementsAre(
                           "memxml:/scene.xml", "memxml:/child.xml",
                           "memxml:/meshes/m.obj", "memxml:/t.png"));

  memxml::g_files = nullptr;
}

// Includes are expanded into the including model, as ExpandIncludeXML in
// xml.cc does, while a <model> asset stays a separate model with its own
// scope. The in-memory files form this tree (the .obj files are not served:
// dependencies are reported by path, not opened):
//
//   scene.xml        meshdir="meshes"; <include file="parts/robot.xml"/>;
//                    <model file="sub/sub.xml"/>
//   meshes/
//     hand.obj       the mesh of parts/hand.xml, through scene.xml's meshdir
//   parts/
//     robot.xml      <include file="parts/arm.xml"/>: relative to scene.xml's
//                    directory
//     arm.xml        <include file="hand.xml"/>: relative to parts/, the
//                    including file's directory
//     hand.xml       <mesh file="hand.obj"/>
//   sub/
//     sub.xml        a separate model: <mesh file="wheel.obj"/> is relative to
//                    sub/ and ignores scene.xml's meshdir
//     wheel.obj
TEST_F(MujocoTest, GetXMLDependenciesExpandsIncludesIntoTheModel) {
  const std::map<std::string, std::string> files = {
      // Root model: sets meshdir, includes a file in parts/, and references a
      // separate <model> in sub/.
      {"incxml:/scene.xml",
       "<mujoco>\n"
       "  <compiler meshdir=\"meshes\"/>\n"
       "  <include file=\"parts/robot.xml\"/>\n"
       "  <asset>\n"
       "    <model name=\"sub\" file=\"sub/sub.xml\"/>\n"
       "  </asset>\n"
       "</mujoco>"},
      // Nested <include> resolved relative to the model directory.
      {"incxml:/parts/robot.xml",
       "<mujoco>\n"
       "  <include file=\"parts/arm.xml\"/>\n"
       "</mujoco>"},
      // Nested <include> resolved relative to the including file's directory.
      {"incxml:/parts/arm.xml",
       "<mujoco>\n"
       "  <include file=\"hand.xml\"/>\n"
       "</mujoco>"},
      // Asset in an included subdirectory file uses the root model's meshdir.
      {"incxml:/parts/hand.xml",
       "<mujoco>\n"
       "  <asset>\n"
       "    <mesh name=\"hand\" file=\"hand.obj\"/>\n"
       "  </asset>\n"
       "</mujoco>"},
      // <model> is a separate model: its asset is relative to sub/, not the
      // root model's meshdir.
      {"incxml:/sub/sub.xml",
       "<mujoco>\n"
       "  <asset>\n"
       "    <mesh name=\"wheel\" file=\"wheel.obj\"/>\n"
       "  </asset>\n"
       "</mujoco>"},
  };
  memxml::g_files = &files;

  mjpResourceProvider provider = {
      .prefix = "incxml",
      .open = memxml::Open,
      .read = memxml::Read,
      .close = memxml::Close,
  };
  ASSERT_GT(mjp_registerProvider(&provider), 0);

  mjStringVec dependencies;
  mju_getXMLDependencies("incxml:/scene.xml", &dependencies);
  std::set<std::string> dep_set(dependencies.begin(), dependencies.end());

  EXPECT_THAT(dep_set, testing::UnorderedElementsAre(
                           "incxml:/scene.xml", "incxml:/parts/robot.xml",
                           "incxml:/parts/arm.xml", "incxml:/parts/hand.xml",
                           "incxml:/meshes/hand.obj", "incxml:/sub/sub.xml",
                           "incxml:/sub/wheel.obj"));

  memxml::g_files = nullptr;
}

// strippath applies to asset filenames before the compiler directory is added.
TEST_F(MujocoTest, GetXMLDependenciesStripPath) {
  struct Case {
    const char* compiler;
    const char* file;
    const char* resolved;
  };
  const Case cases[] = {
      {R"(strippath="true")", "old/path/mesh.obj", "mesh.obj"},
      {R"(strippath="true")", "/old/path/mesh.obj", "mesh.obj"},
      {R"(strippath="true")", R"(C:\old\path\mesh.obj)", "mesh.obj"},
      {R"(strippath="false")", "old/path/mesh.obj", "old/path/mesh.obj"},
      {"", "old/path/mesh.obj", "old/path/mesh.obj"},
  };
  mjpResourceProvider provider = {
      .prefix = "strpxml",
      .open = memxml::Open,
      .read = memxml::Read,
      .close = memxml::Close,
  };
  ASSERT_GT(mjp_registerProvider(&provider), 0);

  for (const auto& [compiler, file, resolved] : cases) {
    SCOPED_TRACE(compiler);
    SCOPED_TRACE(file);
    const std::string mesh_path = std::string("strpxml:/meshes/") + resolved;
    const std::map<std::string, std::string> files = {
        {"strpxml:/scene.xml",
         std::string(R"(<mujoco><compiler meshdir="meshes" )") + compiler +
             R"(/><asset><mesh name="mesh" file=")" + file + R"("/></asset>
             <worldbody><geom type="mesh" mesh="mesh"/></worldbody></mujoco>)"},
        {mesh_path, "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
                    "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n"},
    };
    memxml::g_files = &files;

    // Compile using only the expected asset path, then require discovery to
    // return that same path so the browser can prefetch it.
    char error[1024];
    mjModel* model = mj_loadXML("strpxml:/scene.xml", nullptr, error, sizeof(error));
    EXPECT_NE(model, nullptr) << error;
    if (model) {
      EXPECT_EQ(model->nmesh, 1);
      mj_deleteModel(model);
    }
    mjStringVec dependencies;
    mju_getXMLDependencies("strpxml:/scene.xml", &dependencies);
    EXPECT_THAT(dependencies, testing::UnorderedElementsAre(
                                  "strpxml:/scene.xml", mesh_path));
    memxml::g_files = nullptr;
  }
}

// Includes share the parent's strippath, while model assets have their own
// compiler settings. Neither include nor model filenames are stripped.
TEST_F(MujocoTest, GetXMLDependenciesStripPathScope) {
  const std::map<std::string, std::string> files = {
      {"strpscope:/scene.xml", R"(
        <mujoco>
          <compiler strippath="true" texturedir="textures"/>
          <include file="parts/child.xml"/>
          <asset><model name="sub" file="models/sub.xml"/></asset>
        </mujoco>)"},
      {"strpscope:/parts/child.xml", R"(
        <mujoco><asset>
          <texture name="texture" type="2d" file="old/texture.png"/>
        </asset></mujoco>)"},
      {"strpscope:/models/sub.xml", R"(
        <mujoco><asset><mesh name="mesh" file="old/mesh.obj"/></asset></mujoco>)"},
  };
  memxml::g_files = &files;
  mjpResourceProvider provider = {
      .prefix = "strpscope",
      .open = memxml::Open,
      .read = memxml::Read,
      .close = memxml::Close,
  };
  ASSERT_GT(mjp_registerProvider(&provider), 0);

  mjStringVec dependencies;
  mju_getXMLDependencies("strpscope:/scene.xml", &dependencies);
  EXPECT_THAT(dependencies, testing::UnorderedElementsAre(
      "strpscope:/scene.xml", "strpscope:/parts/child.xml",
      "strpscope:/textures/texture.png", "strpscope:/models/sub.xml",
      "strpscope:/models/old/mesh.obj"));
  memxml::g_files = nullptr;
}

TEST_F(MujocoTest, GetXMLDependenciesHandlesCyclicIncludes) {
  const std::map<std::string, std::string> files = {
      {"cycxml:/a.xml",
       "<mujoco>\n"
       "  <include file=\"b.xml\"/>\n"
       "</mujoco>"},
      {"cycxml:/b.xml",
       "<mujoco>\n"
       "  <include file=\"a.xml\"/>\n"
       "</mujoco>"},
  };
  memxml::g_files = &files;

  mjpResourceProvider provider = {
      .prefix = "cycxml",
      .open = memxml::Open,
      .read = memxml::Read,
      .close = memxml::Close,
  };
  ASSERT_GT(mjp_registerProvider(&provider), 0);

  mjStringVec dependencies;
  mju_getXMLDependencies("cycxml:/a.xml", &dependencies);
  std::set<std::string> dep_set(dependencies.begin(), dependencies.end());

  EXPECT_THAT(dep_set,
              testing::UnorderedElementsAre("cycxml:/a.xml", "cycxml:/b.xml"));

  memxml::g_files = nullptr;
}

// MjuErrorMessageFrom longjmps out of mju_error; AddressSanitizer /
// LeakSanitizer checks that no C++ stack objects leaked on that jump.
TEST_F(MujocoTest, GetXMLDependenciesErrorsDoNotLeakOnLongjmp) {
  const std::map<std::string, std::string> files = {
      {"errxml:/some/deep/path/that/exceeds/sso/buffer/broken.xml",
       "<mujoco><unclosed>"},
      {"errxml:/some/deep/path/that/exceeds/sso/buffer/missing_inc.xml",
       "<mujoco><include file=\"nonexistent.xml\"/></mujoco>"},
  };
  memxml::g_files = &files;

  mjpResourceProvider provider = {
      .prefix = "errxml",
      .open = memxml::Open,
      .read = memxml::Read,
      .close = memxml::Close,
  };
  ASSERT_GT(mjp_registerProvider(&provider), 0);

  mjStringVec dependencies;
  auto call_deps = MjuErrorMessageFrom(mju_getXMLDependencies);

  EXPECT_THAT(
      call_deps("errxml:/some/deep/path/that/exceeds/sso/buffer/broken.xml",
                &dependencies),
      testing::HasSubstr("Problem reading XML file"));
  EXPECT_THAT(
      call_deps(
          "errxml:/some/deep/path/that/exceeds/sso/buffer/missing_inc.xml",
          &dependencies),
      testing::HasSubstr("Could not open"));
  EXPECT_THAT(
      call_deps("errxml:/some/deep/path/that/exceeds/sso/buffer/nope.xml",
                &dependencies),
      testing::HasSubstr("Could not open"));

  memxml::g_files = nullptr;
}
}  // namespace
}  // namespace mujoco
