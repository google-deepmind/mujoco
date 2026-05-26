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
  std::set<std::string> dependency_set{dependencies.begin(),
                                       dependencies.end()};
  std::set<std::string> expected_dependency_set{kModelPaths.begin(),
                                                kModelPaths.end()};
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
TEST_F(MujocoTest, GetXMLDependenciesViaResourceProvider) {
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
  ASSERT_GT(mjp_registerResourceProvider(&provider), 0);

  mjStringVec dependencies;
  mju_getXMLDependencies("memxml:/scene.xml", &dependencies);
  std::set<std::string> dep_set(dependencies.begin(), dependencies.end());

  EXPECT_THAT(dep_set, testing::UnorderedElementsAre(
      "memxml:/scene.xml",
      "memxml:/child.xml",
      "memxml:/meshes/m.obj",
      "memxml:/t.png"));

  memxml::g_files = nullptr;
}
}  // namespace
}  // namespace mujoco
