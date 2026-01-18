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


#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>
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
}  // namespace
}  // namespace mujoco
