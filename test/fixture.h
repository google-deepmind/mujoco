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

#ifndef MUJOCO_TEST_FIXTURE_H_
#define MUJOCO_TEST_FIXTURE_H_

#include <gtest/gtest.h>
#include <absl/strings/string_view.h>
#include <mujoco/mjmodel.h>

namespace mujoco {

// Installs and uninstalls error callbacks on MuJoCo that fail the currently
// running test if triggered. Prefer the use of MujocoTest, unless using a
// test fixture is not possible.
class MujocoErrorTestGuard {
 public:
  // Sets up error and warning callbacks on MuJoCo that will fail the test if
  // triggered.
  MujocoErrorTestGuard();

  // Clears up the callbacks from the constructor.
  ~MujocoErrorTestGuard();
};

// A test fixture which simplifies writing tests for the MuJoCo C API.
// By default, any MuJoCo operation which triggers a warning or error will
// trigger a test failure.
class MujocoTest : public ::testing::Test {
 private:
  MujocoErrorTestGuard error_guard;
};

// Returns a path to a data file, under the mujoco/test directory.
const std::string GetTestDataFilePath(absl::string_view path);

// Returns a path to a data file, under the mujoco/model directory.
const std::string GetModelPath(absl::string_view path);

// Returns a newly-allocated mjModel, loaded from the contents of xml.
// On failure returns nullptr and populates the error array if present.
mjModel* LoadModelFromString(absl::string_view xml, char* error = nullptr,
                             int error_size = 0);

// Returns a string loaded from first saving the model given an input.
const std::string SaveAndReadXml(const mjModel* model);

}  // namespace mujoco
#endif  // MUJOCO_TEST_FIXTURE_H_
