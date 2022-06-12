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

// A benchmark for comparing different implementations of engine_util_spatial methods.

#include <benchmark/benchmark.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/base/attributes.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_spatial.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

// ------------- quaternion-vector rotation ----------------------------

void BM_RotVecQuat(benchmark::State& state) {
  MujocoErrorTestGuard guard;

  // Create axis-angle and convert to a quaternion
  mjtNum quat[4];
  const mjtNum angle = 33 * mjPI / 180;
  mjtNum vec[] = {0.2672612419124244, 0.5345224838248488, 0.8017837257372732};
  mju_axisAngle2Quat(quat, vec, angle);

  for (auto s : state) {
    mjtNum result[3];
    mju_rotVecQuat(result, vec, quat);
    benchmark::DoNotOptimize(result);
  }
  state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_RotVecQuat);

}  // namespace
}  // namespace mujoco
