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

// A benchmark for the elliptic Newton solver with many cone-state contacts.

#include <benchmark/benchmark.h>
#include <absl/base/attributes.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// A pile of boxes sliding under diagonal gravity, keyframed in two regimes:
// "clump" is piled in a corner with a coupled contact graph, where the solver
// folds the cone contributions into the Hessian with one refactorization;
// "scatter" is independent stacks, where per-contact rank-1 updates are
// cheaper (despite holding more cone-state rows than the clump). Together the
// two keep both HessianCone paths, and the flop-count gate between them,
// under measurement.
static void run_cone_benchmark(benchmark::State& state, const char* key) {
  MujocoErrorTestGuard guard;
  static mjModel* model =
      LoadModelFromPath("../test/benchmark/testdata/boxpile.xml");
  mjData* data = mj_makeData(model);
  int keyid = mj_name2id(model, mjOBJ_KEY, key);

  for (auto s : state) {
    mj_resetDataKeyframe(model, data, keyid);
    mj_forward(model, data);
  }

  mj_deleteData(data);
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_ConeClump(benchmark::State& state) {
  run_cone_benchmark(state, "clump");
}
BENCHMARK(BM_ConeClump);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_ConeScatter(benchmark::State& state) {
  run_cone_benchmark(state, "scatter");
}
BENCHMARK(BM_ConeScatter);

}  // namespace
}  // namespace mujoco
