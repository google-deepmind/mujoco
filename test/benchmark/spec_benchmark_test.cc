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

// Benchmarks for procedural model editing with the mjSpec API.

#include <string>

#include <benchmark/benchmark.h>
#include <absl/base/attributes.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// add n geoms to the world body
static void add_geoms(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  for (auto s : state) {
    mjSpec* spec = mj_makeSpec();
    mjsBody* world = mjs_findBody(spec, "world");
    for (int i = 0; i < n; i++) {
      mjs_addGeom(world, nullptr);
    }
    mj_deleteSpec(spec);
  }
  state.SetItemsProcessed(state.iterations() * n);
}

// add n bodies to the world body
static void add_bodies(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  for (auto s : state) {
    mjSpec* spec = mj_makeSpec();
    mjsBody* world = mjs_findBody(spec, "world");
    for (int i = 0; i < n; i++) {
      mjs_addBody(world, nullptr);
    }
    mj_deleteSpec(spec);
  }
  state.SetItemsProcessed(state.iterations() * n);
}

// add a chain of n nested bodies
static void add_nested_bodies(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  for (auto s : state) {
    mjSpec* spec = mj_makeSpec();
    mjsBody* body = mjs_findBody(spec, "world");
    for (int i = 0; i < n; i++) {
      body = mjs_addBody(body, nullptr);
    }
    mj_deleteSpec(spec);
  }
  state.SetItemsProcessed(state.iterations() * n);
}

// add n geoms to the world body, naming each one right after adding it
static void add_named_geoms(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  int n = state.range(0);
  for (auto s : state) {
    mjSpec* spec = mj_makeSpec();
    mjsBody* world = mjs_findBody(spec, "world");
    for (int i = 0; i < n; i++) {
      mjsGeom* geom = mjs_addGeom(world, nullptr);
      mjs_setName(geom->element, ("geom_" + std::to_string(i)).c_str());
    }
    mj_deleteSpec(spec);
  }
  state.SetItemsProcessed(state.iterations() * n);
}

// Use ABSL_ATTRIBUTE_NO_TAIL_CALL to make sure the benchmark functions appear
// separately in CPU profiles (and don't get replaced with raw calls to
// run_*_benchmark).

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_SpecAddGeoms(benchmark::State& state) {
  add_geoms(state);
}
BENCHMARK(BM_SpecAddGeoms)->Arg(1000)->Arg(8000)->Unit(benchmark::kMillisecond);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_SpecAddBodies(benchmark::State& state) {
  add_bodies(state);
}
BENCHMARK(BM_SpecAddBodies)
    ->Arg(1000)
    ->Arg(8000)
    ->Unit(benchmark::kMillisecond);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_SpecAddNestedBodies(benchmark::State& state) {
  add_nested_bodies(state);
}
BENCHMARK(BM_SpecAddNestedBodies)->Arg(1000)->Unit(benchmark::kMillisecond);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_SpecAddNamedGeoms(benchmark::State& state) {
  add_named_geoms(state);
}
BENCHMARK(BM_SpecAddNamedGeoms)
    ->Arg(1000)
    ->Arg(8000)
    ->Unit(benchmark::kMillisecond);

}  // namespace
}  // namespace mujoco
