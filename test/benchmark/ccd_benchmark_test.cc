// Copyright 2024 DeepMind Technologies Limited
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

// A benchmark for parsing and compiling models from XML.

#include <cstddef>
#include <vector>
#include <string>

#include <benchmark/benchmark.h>
#include <absl/base/attributes.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

#include "src/engine/engine_collision_convex.h"
#include "src/engine/engine_collision_primitive.h"

namespace mujoco {
namespace {

// number of steps to roll out before benchmarking
static const int kNumWarmupSteps = 1000;

// number of steps to benchmark (before resetting state)
static const int kBatchSize = 50;

static const char kBoxMeshPath[] =
    "../test/engine/testdata/collision_convex/perf/boxmesh.xml";
static const char kBoxBoxPath[] =
    "../test/engine/testdata/collision_convex/perf/box.xml";
static const char kEllipsoidPath[] =
  "../test/engine/testdata/collision_convex/perf/ellipsoid.xml";
static const char kMixedPath[] =
  "../test/engine/testdata/collision_convex/perf/mixed.xml";

class TestHarness {
 public:
  TestHarness(const char* xml_path, std::string label, int disable_flags = 0) {
    // Fail test if there are any mujoco errors
    MujocoErrorTestGuard guard;
    model_ = LoadModelFromPath(xml_path);
    model_->opt.disableflags |= disable_flags;
    data_ = mj_makeData(model_);
    for (int i=0; i < kNumWarmupSteps; i++) {
      mj_step(model_, data_);
    }

    // save state
    spec_ = mjSTATE_INTEGRATION;
    int size = mj_stateSize(model_, spec_);
    initial_state_.resize(size);
    mj_getState(model_, data_, initial_state_.data(), spec_);
    label_ = label;
  }

  void Reset() {
    mj_setState(model_, data_, initial_state_.data(), spec_);
  }

  void RunBenchmark(benchmark::State& state) {
    std::size_t ncon = 0;
    while (state.KeepRunningBatch(kBatchSize)) {
      Reset();

      // run a batch of steps
      for (int i=0; i < kBatchSize; i++) {
        mj_step(model_, data_);
        ncon += data_->ncon;
      }
    }

    state.SetLabel(label_);
    state.SetItemsProcessed(ncon);  // report number of contacts per second
  }

  ~TestHarness() {
    mj_deleteData(data_);
    mj_deleteModel(model_);
  }

 private:
  std::string label_;
  int spec_;
  mjModel* model_;
  mjData* data_;
  std::vector<mjtNum> initial_state_;
};


// Use ABSL_ATTRIBUTE_NO_TAIL_CALL to make sure the benchmark functions appear
// separately in CPU profiles (and don't get replaced with raw calls to
// run_parse_benchmark).

void ABSL_ATTRIBUTE_NO_TAIL_CALL
    BM_BoxMesh_NativeCCD(benchmark::State& state) {
  static TestHarness harness(kBoxMeshPath, "boxmesh.xml (nativeccd)");
  harness.RunBenchmark(state);
}
BENCHMARK(BM_BoxMesh_NativeCCD);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
    BM_BoxMesh_LibCCD(benchmark::State& state) {
  static TestHarness harness(kBoxMeshPath, "boxmesh.xml (libccd)",
                             mjDSBL_NATIVECCD);
  harness.RunBenchmark(state);
}
BENCHMARK(BM_BoxMesh_LibCCD);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_BoxBox(benchmark::State& state) {
  static TestHarness harness(kBoxBoxPath, "box.xml (BoxBox)", mjDSBL_NATIVECCD);
  harness.RunBenchmark(state);
}
BENCHMARK(BM_BoxBox);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_BoxBox_NativeCCD(benchmark::State& state) {
  mjCOLLISIONFUNC[mjGEOM_BOX][mjGEOM_BOX] = mjc_Convex;
  static TestHarness harness(kBoxBoxPath, "box.xml (NativeCCD)");
  harness.RunBenchmark(state);
  mjCOLLISIONFUNC[mjGEOM_BOX][mjGEOM_BOX] = mjc_BoxBox;
}
BENCHMARK(BM_BoxBox_NativeCCD);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
    BM_Ellipsoid_NativeCCD(benchmark::State& state) {
  static TestHarness harness(kEllipsoidPath, "ellipsoid.xml (nativeccd)");
  harness.RunBenchmark(state);
}
BENCHMARK(BM_Ellipsoid_NativeCCD);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
    BM_Ellipsoid_LibCCD(benchmark::State& state) {
  static TestHarness harness(kEllipsoidPath, "ellipsoid.xml (libccd)",
                             mjDSBL_NATIVECCD);
  harness.RunBenchmark(state);
}
BENCHMARK(BM_Ellipsoid_LibCCD);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_Mixed_NativeCCD(benchmark::State& state) {
  static TestHarness harness(kMixedPath, "mixed.xml (nativeccd)");
  harness.RunBenchmark(state);
}
BENCHMARK(BM_Mixed_NativeCCD);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_Mixed_LibCCD(benchmark::State& state) {
  static TestHarness harness(kMixedPath, "mixed.xml (libccd)",
                             mjDSBL_NATIVECCD);
  harness.RunBenchmark(state);
}
BENCHMARK(BM_Mixed_LibCCD);

}  // namespace
}  // namespace mujoco
