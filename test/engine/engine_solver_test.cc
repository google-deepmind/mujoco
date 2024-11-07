// Copyright 2023 DeepMind Technologies Limited
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

// Tests for engine/engine_solver.c

#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::NotNull;
using ::testing::Pointwise;
using ::std::vector;
using ::std::abs;
using ::std::max;

// compare two vectors, relative error (reduces size of large vector elements)
inline void ExpectEqRel(vector<mjtNum> v1, vector<mjtNum> v2, mjtNum rtol) {
  ASSERT_TRUE(v1.size() == v2.size());

  // make scale vector
  int n = v1.size();
  vector<mjtNum> scale(n);
  for (int i = 0; i < n; i++) {
    scale[i] = max(1.0, abs(v1[i]) + abs(v2[i]));
  }

  // scale and compare
  for (int i = 0; i < n; i++) {
    v1[i] /= scale[i];
    v2[i] /= scale[i];
  }
  EXPECT_THAT(v1, Pointwise(DoubleNear(rtol), v2));
}

using SolverTest = MujocoTest;

static const char* const kIlslandEfcPath =
    "engine/testdata/island/island_efc.xml";

// compare accelerations produced by CG solver with and without islands
TEST_F(SolverTest, IslandsEquivalent) {
  const std::string xml_path = GetTestDataFilePath(kIlslandEfcPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());
  model->opt.solver = mjSOL_CG;                 // use CG solver
  model->opt.tolerance = 0;                     // set tolerance to 0
  model->opt.enableflags &= ~mjENBL_ISLAND;     // disable islands

  int nv = model->nv;

  int state_size = mj_stateSize(model, mjSTATE_INTEGRATION);
  mjtNum* state = (mjtNum*) mju_malloc(sizeof(mjtNum)*state_size);
  mjtNum* qacc_diff = (mjtNum*) mju_malloc(sizeof(mjtNum)*nv);

  mjData* data_island = mj_makeData(model);
  mjData* data_noisland = mj_makeData(model);

  mjtNum rtol = 1e-5;

  for (bool warmstart : {true, false}) {
    if (warmstart) {
      model->opt.disableflags |= mjDSBL_WARMSTART;
    } else {
      model->opt.disableflags &= ~mjDSBL_WARMSTART;
    }
    mj_resetData(model, data_noisland);

    while (data_noisland->time < .3) {
      mj_step(model, data_noisland);

      mj_getState(model, data_noisland, state, mjSTATE_INTEGRATION);
      mj_setState(model, data_island, state, mjSTATE_INTEGRATION);

      mj_forward(model, data_noisland);

      model->opt.enableflags |= mjENBL_ISLAND;  // enable islands
      mj_forward(model, data_island);
      model->opt.enableflags &= ~mjENBL_ISLAND;  // disable islands

      ExpectEqRel(AsVector(data_noisland->qacc, nv),
                  AsVector(data_island->qacc, nv), rtol);
    }
  }

  mj_deleteData(data_noisland);
  mj_deleteData(data_island);
  mju_free(qacc_diff);
  mju_free(state);
  mj_deleteModel(model);
}

// compare qacc from 1 iteration of monolithic CG solver and one big island
TEST_F(SolverTest, OneBigIsland) {
  const std::string xml_path = GetTestDataFilePath(kIlslandEfcPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());
  model->opt.solver = mjSOL_CG;                 // use CG solver
  model->opt.disableflags |= mjDSBL_WARMSTART;  // disable warmstart
  model->opt.tolerance = 0;                     // set tolerance to 0
  model->opt.enableflags &= ~mjENBL_ISLAND;     // disable islands

  int state_size = mj_stateSize(model, mjSTATE_INTEGRATION);
  mjtNum* state = (mjtNum*) mju_malloc(sizeof(mjtNum)*state_size);

  mjData* data_island = mj_makeData(model);
  mjData* data_noisland = mj_makeData(model);

  int nv = model->nv;
  mjtNum rtol = 1e-7;

  // save current (default) iterations
  int iterations_default = model->opt.iterations;

  while (data_noisland->time < .2) {
    // step and copy the state to data_island
    mj_step(model, data_noisland);
    mj_getState(model, data_noisland, state, mjSTATE_INTEGRATION);
    mj_setState(model, data_island, state, mjSTATE_INTEGRATION);

    // set small number of iterations
    model->opt.iterations = 1;

    // call forward on data_noisland
    mj_forward(model, data_noisland);

    // enable islands
    model->opt.enableflags |= mjENBL_ISLAND;

    // call forward (just for smooth dynamics and to allocate islands)
    mj_forward(model, data_island);

    // overwrite island structure with one big island
    data_island->nisland = 1;
    data_island->island_dofnum[0] = nv;
    data_island->island_dofadr[0] = 0;
    for (int i = 0; i < nv; i++) {
      data_island->island_dofind[i] = data_island->dof_islandind[i] = i;
    }
    int nefc = data_island->nefc;
    data_island->island_efcnum[0] = nefc;
    data_island->island_efcadr[0] = 0;
    for (int i = 0; i < nefc; i++) data_island->island_efcind[i] = i;

    // solve using using one big island
    mj_fwdConstraint(model, data_island);

    // re-disable islands and reset iterations
    model->opt.enableflags &= ~mjENBL_ISLAND;
    model->opt.iterations = iterations_default;

    // compare accelerations (relative error)
    ExpectEqRel(AsVector(data_noisland->qacc, nv),
                AsVector(data_island->qacc, nv), rtol);
  }

  mj_deleteData(data_noisland);
  mj_deleteData(data_island);
  mju_free(state);
  mj_deleteModel(model);
}


}  // namespace
}  // namespace mujoco
