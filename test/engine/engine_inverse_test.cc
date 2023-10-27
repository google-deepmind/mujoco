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

// Tests for engine/engine_inverse.c.

#include "src/engine/engine_inverse.h"

#include <string>

#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using InverseTest = MujocoTest;

const int kSteps = 70;
static const char* const kModelPath = "testdata/model.xml";

// test standard continuous-time inverse dynamics
TEST_F(InverseTest, ForwardInverseMatch) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  // simulate, call mj_forward
  for (int i = 0; i < kSteps; ++i) {
    mj_step(model, data);
  }
  mj_forward(model, data);

  // call built-in testing function
  mj_compareFwdInv(model, data);

  // expect mismatch to be small
  mjtNum epsilon = 1e-10;
  EXPECT_LT(data->solver_fwdinv[0], epsilon);
  EXPECT_LT(data->solver_fwdinv[1], epsilon);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// test discrete-time inverse dynamics
TEST_F(InverseTest, DiscreteInverseMatch) {
  // load and allocate
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  int nv = model->nv;
  mjData* data = mj_makeData(model);
  int nstate = mj_stateSize(model, mjSTATE_INTEGRATION);
  mjtNum* state = (mjtNum*)mju_malloc(nstate * sizeof(mjtNum));
  mjtNum* qvel_next = (mjtNum*)mju_malloc(nv * sizeof(mjtNum));
  mjtNum* qacc_fd = (mjtNum*)mju_malloc(nv * sizeof(mjtNum));

  for (auto integrator : {mjINT_EULER, mjINT_IMPLICIT, mjINT_IMPLICITFAST}) {
    model->opt.integrator = integrator;
    for (bool invdiscrete : {false, true}) {
      // simulate
      mj_resetData(model, data);
      for (int i = 0; i < kSteps; ++i) {
        mj_step(model, data);
      }

      // save state
      mj_getState(model, data, state, mjSTATE_INTEGRATION);

      // call step, save new qvel
      mj_step(model, data);
      mju_copy(qvel_next, data->qvel, nv);

      // reset the state, compute discrete-time (finite-differenced) qacc
      mj_setState(model, data, state, mjSTATE_INTEGRATION);
      mju_sub(qacc_fd, qvel_next, data->qvel, nv);
      mju_scl(qacc_fd, qacc_fd, 1/model->opt.timestep, nv);

      // call mj_forward, overwrite qacc with qacc_fd
      mj_forward(model, data);
      mju_copy(data->qacc, qacc_fd, nv);

      // set/unset mjENBL_INVDISCRETE flag
      if (invdiscrete) {
        model->opt.enableflags |= mjENBL_INVDISCRETE;
      } else {
        model->opt.enableflags &= ~mjENBL_INVDISCRETE;
      }

      // call built-in testing function
      mj_compareFwdInv(model, data);

      // depending on mjENBL_INVDISCRETE flag, expect mismatch to be small/large
      if (invdiscrete) {
        mjtNum epsilon = 1e-9;
        EXPECT_LT(data->solver_fwdinv[0], epsilon);
        EXPECT_LT(data->solver_fwdinv[1], epsilon);
      } else {
        EXPECT_GT(data->solver_fwdinv[0], 1.0);
        EXPECT_GT(data->solver_fwdinv[1], 1.0);
      }
    }
  }

  // deallocate
  mju_free(qacc_fd);
  mju_free(qvel_next);
  mju_free(state);
  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
