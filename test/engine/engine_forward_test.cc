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

// Tests for engine/engine_forward.c.

#include "src/engine/engine_forward.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_io.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ForwardTest = MujocoTest;

TEST_F(ForwardTest, ActLimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.01"/>
    <worldbody>
      <body>
        <joint name="slide" type="slide" axis="1 0 0"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="slide" gainprm="100" biasprm="0 -100" biastype="affine"
      dynprm="10" dyntype="integrator"
      actlimited="true" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";

  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  data->ctrl[0] = 1.0;
  // integrating up from 0, we will hit the clamp after 99 steps
  for (int i=0; i<200; i++) {
    mj_step(model, data);
    // always greater than lower bound
    ASSERT_GT(data->act[0], -1);
    // after 99 steps we hit the upper bound
    if (i < 99) ASSERT_LT(data->act[0], 1);
    if (i >= 99) ASSERT_EQ(data->act[0], 1);
  }

  data->ctrl[0] = -1.0;
  // integrating down from 1, we will hit the clamp after 199 steps
  for (int i=0; i<300; i++) {
    mj_step(model, data);
    // always smaller than upper bound
    ASSERT_LT(data->act[0], model->actuator_actrange[1]);
    // after 199 steps we hit the lower bound
    if (i < 199) ASSERT_GT(data->act[0], model->actuator_actrange[0]);
    if (i >= 199) ASSERT_EQ(data->act[0], model->actuator_actrange[0]);
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}


}  // namespace
}  // namespace mujoco
