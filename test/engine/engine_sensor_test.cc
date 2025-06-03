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

// Tests for engine/engine_sensor.c.

#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_spatial.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

const mjtNum tol = 1e-14;  // nearness tolerance for floating point numbers

// returns as a vector the measured values from sensor with index `id`
static std::vector<mjtNum> GetSensor(const mjModel* model,
                                     const mjData* data,
                                     int id) {
  return std::vector<mjtNum>(
      data->sensordata + model->sensor_adr[id],
      data->sensordata + model->sensor_adr[id] + model->sensor_dim[id]);
}

using ::testing::Pointwise;
using ::testing::DoubleNear;
using ::testing::StrEq;

using SensorTest = MujocoTest;

// --------------------- test sensor disableflag  ------------------------------

// hand-picked positions and orientations for simple expected values
TEST_F(SensorTest, DisableSensors) {
  constexpr char xml[] = R"(
  <mujoco>
    <sensor>
      <clock/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  // before calling anything, check that sensors are initialised to 0
  EXPECT_EQ(data->sensordata[0], 0.0);

  // call mj_step, mj_step1, expect clock to be incremented by timestep
  mj_step(model, data);
  mj_step1(model, data);
  EXPECT_EQ(data->sensordata[0], model->opt.timestep);

  // disable sensors, call mj_step, mj_step1, expect clock to not increment
  model->opt.disableflags |= mjDSBL_SENSOR;
  mj_step(model, data);
  mj_step1(model, data);
  EXPECT_EQ(data->time, 2*model->opt.timestep);
  EXPECT_EQ(data->sensordata[0], model->opt.timestep);

  // re-enable sensors, call mj_step, mj_step1, expect clock to match time
  model->opt.disableflags = 0;
  mj_step(model, data);
  mj_step1(model, data);
  EXPECT_EQ(data->time, data->sensordata[0]);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// --------------------- test relative frame sensors  --------------------------

using RelativeFrameSensorTest = MujocoTest;

// hand-picked positions and orientations for simple expected values
TEST_F(RelativeFrameSensorTest, ReferencePosMat) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="reference" pos="3 -4 0" xyaxes="4 3 0 -3 4 0"/>
      <site name="object" pos="4 3 0" xyaxes="3 -4 0 4 3 0"/>
    </worldbody>
    <sensor>
      <framepos objtype="site" objname="object"
                reftype="xbody" refname="reference"/>
      <framexaxis objtype="site" objname="object"
                  reftype="xbody" refname="reference"/>
      <frameyaxis objtype="site" objname="object"
                  reftype="xbody" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // compare actual and expected values
  std::vector pos = GetSensor(model, data, 0);
  EXPECT_THAT(pos, Pointwise(DoubleNear(tol), {5, 5, 0}));

  std::vector xaxis = GetSensor(model, data, 1);
  EXPECT_THAT(xaxis, Pointwise(DoubleNear(tol), {0, -1, 0}));

  std::vector yaxis = GetSensor(model, data, 2);
  EXPECT_THAT(yaxis, Pointwise(DoubleNear(tol), {1, 0, 0}));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// orientations given by quaternion and by orientation matrix are identical
TEST_F(RelativeFrameSensorTest, ReferenceQuatMat) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="reference" euler="10 20 30"/>
      <site name="object" euler="20 40 60"/>
    </worldbody>
    <sensor>
      <framexaxis objtype="site" objname="object"
                  reftype="site" refname="reference"/>
      <frameyaxis objtype="site" objname="object"
                  reftype="site" refname="reference"/>
      <framezaxis objtype="site" objname="object"
                  reftype="site" refname="reference"/>
      <framequat objtype="site" objname="object"
                  reftype="site" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  // call mj_forward and convert orientation matrix to quaternion
  mj_forward(model, data);
  mjtNum mat[9], converted_quat[4];
  mju_transpose(mat, data->sensordata, 3, 3);
  mju_mat2Quat(converted_quat, mat);

  // compare quaternion sensor and quat derived from orientation matrix
  std::vector quat = GetSensor(model, data, 3);
  EXPECT_THAT(quat, Pointwise(DoubleNear(tol), converted_quat));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// compare global frame and initially co-located relative frame on same body
TEST_F(RelativeFrameSensorTest, ReferencePosMatQuat) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <freejoint/>
        <site name="reference"/>
        <geom name="object" euler="20 40 60" pos="1 2 3" size="1"/>
      </body>
    </worldbody>
    <sensor>
      <framepos objtype="geom" objname="object"/>
      <framexaxis objtype="geom" objname="object"/>
      <frameyaxis objtype="geom" objname="object"/>
      <framezaxis objtype="geom" objname="object"/>
      <framequat objtype="geom" objname="object"/>
      <framepos objtype="geom" objname="object"
                reftype="site" refname="reference"/>
      <framexaxis objtype="geom" objname="object"
                  reftype="site" refname="reference"/>
      <frameyaxis objtype="geom" objname="object"
                  reftype="site" refname="reference"/>
      <framezaxis objtype="geom" objname="object"
                  reftype="site" refname="reference"/>
      <framequat objtype="geom" objname="object"
                  reftype="site" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  constexpr int nsensordata = 32;
  ASSERT_EQ(model->nsensordata, nsensordata);
  mjData* data = mj_makeData(model);

  // call mj_forward, save global sensors (colocated with reference frame)
  mj_forward(model, data);
  std::vector expected_values(data->sensordata, data->sensordata+nsensordata/2);

  // set qpos to arbitrary values, call mj_forward
  for (int i=0; i < 7; i++) {
    data->qpos[i] = i+1;
  }
  mj_forward(model, data);

  // get values from relative sensors after moving the object
  std::vector actual_values(data->sensordata+nsensordata/2,
                            data->sensordata+nsensordata);

  // object and reference have moved together, we expect values to not change
  EXPECT_THAT(actual_values, Pointwise(DoubleNear(tol), expected_values));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// hand-picked velocities and orientations for simple expected values
TEST_F(RelativeFrameSensorTest, FrameVelLinearFixed) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body xyaxes="1 -1 0 1 1 0">
        <joint type="slide" axis="1 0 0"/>
        <geom name="reference" size="1"/>
      </body>
      <body>
        <joint type="slide" axis="1 0 0"/>
        <geom name="object" size="1"/>
      </body>
    </worldbody>
    <sensor>
      <framelinvel objtype="geom" objname="object"
                   reftype="geom" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);
  data->qvel[0] = mju_sqrt(2);
  data->qvel[1] = 1;
  mj_forward(model, data);

  // compare to expected values
  std::vector linvel = GetSensor(model, data, 0);
  const mjtNum expected_linvel[3] = {-mju_sqrt(0.5), mju_sqrt(0.5), 0};
  EXPECT_THAT(linvel, Pointwise(DoubleNear(tol), expected_linvel));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// object and reference in the same body, expect angular velocities to be zero
TEST_F(RelativeFrameSensorTest, FrameVelAngFixed) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint type="hinge" axis="1 2 3"/>
        <geom name="reference" size="1" pos="1 2 3"/>
        <geom name="object" size="1" pos="-3 -2 -1"/>
      </body>
    </worldbody>
    <sensor>
      <frameangvel objtype="geom" objname="object"
                   reftype="geom" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  // set joint velocities and call forward dynamics
  data->qvel[0] = 1;
  mj_forward(model, data);

  // obj and ref rotate together, relative angular velocities should be zero
  std::vector angvel = GetSensor(model, data, 0);
  EXPECT_THAT(angvel, Pointwise(DoubleNear(tol), {0, 0, 0}));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// object and reference rotate on the same global axis
TEST_F(RelativeFrameSensorTest, FrameVelAngOpposing) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body xyaxes="0 -1 0 1 0 0">
        <joint type="hinge" axis="0 1 0"/>
        <geom name="reference" size="1"/>
      </body>
      <body>
        <joint type="hinge" axis="1 0 0"/>
        <geom name="object" size="1" pos="-3 -2 -1"/>
      </body>
    </worldbody>
    <sensor>
      <frameangvel objtype="geom" objname="object"
                   reftype="geom" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  // set joint velocities and call forward dynamics
  data->qvel[0] = -1;
  data->qvel[1] = 1;
  mj_forward(model, data);

  // obj and ref rotate on same axis, we can just difference the velocities
  std::vector angvel = GetSensor(model, data, 0);
  const mjtNum expected_angvel[3] = {0, data->qvel[1]-data->qvel[0], 0};
  EXPECT_THAT(angvel, Pointwise(DoubleNear(tol), expected_angvel));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// two arbitrary frames, compare velocity sensors and fin-diffed positions
TEST_F(RelativeFrameSensorTest, FrameVelGeneral) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="1 2 3" euler="10 20 30">
        <joint type="hinge" axis="2 3 4"/>
        <geom name="reference" size="1" pos="0 1 2"/>
      </body>
      <body pos="-3 -2 -1" euler="20 40 60">
        <joint type="hinge" axis="2 3 4"/>
        <geom name="object" size="1" pos="1 2 3"/>
      </body>
    </worldbody>
    <sensor>
      <framepos objtype="geom" objname="object"
                reftype="geom" refname="reference"/>
      <framequat objtype="geom" objname="object"
                 reftype="geom" refname="reference"/>
      <framelinvel objtype="geom" objname="object"
                   reftype="geom" refname="reference"/>
      <frameangvel objtype="geom" objname="object"
                   reftype="geom" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);
  mjtNum dt = 1e-6;  // timestep used for finite differencing

  // set (arbitrary) joint velocities and call forward dynamics
  data->qvel[0] = 1;
  data->qvel[1] = -1;
  mj_forward(model, data);

  // save measured linear and angular velocities as vectors
  std::vector linvel = GetSensor(model, data, 2);
  std::vector angvel = GetSensor(model, data, 3);

  // save current position, quaternion as arrays
  mjtNum pos0[3], quat0[4];
  mju_copy3(pos0, data->sensordata);
  mju_copy4(quat0, data->sensordata+3);

  // explicit Euler integration with small dt
  mju_addToScl(data->qpos, data->qvel, dt, 2);

  // call mj_forward again, save new position and quaternion
  mj_forward(model, data);
  mjtNum pos1[3], quat1[4];
  mju_copy3(pos1, data->sensordata);
  mju_copy4(quat1, data->sensordata+3);

  // compute expected linear velocities using finite differencing
  mjtNum linvel_findiff[3];
  mju_sub3(linvel_findiff, pos1, pos0);
  mju_scl3(linvel_findiff, linvel_findiff, 1/dt);

  // compute expected angular velocities using finite differencing
  mjtNum dquat[4], angvel_findiff[3];
  mju_negQuat(quat0, quat0);
  mju_mulQuat(dquat, quat1, quat0);
  mju_quat2Vel(angvel_findiff, dquat, dt);

  // compare analytic and finite-differenced relative velocities
  EXPECT_THAT(linvel, Pointwise(DoubleNear(10*dt), linvel_findiff));
  EXPECT_THAT(angvel, Pointwise(DoubleNear(10*dt), angvel_findiff));


  mj_deleteData(data);
  mj_deleteModel(model);
}

// ------------------------- general sensor tests  -----------------------------
using SensorTest = MujocoTest;

TEST_F(SensorTest, EnableEnergy) {
  constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 -5">
     <flag energy="enable"/>
    </option>
    <worldbody>
      <body pos="0 0 2">
        <geom size="1" mass="3"/>
        <freejoint/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  mj_forward(model, data);
  EXPECT_EQ(data->energy[0], 2*3*5);

  model->opt.enableflags &= ~mjENBL_ENERGY;
  mj_forward(model, data);
  EXPECT_EQ(data->energy[0], 0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(SensorTest, PotentialEnergy) {
  constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 -5"/>
    <worldbody>
      <body pos="0 0 2">
        <geom size="1" mass="3"/>
        <freejoint/>
      </body>
    </worldbody>
    <sensor>
      <e_potential/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  mj_forward(model, data);
  EXPECT_EQ(data->sensordata[0], 2*3*5);

  data->qpos[2] = 7;
  mj_forward(model, data);
  EXPECT_EQ(data->sensordata[0], 7*3*5);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(SensorTest, PotentialEnergyFreeJointSpring) {
  constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <body>
        <geom size="1" mass="3"/>
        <joint type="free" stiffness="2"/>
      </body>
    </worldbody>
    <sensor>
      <e_potential/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);
  data->qpos[0] = 1;
  data->qpos[1] = 2;
  data->qpos[2] = 3;
  mj_forward(model, data);
  EXPECT_EQ(data->sensordata[0], 0.5*2*14);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(SensorTest, KineticEnergy) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 2">
        <geom size="1" mass="3"/>
        <freejoint/>
      </body>
    </worldbody>
    <sensor>
      <e_kinetic/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  while (data->time < 1.5) {
    mj_step(model, data);
  }
  mj_forward(model, data);

  mjtNum mass = 3;
  mjtNum speed = data->time * mju_norm3(model->opt.gravity);
  EXPECT_FLOAT_EQ(data->sensordata[0], 0.5 * mass * speed * speed);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// test clock sensor
TEST_F(SensorTest, Clock) {
  constexpr char xml[] = R"(
  <mujoco>
    <option timestep="1e-3"/>
    <sensor>
      <clock/>
      <clock name="clampedclock" cutoff="3e-3"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  // call step 4 times, checking that clock works as expected
  for (int i=0; i < 5; i++) {
    mj_step(model, data);
    mj_step1(model, data);  // update values of position-based sensors
    EXPECT_EQ(data->sensordata[0], data->time);
    EXPECT_EQ(data->sensordata[1], mju_min(data->time, 3e-3));
  }

  // check names
  const char* name0 = mj_id2name(model, mjOBJ_SENSOR, 0);
  EXPECT_EQ(name0, nullptr);
  const char* name1 = mj_id2name(model, mjOBJ_SENSOR, 1);
  EXPECT_THAT(name1, StrEq("clampedclock"));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// test clock sensor
TEST_F(SensorTest, CollisionSequential) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="plane" type="plane" size="1 1 1"/>
      <geom name="sphere1" pos="0 0 1" size="0.2"/>
      <geom name="sphere2" pos="1 0 1" size="0.3"/>
    </worldbody>
    <sensor>
      <distance name="0"  geom1="plane"   geom2="sphere1" cutoff="1"/>
      <distance name="1"  geom1="sphere2" geom2="plane" cutoff="1"/>
      <distance name="2"  geom1="sphere1" geom2="sphere2" cutoff="1"/>
      <normal   name="3"  geom1="plane"   geom2="sphere1" cutoff="1"/>
      <normal   name="4"  geom1="sphere2" geom2="plane" cutoff="1"/>
      <normal   name="5"  geom1="sphere1" geom2="sphere2" cutoff="1"/>
      <fromto   name="6"  geom1="plane"   geom2="sphere1" cutoff="1"/>
      <fromto   name="7"  geom1="sphere2" geom2="plane" cutoff="1"/>
      <fromto   name="8"  geom1="sphere1" geom2="sphere2" cutoff="1"/>
      <!-- sequential sensors with identical signature -->
      <distance name="9"  geom1="plane"   geom2="sphere1" cutoff="1"/>
      <fromto   name="10" geom1="plane"   geom2="sphere1" cutoff="1"/>
      <normal   name="11" geom1="plane"   geom2="sphere1" cutoff="1"/>
      <normal   name="12"  geom1="sphere1" geom2="sphere2" cutoff="1"/>
      <fromto   name="13"  geom1="sphere1" geom2="sphere2" cutoff="1"/>
      <distance name="14"  geom1="sphere1" geom2="sphere2" cutoff="1"/>

    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  EXPECT_DOUBLE_EQ(data->sensordata[0], 0.8);
  EXPECT_DOUBLE_EQ(data->sensordata[1], 0.7);
  EXPECT_DOUBLE_EQ(data->sensordata[2], 0.5);

  mjtNum eps = 1e-14;

  EXPECT_THAT(GetSensor(model, data, 3),
              Pointwise(DoubleNear(eps), std::vector<mjtNum>{0, 0, 1}));
  EXPECT_THAT(GetSensor(model, data, 4),
              Pointwise(DoubleNear(eps), std::vector<mjtNum>{0, 0, -1}));
  EXPECT_THAT(GetSensor(model, data, 5),
              Pointwise(DoubleNear(eps), std::vector<mjtNum>{1, 0, 0}));
  EXPECT_THAT(GetSensor(model, data, 6),
              Pointwise(DoubleNear(eps),
                        std::vector<mjtNum>{0, 0, 0, 0, 0, .8}));
  EXPECT_THAT(GetSensor(model, data, 7),
              Pointwise(DoubleNear(eps),
                        std::vector<mjtNum>{1, 0, .7, 1, 0, 0}));
  EXPECT_THAT(GetSensor(model, data, 8),
              Pointwise(DoubleNear(eps),
                        std::vector<mjtNum>{.2, 0, 1, .7, 0, 1}));

  EXPECT_THAT(GetSensor(model, data, 9),
              Pointwise(DoubleNear(eps), GetSensor(model, data, 0)));
  EXPECT_THAT(GetSensor(model, data, 10),
              Pointwise(DoubleNear(eps), GetSensor(model, data, 6)));
  EXPECT_THAT(GetSensor(model, data, 11),
              Pointwise(DoubleNear(eps), GetSensor(model, data, 3)));
  EXPECT_THAT(GetSensor(model, data, 12),
              Pointwise(DoubleNear(eps), GetSensor(model, data, 5)));
  EXPECT_THAT(GetSensor(model, data, 13),
              Pointwise(DoubleNear(eps), GetSensor(model, data, 8)));
  EXPECT_THAT(GetSensor(model, data, 14),
              Pointwise(DoubleNear(eps), GetSensor(model, data, 2)));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// ------------------------- camera sensor tests  ------------------------------

// test clock sensor
TEST_F(SensorTest, CameraProjection) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="1.1 0 1">
        <geom type="box" size=".1 .6 .375"/>
        <site name="frontorigin" pos="-.1  .6  .375"/>
        <site name="frontcorner" pos="-.1 -.6 -.375"/>
      </body>
      <body pos="-1.1 0 1">
        <geom type="box" size=".1 .6 .375"/>
        <site name="backcenter" pos="-.1 0 0"/>
      </body>
      <camera pos="0 0 1" xyaxes="0 -1 0 0 0 1" fovy="41.11209"
              resolution="1920 1200" name="fixedcamera"/>
    </worldbody>
    <sensor>
      <camprojection site="frontorigin" camera="fixedcamera"/>
      <camprojection site="frontcorner" camera="fixedcamera"/>
      <camprojection site="backcenter" camera="fixedcamera"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  // call step to update sensors
  mj_step(model, data);
  mj_step1(model, data);  // update values of position-based sensors
  EXPECT_THAT(model->cam_resolution[0], 1920);
  EXPECT_THAT(model->cam_resolution[1], 1200);
  mjtNum eps = 1e-4;
  EXPECT_NEAR(data->sensordata[0], 0, eps);
  EXPECT_NEAR(data->sensordata[1], 0, eps);
  EXPECT_NEAR(data->sensordata[2], 1920, eps);
  EXPECT_NEAR(data->sensordata[3], 1200, eps);
  EXPECT_NEAR(data->sensordata[4], 960, eps);
  EXPECT_NEAR(data->sensordata[5], 600, eps);

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
