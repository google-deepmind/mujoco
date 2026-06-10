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

// Tests for engine/engine_toi.c.

#include "src/engine/engine_toi.h"

#include <cmath>

#include <mujoco/mujoco.h>
#include <mujoco/mjtype.h>
#include "test/fixture.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::NotNull;

using ToiTest = MujocoTest;

// two free spheres of radius 0.1, centers at x=0 and x=1
constexpr char kTwoSpheresXml[] = R"(
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="ball1">
      <freejoint/>
      <geom name="sphere1" type="sphere" size="0.1"/>
    </body>
    <body name="ball2" pos="1 0 0">
      <freejoint/>
      <geom name="sphere2" type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)";

// thin static wall at x=0.5, free sphere of radius 0.05 at the origin
constexpr char kThinWallXml[] = R"(
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <geom name="wall" type="box" size="0.005 0.5 0.5" pos="0.5 0 0"/>
    <body name="bullet">
      <freejoint/>
      <geom name="bullet" type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
)";

// thin blade centered on a hinge about z, static sphere at (0, 0.5, 0)
constexpr char kPaddleXml[] = R"(
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <geom name="target" type="sphere" size="0.05" pos="0 0.5 0"/>
    <body name="paddle">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom name="blade" type="box" size="0.5 0.02 0.02"/>
    </body>
  </worldbody>
</mujoco>
)";

// blade on a hinge plus a free sphere approaching it
constexpr char kPaddleBallXml[] = R"(
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="paddle">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom name="blade" type="box" size="0.5 0.02 0.02"/>
    </body>
    <body name="ball" pos="0 0.6 0">
      <freejoint/>
      <geom name="ball" type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
)";

// plane and a free sphere (unsupported pair for TOI)
constexpr char kPlaneSphereXml[] = R"(
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 1"/>
    <body name="ball" pos="0 0 1">
      <freejoint/>
      <geom name="ball" type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)";

// two free tetrahedral meshes, centers at x=0 and x=1
constexpr char kTwoMeshesXml[] = R"(
<mujoco>
  <option gravity="0 0 0"/>
  <asset>
    <mesh name="tetra" vertex="0 0 0  0.1 0 0  0 0.1 0  0 0 0.1"/>
  </asset>
  <worldbody>
    <body name="m1">
      <freejoint/>
      <geom name="mesh1" type="mesh" mesh="tetra"/>
    </body>
    <body name="m2" pos="1 0 0">
      <freejoint/>
      <geom name="mesh2" type="mesh" mesh="tetra"/>
    </body>
  </worldbody>
</mujoco>
)";

// load a model, asserting success
static mjModel* LoadModel(const char* xml) {
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model, NotNull()) << error;
  return model;
}

// head-on spheres: gap 0.8, closing speed 4 => TOI = 0.2
TEST_F(ToiTest, HeadOnSpheres) {
  mjModel* model = LoadModel(kTwoSpheresXml);
  mjData* data = mj_makeData(model);
  data->qvel[0] = 2;   // ball1 vx
  data->qvel[6] = -2;  // ball2 vx
  mj_forward(model, data);

  int g1 = mj_name2id(model, mjOBJ_GEOM, "sphere1");
  int g2 = mj_name2id(model, mjOBJ_GEOM, "sphere2");
  mjtNum fromto[6];
  mjtNum toi = mj_geomTOI(model, data, g1, g2, 1.0, fromto);

  EXPECT_NEAR(toi, 0.2, MjTol(1e-6, 1e-3));

  // witness points coincide at the analytic contact point (0.5, 0, 0)
  for (int i = 0; i < 3; i++) {
    mjtNum expected = i == 0 ? 0.5 : 0;
    EXPECT_NEAR(fromto[i], expected, MjTol(1e-5, 1e-2));
    EXPECT_NEAR(fromto[3+i], expected, MjTol(1e-5, 1e-2));
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

// overlapping spheres: TOI = 0 regardless of velocities
TEST_F(ToiTest, AlreadyPenetrating) {
  mjModel* model = LoadModel(kTwoSpheresXml);
  mjData* data = mj_makeData(model);

  // move ball2 to overlap ball1 (free joint qpos is the absolute world position)
  data->qpos[7] = 0.15;  // ball2 center at x = 0.15, overlap 0.05

  for (mjtNum v : {2.0, -2.0, 0.0}) {
    data->qvel[0] = v;
    mj_forward(model, data);
    mjtNum toi = mj_geomTOI(model, data, 0, 1, 1.0, nullptr);
    EXPECT_EQ(toi, 0) << "velocity " << v;
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

// separating or parallel-moving spheres: no impact
TEST_F(ToiTest, NoImpact) {
  mjModel* model = LoadModel(kTwoSpheresXml);
  mjData* data = mj_makeData(model);

  // separating
  data->qvel[0] = -2;
  data->qvel[6] = 2;
  mj_forward(model, data);
  mjtNum fromto[6] = {1, 1, 1, 1, 1, 1};
  EXPECT_EQ(mj_geomTOI(model, data, 0, 1, 1.0, fromto), -1);

  // fromto is zeroed on no impact
  for (int i = 0; i < 6; i++) {
    EXPECT_EQ(fromto[i], 0);
  }

  // parallel motion
  data->qvel[0] = 2;
  data->qvel[6] = 2;
  mj_forward(model, data);
  EXPECT_EQ(mj_geomTOI(model, data, 0, 1, 1.0, nullptr), -1);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// impact at t=0.2 is not found with horizon 0.1
TEST_F(ToiTest, HorizonCutoff) {
  mjModel* model = LoadModel(kTwoSpheresXml);
  mjData* data = mj_makeData(model);
  data->qvel[0] = 2;
  data->qvel[6] = -2;
  mj_forward(model, data);

  EXPECT_EQ(mj_geomTOI(model, data, 0, 1, 0.1, nullptr), -1);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// fast sphere tunnels through a thin wall within one horizon: TOI is found even
// though the endpoint position is past the wall (the discrete-collision failure case)
TEST_F(ToiTest, TunnelingThinWall) {
  mjModel* model = LoadModel(kThinWallXml);
  mjData* data = mj_makeData(model);
  data->qvel[0] = 100;
  mj_forward(model, data);

  int wall = mj_name2id(model, mjOBJ_GEOM, "wall");
  int bullet = mj_name2id(model, mjOBJ_GEOM, "bullet");
  mjtNum fromto[6];
  mjtNum toi = mj_geomTOI(model, data, bullet, wall, 0.01, fromto);

  // gap = 0.5 - 0.005 - 0.05 = 0.445, closing speed 100
  EXPECT_NEAR(toi, 4.45e-3, MjTol(1e-7, 1e-4));
  EXPECT_NEAR(fromto[0], 0.495, MjTol(1e-5, 1e-2));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// rotating blade with zero linear velocity hits a static sphere: angular-only impact
TEST_F(ToiTest, AngularOnlyPaddle) {
  mjModel* model = LoadModel(kPaddleXml);
  mjData* data = mj_makeData(model);
  mjtNum omega = 1;
  data->qvel[0] = omega;
  mj_forward(model, data);

  int blade = mj_name2id(model, mjOBJ_GEOM, "blade");
  int target = mj_name2id(model, mjOBJ_GEOM, "target");
  mjtNum toi = mj_geomTOI(model, data, blade, target, 2.0, nullptr);

  // analytic estimate: blade side face reaches the sphere at
  // theta = pi/2 - asin((0.05 + 0.02)/0.5)
  mjtNum expected = (mjPI/2 - std::asin(0.07/0.5)) / omega;
  EXPECT_GT(toi, 0);
  EXPECT_NEAR(toi, expected, 0.05*expected);

  // primary oracle: advancing the hinge to the TOI angle (exact integration for a
  // hinge through the geom center) must close the gap to within tolerance
  data->qpos[0] = omega*toi;
  mj_forward(model, data);
  EXPECT_LE(mj_geomDistance(model, data, blade, target, 1.0, nullptr), MjTol(1e-5, 1e-2));

  // strictly before the TOI there is no contact
  data->qpos[0] = 0.9*omega*toi;
  mj_forward(model, data);
  EXPECT_GT(mj_geomDistance(model, data, blade, target, 1.0, nullptr), 0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// mixed motion: rotating blade and translating sphere
TEST_F(ToiTest, MixedLinearAngular) {
  mjModel* model = LoadModel(kPaddleBallXml);
  mjData* data = mj_makeData(model);
  mjtNum omega = 1;
  mjtNum vy = -0.05;
  data->qvel[0] = omega;  // hinge
  data->qvel[2] = vy;     // ball vy (free joint dofs follow the hinge dof)
  mj_forward(model, data);

  int blade = mj_name2id(model, mjOBJ_GEOM, "blade");
  int ball = mj_name2id(model, mjOBJ_GEOM, "ball");
  mjtNum toi = mj_geomTOI(model, data, blade, ball, 2.0, nullptr);
  EXPECT_GT(toi, 0);
  EXPECT_LE(toi, 2.0);

  // oracle: advance hinge angle and ball position to the TOI (both exact: the blade
  // only rotates about its center, the ball only translates), expect near-contact
  data->qpos[0] = omega*toi;
  data->qpos[2] += vy*toi;
  mj_forward(model, data);
  EXPECT_LE(mj_geomDistance(model, data, blade, ball, 1.0, nullptr), MjTol(1e-5, 1e-2));

  // strictly before the TOI there is no contact
  data->qpos[0] = 0.9*omega*toi;
  data->qpos[2] = 0.6 + 0.9*vy*toi;
  mj_forward(model, data);
  EXPECT_GT(mj_geomDistance(model, data, blade, ball, 1.0, nullptr), 0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// unsupported geom types produce an error
TEST_F(ToiTest, UnsupportedGeomType) {
  mjModel* model = LoadModel(kPlaneSphereXml);
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  int floor = mj_name2id(model, mjOBJ_GEOM, "floor");
  int ball = mj_name2id(model, mjOBJ_GEOM, "ball");
  EXPECT_THAT(MjuErrorMessageFrom(mj_geomTOI)(model, data, floor, ball, 1.0, nullptr),
              HasSubstr("unsupported geom type"));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// approaching convex meshes: TOI is consistent with mj_geomDistance at advanced poses
TEST_F(ToiTest, MeshPair) {
  mjModel* model = LoadModel(kTwoMeshesXml);
  mjData* data = mj_makeData(model);
  mjtNum vx = 2;
  data->qvel[0] = vx;
  mj_forward(model, data);

  int g1 = mj_name2id(model, mjOBJ_GEOM, "mesh1");
  int g2 = mj_name2id(model, mjOBJ_GEOM, "mesh2");
  mjtNum toi = mj_geomTOI(model, data, g1, g2, 1.0, nullptr);
  EXPECT_GT(toi, 0);
  EXPECT_LE(toi, 1.0);

  // oracle: advance mesh1 by vx*toi (pure translation), expect near-contact
  data->qpos[0] += vx*toi;
  mj_forward(model, data);
  EXPECT_LE(mj_geomDistance(model, data, g1, g2, 1.0, nullptr), MjTol(1e-5, 1e-2));

  // strictly before the TOI there is no contact
  data->qpos[0] = 0.5*vx*toi;
  mj_forward(model, data);
  EXPECT_GT(mj_geomDistance(model, data, g1, g2, 1.0, nullptr), 0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
