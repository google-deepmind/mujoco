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

//---------------------------------- flex elements ------------------------------------------------

// two parallel 1D flex strings along x, axes separated by 0.5 in y
constexpr char kFlexStringsXml[] = R"(
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <flexcomp name="string1" type="grid" dim="1" count="5 1 1" spacing="0.1 1 1"
              radius="0.01" mass="0.05" pos="0 0 0"/>
    <flexcomp name="string2" type="grid" dim="1" count="5 1 1" spacing="0.1 1 1"
              radius="0.01" mass="0.05" pos="0 0.5 0"/>
  </worldbody>
</mujoco>
)";

// 2D flex membrane in the xy plane with a 1D flex string above it along x
constexpr char kStringMembraneXml[] = R"(
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <flexcomp name="membrane" type="grid" dim="2" count="5 5 1" spacing="0.1 0.1 1"
              radius="0.005" mass="0.2" pos="0 0 0"/>
    <flexcomp name="string" type="grid" dim="1" count="5 1 1" spacing="0.1 1 1"
              radius="0.01" mass="0.05" pos="0 0 0.3"/>
  </worldbody>
</mujoco>
)";

// set the velocity of one flex vertex body (slide joints created by flexcomp)
static void SetVertVelocity(const mjModel* m, mjData* d, int f, int vert, const mjtNum v[3]) {
  int bid = m->flex_vertbodyid[m->flex_vertadr[f] + vert];
  for (int j = 0; j < m->body_jntnum[bid]; j++) {
    int jid = m->body_jntadr[bid] + j;
    ASSERT_EQ(m->jnt_type[jid], mjJNT_SLIDE);
    d->qvel[m->jnt_dofadr[jid]] = mju_dot3(v, m->jnt_axis + 3*jid);
  }
}

// set the velocity of every vertex body of a flex
static void SetFlexVelocity(const mjModel* m, mjData* d, int f, const mjtNum v[3]) {
  for (int i = 0; i < m->flex_vertnum[f]; i++) {
    SetVertVelocity(m, d, f, i, v);
  }
}

// minimum TOI of all elements of flex f1 against element e2 of flex f2; -1 if no impact
static mjtNum MinElemTOI(const mjModel* m, mjData* d, int f1, int f2, int e2, mjtNum horizon,
                         int* argmin = nullptr) {
  mjtNum best = -1;
  for (int e1 = 0; e1 < m->flex_elemnum[f1]; e1++) {
    mjtNum toi = mj_flexTOI(m, d, f1, e1, f2, e2, horizon, nullptr);
    if (toi >= 0 && (best < 0 || toi < best)) {
      best = toi;
      if (argmin) *argmin = e1;
    }
  }
  return best;
}

// parallel strings approaching: axes gap 0.5, radii 0.01 each, closing speed 2 => TOI = 0.24
TEST_F(ToiTest, FlexStringsHeadOn) {
  mjModel* model = LoadModel(kFlexStringsXml);
  mjData* data = mj_makeData(model);
  int f1 = mj_name2id(model, mjOBJ_FLEX, "string1");
  int f2 = mj_name2id(model, mjOBJ_FLEX, "string2");

  mjtNum v[3] = {0, 2, 0};
  SetFlexVelocity(model, data, f1, v);
  mj_forward(model, data);

  mjtNum fromto[6];
  mjtNum toi = mj_flexTOI(model, data, f1, 1, f2, 1, 1.0, fromto);
  EXPECT_NEAR(toi, 0.24, MjTol(1e-6, 1e-3));

  // witness points coincide at the contact, between the string axes (y = 0.49..0.51)
  EXPECT_NEAR(fromto[1], 0.49, MjTol(1e-5, 1e-2));
  EXPECT_NEAR(fromto[4], 0.49, MjTol(1e-5, 1e-2));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// separating and already-touching strings
TEST_F(ToiTest, FlexNoImpactAndTouching) {
  mjModel* model = LoadModel(kFlexStringsXml);
  mjData* data = mj_makeData(model);
  int f1 = mj_name2id(model, mjOBJ_FLEX, "string1");
  int f2 = mj_name2id(model, mjOBJ_FLEX, "string2");

  // separating
  mjtNum v[3] = {0, -2, 0};
  SetFlexVelocity(model, data, f1, v);
  mj_forward(model, data);
  EXPECT_EQ(mj_flexTOI(model, data, f1, 1, f2, 1, 1.0, nullptr), -1);

  // already touching: move string1 axes to y = 0.485, surface gap = 0.5 - 0.485 - 0.02 < 0
  mjtNum shift[3] = {0, 0.485, 0};
  SetFlexVelocity(model, data, f1, shift);
  mj_integratePos(model, data->qpos, data->qvel, 1.0);
  mju_zero(data->qvel, model->nv);
  mj_forward(model, data);
  EXPECT_EQ(mj_flexTOI(model, data, f1, 1, f2, 1, 1.0, nullptr), 0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// string translating down onto a membrane: axes gap 0.3, radii 0.01+0.005 => TOI = 0.285
TEST_F(ToiTest, StringOntoMembrane) {
  mjModel* model = LoadModel(kStringMembraneXml);
  mjData* data = mj_makeData(model);
  int fm = mj_name2id(model, mjOBJ_FLEX, "membrane");
  int fs = mj_name2id(model, mjOBJ_FLEX, "string");

  mjtNum v[3] = {0, 0, -1};
  SetFlexVelocity(model, data, fs, v);
  mj_forward(model, data);

  mjtNum toi = MinElemTOI(model, data, fm, fs, 1, 1.0);
  EXPECT_NEAR(toi, 0.285, MjTol(1e-6, 1e-3));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// pure deformation: a single membrane vertex rises toward the static string above it,
// stretching its elements; per-vertex motion must detect the impact
TEST_F(ToiTest, DeformingMembrane) {
  mjModel* model = LoadModel(kStringMembraneXml);
  mjData* data = mj_makeData(model);
  int fm = mj_name2id(model, mjOBJ_FLEX, "membrane");
  int fs = mj_name2id(model, mjOBJ_FLEX, "string");

  // find the center vertex of the membrane, at the origin
  mj_forward(model, data);
  int center = -1;
  for (int i = 0; i < model->flex_vertnum[fm]; i++) {
    const mjtNum* p = data->flexvert_xpos + 3*(model->flex_vertadr[fm] + i);
    if (mju_norm3(p) < 1e-10) {
      center = i;
      break;
    }
  }
  ASSERT_GE(center, 0) << "center vertex not found";

  // center vertex rises at 1 m/s; the string axis is at z = 0.3, gap = 0.3 - 0.015
  mjtNum v[3] = {0, 0, 1};
  SetVertVelocity(model, data, fm, center, v);
  mj_forward(model, data);

  int argmin = -1;
  mjtNum toi = MinElemTOI(model, data, fm, fs, 1, 1.0, &argmin);
  EXPECT_NEAR(toi, 0.285, MjTol(1e-6, 1e-3));

  // oracle: advance positions to the TOI (slide joints integrate exactly), expect touching
  mj_integratePos(model, data->qpos, data->qvel, toi);
  mj_forward(model, data);
  EXPECT_EQ(mj_flexTOI(model, data, fm, argmin, fs, 1, 1.0, nullptr), 0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// invalid flex and element ids produce errors
TEST_F(ToiTest, FlexInvalidIds) {
  mjModel* model = LoadModel(kFlexStringsXml);
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  EXPECT_THAT(MjuErrorMessageFrom(mj_flexTOI)(model, data, 0, 99, 1, 0, 1.0, nullptr),
              HasSubstr("invalid element id"));
  EXPECT_THAT(MjuErrorMessageFrom(mj_flexTOI)(model, data, 5, 0, 1, 0, 1.0, nullptr),
              HasSubstr("invalid flex id"));

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
