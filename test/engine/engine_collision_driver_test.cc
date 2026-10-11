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

// Tests for engine/engine_collision_driver.c.

#include "src/engine/engine_collision_driver.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using MjCollisionTest = MujocoTest;
using GeomPair = std::pair<std::string, std::string>;
using ::testing::ElementsAre;
using ::testing::IsEmpty;
using ::testing::NotNull;

// Returns a sorted list of pairs of colliding geom names, where each pair of
// geom names is sorted.
static std::vector<GeomPair> colliding_pairs(const mjModel* model,
                                             const mjData* data) {
  std::vector<GeomPair> result;
  for (int i = 0; i < data->ncon; i++) {
    std::string geom1 = mj_id2name(model, mjOBJ_GEOM, data->contact[i].geom[0]);
    std::string geom2 = mj_id2name(model, mjOBJ_GEOM, data->contact[i].geom[1]);
    result.push_back(GeomPair(std::min(geom1, geom2), std::max(geom1, geom2)));
  }
  std::sort(result.begin(), result.end());
  return result;
}

TEST_F(MjCollisionTest, AllCollisions) {
  static const char* const kModelFilePath = "engine/testdata/collisions.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // mjCOL_ALL is the default
  mj_fwdPosition(model, data);
  EXPECT_THAT(colliding_pairs(model, data),
              ElementsAre(GeomPair("box", "sphere_collides"),
                          GeomPair("box", "sphere_predefined")));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjCollisionTest, EmptyModel) {
  char error[1024];
  MjModelPtr model = LoadModelFromString("<mujoco/>", error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  mj_fwdPosition(model.get(), data.get());
  EXPECT_THAT(colliding_pairs(model.get(), data.get()), IsEmpty());
}

TEST_F(MjCollisionTest, ZeroedHessian) {
  static const char* const kModelFilePath = "engine/testdata/collisions.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  mj_fwdPosition(model, data);
  for (int i = 0; i < data->ncon; i++) {
    for (int j = 0; j < 36; j++) {
      EXPECT_FALSE(isnan(data->contact[i].H[j]))
          << "NaN in contact[" << i << "].H[" << j << "]";
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjCollisionTest, ContactCount) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom type="plane" size="5 5 .01"/>
      </body>
      <body pos="0 0 0.9">
        <freejoint/>
        <geom type="sphere" size="1" pos="-1 -1 0"/>
        <geom type="sphere" size="1" pos="-1  1 0"/>
        <geom type="sphere" size="1" pos=" 1 -1 0"/>
        <geom type="sphere" size="1" pos=" 1  1 0"/>
        <geom type="sphere" size="1" pos="-2 -2 0"/>
        <geom type="sphere" size="1" pos="-2  2 0"/>
        <geom type="sphere" size="1" pos=" 2 -2 0"/>
        <geom type="sphere" size="1" pos=" 2  2 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m.get(), d.get());

  // there are 8 spheres, all touching the floor
  EXPECT_EQ(d->ncon, 8);
}

TEST_F(MjCollisionTest, InGapContactsMultiGeomBody) {
  // In-gap contacts survive mid-phase BVH pruning. Both bodies
  // have a "far" geom that pulls the BVH root box away, so detecting the A-B
  // pair requires the descent filter to account for gap (body_margin).
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 1">
        <freejoint/>
        <geom name="A" type="box" size=".005 .01 .3" pos="-.015 0 0" gap=".004"/>
        <geom name="far1" type="box" size=".005 .01 .3" pos="-.2 0 0"/>
      </body>
      <body pos="0 0 1">
        <freejoint/>
        <geom name="B" type="box" size=".005 .01 .3" pos="-.0022 0 0" gap=".004"/>
        <geom name="far2" type="box" size=".005 .01 .3" pos=".1 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m.get(), d.get());

  // geoms A and B are separated by 2.8mm, within the 8mm combined gap:
  // inactive contacts are expected
  EXPECT_GT(d->ncon, 0);
  for (int i = 0; i < d->ncon; i++) {
    EXPECT_GT(d->contact[i].dist, 0);
    EXPECT_LT(d->contact[i].dist, 0.008);
    EXPECT_EQ(d->contact[i].efc_address, -1);
  }
}

TEST_F(MjCollisionTest, FilterParent) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body pos="0 0 0">
        <freejoint/>
        <geom name="colliding1" size="1" pos="0 0 100"/>
        <body>
          <geom size="1"/>
          <body>
            <joint axis="1 0 0"/>
            <geom size="1" pos="0 0 50"/>
            <body>
              <geom name="colliding2" size="1" pos="0 0 99.5"/>
            </body>
          </body>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_fwdPosition(m.get(), d.get());

  // there should be zero contacts, because colliding1 and colliding2 are in
  // bodies that have a parent-child relationship, through welds
  EXPECT_EQ(d->ncon, 0);

  // when this filtering is disabled, the geoms should collide
  m->opt.disableflags |= mjDSBL_FILTERPARENT;
  mj_fwdPosition(m.get(), d.get());

  EXPECT_THAT(colliding_pairs(m.get(), d.get()),
              ElementsAre(GeomPair("colliding1", "colliding2")));
}

TEST_F(MjCollisionTest, FilterParentDoesntAffectWorldBody) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="colliding1" size="1" pos="0 0 100"/>
      <body pos="0 0 0">
        <joint axis="1 0 0"/>
        <geom name="colliding2" size="1" pos="0 0 99.5"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_fwdPosition(m.get(), d.get());

  // even though colliding1 and colliding2 are have a parent-child relationship,
  // they collide because colliding1 is in <worldbody>
  EXPECT_THAT(colliding_pairs(m.get(), d.get()),
              ElementsAre(GeomPair("colliding1", "colliding2")));
}

TEST_F(MjCollisionTest, FilterStaticFlex) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="1 1 .1"/>
      <body name="static">
        <geom name="static" size=".1"/>
      </body>
      <body name="rigid">
        <flexcomp name="rigid" type="grid" count="3 3 1" spacing=".1 .1 .1" dim="2" rigid="true"/>
      </body>
      <body name="mocap" mocap="true" pos=".05 .05 .001">
        <flexcomp name="mocap" type="grid" count="3 3 1" spacing=".1 .1 .1" dim="2" rigid="true"/>
      </body>
      <body name="ball" pos="0 0 .09">
        <freejoint/>
        <geom name="ball" size=".1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m.get(), d.get());

  // the floor, the static geom and the two flexes overlap but have no dofs:
  // they collide only with the ball
  int ball = mj_name2id(m.get(), mjOBJ_GEOM, "ball");
  int nflexcon[2] = {0, 0};
  for (int i = 0; i < d->ncon; i++) {
    const mjContact& con = d->contact[i];
    EXPECT_TRUE(con.geom[0] == ball || con.geom[1] == ball);
    if (con.flex[1] >= 0) {
      nflexcon[con.flex[1]]++;
    }
  }
  EXPECT_GT(nflexcon[0], 0);
  EXPECT_GT(nflexcon[1], 0);
}

TEST_F(MjCollisionTest, FilterStaticFlexSelfCollision) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="a" pos="-1 0 0"/>
      <body name="b" pos="1 0 0"/>
      <body name="c" pos="0 -1 .01"/>
      <body name="d" pos="0 1 .01"/>
      <body pos="0 0 1">
        <freejoint/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <deformable>
      <flex name="rope" dim="1" radius=".01" body="a b c d" element="0 1 2 3">
        <edge stiffness="1"/>
      </flex>
    </deformable>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m.get(), d.get());

  // the two elements of the flex cross, but its vertices are in static bodies
  EXPECT_EQ(d->ncon, 0);
}

TEST_F(MjCollisionTest, TestOBB) {
  mjtNum bvh1[6] = {-1, -1, -1, 1, 1, 1};
  mjtNum bvh2[6] = {-1, -1, -1, 1, 1, 1};
  mjtNum pos1[3] = {0, 0, 0};
  mjtNum mat1[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  mjtNum pos2[3] = {1.71, 1.71, 0};  // just a little more than 1+sqrt(2)/2
  mjtNum mat2[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  EXPECT_THAT(
      mj_collideOBB(bvh1, bvh2, pos1, mat1, pos2, mat2, 0, NULL, NULL, 0),
      true);

  // rotate by 45 degrees
  mat2[0] = 1. / mju_sqrt(2.);
  mat2[1] = -1. / mju_sqrt(2.);
  mat2[3] = 1. / mju_sqrt(2.);
  mat2[4] = 1. / mju_sqrt(2.);

  EXPECT_THAT(
      mj_collideOBB(bvh1, bvh2, pos1, mat1, pos2, mat2, 0, NULL, NULL, 0),
      false);
}

TEST_F(MjCollisionTest, PlaneInBody) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom pos="0 0 0" type="plane" size="1 1 .01"/>
      </body>
      <body pos="0 0 .0499">
        <joint type="slide" axis="0 0 1"/>
        <geom size=".05"/>
      </body>
    </worldbody>
    </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());
  mj_step(m.get(), d.get());
}

TEST_F(MjCollisionTest, PinchingSucceeds) {
  constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.002" gravity="0 0 -9.81"/>
    <worldbody>
      <geom name="floor" type="plane" size="0 0 1"/>

      <body name="gripper" pos="0 0 0.5">
        <joint name="lift" type="slide" axis="0 0 1" damping="50"/>
        <geom type="box" size="0.2 0.05 0.02" rgba="0.5 0.5 0.5 1"/> <!-- base -->

        <body name="left_finger" pos="-0.1 0 -0.1">
          <joint name="left_slide" type="slide" axis="1 0 0" damping="10"/>
          <geom type="box" size="0.02 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
        </body>

        <body name="right_finger" pos="0.1 0 -0.1">
          <joint name="right_slide" type="slide" axis="-1 0 0" damping="10"/>
          <geom type="box" size="0.02 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
        </body>
      </body>

      <flexcomp name="cloth" type="grid" dim="2" count="9 9 1" spacing="0.05 0.05 0.05"
                pos="0 0 0.1" radius="0.01">
        <edge equality="true"/>
      </flexcomp>
    </worldbody>

    <equality>
      <joint joint1="right_slide" joint2="left_slide"/>
    </equality>

    <tendon>
      <fixed name="grasp">
        <joint joint="right_slide" coef="1"/>
        <joint joint="left_slide" coef="1"/>
      </fixed>
    </tendon>

    <actuator>
      <position name="lift" joint="lift" kp="600" dampratio="1" ctrlrange="-1 1"/>
      <position name="grasp" tendon="grasp" kp="200" dampratio="1" ctrlrange="0 1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  int lift_id = mj_name2id(m.get(), mjOBJ_ACTUATOR, "lift");
  int grasp_id = mj_name2id(m.get(), mjOBJ_ACTUATOR, "grasp");

  // Phase 1: Lower gripper.
  // The gripper base starts at z=0.5. The finger has length 0.2 (size 0.1),
  // extending from z=0.4 to z=0.2 relative to base (center at -0.1).
  // The cloth is at z=0.1. We need to lower the gripper so the fingertips
  // reach the cloth. A lift value of -0.35 places the fingertips near z=0.05.

  for (int i = 0; i < 500; ++i) {
    d->ctrl[lift_id] = -0.35;  // Lower
    d->ctrl[grasp_id] = 0;     // Open
    mj_step(m.get(), d.get());
  }

  // Phase 2: Pinch
  for (int i = 0; i < 100; ++i) {
    d->ctrl[lift_id] = -0.35;  // Hold height
    d->ctrl[grasp_id] = 0.8;   // Close (max 1)
    mj_step(m.get(), d.get());
  }

  // Phase 3: Lift
  for (int i = 0; i < 1000; ++i) {
    d->ctrl[lift_id] = 0.5;   // Lift up
    d->ctrl[grasp_id] = 0.8;  // Keep closed
    mj_step(m.get(), d.get());
  }

  // Check if cloth is lifted
  // flex verts are in d->flexvert_xpos
  // original z is ~0.1 (falling to floor ~0.0)
  // gripper lifted to > 0.5 probably

  // Find average Z of cloth
  double avg_z = 0;
  int nvert = m->flex_vertnum[0];
  for (int i = 0; i < nvert; ++i) {
    avg_z += d->flexvert_xpos[3 * i + 2];
  }
  avg_z /= nvert;

  // If lifted, avg_z should be significantly > 0.1
  // If failed (slipped), avg_z should be near 0 (floor)

  // Specialized primitives (mjraw_BoxTriangle, mjraw_CapsuleTriangle) should
  // enable stable pinching, so we expect the cloth to be lifted.
  EXPECT_GT(avg_z, 0.2) << "Cloth slipped out of gripper!";
}

TEST_F(MjCollisionTest, MarginSumming) {
  // Two spheres with size 0.1, placed 0.21 apart (distance of 0.01).
  // With margin summing, margin1 + margin2 = 0.00999 + 0.00999 = 0.01998 > 0.01
  // so a contact should be generated.
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom name="sphere1" type="sphere" size=".1" margin="0.00999"/>
        <joint type="slide" axis="1 0 0"/>
      </body>
      <body pos=".21 0 0">
        <geom name="sphere2" type="sphere" size=".1" margin="0.00999"/>
        <joint type="slide" axis="1 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_fwdPosition(m.get(), d.get());

  // With margin summing, we expect 1 contact
  EXPECT_EQ(d->ncon, 1);
}

TEST_F(MjCollisionTest, MaxContact) {
  constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag multiccd="enable"/>
    </option>
    <asset>
      <mesh name="smallbox"
        vertex="-1 -1 -1  1 -1 -1   1  1 -1
                 1  1  1  1 -1  1  -1  1 -1
                -1  1  1 -1 -1  1"/>
    </asset>
    <worldbody>
      <geom name="mesh" type="mesh" mesh="smallbox"/>
      <geom name="box" type="box" size="1 1 1"/>
      <geom name="plane" type="plane" size="1 1 1"/>
      <geom name="sphere" type="sphere" size="1"/>
      <geom name="capsule" type="capsule" size="1 1"/>
      <geom name="ellipsoid" type="ellipsoid" size="1 1 1"/>
      <geom name="cylinder" type="cylinder" size="1 1"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  int mesh = mj_name2id(m.get(), mjOBJ_GEOM, "mesh");
  int box = mj_name2id(m.get(), mjOBJ_GEOM, "box");
  int plane = mj_name2id(m.get(), mjOBJ_GEOM, "plane");
  int sphere = mj_name2id(m.get(), mjOBJ_GEOM, "sphere");
  int capsule = mj_name2id(m.get(), mjOBJ_GEOM, "capsule");
  int ellipsoid = mj_name2id(m.get(), mjOBJ_GEOM, "ellipsoid");
  int cylinder = mj_name2id(m.get(), mjOBJ_GEOM, "cylinder");

  EXPECT_EQ(mj_maxContact(m.get(), mesh, box, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), mesh, plane, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), box, plane, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), mesh, mesh, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), box, box, -1), 8);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, capsule, -1), 2);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, box, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, plane, -1), 2);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, plane, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, sphere, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, capsule, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, box, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, mesh, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, plane, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), sphere, cylinder, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, ellipsoid, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, box, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, mesh, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, plane, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, cylinder, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), ellipsoid, capsule, -1), 1);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, cylinder, -1), 2);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, mesh, -1), 2);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, cylinder, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, box, -1), 4);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, mesh, -1), 4);
}

// flex lying on a plane and on 16 spheres of the world body
std::string FlexOnGeoms(const std::string& flex) {
  return R"(
  <mujoco>
    <worldbody>
      <geom name="plane" type="plane" size="0 0 1" pos="0 0 .05"/>
      <replicate count="4" offset=".2 0 0">
        <replicate count="4" offset="0 .2 0">
          <geom size=".05" pos="-.3 -.3 0"/>
        </replicate>
      </replicate>
      )" +
         flex + R"(
    </worldbody>
  </mujoco>
  )";
}

// contacts with the plane, all contacts, maximum contacts with one sphere
struct FlexContacts {
  int nplane = 0;
  int ncon = 0;
  int maxsphere = 0;
};

FlexContacts CountFlexContacts(const mjModel* m, mjData* d) {
  mj_fwdPosition(m, d);
  int plane = mj_name2id(m, mjOBJ_GEOM, "plane");
  std::vector<int> ngeom(m->ngeom, 0);
  FlexContacts count;
  count.ncon = d->ncon;
  for (int i = 0; i < d->ncon; i++) {
    int g = d->contact[i].geom[0] >= 0 ? d->contact[i].geom[0]
                                       : d->contact[i].geom[1];
    ngeom[g]++;
  }
  for (int g = 0; g < m->ngeom; g++) {
    if (g == plane) {
      count.nplane = ngeom[g];
    } else if (m->geom_type[g] == mjGEOM_SPHERE) {
      count.maxsphere = std::max(count.maxsphere, ngeom[g]);
    }
  }
  return count;
}

TEST_F(MjCollisionTest, VertexFlexContactsAreNotLimited) {
  char error[1024];
  MjModelPtr m = LoadModelFromString(FlexOnGeoms(R"(
      <flexcomp name="cloth" type="grid" count="15 15 1" spacing=".05 .05 .05" dim="2"
                pos="0 0 .053" radius=".005">
        <edge equality="true"/>
        <contact selfcollide="none"/>
      </flexcomp>)"),
                                     error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  ASSERT_EQ(m->flex_rigid[0], 0);
  ASSERT_EQ(m->flex_interp[0], 0);

  MjDataPtr d_bvh = MakeData(m);
  FlexContacts bvh = CountFlexContacts(m.get(), d_bvh.get());

  // every vertex touches the plane, all contacts are kept
  EXPECT_EQ(bvh.nplane, m->flex_vertnum[0]);
  EXPECT_GT(bvh.ncon - bvh.nplane, 0);

  // same contacts without midphase
  m->opt.disableflags |= mjDSBL_MIDPHASE;
  MjDataPtr d_all = MakeData(m);
  FlexContacts all = CountFlexContacts(m.get(), d_all.get());
  EXPECT_EQ(all.nplane, bvh.nplane);
  EXPECT_EQ(all.ncon, bvh.ncon);
}

TEST_F(MjCollisionTest, ReducibleFlexContactLimitIsPerGeom) {
  // rigid cloth in a free body, trilinear soft box with its bottom layer on the
  // geoms
  const char* rigid = R"(
      <body name="cloth" pos="0 0 .053">
        <freejoint/>
        <inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/>
        <flexcomp name="cloth" type="grid" count="15 15 1" spacing=".05 .05 .05" dim="2"
                  radius=".005" rigid="true">
          <contact selfcollide="none"/>
        </flexcomp>
      </body>)";
  const char* trilinear = R"(
      <flexcomp name="box" type="grid" count="15 15 2" spacing=".05 .05 .05" dim="3"
                pos="0 0 .078" radius=".005" mass="1" dof="trilinear">
        <contact selfcollide="none"/>
      </flexcomp>)";
  for (const char* attrib : {rigid, trilinear}) {
    char error[1024];
    MjModelPtr m =
        LoadModelFromString(FlexOnGeoms(attrib), error, sizeof(error));
    ASSERT_THAT(m.get(), NotNull()) << attrib << ": " << error;
    ASSERT_TRUE(m->flex_rigid[0] || m->flex_interp[0]) << attrib;

    MjDataPtr d_bvh = MakeData(m);
    FlexContacts bvh = CountFlexContacts(m.get(), d_bvh.get());

    // every vertex touches the plane, mjMAXCONPAIR contacts are kept
    EXPECT_EQ(bvh.nplane, mjMAXCONPAIR) << attrib;

    // the spheres have more contacts, each sphere at most mjMAXCONPAIR
    EXPECT_GT(bvh.ncon - bvh.nplane, mjMAXCONPAIR) << attrib;
    EXPECT_LE(bvh.maxsphere, mjMAXCONPAIR) << attrib;

    // same contacts without midphase
    m->opt.disableflags |= mjDSBL_MIDPHASE;
    MjDataPtr d_all = MakeData(m);
    FlexContacts all = CountFlexContacts(m.get(), d_all.get());
    EXPECT_EQ(all.nplane, bvh.nplane) << attrib;
    EXPECT_EQ(all.ncon, bvh.ncon) << attrib;
  }
}

TEST_F(MjCollisionTest, Flex3DActiveLayersMidphaseDisabled) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="box" type="box" size="0.15 0.15 0.05" pos="0 0 0"/>
      <flexcomp name="vol1" type="grid" dim="3" count="4 4 4"
                spacing="0.06 0.06 0.06" pos="0 0 0.09" radius="0.005" mass="1">
        <contact selfcollide="none" activelayers="1"/>
        <edge equality="true"/>
      </flexcomp>
      <flexcomp name="vol2" type="grid" dim="3" count="4 4 4"
                spacing="0.06 0.06 0.06" pos="0 0 0.22" radius="0.005" mass="1">
        <contact selfcollide="none" activelayers="1"/>
        <edge equality="true"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;

  MjDataPtr d_bvh = MakeData(m);
  mj_forward(m.get(), d_bvh.get());
  EXPECT_GT(d_bvh->ncon, 0);

  m->opt.disableflags |= mjDSBL_MIDPHASE;
  MjDataPtr d_all = MakeData(m);
  mj_forward(m.get(), d_all.get());
  EXPECT_EQ(d_all->ncon, d_bvh->ncon);

  for (int i = 0; i < d_all->ncon; ++i) {
    for (int k = 0; k < 2; ++k) {
      int f = d_all->contact[i].flex[k];
      int e = d_all->contact[i].elem[k];
      if (f >= 0 && e >= 0) {
        int layer = m->flex_elemlayer[m->flex_elemadr[f] + e];
        EXPECT_LT(layer, m->flex_activelayers[f]);
      }
    }
  }
}

TEST_F(MjCollisionTest, ParallelCapsuleCapsule) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body euler="30 45 60">
        <geom name="a" type="capsule" size="0.01 2.0" pos="0 0.005 0"/>
      </body>
      <body euler="30 0 0">
        <body euler="0 45 60">
          <geom name="b" type="capsule" size="0.01 1.5" pos="0 -0.005 0.3"/>
          <geom name="c" type="capsule" size="0.01 1.5" pos="0 -0.095 0.3"/>
          <geom name="d" type="capsule" size="0.01 1.5" pos="0 -0.095 0.3" euler="0 0.1 0"/>
          <geom name="e" type="capsule" size="0.01 1.5" pos="0 -0.015 0.3" euler="0 0.01 0"/>
        </body>
      </body>
    </worldbody>
    <contact>
      <pair geom1="a" geom2="b"/>
    </contact>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  mj_forward(m.get(), d.get());

  // Penetrating rotated parallel capsules (a, b) should generate 2 contacts.
  ASSERT_EQ(d->ncon, 2);
  EXPECT_THAT(d->contact[0].dist, MjNear(-0.01, 1e-12, 1e-5));
  EXPECT_THAT(d->contact[1].dist, MjNear(-0.01, 1e-12, 1e-5));

  // Separated rotated parallel capsules (a, c) have surface distance 0.08.
  int a = mj_name2id(m.get(), mjOBJ_GEOM, "a");
  int c = mj_name2id(m.get(), mjOBJ_GEOM, "c");
  EXPECT_THAT(mj_geomDistance(m.get(), d.get(), a, c, 0.2, nullptr),
              MjNear(0.08, 1e-12, 1e-5));

  // Near-parallel (0.1 deg) capsules (a, d): surface distance 0.08, closest
  // point on d is at its center cross-section (distance 0.01 from d's xpos).
  int d_id = mj_name2id(m.get(), mjOBJ_GEOM, "d");
  mjtNum fromto[6];
  EXPECT_THAT(mj_geomDistance(m.get(), d.get(), a, d_id, 0.2, fromto),
              MjNear(0.08, 1e-12, 1e-5));
  EXPECT_THAT(mju_dist3(fromto + 3, d->geom_xpos + 3 * d_id),
              MjNear(0.01, 1e-12, 1e-2));

  // Near-parallel (0.01 deg) capsules (a, e): surface distance 0.0, closest
  // point on e is at its center cross-section (distance 0.01 from e's xpos).
  int e_id = mj_name2id(m.get(), mjOBJ_GEOM, "e");
  EXPECT_THAT(mj_geomDistance(m.get(), d.get(), a, e_id, 0.2, fromto),
              MjNear(0.0, 1e-12, 1e-5));
  EXPECT_THAT(mju_dist3(fromto + 3, d->geom_xpos + 3 * e_id),
              MjNear(0.01, 1e-12, 1e-1));
}

}  // namespace
}  // namespace mujoco
