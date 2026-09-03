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
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
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
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
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

TEST_F(MjCollisionTest, FlexFilterCompactsSelectedSet) {
  // A flat flex grid with 8x8 = 64 vertices sits within contact range of a
  // plane, so the plane-flex collision emits more than mjMAXCONPAIR candidate
  // contacts and filterFlexContacts has to trim them down. The retained prefix
  // must be exactly the farthest-point sample of the candidates, in selection
  // order, with no candidate dropped or duplicated.
  //
  // Midphase is disabled so the flex-plane pair takes the direct narrowphase
  // path, which leaves the filtered contacts in selection order instead of
  // re-sorting them by contact id.
  constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag midphase="disable"/>
    </option>
    <worldbody>
      <geom name="floor" type="plane" size="5 5 0.1"/>
      <flexcomp name="cloth" type="grid" dim="2" count="8 8 1"
                spacing="0.08 0.08 0.08" pos="0 0 0.009" radius="0.01">
        <edge equality="true"/>
        <contact internal="false" selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  int plane = mj_name2id(m.get(), mjOBJ_GEOM, "floor");
  int flex = mj_name2id(m.get(), mjOBJ_FLEX, "cloth");
  ASSERT_GE(plane, 0);
  ASSERT_GE(flex, 0);

  const int nvert = m->flex_vertnum[flex];
  ASSERT_GT(nvert, mjMAXCONPAIR);

  mj_fwdPosition(m.get(), d.get());

  // Reconstruct the candidate contacts the way mj_collidePlaneFlex does: one
  // contact per vertex within range of the plane. The contact point and signed
  // distance use the same expressions as the engine, so the reference below
  // operates on the same values the filter sees.
  const mjtNum* xmat = d->geom_xmat + 9*plane;
  const mjtNum nrm[3] = {xmat[2], xmat[5], xmat[8]};
  const mjtNum* xpos = d->geom_xpos + 3*plane;
  const mjtNum margin = m->geom_margin[plane] + m->flex_margin[flex];
  const mjtNum gap = m->geom_gap[plane] + m->flex_gap[flex];
  const mjtNum radius = m->flex_radius[flex];
  const int adr = m->flex_vertadr[flex];

  std::vector<int> cand_vert;
  std::vector<mjtNum> cand_pos;
  std::vector<mjtNum> cand_dist;
  for (int i = 0; i < nvert; i++) {
    const mjtNum* v = d->flexvert_xpos + 3*(adr + i);
    mjtNum dif[3];
    mju_sub3(dif, v, xpos);
    mjtNum dist = mju_dot3(dif, nrm);
    if (dist > margin + gap + radius) {
      continue;
    }
    cand_vert.push_back(i);
    mjtNum pos[3];
    mju_addScl3(pos, v, nrm, -((dist - radius)*0.5 + radius));
    cand_pos.insert(cand_pos.end(), pos, pos + 3);
    cand_dist.push_back(dist - radius);
  }
  ASSERT_EQ(cand_vert.size(), static_cast<size_t>(nvert));  // all in range
  ASSERT_GT(cand_vert.size(), static_cast<size_t>(mjMAXCONPAIR));

  // Reference farthest-point selection over the candidate contacts.
  const int ncand = static_cast<int>(cand_vert.size());
  std::vector<int> selected(ncand, 0);
  std::vector<mjtNum> min_dist(ncand, mjMAXVAL);
  std::vector<int> picks;

  // start with the deepest-penetrating contact (lowest index wins ties)
  int best = 0;
  mjtNum bestpen = -cand_dist[0];
  for (int i = 1; i < ncand; i++) {
    if (-cand_dist[i] > bestpen) {
      bestpen = -cand_dist[i];
      best = i;
    }
  }

  while (static_cast<int>(picks.size()) < mjMAXCONPAIR && best >= 0) {
    picks.push_back(cand_vert[best]);
    selected[best] = 1;
    const mjtNum* bestpos = &cand_pos[3*best];

    int nextbest = -1;
    mjtNum nextbestdist = -1;
    for (int i = 0; i < ncand; i++) {
      if (selected[i]) continue;
      mjtNum dx = cand_pos[3*i] - bestpos[0];
      mjtNum dy = cand_pos[3*i+1] - bestpos[1];
      mjtNum dz = cand_pos[3*i+2] - bestpos[2];
      mjtNum d2 = dx*dx + dy*dy + dz*dz;
      if (d2 < min_dist[i]) min_dist[i] = d2;
      if (min_dist[i] > nextbestdist) {
        nextbestdist = min_dist[i];
        nextbest = i;
      }
    }
    best = nextbest;
  }
  ASSERT_EQ(picks.size(), static_cast<size_t>(mjMAXCONPAIR));

  // The retained prefix must be the selected contacts in selection order,
  // including the final (mjMAXCONPAIR-th) selected contact.
  ASSERT_EQ(d->ncon, mjMAXCONPAIR);
  for (int k = 0; k < mjMAXCONPAIR; k++) {
    EXPECT_EQ(d->contact[k].flex[1], flex) << "contact " << k;
    EXPECT_EQ(d->contact[k].geom[0], plane) << "contact " << k;
    EXPECT_EQ(d->contact[k].vert[1], picks[k]) << "contact " << k;
  }
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
  EXPECT_EQ(mj_maxContact(m.get(), mesh, plane, -1), 3);
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
  EXPECT_EQ(mj_maxContact(m.get(), capsule, cylinder, -1), 5);
  EXPECT_EQ(mj_maxContact(m.get(), capsule, mesh, -1), 5);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, cylinder, -1), 5);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, box, -1), 5);
  EXPECT_EQ(mj_maxContact(m.get(), cylinder, mesh, -1), 5);
}

}  // namespace
}  // namespace mujoco
