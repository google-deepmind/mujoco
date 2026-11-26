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

// Tests for engine/engine_core_constraint.c.

#include "src/engine/engine_core_constraint.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_util.h"
#include "src/engine/engine_support.h"
#include "src/engine/engine_util_misc.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using ::testing::Pointwise;

using CoreConstraintTest = MujocoTest;

// test formulas for penetration at rest
TEST_F(CoreConstraintTest, RestPenetration) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="plane" size="1 1 1"/>
      <body pos="0 0 .2">
        <joint type="slide" axis="0 0 1"/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), testing::NotNull()) << error;
  mjtNum gravity = -model->opt.gravity[2];
  mjtNum damping_ratio = 0.8;
  MjDataPtr data = MakeData(model);

  for (const mjtNum reference : {-100.0, -10.0, 0.1, 0.01}) {
    for (const mjtNum impedance : {0.3, 0.9, 0.99}) {
      // set solimp
      for (int i = 0; i < model->ngeom; i++) {
        model->geom_solimp[i * mjNIMP + 0] = impedance;
        model->geom_solimp[i * mjNIMP + 1] = impedance;
      }

      // set solref
      for (int i = 0; i < model->ngeom; i++) {
        model->geom_solref[i * mjNREF + 0] = reference;
        model->geom_solref[i * mjNREF + 1] =
            reference < 0 ? -10 : damping_ratio;
      }

      // simulate for 50 seconds
      mj_resetData(model.get(), data.get());
      while (data->time < 50) {
        mjtNum time = data->time;
        mj_step(model.get(), data.get());
        ASSERT_GT(data->time, time) << "Divergence detected";
      }

      mjtNum depth = -data->contact[0].dist;
      mjtNum expected_depth;
      if (reference < 0) {
        expected_depth = gravity * (1 - impedance) / -reference;
      } else {
        mjtNum tc_dr = reference * damping_ratio;
        expected_depth = gravity * (1 - impedance) * tc_dr * tc_dr;
      }

      EXPECT_THAT(depth, MjNear(expected_depth, 1e-10, 1e-3));
    }
  }
}

// surfacevel: conveyor belt drags a box to belt speed
TEST_F(CoreConstraintTest, SurfaceVelocityConveyor) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="belt" type="plane" size="5 5 .1" surfacevel=".5 0 0  0 0 0"/>
      <body pos="0 0 .1">
        <freejoint/>
        <geom type="box" size=".1 .1 .1" mass="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), testing::NotNull()) << error;
  MjDataPtr data = MakeData(model);

  for (const mjtCone cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
    model->opt.cone = cone;
    mj_resetData(model.get(), data.get());
    while (data->time < 2) {
      mj_step(model.get(), data.get());
    }

    // box is transported at belt speed, no lateral or angular motion
    EXPECT_NEAR(data->qvel[0], 0.5, 1e-3);
    EXPECT_NEAR(data->qvel[1], 0.0, 1e-3);
    EXPECT_NEAR(data->qvel[5], 0.0, 1e-3);
  }
}

// surfacevel: turntable spins a centered box via the torsional friction row
TEST_F(CoreConstraintTest, SurfaceVelocityTurntable) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="table" type="plane" size="5 5 .1" condim="6"
            surfacevel="0 0 0  0 0 1"/>
      <body pos="0 0 .1">
        <freejoint/>
        <geom type="box" size=".1 .1 .1" mass="1" condim="6"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), testing::NotNull()) << error;
  MjDataPtr data = MakeData(model);

  for (const mjtCone cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
    model->opt.cone = cone;
    mj_resetData(model.get(), data.get());
    while (data->time < 4) {
      mj_step(model.get(), data.get());
    }

    // box spins up to the table's angular velocity, stays in place
    EXPECT_NEAR(data->qvel[5], 1.0, 1e-3);
    EXPECT_NEAR(data->qvel[0], 0.0, 1e-3);
    EXPECT_NEAR(data->qvel[1], 0.0, 1e-3);
  }
}

// surfacevel: the normal component is projected out, normal-only is inert
TEST_F(CoreConstraintTest, SurfaceVelocityNormalProjected) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="plane" size="5 5 .1" surfacevel="0 0 1  0 0 0"/>
      <body pos="0 0 .1">
        <freejoint/>
        <geom type="box" size=".1 .1 .1" mass="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), testing::NotNull()) << error;
  MjDataPtr data = MakeData(model);

  while (data->time < 2) {
    mj_step(model.get(), data.get());
  }

  // box rests as if the floor were plain
  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(data->qvel[i], 0.0, 1e-6);
  }
}

// surfacevel: two facing belts launch a squeezed plank at belt speed
TEST_F(CoreConstraintTest, SurfaceVelocityFacingBelts) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="box" size=".05 .3 .5" pos="-.152 0 .5" surfacevel="0 0 1  0 0 0"/>
      <geom type="box" size=".05 .3 .5" pos=".152 0 .5" surfacevel="0 0 1  0 0 0"/>
      <body pos="0 0 .5">
        <freejoint/>
        <geom type="box" size=".103 .1 .1" mass=".2"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), testing::NotNull()) << error;
  MjDataPtr data = MakeData(model);

  mjtNum vmax = 0;
  while (data->time < 2) {
    mj_step(model.get(), data.get());
    vmax = mju_max(vmax, data->qvel[2]);
  }
  EXPECT_NEAR(vmax, 1.0, 5e-3);
}

// surfacevel: belt on a moving vehicle, cargo vel composes with body vel
TEST_F(CoreConstraintTest, SurfaceVelocityComposition) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="plane" size="10 10 .1" friction="0 0 0" condim="1" priority="1"/>
      <body name="vehicle" pos="0 0 .15">
        <freejoint/>
        <geom type="box" size=".5 .3 .04" mass="10"/>
        <geom type="box" size=".4 .25 .01" pos="0 0 .05" mass=".01"
              surfacevel=".3 0 0  0 0 0"/>
      </body>
      <body name="cargo" pos="0 0 .3">
        <freejoint/>
        <geom type="box" size=".08 .08 .08" mass=".1"/>
      </body>
    </worldbody>
    <keyframe>
      <key qvel=".2 0 0 0 0 0  0 0 0 0 0 0"/>
    </keyframe>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), testing::NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mj_resetDataKeyframe(model.get(), data.get(), 0);

  // measure while the cargo is still riding the belt (exits the far end later)
  while (data->time < 1) {
    mj_step(model.get(), data.get());
  }
  EXPECT_NEAR(data->qvel[6] - data->qvel[0], 0.3, 1e-3);
}

// surfacevel: interpreted in the authored geom frame, including for mesh geoms
// whose compiled frame absorbs the mesh's centering/principal-axes transform
TEST_F(CoreConstraintTest, SurfaceVelocityMeshFrame) {
  constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="ring" builtin="supertorus" params="32 .3 .2 1" scale="1.6 1.6 .8"/>
    </asset>
    <worldbody>
      <geom name="carousel" type="mesh" mesh="ring" pos="0 0 .24"
            surfacevel="0 0 0  0 0 .4"/>
      <body pos="1.6 0 .6">
        <freejoint/>
        <geom type="box" size=".1 .1 .1" mass="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), testing::NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());

  // the compiled angular surfacevel, rotated by the compiled geom orientation,
  // recovers the authored world-frame spin (0, 0, .4)
  int carousel = mj_name2id(model.get(), mjOBJ_GEOM, "carousel");
  mjtNum w_world[3];
  mju_mulMatVec3(w_world, data->geom_xmat + 9 * carousel,
                 model->geom_surfacevel + 6 * carousel + 3);
  EXPECT_NEAR(w_world[0], 0.0, MjTol(1e-10, 1e-6));
  EXPECT_NEAR(w_world[1], 0.0, MjTol(1e-10, 1e-6));
  EXPECT_NEAR(w_world[2], 0.4, MjTol(1e-10, 1e-6));

  // functionally: the box is dragged around the ring at speed omega * r
  while (data->time < 3) {
    mj_step(model.get(), data.get());
  }
  mjtNum x = data->qpos[0], y = data->qpos[1];
  mjtNum r = mju_sqrt(x * x + y * y);
  mjtNum speed =
      mju_sqrt(data->qvel[0] * data->qvel[0] + data->qvel[1] * data->qvel[1]);
  EXPECT_NEAR(speed, 0.4 * r, 5e-3);
}

// surfacevel: writing mjContact.surfacevel directly matches geom_surfacevel
TEST_F(CoreConstraintTest, SurfaceVelocityExternalContact) {
  constexpr char xml_geom[] = R"(
  <mujoco>
    <worldbody>
      <geom type="plane" size="5 5 .1" condim="4" surfacevel=".5 -.2 0  0 0 1"/>
      <body pos=".3 .4 .1">
        <freejoint/>
        <geom type="box" size=".1 .1 .1" mass="1" condim="4"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  constexpr char xml_plain[] = R"(
  <mujoco>
    <worldbody>
      <geom type="plane" size="5 5 .1" condim="4"/>
      <body pos=".3 .4 .1">
        <freejoint/>
        <geom type="box" size=".1 .1 .1" mass="1" condim="4"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model_a = LoadModelFromString(xml_geom, error, sizeof(error));
  ASSERT_THAT(model_a.get(), testing::NotNull()) << error;
  MjDataPtr data_a = MakeData(model_a);

  MjModelPtr model_b = LoadModelFromString(xml_plain, error, sizeof(error));
  ASSERT_THAT(model_b.get(), testing::NotNull()) << error;
  MjDataPtr data_b = MakeData(model_b);

  for (const mjtCone cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
    model_a->opt.cone = cone;
    model_b->opt.cone = cone;
    model_b->flg_surfacevel = 0;

    mj_resetData(model_a.get(), data_a.get());
    mj_forward(model_a.get(), data_a.get());
    ASSERT_EQ(data_a->ncon, 4);

    mj_resetData(model_b.get(), data_b.get());
    mj_fwdPosition(model_b.get(), data_b.get());
    ASSERT_EQ(data_b->ncon, data_a->ncon);

    // contacts on plain model have zero-initialized surfacevel
    for (int i = 0; i < data_b->ncon; i++) {
      EXPECT_EQ(data_b->contact[i].surfacevel[0], 0);
      EXPECT_EQ(data_b->contact[i].surfacevel[1], 0);
      EXPECT_EQ(data_b->contact[i].surfacevel[2], 0);
      mju_copy3(data_b->contact[i].surfacevel, data_a->contact[i].surfacevel);
    }

    // gated off when flg_surfacevel == 0
    mj_forwardSkip(model_b.get(), data_b.get(), mjSTAGE_POS, 0);
    EXPECT_NEAR(data_b->qacc[0], 0.0, MjTol(1e-12, 1e-6));
    EXPECT_NEAR(data_b->qacc[1], 0.0, MjTol(1e-12, 1e-6));
    EXPECT_NEAR(data_b->qacc[5], 0.0, MjTol(1e-12, 1e-6));

    // enabling flg_surfacevel matches geom_surfacevel exactly
    model_b->flg_surfacevel = 1;
    mj_forwardSkip(model_b.get(), data_b.get(), mjSTAGE_POS, 0);
    for (int i = 0; i < data_a->nefc; i++) {
      EXPECT_NEAR(data_b->efc_vel[i], data_a->efc_vel[i], MjTol(1e-12, 1e-6));
    }
    for (int i = 0; i < model_a->nv; i++) {
      EXPECT_NEAR(data_b->qacc[i], data_a->qacc[i], MjTol(1e-11, 1e-5));
    }
  }
}

static const char* const kDoflessContactPath =
    "engine/testdata/core_constraint/dofless_contact.xml";
static const char* const kDoflessTendonFrictionalPath =
    "engine/testdata/core_constraint/dofless_tendon_frictional.xml";
static const char* const kDoflessTendonLimitedPath =
    "engine/testdata/core_constraint/dofless_tendon_limited.xml";
static const char* const kDoflessTendonLimitedMarginPath =
    "engine/testdata/core_constraint/dofless_tendon_limitedmargin.xml";
static const char* const kDoflessWeldPath =
    "engine/testdata/core_constraint/dofless_weld.xml";
static const char* const kJointLimitedBilateralMarginPath =
    "engine/testdata/core_constraint/joint_limited_bilateral_margin.xml";
static const char* const kTendonLimitedBilateralMarginPath =
    "engine/testdata/core_constraint/tendon_limited_bilateral_margin.xml";

TEST_F(CoreConstraintTest, JacobianPreAllocate) {
  for (const char* local_path :
       {kDoflessContactPath, kDoflessTendonFrictionalPath,
        kDoflessTendonLimitedPath, kDoflessTendonLimitedMarginPath,
        kDoflessWeldPath, kJointLimitedBilateralMarginPath,
        kTendonLimitedBilateralMarginPath}) {
    const std::string xml_path = GetTestDataFilePath(local_path);

    // iterate through dense and sparse
    for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
      char error[1024];
      mjModel* model =
          mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
      ASSERT_THAT(model, NotNull()) << error;
      model->opt.jacobian = sparsity;
      mjData* data = mj_makeData(model);

      mj_step(model, data);

      mj_deleteData(data);
      mj_deleteModel(model);
    }
  }
}

TEST_F(CoreConstraintTest, DoflessContactsExcluded) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="2 2 .1"/>
      <body name="static">
        <geom name="static" size=".05" pos="-.13 -.13 0"/>
      </body>
      <body name="mocap" mocap="true">
        <geom name="mocap" size=".05" pos="-.13 -.13 .09"/>
      </body>
      <flexcomp name="cloth" type="grid" count="5 5 1" spacing=".1 .1 .1" dim="2" mass="1">
        <pin id="0 1 2 3 4 5 6 7 8 9"/>
        <edge equality="true"/>
      </flexcomp>
      <body name="moving" pos="1 0 0">
        <freejoint/>
        <body name="weld1">
          <geom name="weld1" size=".05" pos="-.13 -.13 0"/>
        </body>
        <body name="weld2">
          <geom name="weld2" size=".05" pos="-.13 -.13 .09"/>
        </body>
        <flexcomp name="cloth2" type="grid" count="5 5 1" spacing=".1 .1 .1" dim="2" mass="1">
          <pin id="0 1 2 3 4 5 6 7 8 9"/>
          <edge equality="true"/>
        </flexcomp>
      </body>
    </worldbody>
    <contact>
      <pair geom1="static" geom2="mocap"/>
      <pair geom1="weld1" geom2="weld2"/>
    </contact>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d = MakeData(m);
  ASSERT_THAT(d, NotNull());

  // contacts which affect no dofs are excluded, in dense and sparse
  for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
    m->opt.jacobian = sparsity;
    mj_forward(m.get(), d.get());

    int ngeom = 0, nvert = 0, nelem = 0;
    for (int i = 0; i < d->ncon; i++) {
      const mjContact& con = d->contact[i];
      if (con.exclude != 3) {
        continue;
      }
      EXPECT_EQ(con.efc_address, -1);
      if (con.geom[1] >= 0) {
        ngeom++;
      } else if (con.vert[1] >= 0) {
        nvert++;
      } else {
        nelem++;
      }
    }

    // the explicit pairs of (static, mocap) and (weld1, weld2) geoms
    EXPECT_EQ(ngeom, 2);

    // the world-pinned vertices on the floor
    EXPECT_EQ(nvert, 10);

    // the elements with only pinned vertices touching the static or welded geom
    EXPECT_EQ(nelem, 6);
  }
}

TEST_F(CoreConstraintTest, EqualityBodySite) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/equality_site_body_compare.xml");

  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // simulate, get diag(A)
  while (data->time < 0.1) {
    mjtNum time = data->time;
    mj_step(model, data);
    ASSERT_GT(data->time, time) << "Divergence detected";
  }
  int nefc_site = data->nefc;
  std::vector<mjtNum> dA = AsVector(data->efc_diagA, nefc_site);

  // reset
  mj_resetData(model, data);

  // turn site-defined equalities off, equivalent body-defined equalities on
  for (int e = 0; e < 4; e++) data->eq_active[e] = 1 - data->eq_active[e];

  // simulate again, get diag(A)
  while (data->time < 0.1) {
    mjtNum time = data->time;
    mj_step(model, data);
    ASSERT_GT(data->time, time) << "Divergence detected";
  }

  // compare
  EXPECT_EQ(nefc_site, data->nefc);
  EXPECT_THAT(AsVector(data->efc_diagA, data->nefc),
              Pointwise(MjNear(1e-12, 1e-4), dA));

  mj_deleteData(data);
  mj_deleteModel(model);
}

static const char* const kIlslandEfcPath =
    "engine/testdata/island/island_efc.xml";

// validate mj_constraintUpdate_impl
TEST_F(CoreConstraintTest, ConstraintUpdateImpl) {
  const std::string xml_path = GetTestDataFilePath(kIlslandEfcPath);
  char err[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), 0, err, 1024);
  ASSERT_THAT(model, NotNull()) << err;
  mjData* d1 = mj_makeData(model);
  mjData* d2 = mj_makeData(model);

  // iterate over sparsity and cone
  for (mjtJacobian sparsity : {mjJAC_SPARSE, mjJAC_DENSE}) {
    for (mjtCone cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
      model->opt.jacobian = sparsity;
      model->opt.cone = cone;

      // simulate for 0.2 seconds
      mj_resetData(model, d1);
      mj_resetData(model, d2);
      while (d1->time < 0.2) {
        mjtNum time1 = d1->time;
        mjtNum time2 = d2->time;
        mj_step(model, d1);
        ASSERT_GT(d1->time, time1) << "Divergence detected";
        mj_step(model, d2);
        ASSERT_GT(d2->time, time2) << "Divergence detected";
      }
      mj_forward(model, d1);
      mj_forward(model, d2);

      // get sizes
      int nefc = d1->nefc;
      int nv = model->nv;
      int nisland = d1->nisland;
      EXPECT_GT(nisland, 0);

      // get jar = J*a - aref
      mjtNum* jar = (mjtNum*)mju_malloc(sizeof(mjtNum) * nefc);
      mj_mulJacVec(model, d1, jar, d1->qacc);
      mju_subFrom(jar, d1->efc_aref, nefc);

      // constraint update for data1 given jar
      mjtNum cost1;
      mj_constraintUpdate(model, d1, jar, &cost1);

      // iterate over islands, check match
      mjtNum cost2 = 0;
      for (int island = 0; island < nisland; island++) {
        // clear outputs from data2
        for (int i = 0; i < nefc; i++) d2->efc_state[i] = -1;
        mju_zero(d2->efc_force, nefc);
        mju_zero(d2->qfrc_constraint, nv);

        // sizes and indices, in this island
        int efcnum = d2->island_nefc[island];

        // gather values into jari
        mjtNum* jari = (mjtNum*)mju_malloc(sizeof(mjtNum) * efcnum);
        int* map2efc = d2->map_iefc2efc + d2->island_iefcadr[island];
        mju_gather(jari, jar, map2efc, efcnum);

        // update constraints for this island
        mjtNum cost2i;
        int ne = d2->island_ne[island];
        int nf = d2->island_nf[island];
        int adr = d2->island_iefcadr[island];
        int* state = d2->iefc_state + adr;
        mjtNum* force = d2->iefc_force + adr;
        mj_constraintUpdate_impl(ne, nf, efcnum, d2->iefc_D + adr,
                                 d2->iefc_R + adr, d2->iefc_frictionloss + adr,
                                 jari, d2->iefc_type + adr, d2->iefc_id + adr,
                                 d2->contact, state, force, &cost2i);

        // compare nefc vectors
        for (int c = 0; c < efcnum; c++) {
          int i = map2efc[c];
          EXPECT_EQ(d2->efc_island[i], island);
          EXPECT_EQ(state[c], d1->efc_state[i]);
          EXPECT_THAT(force[c], MjNear(d1->efc_force[i], 1e-12, 1e-4));
        }

        // add island cost to total cost
        cost2 += cost2i;

        mju_free(jari);
      }

      // expect monolithic total cost
      EXPECT_THAT(cost1, MjNear(cost2, 1e-12, 1e-4));

      mju_free(jar);
    }
  }

  mj_deleteData(d2);
  mj_deleteData(d1);
  mj_deleteModel(model);
}

// the 4-wide batched dim=3 elliptic path must match the scalar path exactly
TEST_F(CoreConstraintTest, ConstraintUpdateEllipticBatchMatchesScalar) {
  // 9 contiguous dim=3 cones (two 4-wide batches + scalar tail), a dim=4 cone
  // that ends batching, and two more dim=3 cones handled by the scalar loop
  const std::vector<int> dims = {3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 3, 3};
  // per-cone (normal, tangent) jar: top, bottom, middle (N > 0 and N < 0),
  // and T == 0 with N of both signs
  const mjtNum kJar[6][2] = {{1, 0.1},  {-1, 0.05}, {0.1, 0.8},
                             {-0.1, 1}, {0.5, 0},   {-0.5, 0}};
  constexpr int kLead = 2;  // leading frictionless contact rows

  int ncon = dims.size();
  std::vector<mjContact> con(ncon);
  std::vector<int> adr(ncon);
  int nefc = kLead;
  for (int k = 0; k < ncon; k++) {
    con[k].dim = dims[k];
    con[k].mu = 0.5 + 0.1 * k;
    for (int j = 0; j < 5; j++) con[k].friction[j] = 1 + 0.2 * j + 0.05 * k;
    adr[k] = nefc;
    nefc += dims[k];
  }

  std::vector<mjtNum> D(nefc), R(nefc, 0), floss(nefc, 0), jar(nefc);
  std::vector<int> type(nefc), id(nefc, 0);
  for (int i = 0; i < kLead; i++) {
    type[i] = mjCNSTR_CONTACT_FRICTIONLESS;
    jar[i] = i ? 0.3 : -0.3;
    D[i] = 2 + i;
  }
  for (int k = 0; k < ncon; k++) {
    for (int j = 0; j < dims[k]; j++) {
      int i = adr[k] + j;
      type[i] = mjCNSTR_CONTACT_ELLIPTIC;
      id[i] = k;
      D[i] = 1 + 0.1 * i;
      jar[i] = j ? kJar[k % 6][1] * (j % 2 ? 1 : -0.7) : kJar[k % 6][0];
    }
  }

  // batched: all rows at once
  std::vector<int> state(nefc, -1);
  std::vector<mjtNum> force(nefc, -1);
  mjtNum cost = 0;
  mj_constraintUpdate_impl(0, 0, nefc, D.data(), R.data(), floss.data(),
                           jar.data(), type.data(), id.data(), con.data(),
                           state.data(), force.data(), &cost);

  // scalar reference: leading rows via contact == nullptr fast path,
  // then one cone per call
  std::vector<int> state_ref(nefc, -1);
  std::vector<mjtNum> force_ref(nefc, -1);
  mjtNum cost_ref = 0, c = 0;
  mj_constraintUpdate_impl(0, 0, kLead, D.data(), R.data(), floss.data(),
                           jar.data(), type.data(), id.data(), nullptr,
                           state_ref.data(), force_ref.data(), &c);
  cost_ref += c;
  for (int k = 0; k < ncon; k++) {
    int a = adr[k];
    mj_constraintUpdate_impl(0, 0, dims[k], D.data() + a, R.data() + a,
                             floss.data() + a, jar.data() + a, type.data() + a,
                             id.data() + a, con.data(), state_ref.data() + a,
                             force_ref.data() + a, &c);
    cost_ref += c;
  }

  EXPECT_EQ(state, state_ref);
  EXPECT_EQ(force, force_ref);
  EXPECT_NEAR(cost, cost_ref, MjTol(1e-12, 1e-5) * std::abs(cost_ref));

  // all three zones are exercised
  for (int s :
       {mjCNSTRSTATE_SATISFIED, mjCNSTRSTATE_QUADRATIC, mjCNSTRSTATE_CONE}) {
    EXPECT_NE(std::find(state.begin(), state.end(), s), state.end()) << s;
  }

  // cost == nullptr gives the same forces
  std::vector<mjtNum> force_nocost(nefc, -1);
  mj_constraintUpdate_impl(0, 0, nefc, D.data(), R.data(), floss.data(),
                           jar.data(), type.data(), id.data(), con.data(),
                           state.data(), force_nocost.data(), nullptr);
  EXPECT_EQ(force_nocost, force);

  // NaN parity: leading non-elliptic row and batched dim=3 cones propagate NaN
  // the same way as the scalar paths (including T == 0 with N == NaN)
  const mjtNum qnan = std::numeric_limits<mjtNum>::quiet_NaN();
  {
    const mjtNum d_lead = 2, r_lead = 0, fl_lead = 0, j_lead = qnan;
    const int t_lead = mjCNSTR_CONTACT_FRICTIONLESS, id_lead = 0;
    int s_batch = -1, s_ref = -1;
    mjtNum f_batch = 0, f_ref = 0, c_batch = 0, c_ref = 0;
    mj_constraintUpdate_impl(0, 0, 1, &d_lead, &r_lead, &fl_lead, &j_lead,
                             &t_lead, &id_lead, con.data(), &s_batch, &f_batch,
                             &c_batch);
    mj_constraintUpdate_impl(0, 0, 1, &d_lead, &r_lead, &fl_lead, &j_lead,
                             &t_lead, &id_lead, nullptr, &s_ref, &f_ref,
                             &c_ref);
    EXPECT_EQ(s_batch, mjCNSTRSTATE_QUADRATIC);
    EXPECT_EQ(s_batch, s_ref);
    EXPECT_TRUE(std::isnan(f_batch));
    EXPECT_TRUE(std::isnan(f_ref));
    EXPECT_TRUE(std::isnan(c_batch));
    EXPECT_TRUE(std::isnan(c_ref));
  }
  {
    constexpr int kN = 12;
    std::vector<mjtNum> jn = {qnan, 0,    0, qnan, 0.2, 0.3,
                              0.5,  qnan, 0, 1,    0.1, 0.1};
    std::vector<mjtNum> dn(kN, 1), rn(kN, 0), fn(kN, 0);
    std::vector<int> tn(kN, mjCNSTR_CONTACT_ELLIPTIC);
    std::vector<int> idn = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3};
    std::vector<int> sb(kN, -1), sr(kN, -1);
    std::vector<mjtNum> fb(kN, 0), fr(kN, 0);
    mjtNum cb = 0, cr = 0;
    mj_constraintUpdate_impl(0, 0, kN, dn.data(), rn.data(), fn.data(),
                             jn.data(), tn.data(), idn.data(), con.data(),
                             sb.data(), fb.data(), &cb);
    for (int k = 0; k < 4; k++) {
      int a = 3 * k;
      mj_constraintUpdate_impl(0, 0, 3, dn.data() + a, rn.data() + a,
                               fn.data() + a, jn.data() + a, tn.data() + a,
                               idn.data() + a, con.data(), sr.data() + a,
                               fr.data() + a, &cr);
      if (k < 3) {
        EXPECT_TRUE(std::isnan(cr)) << "cone " << k;
      }
    }
    EXPECT_EQ(sb, sr);
    EXPECT_TRUE(std::isnan(cb));
    for (int r = 0; r < kN; r++) {
      if (r < 9) {
        EXPECT_TRUE(std::isnan(fb[r])) << "r=" << r;
        EXPECT_TRUE(std::isnan(fr[r])) << "r=" << r;
      } else {
        EXPECT_EQ(fb[r], fr[r]) << "r=" << r;
      }
    }
  }
}

// exact scalar reference matching the unbatched elliptic loop (using mju_norm)
void ConstraintUpdateScalarRef(int ne, int nf, int nefc, const mjtNum* D,
                               const mjtNum* R, const mjtNum* floss,
                               const mjtNum* jar, const int* type,
                               const int* id, const mjContact* contact,
                               int* state, mjtNum* force, mjtNum* cost) {
  mjtNum s = 0;
  for (int i = 0; i < ne; i++) {
    force[i] = -D[i] * jar[i];
    if (cost) s += 0.5 * D[i] * jar[i] * jar[i];
    state[i] = mjCNSTRSTATE_QUADRATIC;
  }
  for (int i = ne; i < ne + nf; i++) {
    if (jar[i] <= -R[i] * floss[i]) {
      if (cost) s += -0.5 * R[i] * floss[i] * floss[i] - floss[i] * jar[i];
      force[i] = floss[i];
      state[i] = mjCNSTRSTATE_LINEARNEG;
    } else if (jar[i] >= R[i] * floss[i]) {
      if (cost) s += -0.5 * R[i] * floss[i] * floss[i] + floss[i] * jar[i];
      force[i] = -floss[i];
      state[i] = mjCNSTRSTATE_LINEARPOS;
    } else {
      force[i] = -D[i] * jar[i];
      if (cost) s += 0.5 * D[i] * jar[i] * jar[i];
      state[i] = mjCNSTRSTATE_QUADRATIC;
    }
  }
  for (int i = ne + nf; i < nefc; i++) {
    if (type[i] != mjCNSTR_CONTACT_ELLIPTIC) {
      if (jar[i] >= 0) {
        force[i] = 0;
        state[i] = mjCNSTRSTATE_SATISFIED;
      } else {
        force[i] = -D[i] * jar[i];
        if (cost) s += 0.5 * D[i] * jar[i] * jar[i];
        state[i] = mjCNSTRSTATE_QUADRATIC;
      }
    } else {
      const mjContact* con = contact + id[i];
      mjtNum mu = con->mu;
      const mjtNum* friction = con->friction;
      int dim = con->dim;
      mjtNum U[6];
      U[0] = jar[i] * mu;
      for (int j = 1; j < dim; j++) U[j] = jar[i + j] * friction[j - 1];
      mjtNum N = U[0];
      mjtNum T = mju_norm(U + 1, dim - 1);
      if (N >= mu * T || (T <= 0 && N >= 0)) {
        for (int j = 0; j < dim; j++) force[i + j] = 0;
        state[i] = mjCNSTRSTATE_SATISFIED;
      } else if (mu * N + T <= 0 || (T <= 0 && N < 0)) {
        for (int j = 0; j < dim; j++) force[i + j] = -D[i + j] * jar[i + j];
        if (cost) {
          for (int j = 0; j < dim; j++) {
            s += (mjtNum)0.5 * D[i + j] * jar[i + j] * jar[i + j];
          }
        }
        state[i] = mjCNSTRSTATE_QUADRATIC;
      } else {
        mjtNum Dm = D[i] / (mu * mu * (1 + mu * mu));
        mjtNum NmT = N - mu * T;
        if (cost) s += (mjtNum)0.5 * Dm * NmT * NmT;
        mjtNum Uhat[5];
        for (int j = 1; j < dim; j++) Uhat[j - 1] = U[j] / T;
        force[i] = -Dm * NmT * mu;
        for (int j = 1; j < dim; j++) {
          force[i + j] = -force[i] * Uhat[j - 1] * friction[j - 1];
        }
        state[i] = mjCNSTRSTATE_CONE;
      }
      for (int j = 1; j < dim; j++) state[i + j] = state[i];
      i += (dim - 1);
    }
  }
  if (cost) *cost = s;
}

TEST_F(CoreConstraintTest, ConstraintUpdateEllipticSimdMatchesScalar) {
  struct Block {
    int dim;
    int zone;
    int count;
  };
  const std::vector<Block> blocks = {
      {1, 3, 5}, {3, 0, 4}, {3, 1, 4}, {3, 2, 4}, {3, 3, 7},
      {1, 3, 1}, {4, 0, 4}, {4, 1, 4}, {4, 2, 4}, {4, 3, 7},
      {6, 0, 4}, {6, 1, 4}, {6, 2, 4}, {6, 3, 7},
  };

  // seed non-zero equality and friction rows so s != 0 entering every batch
  constexpr int kNe = 2;
  constexpr int kNf = 2;
  std::vector<mjContact> con;
  std::vector<int> type, id;
  std::vector<mjtNum> D, R, floss, jar;
  for (int i = 0; i < kNe; i++) {
    type.push_back(mjCNSTR_EQUALITY);
    id.push_back(i);
    D.push_back(1.3 + 0.2 * i);
    R.push_back(1.0 / D.back());
    floss.push_back(0);
    jar.push_back(0.7 - 1.1 * i);
  }
  for (int i = 0; i < kNf; i++) {
    type.push_back(mjCNSTR_FRICTION_DOF);
    id.push_back(i);
    D.push_back(2.0 + 0.3 * i);
    R.push_back(1.0 / D.back());
    floss.push_back(0.4);
    jar.push_back(i ? 0.1 : -0.9);
  }

  auto push_zone = [&](int dim, int zone, int k) {
    int base = jar.size();
    if (dim == 1) {
      type.push_back(mjCNSTR_CONTACT_FRICTIONLESS);
      id.push_back(0);
      D.push_back(1.2 + 0.1 * k);
      R.push_back(0);
      floss.push_back(0);
      jar.push_back((k % 2) ? 0.4 : -0.4);
      return;
    }
    int cid = con.size();
    mjContact c = {};
    c.dim = dim;
    c.mu = 0.55 + 0.03 * (cid % 7);
    for (int j = 0; j < 5; j++) {
      c.friction[j] = 0.9 + 0.15 * j + 0.02 * (cid % 5);
    }
    con.push_back(c);
    int z = (zone == 3) ? (k % 3) : zone;
    mjtNum jn = (z == 0) ? 1.2 : (z == 1 ? -1.5 : -0.1);
    mjtNum jt = (z == 0) ? 0.05 : (z == 1 ? 0.02 : 0.9);
    for (int j = 0; j < dim; j++) {
      type.push_back(mjCNSTR_CONTACT_ELLIPTIC);
      id.push_back(cid);
      D.push_back(1.0 + 0.07 * (base + j));
      R.push_back(0);
      floss.push_back(0);
      jar.push_back(j == 0 ? jn : jt * ((j % 2) ? 1.0 : -0.6));
    }
  };

  for (const Block& b : blocks) {
    for (int k = 0; k < b.count; k++) {
      push_zone(b.dim, b.zone, k);
    }
  }

  int nefc = jar.size();
  std::vector<int> state(nefc, -1), state_ref(nefc, -1);
  std::vector<mjtNum> force(nefc, -1), force_ref(nefc, -1);
  mjtNum cost = 0, cost_ref = 0;
  mj_constraintUpdate_impl(kNe, kNf, nefc, D.data(), R.data(), floss.data(),
                           jar.data(), type.data(), id.data(), con.data(),
                           state.data(), force.data(), &cost);
  ConstraintUpdateScalarRef(kNe, kNf, nefc, D.data(), R.data(), floss.data(),
                            jar.data(), type.data(), id.data(), con.data(),
                            state_ref.data(), force_ref.data(), &cost_ref);

  EXPECT_EQ(state, state_ref);
  EXPECT_EQ(force, force_ref);
  EXPECT_EQ(std::memcmp(force.data(), force_ref.data(), nefc * sizeof(mjtNum)),
            0);
  EXPECT_EQ(cost, cost_ref);
  EXPECT_EQ(std::memcmp(&cost, &cost_ref, sizeof(mjtNum)), 0);

  std::vector<int> state_nc(nefc, -1);
  std::vector<mjtNum> force_nc(nefc, -1);
  mj_constraintUpdate_impl(kNe, kNf, nefc, D.data(), R.data(), floss.data(),
                           jar.data(), type.data(), id.data(), con.data(),
                           state_nc.data(), force_nc.data(), nullptr);
  EXPECT_EQ(state_nc, state);
  EXPECT_EQ(std::memcmp(force_nc.data(), force.data(), nefc * sizeof(mjtNum)),
            0);
}

// Test flex strain constraint with pinned nodes attached to freejoint parent
TEST_F(CoreConstraintTest, BoxShellPinnedParentWithFreejoint) {
#ifdef mjUSESINGLE
  GTEST_SKIP() << "FD Jacobian mismatch due to float32 precision";
#endif
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="discrete" jacobian="dense" gravity="0 0 0"/>
  <worldbody>
    <geom type="plane" size="10 10 1" pos="0 0 -.1"/>
    <body>
      <joint type="free"/>
      <geom type="box" size="0.13 0.18 0.036"/>
      <body name="parent">
        <flexcomp name="test" type="box"
                  spacing=".1 .02 .1" radius="0.001"
                  pos="0 0 .2" dof="trilinear" xyaxes="0 1 0 0 0 1" mass="1" dim="3">
          <contact selfcollide="none"/>
          <edge equality="strain"/>
          <pin id="0 2 4 6"/>
        </flexcomp>
      </body>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);

  mj_resetData(m.get(), d.get());
  mj_forward(m.get(), d.get());

  // Check that we have constraints
  EXPECT_GT(d->nefc, 0) << "No constraints generated";
  EXPECT_GT(d->ne, 0) << "Expected some strain constraints";

  // Check qacc and forces at rest with gravity=0
  EXPECT_NEAR(d->qacc_smooth[6], 0, 1e-6) << "qacc_smooth should be 0 at rest";
  EXPECT_NEAR(d->qacc[6], 0, 1e-6) << "qacc should be 0 at rest";

  // Check initial constraint values (efc_pos)
  bool has_bad_constraint = false;
  for (int i = 0; i < d->nefc; i++) {
    if (d->efc_type[i] == mjCNSTR_EQUALITY) {
      if (mju_abs(d->efc_pos[i]) > 1.0) {
        has_bad_constraint = true;
      }
    }
  }
  EXPECT_FALSE(has_bad_constraint)
      << "Some constraint values are too large at rest";

  // Check Jacobian values - look for NaN or huge values
  int nv = m->nv;
  bool has_bad_jacobian = false;
  for (int i = 0; i < d->nefc; i++) {
    if (d->efc_type[i] == mjCNSTR_EQUALITY) {
      for (int j = 0; j < nv; j++) {
        mjtNum val = d->efc_J[i * nv + j];
        if (mju_isBad(val) || mju_abs(val) > 1e10) {
          has_bad_jacobian = true;
        }
      }
    }
  }
  EXPECT_FALSE(has_bad_jacobian) << "Jacobian contains NaN or huge values";

  // Verify Jacobian with finite differences for first few constraints
  mjtNum eps = 1e-6;
  std::vector<mjtNum> qpos0(m->nq);
  mju_copy(qpos0.data(), d->qpos, m->nq);

  // Store original constraint values
  std::vector<mjtNum> efc_pos0(d->nefc);
  mju_copy(efc_pos0.data(), d->efc_pos, d->nefc);

  int num_constraints_to_check = mju_min(3, d->ne);
  bool has_jacobian_mismatch = false;
  for (int j = 0; j < nv && j < 6; j++) {
    mju_copy(d->qpos, qpos0.data(), m->nq);
    mjtNum dqpos[100] = {0};
    dqpos[j] = eps;
    mj_integratePos(m.get(), d->qpos, dqpos, 1);
    mj_forward(m.get(), d.get());

    for (int i = 0; i < num_constraints_to_check; i++) {
      mjtNum fd = (d->efc_pos[i] - efc_pos0[i]) / eps;
      mjtNum analytic = d->efc_J[i * nv + j];
      // Use relative tolerance with absolute floor to handle near-zero values
      mjtNum tol = mju_max(1e-8, 0.1 * (mju_abs(fd) + mju_abs(analytic)));
      if (mju_abs(fd - analytic) > tol) {
        has_jacobian_mismatch = true;
      }
    }
  }
  EXPECT_FALSE(has_jacobian_mismatch)
      << "Jacobian FD mismatch at initial config";

  // Test rotation invariance: rotate via freejoint quaternion
  mju_copy(d->qpos, qpos0.data(), m->nq);
  mjtNum angle = 0.785398;          // 45 degrees
  d->qpos[3] = mju_cos(angle / 2);  // w
  d->qpos[4] = 0;
  d->qpos[5] = 0;
  d->qpos[6] = mju_sin(angle / 2);  // z
  mj_forward(m.get(), d.get());

  mjtNum max_strain_rotated = 0;
  for (int i = 0; i < d->ne; i++) {
    if (mju_abs(d->efc_pos[i]) > max_strain_rotated) {
      max_strain_rotated = mju_abs(d->efc_pos[i]);
    }
  }
  EXPECT_LT(max_strain_rotated, 1e-6)
      << "Strain should remain ~0 after rigid rotation";

  // Check Jacobian in rotated configuration via FD
  std::vector<mjtNum> qpos_rot(m->nq);
  mju_copy(qpos_rot.data(), d->qpos, m->nq);
  std::vector<mjtNum> efc_pos_rot(d->nefc);
  mju_copy(efc_pos_rot.data(), d->efc_pos, d->nefc);

  bool has_rotated_jacobian_mismatch = false;
  for (int j = 0; j < nv; j++) {
    mju_copy(d->qpos, qpos_rot.data(), m->nq);
    mjtNum dqpos[100] = {0};
    dqpos[j] = eps;
    mj_integratePos(m.get(), d->qpos, dqpos, 1);
    mj_forward(m.get(), d.get());

    mjtNum fd = (d->efc_pos[0] - efc_pos_rot[0]) / eps;
    mjtNum analytic = d->efc_J[0 * nv + j];
    mjtNum tol = 0.1 * (mju_abs(fd) + mju_abs(analytic) + 1e-8);
    if ((mju_abs(fd) > 1e-8 || mju_abs(analytic) > 1e-8) &&
        mju_abs(fd - analytic) > tol) {
      has_rotated_jacobian_mismatch = true;
    }
  }
  EXPECT_FALSE(has_rotated_jacobian_mismatch)
      << "Jacobian FD mismatch in rotated config";

  // Reset for simulation
  mju_copy(d->qpos, qpos0.data(), m->nq);
  mj_forward(m.get(), d.get());

  // Run simulation only if checks pass
  if (!has_bad_constraint && !has_bad_jacobian) {
    for (int i = 0; i < 2000; i++) {
      mj_step(m.get(), d.get());

      ASSERT_FALSE(mju_isBad(d->qpos[0]))
          << "Simulation became unstable at step " << i;
      ASSERT_FALSE(mju_isBad(d->qvel[0]))
          << "Velocity became unstable at step " << i;

      for (int j = 0; j < m->nv; j++) {
        ASSERT_LT(mju_abs(d->qvel[j]), 1000.0)
            << "Velocity exploded at step " << i << ", qvel[" << j
            << "]=" << d->qvel[j];
      }
    }
  }
}

// Test flex strain constraint WITHOUT pinned nodes (simpler case)
TEST_F(CoreConstraintTest, StrainConstraintNoPinning) {
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="discrete" jacobian="dense"/>
  <worldbody>
    <body name="parent">
      <flexcomp name="test" type="box"
                spacing=".1 .1 .1" radius="0.001"
                pos="0 0 .5" dof="trilinear" mass="1" dim="3">
        <contact selfcollide="none"/>
        <edge equality="strain"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);

  mj_resetData(m.get(), d.get());
  mj_forward(m.get(), d.get());

  // Check constraints
  EXPECT_GT(d->ne, 0) << "Expected strain constraints";

  // Check no contacts
  EXPECT_EQ(d->ncon, 0);

  // Check that initial strain is ~0
  mjtNum max_pos = 0;
  for (int i = 0; i < d->ne; i++) {
    if (mju_abs(d->efc_pos[i]) > max_pos) {
      max_pos = mju_abs(d->efc_pos[i]);
    }
  }
  EXPECT_LT(max_pos, 1e-6) << "Initial strain should be ~0";

  // Check Jacobian for NaN
  int nv = m->nv;
  bool has_bad_jacobian = false;
  for (int i = 0; i < d->ne; i++) {
    for (int j = 0; j < nv; j++) {
      if (mju_isBad(d->efc_J[i * nv + j])) {
        has_bad_jacobian = true;
      }
    }
  }
  EXPECT_FALSE(has_bad_jacobian) << "Jacobian has NaN";

  // Test rigid rotation: rotate flex nodes by 45 degrees around Z axis and
  // check strain still ~0
  std::vector<mjtNum> qpos0(m->nq);
  mju_copy(qpos0.data(), d->qpos, m->nq);
  mjtNum angle = 0.785398;  // 45 degrees
  mjtNum c = mju_cos(angle), s = mju_sin(angle);
  for (int n = 0; n < m->nflexnode; n++) {
    int b = m->flex_nodebodyid[n];
    mjtNum x0 = m->body_pos[3 * b + 0];
    mjtNum y0 = m->body_pos[3 * b + 1];
    d->qpos[3 * n + 0] = (c * x0 - s * y0) - x0;
    d->qpos[3 * n + 1] = (s * x0 + c * y0) - y0;
  }
  mj_forward(m.get(), d.get());

  mjtNum max_strain_rotated = 0;
  for (int i = 0; i < d->ne; i++) {
    if (mju_abs(d->efc_pos[i]) > max_strain_rotated) {
      max_strain_rotated = mju_abs(d->efc_pos[i]);
    }
  }
  EXPECT_LT(max_strain_rotated, 1e-6)
      << "Strain should remain ~0 after rigid rotation";

  // Run simulation for a few steps to check stability
  mju_copy(d->qpos, qpos0.data(), m->nq);
  mj_forward(m.get(), d.get());

  for (int i = 0; i < 100; i++) {
    mj_step(m.get(), d.get());
    ASSERT_FALSE(mju_isBad(d->qpos[0])) << "Simulation unstable at step " << i;
  }
}

// Test flex strain constraint with quadratic interpolation
TEST_F(CoreConstraintTest, StrainConstraintQuadratic) {
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="discrete" jacobian="dense"/>
  <worldbody>
    <body name="parent">
      <flexcomp name="test" type="box"
                spacing=".1 .1 .1" radius="0.001"
                pos="0 0 .5" dof="quadratic" mass="1" dim="3">
        <contact selfcollide="none"/>
        <edge equality="strain"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);

  mj_resetData(m.get(), d.get());
  mj_forward(m.get(), d.get());

  // Check constraints generated
  EXPECT_GT(d->ne, 0) << "Expected strain constraints";

  // Check that initial strain is ~0
  mjtNum max_pos = 0;
  for (int i = 0; i < d->ne; i++) {
    if (mju_abs(d->efc_pos[i]) > max_pos) {
      max_pos = mju_abs(d->efc_pos[i]);
    }
  }
  EXPECT_LT(max_pos, 1e-6) << "Initial strain should be ~0";

  // Check Jacobian for NaN
  int nv = m->nv;
  bool has_bad_jacobian = false;
  for (int i = 0; i < d->ne; i++) {
    for (int j = 0; j < nv; j++) {
      if (mju_isBad(d->efc_J[i * nv + j])) {
        has_bad_jacobian = true;
      }
    }
  }
  EXPECT_FALSE(has_bad_jacobian) << "Jacobian has NaN";

  // Run simulation for a few steps
  for (int i = 0; i < 100; i++) {
    mj_step(m.get(), d.get());
    ASSERT_FALSE(mju_isBad(d->qpos[0])) << "Simulation unstable at step " << i;
  }
}

TEST_F(CoreConstraintTest, ShellModeBendZeroForceAtRest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <flexcomp type="grid" count="8 8 8" spacing=".07 .07 .07" pos="0 0 1"
                dim="3" cellcount="1 1 1" radius=".001" rgba="0 .7 .7 1"
                mass="5" name="softbody" dof="trilinear">
        <elasticity young="0" poisson="0.1" damping="0.01"
                    elastic2d="bend" thickness="0.02"/>
        <edge equality="strain"/>
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), testing::NotNull()) << error;
  MjDataPtr d = MakeData(m);

  mj_forward(m.get(), d.get());

  // Check number of equalities
  EXPECT_EQ(m->neq, 6);

  // 6 faces * 6 modes per face = 36
  // (5 membrane modes from pure 2D eigendecomposition + 1 explicit warp)
  EXPECT_EQ(d->ne, 36);

  // all constraint residuals should be zero at rest
  for (int i = 0; i < d->ne; i++) {
    EXPECT_NEAR(d->efc_pos[i], 0, 1e-10)
        << "nonzero constraint residual at " << i;
  }
}

// Test quadratic passive forces (no constraints) for stability
TEST_F(CoreConstraintTest, QuadraticPassiveForceStability) {
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="discrete" solver="CG" tolerance="1e-6"/>
  <worldbody>
    <geom type="plane" size="10 10 1"/>
    <flexcomp name="test" type="grid" count="3 3 3"
              spacing=".05 .05 .05" radius="0.001"
              pos="0 0 .3" dof="quadratic" mass="1" dim="3">
      <contact selfcollide="none"/>
      <elasticity young="1e4" damping="0.01"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);

  // Run for 500 steps — should stay stable
  for (int i = 0; i < 500; i++) {
    mj_step(m.get(), d.get());
    ASSERT_FALSE(mju_isBad(d->qpos[0]))
        << "Passive quadratic unstable at step " << i;
    for (int j = 0; j < m->nv; j++) {
      ASSERT_LT(mju_abs(d->qvel[j]), 1000.0)
          << "Velocity exploded at step " << i;
    }
  }
}

// Test quadratic with anisotropic cells (like what mesh bounding box creates)
TEST_F(CoreConstraintTest, QuadraticAnisotropicStrain) {
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="discrete" solver="CG" tolerance="1e-6"/>
  <size memory="50M"/>
  <worldbody>
    <geom type="plane" size="10 10 1"/>
    <body name="parent">
      <flexcomp name="test" type="grid" count="3 3 3"
                spacing=".1 .05 .08" radius="0.001"
                pos="0 0 .5" dof="quadratic" mass="1" dim="3">
        <contact selfcollide="none"/>
        <edge equality="strain"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);

  mj_forward(m.get(), d.get());
  EXPECT_GT(d->ne, 0) << "Expected strain constraints";

  // Run for 200 steps with gravity + contact
  for (int i = 0; i < 200; i++) {
    mj_step(m.get(), d.get());
    ASSERT_FALSE(mju_isBad(d->qpos[0]))
        << "Anisotropic quadratic unstable at step " << i;
    for (int j = 0; j < m->nv; j++) {
      ASSERT_LT(mju_abs(d->qvel[j]), 1000.0)
          << "Velocity exploded at step " << i << ", qvel[" << j
          << "]=" << d->qvel[j];
    }
  }
}

TEST_F(CoreConstraintTest, ContactSharedDofJacobian) {
  constexpr char xml[] = R"(
  <mujoco>
    <option jacobian="sparse"/>
    <worldbody>
      <body pos="0 0 0.5">
        <joint type="slide" axis="0 0 1"/>
        <geom size="0.01"/>
        <body pos="0 0.04 0">
          <joint type="slide" axis="0 1 0"/>
          <geom type="sphere" size="0.05" condim="1"/>
        </body>
        <body pos="0 -0.04 0">
          <joint type="slide" axis="0 1 0"/>
          <geom type="sphere" size="0.05" condim="1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  ASSERT_EQ(model->nv, 3);
  ASSERT_TRUE(mj_isSparse(model.get()));
  MjDataPtr data = MakeData(model);

  mj_forward(model.get(), data.get());

  ASSERT_EQ(data->ncon, 1);
  ASSERT_GE(data->nefc, 1);

  EXPECT_EQ(data->efc_J_rownnz[0], 2);
}

// Tendon friction and limit constraints skip sleeping trees.
TEST_F(CoreConstraintTest, SleepingTendonFrictionAndLimit) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag sleep="enable"/>
    </option>
    <worldbody>
      <site name="s0" pos="0 0 0"/>
      <site name="s0_b" pos="2 0 0"/>
      <body name="body1" pos="0 0 0.2" sleep="init">
        <joint name="jnt1" type="slide" axis="0 0 1"/>
        <geom size=".1" contype="0" conaffinity="0"/>
        <site name="s1" pos="0 0 0"/>
      </body>
      <body name="body2" pos="2 0 0.2">
        <joint name="jnt2" type="slide" axis="0 0 1"/>
        <geom size=".1" contype="0" conaffinity="0"/>
        <site name="s2" pos="0 0 0"/>
      </body>
    </worldbody>
    <tendon>
      <spatial name="ten_friction" frictionloss="1">
        <site site="s0"/>
        <site site="s1"/>
      </spatial>
      <spatial name="ten_limit" range="0.5 1.0">
        <site site="s0"/>
        <site site="s1"/>
      </spatial>
      <spatial name="ten_awake_friction" frictionloss="1">
        <site site="s0_b"/>
        <site site="s2"/>
      </spatial>
      <spatial name="ten_awake_limit" range="0.5 1.0">
        <site site="s0_b"/>
        <site site="s2"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  for (mjtJacobian jac : {mjJAC_DENSE, mjJAC_SPARSE}) {
    model->opt.jacobian = jac;
    MjDataPtr data = MakeData(model);

    ASSERT_EQ(model->ntree, 2);
    ASSERT_EQ(data->ntree_awake, 1);
    ASSERT_EQ(data->tree_awake[0], 0);
    ASSERT_EQ(data->tree_awake[1], 1);

    mj_forward(model.get(), data.get());

    // Sleeping tendons skip friction and limit; awake tendons instantiate both.
    EXPECT_EQ(data->nf, 1);
    EXPECT_EQ(data->nl, 1);
    EXPECT_EQ(data->tendon_efcadr[0], -1);
    EXPECT_EQ(data->tendon_efcadr[1], -1);
    EXPECT_GE(data->tendon_efcadr[2], 0);
    EXPECT_GE(data->tendon_efcadr[3], 0);
  }
}

static const char* const kJdotvConnect2dPath =
    "engine/testdata/core_constraint/jdotv_connect_2d.xml";
static const char* const kJdotvConnect3dPath =
    "engine/testdata/core_constraint/jdotv_connect_3d.xml";
static const char* const kJdotvWeld3dPath =
    "engine/testdata/core_constraint/jdotv_weld_3d.xml";

// validate mj_Jdotv against finite-differenced constraint Jacobian
TEST_F(CoreConstraintTest, JdotvFiniteDifference) {
  for (const char* path :
       {kJdotvConnect2dPath, kJdotvConnect3dPath, kJdotvWeld3dPath}) {
    const std::string xml_path = GetTestDataFilePath(path);
    char err[1024];
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
    ASSERT_THAT(m, NotNull()) << err << " for " << path;
    int nv = m->nv;
    mjData* d = mj_makeData(m);

    // simulate for 1 second to accumulate velocity
    while (d->time < 1.0) {
      mj_step(m, d);
    }

    // forward to populate constraints
    mj_forward(m, d);
    ASSERT_GT(d->ne, 0) << "no equality constraints for " << path;
    int ne = d->ne;

    // get dense J_0 (ne x nv)
    std::vector<mjtNum> J0(ne * nv);
    if (mj_isSparse(m)) {
      mju_sparse2dense(J0.data(), d->efc_J, ne, nv, d->efc_J_rownnz,
                       d->efc_J_rowadr, d->efc_J_colind);
    } else {
      mju_copy(J0.data(), d->efc_J, ne * nv);
    }

    // compute mj_Jdotv at current state
    std::vector<mjtNum> jdv(ne, 0);
    mj_Jdotv(m, d, jdv.data());

    // save qpos and qvel
    std::vector<mjtNum> qpos0(m->nq), qvel0(nv);
    mju_copy(qpos0.data(), d->qpos, m->nq);
    mju_copy(qvel0.data(), d->qvel, nv);

    // integrate qpos forward by h using qvel
    const mjtNum h = MjTol(1e-7, 5e-4);
    mj_integratePos(m, d->qpos, d->qvel, h);
    mj_forward(m, d);

    // get dense J_h (ne x nv)
    ASSERT_EQ(d->ne, ne) << "constraint count changed after integration";
    std::vector<mjtNum> Jh(ne * nv);
    if (mj_isSparse(m)) {
      mju_sparse2dense(Jh.data(), d->efc_J, ne, nv, d->efc_J_rownnz,
                       d->efc_J_rowadr, d->efc_J_colind);
    } else {
      mju_copy(Jh.data(), d->efc_J, ne * nv);
    }

    // FD: Jdotv_fd[i] = -sum_j (Jh[i,j] - J0[i,j]) / h * qvel[j]
    // (negated because mj_Jdotv subtracts)
    std::vector<mjtNum> jdv_fd(ne, 0);
    for (int i = 0; i < ne; i++) {
      for (int j = 0; j < nv; j++) {
        jdv_fd[i] -= (Jh[i * nv + j] - J0[i * nv + j]) / h * qvel0[j];
      }
    }

    // compare
    EXPECT_THAT(AsVector(jdv.data(), ne),
                Pointwise(MjNear(1e-4, 1e-2), AsVector(jdv_fd.data(), ne)))
        << "Jdotv FD mismatch for " << path;

    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

// Test 2: forward-inverse identity preserved with Jdot*v correction
TEST_F(CoreConstraintTest, JdotvFwdInvIdentity) {
  for (const char* path :
       {kJdotvConnect2dPath, kJdotvConnect3dPath, kJdotvWeld3dPath}) {
    const std::string xml_path = GetTestDataFilePath(path);
    char err[1024];
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
    ASSERT_THAT(m, NotNull()) << err;
    mjData* d = mj_makeData(m);

    // give initial velocity
    for (int i = 0; i < m->nv; i++) d->qvel[i] = 0.5 * (i + 1);

    // forward (with correction ON by default)
    mj_forward(m, d);
    mj_compareFwdInv(m, d);
    mjtNum fwdinv = d->solver_fwdinv[0];

    mjtNum epsilon = MjTol(1e-10, 1e-2);
    EXPECT_LT(fwdinv, epsilon)
        << "fwdinv broken for " << path << " (fwdinv=" << fwdinv << ")";

    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

// --------------------------- strain constraint rotated parent ----------------

struct StrainConstraintTestCase {
  std::string test_name;
  std::string body_pos;
  std::string body_quat;
  std::string flex_spacing;
  std::string flex_xyaxes;
};

class StrainConstraintRotatedTest
    : public CoreConstraintTest,
      public ::testing::WithParamInterface<StrainConstraintTestCase> {};

TEST_P(StrainConstraintRotatedTest, ResidualIsZero) {
  auto param = GetParam();
  std::string xml = R"(
  <mujoco>
  <option integrator="discrete" jacobian="dense" gravity="0 0 0"/>
  <worldbody>
    <body name="parent" )";

  if (!param.body_pos.empty()) {
    xml += "pos=\"" + param.body_pos + "\" ";
  }
  if (!param.body_quat.empty()) {
    xml += "quat=\"" + param.body_quat + "\" ";
  }
  xml += R"(>
      <flexcomp name="test" type="box" )";

  if (!param.flex_spacing.empty()) {
    xml += "spacing=\"" + param.flex_spacing + "\" ";
  }
  if (!param.flex_xyaxes.empty()) {
    xml += "xyaxes=\"" + param.flex_xyaxes + "\" ";
  }

  xml += R"(radius="0.001"
                pos="0 0 0" dof="trilinear" mass="1" dim="3">
        <contact selfcollide="none"/>
        <edge equality="strain"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml.c_str(), error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);

  mj_forward(m.get(), d.get());

  // Check we have strain constraints
  EXPECT_GT(d->ne, 0) << "Expected strain constraints";

  // The critical check: constraint residuals must be ~0 at the initial
  // (undeformed) configuration, even though the body is rotated.
  mjtNum max_pos = 0;
  for (int i = 0; i < d->ne; i++) {
    max_pos = mju_max(max_pos, mju_abs(d->efc_pos[i]));
  }
  EXPECT_LT(max_pos, MjTol(1e-6, 1e-5))
      << "Strain constraint residual should be ~0"
      << " (max_pos=" << max_pos << ")";

  // Verify stability
  for (int i = 0; i < 200; i++) {
    mj_step(m.get(), d.get());
    ASSERT_FALSE(mju_isBad(d->qpos[0])) << "Simulation unstable at step " << i;
    for (int j = 0; j < m->nv; j++) {
      ASSERT_LT(mju_abs(d->qvel[j]), 1000.0)
          << "Velocity exploded at step " << i << ", qvel[" << j
          << "]=" << d->qvel[j];
    }
  }
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(
  StrainConstraintRotatedTests, StrainConstraintRotatedTest,
  testing::ValuesIn<StrainConstraintTestCase>({
      // Test strain constraint with a rotated parent body.
      // The flexcomp is placed inside a parent body that has a non-identity
      // initial rotation. This reproduces the "grocery scene" bug where the
      // stiffness matrix eigenvectors and reference positions were computed
      // in world frame instead of the unrotated local frame, causing
      // spurious constraint forces.
      {
          "RotatedParent",
          "1 2 3",
          "0.707107 0 0.707107 0",
          ".1 .1 .1",
          ""
      },
      // Same test with an anisotropic box (different spacing per axis) and
      // arbitrary rotation (combined 45-deg Y + 30-deg X).
      {
          "RotatedParentAnisotropic",
          "0.5 -1 2",
          "0.8924 0.2392 0.3696 -0.0990",
          ".15 .08 .05",
          ""
      },
      // Test strain constraint with flexcomp-level xyaxes rotation.
      // This is the "grocery scene" pattern where the flexcomp grid itself is
      // rotated via xyaxes="0 1 0 0 0 1" (X->Y, Y->Z).
      {
          "FlexcompXyaxes",
          "",
          "",
          ".1 .02 .1",
          "0 1 0 0 0 1"
      },
      // Test combining parent body rotation with flexcomp xyaxes rotation.
      // The total rotation is the composition of both.
      {
          "RotatedParentPlusXyaxes",
          "1 2 3",
          "0.707107 0 0.707107 0",
          ".15 .08 .05",
          "0 1 0 0 0 1"
      }
  }),
  [](const testing::TestParamInfo<
      StrainConstraintRotatedTest::ParamType>& info) {
    return info.param.test_name;
  }
);
// clang-format on

TEST_F(CoreConstraintTest, ShellModeContactJacobian) {
  constexpr char xml[] = R"(
  <mujoco>
    <option jacobian="dense"/>
    <worldbody>
      <flexcomp name="flex" type="grid" count="3 3 3" spacing=".1 .1 .1" dim="3" dof="trilinear">
        <elasticity elastic2d="stretch" thickness="0.01"/>
        <contact selfcollide="none"/>
      </flexcomp>
      <geom type="plane" size="1 1 1" pos="0 0 -1"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), testing::NotNull()) << error;
  MjDataPtr data = MakeData(model);

  mj_forward(model.get(), data.get());

  // find central vertex index (13 for 3x3x3 grid)
  int central_idx = 13;

  // verify it is interior
  int nx = 3, ny = 3, nz = 3;
  int k = central_idx / (nx * ny);
  int rest = central_idx % (nx * ny);
  int j = rest / nx;
  int i = rest % nx;
  ASSERT_TRUE(i > 0 && i < nx - 1 && j > 0 && j < ny - 1 && k > 0 &&
              k < nz - 1);

  // create manual contact with central vertex
  mjContact con;
  memset(&con, 0, sizeof(mjContact));
  con.flex[0] = -1;
  con.flex[1] = -1;
  con.vert[0] = -1;
  con.vert[1] = -1;
  con.geom[0] = model->ngeom - 1;  // plane geom
  con.geom[1] = -1;  // must be -1 to trigger flex branch in mj_contactJacobian
  con.flex[1] = 0;
  con.vert[1] = central_idx;
  con.dim = 1;
  mju_copy3(con.pos, data->flexvert_xpos + 3 * central_idx);
  con.frame[0] = 0;
  con.frame[1] = 0;
  con.frame[2] = 1;  // normal

  // buffer for Jacobian
  std::vector<mjtNum> jacdif(3 * model->nv, 0.0);

  mj_contactJacobian(model.get(), data.get(), &con, 1, jacdif.data(), nullptr,
                     nullptr, nullptr, nullptr, nullptr, nullptr);

  // check that boundary nodes have non-zero entries, and central node has zero

  bool boundary_has_dof = false;
  bool interior_has_dof = false;

  for (int n = 0; n < model->flex_nodenum[0]; n++) {
    int b = model->flex_nodebodyid[model->flex_nodeadr[0] + n];
    int dofadr = model->body_dofadr[b];
    int dofnum = model->body_dofnum[b];

    bool has_jac = false;
    if (dofadr >= 0) {
      for (int d = 0; d < dofnum; d++) {
        if (mju_abs(jacdif[dofadr + d]) > 1e-6) {
          has_jac = true;
        }
      }
    }

    int kn = n / (nx * ny);
    int restn = n % (nx * ny);
    int jn = restn / nx;
    int in = restn % nx;
    bool is_interior = (in > 0 && in < nx - 1 && jn > 0 && jn < ny - 1 &&
                        kn > 0 && kn < nz - 1);

    if (is_interior) {
      if (has_jac) interior_has_dof = true;
    } else {
      if (has_jac) boundary_has_dof = true;
    }
  }

  EXPECT_TRUE(boundary_has_dof)
      << "Boundary nodes should receive contact force";
  EXPECT_FALSE(interior_has_dof)
      << "Interior nodes should not receive contact force";
}

// flex element contact Jacobian interpolates the closest point on the element
TEST_F(CoreConstraintTest, FlexElementContactJacobianBarycentric) {
  constexpr char xml[] = R"(
  <mujoco>
    <option jacobian="dense">
      <flag contact="disable" gravity="disable"/>
    </option>
    <worldbody>
      <geom name="floor" type="plane" size="1 1 .1" pos="0 0 -1"/>
      <flexcomp name="seg" type="direct" dim="1" radius=".01"
                point="0 0 0  1 0 0" element="0 1"/>
      <flexcomp name="tri" type="direct" dim="2" radius=".01"
                point="0 0 0  1 0 0  0 1 0" element="0 1 2"/>
      <flexcomp name="tet" type="direct" dim="3" radius=".01"
                point="0 0 0  1 0 0  0 1 0  0 0 1" element="0 1 2 3"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  ASSERT_EQ(model->nflex, 3);
  int floor_id = mj_name2id(model.get(), mjOBJ_GEOM, "floor");
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());
  int nv = model->nv;

  struct TestCase {
    int flex;           // flex id: 0 segment, 1 triangle, 2 tetrahedron
    int vert;           // excluded vertex of the same flex, or -1
    mjtNum pos[3];      // contact position
    mjtNum closest[3];  // expected closest point on the element or fragment
  };
  const TestCase cases[] = {
      // segment: interior, beyond endpoint
      {0, -1, {.3, .2, 0}, {.3, 0, 0}},
      {0, -1, {1.5, 0, 0}, {1, 0, 0}},

      // triangle: above interior, vertex region, edge region
      {1, -1, {.25, .25, .1}, {.25, .25, 0}},
      {1, -1, {-.5, -.5, 0}, {0, 0, 0}},
      {1, -1, {.5, -.2, .1}, {.5, 0, 0}},

      // tetrahedron: interior, below bottom face, outside slanted face
      {2, -1, {.1, .2, .3}, {.1, .2, .3}},
      {2, -1, {.2, .3, -.05}, {.2, .3, 0}},
      {2, -1, {.5, .5, .5}, {1.0 / 3, 1.0 / 3, 1.0 / 3}},

      // tetrahedron without vertex (0, 0, 1): closest point on bottom face
      {2, 3, {.2, .3, .4}, {.2, .3, 0}},
  };

  for (const TestCase& c : cases) {
    SCOPED_TRACE(testing::Message()
                 << "flex " << c.flex << ", pos (" << c.pos[0] << ", "
                 << c.pos[1] << ", " << c.pos[2] << ")");

    // contact of the floor or a flex vertex (side 0) with element 0 (side 1)
    mjContact con = {};
    con.geom[0] = c.vert >= 0 ? -1 : floor_id;
    con.geom[1] = -1;
    con.flex[0] = c.vert >= 0 ? c.flex : -1;
    con.vert[0] = c.vert;
    con.elem[0] = -1;
    con.flex[1] = c.flex;
    con.vert[1] = -1;
    con.elem[1] = 0;
    mju_copy3(con.pos, c.pos);

    std::vector<mjtNum> jacdif(3 * nv, 0);
    mj_contactJacobian(model.get(), data.get(), &con, 1, jacdif.data(), nullptr,
                       nullptr, nullptr, nullptr, nullptr, nullptr);

    // vertex weights are the x-row entries of the vertex x-slider dofs
    int vertadr = model->flex_vertadr[c.flex];
    mjtNum sum = 0;
    mjtNum point[3] = {0, 0, 0};
    for (int v = 0; v < model->flex_vertnum[c.flex]; v++) {
      int dofadr = model->body_dofadr[model->flex_vertbodyid[vertadr + v]];
      mjtNum weight = jacdif[dofadr];

      // excluded vertex is side 0 only
      if (v == c.vert) {
        EXPECT_EQ(weight, -1) << "excluded vertex " << v;
        continue;
      }
      EXPECT_GE(weight, 0) << "vertex " << v;
      sum += weight;
      mju_addToScl3(point, data->flexvert_xpos + 3 * (vertadr + v), weight);
    }

    // weights are a partition of unity which interpolates the closest point
    EXPECT_THAT(sum, MjNear(1, 1e-12, 1e-6));
    EXPECT_THAT(point, Pointwise(MjNear(1e-12, 1e-6), c.closest));
  }
}

// ------------------------------- adhesion ------------------------------------

static const char* kAdhereCeiling = R"(
<mujoco>
  <worldbody>
    <geom type="box" size=".2 .2 .02" pos="0 0 1" gap=".05" adhesion="20"/>
    <body pos="0 0 .93">
      <freejoint/>
      <geom type="box" size=".1 .1 .05" mass="1"/>
    </body>
  </worldbody>
</mujoco>
)";

// a box sticks to an adhesive ceiling, holds up to ncon*delta, then detaches
TEST_F(CoreConstraintTest, AdhesionPulloff) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kAdhereCeiling, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  // hangs at rest
  while (data->time < 2) {
    mj_step(model.get(), data.get());
  }
  ASSERT_EQ(data->ncon, 4);
  EXPECT_GT(data->qpos[2], 0.9);

  // holds a load below 4*delta - mg
  data->xfrc_applied[6 * 1 + 2] = -60;
  while (data->time < 4) {
    mj_step(model.get(), data.get());
  }
  EXPECT_GT(data->qpos[2], 0.9);

  // detaches above 4*delta
  data->xfrc_applied[6 * 1 + 2] = -85;
  while (data->time < 5) {
    mj_step(model.get(), data.get());
  }
  EXPECT_LT(data->qpos[2], 0.5);
}

// under tension the box hangs at positive separation which grows with load
TEST_F(CoreConstraintTest, AdhesionTether) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kAdhereCeiling, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  mjtNum dist[2];
  for (int k = 0; k < 2; k++) {
    data->xfrc_applied[6 * 1 + 2] = k ? -50 : 0;
    mjtNum tend = data->time + 3;
    while (data->time < tend) {
      mj_step(model.get(), data.get());
    }
    ASSERT_GT(data->ncon, 0);
    dist[k] = data->contact[0].dist;
  }
  EXPECT_GT(dist[0], 0);        // hanging in tension at positive separation
  EXPECT_GT(dist[1], dist[0]);  // separation grows with load
}

// a box released inside the adhesion band is captured into steady contact
TEST_F(CoreConstraintTest, AdhesionCapture) {
  std::string xml = kAdhereCeiling;
  size_t pos = xml.find("0 0 .93");
  xml.replace(pos, 7, "0 0 .89");
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  while (data->time < 3) {
    mj_step(model.get(), data.get());
  }
  mjtNum zmin = 10, zmax = -10;
  while (data->time < 4) {
    mj_step(model.get(), data.get());
    zmin = mju_min(zmin, data->qpos[2]);
    zmax = mju_max(zmax, data->qpos[2]);
  }
  EXPECT_GT(zmax, 0.928);        // captured up to the ceiling
  EXPECT_LT(zmax - zmin, 1e-4);  // and steady
}

// resting penetration is independent of adhesion (the R*delta correction)
TEST_F(CoreConstraintTest, AdhesionRestingPenetration) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="plane" size="2 2 .1" adhesion="ADH"/>
      <body pos="0 0 .1">
        <freejoint/>
        <geom type="box" size=".1 .1 .1" mass="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjtNum depth[2];
  for (int k = 0; k < 2; k++) {
    std::string xml2 = xml;
    size_t pos = xml2.find("ADH");
    xml2.replace(pos, 3, k ? "50" : "0");
    MjModelPtr model = LoadModelFromString(xml2.c_str(), error, sizeof(error));
    ASSERT_THAT(model.get(), testing::NotNull()) << error;
    MjDataPtr data = MakeData(model);
    while (data->time < 3) {
      mj_step(model.get(), data.get());
    }
    ASSERT_GT(data->ncon, 0);
    depth[k] = -data->contact[0].dist;

    // net reported force sums to the weight
    if (k) {
      mjtNum total = 0, f[6];
      for (int i = 0; i < data->ncon; i++) {
        mj_contactForce(model.get(), data.get(), i, f);
        total += f[0];
      }
      EXPECT_NEAR(total, 9.81, 1e-2);
    }
  }
  EXPECT_NEAR(depth[0], depth[1], MjTol(1e-9, 1e-6));
}

// adhesion combines by sum, with pair override
TEST_F(CoreConstraintTest, AdhesionCombination) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="2 2 .1" adhesion="10"/>
      <body pos="0 0 .1">
        <freejoint/>
        <geom name="box" type="box" size=".1 .1 .1" mass="1" adhesion="5"/>
      </body>
    </worldbody>
    <contact>
      <pair geom1="floor" geom2="box" adhesion="7"/>
    </contact>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), testing::NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());
  ASSERT_GT(data->ncon, 0);

  // explicit pair overrides: 7, not 10+5
  EXPECT_EQ(data->contact[0].adhesion, 7);
}

TEST_F(CoreConstraintTest, AdhesionPriority) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="2 2 .1" adhesion="10" priority="1"/>
      <body pos="0 0 .1">
        <freejoint/>
        <geom name="box" type="box" size=".1 .1 .1" mass="1" adhesion="5"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), testing::NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mj_forward(model.get(), data.get());
  ASSERT_GT(data->ncon, 0);

  // higher-priority geom wins: 10, not 10+5
  EXPECT_EQ(data->contact[0].adhesion, 10);
}

// Verify that the rotational Jacobian from mj_jacSum in the sparse path matches
// the dense path. This is a regression test for the bug where the sparse path
// wrote rotational rows at offset 3*NV inside a packed buffer, but the caller
// expected them at offset 3*nv.
TEST_F(CoreConstraintTest, SparseRotationalJacobianMatchesDense) {
  // XML template with a %s placeholder for the jacobian type
  constexpr char xml_template[] = R"(
  <mujoco>
    <option jacobian="%s"/>
    <worldbody>
      <body name="parent" pos="0 0 1">
        <freejoint/>
        <geom size=".01" mass="1"/>
        <flexcomp name="flex" type="grid" count="3 3 3" spacing=".1 .1 .1" dim="3" dof="trilinear">
          <elasticity elastic2d="stretch" thickness="0.01"/>
          <contact selfcollide="none"/>
          <pin id="0"/>
        </flexcomp>
      </body>
      <geom name="plane" type="plane" size="1 1 1" pos="0 0 -1"/>
    </worldbody>
  </mujoco>
  )";

  auto run_contact_jacobian = [&](const char* jacobian_type,
                                  std::vector<mjtNum>& jacdifp_dense_out,
                                  std::vector<mjtNum>& jacdifr_dense_out) {
    char xml[1024];
    snprintf(xml, sizeof(xml), xml_template, jacobian_type);
    char error[1024];
    MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
    ASSERT_THAT(model.get(), testing::NotNull()) << error;
    MjDataPtr data = MakeData(model);
    mj_forward(model.get(), data.get());

    int nv = model->nv;

    // pick a boundary vertex so the Jacobian is non-trivial
    int vert_idx = 0;

    int plane_geom_id = mj_name2id(model.get(), mjOBJ_GEOM, "plane");
    ASSERT_GE(plane_geom_id, 0);

    // create a frictional contact with dim=6 (normal + 2 tangent + 3 torsional)
    mjContact con;
    memset(&con, 0, sizeof(mjContact));
    con.flex[0] = -1;
    con.flex[1] = -1;
    con.vert[0] = -1;
    con.vert[1] = -1;
    con.geom[0] = plane_geom_id;  // plane geom (world body)
    con.geom[1] = -1;             // trigger flex branch in mj_contactJacobian
    con.flex[1] = 0;
    con.vert[1] = vert_idx;
    con.dim = 6;  // full frictional contact: exercises rotational Jacobian
    mju_copy3(con.pos, data->flexvert_xpos + 3 * vert_idx);
    // set contact frame to identity
    mju_zero(con.frame, 9);
    con.frame[0] = 1;
    con.frame[4] = 1;
    con.frame[8] = 1;

    // allocate output buffers
    std::vector<mjtNum> jacdifp(3 * nv, 0.0);
    std::vector<mjtNum> jacdifr(3 * nv, 0.0);
    std::vector<int> chain(nv, 0);

    int NV = mj_contactJacobian(model.get(), data.get(), &con, con.dim,
                                jacdifp.data(), jacdifr.data(), nullptr,
                                nullptr, nullptr, nullptr, chain.data());
    ASSERT_GT(NV, 0);

    // for sparse: unpack to dense using chain indices
    if (strcmp(jacobian_type, "sparse") == 0) {
      std::vector<mjtNum> jacdifp_full(3 * nv, 0.0);
      std::vector<mjtNum> jacdifr_full(3 * nv, 0.0);
      for (int row = 0; row < 3; row++) {
        for (int j = 0; j < NV; j++) {
          jacdifp_full[row * nv + chain[j]] = jacdifp[row * NV + j];
          jacdifr_full[row * nv + chain[j]] = jacdifr[row * NV + j];
        }
      }
      jacdifp_dense_out = std::move(jacdifp_full);
      jacdifr_dense_out = std::move(jacdifr_full);
    } else {
      jacdifp_dense_out = std::move(jacdifp);
      jacdifr_dense_out = std::move(jacdifr);
    }
  };

  // compute Jacobians with dense and sparse paths
  std::vector<mjtNum> dense_jacdifp, dense_jacdifr;
  std::vector<mjtNum> sparse_jacdifp, sparse_jacdifr;

  run_contact_jacobian("dense", dense_jacdifp, dense_jacdifr);
  ASSERT_FALSE(HasFatalFailure());
  run_contact_jacobian("sparse", sparse_jacdifp, sparse_jacdifr);
  ASSERT_FALSE(HasFatalFailure());

  ASSERT_EQ(dense_jacdifp.size(), sparse_jacdifp.size());
  ASSERT_EQ(dense_jacdifr.size(), sparse_jacdifr.size());

  // verify positional Jacobian matches
  const mjtNum tol = MjTol(1e-10, 1e-5);
  for (int i = 0; i < dense_jacdifp.size(); i++) {
    EXPECT_NEAR(dense_jacdifp[i], sparse_jacdifp[i], tol)
        << "jacdifp mismatch at index " << i;
  }

  // verify rotational Jacobian matches (this is where the bug was)
  bool has_nonzero_rot = false;
  for (int i = 0; i < dense_jacdifr.size(); i++) {
    EXPECT_NEAR(dense_jacdifr[i], sparse_jacdifr[i], tol)
        << "jacdifr mismatch at index " << i;
    if (mju_abs(dense_jacdifr[i]) > tol) {
      has_nonzero_rot = true;
    }
  }
  EXPECT_TRUE(has_nonzero_rot)
      << "Rotational Jacobian should have non-zero entries";
}

// rows touching some but not all dofs of a simple body (friction loss on a free
// joint, a limit and an equality on a second slide): simple dofs have diagonal
// rows in M, so the sparse Y follows M's pattern rather than the dof tree, and
// matches the dense Y
TEST_F(CoreConstraintTest, SparseYSimpleBodyMatchesDense) {
  static constexpr char xml_template[] = R"(
  <mujoco>
    <option solver="PGS" jacobian="%s"/>
    <worldbody>
      <body>
        <joint type="free" frictionloss=".01"/>
        <geom size=".1"/>
      </body>
      <body pos="1 0 0">
        <joint axis="0 1 0"/>
        <geom type="capsule" size=".05" fromto="0 0 0 .3 0 0"/>
        <body pos=".3 0 0">
          <joint name="hinge" axis="0 1 0"/>
          <geom type="capsule" size=".05" fromto="0 0 0 .3 0 0"/>
        </body>
      </body>
      <body pos="0 1 0">
        <joint type="slide" axis="1 0 0"/>
        <joint name="slide" type="slide" axis="0 1 0" range=".05 .1"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <equality>
      <joint joint1="slide" joint2="hinge"/>
    </equality>
  </mujoco>
  )";
  char error[1024];
  char xml[2048];

  // Y in dense layout, from the dense and sparse Jacobian paths
  std::vector<mjtNum> Y[2];
  for (int s = 0; s < 2; s++) {
    std::snprintf(xml, sizeof(xml), xml_template, s ? "sparse" : "dense");
    MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    MjDataPtr data = MakeData(model);
    mj_forward(model.get(), data.get());
    int nv = model->nv;
    int nefc = data->nefc;

    // one equality, six friction rows, one limit
    ASSERT_EQ(data->ne, 1);
    ASSERT_EQ(data->nf, 6);
    ASSERT_EQ(data->nl, 1);

    Y[s].resize(nefc * nv);
    if (s) {
      mju_sparse2dense(Y[s].data(), data->efc_Y, nefc, nv, data->efc_Y_rownnz,
                       data->efc_Y_rowadr, data->efc_Y_colind);
    } else {
      mju_copy(Y[s].data(), data->efc_Y, nefc * nv);
    }
  }
  EXPECT_THAT(Y[1], Pointwise(MjNear(1e-12, 1e-6), Y[0]));
}

// an arena overflow after the constraint rows are instantiated drops all rows:
// the per-type row counts must clear with the arrays, which consumers such as
// mj_Jdotv index by them
TEST_F(CoreConstraintTest, ArenaOverflowClearsRowCounts) {
  mock_warning_handler.ExpectWarnings("Insufficient arena memory");

  // 100 contacts under PGS: the dense AR (nefc x nefc) does not fit the arena
  static constexpr char xml[] = R"(
  <mujoco>
    <size memory="512K"/>
    <option solver="PGS"/>
    <worldbody>
      <geom type="plane" size="1 1 .1"/>
      <body name="grid" pos="0 0 .049">
        <freejoint/>
        <replicate count="10" offset=".1 0 0">
          <replicate count="10" offset="0 .1 0">
            <geom size=".05"/>
          </replicate>
        </replicate>
      </body>
      <body pos="0 0 1">
        <joint type="slide" axis="0 0 1" range="-1 -0.1" frictionloss="1"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <equality>
      <weld body1="grid"/>
    </equality>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);

  // classic integrators consume the rows at the velocity stage, discrete at the
  // actuation stage
  for (mjtIntegrator integrator : {mjINT_EULER, mjINT_DISCRETE}) {
    m->opt.integrator = integrator;
    mj_resetData(m.get(), d.get());
    mj_step(m.get(), d.get());

    // rows were instantiated, then dropped
    EXPECT_GT(d->warning[mjWARN_CNSTRFULL].number, 0);
    EXPECT_GT(d->maxuse_efc, 0);
    EXPECT_EQ(d->nefc, 0);
    EXPECT_EQ(d->ne, 0);
    EXPECT_EQ(d->nf, 0);
    EXPECT_EQ(d->nl, 0);
    EXPECT_EQ(d->nidof, 0);
    if (integrator == mjINT_EULER) {
      EXPECT_EQ(d->parena, d->ncon * sizeof(mjContact));
    }
  }
}

// tiny tangential residual T where T^3 underflows must not poison constraint
// force/cost
TEST_F(CoreConstraintTest, EllipticConeSmallTangent) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option cone="elliptic"/>
    <worldbody>
      <geom type="plane" size="1 1 .1"/>
      <body pos="0 0 .099">
        <freejoint/>
        <geom size=".1" condim="6" friction="1 .01 .001"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());
  ASSERT_EQ(d->ncon, 1);
  ASSERT_EQ(d->nefc, 6);

  const mjContact& con = d->contact[0];
  const mjtNum mu = con.mu;

  // T > 0 where T^2 is normal but T^3 underflows to 0 (even with subnormals),
  // while orthogonal tangent components are 0 so Inf * 0 would produce NaN
  const mjtNum T = MjEps(1e-110, 1e-17);
  const mjtNum N = -0.5 * T / mu;
  mjtNum jar[6] = {N / mu, T / con.friction[0], 0, 0, 0, 0};

  mjtNum cost = 0;
  mj_constraintUpdate(m.get(), d.get(), jar, &cost);

  EXPECT_EQ(d->efc_state[0], mjCNSTRSTATE_CONE);
  EXPECT_FALSE(mju_isBad(cost));
  for (int j = 0; j < 6; j++) {
    EXPECT_FALSE(mju_isBad(d->efc_force[j])) << "row " << j;
  }
}

TEST_F(CoreConstraintTest, TendonEqualitySetsEfcadr) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="b1" pos="0 0 1">
        <joint name="j1" type="slide" axis="1 0 0"/>
        <geom size=".1"/>
      </body>
      <body name="b2" pos="1 0 1">
        <joint name="j2" type="slide" axis="1 0 0"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <tendon>
      <fixed name="t1"><joint joint="j1" coef="1"/></fixed>
      <fixed name="t2"><joint joint="j2" coef="1"/></fixed>
    </tendon>
    <equality>
      <connect body1="b1" anchor="0 0 0"/>
      <tendon tendon1="t1" tendon2="t2"/>
    </equality>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());

  // connect adds 3 efc rows (0..2); tendon equality (eq index 1) starts at
  // efcadr 3
  EXPECT_EQ(d->tendon_efcadr[0], 3);
  EXPECT_EQ(d->tendon_efcadr[1], 3);
}

TEST_F(CoreConstraintTest, ZeroJacobianTendonDoesNotSetEfcadr) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="s1" pos="0 0 0"/>
      <site name="s2" pos="1 0 0"/>
      <body pos="0 0 1">
        <joint type="slide" axis="0 0 1"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <tendon>
      <spatial name="t_eq">
        <site site="s1"/>
        <site site="s2"/>
      </spatial>
      <spatial name="t_fric" frictionloss="1">
        <site site="s1"/>
        <site site="s2"/>
      </spatial>
      <spatial name="t_lim" limited="true" range="0 0.5">
        <site site="s1"/>
        <site site="s2"/>
      </spatial>
    </tendon>
    <equality>
      <tendon tendon1="t_eq"/>
    </equality>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();

  for (int jacobian : {mjJAC_DENSE, mjJAC_SPARSE}) {
    m->opt.jacobian = jacobian;
    MjDataPtr d = MakeData(m);
    mj_forward(m.get(), d.get());

    EXPECT_EQ(d->nefc, 0);
    EXPECT_EQ(d->tendon_efcadr[0], -1);
    EXPECT_EQ(d->tendon_efcadr[1], -1);
    EXPECT_EQ(d->tendon_efcadr[2], -1);
  }
}

TEST_F(CoreConstraintTest, DiscreteCoarseTimestepTangentialStiction) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.02" integrator="discrete" cone="elliptic" impratio="10"/>
    <worldbody>
      <geom type="plane" size="2 2 .1" euler="0 20 0" friction="0.5 0.5 0.01"
            solref="0.002 1"/>
      <body pos="0.03386 0 0.09303" euler="0 20 0">
        <freejoint/>
        <geom type="box" size="0.1 0.1 0.1" mass="1" friction="0.5 0.5 0.01"
              solref="0.002 1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);

  mj_forward(m.get(), d.get());
  ASSERT_GT(d->ncon, 0);

  // compare normal R[0] (clamped by refsafe excess > 1) vs tangential R[1]
  // (unclamped)
  int adr = d->contact[0].efc_address;
  mjtNum R_normal_safe = d->efc_R[adr];
  mjtNum R_tan_safe = d->efc_R[adr + 1];

  m->opt.disableflags |= mjDSBL_REFSAFE;
  mj_forward(m.get(), d.get());
  mjtNum R_normal_unclamped = d->efc_R[adr];
  mjtNum R_tan_unclamped = d->efc_R[adr + 1];

  // normal R is softened by refsafe to prevent coarse-h rebound,
  // while tangential R matches the unclamped completed weight
  EXPECT_GT(R_normal_safe, 10.0 * R_normal_unclamped);
  EXPECT_NEAR(R_tan_safe, R_tan_unclamped, MjTol(1e-12, 1e-6));

  // re-enable refsafe and simulate 50 coarse steps (1.0 s): block stays in
  // stiction
  m->opt.disableflags &= ~mjDSBL_REFSAFE;
  mj_resetData(m.get(), d.get());
  for (int step = 0; step < 50; ++step) {
    mj_step(m.get(), d.get());
  }
  EXPECT_LT(mju_norm3(d->qvel), MjTol(1e-3, 5e-3));
}

TEST_F(CoreConstraintTest, DenseContactMatchesSparseExact) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="5 5 .1"/>
      <body name="root" pos="0 0 .15">
        <freejoint/>
        <geom type="sphere" size=".16" mass="1"/>
        <body name="arm1" pos=".1 0 .05">
          <joint type="hinge" axis="0 1 0"/>
          <geom type="sphere" size=".12" mass=".5"/>
          <body name="welded_tip" pos=".05 0 0">
            <geom type="sphere" size=".09" mass=".1"/>
          </body>
        </body>
        <body name="arm2" pos=".15 0 .05">
          <joint type="hinge" axis="1 0 0"/>
          <joint type="hinge" axis="0 0 1"/>
          <geom type="sphere" size=".12" mass=".5"/>
        </body>
      </body>
      <body name="other" pos=".25 0 .15">
        <freejoint/>
        <geom type="sphere" size=".16" mass="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();

  for (int condim : {1, 3, 4, 6}) {
    for (int i = 0; i < m->ngeom; ++i) {
      m->geom_condim[i] = condim;
    }
    for (int cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
      m->opt.cone = cone;

      m->opt.jacobian = mjJAC_DENSE;
      MjDataPtr d_dense = MakeData(m);
      mj_forward(m.get(), d_dense.get());

      m->opt.jacobian = mjJAC_SPARSE;
      MjDataPtr d_sparse = MakeData(m);
      mj_forward(m.get(), d_sparse.get());

      ASSERT_GE(d_dense->ncon, 3) << "condim=" << condim << " cone=" << cone;
      ASSERT_EQ(d_dense->nefc, d_sparse->nefc);

      int nv = m->nv;
      std::vector<mjtNum> J_sparse_dense(d_sparse->nefc * nv, 0);
      for (int r = 0; r < d_sparse->nefc; ++r) {
        int adr = d_sparse->efc_J_rowadr[r];
        int nnz = d_sparse->efc_J_rownnz[r];
        for (int k = 0; k < nnz; ++k) {
          J_sparse_dense[r * nv + d_sparse->efc_J_colind[adr + k]] =
              d_sparse->efc_J[adr + k];
        }
      }

      for (int r = 0; r < d_dense->nefc; ++r) {
        EXPECT_EQ(d_dense->efc_type[r], d_sparse->efc_type[r]);
        EXPECT_EQ(d_dense->efc_id[r], d_sparse->efc_id[r]);
        EXPECT_EQ(d_dense->efc_pos[r], d_sparse->efc_pos[r]);
        EXPECT_EQ(d_dense->efc_margin[r], d_sparse->efc_margin[r]);
        EXPECT_EQ(d_dense->efc_frictionloss[r], d_sparse->efc_frictionloss[r]);
        for (int c = 0; c < nv; ++c) {
          EXPECT_EQ(d_dense->efc_J[r * nv + c], J_sparse_dense[r * nv + c])
              << "condim=" << condim << " cone=" << cone << " r=" << r
              << " c=" << c;
        }
      }
    }
  }
}

TEST_F(CoreConstraintTest, MakeImpedanceCacheAndEdgeCases) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.01"/>
    <worldbody>
      <geom name="floor" type="plane" size="5 5 .1"/>
      <body pos="0 0 .0996">
        <joint type="slide" axis="0 0 1" limited="true" range="0 1"
               frictionloss="0.5"/>
        <geom name="s1" type="sphere" size=".1" mass="1"
              friction="1 0.005 0.0001"/>
      </body>
      <body pos="0.5 0 .0992">
        <joint type="slide" axis="0 0 1" limited="true" range="0 1"
               frictionloss="0.5"/>
        <geom name="s2" type="sphere" size=".1" mass="1"
              friction="0.8 0.01 0.002"/>
      </body>
      <body pos="1.0 0 .095">
        <freejoint/>
        <geom name="s3" type="sphere" size=".1" mass="1"
              friction="1 0.005 0.0001"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();

  // spheres take priority over the floor so their solref/solimp are used as-is
  m->geom_priority[1] = m->geom_priority[2] = m->geom_priority[3] = 1;

  // s2 uses direct solref (stiffness, damping)
  m->geom_solref[2 * mjNREF + 0] = -1000;
  m->geom_solref[2 * mjNREF + 1] = -50;

  // vary solimp power, midpoint and width across geoms to exercise cache hits
  // and misses: s1 interior x <= mid (power 2), s2 interior x > mid (power 3),
  // s3 interior with power 1 (wider width keeps it unsaturated)
  m->geom_solimp[1 * mjNIMP + 3] = 0.5;
  m->geom_solimp[1 * mjNIMP + 4] = 2.0;
  m->geom_solimp[2 * mjNIMP + 3] = 0.3;
  m->geom_solimp[2 * mjNIMP + 4] = 3.0;
  m->geom_solimp[3 * mjNIMP + 2] = 0.01;
  m->geom_solimp[3 * mjNIMP + 3] = 0.5;
  m->geom_solimp[3 * mjNIMP + 4] = 1.0;

  // exercise condim 1/3/4/6, pyramidal/elliptic, euler/discrete, impratio 1 vs
  // 5, power 1/2/3, midpoint 0.5 vs 0.3, and direct vs standard solref
  for (int integrator : {mjINT_EULER, mjINT_DISCRETE}) {
    m->opt.integrator = integrator;
    for (int cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
      m->opt.cone = cone;
      for (mjtNum impratio : {1.0, 5.0}) {
        m->opt.impratio = impratio;
        for (int condim : {1, 3, 4, 6}) {
          for (int g = 0; g < m->ngeom; g++) {
            m->geom_condim[g] = condim;
          }
          MjDataPtr d = MakeData(m);
          mj_forward(m.get(), d.get());
          ASSERT_GE(d->ncon, 3);
          for (int i = 0; i < d->nefc; i++) {
            EXPECT_GT(d->efc_R[i], 0);
            EXPECT_NEAR(d->efc_D[i], 1 / d->efc_R[i],
                        MjTol(1e-12, 1e-5) * d->efc_D[i]);
            EXPECT_FALSE(mju_isBad(d->efc_diagA[i]));
            for (int k = 0; k < 4; k++) {
              EXPECT_FALSE(mju_isBad(d->efc_KBIP[4 * i + k]));
            }
          }
          if (condim > 1 && cone == mjCONE_ELLIPTIC) {
            for (int c = 0; c < d->ncon; c++) {
              const mjContact& con = d->contact[c];
              int adr = con.efc_address;
              ASSERT_GE(adr, 0);
              mjtNum f0 = con.friction[0];
              EXPECT_NEAR(con.mu,
                          f0 * mju_sqrt(d->efc_R[adr + 1] / d->efc_R[adr]),
                          MjTol(1e-12, 1e-5) * (1 + con.mu));
              for (int j = 1; j < condim - 1; j++) {
                mjtNum fj = con.friction[j];
                mjtNum expected_R = d->efc_R[adr + 1] * f0 * f0 / (fj * fj);
                EXPECT_NEAR(d->efc_R[adr + j + 1], expected_R,
                            MjTol(1e-11, 1e-4) * expected_R);
              }
            }
          }
        }
      }
    }
  }

  // mixed-sign solref on two consecutive constraints must NOT be cached: both
  // must emit warnings and fall back to default solref
  m->opt.integrator = mjINT_EULER;
  m->opt.cone = mjCONE_ELLIPTIC;
  for (int g = 0; g < m->ngeom; g++) {
    m->geom_condim[g] = 3;
  }
  m->geom_solref[1 * mjNREF + 0] = -10;
  m->geom_solref[1 * mjNREF + 1] = 1;
  m->geom_solref[2 * mjNREF + 0] = -10;
  m->geom_solref[2 * mjNREF + 1] = 1;
  m->geom_priority[1] = 1;
  m->geom_priority[2] = 1;

  EXPECT_CALL(mock_warning_handler,
              Warn(::testing::HasSubstr("mixed solref format")))
      .Times(2)
      .WillRepeatedly(::testing::Return());
  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());
}

// Compute expected dense contact Jacobian rows independently via
// mj_contactJacobian + mju_mulMatMat + friction cone construction.
std::vector<mjtNum> ReferenceContactRows(const mjModel* m, mjData* d,
                                         const mjContact* con, int ispyramid,
                                         int* nrows_out) {
  int nv = m->nv;
  int dim = con->dim;
  int issparse = mj_isSparse(m);

  std::vector<mjtNum> jac(6 * nv, 0);
  std::vector<mjtNum> jacdifp(3 * nv, 0);
  std::vector<mjtNum> jacdifr(3 * nv, 0);
  std::vector<mjtNum> jac1p(3 * nv, 0);
  std::vector<mjtNum> jac2p(3 * nv, 0);
  std::vector<mjtNum> jac1r(3 * nv, 0);
  std::vector<mjtNum> jac2r(3 * nv, 0);
  std::vector<int> chain(nv, 0);

  int NV = mj_contactJacobian(m, d, con, dim, jacdifp.data(), jacdifr.data(),
                              jac1p.data(), jac2p.data(), jac1r.data(),
                              jac2r.data(), issparse ? chain.data() : nullptr);
  if (NV == 0) {
    *nrows_out = 0;
    return {};
  }

  mju_mulMatMat(jac.data(), con->frame, jacdifp.data(), dim > 1 ? 3 : 1, 3, NV);
  if (dim > 3) {
    mju_mulMatMat(jac.data() + 3 * NV, con->frame, jacdifr.data(), dim - 3, 3,
                  NV);
  }

  int nrows = dim == 1 ? 1 : (ispyramid ? 2 * (dim - 1) : dim);
  *nrows_out = nrows;
  std::vector<mjtNum> rows(nrows * NV, 0);
  if (dim == 1 || !ispyramid) {
    std::memcpy(rows.data(), jac.data(), nrows * NV * sizeof(mjtNum));
  } else {
    for (int k = 1; k < dim; ++k) {
      mju_addScl(rows.data() + (2 * (k - 1)) * NV, jac.data(),
                 jac.data() + k * NV, con->friction[k - 1], NV);
      mju_addScl(rows.data() + (2 * (k - 1) + 1) * NV, jac.data(),
                 jac.data() + k * NV, -con->friction[k - 1], NV);
    }
  }

  // unpack sparse columns to dense [nrows x nv] for uniform comparison
  std::vector<mjtNum> dense(nrows * nv, 0);
  for (int r = 0; r < nrows; ++r) {
    for (int k = 0; k < NV; ++k) {
      int col = issparse ? chain[k] : k;
      dense[r * nv + col] = rows[r * NV + k];
    }
  }
  return dense;
}

// Verify instantiated contact metadata, sparse colind monotonicity, and
// Jacobian rows match ReferenceContactRows (identical up to the sign of zero).
void ExpectContactRowsMatchReference(const mjModel* m, const mjData* d,
                                     int conid,
                                     const std::vector<mjtNum>& ref_J,
                                     int ref_nrows, int jac_mode, int condim,
                                     int cone) {
  const mjContact& con = d->contact[conid];
  int nv = m->nv;
  int issparse = mj_isSparse(m);
  int ispyramid = mj_isPyramidal(m);
  int expected_type = condim == 1 ? mjCNSTR_CONTACT_FRICTIONLESS
                                  : (ispyramid ? mjCNSTR_CONTACT_PYRAMIDAL
                                               : mjCNSTR_CONTACT_ELLIPTIC);

  for (int r = 0; r < ref_nrows; ++r) {
    int er = con.efc_address + r;
    EXPECT_EQ(d->efc_type[er], expected_type);
    EXPECT_EQ(d->efc_id[er], conid);
    EXPECT_EQ(d->efc_frictionloss[er], 0);
    mjtNum expected_pos =
        (expected_type == mjCNSTR_CONTACT_ELLIPTIC && r > 0) ? 0 : con.dist;
    mjtNum expected_margin =
        (expected_type == mjCNSTR_CONTACT_ELLIPTIC && r > 0)
            ? 0
            : con.includemargin;
    EXPECT_EQ(d->efc_pos[er], expected_pos);
    EXPECT_EQ(d->efc_margin[er], expected_margin);

    std::vector<mjtNum> actual_row(nv, 0);
    if (issparse) {
      int adr = d->efc_J_rowadr[er];
      int nnz = d->efc_J_rownnz[er];
      ASSERT_GT(nnz, 0);
      for (int k = 0; k < nnz; ++k) {
        int col = d->efc_J_colind[adr + k];
        if (k > 0) {
          EXPECT_GT(col, d->efc_J_colind[adr + k - 1]);
        }
        actual_row[col] = d->efc_J[adr + k];
      }
    } else {
      std::memcpy(actual_row.data(), d->efc_J + er * nv, nv * sizeof(mjtNum));
    }

    // use EXPECT_EQ rather than memcmp because signed zeros (+0.0 vs -0.0)
    // can differ on zero entries
    for (int col = 0; col < nv; ++col) {
      EXPECT_EQ(actual_row[col], ref_J[r * nv + col])
          << "jac=" << jac_mode << " condim=" << condim << " cone=" << cone
          << " con=" << conid << " r=" << r << " col=" << col;
    }
  }
}

TEST_F(CoreConstraintTest, FlexContactMatchesReferenceExact) {
  // exercises rotateJac3 for flex contacts across dim 1, 3, 4, 6
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="5 5 .1"/>
      <flexcomp name="box" type="grid" count="2 2 2" spacing=".2 .2 .2"
        pos="0 0 .15" dim="3" mass="1">
        <contact condim="3"/>
      </flexcomp>
      <body name="probe1" pos="-.05 0 .28">
        <freejoint/>
        <geom type="sphere" size=".05" mass="1"/>
      </body>
      <body name="probe2" pos=".05 0 .28">
        <freejoint/>
        <geom type="sphere" size=".05" mass="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();

  for (int jac_mode : {mjJAC_SPARSE, mjJAC_DENSE}) {
    m->opt.jacobian = jac_mode;
    for (int condim : {1, 3, 4, 6}) {
      m->flex_condim[0] = condim;
      for (int g = 0; g < m->ngeom; ++g) {
        m->geom_condim[g] = condim;
      }
      for (int cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
        m->opt.cone = cone;
        MjDataPtr d = MakeData(m);
        mj_forward(m.get(), d.get());
        ASSERT_GT(d->ncon, 0);

        int ispyramid = mj_isPyramidal(m.get());
        int elem_contacts = 0;
        for (int c = 0; c < d->ncon; ++c) {
          const mjContact& con = d->contact[c];
          if (con.exclude) {
            continue;
          }
          if (con.elem[0] >= 0 || con.elem[1] >= 0) {
            elem_contacts++;
          }
          int ref_nrows = 0;
          std::vector<mjtNum> ref_J = ReferenceContactRows(
              m.get(), d.get(), &con, ispyramid, &ref_nrows);
          ASSERT_GT(ref_nrows, 0);
          ExpectContactRowsMatchReference(m.get(), d.get(), c, ref_J, ref_nrows,
                                          jac_mode, condim, cone);
        }
        EXPECT_GE(elem_contacts, 2);
      }
    }
  }
}

}  // namespace
}  // namespace mujoco
