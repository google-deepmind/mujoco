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

// Tests for engine/engine_collision_box.c.

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_collision_convex.h"
#include "src/engine/engine_collision_primitive.h"
#include "src/engine/engine_util_misc.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using MjCollisionBoxTest = MujocoTest;
using ::testing::NotNull;

static const char* const kBad0FilePath =
    "engine/testdata/collision_box/boxbox_bad0.xml";
static const char* const kBad1FilePath =
    "engine/testdata/collision_box/boxbox_bad1.xml";

TEST_F(MjCollisionBoxTest, BadContacts) {
  for (const char* local_path : {kBad0FilePath, kBad1FilePath}) {
    const std::string xml_path = GetTestDataFilePath(local_path);
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
    ASSERT_THAT(model, NotNull());
    mjData* data = mj_makeData(model);
    mj_forward(model, data);

    // allocate contact array and matching arrays
    std::vector<mjPreContact> precon(mjMAXCONPAIR);
    std::vector<int> match_raw(mjMAXCONPAIR);
    std::vector<int> match(data->ncon);

    int g1 = -1;
    int g2 = -1;
    for (int c = 0; c < data->ncon; c++) {
      mjContact* con = data->contact + c;
      int g1new = con->geom[0];
      int g2new = con->geom[1];

      // not box-box: skip
      if (model->geom_type[g1new] != mjGEOM_BOX ||
          model->geom_type[g2new] != mjGEOM_BOX) {
        continue;
      }

      // same geom pair: skip
      if (g1 == g1new && g2 == g2new) {
        continue;
      }

      g1 = g1new;
      g2 = g2new;

      // call low-level box-box collider
      int num =
          mjc_BoxBox(model, data, precon.data(), g1, g2, con->includemargin);

      // allocate and clear arrays marking already matched contacts
      mju_zeroInt(match_raw.data(), num);
      mju_zeroInt(match.data(), data->ncon);

      // loop over raw contacts, match with contact array using pos
      int nmatched = 0;
      for (int i = 0; i < num; i++) {
        for (int j = 0; j < data->ncon; j++) {
          if (!match[j] && precon[i].pos[0] == data->contact[j].pos[0] &&
              precon[i].pos[1] == data->contact[j].pos[1] &&
              precon[i].pos[2] == data->contact[j].pos[2]) {
            match_raw[i] = match[j] = 1;
            nmatched++;
          }
        }
      }

      // expect some contacts to have been removed
      EXPECT_EQ(nmatched, num) << local_path;

      // get box info
      const mjtNum* pos1 = data->geom_xpos + 3 * g1;
      const mjtNum* mat1 = data->geom_xmat + 9 * g1;
      const mjtNum* size1 = model->geom_size + 3 * g1;
      const mjtNum* pos2 = data->geom_xpos + 3 * g2;
      const mjtNum* mat2 = data->geom_xmat + 9 * g2;
      const mjtNum* size2 = model->geom_size + 3 * g2;
      mjtNum margin = mju_max(model->geom_margin[g1], model->geom_margin[g2]);

      // loop over raw contacts, find removed
      for (int i = 0; i < num; i++) {
        if (!match_raw[i]) {
          // === check if outside

          mjtNum sz1[3] = {size1[0] + margin, size1[1] + margin,
                           size1[2] + margin};
          mjtNum sz2[3] = {size2[0] + margin, size2[1] + margin,
                           size2[2] + margin};

          // relative distance (1%) outside of which contacts are removed
          static mjtNum kRatio = 1.01;

          // is the contact outside: 1, inside: -1, within the removal width: 0
          int out1 = mju_outsideBox(precon[i].pos, pos1, mat1, sz1, kRatio);
          int out2 = mju_outsideBox(precon[i].pos, pos2, mat2, sz2, kRatio);

          // mark as bad if outside one box and not inside the other box
          bool outside = (out1 == 1 && out2 != -1) || (out2 == 1 && out1 != -1);

          // expect that removed contact was outside
          EXPECT_TRUE(outside);
        }
      }
    }

    mj_deleteData(data);
    mj_deleteModel(model);
  }
}

static const char* const kDuplicateFilePath =
    "engine/testdata/collision_box/boxbox_duplicate.xml";

TEST_F(MjCollisionBoxTest, DuplicateContacts) {
  const std::string xml_path = GetTestDataFilePath(kDuplicateFilePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // allocate contact array and matching arrays
  std::vector<mjPreContact> precon(mjMAXCONPAIR);
  std::vector<int> match_raw(mjMAXCONPAIR);
  std::vector<int> match(data->ncon);

  int g1 = -1;
  int g2 = -1;
  for (int c = 0; c < data->ncon; c++) {
    mjContact* con = data->contact + c;
    int g1new = con->geom[0];
    int g2new = con->geom[1];

    // not box-box: skip
    if (model->geom_type[g1new] != mjGEOM_BOX ||
        model->geom_type[g2new] != mjGEOM_BOX) {
      continue;
    }

    // same geom pair: skip
    if (g1 == g1new && g2 == g2new) {
      continue;
    }

    g1 = g1new;
    g2 = g2new;

    // call low-level box-box collider
    int num =
        mjc_BoxBox(model, data, precon.data(), g1, g2, con->includemargin);

    // allocate and clear arrays marking already matched contacts
    mju_zeroInt(match_raw.data(), num);
    mju_zeroInt(match.data(), data->ncon);

    // loop over raw contacts, match with contact array using pos
    int nmatched = 0;
    for (int i = 0; i < num; i++) {
      for (int j = 0; j < data->ncon; j++) {
        if (!match[j] && precon[i].pos[0] == data->contact[j].pos[0] &&
            precon[i].pos[1] == data->contact[j].pos[1] &&
            precon[i].pos[2] == data->contact[j].pos[2]) {
          match_raw[i] = match[j] = 1;
          nmatched++;
        }
      }
    }

    // expect some contacts to have been removed
    EXPECT_EQ(nmatched, num);

    // loop over raw contacts, find removed
    for (int i = 0; i < num; i++) {
      if (!match_raw[i]) {
        // === check if duplicate
        bool duplicate = false;
        for (int j = 0; j < num; j++) {
          if (duplicate || i == j) {
            continue;
          }
          if (precon[i].pos[0] == precon[j].pos[0] &&
              precon[i].pos[1] == precon[j].pos[1] &&
              precon[i].pos[2] == precon[j].pos[2]) {
            duplicate = true;
          }
        }

        // expect that removed contact was duplicated
        EXPECT_TRUE(duplicate);
      }
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

static const char* const kDeepFilePath =
    "engine/testdata/collision_box/boxbox_deep.xml";

TEST_F(MjCollisionBoxTest, DeepPenetration) {
  const std::string xml_path = GetTestDataFilePath(kDeepFilePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // expect 4 contact
  EXPECT_EQ(data->ncon, 4);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(MjCollisionBoxTest, BoxSphere) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="plane" type="plane" size="0.05 0.05 0.001"/>
      <geom name="box" type="box" pos = "0 0 -0.025" size="0.05 0.05 .025"/>
      <body>
        <freejoint/>
        <geom name="sphere" type="sphere" mass="1" size="0.005"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  for (mjtNum z : {-.015, -.00501, -.005, -.00499, 0.0, 0.004}) {
    data->qpos[2] = z;
    mj_forward(model.get(), data.get());
    EXPECT_EQ(data->ncon, 2);
    EXPECT_THAT(data->contact[0].dist,
                MjNear(data->contact[1].dist, 1e-8, 1e-6));
  }
}

TEST_F(MjCollisionBoxTest, BoxBoxContactDistance) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="box" size="1 1 1"/>
      <geom type="box" size="1 1 1" pos="0 0 1.5"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  MjDataPtr data = MakeData(model);
  mj_kinematics(model.get(), data.get());
  mjPreContact precon[mjMAXCONPAIR];

  for (mjfCollision collision : {mjc_BoxBox, mjc_Convex}) {
    int n = collision(model.get(), data.get(), precon, 0, 1, 0.0);
    for (int i = 0; i < n; i++) {
      EXPECT_NEAR(precon[i].dist, -0.5, MjTol(1e-8, 1e-6));
    }
  }
}

TEST_F(MjCollisionBoxTest, ThinBoxNoSpuriousDeepContact) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <freejoint/>
        <geom type="box" size="0.047 0.032 0.00035" margin="1e-5" mass="2e-3"/>
      </body>
      <body>
        <freejoint/>
        <geom type="box" size="0.047 0.032 0.00035" margin="1e-5" mass="2e-3"/>
      </body>
    </worldbody>
    <keyframe>
      <key qpos="2.8218257713084481e-05 -0.013292121195706197 0.029256072037790268
                 0.84130787082244052 0.54054898504499482 0.0015299333749655413
                 0.0023495878161510389
                 -5.3521944872628108e-05 0.01427117523229726 0.029254416229696868
                 0.8413227886432264 -0.54053283716888822 0.00024615852936852315
                 -0.00039580009650069315"/>
    </keyframe>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);

  // thin boxes meeting edge-to-face, separated by ~15um, within the margin
  mj_resetDataKeyframe(model.get(), data.get(), 0);
  mj_forward(model.get(), data.get());
  mjtNum gap = mj_geomDistance(model.get(), data.get(), 0, 1, 0.1, nullptr);
  EXPECT_GT(gap, 0);

  // separated boxes: all margin-admitted contacts must have positive distance
  ASSERT_GT(data->ncon, 0);
  for (int i = 0; i < data->ncon; i++) {
    EXPECT_GT(data->contact[i].dist, 0);
  }

  // same, calling the collider directly (margin is the sum of geom margins)
  mjPreContact precon[mjMAXCONPAIR];
  int num = mjc_BoxBox(model.get(), data.get(), precon, 0, 1, 2e-5);
  for (int i = 0; i < num; i++) {
    EXPECT_GT(precon[i].dist, 0);
  }

  // push box 2 into box 1 along the normal: the deep contact is still reported
  mju_addToScl3(data->qpos + 7, data->contact[0].frame, -1e-4);
  mj_forward(model.get(), data.get());
  gap = mj_geomDistance(model.get(), data.get(), 0, 1, 0.1, nullptr);
  EXPECT_LT(gap, 0);
  ASSERT_GT(data->ncon, 0);
  mjtNum deepest = data->contact[0].dist;
  for (int i = 1; i < data->ncon; i++) {
    deepest = mju_min(deepest, data->contact[i].dist);
  }
  EXPECT_THAT(deepest, MjNear(gap, 1e-8, 1e-6));
}

TEST_F(MjCollisionBoxTest, ThinBoxShallowPenetration) {
  // thin boxes with zero margin, penetrating by ~100um: the deepest contact reaches the
  // separating-axis bound up to rounding, and must not be dropped by the depth filter;
  // the pose is written directly into mjData since the exact bits matter
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="box" size="0.1 0.1 0.1"/>
      <geom type="box" size="0.1 0.1 0.1"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mj_kinematics(model.get(), data.get());

  const mjtNum size1[3] = {0.00044359468518251132, 0.0019427705390657074,
                           0.00056716790015765711};
  const mjtNum size2[3] = {0.00062288175555326323, 0.0010906336314477707,
                           0.00030803275652456415};
  const mjtNum pos2[3] = {-0.0011513383735175778, -0.0013813387665010076,
                          -0.00043818094448460645};
  const mjtNum quat1[4] = {0.28720146798163904, -0.083561810000907483,
                           -0.51853459717827233, 0.80103346511099693};
  const mjtNum quat2[4] = {0.51588977358544519, 0.63445106644895233,
                           0.50962161111201376, -0.26761053656263345};
  mju_copy3(model->geom_size, size1);
  mju_copy3(model->geom_size + 3, size2);
  mju_zero3(data->geom_xpos);
  mju_copy3(data->geom_xpos + 3, pos2);
  mju_quat2Mat(data->geom_xmat, quat1);
  mju_quat2Mat(data->geom_xmat + 9, quat2);

  mjtNum gap = mj_geomDistance(model.get(), data.get(), 0, 1, 0.1, nullptr);
  EXPECT_LT(gap, 0);

  mjPreContact precon[mjMAXCONPAIR];
  int num = mjc_BoxBox(model.get(), data.get(), precon, 0, 1, 0);
  ASSERT_GT(num, 0);
  mjtNum deepest = precon[0].dist;
  for (int i = 1; i < num; i++) {
    deepest = mju_min(deepest, precon[i].dist);
  }
  EXPECT_THAT(deepest, MjNear(gap, 1e-8, 1e-6));
}

TEST_F(MjCollisionBoxTest, EdgeContactAtDepthBound) {
  // edge-edge contacts whose depth equals the separating-axis bound up to rounding: the
  // depth filter's slack must cover single-precision rounding or the whole manifold is
  // rejected; the pose is written directly into mjData since the exact bits matter
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom type="box" size="0.1 0.1 0.1"/>
      <geom type="box" size="0.1 0.1 0.1"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  mj_kinematics(model.get(), data.get());

  struct Config {
    mjtNum size1[3], size2[3], pos2[3], quat1[4], quat2[4];
  };
  const Config configs[2] = {
      {{0.018352361395955086, 0.038452208042144775, 0.047704100608825684},
       {0.026817722246050835, 0.0014398059574887156, 0.0082265362143516541},
       {-0.026325162500143051, 0.030753342434763908, 0.02338058315217495},
       {-0.90091776847839355, -0.42279955744743347, 0.097014322876930237,
        -0.013262901455163956},
       {0.23919239640235901, 0.056073460727930069, 0.91227829456329346,
        0.32770577073097229}},
      {{0.044790275394916534, 0.0057221869938075542, 0.0049537895247340202},
       {0.026496950536966324, 0.019804427400231361, 0.0057344711385667324},
       {0.0094044031575322151, 0.020275400951504707, -0.017358051612973213},
       {0.040385473519563675, 0.75189536809921265, -0.55430221557617188,
        0.35464280843734741},
       {-0.6716417670249939, 0.085770353674888611, 0.69683432579040527,
        -0.23656430840492249}}};

  for (const Config& config : configs) {
    mju_copy3(model->geom_size, config.size1);
    mju_copy3(model->geom_size + 3, config.size2);
    mju_zero3(data->geom_xpos);
    mju_copy3(data->geom_xpos + 3, config.pos2);
    mju_quat2Mat(data->geom_xmat, config.quat1);
    mju_quat2Mat(data->geom_xmat + 9, config.quat2);

    mjtNum gap = mj_geomDistance(model.get(), data.get(), 0, 1, 0.1, nullptr);
    EXPECT_LT(gap, 0);

    mjPreContact precon[mjMAXCONPAIR];
    int num = mjc_BoxBox(model.get(), data.get(), precon, 0, 1, 0);
    ASSERT_GT(num, 0);
    mjtNum deepest = precon[0].dist;
    for (int i = 1; i < num; i++) {
      deepest = mju_min(deepest, precon[i].dist);
    }
    EXPECT_THAT(deepest, MjNear(gap, 1e-8, 1e-6));
  }
}

}  // namespace
}  // namespace mujoco
