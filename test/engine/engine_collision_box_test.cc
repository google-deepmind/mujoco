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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"
#include "src/engine/engine_collision_primitive.h"
#include "src/engine/engine_util_misc.h"


namespace mujoco {
namespace {

using MjCollisionBoxTest = MujocoTest;
using ::testing::NotNull;
using ::testing::DoubleNear;

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
    mj_markStack(data);
    mjContact* con_raw = (mjContact*) mj_stackAllocByte(
        data, mjMAXCONPAIR * sizeof(mjContact), alignof(mjContact));
    int* match_raw = mj_stackAllocInt(data, mjMAXCONPAIR);
    int* match = mj_stackAllocInt(data, data->ncon);

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
      int num = mjc_BoxBox(model, data, con_raw, g1, g2, con->includemargin);

      // allocate and clear arrays marking already matched contacts
      mju_zeroInt(match_raw, num);
      mju_zeroInt(match, data->ncon);

      // loop over raw contacts, match with contact array using pos
      int nmatched = 0;
      for (int i = 0; i < num; i++) {
        for (int j = 0; j < data->ncon; j++) {
          if (!match[j] &&
              con_raw[i].pos[0] == data->contact[j].pos[0] &&
              con_raw[i].pos[1] == data->contact[j].pos[1] &&
              con_raw[i].pos[2] == data->contact[j].pos[2]) {
            match_raw[i] = match[j] = 1;
            nmatched++;
          }
        }
      }

      // expect some contacts to have been removed
      EXPECT_LT(nmatched, num) << local_path;

      // get box info
      const mjtNum* pos1 =  data->geom_xpos  + 3 * g1;
      const mjtNum* mat1 =  data->geom_xmat  + 9 * g1;
      const mjtNum* size1 = model->geom_size + 3 * g1;
      const mjtNum* pos2 =  data->geom_xpos  + 3 * g2;
      const mjtNum* mat2 =  data->geom_xmat  + 9 * g2;
      const mjtNum* size2 = model->geom_size + 3 * g2;
      mjtNum margin = mju_max(model->geom_margin[g1], model->geom_margin[g2]);

      // loop over raw contacts, find removed
      for (int i = 0; i < num; i++) {
        if (!match_raw[i]) {
          // === check if outside

          mjtNum sz1[3] = {size1[0]+margin, size1[1]+margin, size1[2]+margin};
          mjtNum sz2[3] = {size2[0]+margin, size2[1]+margin, size2[2]+margin};

          // relative distance (1%) outside of which contacts are removed
          static mjtNum kRatio = 1.01;

          // is the contact outside: 1, inside: -1, within the removal width: 0
          int out1 = mju_outsideBox(con_raw[i].pos, pos1, mat1, sz1, kRatio);
          int out2 = mju_outsideBox(con_raw[i].pos, pos2, mat2, sz2, kRatio);

          // mark as bad if outside one box and not inside the other box
          bool outside = (out1 == 1 && out2 != -1) || (out2 == 1 && out1 != -1);

          // expect that removed contact was outside
          EXPECT_TRUE(outside);
        }
      }
    }

    mj_freeStack(data);
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
  mj_markStack(data);
  mjContact* con_raw = (mjContact*) mj_stackAllocByte(
      data, mjMAXCONPAIR * sizeof(mjContact), alignof(mjContact));
  int* match_raw = mj_stackAllocInt(data, mjMAXCONPAIR);
  int* match = mj_stackAllocInt(data, data->ncon);


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
    int num = mjc_BoxBox(model, data, con_raw, g1, g2, con->includemargin);

    // allocate and clear arrays marking already matched contacts
    mju_zeroInt(match_raw, num);
    mju_zeroInt(match, data->ncon);

    // loop over raw contacts, match with contact array using pos
    int nmatched = 0;
    for (int i = 0; i < num; i++) {
      for (int j = 0; j < data->ncon; j++) {
        if (!match[j] &&
            con_raw[i].pos[0] == data->contact[j].pos[0] &&
            con_raw[i].pos[1] == data->contact[j].pos[1] &&
            con_raw[i].pos[2] == data->contact[j].pos[2]) {
          match_raw[i] = match[j] = 1;
          nmatched++;
        }
      }
    }

    // expect some contacts to have been removed
    EXPECT_LT(nmatched, num);

    // loop over raw contacts, find removed
    for (int i = 0; i < num; i++) {
      if (!match_raw[i]) {
        // === check if duplicate
        bool duplicate = false;
        for (int j = 0; j < num; j++) {
          if (duplicate || i == j) {
            continue;
          }
          if (con_raw[i].pos[0] == con_raw[j].pos[0] &&
              con_raw[i].pos[1] == con_raw[j].pos[1] &&
              con_raw[i].pos[2] == con_raw[j].pos[2]) {
            duplicate = true;
          }
        }

        // expect that removed contact was duplicated
        EXPECT_TRUE(duplicate);
      }
    }
  }

  mj_freeStack(data);
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
  EXPECT_EQ(data->ncon ,4);

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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);

  for (mjtNum z : {-.015, -.00501, -.005, -.00499, 0.0, 0.004}) {
    data->qpos[2] = z;
    mj_forward(model, data);
    EXPECT_EQ(data->ncon, 2);
    EXPECT_THAT(data->contact[0].dist, DoubleNear(data->contact[1].dist, 1e-8));
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}


}  // namespace
}  // namespace mujoco
