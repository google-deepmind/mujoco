// Copyright 2025 DeepMind Technologies Limited
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

// Tests for engine/engine_setconst.c.

#include <cstddef>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::std::string;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;


using SetConstTest = MujocoTest;

TEST_F(SetConstTest, AwakeActuatedJoint) {
  string xml = R"(
  <mujoco>
    <worldbody>
      <body name="B1" sleep="POLICY1">
        <joint name="J1" type="slide"/>
        <geom size=".1"/>
      </body>
      <body name="B2" sleep="POLICY2">
        <joint name="J2" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="J1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m;

  string sleep[] = {"auto", "never", "allowed", "init"};
  int tsp0[] = {mjSLEEP_AUTO_NEVER, mjSLEEP_NEVER, mjSLEEP_ALLOWED,
                mjSLEEP_INIT};
  int tsp1[] = {mjSLEEP_AUTO_ALLOWED, mjSLEEP_NEVER, mjSLEEP_ALLOWED,
                mjSLEEP_INIT};

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      string xml_copy = xml;
      size_t pos1 = xml_copy.find("POLICY1");
      xml_copy.replace(pos1, 7, sleep[i]);
      size_t pos2 = xml_copy.find("POLICY2");
      xml_copy.replace(pos2, 7, sleep[j]);
      m = LoadModelFromString(xml_copy.c_str(), error, sizeof(error));
      ASSERT_THAT(m.get(), NotNull()) << error;
      EXPECT_EQ(m->tree_sleep_policy[0], tsp0[i]);
      EXPECT_EQ(m->tree_sleep_policy[1], tsp1[j]);
    }
  }
}

TEST_F(SetConstTest, AwakeActuatedSite) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1">
        <joint name="J1" type="slide"/>
        <geom size=".1"/>
        <site name="S1"/>
      </body>
      <body name="B2">
        <joint name="J2" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <actuator>
      <general site="S1" gear="1 0 0 0 0 0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_NEVER);
  EXPECT_EQ(model->tree_sleep_policy[1], mjSLEEP_AUTO_ALLOWED);
}

TEST_F(SetConstTest, AwakeActuatedBody) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1">
        <joint name="J1" type="slide"/>
        <geom size=".1"/>
      </body>
      <body name="B2">
        <joint name="J2" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <actuator>
      <adhesion body="B1" ctrlrange="0 1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_NEVER);
  EXPECT_EQ(model->tree_sleep_policy[1], mjSLEEP_AUTO_ALLOWED);
}

TEST_F(SetConstTest, AwakeActuatedTendon) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="S1"/>
      <body name="B1">
        <joint name="J1" type="slide"/>
        <geom size=".1"/>
        <site name="S2"/>
      </body>
      <body name="B2">
        <joint name="J2" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <tendon>
      <spatial name="T1">
        <site site="S1"/>
        <site site="S2"/>
      </spatial>
    </tendon>
    <actuator>
      <motor tendon="T1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_NEVER);
  EXPECT_EQ(model->tree_sleep_policy[1], mjSLEEP_AUTO_ALLOWED);
}

TEST_F(SetConstTest, AwakeStiffTendonMultiTree) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1">
        <joint name="J1" type="slide"/>
        <geom size=".1"/>
        <site name="S1"/>
      </body>
      <body name="B2">
        <joint name="J2" type="slide"/>
        <geom size=".1"/>
        <site name="S2"/>
      </body>
    </worldbody>
    <tendon>
      <spatial name="T1" stiffness="1">
        <site site="S1"/>
        <site site="S2"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_NEVER);
  EXPECT_EQ(model->tree_sleep_policy[1], mjSLEEP_AUTO_NEVER);
}

TEST_F(SetConstTest, SleepyTendonSingleTree) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1">
        <joint name="J1" type="slide"/>
        <geom size=".1"/>
        <site name="S1"/>
        <body name="B2">
          <joint name="J2" type="slide"/>
          <geom size=".1"/>
          <site name="S2"/>
        </body>
      </body>
    </worldbody>
    <tendon>
      <spatial name="T1" stiffness="1">
        <site site="S1"/>
        <site site="S2"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_ALLOWED);
}

TEST_F(SetConstTest, SleepyTendonZeroStiffness) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1">
        <joint name="J1" type="slide"/>
        <geom size=".1"/>
        <site name="S1"/>
      </body>
      <body name="B2">
        <joint name="J2" type="slide"/>
        <geom size=".1"/>
        <site name="S2"/>
      </body>
    </worldbody>
    <tendon>
      <spatial name="T1" stiffness="0" damping="0">
        <site site="S1"/>
        <site site="S2"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_ALLOWED);
  EXPECT_EQ(model->tree_sleep_policy[1], mjSLEEP_AUTO_ALLOWED);
}

TEST_F(SetConstTest, TendonTreeId) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <!-- Static body 1: world -->
      <site name="S1"/>

      <!-- Static body 2: child of world -->
      <body name="B_static">
        <site name="S2"/>
      </body>

      <!-- Tree 1 -->
      <body name="B1_1">
        <joint name="J1_1" type="slide"/>
        <geom size=".1"/>
        <site name="S3"/>
        <body name="B1_2">
          <joint name="J1_2" type="slide"/>
          <geom size=".1"/>
          <site name="S4"/>
        </body>
      </body>

      <!-- Tree 2 -->
      <body name="B2_1">
        <joint name="J2_1" type="slide"/>
        <geom size=".1"/>
        <site name="S5"/>
      </body>

      <!-- Tree 3 -->
      <body name="B3_1">
        <joint name="J3_1" type="slide"/>
        <geom size=".1"/>
        <site name="S6"/>
      </body>
    </worldbody>

    <tendon>
      <!-- Tendon 1: Between static bodies -->
      <spatial name="T_static">
        <site site="S1"/>
        <site site="S2"/>
      </spatial>

      <!-- Tendon 2: Within Tree 1 -->
      <spatial name="T_tree1">
        <site site="S3"/>
        <site site="S4"/>
      </spatial>

      <!-- Tendon 3: Between Tree 1 and Tree 2 -->
      <spatial name="T_intertree12">
        <site site="S4"/>
        <site site="S5"/>
      </spatial>

      <!-- Tendon 4: Between Tree 1, 2 and 3 -->
      <spatial name="T_intertree123">
        <site site="S4"/>
        <site site="S5"/>
        <site site="S6"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  int t_static_id = mj_name2id(model.get(), mjOBJ_TENDON, "T_static");
  int t_tree1_id = mj_name2id(model.get(), mjOBJ_TENDON, "T_tree1");
  int t_intertree12_id = mj_name2id(model.get(), mjOBJ_TENDON, "T_intertree12");
  int t_intertree123_id =
      mj_name2id(model.get(), mjOBJ_TENDON, "T_intertree123");
  int b1_1_treeid =
      model->body_treeid[mj_name2id(model.get(), mjOBJ_BODY, "B1_1")];
  int b2_1_treeid =
      model->body_treeid[mj_name2id(model.get(), mjOBJ_BODY, "B2_1")];

  // Tendon 1: Not associated with any tree
  EXPECT_EQ(model->tendon_treenum[t_static_id], 0);
  EXPECT_EQ(model->tendon_treeid[2*t_static_id], -1);
  EXPECT_EQ(model->tendon_treeid[2*t_static_id+1], -1);

  // Tendon 2: Should be in Tree 1
  EXPECT_EQ(model->tendon_treenum[t_tree1_id], 1);
  EXPECT_EQ(model->tendon_treeid[2*t_tree1_id], b1_1_treeid);
  EXPECT_EQ(model->tendon_treeid[2*t_tree1_id+1], -1);
  EXPECT_GE(model->tendon_treeid[2*t_tree1_id], 0);

  // Tendon 3: Spans two trees (Tree 1 and Tree 2)
  EXPECT_EQ(model->tendon_treenum[t_intertree12_id], 2);
  EXPECT_EQ(model->tendon_treeid[2*t_intertree12_id], b1_1_treeid);
  EXPECT_EQ(model->tendon_treeid[2*t_intertree12_id+1], b2_1_treeid);

  // Tendon 4: Spans three trees (Tree 1, 2 and 3)
  EXPECT_EQ(model->tendon_treenum[t_intertree123_id], 3);
  EXPECT_EQ(model->tendon_treeid[2*t_intertree123_id], b1_1_treeid);
  EXPECT_EQ(model->tendon_treeid[2*t_intertree123_id+1], b2_1_treeid);
  // The third tree ID is not stored in tendon_treeid
}

TEST_F(SetConstTest, SleepingNotAllowed) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>

      <!-- Tree 0 -->
      <body name="B1_1">
        <joint name="J1_1" type="slide"/>
        <geom size=".1"/>
        <site name="S3"/>
        <body name="B1_2">
          <joint name="J1_2" type="slide"/>
          <geom size=".1"/>
          <site name="S4"/>
        </body>
      </body>

      <!-- Tree 1: forbidden user sleep override -->
      <body name="B2_1" sleep="allowed">
        <joint name="J2_1" type="slide"/>
        <geom size=".1"/>
        <site name="S5"/>
      </body>

      <!-- Tree 2 -->
      <body name="B3_1">
        <joint name="J3_1" type="slide"/>
        <geom size=".1"/>
        <site name="S6"/>
      </body>
    </worldbody>

    <tendon>
      <!-- Tendon 0: Between Tree 0, 1 and 2 -->
      <spatial name="T_intertree123">
        <site site="S4"/>
        <site site="S5"/>
        <site site="S6"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model.get(), IsNull()) << error;
  EXPECT_THAT(string(error), HasSubstr(
              "tree 1 connected to tendon 0 which spans more than 2 trees, "
              "sleeping not allowed"));
}


TEST_F(SetConstTest, DofLength) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1">
        <joint name="S1" type="slide"/>
        <geom size="2"/>
      </body>
      <body name="B2">
        <joint name="H1" type="hinge"/>
        <geom size="3"/>
      </body>
      <body name="B3">
        <joint name="BA1" type="ball"/>
        <geom size="4"/>
      </body>
      <body name="B4">
        <joint name="F1" type="free"/>
        <geom size="5"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  mjtNum tol = 1e-5;

  // B1: Slider
  EXPECT_EQ(model->dof_length[0], 1);

  // B2: Hinge
  EXPECT_NEAR(model->dof_length[1], 3, tol);

  // B3: Ball
  EXPECT_NEAR(model->dof_length[2], 4, tol);
  EXPECT_NEAR(model->dof_length[3], 4, tol);
  EXPECT_NEAR(model->dof_length[4], 4, tol);

  // B4: Free
  EXPECT_EQ(model->dof_length[5], 1);
  EXPECT_EQ(model->dof_length[6], 1);
  EXPECT_EQ(model->dof_length[7], 1);
  EXPECT_NEAR(model->dof_length[8], 5, tol);
  EXPECT_NEAR(model->dof_length[9], 5, tol);
  EXPECT_NEAR(model->dof_length[10], 5, tol);
}

TEST_F(SetConstTest, BodySameframeRecomputed) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1" simple="false">
        <joint type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d(mj_makeData(m.get()));

  int b = mj_name2id(m.get(), mjOBJ_BODY, "B1");

  // initially sameframe should be BODY (ipos=0, iquat=identity)
  EXPECT_EQ(m->body_sameframe[b], mjSAMEFRAME_BODY);

  // perturb body_ipos, call mj_setConst
  m->body_ipos[3*b+0] = 1.0;
  mj_setConst(m.get(), d.get());
  EXPECT_EQ(m->body_sameframe[b], mjSAMEFRAME_BODYROT);

  // also perturb body_iquat
  m->body_iquat[4*b+0] = 0.5;
  m->body_iquat[4*b+1] = 0.5;
  m->body_iquat[4*b+2] = 0.5;
  m->body_iquat[4*b+3] = 0.5;
  mj_setConst(m.get(), d.get());
  EXPECT_EQ(m->body_sameframe[b], mjSAMEFRAME_NONE);

  // restore to identity, should go back to BODY
  m->body_ipos[3*b+0] = 0;
  m->body_iquat[4*b+0] = 1;
  m->body_iquat[4*b+1] = 0;
  m->body_iquat[4*b+2] = 0;
  m->body_iquat[4*b+3] = 0;
  mj_setConst(m.get(), d.get());
  EXPECT_EQ(m->body_sameframe[b], mjSAMEFRAME_BODY);
}

TEST_F(SetConstTest, GeomSameframeRecomputed) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1">
        <joint type="slide"/>
        <geom name="G1" size=".1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d(mj_makeData(m.get()));

  int g = mj_name2id(m.get(), mjOBJ_GEOM, "G1");

  // initially sameframe should be BODY
  EXPECT_EQ(m->geom_sameframe[g], mjSAMEFRAME_BODY);

  // perturb geom_pos
  m->geom_pos[3*g+1] = 0.5;
  mj_setConst(m.get(), d.get());
  EXPECT_EQ(m->geom_sameframe[g], mjSAMEFRAME_BODYROT);

  // restore, should go back to BODY
  m->geom_pos[3*g+1] = 0;
  mj_setConst(m.get(), d.get());
  EXPECT_EQ(m->geom_sameframe[g], mjSAMEFRAME_BODY);
}

TEST_F(SetConstTest, SiteSameframeRecomputed) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1">
        <joint type="slide"/>
        <geom size=".1"/>
        <site name="S1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d(mj_makeData(m.get()));

  int s = mj_name2id(m.get(), mjOBJ_SITE, "S1");

  // initially sameframe should be BODY
  EXPECT_EQ(m->site_sameframe[s], mjSAMEFRAME_BODY);

  // perturb site_pos
  m->site_pos[3*s+2] = 0.3;
  mj_setConst(m.get(), d.get());
  EXPECT_EQ(m->site_sameframe[s], mjSAMEFRAME_BODYROT);

  // restore
  m->site_pos[3*s+2] = 0;
  mj_setConst(m.get(), d.get());
  EXPECT_EQ(m->site_sameframe[s], mjSAMEFRAME_BODY);
}

TEST_F(SetConstTest, SameframeKinematicsCorrect) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1" pos="1 0 0" simple="false">
        <joint type="slide" axis="1 0 0"/>
        <geom name="G1" size=".1"/>
        <site name="S1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d(mj_makeData(m.get()));

  int b = mj_name2id(m.get(), mjOBJ_BODY, "B1");
  int g = mj_name2id(m.get(), mjOBJ_GEOM, "G1");

  // perturb body inertial offset, breaking sameframe
  m->body_ipos[3*b+1] = 0.5;
  mj_setConst(m.get(), d.get());
  EXPECT_EQ(m->body_sameframe[b], mjSAMEFRAME_BODYROT);

  // run forward kinematics, check that xipos != xpos
  mj_forward(m.get(), d.get());
  EXPECT_NEAR(d->xipos[3*b+1], 0.5, MjTol(1e-10, 1e-6));
  EXPECT_NEAR(d->xpos[3*b+1], 0.0, MjTol(1e-10, 1e-6));

  // perturb geom_pos, check geom global position
  m->geom_pos[3*g+2] = 0.3;
  mj_setConst(m.get(), d.get());
  EXPECT_NE(m->geom_sameframe[g], mjSAMEFRAME_BODY);
  mj_forward(m.get(), d.get());
  EXPECT_NEAR(d->geom_xpos[3*g+2], 0.3, MjTol(1e-10, 1e-6));
}

TEST_F(SetConstTest, SimpleBodyLostSameframeError) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="B1">
        <joint type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << error;
  MjDataPtr d(mj_makeData(m.get()));

  int b = mj_name2id(m.get(), mjOBJ_BODY, "B1");

  // confirm body is compiled as simple
  EXPECT_GT(m->body_simple[b], 0);

  // perturb body_ipos, breaking sameframe; calling mj_setConst should fail
  m->body_ipos[3*b+0] = 1.0;

  std::string err = MjuErrorMessageFrom(mj_setConst)(m.get(), d.get());
  EXPECT_THAT(err, HasSubstr("body 1 is compiled as simple but "
                             "sameframe no longer holds"));
}

}  // namespace
}  // namespace mujoco
