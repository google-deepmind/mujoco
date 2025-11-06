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
using ::testing::DoubleNear;
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
  mjModel* m;

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
      ASSERT_THAT(m, NotNull()) << error;
      EXPECT_EQ(m->tree_sleep_policy[0], tsp0[i]);
      EXPECT_EQ(m->tree_sleep_policy[1], tsp1[j]);
      mj_deleteModel(m);
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
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_NEVER);
  EXPECT_EQ(model->tree_sleep_policy[1], mjSLEEP_AUTO_ALLOWED);

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_NEVER);
  EXPECT_EQ(model->tree_sleep_policy[1], mjSLEEP_AUTO_ALLOWED);

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_NEVER);
  EXPECT_EQ(model->tree_sleep_policy[1], mjSLEEP_AUTO_ALLOWED);

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_NEVER);
  EXPECT_EQ(model->tree_sleep_policy[1], mjSLEEP_AUTO_NEVER);

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_ALLOWED);

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  EXPECT_EQ(model->tree_sleep_policy[0], mjSLEEP_AUTO_ALLOWED);
  EXPECT_EQ(model->tree_sleep_policy[1], mjSLEEP_AUTO_ALLOWED);

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  int t_static_id = mj_name2id(model, mjOBJ_TENDON, "T_static");
  int t_tree1_id = mj_name2id(model, mjOBJ_TENDON, "T_tree1");
  int t_intertree12_id = mj_name2id(model, mjOBJ_TENDON, "T_intertree12");
  int t_intertree123_id = mj_name2id(model, mjOBJ_TENDON, "T_intertree123");

  int b1_1_treeid = model->body_treeid[mj_name2id(model, mjOBJ_BODY, "B1_1")];
  int b2_1_treeid = model->body_treeid[mj_name2id(model, mjOBJ_BODY, "B2_1")];

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

  mj_deleteModel(model);
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
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model, IsNull()) << error;
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
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  mjtNum tol = 1e-5;

  // B1: Slider
  EXPECT_EQ(model->dof_length[0], 1);

  // B2: Hinge
  EXPECT_THAT(model->dof_length[1], DoubleNear(3, tol));

  // B3: Ball
  EXPECT_THAT(model->dof_length[2], DoubleNear(4, tol));
  EXPECT_THAT(model->dof_length[3], DoubleNear(4, tol));
  EXPECT_THAT(model->dof_length[4], DoubleNear(4, tol));

  // B4: Free
  EXPECT_EQ(model->dof_length[5], 1);
  EXPECT_EQ(model->dof_length[6], 1);
  EXPECT_EQ(model->dof_length[7], 1);
  EXPECT_THAT(model->dof_length[8], DoubleNear(5, tol));
  EXPECT_THAT(model->dof_length[9], DoubleNear(5, tol));
  EXPECT_THAT(model->dof_length[10], DoubleNear(5, tol));

  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
