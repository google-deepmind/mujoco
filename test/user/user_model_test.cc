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

// Tests for user/user_model.cc.

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/str_format.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "src/xml/xml_numeric_format.h"
#include "test/compare_model.h"
#include "test/compare_spec.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsEmpty;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::Pointwise;

static std::vector<mjtNum> GetRow(const mjtNum* array, int ncolumn, int row) {
  return std::vector<mjtNum>(array + ncolumn * row,
                             array + ncolumn * (row + 1));
}

// ----------------------------- test mjCModel  --------------------------------

using UserModelTest = MujocoTest;

// clarify the semantic of body_rootid and body_weldid
TEST_F(UserModelTest, WeldRootID) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="1">
        <joint/>
        <geom size="1"/>
        <body name="2">
          <joint/>
          <geom size="1"/>
        </body>
      </body>

      <body name="3">
        <joint/>
        <geom size="1"/>
        <body name="4">
          <geom size="1"/>
        </body>
      </body>

      <body name="5">
        <geom size="1"/>
        <body name="6">
          <geom size="1"/>
        </body>
      </body>

      <body name="7" mocap="true">
        <geom size="1"/>
        <body name="8">
          <geom size="1"/>
        </body>
      </body>

      <body name="9" mocap="true">
        <geom size="1"/>
        <body name="10">
          <joint/>
          <geom size="1"/>
        </body>
      </body>
     </worldbody>
   </mujoco>
   )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model.get(), NotNull());

  EXPECT_THAT(AsVector(model->body_rootid, model->nbody),
              ElementsAre(0, 1, 1, 3, 3, 5, 5, 7, 7, 9, 9));
  // mocap bodies (7, 9) are their own weld roots,
  // inherited by static children (8)
  EXPECT_THAT(AsVector(model->body_weldid, model->nbody),
              ElementsAre(0, 1, 2, 3, 3, 0, 0, 7, 7, 9, 10));
}

TEST_F(UserModelTest, RepeatedNames) {
  static constexpr char xml[] = R"(
   <mujoco>
     <worldbody>
       <body name="body1">
         <joint axis="0 1 0" name="joint1"/>
         <geom size="1" name="geom1"/>
         <geom size="1" name="geom1"/>
        </body>
      </worldbody>
    </mujoco>)";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model.get(), IsNull());
  EXPECT_THAT(error.data(), HasSubstr("repeated name 'geom1' in geom"));
}

TEST_F(UserModelTest, SameFrame) {
  static constexpr char xml[] = R"(
   <mujoco>
     <default>
      <geom type="box" size="1 2 3"/>
     </default>

     <worldbody>
       <body name="body1">
         <geom name="none"       mass="0" pos="1 1 1" euler="10 10 10"/>
         <geom name="body"       mass="0"/>
         <geom name="inertia"    mass="1" pos="3 2 1" euler="20 30 40"/>
         <geom name="bodyrot"    mass="0" pos="1 1 1"/>
         <geom name="inertiarot" mass="0" euler="20 30 40"/>
        </body>
      </worldbody>
    </mujoco>)";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull()) << error.data();
  EXPECT_EQ(model->geom_sameframe[0], mjSAMEFRAME_NONE);
  EXPECT_EQ(model->geom_sameframe[1], mjSAMEFRAME_BODY);
  EXPECT_EQ(model->geom_sameframe[2], mjSAMEFRAME_INERTIA);
  EXPECT_EQ(model->geom_sameframe[3], mjSAMEFRAME_BODYROT);
  EXPECT_EQ(model->geom_sameframe[4], mjSAMEFRAME_INERTIAROT);

  // make data, get geom_xpos
  MjDataPtr data = MakeData(model);
  mj_kinematics(model.get(), data.get());
  auto geom_xpos = AsVector(data->geom_xpos, model->ngeom * 3);
  auto geom_xmat = AsVector(data->geom_xmat, model->ngeom * 9);

  // set all geom_sameframe to 0, call kinematics again
  for (int i = 0; i < model->ngeom; i++) {
    model->geom_sameframe[i] = mjSAMEFRAME_NONE;
  }
  mj_resetData(model.get(), data.get());
  mj_kinematics(model.get(), data.get());
  auto geom_xpos2 = AsVector(data->geom_xpos, model->ngeom * 3);
  auto geom_xmat2 = AsVector(data->geom_xmat, model->ngeom * 9);

  // expect them to be equal
  constexpr double eps = 1e-6;
  EXPECT_THAT(geom_xpos, Pointwise(MjNear(eps, eps), geom_xpos2));
  EXPECT_THAT(geom_xmat, Pointwise(MjNear(eps, eps), geom_xmat2));
}

TEST_F(UserModelTest, ActuatorSparsity) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="a"/>
        <body>
          <geom size="1"/>
          <joint name="b"/>
        </body>
      </body>
    </worldbody>
    <actuator>
      <motor joint="a"/>
      <motor joint="b"/>
    </actuator>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_EQ(m->nJmom, 2);
}

TEST_F(UserModelTest, FixedTendonSparsity) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size=".1"/>
        <joint name="0"/>
      </body>
      <body pos="1 0 0">
        <geom size=".1"/>
        <joint name="1"/>
      </body>
      <body pos="2 0 0">
        <geom size=".1"/>
        <joint name="2"/>
      </body>
    </worldbody>

    <tendon>
      <fixed>
        <joint coef="3" joint="2"/>
        <joint coef="2" joint="1"/>
        <joint coef="1" joint="0"/>
      </fixed>
    </tendon>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_THAT(m.get(), NotNull());

  EXPECT_EQ(m->nJten, 3);
  EXPECT_EQ(m->ten_J_rownnz[0], 3);
  EXPECT_EQ(m->ten_J_rowadr[0], 0);
  EXPECT_EQ(m->wrap_type[m->tendon_adr[0]], mjWRAP_JOINT);

  int rowadr = m->ten_J_rowadr[0];
  int* colind = m->ten_J_colind + rowadr;
  EXPECT_THAT(std::vector<int>(colind, colind + 3), ElementsAre(0, 1, 2));
}

TEST_F(UserModelTest, NestedZeroMassBodiesOK) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <freejoint/>
        <body>
          <body>
            <body>
              <geom size="1"/>
            </body>
          </body>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
}

TEST_F(UserModelTest, NestedZeroMassBodiesWithJointOK) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <freejoint/>
        <body>
          <body>
            <body>
              <joint/>
              <geom size="1"/>
            </body>
            <body>
              <geom size="1"/>
            </body>
          </body>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
}

TEST_F(UserModelTest, NestedZeroMassBodiesFail) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <body name="bad">
          <freejoint/>
          <body>
            <body>
            </body>
          </body>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), IsNull());
  EXPECT_THAT(
      error,
      HasSubstr(
          "mass and inertia of moving bodies must be larger than mjMINVAL"));
  EXPECT_THAT(error, HasSubstr("Element name 'bad'"));
}

// settotalmass is deprecated: it still scales the masses, with a warning which
// says what to do instead
TEST_F(UserModelTest, SetTotalMassDeprecated) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint/>
        <geom size=".1" pos=".2 0 0"/>
        <body pos=".4 0 0">
          <joint axis="0 1 0"/>
          <inertial pos=".1 0 0" mass="2" diaginertia=".1 .2 .3"/>
          <geom type="capsule" size=".05" fromto="0 0 0 .3 0 0"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, nullptr, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();

  // without the attribute there is no warning
  mjModel* unscaled = mj_compile(spec, nullptr);
  ASSERT_THAT(unscaled, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(mjs_numWarnings(spec), 0);

  // with it the masses are scaled as before, and compilation warns
  mock_warning_handler.ExpectWarnings("settotalmass");
  spec->compiler.settotalmass = 8;
  mjModel* scaled = mj_compile(spec, nullptr);
  ASSERT_THAT(scaled, NotNull()) << mjs_getError(spec);
  EXPECT_NEAR(mj_getTotalmass(scaled), 8, MjTol(1e-12, 1e-5));
  ASSERT_EQ(mjs_numWarnings(spec), 1);
  EXPECT_THAT(mjs_getWarning(spec, 0), HasSubstr("deprecated"));
  EXPECT_THAT(mjs_getWarning(spec, 0), HasSubstr("mj_setTotalmass"));

  // what the warning says to do instead gives the same model
  mj_setTotalmass(unscaled, 8);
  mjData* data = mj_makeData(unscaled);
  mj_setConst(unscaled, data);
  std::string field;
  EXPECT_LE(CompareModel(scaled, unscaled, field), MjTol(1e-12, 1e-5)) << field;

  mj_deleteData(data);
  mj_deleteModel(scaled);
  mj_deleteModel(unscaled);
  mj_deleteSpec(spec);
}

TEST_F(UserModelTest, ConvexHullForCollisionMeshes) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh_no_hull" vertex="0 0 0  1 0 0  0 1 0  0 0 1"
            face="0 2 1  0 1 3  0 3 2  1 2 3"/>
      <mesh name="mesh_with_hull_contype" vertex="0 0 0  1 0 0  0 1 0  0 0 1"
            face="0 2 1  0 1 3  0 3 2  1 2 3"/>
      <mesh name="mesh_with_hull_conaffinity" vertex="0 0 0  1 0 0  0 1 0  0 0 1"
            face="0 2 1  0 1 3  0 3 2  1 2 3"/>
    </asset>
    <worldbody>
      <geom name="geom_no_hull" type="mesh" mesh="mesh_no_hull" contype="0" conaffinity="0"/>
      <geom name="geom_with_hull_contype" type="mesh" mesh="mesh_with_hull_contype" contype="1"/>
      <geom name="geom_with_hull_conaffinity" type="mesh" mesh="mesh_with_hull_conaffinity" conaffinity="1"/>
    </worldbody>
  </mujoco>)";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull()) << error.data();

  int no_hull_id = mj_name2id(model.get(), mjOBJ_MESH, "mesh_no_hull");
  int with_hull_contype_id =
      mj_name2id(model.get(), mjOBJ_MESH, "mesh_with_hull_contype");
  int with_hull_conaffinity_id =
      mj_name2id(model.get(), mjOBJ_MESH, "mesh_with_hull_conaffinity");

  EXPECT_NE(no_hull_id, -1);
  EXPECT_NE(with_hull_contype_id, -1);
  EXPECT_NE(with_hull_conaffinity_id, -1);

  // mesh_no_hull should not have a convex hull.
  EXPECT_EQ(model->mesh_graphadr[no_hull_id], -1);

  // mesh_with_hull_contype should have a convex hull.
  EXPECT_NE(model->mesh_graphadr[with_hull_contype_id], -1);

  // mesh_with_hull_conaffinity should have a convex hull.
  EXPECT_NE(model->mesh_graphadr[with_hull_conaffinity_id], -1);
}

TEST_F(UserModelTest, MeshExtremaValidIndices) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="test_mesh" vertex="0 0 0  1 0 0  0 1 0  0 0 1"
            face="0 2 1  0 1 3  0 3 2  1 2 3"/>
    </asset>
    <worldbody>
      <geom name="test_geom" type="mesh" mesh="test_mesh"/>
    </worldbody>
  </mujoco>)";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull()) << error.data();

  int mesh_id = mj_name2id(model.get(), mjOBJ_MESH, "test_mesh");
  EXPECT_NE(mesh_id, -1);
  EXPECT_NE(model->mesh_graphadr[mesh_id], -1);

  const float* verts = model->mesh_vert + 3 * model->mesh_vertadr[mesh_id];
  const int* graph = model->mesh_graph + model->mesh_graphadr[mesh_id];
  const int* vert_globalid = graph + 2 + graph[0];

  int k = 0;
  for (int cx = -1; cx <= 1; cx++) {
    for (int cy = -1; cy <= 1; cy++) {
      for (int cz = -1; cz <= 1; cz++) {
        int v_idx = model->mesh_extrema[mesh_id * 27 + k];
        EXPECT_GE(v_idx, 0);
        EXPECT_LT(v_idx, graph[0]);

        int global_id = vert_globalid[v_idx];
        float max_dot = verts[3 * global_id + 0] * cx +
                        verts[3 * global_id + 1] * cy +
                        verts[3 * global_id + 2] * cz;

        // verify no other vertex yields a strictly greater dot product.
        for (int i = 0; i < graph[0]; i++) {
          int other_gid = vert_globalid[i];
          float dot = verts[3 * other_gid + 0] * cx +
                      verts[3 * other_gid + 1] * cy +
                      verts[3 * other_gid + 2] * cz;
          EXPECT_LE(dot, max_dot);
        }
        k++;
      }
    }
  }
}

TEST_F(UserModelTest, ConvexHullForPairCollisionMeshes) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="mesh" vertex="0 0 0  1 0 0  0 1 0  0 0 1"
            face="0 2 1  0 1 3  0 3 2  1 2 3"/>
    </asset>
    <worldbody>
      <geom name="geom1" type="sphere" size="1"/>
      <geom name="geom_mesh" type="mesh" mesh="mesh" contype="0" conaffinity="0"/>
    </worldbody>
    <contact>
      <pair name="hello" geom1="geom1" geom2="geom_mesh"/>
    </contact>
  </mujoco>)";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model.get(), NotNull()) << error.data();

  int mesh_in_pair_id = mj_name2id(model.get(), mjOBJ_MESH, "mesh");

  EXPECT_NE(mesh_in_pair_id, -1);

  // mesh_in_pair should have a convex hull because it is in a collision pair.
  EXPECT_NE(model->mesh_graphadr[mesh_in_pair_id], -1);
}

// ------------- test automatic inference of nuser_xxx -------------------------

using UserDataTest = MujocoTest;

TEST_F(UserDataTest, AutoNUserBody) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body user="1 2 3"/>
      <body user="2 3"/>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_body, 3);
  EXPECT_THAT(GetRow(m->body_user, m->nuser_body, 1), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->body_user, m->nuser_body, 2), ElementsAre(2, 3, 0));
}

TEST_F(UserDataTest, AutoNUserJoint) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint user="1 2 3"/>
        <joint user="2 3" axis="1 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_jnt, 3);
  EXPECT_THAT(GetRow(m->jnt_user, m->nuser_jnt, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->jnt_user, m->nuser_jnt, 1), ElementsAre(2, 3, 0));
}

TEST_F(UserDataTest, AutoNUserGeom) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom size="1" user="1 2 3"/>
      <geom size="1" user="2 3"/>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_geom, 3);
  EXPECT_THAT(GetRow(m->geom_user, m->nuser_geom, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->geom_user, m->nuser_geom, 1), ElementsAre(2, 3, 0));
}

TEST_F(UserDataTest, AutoNUserSite) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site user="1 2 3"/>
      <site user="2 3"/>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_site, 3);
  EXPECT_THAT(GetRow(m->site_user, m->nuser_site, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->site_user, m->nuser_site, 1), ElementsAre(2, 3, 0));
}

TEST_F(UserDataTest, AutoNUserCamera) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <camera user="1 2 3"/>
      <camera user="2 3"/>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_cam, 3);
  EXPECT_THAT(GetRow(m->cam_user, m->nuser_cam, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->cam_user, m->nuser_cam, 1), ElementsAre(2, 3, 0));
}

TEST_F(UserDataTest, AutoNUserTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="a"/>
      <site name="b"/>
    </worldbody>
    <tendon>
      <spatial user="1 2 3">
        <site site="a"/>
        <site site="b"/>
      </spatial>
      <spatial user="2 3">
        <site site="a"/>
        <site site="b"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_tendon, 3);
  EXPECT_THAT(GetRow(m->tendon_user, m->nuser_tendon, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->tendon_user, m->nuser_tendon, 1), ElementsAre(2, 3, 0));
}

TEST_F(UserDataTest, AutoNUserActuator) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="a"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="a" user="1 2 3"/>
      <motor joint="a" user="2 3"/>
    </actuator>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_actuator, 3);
  EXPECT_THAT(GetRow(m->actuator_user, m->nuser_actuator, 0),
              ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->actuator_user, m->nuser_actuator, 1),
              ElementsAre(2, 3, 0));
}

TEST_F(UserDataTest, AutoNUserSensor) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="a"/>
    </worldbody>
    <sensor>
      <accelerometer site="a" user="1 2 3"/>
      <gyro site="a" user="2 3"/>
    </sensor>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_sensor, 3);
  EXPECT_THAT(GetRow(m->sensor_user, m->nuser_sensor, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->sensor_user, m->nuser_sensor, 1), ElementsAre(2, 3, 0));
}

// ------------- test duplicate names ------------------------------------------
TEST_F(UserDataTest, DuplicateNames) {
  static const char* const kFilePath = "user/testdata/malformed_duplicated.xml";
  const std::string xml_path = GetTestDataFilePath(kFilePath);

  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());

  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("repeated name 'cube' in mesh"));
}

// ------------- test fusestatic -----------------------------------------------

using FuseStaticTest = MujocoTest;
TEST_F(FuseStaticTest, FuseStaticEquivalent) {
  static constexpr char xml_template[] = R"(
  <mujoco>
    <compiler fusestatic="%s"/>
    <worldbody>
      <body>
        <joint axis="1 0 0"/>
        <geom size="0.5" pos="1 0 0" contype="0" conaffinity="0"/>
        <body>
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::string fuse = absl::StrFormat(xml_template, "true");
  std::string no_fuse = absl::StrFormat(xml_template, "false");

  MjModelPtr m_fuse = LoadModelFromString(fuse.c_str());
  MjModelPtr m_no_fuse = LoadModelFromString(no_fuse.c_str());
  ASSERT_THAT(m_fuse.get(), NotNull());
  ASSERT_THAT(m_no_fuse.get(), NotNull());

  EXPECT_EQ(m_fuse->nbody, 2) << "Expecting a world body and one other body";
  EXPECT_EQ(m_no_fuse->nbody, 3) << "Expecting a world body and two others";

  EXPECT_EQ(m_no_fuse->body_contype[2], 1);
  EXPECT_EQ(m_no_fuse->body_conaffinity[2], 1);
  EXPECT_EQ(m_fuse->body_contype[1], 1);
  EXPECT_EQ(m_fuse->body_conaffinity[1], 1);

  EXPECT_EQ(m_no_fuse->body_bvhnum[2], 3);
  EXPECT_EQ(m_fuse->body_bvhnum[1], 3);

  MjDataPtr d_fuse = MakeData(m_fuse);
  MjDataPtr d_no_fuse = MakeData(m_no_fuse);

  mj_step(m_fuse.get(), d_fuse.get());
  mj_step(m_no_fuse.get(), d_no_fuse.get());

  EXPECT_NEAR(d_fuse.get()->qvel[0], d_no_fuse.get()->qvel[0],
              MjTol(2e-16, 1e-8))
      << "Velocity should be the same after 1 step";
  EXPECT_NE(d_fuse.get()->qvel[0], 0);
}

TEST_F(FuseStaticTest, FuseStaticActuatorReferencedBody) {
  static constexpr char xml_template[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>

    <worldbody>
      <body>
        <joint axis="1 0 0"/>
        <geom size="0.5" pos="1 0 0" contype="0" conaffinity="0"/>
        <body name="not_referenced">
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
        <body name="referenced">
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
      </body>
    </worldbody>

    <actuator>
      <adhesion body="referenced" ctrlrange="0 1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml_template, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 3) << "Expecting a world body and two others";
}

TEST_F(FuseStaticTest, FuseStaticLightReferencedBody) {
  static constexpr char xml_template[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>

    <worldbody>
      <light mode="targetbody" target="referenced"/>
      <body>
        <joint axis="1 0 0"/>
        <geom size="0.5" pos="1 0 0" contype="0" conaffinity="0"/>
        <body name="not_referenced">
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
        <body name="referenced">
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml_template, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 3) << "Expecting a world body and two others";
}

TEST_F(FuseStaticTest, FuseStaticForceSensorReferencedBody) {
  static constexpr char xml_template[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>

    <worldbody>
      <body>
        <joint axis="1 0 0"/>
        <geom size="0.5" pos="1 0 0" contype="0" conaffinity="0"/>
        <body name="not_referenced">
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
        <body name="referenced">
          <site name="force"/>
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
      </body>
    </worldbody>

    <sensor>
      <force site="force"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml_template, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 3) << "Expecting a world body and two others";
}

// fusing a body changes the ids of the bodies which follow it
TEST_F(FuseStaticTest, FuseStaticBodyIdsAfterFuse) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>
    <worldbody>
      <body>
        <geom size=".1"/>
      </body>
      <body name="B">
        <geom size=".1"/>
      </body>
      <body name="C">
        <freejoint/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <sensor>
      <framepos objtype="body" objname="C"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 2) << "Expecting a world body and the moving body";
  EXPECT_EQ(m->sensor_objid[0], mj_name2id(m.get(), mjOBJ_BODY, "C"));
}

// fusing a body which follows a sibling moves its elements ahead of the
// sibling's; names keep their objects, in the model and in the spec
TEST_F(FuseStaticTest, FuseStaticElementIdsAfterFuse) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>
    <worldbody>
      <body name="moving">
        <freejoint/>
        <geom name="moving" size=".1"/>
        <site name="moving"/>
        <camera name="moving"/>
        <light name="moving"/>
      </body>
      <body pos="1 0 0">
        <geom name="static" size=".2"/>
        <site name="static"/>
        <camera name="static"/>
        <light name="static"/>
      </body>
    </worldbody>
    <sensor>
      <framepos objtype="geom" objname="moving"/>
      <framepos objtype="site" objname="moving"/>
      <framepos objtype="camera" objname="moving"/>
    </sensor>
    <custom>
      <tuple name="tuple">
        <element objtype="light" objname="moving"/>
      </tuple>
    </custom>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);
  ASSERT_EQ(m->nbody, 2) << "Static body should be fused";

  // the fused elements precede the elements of the moving body
  EXPECT_EQ(m->sensor_objid[0], 1);
  EXPECT_EQ(m->sensor_objid[1], 1);
  EXPECT_EQ(m->sensor_objid[2], 1);
  EXPECT_EQ(m->tuple_objid[0], 1);

  for (mjtObj type : {mjOBJ_GEOM, mjOBJ_SITE, mjOBJ_CAMERA, mjOBJ_LIGHT}) {
    EXPECT_EQ(mj_name2id(m, type, "moving"), 1);
    for (const char* name : {"moving", "static"}) {
      mjsElement* element = mjs_findElement(spec, type, name);
      ASSERT_THAT(element, NotNull());
      EXPECT_STREQ(mjs_getString(mjs_getName(element)), name);
      EXPECT_EQ(mjs_getId(element), mj_name2id(m, type, name));
    }
  }

  mj_deleteModel(m);
  mj_deleteSpec(spec);
}

// the fuse is an operation on the spec, which the compiler option applies
TEST_F(FuseStaticTest, FuseStaticOperation) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="base" pos="0 0 1">
        <joint name="base"/>
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        <body name="static" pos="1 0 0" euler="0 0 90">
          <geom name="static" size=".1" pos="0 0 .5"/>
          <body name="first" pos="0 1 0">
            <joint name="first"/>
            <geom size=".1"/>
          </body>
        </body>
        <body name="second" pos="0 0 1">
          <joint name="second"/>
          <geom size=".1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  std::string field;

  // compiled with the option
  mjSpec* spec_option = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec_option, NotNull()) << error.data();
  spec_option->compiler.fusestatic = 1;
  mjModel* m_option = mj_compile(spec_option, nullptr);
  ASSERT_THAT(m_option, NotNull()) << mjs_getError(spec_option);
  ASSERT_EQ(m_option->nbody, 4);

  // the operation followed by a compilation without the option
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  EXPECT_EQ(mjs_fuseStatic(spec, nullptr), 0) << mjs_getError(spec);
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(CompareModel(m_option, m, field), 0) << field;

  // the body is a frame in its parent, which holds its geom as it was written
  EXPECT_THAT(mjs_findBody(spec, "static"), IsNull());
  mjsElement* geom = mjs_findElement(spec, mjOBJ_GEOM, "static");
  ASSERT_THAT(geom, NotNull());
  mjsFrame* frame = mjs_getFrame(geom);
  ASSERT_THAT(frame, NotNull());
  EXPECT_EQ(frame->pos[0], 1);
  EXPECT_EQ(mjs_asGeom(geom)->pos[2], 0.5);
  EXPECT_EQ(mjs_getParent(geom), mjs_findBody(spec, "base"));

  // its child body takes its place, so the joints keep their order
  EXPECT_EQ(mj_name2id(m, mjOBJ_JOINT, "first"), 1);
  EXPECT_EQ(mj_name2id(m, mjOBJ_JOINT, "second"), 2);

  // the parent, whose inertial is given, has the mass of the fused geom too
  const double sphere = 4.0 / 3.0 * mjPI * 1e-3 * 1000;
  EXPECT_NEAR(m->body_mass[1], 1 + sphere, MjTol(1e-12, 1e-6));

  // applying it again changes nothing
  mjSpec* fused = mj_copySpec(spec);
  EXPECT_EQ(mjs_fuseStatic(spec, nullptr), 0);
  EXPECT_THAT(CompareSpec(fused, spec), IsEmpty());

  // if the compilation fails after the fuse, the fuse stays and the error says
  // so
  mjSpec* spec_fail = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec_fail, NotNull()) << error.data();
  spec_fail->compiler.fusestatic = 1;
  mjsKey* key = mjs_addKey(spec_fail);
  const std::vector<double> qpos(5, 0);
  mjs_setDouble(key->qpos, qpos.data(), qpos.size());
  EXPECT_THAT(mj_compile(spec_fail, nullptr), IsNull());
  EXPECT_THAT(mjs_getError(spec_fail), HasSubstr("were fused"));
  EXPECT_THAT(mjs_findBody(spec_fail, "static"), IsNull());
  EXPECT_EQ(mjs_delete(spec_fail, key->element), 0);
  mjModel* m_fixed = mj_compile(spec_fail, nullptr);
  ASSERT_THAT(m_fixed, NotNull()) << mjs_getError(spec_fail);
  EXPECT_EQ(CompareModel(m_option, m_fixed, field), 0) << field;

  mj_deleteModel(m_option);
  mj_deleteModel(m);
  mj_deleteModel(m_fixed);
  mj_deleteSpec(fused);
  mj_deleteSpec(spec_fail);
  mj_deleteSpec(spec);
  mj_deleteSpec(spec_option);
}

// a body whose fuse attribute is "false" is kept
TEST_F(FuseStaticTest, FuseStaticKeepsBody) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>
    <worldbody>
      <body name="moving">
        <joint/>
        <geom size=".1"/>
        <body name="kept" pos="1 0 0" fuse="false">
          <geom size=".1"/>
          <body name="inner" pos="0 1 0">
            <geom size=".1"/>
          </body>
        </body>
        <body name="fused" pos="0 0 1">
          <geom size=".1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, nullptr, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  EXPECT_EQ(mjs_findBody(spec, "kept")->fuse, 0);
  EXPECT_EQ(mjs_findBody(spec, "fused")->fuse, 1);
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);

  // the body is kept, the static bodies in it and beside it are fused
  EXPECT_EQ(m->nbody, 3);
  EXPECT_EQ(mj_name2id(m, mjOBJ_BODY, "kept"), 2);
  EXPECT_THAT(mjs_findBody(spec, "inner"), IsNull());
  EXPECT_THAT(mjs_findBody(spec, "fused"), IsNull());

  // the attribute is saved, only where it is not the default
  std::string saved = SaveAndReadXml(spec);
  EXPECT_THAT(saved,
              HasSubstr(R"(<body name="kept" pos="1 0 0" fuse="false">)"));
  EXPECT_EQ(saved.find("fuse="), saved.rfind("fuse="));

  // without the attribute, the operation fuses the body
  mjs_findBody(spec, "kept")->fuse = 1;
  EXPECT_EQ(mjs_fuseStatic(spec, nullptr), 0) << mjs_getError(spec);
  EXPECT_THAT(mjs_findBody(spec, "kept"), IsNull());

  mj_deleteModel(m);
  mj_deleteSpec(spec);
}

// inertia which compilation adjusted is kept: the mass of each body is raised
// to boundmass, which the geoms of both in one body would not reproduce
TEST_F(FuseStaticTest, FuseStaticBoundMass) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler boundmass="1" inertiafromgeom="%s"/>
    <worldbody>
      <body name="moving">
        <joint name="slide" type="slide"/>
        <geom name="moving" size=".1" mass=".1"/>
        <body name="static" pos="1 0 0">
          <geom name="static" size=".1" mass=".1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  for (const char* inertiafromgeom : {"auto", "true"}) {
    SCOPED_TRACE(inertiafromgeom);
    std::string model = absl::StrFormat(xml, inertiafromgeom);
    MjModelPtr no_fuse = LoadModelFromString(model, error.data(), error.size());
    ASSERT_THAT(no_fuse.get(), NotNull()) << error.data();
    MjDataPtr d_no_fuse = MakeData(no_fuse);
    mj_forward(no_fuse.get(), d_no_fuse.get());
    EXPECT_EQ(d_no_fuse->M[0], 2);

    mjSpec* spec =
        mj_parseXMLString(model.c_str(), 0, error.data(), error.size());
    ASSERT_THAT(spec, NotNull()) << error.data();
    EXPECT_EQ(mjs_fuseStatic(spec, nullptr), 0) << mjs_getError(spec);
    MjModelPtr fuse(mj_compile(spec, nullptr));
    ASSERT_THAT(fuse.get(), NotNull()) << mjs_getError(spec);

    // the sum cannot be written when the inertia of every body is inferred: the
    // body is kept
    EXPECT_EQ(fuse->nbody, std::string(inertiafromgeom) == "true" ? 3 : 2);
    MjDataPtr d_fuse = MakeData(fuse);
    mj_forward(fuse.get(), d_fuse.get());
    EXPECT_NEAR(d_fuse->M[0], 2, MjTol(1e-12, 1e-6));
    mj_deleteSpec(spec);
  }
}

// a body which is referenced by a skin is not fused, also if the skin is read
// from a file
TEST_F(FuseStaticTest, FuseStaticSkinReferencedBody) {
  const std::string path = GetTestDataFilePath("user/testdata/cube_skin.xml");
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXML(path.c_str(), nullptr, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  spec->compiler.fusestatic = 1;
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(m->nbody, 2);
  EXPECT_EQ(m->skin_bonebodyid[0], mj_name2id(m, mjOBJ_BODY, "box"));
  mj_deleteModel(m);
  mj_deleteSpec(spec);
}

// a body which is referenced by a flex is not fused
TEST_F(FuseStaticTest, FuseStaticFlexReferencedBody) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>

    <worldbody>
      <body>
        <freejoint/>
        <geom size="0.5"/>
        <body name="not_referenced">
          <geom size="0.1"/>
        </body>
        <body name="v0">
          <geom size="0.1"/>
        </body>
        <body name="v1" pos="1 0 0">
          <geom size="0.1"/>
        </body>
        <body name="v2" pos="0 1 0">
          <geom size="0.1"/>
        </body>
      </body>
    </worldbody>

    <deformable>
      <flex name="flex" dim="2" body="v0 v1 v2" element="0 1 2"/>
    </deformable>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 5) << "Expecting world, a moving body and 3 flex bodies";
  EXPECT_EQ(mj_name2id(m.get(), mjOBJ_BODY, "not_referenced"), -1);
  for (int i = 0; i < 3; i++) {
    std::string name = absl::StrFormat("v%d", i);
    EXPECT_EQ(m->flex_vertbodyid[i],
              mj_name2id(m.get(), mjOBJ_BODY, name.c_str()));
  }
}

TEST_F(FuseStaticTest, FuseStaticCameraInBody) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>
    <worldbody>
      <body>
        <joint axis="1 0 0"/>
        <geom size="0.5"/>
        <body pos="1 0 0">
          <site name="site1"/>
          <camera name="cam1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 2) << "Static body should be fused";
  EXPECT_EQ(m->ncam, 1);
}

TEST_F(FuseStaticTest, FuseStaticLightInBody) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>
    <worldbody>
      <body>
        <joint axis="1 0 0"/>
        <geom size="0.5"/>
        <body pos="1 0 0">
          <light name="light1" dir="0 0 -1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 2) << "Static body should be fused";
  EXPECT_EQ(m->nlight, 1);
}

// expect equal global poses of same-named elements, equal joint-space inertia
// and equal unconstrained acceleration of a moving model
static void ExpectEquivalent(const MjModelPtr& m1, const MjModelPtr& m2) {
  ASSERT_EQ(m1->nv, m2->nv);
  ASSERT_EQ(m1->nC, m2->nC);
  MjDataPtr d1 = MakeData(m1);
  MjDataPtr d2 = MakeData(m2);
  mju_fill(d1->qvel, 1, m1->nv);
  mju_fill(d2->qvel, 1, m2->nv);
  mj_forward(m1.get(), d1.get());
  mj_forward(m2.get(), d2.get());

  struct Field {
    const char* name;
    mjtObj type;
    mjtSize num;
    int dim;
    const mjtNum* x1;
    const mjtNum* x2;
  };
  const Field fields[] = {
      {"xpos", mjOBJ_BODY, m1->nbody, 3, d1->xpos, d2->xpos},
      {"xmat", mjOBJ_BODY, m1->nbody, 9, d1->xmat, d2->xmat},
      {"xanchor", mjOBJ_JOINT, m1->njnt, 3, d1->xanchor, d2->xanchor},
      {"xaxis", mjOBJ_JOINT, m1->njnt, 3, d1->xaxis, d2->xaxis},
      {"geom_xpos", mjOBJ_GEOM, m1->ngeom, 3, d1->geom_xpos, d2->geom_xpos},
      {"geom_xmat", mjOBJ_GEOM, m1->ngeom, 9, d1->geom_xmat, d2->geom_xmat},
      {"site_xpos", mjOBJ_SITE, m1->nsite, 3, d1->site_xpos, d2->site_xpos},
      {"site_xmat", mjOBJ_SITE, m1->nsite, 9, d1->site_xmat, d2->site_xmat},
      {"cam_xpos", mjOBJ_CAMERA, m1->ncam, 3, d1->cam_xpos, d2->cam_xpos},
      {"cam_xmat", mjOBJ_CAMERA, m1->ncam, 9, d1->cam_xmat, d2->cam_xmat},
      {"light_xpos", mjOBJ_LIGHT, m1->nlight, 3, d1->light_xpos,
       d2->light_xpos},
      {"light_xdir", mjOBJ_LIGHT, m1->nlight, 3, d1->light_xdir,
       d2->light_xdir},
  };
  for (const Field& f : fields) {
    for (int i = 0; i < f.num; i++) {
      const char* name = mj_id2name(m1.get(), f.type, i);
      ASSERT_THAT(name, NotNull()) << f.name << " " << i;
      int j = mj_name2id(m2.get(), f.type, name);
      ASSERT_GE(j, 0) << name;
      EXPECT_THAT(
          AsVector(f.x1 + f.dim * i, f.dim),
          Pointwise(MjNear(1e-15, 2e-6), AsVector(f.x2 + f.dim * j, f.dim)))
          << f.name << " of " << name;
    }
  }

  // fused inertia is as accurate as its principal axes
  mjtNum scale = 1 / mju_norm(d2->M, m2->nC);
  mju_scl(d1->M, d1->M, scale, m1->nC);
  mju_scl(d2->M, d2->M, scale, m2->nC);
  EXPECT_THAT(AsVector(d1->M, m1->nC),
              Pointwise(MjNear(1e-6, 1e-6), AsVector(d2->M, m2->nC)));

  // so are the bias and passive forces
  scale = 1 / mju_norm(d2->qacc_smooth, m2->nv);
  mju_scl(d1->qacc_smooth, d1->qacc_smooth, scale, m1->nv);
  mju_scl(d2->qacc_smooth, d2->qacc_smooth, scale, m2->nv);
  EXPECT_THAT(AsVector(d1->qacc_smooth, m1->nv),
              Pointwise(MjNear(2e-6, 1e-5), AsVector(d2->qacc_smooth, m2->nv)));
}

// expect the fused model to be equivalent to the model which is not fused,
// and to be reproduced by recompiling, copying and saving the fused spec
static void ExpectCoherentFuse(const std::string& fuse_xml,
                               const std::string& no_fuse_xml, mjtNum tol) {
  std::array<char, 1024> error;
  MjModelPtr no_fuse =
      LoadModelFromString(no_fuse_xml, error.data(), error.size());
  ASSERT_THAT(no_fuse.get(), NotNull()) << error.data();

  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parseXMLString(fuse_xml.c_str(), nullptr, error.data(), error.size()),
      mj_deleteSpec);
  ASSERT_THAT(spec.get(), NotNull()) << error.data();
  MjModelPtr fuse(mj_compile(spec.get(), nullptr));
  ASSERT_THAT(fuse.get(), NotNull()) << mjs_getError(spec.get());
  EXPECT_LT(fuse->nbody, no_fuse->nbody);
  ExpectEquivalent(fuse, no_fuse);

  std::string field;
  MjModelPtr recompiled(mj_compile(spec.get(), nullptr));
  ASSERT_THAT(recompiled.get(), NotNull()) << mjs_getError(spec.get());
  EXPECT_LE(CompareModel(fuse.get(), recompiled.get(), field), tol)
      << "recompiled model is different: " << field;

  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> copy(
      mj_copySpec(spec.get()), mj_deleteSpec);
  MjModelPtr copied(mj_compile(copy.get(), nullptr));
  ASSERT_THAT(copied.get(), NotNull()) << mjs_getError(copy.get());
  EXPECT_LE(CompareModel(fuse.get(), copied.get(), field), tol)
      << "copied model is different: " << field;

  FullFloatPrecision increase_precision;
  MjModelPtr saved = LoadModelFromString(SaveAndReadXml(spec.get()),
                                         error.data(), error.size());
  ASSERT_THAT(saved.get(), NotNull()) << error.data();
  EXPECT_LE(CompareModel(fuse.get(), saved.get(), field), tol)
      << "saved model is different: " << field;
}

TEST_F(FuseStaticTest, FuseStaticFrames) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="%s"/>
    <worldbody>
      <body name="fixed" pos="1 2 3" euler="10 20 30">
        <frame pos="0 0 1" euler="0 0 30">
          <geom name="fixed" type="box" size=".1 .2 .3" pos=".1 0 0"/>
        </frame>
      </body>
      <body name="moving" pos="0 0 1">
        <freejoint name="free"/>
        <inertial pos=".1 .2 .3" mass="2" diaginertia="1 2 3"/>
        <geom name="moving" size=".1"/>
        <frame name="wrapper" pos="0 0 .5" euler="0 90 0">
          <body name="static" pos="1 0 0" euler="0 0 90">
            <geom name="static" type="box" size=".1 .2 .3" pos="0 0 1"/>
            <frame name="outer" pos="0 1 0" euler="90 0 0">
              <geom name="outer" type="box" size=".3 .2 .1" pos=".1 .2 .3"/>
              <geom name="fromto" type="box" size=".1" fromto="0 0 0 .1 .2 .3"/>
              <frame name="inner" pos="0 0 1" euler="0 45 0">
                <site name="inner" pos=".2 .3 .4" euler="0 30 0"/>
                <camera name="inner" pos=".3 .4 .5" euler="10 20 30"/>
                <light name="inner" pos=".4 .5 .6" dir="1 2 3"/>
                <body name="child" pos=".5 .6 .7" euler="30 20 10">
                  <joint name="hinge" axis="1 2 3"/>
                  <geom name="child" type="box" size=".1 .2 .3" pos=".1 0 0"/>
                </body>
                <body name="nested" pos=".7 .6 .5" euler="10 30 20">
                  <geom name="nested" type="box" size=".2 .3 .1" pos="0 .1 0"/>
                </body>
              </frame>
            </frame>
          </body>
        </frame>
      </body>
    </worldbody>
  </mujoco>
  )";
  ExpectCoherentFuse(absl::StrFormat(xml, "true"),
                     absl::StrFormat(xml, "false"), 0);
}

TEST_F(FuseStaticTest, FuseStaticInertia) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="%s"/>
    <worldbody>
      <body name="moving">
        <freejoint name="free"/>
        %s
        <geom name="moving" type="box" size=".1 .2 .3" pos=".1 0 0"/>
        <body name="static" pos="1 0 0" euler="10 20 30">
          %s
          <geom name="static" type="box" size=".3 .1 .2" pos="0 .2 0"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  static constexpr char parent[] =
      R"(<inertial pos=".1 .2 .3" mass="2" diaginertia="1 2 3"/>)";
  static constexpr char parent_full[] =
      R"(<inertial pos=".1 .2 .3" mass="2" fullinertia="4 3 2 .3 .2 .1"/>)";
  static constexpr char parent_framed[] = R"(
      <frame pos="0 0 1" euler="0 0 30">
        <inertial pos=".1 .2 .3" mass="2" diaginertia="1 2 3"/>
      </frame>)";
  static constexpr char child[] =
      R"(<inertial pos=".3 .2 .1" mass="3" diaginertia="3 2 4"/>)";
  static constexpr char child_framed[] = R"(
      <frame pos="0 1 0" euler="30 0 0">
        <inertial pos=".3 .2 .1" mass="3" diaginertia="3 2 4"/>
      </frame>)";

  for (const char* parent_inertial : {"", parent, parent_full, parent_framed}) {
    for (const char* child_inertial : {"", child, child_framed}) {
      SCOPED_TRACE(absl::StrFormat("parent '%s' child '%s'", parent_inertial,
                                   child_inertial));

      // inertia inferred from the fused geoms is as accurate as principal axes
      bool inferred = !*parent_inertial && !*child_inertial;
      ExpectCoherentFuse(
          absl::StrFormat(xml, "true", parent_inertial, child_inertial),
          absl::StrFormat(xml, "false", parent_inertial, child_inertial),
          inferred ? 1e-5 : 0);
    }
  }
}

// the geoms of a body which gives a massless inertial do not add their mass to
// a parent which infers its inertia when the body is fused
TEST_F(FuseStaticTest, FuseStaticMasslessInertial) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="%s"/>
    <worldbody>
      <body name="moving">
        <freejoint name="free"/>
        <geom name="moving" type="box" size=".1 .2 .3" pos=".1 0 0"/>
        <body name="static" pos="1 0 0" euler="10 20 30">
          <inertial pos="%s" mass="%s" diaginertia="0 0 0"/>
          <geom name="static" type="box" size=".3 .1 .2" pos="0 .2 0"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  struct Case {
    const char* pos;
    const char* mass;
  };
  const Case cases[] = {{"0 0 0", "0"}, {".1 .2 .3", "1e-20"}};
  for (const Case& c : cases) {
    SCOPED_TRACE(absl::StrFormat("pos '%s' mass '%s'", c.pos, c.mass));
    std::string fuse_xml = absl::StrFormat(xml, "true", c.pos, c.mass);
    std::string no_fuse_xml = absl::StrFormat(xml, "false", c.pos, c.mass);
    MjModelPtr fuse = LoadModelFromString(fuse_xml);
    MjModelPtr no_fuse = LoadModelFromString(no_fuse_xml);
    ASSERT_THAT(fuse.get(), NotNull());
    ASSERT_THAT(no_fuse.get(), NotNull());
    ASSERT_EQ(fuse->nbody, 2);
    ASSERT_EQ(no_fuse->nbody, 3);

    int i = mj_name2id(fuse.get(), mjOBJ_BODY, "moving");
    int j = mj_name2id(no_fuse.get(), mjOBJ_BODY, "moving");
    ASSERT_GE(i, 0);
    ASSERT_GE(j, 0);
    EXPECT_NEAR(fuse->body_mass[i], no_fuse->body_mass[j], 1e-12);
    EXPECT_NEAR(fuse->body_subtreemass[i], no_fuse->body_subtreemass[j], 1e-12);
    EXPECT_THAT(AsVector(fuse->body_inertia + 3 * i, 3),
                Pointwise(DoubleNear(1e-12),
                          AsVector(no_fuse->body_inertia + 3 * j, 3)));
    EXPECT_THAT(
        AsVector(fuse->body_ipos + 3 * i, 3),
        Pointwise(DoubleNear(1e-12), AsVector(no_fuse->body_ipos + 3 * j, 3)));
    ExpectCoherentFuse(fuse_xml, no_fuse_xml, 0);
  }
}

// fusestatic does not discard the free joint alignment of cameras and lights
TEST_F(FuseStaticTest, FuseStaticAlignFree) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="%s" alignfree="true"/>
    <worldbody>
      <body name="moving" pos="0 0 1">
        <freejoint name="free"/>
        <geom name="moving" type="box" size=".1 .2 .3" pos="1 2 3" euler="10 20 30"/>
        <camera name="moving" pos="0 1 0" euler="30 20 10"/>
        <light name="moving" pos="0 1 0" dir="1 2 3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr fuse = LoadModelFromString(absl::StrFormat(xml, "true"));
  MjModelPtr no_fuse = LoadModelFromString(absl::StrFormat(xml, "false"));
  ASSERT_THAT(fuse.get(), NotNull());
  ASSERT_THAT(no_fuse.get(), NotNull());
  ExpectEquivalent(fuse, no_fuse);
}

// gravcomp applies to the mass of each body, fused masses have equal gravcomp
TEST_F(FuseStaticTest, FuseStaticGravcomp) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="%s"/>
    <worldbody>
      <body name="fixed" gravcomp="1">
        <geom name="fixed" size=".1"/>
      </body>
      <body name="moving" pos="0 0 1" gravcomp="%s">
        <joint name="hinge" axis="0 1 0"/>
        <geom name="moving" size=".1"/>
        <body name="massless" gravcomp="2">
          <site name="massless"/>
        </body>
        <body name="static" pos="1 0 0" gravcomp="%s">
          <geom name="static" size=".1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  struct Case {
    const char* parent;
    const char* child;
    int nbody;
  };

  // bodies which are fixed to the world or massless are always fused
  const Case cases[] = {
      {"0", ".5", 3}, {".5", "0", 3}, {".5", "2", 3}, {".5", ".5", 2}};
  for (const Case& c : cases) {
    SCOPED_TRACE(absl::StrFormat("parent %s child %s", c.parent, c.child));
    std::string fuse_xml = absl::StrFormat(xml, "true", c.parent, c.child);
    MjModelPtr fuse = LoadModelFromString(fuse_xml);
    ASSERT_THAT(fuse.get(), NotNull());
    EXPECT_EQ(fuse->nbody, c.nbody);
    ExpectCoherentFuse(fuse_xml,
                       absl::StrFormat(xml, "false", c.parent, c.child),
                       c.nbody == 2 ? 1e-5 : 0);
  }
}

// fluid forces apply to the inertia and ellipsoid geoms of each body, which are
// not fused in a fluid
TEST_F(FuseStaticTest, FuseStaticFluid) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="%s"/>
    <option %s/>
    <worldbody>
      <body name="fixed">
        <geom name="fixed" size=".1"/>
      </body>
      <body name="moving" pos="0 0 1">
        <joint name="hinge" axis="0 1 0"/>
        <geom name="moving" size=".1"/>
        <body name="massless" pos="0 0 1">
          <site name="massless"/>
        </body>
        <body name="static" pos="1 0 0">
          <geom name="static" size=".1" %s/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  struct Case {
    const char* option;
    const char* geom;
    int nbody;
  };

  // bodies which are fixed to the world or have no mass and no ellipsoid geoms
  // are always fused
  const Case cases[] = {
      {"", "", 2},
      {R"(density="1000")", "", 3},
      {R"(viscosity="1")", "", 3},
      {R"(density="1000")", R"(mass="0")", 2},
      {R"(density="1000")", R"(mass="0" fluidshape="ellipsoid")", 3}};
  for (const Case& c : cases) {
    SCOPED_TRACE(absl::StrFormat("option '%s' geom '%s'", c.option, c.geom));
    std::string fuse_xml = absl::StrFormat(xml, "true", c.option, c.geom);
    MjModelPtr fuse = LoadModelFromString(fuse_xml);
    ASSERT_THAT(fuse.get(), NotNull());
    EXPECT_EQ(fuse->nbody, c.nbody);
    ExpectCoherentFuse(fuse_xml,
                       absl::StrFormat(xml, "false", c.option, c.geom),
                       c.nbody == 2 ? 1e-5 : 0);
  }
}

// a body with a plugin is not fused, the plugin's forces are specific to it
TEST_F(FuseStaticTest, FuseStaticPlugin) {
  // passive plugin applying an upward force to the center of mass of its bodies
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "mujoco.test.lift";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int type) {
    for (int i = 1; i < m->nbody; i++) {
      if (m->body_plugin[i] == instance) {
        mjtNum force[3] = {0, 0, 10}, torque[3] = {0};
        mj_applyFT(m, d, force, torque, d->xipos + 3 * i, i, d->qfrc_passive);
      }
    }
  };
  mjp_registerPlugin(&plugin);

  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="%s"/>
    <extension>
      <plugin plugin="mujoco.test.lift"/>
    </extension>
    <worldbody>
      <body name="moving" pos="0 0 1">
        <joint name="hinge" axis="0 1 0"/>
        <geom name="moving" size=".1"/>
        <body name="static" pos="1 0 0">
          <geom name="static" size=".1"/>
          <plugin plugin="mujoco.test.lift"/>
        </body>
        <body name="fused" pos="0 1 0">
          <geom name="fused" size=".1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::string fuse_xml = absl::StrFormat(xml, "true");
  MjModelPtr fuse = LoadModelFromString(fuse_xml);
  ASSERT_THAT(fuse.get(), NotNull());
  EXPECT_EQ(fuse->nbody, 3);
  ExpectCoherentFuse(fuse_xml, absl::StrFormat(xml, "false"), 1e-5);
}

// the sleep policy of a static body is an error, with or without fusing
TEST_F(FuseStaticTest, FuseStaticSleepPolicy) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler fusestatic="%s"/>
    <worldbody>
      <body>
        <joint/>
        <geom size=".1"/>
        <body sleep="never">
          <geom size=".1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  for (const char* fusestatic : {"false", "true"}) {
    std::array<char, 1024> error;
    MjModelPtr m = LoadModelFromString(absl::StrFormat(xml, fusestatic),
                                       error.data(), error.size());
    EXPECT_THAT(m.get(), IsNull()) << "fusestatic " << fusestatic;
    EXPECT_THAT(error.data(), HasSubstr("sleep policy only allowed"));
  }
}

// ------------- test discardvisual --------------------------------------------

using DiscardVisualTest = MujocoTest;
TEST_F(DiscardVisualTest, DiscardVisualKeepsInertia) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler discardvisual="true"/>

    <asset>
      <mesh name="visual_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        normal="1 0 0  0 1 0  0 0 1  0.707 0 0.707"
        face="0 2 1  0 3 2" />

      <mesh name="collision_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        normal="1 0 0  0 1 0  0 0 1  0.707 0 0.707"
        face="0 2 1  0 3 2" />
    </asset>

    <worldbody>
      <body>
        <geom type="mesh" mesh="visual_mesh" contype="0" conaffinity="0"/>
      </body>
      <body>
        <geom type="mesh" mesh="collision_mesh"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model.get(), NotNull()) << error.data();
  EXPECT_THAT(model->nmesh, 1);
  EXPECT_THAT(model->body_inertia[3], model->body_inertia[6]);
  EXPECT_THAT(model->body_inertia[4], model->body_inertia[7]);
  EXPECT_THAT(model->body_inertia[5], model->body_inertia[8]);
}

TEST_F(DiscardVisualTest, DiscardVisualEquivalent) {
  char error[1024];
  size_t error_sz = 1024;

  static const char* const kDiscardvisualPath =
      "user/testdata/discardvisual.xml";
  static const char* const kDiscardvisualFalsePath =
      "user/testdata/discardvisual_false.xml";

  const std::string xml_path1 = GetTestDataFilePath(kDiscardvisualPath);
  mjModel* model1 = mj_loadXML(xml_path1.c_str(), 0, error, error_sz);
  EXPECT_THAT(model1, NotNull()) << error;

  const std::string xml_path2 = GetTestDataFilePath(kDiscardvisualFalsePath);
  mjModel* model2 = mj_loadXML(xml_path2.c_str(), 0, error, error_sz);
  EXPECT_THAT(model2, NotNull()) << error;

  EXPECT_THAT(model1->nq, model2->nq);
  EXPECT_THAT(model1->nmat, 0);
  EXPECT_THAT(model1->ntex, 0);
  EXPECT_THAT(model2->ngeom - model1->ngeom, 3);
  EXPECT_THAT(model2->nmesh - model1->nmesh, 2);
  EXPECT_THAT(model1->npair, model2->npair);
  EXPECT_THAT(model1->nsensor, model2->nsensor);
  EXPECT_THAT(model1->nwrap, model2->nwrap);

  for (int i = 0; i < model1->ngeom; i++) {
    std::string name = std::string(model1->names + model1->name_geomadr[i]);
    EXPECT_NE(name.find("kept"), std::string::npos);
    EXPECT_EQ(name.find("discard"), std::string::npos);
  }

  for (int i = 0; i < model1->npair; i++) {
    int adr1 = model1->name_geomadr[model1->pair_geom1[i]];
    int adr2 = model2->name_geomadr[model2->pair_geom1[i]];
    EXPECT_STREQ(model1->names + adr1, model2->names + adr2);
    adr1 = model1->name_geomadr[model1->pair_geom2[i]];
    adr2 = model2->name_geomadr[model2->pair_geom2[i]];
    EXPECT_STREQ(model1->names + adr1, model2->names + adr2);
  }

  for (int i = 0; i < model1->nsensor; i++) {
    int adr1 = model1->name_geomadr[model1->sensor_objid[i]];
    int adr2 = model2->name_geomadr[model2->sensor_objid[i]];
    EXPECT_STREQ(model1->names + adr1, model2->names + adr2);
  }

  for (int i = 0; i < model1->nwrap; i++) {
    int adr1 = model1->name_geomadr[model1->wrap_objid[i]];
    int adr2 = model2->name_geomadr[model2->wrap_objid[i]];
    EXPECT_STREQ(model1->names + adr1, model2->names + adr2);
  }

  mjData* d1 = mj_makeData(model1);
  mjData* d2 = mj_makeData(model2);
  for (int i = 0; i < 100; i++) {
    mj_step(model1, d1);
    mj_step(model2, d2);
  }

  for (int i = 0; i < model1->nq; i++) {
    EXPECT_THAT(d1->qpos[i], d2->qpos[i]);
  }

  mj_deleteModel(model1);
  mj_deleteModel(model2);
  mj_deleteData(d1);
  mj_deleteData(d2);
}

TEST_F(DiscardVisualTest, FindElementAfterDiscard) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler discardvisual="true"/>
    <asset>
      <texture name="tex" type="2d" builtin="checker" width="8" height="8"/>
      <material name="mat" texture="tex"/>
      <mesh name="vismesh" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      <mesh name="colmesh" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
    </asset>
    <worldbody>
      <body name="b">
        <joint/>
        <geom name="vis1" type="mesh" mesh="vismesh" contype="0" conaffinity="0" material="mat"/>
        <geom name="vis2" size=".2" contype="0" conaffinity="0"/>
        <geom name="col1" size=".1"/>
        <geom name="col2" type="mesh" mesh="colmesh"/>
        <geom name="col3" size=".1" pos="1 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> err;
  mjSpec* spec = mj_parseXMLString(xml, 0, err.data(), err.size());
  ASSERT_THAT(spec, NotNull()) << err.data();
  mjModel* model = mj_compile(spec, 0);
  ASSERT_THAT(model, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(model->ngeom, 3);
  EXPECT_EQ(model->nmesh, 1);

  // discarded elements are not found
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, "vis1"), IsNull());
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, "vis2"), IsNull());
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_MESH, "vismesh"), IsNull());
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_MATERIAL, "mat"), IsNull());
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_TEXTURE, "tex"), IsNull());

  // surviving elements are found under their own names, at their new ids
  for (const char* name : {"col1", "col2", "col3"}) {
    mjsElement* el = mjs_findElement(spec, mjOBJ_GEOM, name);
    ASSERT_THAT(el, NotNull()) << name;
    EXPECT_STREQ(mjs_getString(mjs_getName(el)), name);
    EXPECT_EQ(mjs_getId(el), mj_name2id(model, mjOBJ_GEOM, name));
  }
  mjsElement* colmesh = mjs_findElement(spec, mjOBJ_MESH, "colmesh");
  ASSERT_THAT(colmesh, NotNull());
  EXPECT_STREQ(mjs_getString(mjs_getName(colmesh)), "colmesh");
  EXPECT_EQ(mjs_getId(colmesh), mj_name2id(model, mjOBJ_MESH, "colmesh"));

  // the spec recompiles to the same sizes
  mjModel* model2 = mj_compile(spec, 0);
  ASSERT_THAT(model2, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(model2->ngeom, 3);
  EXPECT_EQ(model2->nmesh, 1);
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, "vis1"), IsNull());

  mj_deleteModel(model2);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

// discarding is an operation on the spec, which the compiler option applies
TEST_F(DiscardVisualTest, DiscardVisualOperation) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option density="1.2" viscosity="1e-5"/>
    <asset>
      <texture name="tex" type="2d" builtin="checker" width="8" height="8"/>
      <material name="mat" texture="tex"/>
      <mesh name="visual" vertex="0 0 0  1 0 0  0 1 0  0 0 1" material="mat"/>
      <mesh name="collision" vertex="0 0 0  1 0 0  0 1 0  0 0 1" material="mat"/>
      <mesh name="site" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      <mesh name="tuple" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      <mesh name="unused" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
    </asset>
    <default>
      <geom material="mat"/>
      <site material="mat"/>
      <default class="visual">
        <geom contype="0" conaffinity="0"/>
      </default>
    </default>
    <worldbody>
      <light name="light" texture="tex"/>
      <site name="s0"/>
      <geom name="visual_world" class="visual" size=".1"/>
      <body name="inferred">
        <joint/>
        <geom name="collision" type="mesh" mesh="collision"/>
        <geom name="visual" class="visual" type="mesh" mesh="visual" pos="1 0 0"/>
        <geom name="visual_massless" class="visual" size=".1" mass="0"/>
        <geom name="wing" class="visual" type="ellipsoid" size=".3 .2 .01" fluidshape="ellipsoid"/>
        <site name="s1" type="mesh" mesh="site" pos="0 0 1"/>
        <body name="explicit" pos="0 0 1">
          <joint/>
          <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
          <geom name="visual_explicit" class="visual" size=".1"/>
          <geom name="sensor" class="visual" size=".1"/>
          <geom name="pair" class="visual" size=".1"/>
          <geom name="wrap" class="visual" size=".1" pos="0 .5 0"/>
        </body>
      </body>
    </worldbody>
    <contact>
      <pair geom1="collision" geom2="pair"/>
    </contact>
    <tendon>
      <spatial name="tendon" material="mat">
        <site site="s0"/>
        <geom geom="wrap"/>
        <site site="s1"/>
      </spatial>
    </tendon>
    <sensor>
      <framepos objtype="geom" objname="sensor"/>
    </sensor>
    <custom>
      <tuple name="tuple">
        <element objtype="mesh" objname="tuple"/>
      </tuple>
    </custom>
  </mujoco>
  )";
  std::array<char, 1024> error;
  std::string field;

  // the model with its visual elements
  mjSpec* spec_full = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec_full, NotNull()) << error.data();
  mjModel* m_full = mj_compile(spec_full, nullptr);
  ASSERT_THAT(m_full, NotNull()) << mjs_getError(spec_full);

  // compiled with the option
  mjSpec* spec_option = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec_option, NotNull()) << error.data();
  spec_option->compiler.discardvisual = 1;
  mjModel* m_option = mj_compile(spec_option, nullptr);
  ASSERT_THAT(m_option, NotNull()) << mjs_getError(spec_option);

  // the operation followed by a compilation without the option
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  EXPECT_EQ(mjs_discardVisual(spec, nullptr), 0) << mjs_getError(spec);
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(CompareModel(m_option, m, field), 0) << field;

  // materials and textures are gone, with the references to them
  EXPECT_EQ(m->nmat, 0);
  EXPECT_EQ(m->ntex, 0);
  EXPECT_EQ(m->light_texid[0], -1);

  // geoms are kept if they collide, have fluid forces or are referenced
  for (const char* name : {"collision", "wing", "sensor", "pair", "wrap"}) {
    EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, name), NotNull()) << name;
  }
  for (const char* name :
       {"visual_world", "visual", "visual_massless", "visual_explicit"}) {
    EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, name), IsNull()) << name;
  }

  // meshes are kept if a remaining geom or a site uses them, or if they are
  // referenced
  for (const char* name : {"collision", "site", "tuple"}) {
    EXPECT_THAT(mjs_findElement(spec, mjOBJ_MESH, name), NotNull()) << name;
  }
  for (const char* name : {"visual", "unused"}) {
    EXPECT_THAT(mjs_findElement(spec, mjOBJ_MESH, name), IsNull()) << name;
  }

  // inertia which was inferred from a discarded geom is kept, as an explicit
  // inertial
  int inferred = mj_name2id(m, mjOBJ_BODY, "inferred");
  EXPECT_EQ(mjs_findBody(spec, "inferred")->explicitinertial, 1);
  EXPECT_NEAR(m->body_mass[inferred], m_full->body_mass[inferred],
              MjTol(1e-12, 1e-6));
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(m->body_inertia[3 * inferred + i],
                m_full->body_inertia[3 * inferred + i], MjTol(1e-12, 1e-6));
    EXPECT_NEAR(m->body_ipos[3 * inferred + i],
                m_full->body_ipos[3 * inferred + i], MjTol(1e-12, 1e-6));
  }

  // applying it again changes nothing
  mjSpec* discarded = mj_copySpec(spec);
  EXPECT_EQ(mjs_discardVisual(spec, nullptr), 0);
  EXPECT_THAT(CompareSpec(discarded, spec), IsEmpty());

  // compiling again and saving reproduce the model
  mjModel* m_again = mj_compile(spec, nullptr);
  ASSERT_THAT(m_again, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(CompareModel(m, m_again, field), 0) << field;
  {
    FullFloatPrecision increase_precision;
    MjModelPtr saved =
        LoadModelFromString(SaveAndReadXml(spec), error.data(), error.size());
    ASSERT_THAT(saved.get(), NotNull()) << error.data();
    EXPECT_LE(CompareModel(m, saved.get(), field), MjTol(1e-12, 1e-6)) << field;
  }

  mj_deleteModel(m_again);
  mj_deleteModel(m);
  mj_deleteModel(m_option);
  mj_deleteModel(m_full);
  mj_deleteSpec(discarded);
  mj_deleteSpec(spec);
  mj_deleteSpec(spec_option);
  mj_deleteSpec(spec_full);
}

// a material or texture which a sensor or tuple refers to is kept, as an
// element which nothing renders with
TEST_F(DiscardVisualTest, DiscardVisualKeepsReferencedAssets) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <texture name="discarded" type="2d" builtin="checker" width="8" height="8"/>
      <texture name="in_material" type="2d" builtin="checker" width="8" height="8"/>
      <texture name="in_tuple" type="2d" builtin="checker" width="8" height="8"/>
      <material name="discarded" texture="discarded"/>
      <material name="in_tuple" texture="in_material"/>
    </asset>
    <worldbody>
      <body>
        <joint/>
        <geom name="geom" size=".1" material="in_tuple"/>
      </body>
    </worldbody>
    <custom>
      <tuple name="tuple">
        <element objtype="material" objname="in_tuple"/>
        <element objtype="texture" objname="in_tuple"/>
      </tuple>
    </custom>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  EXPECT_EQ(mjs_discardVisual(spec, nullptr), 0) << mjs_getError(spec);
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);

  // what the tuple refers to is kept, and the tuple refers to it
  EXPECT_EQ(m->nmat, 1);
  EXPECT_EQ(m->ntex, 1);
  EXPECT_EQ(m->tuple_objid[0], mj_name2id(m, mjOBJ_MATERIAL, "in_tuple"));
  EXPECT_EQ(m->tuple_objid[1], mj_name2id(m, mjOBJ_TEXTURE, "in_tuple"));

  // nothing uses it: the geom has no material, the material no texture
  EXPECT_EQ(m->geom_matid[0], -1);
  for (int i = 0; i < mjNTEXROLE; i++) {
    EXPECT_EQ(m->mat_texid[i], -1) << i;
  }

  // applying it again changes nothing
  mjSpec* discarded = mj_copySpec(spec);
  EXPECT_EQ(mjs_discardVisual(spec, nullptr), 0);
  EXPECT_THAT(CompareSpec(discarded, spec), IsEmpty());

  mj_deleteModel(m);
  mj_deleteSpec(discarded);
  mj_deleteSpec(spec);
}

// with inertiafromgeom "true", inertia which is inferred from a discarded geom
// cannot be kept
TEST_F(DiscardVisualTest, DiscardVisualInertiaFromGeomTrue) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler inertiafromgeom="true"/>
    <asset>
      <material name="mat"/>
    </asset>
    <worldbody>
      <body name="body">
        <joint/>
        <geom size=".1" material="mat"/>
        <geom name="visual" size=".2" contype="0" conaffinity="0" %s/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;

  // the geom has mass: an error, and nothing is discarded
  std::string xml_mass = absl::StrFormat(xml, "");
  mjSpec* spec =
      mj_parseXMLString(xml_mass.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  mjSpec* copy = mj_copySpec(spec);
  EXPECT_EQ(mjs_discardVisual(spec, nullptr), -1);
  EXPECT_THAT(mjs_getError(spec), HasSubstr("inertiafromgeom 'true'"));
  EXPECT_THAT(mjs_getError(spec), HasSubstr("body"));
  EXPECT_THAT(CompareSpec(copy, spec), IsEmpty());

  // the same through the compiler option
  spec->compiler.discardvisual = 1;
  EXPECT_THAT(mj_compile(spec, nullptr), IsNull());
  EXPECT_THAT(mjs_getError(spec), HasSubstr("inertiafromgeom 'true'"));
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, "visual"), NotNull());
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_MATERIAL, "mat"), NotNull());
  mj_deleteSpec(copy);
  mj_deleteSpec(spec);

  // the geom has no mass: it is discarded
  std::string xml_massless = absl::StrFormat(xml, R"(mass="0")");
  spec = mj_parseXMLString(xml_massless.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  mjModel* m_full = mj_compile(spec, nullptr);
  ASSERT_THAT(m_full, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(mjs_discardVisual(spec, nullptr), 0) << mjs_getError(spec);
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, "visual"), IsNull());
  EXPECT_EQ(mjs_findBody(spec, "body")->explicitinertial, 0);
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(m->body_mass[1], m_full->body_mass[1]);
  mj_deleteModel(m);
  mj_deleteModel(m_full);
  mj_deleteSpec(spec);
}

// assets are read only if inertia is inferred from a discarded geom
TEST_F(DiscardVisualTest, DiscardVisualReadsAssetsIfNeeded) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="visual" file="this_file_does_not_exist.obj"/>
    </asset>
    <worldbody>
      <body name="body">
        <joint/>
        %s
        <geom size=".1"/>
        <geom name="visual" type="mesh" mesh="visual" contype="0" conaffinity="0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;

  // the body has an explicit inertial: the visual mesh is discarded without
  // being read
  std::string xml_explicit = absl::StrFormat(
      xml, R"(<inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>)");
  mjSpec* spec =
      mj_parseXMLString(xml_explicit.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  EXPECT_EQ(mjs_discardVisual(spec, nullptr), 0) << mjs_getError(spec);
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(m->nmesh, 0);
  EXPECT_EQ(m->ngeom, 1);
  mj_deleteModel(m);
  mj_deleteSpec(spec);

  // its inertia is inferred: the mesh is needed, so it is an error and nothing
  // is discarded
  std::string xml_inferred = absl::StrFormat(xml, "");
  spec = mj_parseXMLString(xml_inferred.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  EXPECT_EQ(mjs_discardVisual(spec, nullptr), -1);
  EXPECT_THAT(mjs_getError(spec), HasSubstr("this_file_does_not_exist.obj"));
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, "visual"), NotNull());
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_MESH, "visual"), NotNull());
  mj_deleteSpec(spec);
}

// if compilation fails after the discard, the discard stays and the error says
// so
TEST_F(DiscardVisualTest, DiscardVisualThenError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler discardvisual="true"/>
    <worldbody>
      <body name="body">
        <joint/>
        <geom size=".1"/>
        <geom name="visual" size=".2" contype="0" conaffinity="0"/>
      </body>
    </worldbody>
    <keyframe>
      <key name="key" qpos="1 2 3"/>
    </keyframe>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  EXPECT_THAT(mj_compile(spec, nullptr), IsNull());
  EXPECT_THAT(mjs_getError(spec), HasSubstr("were discarded"));
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, "visual"), IsNull());

  // the spec is valid once the error is fixed
  EXPECT_EQ(mjs_delete(spec, mjs_findElement(spec, mjOBJ_KEY, "key")), 0);
  mjModel* m = mj_compile(spec, nullptr);
  ASSERT_THAT(m, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(m->ngeom, 1);
  mj_deleteModel(m);
  mj_deleteSpec(spec);
}

// discarding visual elements deletes the plugin instances which only they refer
// to, also in a spec which was never compiled, and keeps those which another
// element refers to
TEST_F(DiscardVisualTest, DiscardVisualDeletesPluginInstances) {
  static constexpr char xml[] = R"(
  <mujoco>
    <extension>
      <plugin plugin="mujoco.sdf.torus">
        <instance name="shared">
          <config key="radius1" value="0.35"/>
          <config key="radius2" value="0.15"/>
        </instance>
      </plugin>
    </extension>
    <asset>
      <mesh name="shared">
        <plugin instance="shared"/>
      </mesh>
      <mesh name="visual">
        <plugin plugin="mujoco.sdf.torus">
          <config key="radius1" value="0.25"/>
          <config key="radius2" value="0.125"/>
        </plugin>
      </mesh>
    </asset>
    <worldbody>
      <body>
        <freejoint/>
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        <geom name="collides" type="sdf" mesh="shared">
          <plugin instance="shared"/>
        </geom>
        <geom name="visual" type="sdf" mesh="visual" contype="0" conaffinity="0">
          <plugin plugin="mujoco.sdf.torus">
            <config key="radius1" value="0.25"/>
            <config key="radius2" value="0.125"/>
          </plugin>
        </geom>
        <geom name="shares" type="sdf" mesh="shared" contype="0" conaffinity="0">
          <plugin instance="shared"/>
        </geom>
      </body>
    </worldbody>
  </mujoco>
  )";
  auto instances = [](mjSpec* spec) {
    int n = 0;
    for (mjsElement* plugin = mjs_firstElement(spec, mjOBJ_PLUGIN); plugin;
         plugin = mjs_nextElement(spec, plugin)) {
      n++;
    }
    return n;
  };
  std::array<char, 1024> error;
  mjSpec* spec = mj_parseXMLString(xml, 0, error.data(), error.size());
  ASSERT_THAT(spec, NotNull()) << error.data();
  ASSERT_EQ(instances(spec), 3);

  ASSERT_EQ(mjs_discardVisual(spec, nullptr), 0) << mjs_getError(spec);
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, "visual"), IsNull());
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_GEOM, "shares"), IsNull());
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_MESH, "visual"), IsNull());
  ASSERT_EQ(instances(spec), 1);
  EXPECT_THAT(mjs_findElement(spec, mjOBJ_PLUGIN, "shared"), NotNull());

  mjModel* model = mj_compile(spec, nullptr);
  ASSERT_THAT(model, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(model->nplugin, 1);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

// ------------- test lengthrange ----------------------------------------------

using LengthRangeTest = MujocoTest;

TEST_F(LengthRangeTest, LengthRangeThreading) {
  char error[1024];
  size_t error_sz = 1024;
  std::string field = "";

  static const char* const kLengthrangePath = "user/testdata/lengthrange.xml";

  const std::string xml_path1 = GetTestDataFilePath(kLengthrangePath);
  mjSpec* spec = mj_parseXML(xml_path1.c_str(), 0, error, error_sz);
  EXPECT_THAT(spec, NotNull()) << error;
  mjModel* model1 = mj_compile(spec, 0);
  EXPECT_THAT(model1, NotNull()) << error;

  // model is such that the lengthrange for first actuator is [1, sqrt(5)]
  EXPECT_NEAR(model1->actuator_lengthrange[0], 1.0, 1e-3);
  EXPECT_NEAR(model1->actuator_lengthrange[1], std::sqrt(5.0), 1e-3);

  // recompile without threads
  ASSERT_EQ(spec->compiler.usethread, 1);
  spec->compiler.usethread = 0;
  mjModel* model2 = mj_compile(spec, 0);
  EXPECT_THAT(model2, NotNull()) << error;

  // expect threaded and unthreaded models to be identical
  EXPECT_LE(CompareModel(model1, model2, field), 0)
      << "Threaded and unthreaded lengthrange models are different!\n"
      << "Different field: " << field << '\n';

  mj_deleteModel(model1);
  mj_deleteModel(model2);
  mj_deleteSpec(spec);
}

TEST_F(MujocoTest, ResolvePluginMissingInstanceThrowsError) {
  // Instance name="my_pid_config" dos not match actuator plugin's
  // instance="pid_config", this should throw an appropriate error
  static constexpr char xml_mismatch[] = R"(
  <mujoco>
    <extension>
      <plugin plugin="mujoco.pid">
        <instance name="my_pid_config" />
      </plugin>
    </extension>
    <worldbody>
      <body name="block" pos="0 0 0.5">
        <joint name="slide_z" type="slide" axis="0 0 1" />
        <geom type="box" size="0.1 0.1 0.1" mass="1.0" rgba="0 0.7 0 1"/>
      </body>
    </worldbody>
    <actuator>
      <plugin name="pid_actuator" joint="slide_z" plugin="mujoco.pid" instance="pid_config" />
    </actuator>
  </mujoco>
  )";

  std::array<char, 1024> error_buffer;
  mjSpec* spec = mj_parseXMLString(xml_mismatch, 0, error_buffer.data(),
                                   error_buffer.size());
  ASSERT_THAT(spec, NotNull()) << error_buffer.data();

  mjModel* model = mj_compile(spec, nullptr);
  EXPECT_THAT(model, IsNull());

  std::string error_msg = mjs_getError(spec);
  EXPECT_THAT(error_msg,
              HasSubstr("unrecognized name 'pid_config' for plugin instance"));

  if (model) mj_deleteModel(model);
  mj_deleteSpec(spec);
}

// ----------------------------- test modeldir  --------------------------------

TEST_F(MujocoTest, Modeldir) {
  static constexpr char cube[] = R"(
  v -1 -1  1
  v  1 -1  1
  v -1  1  1
  v  1  1  1
  v -1  1 -1
  v  1  1 -1
  v -1 -1 -1
  v  1 -1 -1)";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "meshdir/cube.obj", cube, sizeof(cube));

  // child with the asset
  mjSpec* child = mj_makeSpec();
  mjsMesh* mesh = mjs_addMesh(child, 0);
  mjsFrame* frame = mjs_addFrame(mjs_findBody(child, "world"), 0);
  mjsGeom* geom = mjs_addGeom(mjs_findBody(child, "world"), 0);
  mjs_setString(child->compiler.meshdir, "meshdir");
  mjs_setString(mesh->file, "cube.obj");
  mjs_setName(mesh->element, "cube");
  mjs_setString(geom->meshname, "cube");
  mjs_setFrame(geom->element, frame);
  geom->type = mjGEOM_MESH;

  // parent attaching the child
  mjSpec* spec = mj_makeSpec();
  mjs_setDeepCopy(spec, true);
  mjs_setString(spec->compiler.meshdir, "asset");
  mjs_attach(mjs_findBody(spec, "world")->element, frame->element, "_", "");
  mjModel* model = mj_compile(spec, vfs.get());
  EXPECT_THAT(model, NotNull());

  mj_deleteSpec(child);
  mj_deleteSpec(spec);
  mj_deleteModel(model);
  mj_deleteVFS(vfs.get());
}

TEST_F(MujocoTest, NestedMeshDir) {
  static constexpr char cube[] = R"(
  v -1 -1  1
  v  1 -1  1
  v -1  1  1
  v  1  1  1
  v -1  1 -1
  v  1  1 -1
  v -1 -1 -1
  v  1 -1 -1)";

  static constexpr char child_xml[] = R"(
  <mujoco>
    <compiler meshdir="child_meshdir"/>

    <asset>
      <mesh name="m" file="child_mesh.obj"/>
    </asset>

    <worldbody>
      <body name="child">
        <geom type="mesh" mesh="m"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char parent_xml[] = R"(
  <mujoco>
    <compiler meshdir="parent_meshdir"/>

    <asset>
      <mesh name="m" file="parent_mesh.obj"/>
      <model name="child" file="child.xml"/>
    </asset>

    <worldbody>
      <body name="parent">
        <geom type="mesh" mesh="m"/>
        <attach model="child" body="child" prefix="child_"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char grandparent_xml[] = R"(
  <mujoco>
    <compiler meshdir="grandparent_meshdir"/>

    <asset>
      <mesh name="m" file="grandparent_mesh.obj"/>
      <model name="parent" file="parent.xml"/>
    </asset>

    <worldbody>
      <geom type="mesh" mesh="m"/>
      <attach model="parent" body="parent" prefix="parent_"/>
    </worldbody>
  </mujoco>
  )";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "child_meshdir/child_mesh.obj", cube,
                  sizeof(cube));
  mj_addBufferVFS(vfs.get(), "child.xml", child_xml, sizeof(child_xml));
  mj_addBufferVFS(vfs.get(), "parent_meshdir/parent_mesh.obj", cube,
                  sizeof(cube));
  mj_addBufferVFS(vfs.get(), "parent.xml", parent_xml, sizeof(parent_xml));
  mj_addBufferVFS(vfs.get(), "grandparent_meshdir/grandparent_mesh.obj", cube,
                  sizeof(cube));

  std::array<char, 1024> error;
  MjModelPtr child_model =
      LoadModelFromString(child_xml, error.data(), error.size(), vfs.get());
  EXPECT_THAT(child_model.get(), NotNull()) << error.data();

  MjModelPtr parent_model =
      LoadModelFromString(parent_xml, error.data(), error.size(), vfs.get());
  EXPECT_THAT(parent_model.get(), NotNull()) << error.data();

  MjModelPtr grandparent_model = LoadModelFromString(
      grandparent_xml, error.data(), error.size(), vfs.get());
  EXPECT_THAT(grandparent_model.get(), NotNull()) << error.data();

  mj_deleteVFS(vfs.get());
}

TEST_F(MujocoTest, ConvertSpringdamper) {
  static constexpr char xml[] = R"(
    <mujoco>
    <worldbody>
      <body>
        <joint axis="0 1 0" springdamper="1 1"/>
        <geom size="0.2 0.2 0.2" type="box"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> err;
  mjSpec* spec = mj_parseXMLString(xml, 0, err.data(), err.size());
  ASSERT_THAT(spec, NotNull()) << err.data();
  mjModel* model = mj_compile(spec, 0);
  ASSERT_THAT(model, NotNull()) << err.data();
  std::array<char, 1024> str;
  mj_saveXMLString(spec, str.data(), str.size(), err.data(), err.size());
  EXPECT_THAT(str.data(), HasSubstr("damping"));
  EXPECT_THAT(str.data(), HasSubstr("stiffness"));
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

// ------------- test history buffer computation -------------------------------

using DelayBufferTest = MujocoTest;

TEST_F(DelayBufferTest, ActuatorDelayBufferSizes) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="1"/>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt1"/>
        <joint name="jnt2" axis="1 0 0"/>
        <joint name="jnt3" axis="0 1 0"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="jnt1"/>
      <motor joint="jnt2" delay="3" nsample="3"/>
      <motor joint="jnt3" delay="10" nsample="10"/>
    </actuator>
  </mujoco>
  )";
  MjModelPtr m = LoadModelFromString(xml);
  ASSERT_THAT(m.get(), NotNull());
  ASSERT_EQ(m->nu, 3);

  // nhistory = (2+2*3) + (2+2*10) = 8 + 22 = 30
  EXPECT_EQ(m->nhistory, 30);

  // verify per-actuator delay and addresses
  EXPECT_EQ(m->actuator_history[0], 0);
  EXPECT_EQ(m->actuator_history[2], 3);
  EXPECT_EQ(m->actuator_history[4], 10);
  EXPECT_EQ(m->actuator_historyadr[0], -1);
  EXPECT_EQ(m->actuator_historyadr[1], 0);
  EXPECT_EQ(m->actuator_historyadr[2], 8);
}

}  // namespace
}  // namespace mujoco
