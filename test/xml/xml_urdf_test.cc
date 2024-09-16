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

// Tests for xml/xml_api.cc.

#include <array>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::IsNull;
using ::testing::NotNull;

// ---------------------------- test capsule --------------------------------

TEST_F(MujocoTest, ReadsCapsule) {
  static constexpr char urdf[] = R"(
  <robot name="">
  <link name="torso">
    <collision>
      <origin rpy="-1.57080 0.00000 0.00000" xyz="0.00000 0.00000 0.00000"/>
      <geometry>
        <capsule length="0.14000" radius="0.07000"/>
      </geometry>
    </collision>
  </link>
  </robot>
  )";
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(urdf, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  mj_deleteModel(model);
}

TEST_F(MujocoTest, ReadsGeomNames) {
  static constexpr char urdf[] = R"(
  <robot name="">
  <mujoco>
    <compiler discardvisual="false"/>
  </mujoco>

  <link name="torso">
    <collision name="collision_box">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.1 0.2 0.3"/>
      </geometry>
    </collision>
    <visual name="visual_sphere">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.5"/>
      </geometry>
    </visual>
  </link>
  </robot>
  )";
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(urdf, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  // Check the geoms have been loaded with the right names
  int collision_box_id = mj_name2id(model, mjtObj::mjOBJ_GEOM, "collision_box");
  ASSERT_GE(collision_box_id, 0);
  EXPECT_EQ(model->geom_type[collision_box_id], mjtGeom::mjGEOM_BOX);

  int visual_sphere_id = mj_name2id(model, mjtObj::mjOBJ_GEOM, "visual_sphere");
  ASSERT_GE(visual_sphere_id, 0);
  EXPECT_EQ(model->geom_type[visual_sphere_id], mjtGeom::mjGEOM_SPHERE);

  mj_deleteModel(model);
}

TEST_F(MujocoTest, CanLoadUrdfWithNonUniqueNamesCollisionBeforeVisual) {
  static constexpr char urdf[] = R"(
  <robot name="">
  <mujoco>
    <compiler discardvisual="false"/>
  </mujoco>

  <link name="torso">
    <collision name="shared_name">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.1 0.2 0.3"/>
      </geometry>
    </collision>
    <visual name="shared_name">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.5"/>
      </geometry>
    </visual>
  </link>
  </robot>
  )";
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(urdf, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  // Check the collision geom gets its name from the URDF. The visual sphere
  // should not have a name to avoid duplicates.
  int collision_box_id = mj_name2id(model, mjtObj::mjOBJ_GEOM, "shared_name");
  ASSERT_GE(collision_box_id, 0);
  EXPECT_EQ(model->geom_type[collision_box_id], mjtGeom::mjGEOM_BOX);

  mj_deleteModel(model);
}

TEST_F(MujocoTest, CanLoadUrdfWithNonUniqueNamesVisualBeforeCollision) {
  static constexpr char urdf[] = R"(
  <robot name="">
  <mujoco>
    <compiler discardvisual="false"/>
  </mujoco>

  <link name="torso">
    <visual name="shared_name">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.5"/>
      </geometry>
    </visual>
    <collision name="shared_name">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.1 0.2 0.3"/>
      </geometry>
    </collision>
  </link>
  </robot>
  )";
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(urdf, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  // Check the visual geom gets its name from the URDF. The collision geom
  // should not have a name to avoid duplicates.
  int visual_sphere_id = mj_name2id(model, mjtObj::mjOBJ_GEOM, "shared_name");
  ASSERT_GE(visual_sphere_id, 0);
  EXPECT_EQ(model->geom_type[visual_sphere_id], mjtGeom::mjGEOM_SPHERE);

  mj_deleteModel(model);
}

TEST_F(MujocoTest, ReadsJointTypes) {
  static constexpr char urdf[] = R"(
  <robot name="">
  <mujoco>
    <compiler discardvisual="false"/>
  </mujoco>

  <link name="world"/>
  <joint type="floating" name="floating">
      <parent link="world"/>
      <child link="link1"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
  </joint>
  <link name="link1">
      <collision>
        <geometry>
          <box size="0.1 0.2 0.3"/>
        </geometry>
      </collision>
  </link>

  <joint type="revolute" name="revolute">
      <parent link="world"/>
      <child link="link2"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <axis xyz="1.0 0.0 -1.0"/>
  </joint>
  <link name="link2">
      <collision>
        <geometry>
          <box size="0.1 0.2 0.3"/>
        </geometry>
      </collision>
  </link>

  <joint type="spherical" name="spherical">
      <parent link="world"/>
      <child link="link3"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
  </joint>
  <link name="link3">
      <collision>
        <geometry>
          <box size="0.1 0.2 0.3"/>
        </geometry>
      </collision>
  </link>

  <joint type="prismatic" name="prismatic">
      <parent link="world"/>
      <child link="link4"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <axis xyz="0.0 0.0 1.0"/>
  </joint>
  <link name="link4">
      <collision>
        <geometry>
          <box size="0.1 0.2 0.3"/>
        </geometry>
      </collision>
  </link>

  </robot>
  )";
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(urdf, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  constexpr float eps = 1e-6;

  std::vector<std::string> joint_names = {"floating", "revolute", "spherical",
                                          "prismatic"};
  std::vector<mjtJoint> expected_joint_types = {
      mjtJoint::mjJNT_FREE, mjtJoint::mjJNT_HINGE, mjtJoint::mjJNT_BALL,
      mjtJoint::mjJNT_SLIDE};
  std::vector<std::vector<float>> expected_axis = {
    {0.0, 0.0, 1.0}, {0.707107, 0.0, -0.707107}, {0.0, 0.0, 1.0},
    {0.0, 0.0, 1.0}
  };
  for (int i = 0; i < joint_names.size(); ++i) {
    int id = mj_name2id(model, mjtObj::mjOBJ_JOINT, joint_names[i].c_str());
    EXPECT_EQ(model->jnt_type[id], expected_joint_types[i]);
    EXPECT_NEAR(model->jnt_axis[3 * id], expected_axis[i][0], eps);
    EXPECT_NEAR(model->jnt_axis[3 * id + 1], expected_axis[i][1], eps);
    EXPECT_NEAR(model->jnt_axis[3 * id + 2], expected_axis[i][2], eps);
  }

  mj_deleteModel(model);
}

TEST_F(MujocoTest, RepeatedMeshName) {
  static constexpr char urdf[] = R"(
  <robot name="">
  <mujoco>
    <compiler discardvisual="false"/>
  </mujoco>

  <link name="geom1">
    <visual name="vis1">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="mesh.obj" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>

  <link name="geom2">
    <visual name="vis2">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="mesh.obj" scale="2 2 2"/>
      </geometry>
    </visual>
  </link>

  <link name="geom3">
    <visual name="vis3">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="mesh.obj" scale="3 3 3"/>
      </geometry>
    </visual>
  </link>

  <link name="geom4">
    <visual name="vis4">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="mesh.obj" scale="2 2 2"/>
      </geometry>
    </visual>
  </link>
  </robot>
  )";

  std::array<char, 1000> error;
  mjSpec* spec = mj_parseXMLString(urdf, 0, error.data(), error.size());
  EXPECT_THAT(spec, NotNull()) << error.data();

  mjsMesh* mesh = mjs_asMesh(mjs_findElement(spec, mjOBJ_MESH, "mesh"));
  mjsMesh* mesh1 = mjs_asMesh(mjs_findElement(spec, mjOBJ_MESH, "mesh1"));
  mjsMesh* mesh2 = mjs_asMesh(mjs_findElement(spec, mjOBJ_MESH, "mesh2"));
  mjsMesh* mesh3 = mjs_asMesh(mjs_findElement(spec, mjOBJ_MESH, "mesh3"));
  EXPECT_THAT(mesh, NotNull());
  EXPECT_THAT(mesh1, NotNull());
  EXPECT_THAT(mesh2, NotNull());
  EXPECT_THAT(mesh3, IsNull());
  EXPECT_STREQ(mjs_getString(mesh->name), "mesh");
  EXPECT_STREQ(mjs_getString(mesh1->name), "mesh1");
  EXPECT_STREQ(mjs_getString(mesh2->name), "mesh2");

  mj_deleteSpec(spec);
}

}  // namespace
}  // namespace mujoco
