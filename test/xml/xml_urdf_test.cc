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

#include <cstddef>
#include <cstring>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

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

}  // namespace
}  // namespace mujoco
