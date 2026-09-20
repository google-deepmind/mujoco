// Copyright 2022 DeepMind Technologies Limited
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

// Tests for sdf collisions.

#include "src/engine/engine_collision_sdf.h"

#include <cstdio>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using SdfTest = MujocoTest;

static constexpr int kpoints = 6;
static constexpr int kgeoms = 5;
static constexpr char kSdfModel[] = R"(
<mujoco>
  <worldbody>
    <geom type="plane" size="5 5 .1" pos="0 0 -1"/>
    <body pos="-.1 .2 2" euler="0 0 45">
      <geom type="sphere" size="1"/>
      <geom type="capsule" size=".1" fromto="-2 -2 1 2 2 1"/>
      <geom type="cylinder" size="1" fromto="0 0 0 2 0 0"/>
      <geom type="box" size="1 1 1"/>
    </body>
  </worldbody>
</mujoco>
)";

TEST_F(SdfTest, SdfPrimitive) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kSdfModel, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());
  ASSERT_THAT(model->ngeom, kgeoms);

  mjSDF sdf;
  const mjpPlugin* null_plugin = NULL;
  mjtNum gradient[3],
      dist[kgeoms][kpoints] = {
          {0, 0, 0, 0, 1, 1},  // plane
          {-1, 0, 0, mju_sqrt(2) - 1, mju_sqrt(2) - 1,
           mju_sqrt(3) - 1},  // sphere
          {(mjtNum)-.1, (mjtNum).9, (mjtNum).9, mju_sqrt(2) - (mjtNum).1,
           (mjtNum).9, mju_sqrt(2) - (mjtNum).1},           // capsule
          {-1, 0, 0, mju_sqrt(2) - 1, 0, mju_sqrt(2) - 1},  // cylinder
          {-mju_sqrt(3), 0, 0, 0, 0, 0},                    // box
      };
  mjtNum points[kpoints][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0},
                               {1, 1, 0}, {0, 1, 1}, {1, 1, 1}};

  for (int i = 0; i < kgeoms; i++) {
    sdf.plugin = &null_plugin;
    sdf.id = &i;
    sdf.type = mjSDFTYPE_SINGLE;
    sdf.geomtype = (mjtGeom*)(model->geom_type + i);
    for (int j = 0; j < kpoints; j++) {
      EXPECT_NEAR(mjc_distance(model.get(), data.get(), &sdf, points[j]),
                  dist[i][j], MjTol(1e-9, 5e-7));
      mjc_gradient(model.get(), data.get(), &sdf, gradient, points[j]);
    }
  }
}

static constexpr char kFlexSdfModel[] = R"(
<mujoco>
  <extension>
    <plugin plugin="mujoco.sdf.torus">
      <instance name="torus">
        <config key="radius1" value="0.35"/>
        <config key="radius2" value="0.15"/>
      </instance>
    </plugin>
  </extension>

  <asset>
    <mesh name="torus">
      <plugin instance="torus"/>
    </mesh>
  </asset>

  <worldbody>
    <body pos="0 0 -0.3">
      <geom type="sdf" mesh="torus">
        <plugin instance="torus"/>
      </geom>
    </body>
    <body name="flex">
      <flexcomp name="test" type="grid" count="3 3 1" spacing=".2 .2 .2" dim="2">
        <elasticity young="1e4"/>
        <contact selfcollide="none" internal="false"/>
      </flexcomp>
    </body>
  </worldbody>
</mujoco>
)";

TEST_F(SdfTest, FlexSdfCollision) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(kFlexSdfModel, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());

  // check there is at least one flex
  ASSERT_GT(model->nflex, 0);

  // simulate for a few steps to let flex fall onto torus
  for (int i = 0; i < 100; i++) {
    mj_step(model.get(), data.get());
  }

  // should have contacts between flex and SDF
  EXPECT_GT(data->ncon, 0) << "Expected flex-SDF contacts";

  // check that contacts involve the flex
  bool has_flex_contact = false;
  for (int i = 0; i < data->ncon; i++) {
    if (data->contact[i].flex[0] >= 0 || data->contact[i].flex[1] >= 0) {
      has_flex_contact = true;
      break;
    }
  }
  EXPECT_TRUE(has_flex_contact) << "Expected at least one flex contact";
}

extern "C" {
mjtNum oct_distance(const mjModel* m, const mjtNum p[3], int meshid);
void oct_gradient(const mjModel* m, mjtNum grad[3], const mjtNum point[3],
                  int meshid);
}

TEST_F(SdfTest, OctreeAnalyticGradientMatchesFiniteDiff) {
  static constexpr char kCubeXml[] = R"(
  <mujoco>
    <asset>
      <mesh name="cube" vertex="-1 -1 -1  1 -1 -1  -1 1 -1  1 1 -1
                                -1 -1  1  1 -1  1  -1 1  1  1 1  1"/>
    </asset>
    <worldbody>
      <geom type="sdf" mesh="cube"/>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(kCubeXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  ASSERT_GT(model->mesh_octnum[0], 0);

  const mjtNum points[][3] = {
      {0.8, 0.4, -0.2},
      {0.2, -0.3, 0.5},
      {-0.4, 0.2, -0.6},
      {0.1, 0.1, 0.1},
  };
  const mjtNum eps = MjTol(1e-6, 1e-3);

  for (const auto& point : points) {
    mjtNum grad_analytic[3];
    oct_gradient(model.get(), grad_analytic, point, 0);

    mjtNum grad_fd[3];
    for (int k = 0; k < 3; ++k) {
      mjtNum p_plus[3] = {point[0], point[1], point[2]};
      mjtNum p_minus[3] = {point[0], point[1], point[2]};
      p_plus[k] += eps;
      p_minus[k] -= eps;
      mjtNum d_plus = oct_distance(model.get(), p_plus, 0);
      mjtNum d_minus = oct_distance(model.get(), p_minus, 0);
      grad_fd[k] = (d_plus - d_minus) / (2.0 * eps);
    }

    for (int k = 0; k < 3; ++k) {
      EXPECT_NEAR(grad_analytic[k], grad_fd[k], MjTol(1e-3, 0.05));
    }
  }
}

TEST_F(SdfTest, PenetrationContactPositionIsMidpoint) {
  static constexpr char kPenetrationXml[] = R"(
  <mujoco>
    <asset>
      <mesh name="cube" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  -0.5 0.5 -0.5  0.5 0.5 -0.5
                                -0.5 -0.5  0.5  0.5 -0.5  0.5  -0.5 0.5  0.5  0.5 0.5  0.5"/>
      <mesh name="small_cube" vertex="-0.1 -0.1 -0.1  0.1 -0.1 -0.1  -0.1 0.1 -0.1  0.1 0.1 -0.1
                                      -0.1 -0.1  0.1  0.1 -0.1  0.1  -0.1 0.1  0.1  0.1 0.1  0.1"/>
    </asset>
    <worldbody>
      <body pos="0 0 0.55">
        <geom name="geom1" type="mesh" mesh="small_cube"/>
      </body>
      <body pos="0 0 0">
        <geom name="geom2" type="sdf" mesh="cube"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(kPenetrationXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());
  mj_forward(model.get(), data.get());

  mjPreContact con[mjMAXCONPAIR];
  int ncon = mjc_MeshSDF(model.get(), data.get(), con, 0, 1, 0.0);
  ASSERT_GT(ncon, 0);

  // Mesh bottom face is at z = 0.45, SDF top face is at z = 0.50
  // Penetration distance = -0.05m
  // Midpoint is at z = (0.45 + 0.50) / 2 = 0.475m
  for (int i = 0; i < ncon; ++i) {
    EXPECT_NEAR(con[i].dist, -0.05, MjTol(2e-3, 5e-3));
    EXPECT_NEAR(con[i].pos[2], 0.475, MjTol(2e-3, 5e-3));
    EXPECT_NEAR(con[i].normal[2], -1.0, MjTol(1e-3, 5e-3));
  }
}

TEST_F(SdfTest, SdfCollisionExcessContactsNoCrash) {
  static constexpr char kSdfSdfXml[] = R"(
  <mujoco>
    <option sdf_initpoints="50"/>
    <asset>
      <mesh name="cube" vertex="-0.5 -0.5 -0.5  0.5 -0.5 -0.5  -0.5 0.5 -0.5  0.5 0.5 -0.5
                                -0.5 -0.5  0.5  0.5 -0.5  0.5  -0.5 0.5  0.5  0.5 0.5  0.5"/>
    </asset>
    <worldbody>
      <body pos="0 0 0.8">
        <geom name="geom1" type="sdf" mesh="cube"/>
      </body>
      <body pos="0 0 0">
        <geom name="geom2" type="sdf" mesh="cube"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(kSdfSdfXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());
  mj_forward(model.get(), data.get());

  mjPreContact con[mjMAXCONPAIR];
  int ncon = mjc_SDF(model.get(), data.get(), con, 0, 1, 0.0);
  EXPECT_LE(ncon, mjMAXCONPAIR);
  EXPECT_GT(ncon, 0);
}

}  // namespace
}  // namespace mujoco
