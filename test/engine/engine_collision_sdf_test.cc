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
#include <ostream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
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
        <contact selfcollide="none"/>
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

  const mjpPlugin* plugin = nullptr;
  int meshid = 0;
  mjtGeom geomtype = mjGEOM_SDF;
  mjSDF sdf{};
  sdf.type = mjSDFTYPE_SINGLE;
  sdf.id = &meshid;
  sdf.plugin = &plugin;
  sdf.geomtype = &geomtype;

  const mjtNum points[][3] = {
      {0.8, 0.4, -0.2},
      {0.2, -0.3, 0.5},
      {-0.4, 0.2, -0.6},
      {0.1, 0.1, 0.1},
  };
  const mjtNum eps = MjTol(1e-6, 1e-3);

  for (const auto& point : points) {
    mjtNum grad_analytic[3];
    mjc_gradient(model.get(), nullptr, &sdf, grad_analytic, point);

    mjtNum grad_fd[3];
    for (int k = 0; k < 3; ++k) {
      mjtNum p_plus[3] = {point[0], point[1], point[2]};
      mjtNum p_minus[3] = {point[0], point[1], point[2]};
      p_plus[k] += eps;
      p_minus[k] -= eps;
      mjtNum d_plus = mjc_distance(model.get(), nullptr, &sdf, p_plus);
      mjtNum d_minus = mjc_distance(model.get(), nullptr, &sdf, p_minus);
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
        <geom name="geom1" type="sdf" mesh="small_cube"/>
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
  // Exercise mesh-SDF, octree SDF-SDF, and primitive-SDF reconstruction.
  for (mjtGeom type : {mjGEOM_MESH, mjGEOM_SDF, mjGEOM_SPHERE}) {
    SCOPED_TRACE(type);
    model->geom_type[0] = type;
    model->opt.sdf_iterations = 100;
    mj_forward(model.get(), data.get());
    mjPreContact con[mjMAXCONPAIR];
    int ncon = mjCOLLISIONFUNC[type][mjGEOM_SDF](model.get(), data.get(), con, 0, 1, 0);
    ASSERT_GT(ncon, 0);

    // The SDF search can also return edge contacts. Check its deepest contact;
    // mesh-SDF searches the bottom triangles, so check all of those contacts.
    int deepest = 0;
    for (int i = 1; i < ncon; ++i) {
      if (con[i].dist < con[deepest].dist) {
        deepest = i;
      }
    }

    // The facing surfaces are at z = 0.45 and z = 0.50.
    // Penetration is -0.05 and midpoint is (0.45 + 0.50) / 2 = 0.475.
    for (int i = 0; i < ncon; ++i) {
      if (type != mjGEOM_MESH && i != deepest) {
        continue;
      }
      EXPECT_NEAR(con[i].dist, -0.05, MjTol(2e-3, 5e-3));
      EXPECT_NEAR(con[i].pos[2], 0.475, MjTol(2e-3, 5e-3));
      EXPECT_NEAR(con[i].normal[0], 0.0, 5e-3);
      EXPECT_NEAR(con[i].normal[1], 0.0, 5e-3);
      EXPECT_NEAR(con[i].normal[2], -1.0, MjTol(1e-3, 5e-3));
    }
  }
}

// An exact slab SDF isolates reconstruction from octree interpolation. The
// supplied cube mesh bounds the patch of the two parallel faces being tested.
int RegisterSlabSdf() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "mujoco.test.sdf_slab";
  plugin.capabilityflags = mjPLUGIN_SDF;
  plugin.nstate = +[](const mjModel*, int) { return 0; };
  plugin.init = +[](const mjModel*, mjData*, int) { return 0; };
  plugin.reset = +[](const mjModel*, mjtNum*, void*, int) {};
  plugin.compute = +[](const mjModel*, mjData*, int, int) {};
  plugin.sdf_distance = +[](const mjtNum p[3], const mjData*, int) -> mjtNum {
    return mju_abs(p[2]) - 0.5;
  };
  plugin.sdf_gradient = +[](mjtNum g[3], const mjtNum p[3], const mjData*, int) {
    g[0] = g[1] = 0;
    g[2] = p[2] > 0 ? 1 : -1;
  };
  return mjp_registerPlugin(&plugin);
}

TEST_F(SdfTest, SdfSdfParallelSurfaces) {
  static const int slot = RegisterSlabSdf();
  ASSERT_GE(slot, 0);
  static constexpr char kXml[] = R"(
  <mujoco>
    <extension>
      <plugin plugin="mujoco.test.sdf_slab"/>
    </extension>
    <asset>
      <mesh name="cube" vertex="-.5 -.5 -.5  .5 -.5 -.5  -.5 .5 -.5  .5 .5 -.5
                               -.5 -.5  .5  .5 -.5  .5  -.5 .5  .5  .5 .5  .5"/>
    </asset>
    <worldbody>
      <body name="upper" pos="0 0 .8">
        <geom type="sdf" mesh="cube"><plugin plugin="mujoco.test.sdf_slab"/></geom>
      </body>
      <body name="lower">
        <geom type="sdf" mesh="cube"><plugin plugin="mujoco.test.sdf_slab"/></geom>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  MjModelPtr model = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());

  // Exercise both relative and world rotations, translation, and geom order.
  const mjtNum quats[][4] = {{1, 0, 0, 0}, {0.5, 0.5, 0.5, 0.5}, {1, 2, 3, 4}};
  const mjtNum origin[3] = {1.2, -2.3, 0.7};
  const mjtNum zaxis[3] = {0, 0, 1};
  const mjtNum flip[4] = {0, 1, 0, 0};
  for (const auto& rotation : quats) {
    mjtNum axis[3], quat[4];
    mju_copy4(quat, rotation);
    mju_normalize4(quat);
    mju_rotVecQuat(axis, zaxis, quat);
    mju_copy4(model->body_quat + 4, quat);
    mju_mulQuat(model->body_quat + 8, quat, flip);
    mju_copy3(model->body_pos + 6, origin);
    for (mjtNum depth : {-0.02, 0.02, 0.2, 0.4}) {
      SCOPED_TRACE(depth);
      mju_addScl3(model->body_pos + 3, origin, axis, 1 - depth);
      mj_forward(model.get(), data.get());
      for (int swap : {0, 1}) {
        SCOPED_TRACE(swap);
        mjPreContact con[mjMAXCONPAIR];
        int ncon = mjCOLLISIONFUNC[mjGEOM_SDF][mjGEOM_SDF](
            model.get(), data.get(), con, swap, 1 - swap, 0);
        if (depth < 0) {
          EXPECT_EQ(ncon, 0);
          continue;
        }
        ASSERT_GT(ncon, 0);
        for (int i = 0; i < ncon; ++i) {
          SCOPED_TRACE(i);
          EXPECT_NEAR(con[i].dist, -depth, MjTol(1e-9, 2e-6));
          mjtNum pos[3];
          mju_sub3(pos, con[i].pos, origin);
          EXPECT_NEAR(mju_dot3(pos, axis), (1 - depth) / 2, MjTol(1e-9, 2e-6));
          for (int k = 0; k < 3; ++k) {
            EXPECT_NEAR(con[i].normal[k], (swap ? 1 : -1) * axis[k],
                        MjTol(1e-9, 2e-6));
          }
        }
      }
    }
  }
}

struct SdfShape {
  const char* name;
  mjtNum separation;
  bool flip_upper;
  bool flip_lower;
};

void PrintTo(const SdfShape& shape, std::ostream* os) { *os << shape.name; }

class SdfPluginTest : public MujocoTest,
                      public ::testing::WithParamInterface<SdfShape> {};

TEST_P(SdfPluginTest, ContactReconstruction) {
  const auto& shape = GetParam();
#ifdef mjUSESINGLE
  if (std::string(shape.name) == "bowl") {
    GTEST_SKIP() << "Bowl's 1e-8 finite-difference step vanishes at unit scale in float";
  }
#endif
  const mjtNum identity[4] = {1, 0, 0, 0};
  const mjtNum flip[4] = {0, 1, 0, 0};
  const mjtNum zaxis[3] = {0, 0, 1};
  std::string name = shape.name;
  std::string config =
      name == "gear"
          ? "<config key='diameter' value='1'/><config key='teeth' value='12'/>"
            "<config key='innerdiameter' value='.25'/>"
          : "";
  std::string xml =
      "<mujoco><option sdf_initpoints='40' sdf_iterations='100'/>"
      "<extension><plugin plugin='mujoco.sdf." + name + "'><instance name='shape'>" +
      config + "</instance></plugin></extension>"
      "<asset><mesh name='shape'><plugin instance='shape'/></mesh></asset>"
      "<worldbody><body><geom type='sdf' mesh='shape'>"
      "<plugin instance='shape'/></geom></body>"
      "<body><geom type='sdf' mesh='shape'><plugin instance='shape'/></geom></body>"
      "</worldbody></mujoco>";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml.c_str(), error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());

  for (mjtNum depth : {.005, .02, .04}) {
    SCOPED_TRACE(depth);
    for (bool rotated : {false, true}) {
      SCOPED_TRACE(rotated);
      mjtNum quat[4] = {1, 0, 0, 0}, origin[3] = {0, 0, 0};
      if (rotated) {
        quat[1] = 2;
        quat[2] = 3;
        quat[3] = 4;
        mju_normalize4(quat);
        origin[0] = .7;
        origin[1] = -.4;
        origin[2] = .2;
      }
      mjtNum axis[3], midpoint[3];
      mju_rotVecQuat(axis, zaxis, quat);
      mju_mulQuat(model->body_quat + 4, quat, shape.flip_upper ? flip : identity);
      mju_mulQuat(model->body_quat + 8, quat, shape.flip_lower ? flip : identity);
      mju_addScl3(model->body_pos + 3, origin, axis, shape.separation - depth);
      mju_copy3(model->body_pos + 6, origin);
      mju_addScl3(midpoint, origin, axis, (shape.separation - depth) / 2);
      mj_forward(model.get(), data.get());

      for (int swap : {0, 1}) {
        SCOPED_TRACE(swap);
        mjPreContact con[mjMAXCONPAIR];
        int ncon = mjCOLLISIONFUNC[mjGEOM_SDF][mjGEOM_SDF](
            model.get(), data.get(), con, swap, 1 - swap, 0);
        ASSERT_GT(ncon, 0);
        int deepest = 0;
        for (int i = 1; i < ncon; ++i) {
          if (con[i].dist < con[deepest].dist) {
            deepest = i;
          }
        }
        // Edge contacts need not attain the axial overlap. The deepest
        // contact should, up to the finite search error on curved surfaces.
        mjtNum depth_tol = name == "bowl" ? MjTol(2e-4, 5e-4) : MjTol(2e-6, 1e-4);
        EXPECT_NEAR(con[deepest].dist, -depth, depth_tol);
        mjtNum delta[3];
        mju_sub3(delta, con[deepest].pos, midpoint);
        EXPECT_NEAR(mju_dot3(delta, axis), 0, MjTol(2e-6, 1e-4));
        for (int k = 0; k < 3; ++k) {
          EXPECT_NEAR(con[deepest].normal[k], (swap ? 1 : -1) * axis[k],
                      MjTol(1e-3, 1e-2));
        }
      }
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    Shapes, SdfPluginTest,
    ::testing::Values(SdfShape{"torus", .3, false, false},
                      SdfShape{"gear", .2, false, false},
                      SdfShape{"bowl", 2.04, false, true},
                      SdfShape{"bolt", 0, true, false},
                      SdfShape{"nut", 0, true, false}),
    [](const ::testing::TestParamInfo<SdfShape>& info) { return info.param.name; });

TEST_F(SdfTest, SdfCollisionExcessContactsNoCrash) {
  static constexpr char kSdfSdfXml[] = R"(
  <mujoco>
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
  // More starting points than contact slots must stop before writing past the
  // buffer.
  model->opt.sdf_initpoints = 2 * mjMAXCONPAIR;
  MjDataPtr data = MakeData(model);
  ASSERT_THAT(data, NotNull());
  mj_forward(model.get(), data.get());

  mjPreContact con[mjMAXCONPAIR];
  int ncon = mjCOLLISIONFUNC[mjGEOM_SDF][mjGEOM_SDF](model.get(), data.get(),
                                                     con, 0, 1, 0.0);
  EXPECT_EQ(ncon, mjMAXCONPAIR);
}

}  // namespace
}  // namespace mujoco
