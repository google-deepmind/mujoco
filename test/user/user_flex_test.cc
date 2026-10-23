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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::Pointwise;
using UserFlexTest = MujocoTest;

// SNH is selected through mjSpec, with StVK remaining the default for MJCF.
TEST_F(UserFlexTest, SNHSpecOnly) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <flexcomp name="old" type="grid" dim="3" count="2 2 2" spacing="1 1 1">
        <contact contype="0" conaffinity="0" selfcollide="none"/>
        <elasticity young="1000" poisson=".3"/>
      </flexcomp>
      <flexcomp name="new" type="grid" dim="3" count="2 2 2" spacing="1 1 1" pos="3 0 0">
        <contact contype="0" conaffinity="0" selfcollide="none"/>
        <elasticity young="1000" poisson=".3"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
  SpecPtr spec(mj_parseXMLString(xml, nullptr, nullptr, 0), mj_deleteSpec);
  ASSERT_THAT(spec.get(), NotNull());
  mjsFlex* flex = mjs_asFlex(mjs_findElement(spec.get(), mjOBJ_FLEX, "new"));
  ASSERT_THAT(flex, NotNull());
  EXPECT_EQ(flex->elastic3d, 0);
  MjModelPtr initial(mj_compile(spec.get(), nullptr));
  ASSERT_THAT(initial.get(), NotNull()) << mjs_getError(spec.get());
  for (int t = 0; t < initial->nflexelem; t++) {
    for (int i = 21; i < 24; i++)
      EXPECT_EQ(initial->flex_stiffness[24 * t + i], 0);
  }

  // Recompilation and spec copying retain the programmatic selection.
  flex->elastic3d = 1;
  spec->option.integrator = mjINT_DISCRETE;
  SpecPtr copy(mj_copySpec(spec.get()), mj_deleteSpec);
  ASSERT_THAT(copy.get(), NotNull());
  EXPECT_EQ(
      mjs_asFlex(mjs_findElement(copy.get(), mjOBJ_FLEX, "new"))->elastic3d, 1);
  MjModelPtr m(mj_compile(copy.get(), nullptr));
  ASSERT_THAT(m.get(), NotNull()) << mjs_getError(copy.get());
  ASSERT_EQ(m->nflexstiffness, 24 * m->nflexelem);
  for (int f = 0; f < 2; f++) {
    for (int t = 0; t < m->flex_elemnum[f]; t++) {
      const mjtNum* k = m->flex_stiffness + m->flex_stiffnessadr[f] + 24 * t;
      if (f == 0) {
        EXPECT_EQ(k[21], 0);
        EXPECT_EQ(k[22], 0);
        EXPECT_EQ(k[23], 0);
      } else {
        EXPECT_LT(k[21], 0);
        EXPECT_GT(k[22], 0);
        EXPECT_NE(k[23], 0);
      }
    }
  }

  // Reflection preserves every edge length. StVK has zero force, SNH does not.
  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());
  for (int v = 0; v < m->nflexvert; v++) {
    int adr = m->body_dofadr[m->flex_vertbodyid[v]];
    d->qpos[adr + 2] = -2 * d->flexvert_xpos[3 * v + 2];
  }
  mj_forward(m.get(), d.get());
  mjtNum snh_force = 0;
  for (int f = 0; f < 2; f++) {
    for (int v = 0; v < m->flex_vertnum[f]; v++) {
      int adr = m->body_dofadr[m->flex_vertbodyid[m->flex_vertadr[f] + v]];
      for (int x = 0; x < 3; x++) {
        if (f == 0) {
          EXPECT_NEAR(d->qfrc_spring[adr + x], 0, MjTol(1e-10, 1e-3));
        } else {
          snh_force += mju_abs(d->qfrc_spring[adr + x]);
        }
      }
    }
  }
  EXPECT_GT(snh_force, 1);

  // The compiled coefficients preserve the choice in MJB without a model flag.
  std::vector<char> buffer(mj_sizeModel(m.get()));
  mj_saveModel(m.get(), nullptr, buffer.data(), buffer.size());
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  ASSERT_EQ(mj_addBufferVFS(&vfs, "mixed.mjb", buffer.data(), buffer.size()),
            0);
  MjModelPtr restored(mj_loadModel("mixed.mjb", &vfs));
  mj_deleteVFS(&vfs);
  ASSERT_THAT(restored.get(), NotNull());
  ASSERT_EQ(restored->nflexstiffness, m->nflexstiffness);
  for (int i = 0; i < m->nflexstiffness; i++) {
    EXPECT_EQ(restored->flex_stiffness[i], m->flex_stiffness[i]);
  }
  MjDataPtr restored_data = MakeData(restored);
  mju_copy(restored_data->qpos, d->qpos, m->nq);
  mj_forward(restored.get(), restored_data.get());
  for (int i = 0; i < m->nv; i++)
    EXPECT_EQ(restored_data->qfrc_spring[i], d->qfrc_spring[i]);

  // MJCF cannot represent the choice: report an error instead of changing the
  // material.
  std::array<char, 32768> output;
  std::array<char, 1024> error;
  EXPECT_EQ(mj_saveXMLString(copy.get(), output.data(), output.size(),
                             error.data(), error.size()),
            -1);
  EXPECT_THAT(error.data(), HasSubstr("mjSpec-only"));
  flex->elastic3d = 0;
  MjModelPtr reset(mj_compile(spec.get(), nullptr));
  ASSERT_THAT(reset.get(), NotNull());
  EXPECT_EQ(reset->flex_stiffness[reset->flex_stiffnessadr[1] + 21], 0);
  EXPECT_EQ(mj_saveXMLString(spec.get(), output.data(), output.size(),
                             error.data(), error.size()),
            0);
}

TEST_F(UserFlexTest, Elastic3DNotInMJCF) {
  for (const char* xml : {
           R"(<mujoco>
                <worldbody>
                  <flexcomp name="test" dim="3">
                    <elasticity young="1000" elastic3d="1"/>
                  </flexcomp>
                </worldbody>
              </mujoco>)",
           R"(<mujoco>
                <deformable>
                  <flex name="test" dim="3" body="world">
                    <elasticity young="1000" elastic3d="1"/>
                  </flex>
                </deformable>
              </mujoco>)"}) {
    std::array<char, 1024> error;
    MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
    EXPECT_THAT(m.get(), IsNull());
    EXPECT_THAT(error.data(), HasSubstr("unrecognized attribute: 'elastic3d'"));
  }
}

TEST_F(UserFlexTest, Elastic3DInvalidSelector) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="test" dim="3" count="2 2 2">
        <elasticity young="1000"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  mjSpec* spec = mj_parseXMLString(xml, nullptr, nullptr, 0);
  ASSERT_THAT(spec, NotNull());
  mjsFlex* flex = mjs_asFlex(mjs_findElement(spec, mjOBJ_FLEX, "test"));
  for (int value : {-1, 2}) {
    flex->elastic3d = value;
    MjModelPtr m(mj_compile(spec, nullptr));
    EXPECT_THAT(m.get(), IsNull());
    EXPECT_THAT(mjs_getError(spec),
                HasSubstr("elastic3d must be 0 (StVK) or 1 (SNH)"));
  }
  mj_deleteSpec(spec);
}

TEST_F(UserFlexTest, SNHRequiresStandard3D) {
  for (const char* attributes :
       {"dim='2' count='2 2 1'", "dim='3' count='2 2 2' dof='trilinear'"}) {
    std::string xml =
        "<mujoco>\n"
        "  <worldbody>\n"
        "    <flexcomp name='test' " +
        std::string(attributes) +
        ">\n"
        "      <contact selfcollide='none'/>\n"
        "      <elasticity young='1000'/>\n"
        "    </flexcomp>\n"
        "  </worldbody>\n"
        "</mujoco>";
    mjSpec* spec = mj_parseXMLString(xml.c_str(), nullptr, nullptr, 0);
    ASSERT_THAT(spec, NotNull());
    mjs_asFlex(mjs_findElement(spec, mjOBJ_FLEX, "test"))->elastic3d = 1;
    MjModelPtr m(mj_compile(spec, nullptr));
    EXPECT_THAT(m.get(), IsNull());
    EXPECT_THAT(mjs_getError(spec),
                HasSubstr("requires a non-interpolated 3d flex"));
    mj_deleteSpec(spec);
  }
}

TEST_F(UserFlexTest, SNHRequiresDiscreteIntegrator) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="test" dim="3" count="2 2 2">
        <contact contype="0" conaffinity="0" selfcollide="none"/>
        <elasticity young="1000"/>
      </flexcomp>
    </worldbody>
  </mujoco>)";
  mjSpec* spec = mj_parseXMLString(xml, nullptr, nullptr, 0);
  ASSERT_THAT(spec, NotNull());
  mjs_asFlex(mjs_findElement(spec, mjOBJ_FLEX, "test"))->elastic3d = 1;
  for (int integrator :
       {mjINT_EULER, mjINT_RK4, mjINT_IMPLICIT, mjINT_IMPLICITFAST}) {
    SCOPED_TRACE(integrator);
    spec->option.integrator = integrator;
    MjModelPtr m(mj_compile(spec, nullptr));
    EXPECT_THAT(m.get(), IsNull());
    EXPECT_THAT(
        mjs_getError(spec),
        HasSubstr(
            "stable Neo-Hookean elasticity requires integrator='discrete'"));
  }
  spec->option.integrator = mjINT_DISCRETE;
  for (int solver : {mjSOL_CG, mjSOL_NEWTON}) {
    spec->option.solver = solver;
    MjModelPtr m(mj_compile(spec, nullptr));
    EXPECT_THAT(m.get(), NotNull()) << mjs_getError(spec);
  }
  mj_deleteSpec(spec);
}

TEST_F(UserFlexTest, ParentMustHaveName) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(error.data(), HasSubstr("required attribute missing: 'name'"));
}

TEST_F(UserFlexTest, InvalidDim) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" dim="4"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Invalid dim, must be between 1 and 3"));
}

TEST_F(UserFlexTest, CountTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" count="2 2 0"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Count too small"));
}

TEST_F(UserFlexTest, CellnumZeroInterpolated) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <body name="b0"/>
    <body name="b1"/>
    <body name="b2"/>
    <body name="b3"/>
  </worldbody>
  <deformable>
    <flex name="test" cellcount="2 2 0" dof="trilinear"
          dim="3" body="b0 b1 b2 b3" element="0 1 2 3"/>
  </deformable>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(error.data(), HasSubstr("cellcount cannot be 0"));
}

TEST_F(UserFlexTest, SpacingGreaterThanGeometry) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" spacing="0.5 0.5 0.5" radius="1"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("Spacing must be larger than geometry size"));
}

TEST_F(UserFlexTest, ScaleMinValue) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" scale="0 1 1"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("Scale must be larger than mjMINVAL"));
}

TEST_F(UserFlexTest, MassMinValue) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" mass="0"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("Mass and inertiabox must be larger than mjMINVAL"));
}

TEST_F(UserFlexTest, PointSizeNotMultipleOf3) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" point="0 0 0 0"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("Point size must be a multiple of 3"));
}

TEST_F(UserFlexTest, ElementSize) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" point="0 0 0" element="0 1 2" dim="3"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("Element size must be a multiple of dim+1"));
}

TEST_F(UserFlexTest, PointAndElementNotInDirect) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="direct"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("Point and element required"));
}

TEST_F(UserFlexTest, UnknownFlexCompType) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="unknown"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("invalid keyword: 'unknown'"));
}

TEST_F(UserFlexTest, InvalidPinid) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="direct" point="0 0 0" element="0 1 2">
      <pin id="1"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("element 1 has point id 1, number of points is 1"));
}

TEST_F(UserFlexTest, MeshFileMissing) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="mesh"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("File is required"));
}

TEST_F(UserFlexTest, VertexOrFaceDataMissing) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/malformed_flex_nofaces.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(), HasSubstr("Vertex and face data required"));
}

TEST_F(UserFlexTest, CreateBVHSuccess) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/robot_arm.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, RigidFlex) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/rigid_flex.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, FlexNotCollide) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="test" pos="1 0 -1" type="grid"
                count="5 5 5" spacing="1 1 1" dim="3">
        <contact contype="0" conaffinity="0"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);
  mj_step(m.get(), d.get());
}

TEST_F(UserFlexTest, BoundingBoxCoordinates) {
#ifdef mjUSESINGLE
  GTEST_SKIP()
      << "Float32 rounding in bounding box centering gives ~3e-8 error";
#endif
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" pos="1 0 -1" type="grid"
              count="5 5 5" spacing="1 1 1" dim="3"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);
  mj_kinematics(m.get(), d.get());
  mj_flex(m.get(), d.get());

  EXPECT_EQ(m->nflexvert, 5 * 5 * 5);
  EXPECT_EQ(m->nflexelem, 4 * 4 * 4 * 6);
  EXPECT_EQ(m->flex_dim[0], 3);

  // Cartesian coordinates
  EXPECT_EQ(d->flexvert_xpos[0], -1);
  EXPECT_EQ(d->flexvert_xpos[1], -2);
  EXPECT_EQ(d->flexvert_xpos[2], -3);
  EXPECT_EQ(d->flexvert_xpos[3 * m->nflexvert - 3], 3);
  EXPECT_EQ(d->flexvert_xpos[3 * m->nflexvert - 2], 2);
  EXPECT_EQ(d->flexvert_xpos[3 * m->nflexvert - 1], 1);

  // bounding box coordinates
  EXPECT_EQ(m->flex_vert0[0], 0);
  EXPECT_EQ(m->flex_vert0[1], 0);
  EXPECT_EQ(m->flex_vert0[2], 0);
  EXPECT_EQ(m->flex_vert0[3 * m->nflexvert - 3], 1);
  EXPECT_EQ(m->flex_vert0[3 * m->nflexvert - 2], 1);
  EXPECT_EQ(m->flex_vert0[3 * m->nflexvert - 1], 1);
}

TEST_F(UserFlexTest, InterpolatedFlexCanSelfCollide) {
  for (const char* dof : {"trilinear", "quadratic"}) {
    std::string xml = std::string(R"(
    <mujoco>
    <worldbody>
      <flexcomp name="test" type="grid" count="3 3 3" spacing="1 1 1" dim="3"
                dof=")") +
                      dof + R"(">
        <contact selfcollide="auto"/>
      </flexcomp>
    </worldbody>
    </mujoco>
    )";
    std::array<char, 1024> error;
    MjModelPtr m = LoadModelFromString(xml.c_str(), error.data(), error.size());
    ASSERT_THAT(m.get(), NotNull()) << dof << ": " << error.data();
    EXPECT_NE(m->flex_interp[0], 0) << dof;
    EXPECT_EQ(m->flex_selfcollide[0], mjFLEXSELF_AUTO) << dof;
    MjDataPtr d = MakeData(m);
    mj_step(m.get(), d.get());
  }
}

// cellcount sizes the nodal grid of every flexcomp type
TEST_F(UserFlexTest, InterpolatedCellcountAllTypes) {
  const char* types[][2] = {
      {"grid", "3"}, {"box", "2"}, {"cylinder", "2"}, {"ellipsoid", "2"}};
  for (const auto& [type, dim] : types) {
    for (const char* dof : {"trilinear", "quadratic"}) {
      std::string xml = std::string(R"(
      <mujoco>
      <worldbody>
        <flexcomp name="test" type=")") +
                        type + R"(" count="4 4 4" spacing=".1 .1 .1" dim=")" +
                        dim + R"(" dof=")" + dof + R"(" cellcount="2 3 2">
          <contact selfcollide="none"/>
        </flexcomp>
      </worldbody>
      </mujoco>
      )";
      std::array<char, 1024> error;
      MjModelPtr m =
          LoadModelFromString(xml.c_str(), error.data(), error.size());
      ASSERT_THAT(m.get(), NotNull())
          << type << ", " << dof << ": " << error.data();
      int order = std::abs(m->flex_interp[0]);
      EXPECT_EQ(m->flex_nodenum[0],
                (2 * order + 1) * (3 * order + 1) * (2 * order + 1))
          << type << ", " << dof;
      MjDataPtr d = MakeData(m);
      mj_step(m.get(), d.get());
    }
  }
}

TEST_F(UserFlexTest, TrilinearInterpolation) {
  static constexpr char xml_trilinear[] = R"(
  <mujoco>
  <worldbody>
    <geom type="plane" pos="0 0 -.5" size="10 10 .1"/>
    <flexcomp name="test" type="grid" count="2 2 2" spacing="1 1 1" dim="3" dof="trilinear">
      <contact selfcollide="none"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m1 =
      LoadModelFromString(xml_trilinear, error.data(), error.size());
  ASSERT_THAT(m1.get(), NotNull()) << error.data();
  MjDataPtr d1 = MakeData(m1);
  mj_step(m1.get(), d1.get());

  static constexpr char xml_linear[] = R"(
  <mujoco>
  <worldbody>
    <geom type="plane" pos="0 0 -.5" size="10 10 .1"/>
    <flexcomp name="test" type="grid" count="2 2 2" spacing="1 1 1" dim="3">
      <contact selfcollide="none"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  MjModelPtr m2 = LoadModelFromString(xml_linear, error.data(), error.size());
  ASSERT_THAT(m2.get(), NotNull()) << error.data();
  MjDataPtr d2 = MakeData(m2);
  mj_step(m2.get(), d2.get());

  EXPECT_EQ(m1->nflexvert, m2->nflexvert);
  for (int i = 0; i < 3 * m1->nflexvert; ++i) {
    EXPECT_NEAR(m1->flex_vert[i], d2->flexvert_xpos[i], 1e-7);
    EXPECT_NEAR(m1->flex_vert0[i], m2->flex_vert0[i], 1e-7);
    EXPECT_NEAR(d1->flexvert_xpos[i], d2->flexvert_xpos[i], 1e-7);
  }

  EXPECT_EQ(m1->nC, m2->nC);
  for (int i = 0; i < m1->nC; ++i) {
    EXPECT_EQ(d1->M[i], d2->M[i]);
  }

  EXPECT_EQ(m1->nbody, m2->nbody);
  for (int i = 0; i < m2->nbody; ++i) {
    if (i == 0) {
      continue;
    }
    EXPECT_EQ(m1->body_mass[i], m2->body_mass[i]);
    for (int j = 0; j < 2; ++j) {
      EXPECT_EQ(m1->body_invweight0[i * 2 + j], m2->body_invweight0[i * 2 + j]);
    }
    for (int j = 0; j < 10; ++j) {
      EXPECT_NEAR(d1->cinert[10 * i + j], d2->cinert[i * 10 + j], 1e-5) << i;
      EXPECT_NEAR(d1->crb[10 * i + j], d2->crb[i * 10 + j], 1e-5) << i;
    }
  }

  EXPECT_EQ(d1->ncon, 4);
  EXPECT_EQ(d2->ncon, 4);
  for (int i = 0; i < d1->ncon; ++i) {
    EXPECT_EQ(d1->contact[i].dist, d2->contact[i].dist);
    EXPECT_EQ(d1->contact[i].mu, d2->contact[i].mu);
    for (int j = 0; j < 5; ++j) {
      EXPECT_EQ(d1->contact[i].friction[j], d2->contact[i].friction[j]);
    }
    for (int j = 0; j < 3; ++j) {
      EXPECT_EQ(d1->contact[i].pos[j], d2->contact[i].pos[j]);
    }
    for (int j = 0; j < 9; ++j) {
      EXPECT_EQ(d1->contact[i].frame[j], d2->contact[i].frame[j]);
    }
    for (int j = 0; j < 36; ++j) {
      EXPECT_EQ(d1->contact[i].H[j], d2->contact[i].H[j]);
    }
  }

  EXPECT_EQ(d1->nefc, 4 * (d1->contact[0].dim - 1) * 2);
  EXPECT_EQ(d2->nefc, 4 * (d2->contact[0].dim - 1) * 2);
  EXPECT_EQ(d1->nJ, d2->nJ);
  for (int i = 0; i < d1->nefc; ++i) {
    EXPECT_EQ(d1->efc_diagA[i], d2->efc_diagA[i]);
    EXPECT_EQ(d1->efc_D[i], d2->efc_D[i]);
  }
}

TEST_F(UserFlexTest, StiffnessMatrix) {
#ifdef mjUSESINGLE
  GTEST_SKIP() << "Stiffness matrix kernel check fails in float32 precision";
#endif
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="grid" count="3 3 3" spacing="1 1 1" dim="3" dof="trilinear">
      <contact selfcollide="none"/>
      <elasticity young="1"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  EXPECT_NE(m->flex_stiffness[m->flex_stiffnessadr[0]], 0);
  EXPECT_EQ(m->nflexnode, 8);

  // constants are in the kernel
  mjtNum ones[24], zeros[24], res[24];
  for (int i = 0; i < 3 * m->nflexnode; ++i) {
    zeros[i] = 0;
    ones[i] = 1;
  }
  mju_mulMatVec(res, m->flex_stiffness + m->flex_stiffnessadr[0], ones,
                3 * m->nflexnode, 3 * m->nflexnode);
  EXPECT_THAT(res, Pointwise(MjNear(1e-8, 1e-4), zeros));
}

TEST_F(UserFlexTest, StiffnessCacheDiffersByGeometry) {
  std::array<char, 1024> error;

  // Create two flexes with same material but different bounding boxes
  static constexpr char xml_small[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="grid" count="3 3 3" spacing="1 1 1" dim="3" dof="trilinear">
      <contact selfcollide="none"/>
      <elasticity young="1" poisson="0.3"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";

  static constexpr char xml_large[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="grid" count="3 3 3" spacing="2 2 2" dim="3" dof="trilinear">
      <contact selfcollide="none"/>
      <elasticity young="1" poisson="0.3"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";

  MjModelPtr m_small =
      LoadModelFromString(xml_small, error.data(), error.size());
  ASSERT_THAT(m_small.get(), NotNull()) << error.data();

  MjModelPtr m_large =
      LoadModelFromString(xml_large, error.data(), error.size());
  ASSERT_THAT(m_large.get(), NotNull()) << error.data();

  // Same number of nodes but different stiffness due to different geometry
  EXPECT_EQ(m_small->nflexnode, m_large->nflexnode);
  EXPECT_NE(m_small->flex_stiffness[m_small->flex_stiffnessadr[0]],
            m_large->flex_stiffness[m_large->flex_stiffnessadr[0]]);
}

TEST_F(UserFlexTest, LoadTexture) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/textured_torus_flex.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  EXPECT_THAT(m->nflextexcoord, 637);
  EXPECT_THAT(m->flex_elemtexcoord[0], 0);
  EXPECT_THAT(m->flex_elemtexcoord[1], 1);
  EXPECT_THAT(m->flex_elemtexcoord[2], 2);
  EXPECT_THAT(m->flex_elemtexcoord[3], 0);
  EXPECT_THAT(m->flex_elemtexcoord[4], 2);
  EXPECT_THAT(m->flex_elemtexcoord[5], 3);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHBinary_41_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_41_binary_vol_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHBinary_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_22_binary_vol_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHSurfaceBinary_41_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_41_binary_surf_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 2);

  // first node x y z
  EXPECT_EQ(d->flexvert_xpos[0], -0.5);
  EXPECT_EQ(d->flexvert_xpos[1], -0.5);
  EXPECT_EQ(d->flexvert_xpos[2], 0);

  // first element
  EXPECT_EQ(m->flex_elem[0], 9 - 1);
  EXPECT_EQ(m->flex_elem[1], 4 - 1);
  EXPECT_EQ(m->flex_elem[2], 3 - 1);

  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHSurfaceBinary_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_22_binary_surf_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 2);

  // first node x y z
  EXPECT_EQ(d->flexvert_xpos[0], -0.5);
  EXPECT_EQ(d->flexvert_xpos[1], -0.5);
  EXPECT_EQ(d->flexvert_xpos[2], 0);

  // first element
  EXPECT_EQ(m->flex_elem[0], 9 - 1);
  EXPECT_EQ(m->flex_elem[1], 4 - 1);
  EXPECT_EQ(m->flex_elem[2], 3 - 1);

  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHBinaryFTETWILD_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/shark_22_binary_vol_fTetWild.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 429);
  EXPECT_EQ(m->nflexelem, 1073);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_41_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_41_ascii_vol_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_22_ascii_vol_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHSurfaceASCII_41_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_41_ascii_surf_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 2);

  // first node x y z
  EXPECT_EQ(d->flexvert_xpos[0], -0.5);
  EXPECT_EQ(d->flexvert_xpos[1], -0.5);
  EXPECT_EQ(d->flexvert_xpos[2], 0);

  // first element
  EXPECT_EQ(m->flex_elem[0], 9 - 1);
  EXPECT_EQ(m->flex_elem[1], 4 - 1);
  EXPECT_EQ(m->flex_elem[2], 3 - 1);

  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHSurfaceASCII_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/cube_22_ascii_surf_gmshApp.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  EXPECT_EQ(m->flex_dim[0], 2);

  // first node x y z
  EXPECT_EQ(d->flexvert_xpos[0], -0.5);
  EXPECT_EQ(d->flexvert_xpos[1], -0.5);
  EXPECT_EQ(d->flexvert_xpos[2], 0);

  // first element
  EXPECT_EQ(m->flex_elem[0], 9 - 1);
  EXPECT_EQ(m->flex_elem[1], 4 - 1);
  EXPECT_EQ(m->flex_elem[2], 3 - 1);

  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCIIFTETWILD_22_Success) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/shark_22_ascii_vol_fTetWild.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  EXPECT_EQ(m->nflexvert, 425);
  EXPECT_EQ(m->nflexelem, 1070);
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_41_MissingNodeHeader_Fail) {
  const std::string xml_path = GetTestDataFilePath(
      "user/testdata/malformed_cube_41_ascii_missing_node_header.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("All nodes must be in single block"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_41_MissingNodeIndex_Fail) {
  const std::string xml_path = GetTestDataFilePath(
      "user/testdata/malformed_cube_41_ascii_missing_node_index.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Node tags must be sequential"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_41_MissingElementHeader_Fail) {
  const std::string xml_path = GetTestDataFilePath(
      "user/testdata/malformed_cube_41_ascii_missing_element_header.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("All elements must be in single block"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_41_MissingElement_Fail) {
  const std::string xml_path = GetTestDataFilePath(
      "user/testdata/malformed_cube_41_ascii_missing_element.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Error reading Elements"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest,
       LoadMSHASCII_41_MismatchBetweenMaxNodesAndNodesInBlock_Fail) {
  const std::string xml_path = GetTestDataFilePath(
      "user/testdata/"
      "malformed_cube_41_ascii_mismatch_between_max_nodes_and_nodes_in_block."
      "xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Maximum number of nodes must be equal "
                                      "to number of nodes in a block"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_22_MissingNumNodes_Fail) {
  // TODO(b/347277884): Assert on warning message. Error message differs on
  // Windows in GH Actions.
  const std::string xml_path = GetTestDataFilePath(
      "user/testdata/malformed_cube_22_ascii_missing_num_nodes.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("GMSH decoder: "));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_22_MissingNode_Fail) {
  const std::string xml_path = GetTestDataFilePath(
      "user/testdata/malformed_cube_22_ascii_missing_node.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Error reading node tags"));
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_22_MissingNumElements_Fail) {
  const std::string xml_path = GetTestDataFilePath(
      "user/testdata/malformed_shark_22_ascii_missing_num_elements.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  // TODO(mohammadhamid): Replace with an assertion about the error message. For
  // some reason, on Windows the error message is different on GH Actions
  EXPECT_THAT(m, IsNull());
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHASCII_22_MissingElement_Fail) {
  const std::string xml_path = GetTestDataFilePath(
      "user/testdata/malformed_cube_22_ascii_missing_element.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Error reading Elements"));
  mj_deleteModel(m);
}

// Resource buffers are not null-terminated, so the GMSH header parser must stay
// within the reported size. A file that is exactly "$MeshFormat" leaves nothing
// after the tag; reading on is a heap overflow.
TEST_F(UserFlexTest, LoadMSHTruncatedHeader_Fail) {
  static constexpr char msh[] = "$MeshFormat";

  // sizeof - 1: the buffer must be added without its terminating null
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "truncated.msh", msh, sizeof(msh) - 1);

  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="test" type="gmsh" dim="3" radius=".001"
                file="truncated.msh"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size(), &vfs);
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(error.data(), HasSubstr("Could not read GMSH file header"));
  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, LoadMSHMissingNodesSection_Fail) {
  static constexpr char msh[] =
      "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n"
      "$Elements\n1 1 1 1\n3 1 4 1\n1 1 1 1 1\n$EndElements\n";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "nonodes.msh", msh, sizeof(msh) - 1);

  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="test" type="gmsh" dim="3" radius=".001"
                file="nonodes.msh"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size(), &vfs);
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(error.data(), HasSubstr("GMSH file missing $Nodes"));
  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, LoadMSHMissingElementsSection_Fail) {
  static constexpr char msh[] =
      "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n"
      "$Nodes\n1 1 1 1\n3 1 0 1\n1\n0 0 0\n$EndNodes\n";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "noelements.msh", msh, sizeof(msh) - 1);

  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="test" type="gmsh" dim="3" radius=".001"
                file="noelements.msh"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size(), &vfs);
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(error.data(), HasSubstr("GMSH file missing $Elements"));
  mj_deleteVFS(&vfs);
}

// The last section marker of a GMSH file need not be followed by a newline.
TEST_F(UserFlexTest, LoadMSHEndMarkerAtEOF_Success) {
  // read a well-formed GMSH file
  const std::string msh_path =
      GetTestDataFilePath("user/testdata/cube_41_ascii_vol_gmshApp.msh");
  FILE* f = fopen(msh_path.c_str(), "rb");
  ASSERT_THAT(f, NotNull()) << "Could not open " << msh_path;
  fseek(f, 0, SEEK_END);
  long msh_size = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::string msh(msh_size, '\0');
  fread(msh.data(), 1, msh_size, f);
  fclose(f);

  // drop the newline after the final $EndElements
  static constexpr char kEndElements[] = "$EndElements";
  size_t end = msh.rfind(kEndElements);
  ASSERT_NE(end, std::string::npos);
  msh.resize(end + sizeof(kEndElements) - 1);

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "cube.msh", msh.data(), msh.size());

  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="test" type="gmsh" dim="3" radius=".001" file="cube.msh">
        <edge equality="true"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size(), &vfs);
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  EXPECT_EQ(m->nflexvert, 14);
  EXPECT_EQ(m->nflexelem, 24);
  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, LoadMSHASCII_dim_missing_in_xml) {
  const std::string xml_path = GetTestDataFilePath(
      "user/testdata/cube_22_ascii_vol_gmshApp_missing_dim.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  EXPECT_EQ(m->flex_dim[0], 3);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, LoadMSHVolumetric_dim_mismatch_fails) {
  // cube_22_ascii_vol_gmshApp.msh is a 3D volumetric mesh.
  // Specifying dim="2" must fail compilation without silent fallback.
  const std::string dir = GetTestDataFilePath("user/testdata");
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler meshdir="%s"/>
    <worldbody>
      <flexcomp name="test" type="gmsh" dim="2" radius=".001"
                file="cube_22_ascii_vol_gmshApp.msh"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> xml_buf;
  snprintf(xml_buf.data(), xml_buf.size(), xml, dir.c_str());
  std::array<char, 1024> error;
  MjModelPtr m =
      LoadModelFromString(xml_buf.data(), error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(
      error.data(),
      HasSubstr("flexcomp dim does not match GMSH mesh dimensionality"));
}

TEST_F(UserFlexTest, LoadMSHSurface_dim_mismatch_fails) {
  // cube_22_ascii_surf_gmshApp.msh is a 2D surface mesh.
  // Specifying dim="3" must fail compilation.
  const std::string dir = GetTestDataFilePath("user/testdata");
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler meshdir="%s"/>
    <worldbody>
      <flexcomp name="test" type="gmsh" dim="3" radius=".001"
                file="cube_22_ascii_surf_gmshApp.msh"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> xml_buf;
  snprintf(xml_buf.data(), xml_buf.size(), xml, dir.c_str());
  std::array<char, 1024> error;
  MjModelPtr m =
      LoadModelFromString(xml_buf.data(), error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(
      error.data(),
      HasSubstr("flexcomp dim does not match GMSH mesh dimensionality"));
}

TEST_F(UserFlexTest, TrilinearUnusedVertices_Crash) {
  // This XML defines 5 points but only uses 4 in the element.
  // The last point (2 2 2) is unused.
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="direct"
              point="0 0 0  1 0 0  0 1 0  0 0 1  2 2 2"
              element="0 1 2 3"
              dof="trilinear" dim="3">
      <contact selfcollide="none"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), testing::NotNull()) << error.data();
}

TEST_F(UserFlexTest, MeshNodePinning) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="direct"
              point="0 0 0  1 0 0  0 1 0  0 0 1  2 2 2"
              element="0 1 2 3"
              dof="trilinear" dim="3">
      <contact selfcollide="none"/>
      <pin id="0"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), testing::NotNull()) << error.data();

  // Verify that node 0 (corner) is pinned
  // Trilinear 3D has 8 nodes.
  // If 0 are pinned, we get 8 bodies.
  // If 1 is pinned, we get 7 bodies (created) + 1 existing (the parent).
  // m->nbody should reflect this. Flex bodies are added to the model.
  // Model has 1 world body + flex bodies.
  EXPECT_EQ(m->nbody, 1 + 7);  // 1 world + 7 flex nodes (1 pinned)
}

TEST_F(UserFlexTest, FlexcompMeshLoadsFromVFS) {
  // read cube.stl from testdata into a buffer
  const std::string stl_path = GetTestDataFilePath("user/testdata/cube.stl");
  FILE* f = fopen(stl_path.c_str(), "rb");
  ASSERT_THAT(f, NotNull()) << "Could not open " << stl_path;
  fseek(f, 0, SEEK_END);
  long stl_size = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::string stl_data(stl_size, '\0');
  fread(stl_data.data(), 1, stl_size, f);
  fclose(f);

  // add the STL data to a VFS
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "cube.stl", stl_data.data(), stl_size);

  // XML that uses flexcomp type="mesh" referencing the VFS file
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="flex_body" pos="0 0 1">
        <flexcomp name="flex_object" type="mesh" file="cube.stl" rigid="true">
          <contact contype="1" conaffinity="1"/>
        </flexcomp>
      </body>
    </worldbody>
  </mujoco>
  )";

  // cleanup
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size(), &vfs);
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  MjDataPtr d = MakeData(m);
  mj_step(m.get(), d.get());
  mj_deleteVFS(&vfs);
}

// Test that flex constraints are preserved when attaching a model
TEST_F(UserFlexTest, FlexAttachConstraintPreserved) {
  // Child model with flex and strain constraint
  static constexpr char flex_xml[] = R"(
  <mujoco>
  <worldbody>
    <body name="flex_parent">
      <flexcomp name="test" type="box"
                spacing=".1 .1 .1" radius="0.001"
                dof="trilinear" mass="1" dim="3">
        <contact selfcollide="none"/>
        <edge equality="strain"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";

  // Parent model that attaches the flex model
  static constexpr char parent_xml[] = R"(
  <mujoco>
  <asset>
    <model name="flex" file="flex.xml"/>
  </asset>
  <worldbody>
    <frame pos="0 0 0.3">
      <attach model="flex" prefix="flex_"/>
    </frame>
  </worldbody>
  </mujoco>
  )";

  // Set up VFS with both XML files
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "flex.xml", flex_xml, sizeof(flex_xml));

  // First verify the standalone flex model has constraints
  std::array<char, 1024> error;
  MjModelPtr m_standalone =
      LoadModelFromString(flex_xml, error.data(), error.size(), vfs.get());
  ASSERT_THAT(m_standalone.get(), NotNull()) << error.data();
  MjDataPtr d_standalone = MakeData(m_standalone);
  mj_forward(m_standalone.get(), d_standalone.get());
  int standalone_neq = m_standalone->neq;
  EXPECT_GT(standalone_neq, 0) << "Standalone flex should have constraints";

  // Now load the parent model which attaches the flex
  MjModelPtr m_attached =
      LoadModelFromString(parent_xml, error.data(), error.size(), vfs.get());
  ASSERT_THAT(m_attached.get(), NotNull()) << error.data();
  MjDataPtr d_attached = MakeData(m_attached);
  mj_forward(m_attached.get(), d_attached.get());

  // THE BUG: flex constraints disappear when attached
  EXPECT_GT(m_attached->neq, 0)
      << "Attached flex should preserve strain constraints";
  EXPECT_EQ(m_attached->neq, standalone_neq)
      << "Attached flex should have same number of constraints as standalone";

  mj_deleteVFS(vfs.get());
}

TEST_F(UserFlexTest, FlexNoConstraintsWarning) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="grid" count="4 4 1" spacing=".2 .2 .2"
              dim="2" radius=".1"/>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("no equality constraints or passive forces"));
}

TEST_F(UserFlexTest, EmptyCellNodePinning) {
  // A 2x2x2 grid with a box mesh that fills all cells.
  // No nodes should be pinned.
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="box" spacing=".1 .1 .1" dim="3"
              dof="trilinear" mass="1" cellcount="2 2 2">
      <contact selfcollide="none"/>
      <elasticity young="1"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();

  // A 2x2x2 grid with trilinear order has (2+1)^3 = 27 node positions.
  int nadr = m->flex_nodeadr[0];
  int nnode = m->flex_nodenum[0];
  EXPECT_EQ(nnode, 27);

  // All cells are occupied by the box, so no node should be pinned.
  int pinned = 0;
  for (int n = nadr; n < nadr + nnode; n++) {
    int bid = m->flex_nodebodyid[n];
    if (m->body_jntnum[bid] == 0) {
      pinned++;
    }
  }
  EXPECT_EQ(pinned, 0);

  // Verify simulation works
  MjDataPtr d = MakeData(m);
  for (int i = 0; i < 10; i++) {
    mj_step(m.get(), d.get());
  }
}

// the elastic mechanisms use 1D rods with bending elasticity: they load and
// simulate stably, the drives move them, and the edge equalities keep the rods
// nearly inextensible
TEST_F(UserFlexTest, MechanismRods) {
  struct Mechanism {
    const char* file;
    const char* flex;
    const char* joint;  // driven joint that must move, or nullptr
    int nstep;
  };
  const Mechanism mechanisms[] = {
      {"flex/mechanisms/cantilever.xml", "rod", nullptr, 500},
      {"flex/mechanisms/slidercrank.xml", "rod", "slide", 1500},
      {"flex/mechanisms/dipper.xml", "arm_flex", "arm", 3000},
  };
  for (const Mechanism& mech : mechanisms) {
    SCOPED_TRACE(mech.file);
    const std::string xml_path = GetModelPath(mech.file);
    std::array<char, 1024> error;
    mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
    ASSERT_THAT(m, NotNull()) << error.data();
    int f = mj_name2id(m, mjOBJ_FLEX, mech.flex);
    ASSERT_GE(f, 0);
    EXPECT_EQ(m->flex_dim[f], 1);
    EXPECT_GE(m->flex_bendingadr[f], 0);

    mjData* d = mj_makeData(m);
    int jnt = mech.joint ? mj_name2id(m, mjOBJ_JOINT, mech.joint) : -1;
    mjtNum q0 = jnt >= 0 ? d->qpos[m->jnt_qposadr[jnt]] : 0;
    mjtNum qmin = q0, qmax = q0, strain = 0;
    for (int i = 0; i < mech.nstep; i++) {
      mj_step(m, d);
      if (jnt >= 0) {
        qmin = std::min(qmin, d->qpos[m->jnt_qposadr[jnt]]);
        qmax = std::max(qmax, d->qpos[m->jnt_qposadr[jnt]]);
      }
      for (int e = m->flex_edgeadr[f];
           e < m->flex_edgeadr[f] + m->flex_edgenum[f]; e++) {
        strain = std::max(
            strain,
            std::abs(d->flexedge_length[e] / m->flexedge_length0[e] - 1));
      }
    }
    EXPECT_EQ(d->warning[mjWARN_BADQACC].number, 0);
    for (int i = 0; i < m->nq; i++) {
      ASSERT_TRUE(std::isfinite(d->qpos[i])) << "qpos " << i;
    }
    EXPECT_LT(strain, 0.01);
    if (jnt >= 0) {
      EXPECT_GT(qmax - qmin, 0.05) << mech.joint;
    }
    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

TEST_F(UserFlexTest, EmptyCellNodePinningMesh) {
  // Load bunny_multicell.xml which has a 3x3x3 grid.
  // The bunny mesh only occupies some cells, so many nodes should be pinned.
  const std::string xml_path = GetModelPath("flex/bunny_multicell.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();

  // 3x3x3 grid, order=1: (3+1)^3 = 64 node positions
  int nadr = m->flex_nodeadr[0];
  int nnode = m->flex_nodenum[0];
  EXPECT_EQ(nnode, 64);

  // Count pinned nodes (no joints)
  int pinned = 0;
  int free_nodes = 0;
  for (int n = nadr; n < nadr + nnode; n++) {
    int bid = m->flex_nodebodyid[n];
    if (m->body_jntnum[bid] == 0) {
      pinned++;
    } else {
      free_nodes++;
    }
  }

  // At least some nodes should be pinned since the bunny doesn't fill all cells
  EXPECT_GT(pinned, 0) << "Expected some nodes to be pinned from empty cells";
  EXPECT_GT(free_nodes, 0) << "Expected some nodes to remain free";
  EXPECT_EQ(pinned + free_nodes, nnode);

  // Verify the model can simulate
  mjData* d = mj_makeData(m);
  mj_forward(m, d);
  for (int i = 0; i < 10; i++) {
    mj_step(m, d);
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, EmptyCellNodePinningQuadratic) {
  // Regression test for ci_min calculation with order=2.
  // A 2x1x1 quadratic grid has nodes at gi=0..4 (5 nodes per axis).
  // We place mesh vertices only in cell 0 (x in [0, 0.5]), so cell 1 is empty.
  //
  // Node gi=3 belongs only to cell 1 (1*2 <= 3 <= 2*2).
  // With the old formula (gi-order)/order = (3-2)/2 = 0, it would also check
  // cell 0 (non-empty), incorrectly marking gi=3 as non-pinned.
  // Single hex element at x=[0,0.3], well inside cell 0 of a 3x1x1 grid.
  // Anchor vertex at x=1.0 extends the bounding box to [0,1]^3.
  // The 3x1x1 quadratic grid splits at x=0.33, 0.67.
  // Cell 0 has vertices, cells 1 and 2 are empty.
  // Interior nodes for cells 1,2 should be pinned to the parent body.
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <body name="parent">
    <freejoint/>
    <inertial mass="0.01" pos="0 0 0"
             diaginertia="0.001 0.001 0.001"/>
    <flexcomp name="test" type="direct" dim="3"
              dof="quadratic" mass="1" cellcount="3 1 1"
              point="0.0 0.0 0.0  0.3 0.0 0.0
                     0.0 1.0 0.0  0.3 1.0 0.0
                     0.0 0.0 1.0  0.3 0.0 1.0
                     0.0 1.0 1.0  0.3 1.0 1.0
                     1.0 0.5 0.5"
              element="0 1 3 2 4 5 7 6">
      <contact selfcollide="none"/>
      <elasticity young="1"/>
    </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();

  // 3x1x1 quadratic grid: (3*2+1) * (1*2+1) * (1*2+1) = 7*3*3 = 63 nodes
  int nadr = m->flex_nodeadr[0];
  int nnode = m->flex_nodenum[0];
  EXPECT_EQ(nnode, 63);

  // Count pinned nodes: pinned nodes are assigned to the parent body.
  int parent_bid = mj_name2id(m.get(), mjOBJ_BODY, "parent");
  ASSERT_GT(parent_bid, 0);
  int pinned = 0;
  for (int n = nadr; n < nadr + nnode; n++) {
    if (m->flex_nodebodyid[n] == parent_bid) {
      pinned++;
    }
  }

  // Cells 1 and 2 are empty, so nodes exclusively in those cells are pinned.
  // Nodes at gi=3..6 (with any gj, gk) are only in cells 1 and/or 2.
  // That's 4 * 3 * 3 = 36 nodes.
  EXPECT_EQ(pinned, 36);

  mj_deleteData(mj_makeData(m.get()));
}

TEST_F(UserFlexTest, EmptyCellDetectsElements) {
  // A cube surface mesh (dim=2, 12 triangles) spanning [0,1]^3.
  // With cellcount="6 6 6" (216 cells), only 8 corner cells contain
  // mesh vertices.
  //
  // Bug: MarkEmptyCells only checked vertices, so 208/216 cells are
  // marked empty, causing most interior nodes to be incorrectly pinned.
  // Fix: check element AABBs to correctly identify occupied cells.
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <body name="parent">
    <freejoint/>
    <inertial mass="0.01" pos="0.5 0.5 0.5"
             diaginertia="0.001 0.001 0.001"/>
    <flexcomp name="test" type="direct" dim="2"
              dof="trilinear" mass="1" cellcount="6 6 6"
              point="0 0 0  1 0 0  1 1 0  0 1 0
                     0 0 1  1 0 1  1 1 1  0 1 1"
              element="0 1 2  0 2 3  4 6 5  4 7 6
                       0 5 1  0 4 5  2 7 3  2 6 7
                       0 3 7  0 7 4  1 5 6  1 6 2">
      <contact selfcollide="none"/>
    </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();

  // 6x6x6 trilinear grid: (6+1)^3 = 343 nodes
  int nadr = m->flex_nodeadr[0];
  int nnode = m->flex_nodenum[0];
  ASSERT_EQ(nnode, 343);

  // Count pinned nodes: those assigned to the parent body.
  int parent_bid = mj_name2id(m.get(), mjOBJ_BODY, "parent");
  ASSERT_GT(parent_bid, 0);
  int pinned = 0;
  for (int n = nadr; n < nadr + nnode; n++) {
    if (m->flex_nodebodyid[n] == parent_bid) {
      pinned++;
    }
  }

  // The cube surface fills the entire bounding box. The element-AABB
  // marks all boundary cells as surface cells (152/216). The interior
  // flood-fill finds no exterior seeds (all boundary cells are surface),
  // so the remaining 64 cells are classified as interior (non-empty).
  // No cells are empty → 0 nodes pinned.
  EXPECT_EQ(pinned, 0);

  mj_deleteData(mj_makeData(m.get()));
}

TEST_F(UserFlexTest, TotalMassTrilinear) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="grid" count="2 2 2" spacing="1 1 1"
              dim="3" dof="trilinear" mass="1.5">
      <contact selfcollide="none"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();

  double total_mass = 0;
  for (int i = 1; i < m->nbody; ++i) {
    total_mass += m->body_mass[i];
  }

  EXPECT_NEAR(total_mass, 1.5, 1e-5);
}

TEST_F(UserFlexTest, TotalMassQuadratic) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="grid" count="3 2 2" spacing="1 1 1"
              dim="3" dof="quadratic" mass="2.0">
      <contact selfcollide="none"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();

  double total_mass = 0;
  for (int i = 1; i < m->nbody; ++i) {
    total_mass += m->body_mass[i];
  }

  EXPECT_NEAR(total_mass, 2.0, 1e-5);
}

TEST_F(UserFlexTest, Dof2d) {
  // 3x3 grid with dof="2d": 9 vertices, 2 DOFs each -> nv = 18
  static constexpr char xml_2d[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="grid" count="3 3 1" spacing=".1 .1 .1"
              dim="2" radius=".01" dof="2d">
      <edge equality="true"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";

  // same model with dof="full" for comparison: 9 vertices, 3 DOFs each -> nv =
  // 27
  static constexpr char xml_full[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="grid" count="3 3 1" spacing=".1 .1 .1"
              dim="2" radius=".01">
      <edge equality="true"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;

  // load 2d model
  MjModelPtr m_2d = LoadModelFromString(xml_2d, error.data(), error.size());
  ASSERT_THAT(m_2d.get(), NotNull()) << error.data();
  MjDataPtr d_2d = MakeData(m_2d);

  // load full model
  MjModelPtr m_full = LoadModelFromString(xml_full, error.data(), error.size());
  ASSERT_THAT(m_full.get(), NotNull()) << error.data();
  MjDataPtr d_full = MakeData(m_full);

  // verify DOF counts
  EXPECT_EQ(m_2d->nv, 18);    // 9 vertices * 2 DOFs
  EXPECT_EQ(m_full->nv, 27);  // 9 vertices * 3 DOFs

  // same number of vertices and elements
  EXPECT_EQ(m_2d->nflexvert, m_full->nflexvert);
  EXPECT_EQ(m_2d->nflexelem, m_full->nflexelem);

  // each body has 2 DOFs in 2d mode, 3 in full mode
  for (int i = 1; i < m_2d->nbody; i++) {
    EXPECT_EQ(m_2d->body_dofnum[i], 2) << "body " << i;
  }
  for (int i = 1; i < m_full->nbody; i++) {
    EXPECT_EQ(m_full->body_dofnum[i], 3) << "body " << i;
  }

  // simulate a few steps to make sure nothing crashes
  for (int i = 0; i < 10; i++) {
    mj_step(m_2d.get(), d_2d.get());
    mj_step(m_full.get(), d_full.get());
  }
}

TEST_F(UserFlexTest, Vert0RotationInvariant) {
  // unrotated trilinear grid
  static constexpr char xml_unrotated[] = R"(
  <mujoco>
  <worldbody>
    <body name="parent">
      <flexcomp name="test" type="grid" count="2 2 2" spacing="1 1 1"
                dim="3" dof="trilinear">
        <contact selfcollide="none"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";

  // same grid rotated 45 degrees around Z via parent body quaternion
  static constexpr char xml_rotated[] = R"(
  <mujoco>
  <worldbody>
    <body name="parent" quat="0.9238795 0 0 0.3826834">
      <flexcomp name="test" type="grid" count="2 2 2" spacing="1 1 1"
                dim="3" dof="trilinear">
        <contact selfcollide="none"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  MjModelPtr m1 =
      LoadModelFromString(xml_unrotated, error.data(), error.size());
  ASSERT_THAT(m1.get(), NotNull()) << error.data();

  MjModelPtr m2 = LoadModelFromString(xml_rotated, error.data(), error.size());
  ASSERT_THAT(m2.get(), NotNull()) << error.data();

  // same number of vertices
  ASSERT_EQ(m1->nflexvert, m2->nflexvert);

  // vert0 must be identical regardless of rotation
  for (int i = 0; i < 3 * m1->nflexvert; ++i) {
    EXPECT_NEAR(m1->flex_vert0[i], m2->flex_vert0[i], 1e-10)
        << "vert0 mismatch at index " << i;
  }
}

TEST_F(UserFlexTest, Load1DFlexFromOBJ) {
  const std::string xml_path =
      GetTestDataFilePath("user/testdata/flex_line_obj.xml");
  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  EXPECT_EQ(m->nflexvert, 4);
  EXPECT_EQ(m->nflexelem, 3);
  EXPECT_EQ(m->flex_dim[0], 1);
  mj_deleteModel(m);
}

TEST_F(UserFlexTest, PinBendingAcceptsMovingBody) {
  // pinned vertex inherits the flexcomp's parent body, which here has a joint
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <body name="moving">
      <joint type="hinge"/>
      <geom size="0.1"/>
      <flexcomp name="test" type="grid" count="3 3 1" spacing="1 1 1"
                radius="0.01" dim="2">
        <elasticity young="1" poisson="0" thickness="1" elastic2d="bend"/>
        <pin id="0"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), NotNull()) << error.data();
}

TEST_F(UserFlexTest, PinBendingAcceptsStaticBody) {
  // pinned vertex inherits the flexcomp's parent body (world), which is static
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <body name="parent">
      <flexcomp name="test" type="grid" count="3 3 1" spacing="1 1 1"
                radius="0.01" dim="2">
        <elasticity young="1" poisson="0" thickness="1" elastic2d="bend"/>
        <pin id="0"/>
      </flexcomp>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), NotNull()) << error.data();
}

TEST_F(UserFlexTest, FlatIrregularHingeHasNoRestBendingForce) {
  // The first triangle traverses the shared edge as 1->0, opposite to the
  // (min, max) order of flex_edge. The triangles are not mirror images, so
  // applying the bending coefficients to swapped edge vertices would produce a
  // nonzero force in the flat rest configuration.
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp name="test" type="direct" dim="2" radius="0.01"
              point="0 0 0  1 0 0  0.3 0.8 0  0.6 -0.7 0"
              element="1 0 2  0 1 3">
      <contact selfcollide="none"/>
      <elasticity young="1e4" poisson="0" thickness="0.1" elastic2d="bend"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << error.data();
  ASSERT_GE(m->flex_bendingadr[0], 0);
  MjDataPtr d = MakeData(m);
  mj_forward(m.get(), d.get());
  for (int i = 0; i < m->nv; i++) {
    EXPECT_NEAR(d->qfrc_spring[i], 0, MjTol(1e-10, 1e-5)) << "dof " << i;
  }
}

TEST_F(UserFlexTest, FlexConstraintsAndElasticityError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="test" type="grid" count="3 3 1" spacing="1 1 1"
                radius="0.01" dim="2">
        <elasticity young="1" poisson="0" thickness="1" elastic2d="stretch"/>
      </flexcomp>
    </worldbody>
    <equality>
      <flex flex="test"/>
    </equality>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(
      error.data(),
      HasSubstr(
          "flex constraints and elasticity (young) cannot both be present"));
}

TEST_F(UserFlexTest, FlexConstraintsAndEdgeStiffnessError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="test" type="grid" count="3 1 1" spacing="1 1 1"
                radius="0.01" dim="1">
        <edge stiffness="10"/>
      </flexcomp>
    </worldbody>
    <equality>
      <flex flex="test"/>
    </equality>
  </mujoco>
  )";
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m.get(), IsNull());
  EXPECT_THAT(
      error.data(),
      HasSubstr("flex constraints and edge stiffness cannot both be present"));
}

struct FlexDampingCase {
  const char* name;
  const char* flex_attributes;
  const char* joint_attributes;
};

class FlexDampingTest : public MujocoTest,
                        public testing::WithParamInterface<FlexDampingCase> {};

TEST_P(FlexDampingTest, RejectsInheritedDamping) {
  std::string xml =
      std::string("<mujoco><default><joint ") + GetParam().joint_attributes +
      "/></default><worldbody><flexcomp name='test' type='grid' " +
      GetParam().flex_attributes + R"(>
        <contact selfcollide="none"/>
        <elasticity young="10"/>
      </flexcomp></worldbody></mujoco>)";
  char error[1024];
  auto model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model.get(), IsNull());
  EXPECT_THAT(error, HasSubstr("cannot have joint damping"));
}

INSTANTIATE_TEST_SUITE_P(
    VerticesAndNodes, FlexDampingTest,
    testing::Values(
        FlexDampingCase{"Vertex", "dim='2' count='3 3 1'", "damping='0.1'"},
        FlexDampingCase{"VertexPolynomial", "dim='2' count='3 3 1'",
                        "damping='0 0.1'"},
        FlexDampingCase{"VertexSpringDamper", "dim='2' count='3 3 1'",
                        "springdamper='0.1 1'"},
        FlexDampingCase{"Node", "dim='3' count='3 3 2' dof='trilinear'",
                        "damping='0.1'"},
        FlexDampingCase{"NodePolynomial",
                        "dim='3' count='3 3 2' dof='trilinear'",
                        "damping='0 0.1'"},
        FlexDampingCase{"NodeSpringDamper",
                        "dim='3' count='3 3 2' dof='trilinear'",
                        "springdamper='0.1 1'"}),
    [](const testing::TestParamInfo<FlexDampingCase>& info) {
      return info.param.name;
    });

TEST_F(UserFlexTest, AllowsDampingOnPinnedParent) {
  static constexpr char xml[] = R"(
    <mujoco><worldbody>
      <body name="slider" pos="0 0 1">
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        <joint type="slide" axis="0 0 1" damping="5"/>
        <flexcomp name="test" type="grid" count="3 3 1" spacing="0.1 0.1 0.1" dim="2">
          <pin id="4"/>
          <edge equality="true"/>
        </flexcomp>
      </body>
    </worldbody></mujoco>)";
  char error[1024];
  auto model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model.get(), NotNull()) << error;
}

class FlexPinnedDampingTest : public MujocoTest,
                              public testing::WithParamInterface<bool> {};

TEST_P(FlexPinnedDampingTest, AllowsDampingOnRigidAttachment) {
  std::string pin =
      GetParam()
          ? "<joint type='slide' damping='5'/><geom type='sphere' size='0.1'/>"
          : "<joint type='hinge' damping='5'/>";
  std::string xml = R"(
  <mujoco>
    <worldbody>
      <body name="pin">
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>)" +
                    pin + R"(
      </body>
      <body name="end" pos="1 0 0">
        <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
        <joint type="slide" axis="1 0 0"/>
        <joint type="slide" axis="0 1 0"/>
        <joint type="slide" axis="0 0 1"/>
      </body>
    </worldbody>
    <deformable>
      <flex dim="1" body="pin end" element="0 1">
        <edge damping="0.1"/>
      </flex>
    </deformable>
  </mujoco>)";
  char error[1024];
  auto model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model.get(), NotNull()) << error;
}

INSTANTIATE_TEST_SUITE_P(LeafBodies, FlexPinnedDampingTest, testing::Bool(),
                         [](const testing::TestParamInfo<bool>& info) {
                           return info.param ? "SliderWithGeom" : "Hinge";
                         });

constexpr char kXYZSlides[] = R"(
    <joint type="slide" axis="1 0 0"/>
    <joint type="slide" axis="0 1 0"/>
    <joint type="slide" axis="0 0 1"/>)";

struct BendingAttachmentCase {
  const char* name;
  std::string joints;
  std::string ancestor;
  std::string attributes;
};

class FlexBendingAttachmentTest
    : public MujocoTest,
      public testing::WithParamInterface<BendingAttachmentCase> {
 protected:
  MjModelPtr LoadAttachment(char* error, int error_size,
                            const char* elastic2d = "bend") {
    std::string xml = R"(
  <mujoco>
    <worldbody>
      <body name="v0"/>
      <body name="v1" pos="0.1 0 0"/>
      <body name="ancestor" pos="0.1 0.1 0" )" +
                      GetParam().attributes + R"(>
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>)" +
                      GetParam().ancestor + R"(
        <body name="carrier">
          <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>)" +
                      GetParam().joints + R"(
          <body name="pin"/>
        </body>
      </body>
    </worldbody>
    <deformable>
      <flex dim="2" body="v0 v1 pin" element="0 1 2">
        <contact selfcollide="none" contype="0" conaffinity="0"/>
        <elasticity young="10" thickness="0.01" elastic2d=")" +
                      elastic2d + R"("/>
      </flex>
    </deformable>
  </mujoco>)";
    return LoadModelFromString(xml, error, error_size);
  }
};

TEST_P(FlexBendingAttachmentTest, AcceptsArticulatedBendingMotion) {
  char error[1024];
  auto model = LoadAttachment(error, sizeof(error));
  EXPECT_THAT(model.get(), NotNull()) << error;
}

TEST_P(FlexBendingAttachmentTest, DoesNotRestrictStretchOnlyAttachments) {
  char error[1024];
  auto model = LoadAttachment(error, sizeof(error), "stretch");
  EXPECT_THAT(model.get(), NotNull()) << error;
}

INSTANTIATE_TEST_SUITE_P(
    ArticulatedMotion, FlexBendingAttachmentTest,
    testing::Values(
        BendingAttachmentCase{"Ball", "<joint type='ball'/>", ""},
        BendingAttachmentCase{
            "Hinges", "<joint/><joint axis='1 0 0'/><joint axis='0 1 0'/>", ""},
        BendingAttachmentCase{"SingleSlide", "<joint type='slide'/>", ""},
        BendingAttachmentCase{"ReorderedSlides", R"(
          <joint type="slide" axis="0 1 0"/>
          <joint type="slide" axis="1 0 0"/>
          <joint type="slide" axis="0 0 1"/>)",
                              ""},
        BendingAttachmentCase{"ReversedSlide", R"(
          <joint type="slide" axis="-1 0 0"/>
          <joint type="slide" axis="0 1 0"/>
          <joint type="slide" axis="0 0 1"/>)",
                              ""},
        BendingAttachmentCase{
            "JointFrame",
            std::string("<frame euler='0 0 30'>") + kXYZSlides + "</frame>",
            ""},
        BendingAttachmentCase{"MovingAncestor", kXYZSlides,
                              "<joint type='ball'/>"}),
    [](const testing::TestParamInfo<BendingAttachmentCase>& info) {
      return info.param.name;
    });

class FlexMocapAttachmentTest : public FlexBendingAttachmentTest {};

TEST_P(FlexMocapAttachmentTest, RejectsMocapElasticity) {
  for (const char* elasticity : {"bend", "stretch"}) {
    SCOPED_TRACE(elasticity);
    char error[1024];
    auto model = LoadAttachment(error, sizeof(error), elasticity);
    EXPECT_EQ(model.get(), nullptr);
    EXPECT_THAT(
        error, HasSubstr("flex elasticity does not support mocap attachments"));
  }
}

INSTANTIATE_TEST_SUITE_P(
    Mocap, FlexMocapAttachmentTest,
    testing::Values(BendingAttachmentCase{"FixedChild", "", "", "mocap='true'"},
                    BendingAttachmentCase{"SliderChild", kXYZSlides, "",
                                          "mocap='true'"}),
    [](const testing::TestParamInfo<BendingAttachmentCase>& info) {
      return info.param.name;
    });

class FlexBendingSupportedAttachmentTest : public FlexBendingAttachmentTest {};

TEST_P(FlexBendingSupportedAttachmentTest, AcceptsFixedOrXYZSlideAttachments) {
  char error[1024];
  auto model = LoadAttachment(error, sizeof(error));
  EXPECT_THAT(model.get(), NotNull()) << error;
}

INSTANTIATE_TEST_SUITE_P(
    SupportedMotion, FlexBendingSupportedAttachmentTest,
    testing::Values(BendingAttachmentCase{"Fixed", "", ""},
                    BendingAttachmentCase{"XYZSlides", kXYZSlides, ""},
                    BendingAttachmentCase{"NormalizedAxes", R"(
          <joint type="slide" axis="2 0 0"/>
          <joint type="slide" axis="0 3 0"/>
          <joint type="slide" axis="0 0 4"/>)",
                                          ""},
                    BendingAttachmentCase{"CompiledJointFrame", R"(
          <frame euler="0 0 90">
            <joint type="slide" axis="0 -1 0"/>
            <joint type="slide" axis="1 0 0"/>
            <joint type="slide" axis="0 0 1"/>
          </frame>)",
                                          ""}),
    [](const testing::TestParamInfo<BendingAttachmentCase>& info) {
      return info.param.name;
    });

// ---------------- flexcomp consuming mesh asset tests ------------------------

static constexpr char kTetrahedronMsh[] =
    "$MeshFormat\n"
    "2.2 0 8\n"
    "$EndMeshFormat\n"
    "$Nodes\n"
    "4\n"
    "1 0 0 0\n"
    "2 1 0 0\n"
    "3 0 1 0\n"
    "4 0 0 1\n"
    "$EndNodes\n"
    "$Elements\n"
    "1\n"
    "1 4 0 1 2 3 4\n"
    "$EndElements\n";

static constexpr char kTetrahedronObj[] =
    "v 0 0 0\n"
    "v 1 0 0\n"
    "v 0 1 0\n"
    "v 0 0 1\n"
    "f 1 3 2\n"
    "f 1 2 4\n"
    "f 2 3 4\n"
    "f 3 1 4\n";

TEST_F(UserFlexTest, FlexcompMeshAssetVolumetric) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tetmesh" content_type="model/vnd.gmsh" file="tetrahedron.msh"/>
    </asset>
    <worldbody>
      <flexcomp name="soft" mesh="tetmesh"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.msh", kTetrahedronMsh,
                  sizeof(kTetrahedronMsh) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_EQ(model->nflex, 1);
  EXPECT_EQ(model->flex_dim[0], 3);
  EXPECT_EQ(model->nflexvert, 4);
  EXPECT_EQ(model->nflexelem, 1);

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, FlexcompMeshAssetDimMismatch) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tetmesh" content_type="model/vnd.gmsh" file="tetrahedron.msh"/>
    </asset>
    <worldbody>
      <flexcomp name="soft" mesh="tetmesh" dim="2"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.msh", kTetrahedronMsh,
                  sizeof(kTetrahedronMsh) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("dimensionality"));

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, FlexcompMeshAssetSurface2D) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tet_surface" file="tetrahedron.obj"/>
    </asset>
    <worldbody>
      <flexcomp name="soft" mesh="tet_surface" dim="2"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.obj", kTetrahedronObj,
                  sizeof(kTetrahedronObj) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_EQ(model->nflex, 1);
  EXPECT_EQ(model->flex_dim[0], 2);
  EXPECT_EQ(model->nflexvert, 4);
  EXPECT_EQ(model->nflexelem, 4);

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, FlexcompMeshAssetSurface3DWithOrigin) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tet_surface" file="tetrahedron.obj"/>
    </asset>
    <worldbody>
      <flexcomp name="soft" mesh="tet_surface" dim="3" origin=".25 .25 .25"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.obj", kTetrahedronObj,
                  sizeof(kTetrahedronObj) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_EQ(model->nflex, 1);
  EXPECT_EQ(model->flex_dim[0], 3);
  EXPECT_EQ(model->nflexvert, 5);  // 4 mesh vertices + origin
  EXPECT_EQ(model->nflexelem, 4);  // 4 extruded tetrahedra

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, FlexcompMeshAssetSurface3DWithoutOriginFails) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tet_surface" file="tetrahedron.obj"/>
    </asset>
    <worldbody>
      <flexcomp name="soft" mesh="tet_surface" dim="3"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.obj", kTetrahedronObj,
                  sizeof(kTetrahedronObj) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("origin must be specified"));

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, FlexcompMeshAssetNotFound) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="soft" mesh="nonexistent"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("mesh 'nonexistent' not found"));
}

TEST_F(UserFlexTest, FlexcompMeshAndFileMutuallyExclusive) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="trimesh" file="triangle.obj"/>
    </asset>
    <worldbody>
      <flexcomp name="soft" mesh="trimesh" file="triangle.obj"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("both 'file' and 'mesh' specified"));
}

TEST_F(UserFlexTest, FlexcompMeshAssetBuiltinGrid3D) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="grid_mesh" builtin="grid" params="4 4 4  0.02 0.02 0.02  3"/>
    </asset>
    <worldbody>
      <flexcomp name="soft" mesh="grid_mesh" radius=".001"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_EQ(model->nflex, 1);
  EXPECT_EQ(model->flex_dim[0], 3);
  EXPECT_EQ(model->flex_vertnum[0], 64);
  EXPECT_EQ(model->flex_elemnum[0], 162);  // 3*3*3 * 6 tets
}

TEST_F(UserFlexTest, FlexcompMeshAssetBuiltinGrid2D) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="grid_2d" builtin="grid" params="5 5 1  0.02 0.02 0  2"/>
    </asset>
    <worldbody>
      <flexcomp name="soft2d" mesh="grid_2d" radius=".001"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_EQ(model->nflex, 1);
  EXPECT_EQ(model->flex_dim[0], 2);
  EXPECT_EQ(model->flex_vertnum[0], 25);
  EXPECT_EQ(model->flex_elemnum[0], 32);  // 4*4 * 2 triangles
}

TEST_F(UserFlexTest, FlexcompMeshAssetBuiltinBox) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="box_mesh" builtin="box" params="3 3 3  0.05 0.05 0.05  3"/>
    </asset>
    <worldbody>
      <flexcomp name="softbox" mesh="box_mesh" radius=".001"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_EQ(model->nflex, 1);
  EXPECT_EQ(model->flex_dim[0], 3);
}

TEST_F(UserFlexTest, FlexcompMeshAssetBuiltinDisc) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="disc_mesh" builtin="disc" params="5 5 1  0.02 0.02 0"/>
    </asset>
    <worldbody>
      <flexcomp name="softdisc" mesh="disc_mesh" radius=".001"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_EQ(model->nflex, 1);
  EXPECT_EQ(model->flex_dim[0], 2);
}

TEST_F(UserFlexTest, FlexcompBoxTexcoord) {
  // one generated texcoord per vertex, the center vertex only exists in 3D
  for (int dim : {2, 3}) {
    std::string xml = R"(
    <mujoco>
      <asset>
        <material name="mat"/>
      </asset>
      <worldbody>
        <flexcomp name="box" type="box" count="3 3 3" spacing=".1 .1 .1"
                  radius=".001" material="mat" dim=")";
    xml += std::to_string(dim);
    xml += R"("/>
      </worldbody>
    </mujoco>
    )";
    char error[1024] = {0};
    MjModelPtr model = LoadModelFromString(xml.c_str(), error, sizeof(error));
    ASSERT_THAT(model, NotNull()) << error;
    EXPECT_EQ(model->nflextexcoord, model->flex_vertnum[0]);
  }
}

// flex vertex positions of flex f, after kinematics
static std::vector<double> FlexVertPos(const mjModel* m, const mjData* d,
                                       int f) {
  const mjtNum* x = d->flexvert_xpos + 3 * m->flex_vertadr[f];
  return std::vector<double>(x, x + 3 * m->flex_vertnum[f]);
}

// positions of the vertices of mesh geom g, after kinematics
static std::vector<double> GeomMeshVertPos(const mjModel* m, const mjData* d,
                                           int g) {
  int mesh = m->geom_dataid[g];
  std::vector<double> pos;
  for (int v = 0; v < m->mesh_vertnum[mesh]; v++) {
    const float* vert = m->mesh_vert + 3 * (m->mesh_vertadr[mesh] + v);
    mjtNum local[3] = {vert[0], vert[1], vert[2]}, global[3];
    mju_mulMatVec3(global, d->geom_xmat + 9 * g, local);
    for (int j = 0; j < 3; j++) {
      pos.push_back(global[j] + d->geom_xpos[3 * g + j]);
    }
  }
  return pos;
}

TEST_F(UserFlexTest, FlexcompMeshAssetAppliesMeshTransforms) {
  // mesh= applies scale, refpos and refquat in the asset frame, and coincides
  // with a geom using the same mesh; file= uses the raw file coordinates
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="surface" file="tetrahedron.obj" scale="2 2 2"
            refpos="0.5 0 0" refquat="0.7071067811865476 0 0 0.7071067811865476"/>
      <mesh name="volume" content_type="model/vnd.gmsh" file="tetrahedron.msh"
            scale="2 2 2" refpos="0.5 0 0"
            refquat="0.7071067811865476 0 0 0.7071067811865476"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="surface" contype="0" conaffinity="0"/>
      <flexcomp name="from_mesh" mesh="surface" dim="2" radius=".001"/>
      <flexcomp name="from_file" type="mesh" file="tetrahedron.obj" dim="2"
                radius=".001"/>
      <flexcomp name="from_gmsh" mesh="volume" radius=".001"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.obj", kTetrahedronObj,
                  sizeof(kTetrahedronObj) - 1);
  mj_addBufferVFS(&vfs, "tetrahedron.msh", kTetrahedronMsh,
                  sizeof(kTetrahedronMsh) - 1);

  char error[1024] = {0};
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error), &vfs);
  ASSERT_THAT(m, NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_kinematics(m.get(), d.get());
  mj_flex(m.get(), d.get());

  // 2 * R^T (v - refpos), where R is the rotation by 90 degrees about z
  std::vector<double> expected = {0, 1, 0, 0, -1, 0, 2, 1, 0, 0, 1, 2};
  EXPECT_THAT(FlexVertPos(m.get(), d.get(), 0),
              Pointwise(DoubleNear(1e-6), expected));
  EXPECT_THAT(
      FlexVertPos(m.get(), d.get(), 0),
      Pointwise(DoubleNear(1e-6), GeomMeshVertPos(m.get(), d.get(), 0)));

  std::vector<double> raw = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1};
  EXPECT_THAT(FlexVertPos(m.get(), d.get(), 1),
              Pointwise(DoubleNear(1e-6), raw));

  // volumetric nodes follow the same transformation as the surface vertices
  EXPECT_THAT(FlexVertPos(m.get(), d.get(), 2),
              Pointwise(DoubleNear(1e-6), expected));

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, FlexcompMeshAssetGrid2DStaysInPlane) {
  // the principal axes of a flat mesh are not aligned with its plane, the flex
  // must stay in the frame of the asset
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="grid" builtin="grid" params="3 3 1  0.1 0.1 0  2"/>
    </asset>
    <worldbody>
      <flexcomp name="cloth" mesh="grid" radius=".001"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  MjDataPtr d = MakeData(m);
  mj_kinematics(m.get(), d.get());
  mj_flex(m.get(), d.get());

  std::vector<double> pos = FlexVertPos(m.get(), d.get(), 0);
  ASSERT_EQ(pos.size(), 9 * 3);
  for (int v = 0; v < 9; v++) {
    EXPECT_NEAR(std::abs(pos[3 * v + 0]), v / 3 == 1 ? 0 : 0.1, 1e-6);
    EXPECT_NEAR(std::abs(pos[3 * v + 1]), v % 3 == 1 ? 0 : 0.1, 1e-6);
    EXPECT_NEAR(pos[3 * v + 2], 0, 1e-6);
  }
}

TEST_F(UserFlexTest, MeshAssetNodeOnlyBuiltinsRejected) {
  // mesh assets must have faces or tetrahedra, use flexcomp type="circle" or
  // type="grid" with dim=1 for 1D flexes
  static constexpr char circle_xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="circle_mesh" builtin="circle" params="9 1 1  0.02 0 0"/>
    </asset>
  </mujoco>
  )";
  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(circle_xml, error, sizeof(error));
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("circle"));

  static constexpr char line_xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="line" builtin="grid" params="5 1 1  0.02 0 0  1"/>
    </asset>
  </mujoco>
  )";
  model = LoadModelFromString(line_xml, error, sizeof(error));
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("dim must be 2 or 3"));
}

// tetrahedron surface in GMSH 2.2 format (4 nodes, 4 triangles, element type 2)
static constexpr char kTetrahedronSurfaceMsh[] =
    "$MeshFormat\n"
    "2.2 0 8\n"
    "$EndMeshFormat\n"
    "$Nodes\n"
    "4\n"
    "1 0 0 0\n"
    "2 1 0 0\n"
    "3 0 1 0\n"
    "4 0 0 1\n"
    "$EndNodes\n"
    "$Elements\n"
    "4\n"
    "1 2 0 1 3 2\n"
    "2 2 0 1 2 4\n"
    "3 2 0 2 3 4\n"
    "4 2 0 3 1 4\n"
    "$EndElements\n";

TEST_F(UserFlexTest, FlexcompMeshAssetGmshSurface) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="surf" content_type="model/vnd.gmsh" file="surface.msh"/>
    </asset>
    <worldbody>
      <flexcomp name="soft" mesh="surf" dim="2" radius=".001"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "surface.msh", kTetrahedronSurfaceMsh,
                  sizeof(kTetrahedronSurfaceMsh) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_EQ(model->flex_dim[0], 2);
  EXPECT_EQ(model->flex_vertnum[0], 4);
  EXPECT_EQ(model->flex_elemnum[0], 4);

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, FlexcompGmshDecoderErrorIsForwarded) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="soft" type="gmsh" file="bad.msh" radius=".001"/>
    </worldbody>
  </mujoco>
  )";
  static constexpr char kBadMsh[] =
      "$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$Nodes\nnot_a_number\n";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "bad.msh", kBadMsh, sizeof(kBadMsh) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("GMSH decoder: GMSH file missing $Elements"));

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, MeshAssetSharedByGeomAndFlexcomp) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tet" file="tetrahedron.obj"/>
    </asset>
    <worldbody>
      <geom type="mesh" mesh="tet"/>
      <flexcomp name="soft" mesh="tet" dim="2" radius=".001" pos="0 0 2"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.obj", kTetrahedronObj,
                  sizeof(kTetrahedronObj) - 1);

  char error[1024] = {0};
  MjModelPtr m = LoadModelFromString(xml, error, sizeof(error), &vfs);
  ASSERT_THAT(m, NotNull()) << error;
  EXPECT_EQ(m->nmesh, 1);
  EXPECT_EQ(m->mesh_vertnum[0], 4);
  ASSERT_EQ(m->flex_vertnum[0], 4);

  // the flex coincides with the geom, offset by the flexcomp position
  MjDataPtr d = MakeData(m);
  mj_kinematics(m.get(), d.get());
  mj_flex(m.get(), d.get());
  std::vector<double> expected = GeomMeshVertPos(m.get(), d.get(), 0);
  for (int i = 0; i < 4; i++) {
    expected[3 * i + 2] += 2;
  }
  EXPECT_THAT(FlexVertPos(m.get(), d.get(), 0),
              Pointwise(DoubleNear(1e-6), expected));

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, FlexcompMeshAssetCacheHit) {
  // the second load reads the volumetric mesh (nodes and tets) from the cache
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tetmesh" content_type="model/vnd.gmsh" file="cached_tet.msh"/>
    </asset>
    <worldbody>
      <flexcomp name="soft" mesh="tetmesh" radius=".001"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "cached_tet.msh", kTetrahedronMsh,
                  sizeof(kTetrahedronMsh) - 1);

  // the nodes are in the frame of the asset, i.e. as in the file
  std::vector<double> nodes = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (int k = 0; k < 2; k++) {
    char error[1024] = {0};
    MjModelPtr m = LoadModelFromString(xml, error, sizeof(error), &vfs);
    ASSERT_THAT(m, NotNull()) << error;
    EXPECT_EQ(m->flex_dim[0], 3);
    EXPECT_EQ(m->flex_elemnum[0], 1);
    MjDataPtr d = MakeData(m);
    mj_kinematics(m.get(), d.get());
    mj_flex(m.get(), d.get());
    EXPECT_THAT(FlexVertPos(m.get(), d.get(), 0),
                Pointwise(DoubleNear(1e-6), nodes))
        << "load " << k;
  }

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, MakeFlexFileOrMeshName) {
  mjSpec* spec = mj_makeSpec();
  mjsMesh* mesh = mjs_addMesh(spec, nullptr);
  mjs_setName(mesh->element, "grid");
  double params[7] = {3, 3, 1, 0.1, 0.1, 0, 2};
  ASSERT_EQ(mjs_makeMesh(mesh, mjMESH_BUILTIN_GRID, params, 7), 0);

  // file is the name of a mesh asset
  mjsFlex* flex =
      mjs_makeFlex(mjs_findBody(spec, "world"), "from_mesh", "mesh", 2, nullptr,
                   nullptr, nullptr, nullptr, nullptr, 0.001, 1, 0, 0, 0, 0, 0,
                   nullptr, nullptr, nullptr, "grid", nullptr);
  ASSERT_THAT(flex, NotNull()) << mjs_getError(spec);

  // typo: neither a mesh nor a file
  flex = mjs_makeFlex(mjs_findBody(spec, "world"), "typo", "mesh", 2, nullptr,
                      nullptr, nullptr, nullptr, nullptr, 0.001, 1, 0, 0, 0, 0,
                      0, nullptr, nullptr, nullptr, "gird", nullptr);
  EXPECT_THAT(flex, IsNull());
  EXPECT_THAT(mjs_getError(spec), HasSubstr("neither a mesh asset nor a file"));

  mj_deleteSpec(spec);
}

// make a trilinear box surface flex, as a strain-constrained shell or as a
// solid
static mjsFlex* MakeTrilinearBox(mjSpec* spec, bool shell) {
  mjsBody* body = mjs_addBody(mjs_findBody(spec, "world"), nullptr);
  mjs_setName(body->element, "obj");
  int count[3] = {4, 4, 4};
  int cellcount[3] = {2, 2, 2};
  double spacing[3] = {.02, .02, .02};
  mjsFlex* flex =
      mjs_makeFlex(body, "cube", "box", 2, "trilinear", count, cellcount,
                   spacing, nullptr, 0.001, 0.05, 0, shell ? 3 : 0, 0, 0,
                   shell ? 1 : 0, nullptr, nullptr, nullptr, nullptr, nullptr);
  if (flex) {
    flex->selfcollide = mjFLEXSELF_NONE;
  }
  return flex;
}

// a flex made as a strain-constrained shell and then switched to a volumetric
// solid (elastic2d none, strain equalities deleted) must compile the same
// stiffness as a flex made volumetric from the start, and stay stable
TEST_F(UserFlexTest, ShellSwitchedToSolidStiffness) {
  using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
  auto set_solid = [](mjsFlex* flex) {
    flex->young = 3e4;
    flex->poisson = 0.4;
    flex->damping = 0.002;
  };

  // reference: made volumetric
  SpecPtr ref_spec(mj_makeSpec(), mj_deleteSpec);
  mjsFlex* ref_flex = MakeTrilinearBox(ref_spec.get(), /*shell=*/false);
  ASSERT_THAT(ref_flex, NotNull()) << mjs_getError(ref_spec.get());
  set_solid(ref_flex);
  MjModelPtr ref(mj_compile(ref_spec.get(), nullptr));
  ASSERT_THAT(ref, NotNull()) << mjs_getError(ref_spec.get());
  ASSERT_EQ(ref->flex_interp[0], 1);
  int nstiffness = ref->nflexstiffness;
  ASSERT_GT(nstiffness, 0);
  ASSERT_LT(ref->flex_stiffness[0], 0) << "expected a raw stiffness matrix";

  // switched without and with an intermediate compile of the shell
  for (bool precompile : {false, true}) {
    SpecPtr spec(mj_makeSpec(), mj_deleteSpec);
    spec->option.integrator = mjINT_DISCRETE;
    spec->option.solver = mjSOL_CG;
    spec->option.gravity[0] = spec->option.gravity[1] = 0;
    spec->option.gravity[2] = 0;
    mjsFlex* flex = MakeTrilinearBox(spec.get(), /*shell=*/true);
    ASSERT_THAT(flex, NotNull()) << mjs_getError(spec.get());
    ASSERT_THAT(mjs_firstElement(spec.get(), mjOBJ_EQUALITY), NotNull());
    if (precompile) {
      flex->young = 3e4;
      flex->thickness = 0.01;
      MjModelPtr shell(mj_compile(spec.get(), nullptr));
      ASSERT_THAT(shell, NotNull()) << mjs_getError(spec.get());
      EXPECT_EQ(shell->flex_edgeequality[0], 3);
    }

    // switch to a volumetric solid
    flex->elastic2d = 0;
    mjsElement* eq = mjs_firstElement(spec.get(), mjOBJ_EQUALITY);
    while (eq) {
      mjsElement* next = mjs_nextElement(spec.get(), eq);
      ASSERT_EQ(mjs_delete(spec.get(), eq), 0);
      eq = next;
    }
    set_solid(flex);
    MjModelPtr m(mj_compile(spec.get(), nullptr));
    ASSERT_THAT(m, NotNull()) << mjs_getError(spec.get());
    EXPECT_EQ(m->neq, 0);
    EXPECT_EQ(m->flex_edgeequality[0], 0);
    EXPECT_EQ(m->flex_interp[0], 1);

    // same stiffness as the reference, not the strain eigenmode table
    ASSERT_EQ(m->nflexstiffness, nstiffness) << "precompile " << precompile;
    for (int i = 0; i < nstiffness && !HasFailure(); i++) {
      EXPECT_MJTNUM_EQ(m->flex_stiffness[i], ref->flex_stiffness[i])
          << "precompile " << precompile << ", index " << i;
    }

    // simulate: must stay finite and bounded
    MjDataPtr d = MakeData(m);
    for (int i = 0; i < 200; i++) {
      mj_step(m.get(), d.get());
    }
    for (int i = 0; i < m->nq; i++) {
      ASSERT_TRUE(std::isfinite(d->qpos[i])) << "qpos " << i;
      EXPECT_LT(mju_abs(d->qpos[i]), 0.01) << "qpos " << i;
    }
  }
}

// state derived by a compile (flap stencils, rigid, centered, empty cells) is
// recomputed from the current spec when it is compiled again
TEST_F(UserFlexTest, RecompileResetsDerivedState) {
  using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
  auto compile = [](mjSpec* spec) {
    MjModelPtr m(mj_compile(spec, nullptr));
    EXPECT_THAT(m, NotNull()) << mjs_getError(spec);
    return m;
  };
  auto first_flex = [](mjSpec* spec) {
    return mjs_asFlex(mjs_firstElement(spec, mjOBJ_FLEX));
  };

  // flap stencils: reverse the triangle order between compiles
  static constexpr char cloth_xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="cloth" type="grid" count="3 3 1" spacing=".1 .1 .1"
                dim="2" radius=".001">
        <elasticity young="1e3" thickness="1e-3" elastic2d="bend"/>
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  auto reverse_triangles = [](mjsFlex* flex) {
    std::vector<int>& elem = *flex->elem;
    std::vector<int> reversed;
    for (int t = static_cast<int>(elem.size()) / 3 - 1; t >= 0; t--) {
      reversed.insert(reversed.end(), elem.begin() + 3 * t,
                      elem.begin() + 3 * t + 3);
    }
    elem = reversed;
  };
  SpecPtr cloth(mj_parseXMLString(cloth_xml, nullptr, nullptr, 0),
                mj_deleteSpec);
  ASSERT_THAT(cloth, NotNull());
  ASSERT_THAT(compile(cloth.get()), NotNull());
  reverse_triangles(first_flex(cloth.get()));
  MjModelPtr recompiled = compile(cloth.get());
  SpecPtr cloth_ref(mj_parseXMLString(cloth_xml, nullptr, nullptr, 0),
                    mj_deleteSpec);
  ASSERT_THAT(cloth_ref, NotNull());
  reverse_triangles(first_flex(cloth_ref.get()));
  MjModelPtr cloth_expected = compile(cloth_ref.get());
  ASSERT_THAT(recompiled, NotNull());
  ASSERT_THAT(cloth_expected, NotNull());
  ASSERT_EQ(recompiled->nflexedge, cloth_expected->nflexedge);
  EXPECT_EQ(AsVector(recompiled->flex_edge, 2 * recompiled->nflexedge),
            AsVector(cloth_expected->flex_edge, 2 * cloth_expected->nflexedge));
  EXPECT_EQ(
      AsVector(recompiled->flex_edgeflap, 2 * recompiled->nflexedge),
      AsVector(cloth_expected->flex_edgeflap, 2 * cloth_expected->nflexedge));

  // rigid and centered: attach the vertices to different bodies, move them
  static constexpr char rod_xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="a"><freejoint/><geom size=".1"/></body>
      <body name="b" pos="1 0 0"><freejoint/><geom size=".1"/></body>
    </worldbody>
    <deformable>
      <flex name="rod" dim="1" body="a a" vertex="0 0 0  0 0 0" element="0 1"/>
    </deformable>
  </mujoco>
  )";
  char error[1024] = {0};
  SpecPtr rod(mj_parseXMLString(rod_xml, nullptr, error, sizeof(error)),
              mj_deleteSpec);
  ASSERT_THAT(rod, NotNull()) << error;
  MjModelPtr rigid = compile(rod.get());
  ASSERT_THAT(rigid, NotNull());
  EXPECT_EQ(rigid->flex_rigid[0], 1);
  EXPECT_EQ(rigid->flex_centered[0], 1);
  mjsFlex* rod_flex = first_flex(rod.get());
  mjs_setStringVec(rod_flex->vertbody, "a b");
  const double vert[6] = {0, 0, 0, .1, 0, 0};
  mjs_setDouble(rod_flex->vert, vert, 6);
  MjModelPtr moved = compile(rod.get());
  ASSERT_THAT(moved, NotNull());
  EXPECT_EQ(moved->flex_rigid[0], 0);
  EXPECT_EQ(moved->flex_centered[0], 0);

  // empty cells: recomputed on every compile, same stiffness when compiled
  // again (of the 3x1x1 cells only the last one is occupied)
  static constexpr char solid_xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="solid" type="direct" dim="3" dof="trilinear" mass="1"
                cellcount="3 1 1" radius=".001"
                point=".7 0 0  1 0 0  .7 1 0  1 1 0
                       .7 0 1  1 0 1  .7 1 1  1 1 1
                       0 .5 .5"
                element="0 1 2 4  3 2 1 7  5 1 4 7  6 2 4 7  1 2 4 7">
        <contact selfcollide="none"/>
        <elasticity young="100" poisson="0.3"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  SpecPtr solid(mj_parseXMLString(solid_xml, nullptr, nullptr, 0),
                mj_deleteSpec);
  ASSERT_THAT(solid, NotNull());
  MjModelPtr once = compile(solid.get());
  MjModelPtr twice = compile(solid.get());
  ASSERT_THAT(once, NotNull());
  ASSERT_THAT(twice, NotNull());
  ASSERT_EQ(once->nflexstiffness, twice->nflexstiffness);
  int block = 24 * 24;
  ASSERT_EQ(once->nflexstiffness, 3 * block);
  EXPECT_EQ(once->flex_stiffness[0], 0) << "expected an empty first cell";
  EXPECT_NE(once->flex_stiffness[2 * block], 0);
  for (int i = 0; i < once->nflexstiffness && !HasFailure(); i++) {
    EXPECT_MJTNUM_EQ(once->flex_stiffness[i], twice->flex_stiffness[i])
        << "index " << i;
  }
}

TEST_F(UserFlexTest, FlexcompMeshAssetWrongType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="tet" file="tetrahedron.obj"/>
    </asset>
    <worldbody>
      <flexcomp name="soft" type="grid" mesh="tet" dim="2" radius=".001"/>
    </worldbody>
  </mujoco>
  )";

  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "tetrahedron.obj", kTetrahedronObj,
                  sizeof(kTetrahedronObj) - 1);

  char error[1024] = {0};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error), &vfs);
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error, HasSubstr("only be used with flexcomp type mesh or gmsh"));

  mj_deleteVFS(&vfs);
}

TEST_F(UserFlexTest, InterpolatedShellElastic2dStiffness) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="shell" type="grid" count="4 4 4" spacing=".05 .05 .05"
                dim="3" cellcount="2 2 2" radius=".001" mass="1"
                dof="trilinear">
        <elasticity young="1e4" poisson="0.3" thickness="0.01"
                    elastic2d="bend"/>
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
  SpecPtr spec(mj_parseXMLString(xml, nullptr, error, sizeof(error)),
               mj_deleteSpec);
  ASSERT_THAT(spec, NotNull()) << error;
  mjsFlex* flex = mjs_asFlex(mjs_firstElement(spec.get(), mjOBJ_FLEX));
  ASSERT_THAT(flex, NotNull());

  // elastic2d="bend" (1) without strain equality: no membrane stiffness
  MjModelPtr bend(mj_compile(spec.get(), nullptr));
  ASSERT_THAT(bend, NotNull()) << mjs_getError(spec.get());
  EXPECT_EQ(bend->flex_interp[0], -1);
  EXPECT_EQ(bend->flex_stiffnessadr[0], -1);
  EXPECT_EQ(bend->nflexstiffness, 0);
  EXPECT_GE(bend->flex_bendingadr[0], 0);

  // elastic2d="stretch" (2): membrane stiffness allocated, no bending
  flex->elastic2d = 2;
  MjModelPtr stretch(mj_compile(spec.get(), nullptr));
  ASSERT_THAT(stretch, NotNull()) << mjs_getError(spec.get());
  EXPECT_EQ(stretch->flex_interp[0], -1);
  EXPECT_EQ(stretch->flex_stiffnessadr[0], 0);
  EXPECT_GT(stretch->nflexstiffness, 0);
  EXPECT_NE(stretch->flex_stiffness[3 * 4 + 1], 0);
  EXPECT_EQ(stretch->flex_bendingadr[0], -1);

  // elastic2d="both" (3): both membrane stiffness and bending allocated
  flex->elastic2d = 3;
  MjModelPtr both(mj_compile(spec.get(), nullptr));
  ASSERT_THAT(both, NotNull()) << mjs_getError(spec.get());
  EXPECT_EQ(both->flex_interp[0], -1);
  EXPECT_EQ(both->flex_stiffnessadr[0], 0);
  EXPECT_EQ(both->nflexstiffness, stretch->nflexstiffness);
  EXPECT_NE(both->flex_stiffness[3 * 4 + 1], 0);
  EXPECT_GE(both->flex_bendingadr[0], 0);
}

TEST_F(UserFlexTest, FlexStrainEqualityValidation) {
  using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
  char error[1024] = {0};

  // Non-interpolated flex with flexstrain equality fails to compile.
  static constexpr char non_interp_xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="f" type="grid" count="2 2 2" spacing=".1 .1 .1"
                dim="3" radius=".001">
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
    <equality>
      <flexstrain flex="f" cell="0 0 0"/>
    </equality>
  </mujoco>
  )";
  MjModelPtr non_interp =
      LoadModelFromString(non_interp_xml, error, sizeof(error));
  EXPECT_THAT(non_interp, IsNull());
  EXPECT_THAT(error,
              HasSubstr("flex strain equality requires an interpolated flex"));

  // Switching a shell flex with strain equalities to volume mode (elastic2d=0)
  // without removing the shell strain equalities fails to compile.
  {
    SpecPtr spec(mj_makeSpec(), mj_deleteSpec);
    mjsFlex* flex = MakeTrilinearBox(spec.get(), /*shell=*/true);
    ASSERT_THAT(flex, NotNull()) << mjs_getError(spec.get());
    flex->elastic2d = 0;
    MjModelPtr m(mj_compile(spec.get(), nullptr));
    EXPECT_THAT(m, IsNull());
    EXPECT_THAT(mjs_getError(spec.get()),
                HasSubstr("shell flex strain equality on volumetric flex"));
  }

  // Switching a single-cell volumetric flex with strain equalities to shell
  // mode (elastic2d=1) without updating the equalities fails to compile, and
  // out-of-range face/cell indices are rejected.
  static constexpr char vol_strain_xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="f" type="box" count="4 4 4" cellcount="1 1 1"
                spacing=".02 .02 .02" dim="2" dof="trilinear" radius=".001">
        <edge equality="strain"/>
        <elasticity thickness="0.01"/>
        <contact selfcollide="none"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  {
    SpecPtr spec(
        mj_parseXMLString(vol_strain_xml, nullptr, error, sizeof(error)),
        mj_deleteSpec);
    ASSERT_THAT(spec, NotNull()) << error;
    mjsFlex* flex = mjs_asFlex(mjs_firstElement(spec.get(), mjOBJ_FLEX));
    mjsEquality* eq =
        mjs_asEquality(mjs_firstElement(spec.get(), mjOBJ_EQUALITY));
    ASSERT_THAT(flex, NotNull());
    ASSERT_THAT(eq, NotNull());

    // Out-of-range cell index in volume mode.
    eq->data[0] = 1;
    MjModelPtr m_bad_cell(mj_compile(spec.get(), nullptr));
    EXPECT_THAT(m_bad_cell, IsNull());
    EXPECT_THAT(mjs_getError(spec.get()),
                HasSubstr("flex strain cell index out of range"));
    eq->data[0] = 0;

    // Volumetric strain equality (data[1] >= 0) on shell flex (elastic2d != 0).
    flex->elastic2d = 1;
    MjModelPtr m_mismatch(mj_compile(spec.get(), nullptr));
    EXPECT_THAT(m_mismatch, IsNull());
    EXPECT_THAT(mjs_getError(spec.get()),
                HasSubstr("volumetric flex strain equality on shell flex"));

    // Out-of-range face index in shell mode (1x1x1 has 6 faces: 0..5).
    eq->data[0] = 6;
    eq->data[1] = -1;
    eq->data[2] = -1;
    MjModelPtr m_bad_face(mj_compile(spec.get(), nullptr));
    EXPECT_THAT(m_bad_face, IsNull());
    EXPECT_THAT(mjs_getError(spec.get()),
                HasSubstr("flex strain face index out of range"));
  }
}

TEST_F(UserFlexTest, InterpolatedElastic2dSwitchValidation) {
  using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
  char error[1024] = {0};

  // Volume -> shell switch on a 2x2x2 grid leaves interior node (1, 1, 1)
  // attached to an unreferenced 3-DOF slide body and must fail to compile.
  {
    SpecPtr spec(mj_makeSpec(), mj_deleteSpec);
    mjsFlex* flex = MakeTrilinearBox(spec.get(), /*shell=*/false);
    ASSERT_THAT(flex, NotNull()) << mjs_getError(spec.get());
    flex->elastic2d = 1;
    flex->thickness = 0.01;
    MjModelPtr m(mj_compile(spec.get(), nullptr));
    EXPECT_THAT(m, IsNull());
    EXPECT_THAT(mjs_getError(spec.get()),
                HasSubstr("has unpinned interior node 13 with free DOFs"));
  }

  // Shell -> volume switch on a mesh with an empty corner cell leaves the
  // empty-cell-only corner nodes attached to zero-stiffness 3-DOF slide bodies
  // and must fail to compile.
  static constexpr char empty_cell_shell_xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="flex" type="direct" dim="3" dof="trilinear" mass="1"
                cellcount="2 2 1" radius=".001"
                point="0 0 0  1 0 0  0 0.4 0  0 0 1  0.4 0 0  0 1 0"
                element="0 1 2 3  0 4 5 3">
        <contact selfcollide="none"/>
        <elasticity young="1e4" poisson="0.3" thickness="0.01"
                    elastic2d="bend"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  {
    SpecPtr spec(
        mj_parseXMLString(empty_cell_shell_xml, nullptr, error, sizeof(error)),
        mj_deleteSpec);
    ASSERT_THAT(spec, NotNull()) << error;
    mjsFlex* flex = mjs_asFlex(mjs_firstElement(spec.get(), mjOBJ_FLEX));
    ASSERT_THAT(flex, NotNull());
    flex->elastic2d = 0;
    MjModelPtr m(mj_compile(spec.get(), nullptr));
    EXPECT_THAT(m, IsNull());
    EXPECT_THAT(mjs_getError(spec.get()),
                HasSubstr("has unpinned node 16 in empty cell with free DOFs"));
  }
}

TEST_F(UserFlexTest, DefaultArenaSize) {
  constexpr size_t kMB = 1 << 20;

  // 40x40 2D grid with default contact settings (selfcollide="auto", nv =
  // 4800): symbolically sized instead of quadratic in nv (< 256 MB instead
  // of 1.15 GB)
  static constexpr char kGrid40Xml[] = R"(
  <mujoco>
    <option integrator="discrete"/>
    <worldbody>
      <geom type="plane" size="2 2 .01"/>
      <flexcomp name="cloth" type="grid" dim="2" count="40 40 1"
                spacing=".01 .01 .01" pos="0 0 .05" radius=".002" mass="1">
        <elasticity young="1e3" damping="1"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  char error[1024] = {0};
  MjModelPtr grid40 = LoadModelFromString(kGrid40Xml, error, sizeof(error));
  ASSERT_THAT(grid40, NotNull()) << error;
  EXPECT_LE(grid40->narena, 256 * kMB);
  {
    MjDataPtr d(mj_makeData(grid40.get()));
    for (int step = 0; step < 5; step++) {
      mj_step(grid40.get(), d.get());
    }
    for (int w = 0; w < mjNWARNING; w++) {
      EXPECT_EQ(d->warning[w].number, 0) << "warning " << w;
    }
  }

  // 60x60 2D grid with default contact settings (nv = 10800): fits in < 256 MB
  // and steps cleanly
  static constexpr char kGrid60Xml[] = R"(
  <mujoco>
    <option integrator="discrete"/>
    <worldbody>
      <geom type="plane" size="2 2 .01"/>
      <flexcomp name="cloth" type="grid" dim="2" count="60 60 1"
                spacing=".01 .01 .01" pos="0 0 .05" radius=".002" mass="1">
        <elasticity young="1e3" damping="1"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr grid60 = LoadModelFromString(kGrid60Xml, error, sizeof(error));
  ASSERT_THAT(grid60, NotNull()) << error;
  EXPECT_LE(grid60->narena, 256 * kMB);
  {
    MjDataPtr d(mj_makeData(grid60.get()));
    for (int step = 0; step < 5; step++) {
      mj_step(grid60.get(), d.get());
    }
    for (int w = 0; w < mjNWARNING; w++) {
      EXPECT_EQ(d->warning[w].number, 0) << "warning " << w;
    }
  }

  // 16^3 3D grid (nv = 12288): symbolic Cholesky sizing with 1.5x safety
  // factor covers 3D fill (< 300 MB instead of 6.7 GB)
  static constexpr char kGrid3dXml[] = R"(
  <mujoco>
    <option integrator="discrete"/>
    <worldbody>
      <geom type="plane" size="2 2 .01"/>
      <flexcomp name="cube" type="grid" dim="3" count="16 16 16"
                spacing=".01 .01 .01" pos="0 0 .1" radius=".002" mass="10">
        <contact selfcollide="none"/>
        <elasticity young="1e2" damping="0.01"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr grid3d = LoadModelFromString(kGrid3dXml, error, sizeof(error));
  ASSERT_THAT(grid3d, NotNull()) << error;
  EXPECT_LE(grid3d->narena, 300 * kMB);
  {
    MjDataPtr d(mj_makeData(grid3d.get()));
    for (int step = 0; step < 5; step++) {
      mj_step(grid3d.get(), d.get());
    }
    for (int w = 0; w < mjNWARNING; w++) {
      EXPECT_EQ(d->warning[w].number, 0) << "warning " << w;
    }
  }

  // two colliding 40x40 grids (nv = 9600): < 256 MB instead of 4.2 GB, steps
  // cleanly
  static constexpr char kTwoGrids40Xml[] = R"(
  <mujoco>
    <option integrator="discrete"/>
    <worldbody>
      <geom type="plane" size="2 2 .01"/>
      <flexcomp name="c1" type="grid" dim="2" count="40 40 1"
                spacing=".01 .01 .01" pos="0 0 .05" radius=".002" mass="1">
        <elasticity young="1e3" damping="1"/>
      </flexcomp>
      <flexcomp name="c2" type="grid" dim="2" count="40 40 1"
                spacing=".01 .01 .01" pos="0 0 .08" radius=".002" mass="1">
        <elasticity young="1e3" damping="1"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";
  MjModelPtr two_grids40 =
      LoadModelFromString(kTwoGrids40Xml, error, sizeof(error));
  ASSERT_THAT(two_grids40, NotNull()) << error;
  EXPECT_LE(two_grids40->narena, 256 * kMB);
  {
    MjDataPtr d(mj_makeData(two_grids40.get()));
    for (int step = 0; step < 5; step++) {
      mj_step(two_grids40.get(), d.get());
    }
    for (int w = 0; w < mjNWARNING; w++) {
      EXPECT_EQ(d->warning[w].number, 0) << "warning " << w;
    }
  }
}

// Cells that are only grazed by a tiny mesh sliver (within kSliverMargin = 2%
// of the cell size) are pruned as empty: their stiffness block is zero, their
// exterior-only nodes are pinned, and vertices inside the pruned sliver retain
// their exact flex_vert0 and extrapolate from the nearest active cell in
// mju_cellLookup without snapping to the cell boundary.
TEST_F(UserFlexTest, SliverCellPruningAndExtrapolation) {
  // 2x1x1 grid over x in [0, 1] (anchor point at x=1 extends the bounding box):
  // - Cell 0 (x in [0, 0.5]) is 100% full.
  // - Cell 1 (x in [0.5, 1.0]) only has a 1% sliver x in [0.5, 0.505]
  //   (penetration depth 0.005 / 0.5 = 1% < kSliverMargin = 2%).
  static constexpr char kSliverXml[] = R"(
  <mujoco>
    <option gravity="0 0 -9.81" integrator="discrete" timestep="2e-3"/>
    <worldbody>
      <body name="parent">
        <inertial mass="1" pos="0 0 0" diaginertia="0.01 0.01 0.01"/>
        <flexcomp name="sliver" type="direct" dim="3" dof="DOF_TYPE" mass="1.0"
                  cellcount="2 1 1" radius="0.001"
                  point="0.0 0 0  0.5 0 0  0.0 1 0  0.5 1 0
                         0.0 0 1  0.5 0 1  0.0 1 1  0.5 1 1
                         0.505 0 0  0.505 1 0  0.505 0 1  0.505 1 1
                         1.0 0.5 0.5"
                  element="0 1 2 4  3 2 1 7  5 1 4 7  6 2 4 7  1 2 4 7
                           1 8 3 5  9 3 8 11  10 8 5 11  7 3 5 11  8 3 5 11">
          <pin id="0 2 4 6"/>
          <contact selfcollide="auto"/>
          <elasticity young="1e4" poisson="0.3" damping="0.01"/>
        </flexcomp>
      </body>
    </worldbody>
  </mujoco>
  )";

  for (const char* dof : {"trilinear", "quadratic"}) {
    std::string xml = kSliverXml;
    xml.replace(xml.find("DOF_TYPE"), 8, dof);
    char error[1024] = {0};
    using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
    SpecPtr spec(mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error)),
                 mj_deleteSpec);
    ASSERT_THAT(spec.get(), NotNull()) << dof << ": " << error;

    // Compile twice to verify both flexcomp MarkEmptyCells and mjCFlex::Compile
    // recomputation.
    MjModelPtr m1(mj_compile(spec.get(), nullptr));
    ASSERT_THAT(m1.get(), NotNull()) << dof << ": " << mjs_getError(spec.get());
    MjModelPtr m(mj_compile(spec.get(), nullptr));
    ASSERT_THAT(m.get(), NotNull()) << dof << ": " << mjs_getError(spec.get());

    int order = (std::string(dof) == "trilinear") ? 1 : 2;
    int npe = (order + 1) * (order + 1) * (order + 1);
    int ndof_elem = 3 * npe;
    int block = ndof_elem * ndof_elem;
    ASSERT_EQ(m->nflexstiffness, 2 * block);

    // Cell 0 has non-zero stiffness; Cell 1 (1% sliver) is pruned as empty.
    double norm0 = 0, norm1 = 0;
    for (int i = 0; i < block; i++) {
      norm0 += m->flex_stiffness[i] * m->flex_stiffness[i];
      norm1 += m->flex_stiffness[block + i] * m->flex_stiffness[block + i];
    }
    EXPECT_GT(norm0, 0) << dof;
    EXPECT_EQ(norm1, 0) << dof << ": 1% sliver cell must be pruned as empty";

    // Exterior nodes of pruned Cell 1 (gi > order) must be pinned to parent.
    int parent_bid = mj_name2id(m.get(), mjOBJ_BODY, "parent");
    int ny = order + 1, nz = order + 1;
    int nadr = m->flex_nodeadr[0];
    for (int gi = order + 1; gi <= 2 * order; gi++) {
      for (int gj = 0; gj < ny; gj++) {
        for (int gk = 0; gk < nz; gk++) {
          int bid = m->flex_nodebodyid[nadr + gi * ny * nz + gj * nz + gk];
          EXPECT_EQ(bid, parent_bid)
              << dof << " gi=" << gi << " gj=" << gj << " gk=" << gk;
        }
      }
    }

    // Vertices 8..11 at x = 0.505 retain their exact unprojected parametric
    // coordinate flex_vert0[x] = 0.505, are pre-assigned to active Cell 0 in
    // flex_vertcell, and extrapolate from Cell 0 in mj_flex without snapping.
    for (int v = 0; v < m->nflexvert; v++) {
      EXPECT_EQ(m->flex_vertcell[v], 0) << dof << " vertex " << v;
    }
    for (int v = 8; v < m->nflexvert; v++) {
      EXPECT_NEAR(m->flex_vert0[3 * v + 0], 0.505, MjTol(1e-12, 1e-5))
          << dof << " vertex " << v;
    }

    MjDataPtr d = MakeData(m);
    mj_forward(m.get(), d.get());
    EXPECT_EQ(d->ncon, 0)
        << dof << ": redirected sliver vertices share active cell 0 nodes";
    for (int v = 8; v < m->nflexvert; v++) {
      EXPECT_NEAR(d->flexvert_xpos[3 * v + 0], 0.505, MjTol(1e-12, 1e-5))
          << dof << " vertex " << v
          << " must retain exact initial position without snapping";
    }
    for (int step = 0; step < 50; step++) {
      mj_step(m.get(), d.get());
      ASSERT_EQ(d->warning[mjWARN_BADQACC].number, 0) << dof;
    }
  }
}

}  // namespace
}  // namespace mujoco
