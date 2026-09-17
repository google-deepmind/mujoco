#include "plugin/plasticity/j2.h"

#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "test/fixture.h"

#include <mujoco/mujoco.h>

namespace mujoco::plugin::plasticity {
namespace {

constexpr mjtNum kDoubleTolerance = 1e-14;
constexpr mjtNum kFloatTolerance = 1e-5;

mjtNum Tol() { return MjTol(kDoubleTolerance, kFloatTolerance); }

void ExpectNear(const std::array<mjtNum, 9>& actual, const mjtNum expected[9]) {
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(actual[i], expected[i], Tol());
  }
}

// -----------------------------------------------------------------------------
// Pure mathematical tests.
// -----------------------------------------------------------------------------

TEST(J2KinematicsTest, Rest) {
  const mjtNum identity[9] = {
      1, 0, 0, 0, 1, 0, 0, 0, 1,
  };

  const auto kinematics = internal::ComputeKinematics(identity, identity);

  const mjtNum zero[9] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  ExpectNear(kinematics.deformation_gradient, identity);

  ExpectNear(kinematics.green_strain, zero);

  EXPECT_NEAR(kinematics.jacobian, 1.0, Tol());
}

TEST(J2KinematicsTest, RigidRotation) {
  const mjtNum inv_dm[9] = {
      1, 0, 0, 0, 1, 0, 0, 0, 1,
  };

  // 90-degree rotation around z.
  const mjtNum rotation[9] = {
      0, -1, 0, 1, 0, 0, 0, 0, 1,
  };

  const auto kinematics = internal::ComputeKinematics(rotation, inv_dm);

  const mjtNum zero[9] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  ExpectNear(kinematics.deformation_gradient, rotation);

  ExpectNear(kinematics.green_strain, zero);

  EXPECT_NEAR(kinematics.jacobian, 1.0, Tol());
}

TEST(J2KinematicsTest, UniaxialCompression) {
  const mjtNum inv_dm[9] = {
      1, 0, 0, 0, 1, 0, 0, 0, 1,
  };

  const mjtNum ds[9] = {
      1, 0, 0, 0, 1, 0, 0, 0, 0.99,
  };

  const auto kinematics = internal::ComputeKinematics(ds, inv_dm);

  const mjtNum expected_f[9] = {
      1, 0, 0, 0, 1, 0, 0, 0, 0.99,
  };

  // Ezz = 1/2 * (0.99^2 - 1)
  //     = -0.00995
  const mjtNum expected_e[9] = {
      0, 0, 0, 0, 0, 0, 0, 0, -0.00995,
  };

  ExpectNear(kinematics.deformation_gradient, expected_f);

  ExpectNear(kinematics.green_strain, expected_e);

  EXPECT_NEAR(kinematics.jacobian, 0.99, Tol());
}

TEST(J2KinematicsTest, SimpleShear) {
  const mjtNum inv_dm[9] = {
      1, 0, 0, 0, 1, 0, 0, 0, 1,
  };

  constexpr mjtNum gamma = 0.1;

  const mjtNum ds[9] = {
      1, gamma, 0, 0, 1, 0, 0, 0, 1,
  };

  const auto kinematics = internal::ComputeKinematics(ds, inv_dm);

  // For:
  //
  // F =
  // [ 1  gamma  0 ]
  // [ 0    1    0 ]
  // [ 0    0    1 ]
  //
  // E = 1/2 (F^T F - I)
  //
  // E =
  // [ 0          gamma/2    0 ]
  // [ gamma/2    gamma^2/2  0 ]
  // [ 0          0          0 ]
  //
  const mjtNum expected_e[9] = {
      0, 0.05, 0, 0.05, 0.005, 0, 0, 0, 0,
  };

  ExpectNear(kinematics.green_strain, expected_e);

  EXPECT_NEAR(kinematics.jacobian, 1.0, Tol());
}

// -----------------------------------------------------------------------------
// Real MuJoCo flex tests.
// -----------------------------------------------------------------------------

TEST(J2FlexKinematicsTest, RealFlexAtRest) {
  static constexpr char xml[] = R"(
<mujoco>
  <worldbody>
    <flexcomp
        name="tet"
        type="direct"
        dim="3"
        radius="0.001"
        mass="1"
        point="
          0 0 0
          1 0 0
          0 1 0
          0 0 1"
        element="0 1 2 3">
      <elasticity
          young="1e5"
          poisson="0.25"/>
    </flexcomp>
  </worldbody>
</mujoco>
)";

  char error[1024] = "";

  mjSpec* spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));

  ASSERT_NE(spec, nullptr) << error;

  mjModel* model = mj_compile(spec, nullptr);

  ASSERT_NE(model, nullptr) << mjs_getError(spec);

  ASSERT_EQ(model->nflex, 1);

  ASSERT_EQ(model->flex_dim[0], 3);

  ASSERT_EQ(model->flex_elemnum[0], 1);

  ASSERT_EQ(model->flex_vertnum[0], 4);

  ASSERT_GE(model->flex_stiffnessadr[0], 0);

  mjData* data = mj_makeData(model);

  ASSERT_NE(data, nullptr);

  mj_forward(model, data);

  const auto reference = internal::BuildElementReference(model, 0, 0);

  const auto kinematics =
      internal::ComputeElementKinematics(model, data, 0, 0, reference);

  const mjtNum identity[9] = {
      1, 0, 0, 0, 1, 0, 0, 0, 1,
  };

  const mjtNum zero[9] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  ExpectNear(kinematics.deformation_gradient, identity);

  ExpectNear(kinematics.green_strain, zero);

  EXPECT_NEAR(kinematics.jacobian, 1.0, Tol());

  EXPECT_NEAR(reference.volume, 1.0 / 6.0, Tol());

  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

TEST(J2FlexKinematicsTest, TranslationDoesNotCreateStrain) {
  static constexpr char xml[] = R"(
<mujoco>
  <worldbody>
    <flexcomp
        name="tet"
        type="direct"
        dim="3"
        radius="0.001"
        mass="1"
        point="
          0 0 0
          1 0 0
          0 1 0
          0 0 1"
        element="0 1 2 3">
      <elasticity
          young="1e5"
          poisson="0.25"/>
    </flexcomp>
  </worldbody>
</mujoco>
)";

  char error[1024] = "";

  mjSpec* spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));

  ASSERT_NE(spec, nullptr) << error;

  mjModel* model = mj_compile(spec, nullptr);

  ASSERT_NE(model, nullptr) << mjs_getError(spec);

  mjData* data = mj_makeData(model);

  ASSERT_NE(data, nullptr);

  mj_forward(model, data);

  const auto reference = internal::BuildElementReference(model, 0, 0);

  const int vert_adr = model->flex_vertadr[0];

  for (int v = 0; v < model->flex_vertnum[0]; ++v) {
    mjtNum* x = data->flexvert_xpos + 3 * (vert_adr + v);

    x[0] += 2.3;
    x[1] -= 4.7;
    x[2] += 1.1;
  }

  const auto kinematics =
      internal::ComputeElementKinematics(model, data, 0, 0, reference);

  const mjtNum identity[9] = {
      1, 0, 0, 0, 1, 0, 0, 0, 1,
  };

  const mjtNum zero[9] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  ExpectNear(kinematics.deformation_gradient, identity);

  ExpectNear(kinematics.green_strain, zero);

  EXPECT_NEAR(kinematics.jacobian, 1.0, Tol());

  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

TEST(J2FlexKinematicsTest, RealFlexCompression) {
  static constexpr char xml[] = R"(
<mujoco>
  <worldbody>
    <flexcomp
        name="tet"
        type="direct"
        dim="3"
        radius="0.001"
        mass="1"
        point="
          0 0 0
          1 0 0
          0 1 0
          0 0 1"
        element="0 1 2 3">
      <elasticity
          young="1e5"
          poisson="0.25"/>
    </flexcomp>
  </worldbody>
</mujoco>
)";

  char error[1024] = "";

  mjSpec* spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));

  ASSERT_NE(spec, nullptr) << error;

  mjModel* model = mj_compile(spec, nullptr);

  ASSERT_NE(model, nullptr) << mjs_getError(spec);

  mjData* data = mj_makeData(model);

  ASSERT_NE(data, nullptr);

  mj_forward(model, data);

  const auto reference = internal::BuildElementReference(model, 0, 0);

  const int vert_adr = model->flex_vertadr[0];

  // Apply a 1% affine compression along z.
  for (int v = 0; v < model->flex_vertnum[0]; ++v) {
    mjtNum* x = data->flexvert_xpos + 3 * (vert_adr + v);

    x[2] *= 0.99;
  }

  const auto kinematics =
      internal::ComputeElementKinematics(model, data, 0, 0, reference);

  const mjtNum expected_f[9] = {
      1, 0, 0, 0, 1, 0, 0, 0, 0.99,
  };

  const mjtNum expected_e[9] = {
      0, 0, 0, 0, 0, 0, 0, 0, -0.00995,
  };

  ExpectNear(kinematics.deformation_gradient, expected_f);

  ExpectNear(kinematics.green_strain, expected_e);

  EXPECT_NEAR(kinematics.jacobian, 0.99, Tol());

  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

}  // namespace
}  // namespace mujoco::plugin::plasticity