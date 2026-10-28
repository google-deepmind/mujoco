#include "plugin/plasticity/j2.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "test/fixture.h"

#include <mujoco/mujoco.h>

namespace mujoco::plugin::plasticity {
namespace {

constexpr mjtNum kYoungModulus = 70e9;
constexpr mjtNum kPoissonRatio = 0.33;
constexpr mjtNum kYieldStress = 250e6;

constexpr mjtNum kTetEdgeLength = 0.1;  // 100 mm

// -----------------------------------------------------------------------------
// Precision-aware tolerances.
//
// Float tolerances are temporary until calibrated with MJTOL_SCALE=0.
// -----------------------------------------------------------------------------

constexpr mjtNum kStateDoubleTolerance = 3e-16;
constexpr mjtNum kStateFloatTolerance = 5e-7;

constexpr mjtNum kGeometryDoubleTolerance = 1e-12;
constexpr mjtNum kGeometryFloatTolerance = 1e-6;

constexpr mjtNum kForceDoubleTolerance = 1e-6;
constexpr mjtNum kForceFloatTolerance = 1e2;

constexpr mjtNum kShearRelativeDoubleTolerance = 1e-10;
constexpr mjtNum kShearRelativeFloatTolerance = 1e-5;

constexpr mjtNum kMillimeterDoubleTolerance = 1e-3;
constexpr mjtNum kMillimeterFloatTolerance = 1e-4;

constexpr mjtNum kResidualRatioDoubleTolerance = 1e-10;
constexpr mjtNum kResidualRatioFloatTolerance = 2e-4;

mjtNum StateTol() { return MjTol(kStateDoubleTolerance, kStateFloatTolerance); }

mjtNum GeometryTol() {
  return MjTol(kGeometryDoubleTolerance, kGeometryFloatTolerance);
}

mjtNum ForceTol() { return MjTol(kForceDoubleTolerance, kForceFloatTolerance); }

mjtNum ShearRelativeTol() {
  return MjTol(kShearRelativeDoubleTolerance, kShearRelativeFloatTolerance);
}

mjtNum MillimeterTol() {
  return MjTol(kMillimeterDoubleTolerance, kMillimeterFloatTolerance);
}

mjtNum ResidualRatioTol() {
  return MjTol(kResidualRatioDoubleTolerance, kResidualRatioFloatTolerance);
}

// -----------------------------------------------------------------------------
// Register the J2 plugin once for this standalone test executable.
// -----------------------------------------------------------------------------

void EnsurePluginRegistered() {
  static const bool registered = []() {
    J2::RegisterPlugin();
    return true;
  }();

  (void)registered;
}

// -----------------------------------------------------------------------------
// Physical single-tetrahedron aluminium specimen.
//
// Reference geometry:
//
//          z
//          |
//          3
//          |
//          0------1  x
//         /
//        2
//       y
//
// The three reference edges from vertex 0 are 100 mm long.
//
// The tetrahedron volume is:
//
//   V = 0.1^3 / 6 = 1.666...e-4 m^3
//
// giving a physical aluminium mass of approximately:
//
//   rho V = 2700 * V = 0.45 kg.
//
// Mass does not affect the quasi-static force checks below, but keeping the
// physical value makes the benchmark self-consistent.
// -----------------------------------------------------------------------------

struct TestModel {
  mjSpec* spec = nullptr;
  mjModel* model = nullptr;
  mjData* data = nullptr;

  ~TestModel() {
    if (data) {
      mj_deleteData(data);
    }

    if (model) {
      mj_deleteModel(model);
    }

    if (spec) {
      mj_deleteSpec(spec);
    }
  }
};

TestModel MakeAluminiumTet() {
  EnsurePluginRegistered();

  static constexpr char xml[] = R"(
<mujoco>
  <extension>
    <plugin plugin="mujoco.plasticity.j2"/>
  </extension>

  <option
      gravity="0 0 0"
      timestep="0.001"
      integrator="Euler"/>

  <worldbody>
    <flexcomp
        name="aluminium_tet"
        type="direct"
        dim="3"
        radius="0.001"
        mass="0.45"
        point="
          0   0   0
          0.1 0   0
          0   0.1 0
          0   0   0.1"
        element="0 1 2 3">

      <elasticity
          young="70000000000"
          poisson="0.33"
          damping="0"/>

      <plugin plugin="mujoco.plasticity.j2">
        <config
            key="yield"
            value="250000000"/>
      </plugin>
    </flexcomp>
  </worldbody>
</mujoco>
)";

  TestModel result;

  char error[1024] = "";

  result.spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));

  if (!result.spec) {
    ADD_FAILURE() << "XML parsing failed:\n" << error;

    return result;
  }

  result.model = mj_compile(result.spec, nullptr);

  if (!result.model) {
    ADD_FAILURE() << "Model compilation failed:\n" << mjs_getError(result.spec);

    return result;
  }

  result.data = mj_makeData(result.model);

  if (!result.data) {
    ADD_FAILURE() << "mj_makeData failed.";

    return result;
  }

  mj_forward(result.model, result.data);

  return result;
}

// -----------------------------------------------------------------------------
// Read the four current flex-vertex positions.
// -----------------------------------------------------------------------------

std::array<mjtNum, 12> GetFlexPositions(const mjModel* model,
                                        const mjData* data) {
  std::array<mjtNum, 12> positions{};

  EXPECT_EQ(model->flex_vertnum[0], 4);

  const int vert_adr = model->flex_vertadr[0];

  for (int vertex = 0; vertex < 4; ++vertex) {
    for (int axis = 0; axis < 3; ++axis) {
      positions[3 * vertex + axis] =
          data->flexvert_xpos[3 * (vert_adr + vertex) + axis];
    }
  }

  return positions;
}

// -----------------------------------------------------------------------------
// Impose an affine deformation gradient F through the slide-joint qpos values
// generated by flexcomp.
//
// F is row-major.
// -----------------------------------------------------------------------------

void SetAffineDeformation(const mjModel* model, mjData* data,
                          const std::array<mjtNum, 12>& reference_positions,
                          const mjtNum F[9]) {
  const int vert_adr = model->flex_vertadr[0];

  const mjtNum origin[3] = {
      reference_positions[0],
      reference_positions[1],
      reference_positions[2],
  };

  for (int vertex = 0; vertex < 4; ++vertex) {
    const mjtNum X[3] = {
        reference_positions[3 * vertex + 0] - origin[0],
        reference_positions[3 * vertex + 1] - origin[1],
        reference_positions[3 * vertex + 2] - origin[2],
    };

    mjtNum target[3];

    for (int row = 0; row < 3; ++row) {
      target[row] = origin[row] + F[3 * row + 0] * X[0] +
                    F[3 * row + 1] * X[1] + F[3 * row + 2] * X[2];
    }

    const mjtNum displacement[3] = {
        target[0] - reference_positions[3 * vertex + 0],
        target[1] - reference_positions[3 * vertex + 1],
        target[2] - reference_positions[3 * vertex + 2],
    };

    const int global_vertex = vert_adr + vertex;
    const int body_id = model->flex_vertbodyid[global_vertex];

    ASSERT_EQ(model->body_jntnum[body_id], 3);

    const int joint_adr = model->body_jntadr[body_id];

    for (int local_joint = 0; local_joint < 3; ++local_joint) {
      const int joint = joint_adr + local_joint;

      ASSERT_EQ(model->jnt_type[joint], mjJNT_SLIDE);

      const mjtNum* axis = model->jnt_axis + 3 * joint;

      const mjtNum amount = displacement[0] * axis[0] +
                            displacement[1] * axis[1] +
                            displacement[2] * axis[2];

      const int qpos_adr = model->jnt_qposadr[joint];

      data->qpos[qpos_adr] = model->qpos0[qpos_adr] + amount;
    }
  }

  mj_forward(model, data);
}

// -----------------------------------------------------------------------------
// Euclidean length between two local flex vertices.
// -----------------------------------------------------------------------------

mjtNum EdgeLength(const mjModel* model, const mjData* data, int vertex_a,
                  int vertex_b) {
  const int vert_adr = model->flex_vertadr[0];

  const mjtNum* xa = data->flexvert_xpos + 3 * (vert_adr + vertex_a);
  const mjtNum* xb = data->flexvert_xpos + 3 * (vert_adr + vertex_b);

  mjtNum length_squared = 0;

  for (int axis = 0; axis < 3; ++axis) {
    const mjtNum dx = xb[axis] - xa[axis];

    length_squared += dx * dx;
  }

  return std::sqrt(length_squared);
}

// -----------------------------------------------------------------------------
// Generalized-force norm.
// -----------------------------------------------------------------------------

mjtNum ForceNorm(const mjtNum* force, int size) {
  mjtNum norm_squared = 0;

  for (int i = 0; i < size; ++i) {
    norm_squared += force[i] * force[i];
  }

  return std::sqrt(norm_squared);
}

// =============================================================================
// Benchmark 1:
//
// A small deviatoric compression stays below yield.
//
// Loading:
//
//   E = diag(-0.002, +0.002, 0)
//
// gives:
//
//   sigma_eq = 2 G sqrt(3) e
//
// which is below 250 MPa for aluminium.
//
// After committing the step, E_p must still be exactly zero.
// Returning to F = I must therefore recover the undeformed, stress-free state.
// =============================================================================

TEST(J2AluminiumSingleTetBenchmark, SubYieldCycleLeavesNoPlasticDeformation) {
  auto test = MakeAluminiumTet();

  ASSERT_NE(test.model, nullptr);
  ASSERT_NE(test.data, nullptr);

  auto* plasticity = reinterpret_cast<J2*>(test.data->plugin_data[0]);

  ASSERT_NE(plasticity, nullptr);

  const auto reference_positions = GetFlexPositions(test.model, test.data);

  const auto reference = internal::BuildElementReference(test.model, 0, 0);

  const mjtNum expected_shear_modulus =
      kYoungModulus / (mjtNum(2.0) * (mjtNum(1.0) + kPoissonRatio));

  EXPECT_NEAR(reference.shear_modulus, expected_shear_modulus,
              ShearRelativeTol() * expected_shear_modulus);

  constexpr mjtNum e = 0.002;

  const mjtNum trial_equivalent_stress =
      mjtNum(2.0) * reference.shear_modulus * std::sqrt(mjtNum(3.0)) * e;

  ASSERT_LT(trial_equivalent_stress, kYieldStress);

  // Construct F so that:
  //
  //   1/2 (F^T F - I)
  //     = diag(-e, +e, 0).
  //
  const mjtNum F_load[9] = {
      std::sqrt(mjtNum(1.0) - mjtNum(2.0) * e),
      0,
      0,
      0,
      std::sqrt(mjtNum(1.0) + mjtNum(2.0) * e),
      0,
      0,
      0,
      1,
  };

  SetAffineDeformation(test.model, test.data, reference_positions, F_load);

  ASSERT_GT(ForceNorm(test.data->qfrc_passive, test.model->nv), mjtNum(1.0));

  plasticity->Advance(test.model, test.data, 0);

  mjtNum* state = test.data->plugin_state + test.model->plugin_stateadr[0];

  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(state[i], mjtNum(0.0), StateTol());
  }

  // Complete unloading.
  const mjtNum identity[9] = {
      1, 0, 0, 0, 1, 0, 0, 0, 1,
  };

  SetAffineDeformation(test.model, test.data, reference_positions, identity);

  EXPECT_NEAR(EdgeLength(test.model, test.data, 0, 1), kTetEdgeLength,
              GeometryTol());

  EXPECT_NEAR(ForceNorm(test.data->qfrc_passive, test.model->nv), mjtNum(0.0),
              ForceTol());
}

// =============================================================================
// Benchmark 2:
//
// Plastic loading followed by unloading to the analytically predicted
// stress-free residual configuration.
//
// Loading:
//
//   E1 = diag(-0.01, +0.01, 0)
//
// Perfect J2 radial return gives:
//
//   E_p = (1 - r) E1
//
// where:
//
//   r = sigma_y / sigma_eq_trial.
//
// If the geometry is subsequently set to:
//
//   E_residual = E_p,
//
// then:
//
//   E_e = E_residual - E_p = 0.
//
// The native flex alone still sees a deformed tetrahedron and therefore
// produces a nonzero spring force. The J2 correction must cancel it.
//
// This is the key physical demonstration:
//
//   deformed geometry + zero restoring force
//
// because the material has acquired a permanent plastic reference state.
// =============================================================================

TEST(J2AluminiumSingleTetBenchmark,
     PlasticCycleHasStressFreeResidualDeformation) {
  auto test = MakeAluminiumTet();

  ASSERT_NE(test.model, nullptr);
  ASSERT_NE(test.data, nullptr);

  auto* plasticity = reinterpret_cast<J2*>(test.data->plugin_data[0]);

  ASSERT_NE(plasticity, nullptr);

  const auto reference_positions = GetFlexPositions(test.model, test.data);

  const auto reference = internal::BuildElementReference(test.model, 0, 0);

  constexpr mjtNum e = 0.01;

  const mjtNum trial_equivalent_stress =
      mjtNum(2.0) * reference.shear_modulus * std::sqrt(mjtNum(3.0)) * e;

  ASSERT_GT(trial_equivalent_stress, kYieldStress);

  const mjtNum radial_scale = kYieldStress / trial_equivalent_stress;

  ASSERT_GT(radial_scale, mjtNum(0.0));
  ASSERT_LT(radial_scale, mjtNum(1.0));

  const mjtNum plastic_magnitude = (mjtNum(1.0) - radial_scale) * e;

  ASSERT_GT(plastic_magnitude, mjtNum(0.0));

  // ---------------------------------------------------------------------------
  // Plastic loading.
  // ---------------------------------------------------------------------------

  const mjtNum F_load[9] = {
      std::sqrt(mjtNum(1.0) - mjtNum(2.0) * e),
      0,
      0,
      0,
      std::sqrt(mjtNum(1.0) + mjtNum(2.0) * e),
      0,
      0,
      0,
      1,
  };

  SetAffineDeformation(test.model, test.data, reference_positions, F_load);

  const mjtNum native_loaded_norm =
      ForceNorm(test.data->qfrc_spring, test.model->nv);

  const mjtNum plastic_loaded_norm =
      ForceNorm(test.data->qfrc_passive, test.model->nv);

  ASSERT_GT(native_loaded_norm, mjtNum(1.0));
  ASSERT_GT(plastic_loaded_norm, mjtNum(1.0));

  // Radial return must reduce the restoring force.
  EXPECT_LT(plastic_loaded_norm, native_loaded_norm);

  // Commit the irreversible history.
  plasticity->Advance(test.model, test.data, 0);

  mjtNum* state = test.data->plugin_state + test.model->plugin_stateadr[0];

  // Expected:
  //
  //   Ep = diag(-plastic_magnitude,
  //             +plastic_magnitude,
  //              0)
  //
  EXPECT_NEAR(state[0], -plastic_magnitude, StateTol());
  EXPECT_NEAR(state[1], plastic_magnitude, StateTol());
  EXPECT_NEAR(state[2], mjtNum(0.0), StateTol());
  EXPECT_NEAR(state[3], mjtNum(0.0), StateTol());
  EXPECT_NEAR(state[4], mjtNum(0.0), StateTol());
  EXPECT_NEAR(state[5], mjtNum(0.0), StateTol());

  // ---------------------------------------------------------------------------
  // Unload to the analytically predicted permanent configuration:
  //
  //   E = Ep.
  //
  // Since:
  //
  //   E = 1/2 (F^T F - I),
  //
  // the corresponding principal stretches are:
  //
  //   lambda_x = sqrt(1 - 2 ep)
  //   lambda_y = sqrt(1 + 2 ep)
  //   lambda_z = 1.
  // ---------------------------------------------------------------------------

  const mjtNum residual_stretch_x =
      std::sqrt(mjtNum(1.0) - mjtNum(2.0) * plastic_magnitude);

  const mjtNum residual_stretch_y =
      std::sqrt(mjtNum(1.0) + mjtNum(2.0) * plastic_magnitude);

  const mjtNum F_residual[9] = {
      residual_stretch_x, 0, 0, 0, residual_stretch_y, 0, 0, 0, 1,
  };

  SetAffineDeformation(test.model, test.data, reference_positions, F_residual);

  // The specimen must still be geometrically deformed.
  const mjtNum residual_x_length = EdgeLength(test.model, test.data, 0, 1);

  const mjtNum expected_residual_x_length = kTetEdgeLength * residual_stretch_x;

  EXPECT_NEAR(residual_x_length, expected_residual_x_length, GeometryTol());

  EXPECT_LT(residual_x_length, kTetEdgeLength);

  // Native elasticity still wants to restore the original shape.
  const mjtNum native_residual_norm =
      ForceNorm(test.data->qfrc_spring, test.model->nv);

  ASSERT_GT(native_residual_norm, mjtNum(1.0));

  // But with the plastic reference strain included, this configuration is
  // stress-free.
  const mjtNum corrected_residual_norm =
      ForceNorm(test.data->qfrc_passive, test.model->nv);

  EXPECT_LT(corrected_residual_norm / native_residual_norm, ResidualRatioTol());

  // For the chosen aluminium parameters and 1% deviatoric Green strain,
  // the x-edge should remain at approximately 99.27 mm rather than 100 mm.
  EXPECT_NEAR(mjtNum(1000.0) * residual_x_length, mjtNum(99.2716),
              MillimeterTol());

  // Evaluating the residual configuration must not change the already
  // committed history.
  EXPECT_NEAR(state[0], -plastic_magnitude, StateTol());
  EXPECT_NEAR(state[1], plastic_magnitude, StateTol());
  EXPECT_NEAR(state[2], mjtNum(0.0), StateTol());
}

}  // namespace
}  // namespace mujoco::plugin::plasticity