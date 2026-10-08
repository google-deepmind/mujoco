#include "plugin/plasticity/j2.h"

#include <array>
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "test/fixture.h"

#include <mujoco/mujoco.h>

namespace mujoco::plugin::plasticity {
namespace {

constexpr mjtNum kYieldStress = 250.0;

constexpr mjtNum kDoubleTolerance = 1e-10;
constexpr mjtNum kFloatTolerance = 2e-2;

mjtNum Tol() { return MjTol(kDoubleTolerance, kFloatTolerance); }

// -----------------------------------------------------------------------------
// Register the plugin once for this standalone test executable.
// -----------------------------------------------------------------------------

void EnsurePluginRegistered() {
  static const bool registered = []() {
    J2::RegisterPlugin();
    return true;
  }();

  (void)registered;
}

// -----------------------------------------------------------------------------
// One tetrahedron with native elasticity + J2 plugin.
//
// Gravity and elastic damping are disabled so that:
//
//   qfrc_passive
//
// contains:
//
//   native flex elasticity + J2 plastic correction
//
// and:
//
//   qfrc_spring
//
// contains only the native flex elasticity.
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

TestModel MakeTestModel() {
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
          young="100000"
          poisson="0.25"
          damping="0"/>

      <plugin plugin="mujoco.plasticity.j2">
        <config
            key="yield"
            value="250"/>
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
// Store the tetrahedron's reference world-space positions.
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
// Apply a desired affine deformation to the flex by changing the qpos of the
// three slide joints generated for each free flex vertex.
//
// The test model has no parent rotation, so the generated slide axes are the
// world x/y/z directions.
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

    // flexcomp dof="full" creates three translational slide joints.
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
// Check that qpos really produced the affine deformation requested above.
//
// This prevents the force test from silently succeeding with a malformed
// configuration setter.
// -----------------------------------------------------------------------------

void ExpectAffineConfiguration(
    const mjModel* model, const mjData* data,
    const std::array<mjtNum, 12>& reference_positions, const mjtNum F[9]) {
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

    for (int row = 0; row < 3; ++row) {
      const mjtNum expected = origin[row] + F[3 * row + 0] * X[0] +
                              F[3 * row + 1] * X[1] + F[3 * row + 2] * X[2];

      EXPECT_NEAR(data->flexvert_xpos[3 * (vert_adr + vertex) + row], expected,
                  Tol());
    }
  }
}

// =============================================================================
// Below yield:
//
// E = diag(e, -e, 0)
//
// with e = 0.001.
//
// sigma_eq = 2 G sqrt(3) e
//
// For G = 40000:
//
// sigma_eq ~= 138.56 < 250.
//
// Therefore J2 must add exactly zero correction.
// =============================================================================

TEST(J2ForceIntegrationTest, ElasticForwardMatchesNativeFlexForce) {
  auto test = MakeTestModel();

  ASSERT_NE(test.model, nullptr);
  ASSERT_NE(test.data, nullptr);

  const auto reference_positions = GetFlexPositions(test.model, test.data);

  constexpr mjtNum e = 0.001;

  // Choose F such that:
  //
  //   E = 1/2 (F^T F - I)
  //     = diag(e, -e, 0)
  //
  const mjtNum F[9] = {
      std::sqrt(mjtNum(1.0) + mjtNum(2.0) * e),
      0,
      0,
      0,
      std::sqrt(mjtNum(1.0) - mjtNum(2.0) * e),
      0,
      0,
      0,
      1,
  };

  SetAffineDeformation(test.model, test.data, reference_positions, F);

  ExpectAffineConfiguration(test.model, test.data, reference_positions, F);

  const auto reference = internal::BuildElementReference(test.model, 0, 0);

  const mjtNum expected_equivalent_stress =
      mjtNum(2.0) * reference.shear_modulus * std::sqrt(mjtNum(3.0)) * e;

  ASSERT_LT(expected_equivalent_stress, kYieldStress);

  mjtNum native_norm_squared = 0;

  for (int dof = 0; dof < test.model->nv; ++dof) {
    native_norm_squared +=
        test.data->qfrc_spring[dof] * test.data->qfrc_spring[dof];

    EXPECT_NEAR(test.data->qfrc_passive[dof], test.data->qfrc_spring[dof],
                Tol());
  }

  // Ensure this is not a vacuous zero-force comparison.
  EXPECT_GT(std::sqrt(native_norm_squared), mjtNum(1.0));
}

// =============================================================================
// Above yield:
//
// E = diag(e, -e, 0)
//
// is exactly traceless.
//
// For perfect J2 plasticity:
//
//   s_trial = 2 G E
//
// and radial return gives:
//
//   s_corrected
//     = (sigma_y / sigma_eq_trial) s_trial.
//
// Because the native tetrahedral force is linear in E at fixed current
// configuration, the complete plugin result must satisfy:
//
//   qfrc_passive
//     = (sigma_y / sigma_eq_trial) qfrc_spring.
//
// This exercises the actual mj_forward() plugin callback path.
// =============================================================================

TEST(J2ForceIntegrationTest, PlasticForwardReturnsForceToYieldSurface) {
  auto test = MakeTestModel();

  ASSERT_NE(test.model, nullptr);
  ASSERT_NE(test.data, nullptr);

  const auto reference_positions = GetFlexPositions(test.model, test.data);

  constexpr mjtNum e = 0.01;

  const mjtNum F[9] = {
      std::sqrt(mjtNum(1.0) + mjtNum(2.0) * e),
      0,
      0,
      0,
      std::sqrt(mjtNum(1.0) - mjtNum(2.0) * e),
      0,
      0,
      0,
      1,
  };

  SetAffineDeformation(test.model, test.data, reference_positions, F);

  ExpectAffineConfiguration(test.model, test.data, reference_positions, F);

  const auto reference = internal::BuildElementReference(test.model, 0, 0);

  const mjtNum trial_equivalent_stress =
      mjtNum(2.0) * reference.shear_modulus * std::sqrt(mjtNum(3.0)) * e;

  ASSERT_GT(trial_equivalent_stress, kYieldStress);

  const mjtNum radial_scale = kYieldStress / trial_equivalent_stress;

  ASSERT_GT(radial_scale, mjtNum(0.0));
  ASSERT_LT(radial_scale, mjtNum(1.0));

  mjtNum native_norm_squared = 0;
  mjtNum corrected_norm_squared = 0;

  for (int dof = 0; dof < test.model->nv; ++dof) {
    native_norm_squared +=
        test.data->qfrc_spring[dof] * test.data->qfrc_spring[dof];

    corrected_norm_squared +=
        test.data->qfrc_passive[dof] * test.data->qfrc_passive[dof];

    EXPECT_NEAR(test.data->qfrc_passive[dof],
                radial_scale * test.data->qfrc_spring[dof], Tol());
  }

  const mjtNum native_norm = std::sqrt(native_norm_squared);

  const mjtNum corrected_norm = std::sqrt(corrected_norm_squared);

  ASSERT_GT(native_norm, mjtNum(1.0));

  EXPECT_NEAR(corrected_norm, radial_scale * native_norm, Tol());

  EXPECT_LT(corrected_norm, native_norm);

  // mj_forward() evaluates the candidate constitutive state but must not
  // commit irreversible history. Advance() owns that transition.
  const mjtNum* state =
      test.data->plugin_state + test.model->plugin_stateadr[0];

  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(state[i], mjtNum(0.0), Tol());
  }
}

// =============================================================================
// History-dependent integration test.
//
// 1. Load plastically to:
//
//      E1 = diag(e1, -e1, 0)
//
// 2. mj_forward() computes the radial-return candidate but does not commit it.
//
// 3. Advance() commits E_p.
//
// 4. Unload slightly to:
//
//      E2 = diag(e2, -e2, 0)
//
//    where the new elastic trial state lies strictly inside the yield surface.
//
// 5. Verify:
//
//      - plastic history survives,
//      - unloading produces no additional plastic strain,
//      - the passive force corresponds to E2 - E_p,
//        not to the native elastic response at E2.
// =============================================================================

TEST(J2ForceIntegrationTest, PlasticHistorySurvivesElasticUnloading) {
  auto test = MakeTestModel();

  ASSERT_NE(test.model, nullptr);
  ASSERT_NE(test.data, nullptr);

  ASSERT_EQ(test.model->nplugin, 1);

  auto* plasticity = reinterpret_cast<J2*>(test.data->plugin_data[0]);

  ASSERT_NE(plasticity, nullptr);

  const auto reference_positions = GetFlexPositions(test.model, test.data);

  const auto reference = internal::BuildElementReference(test.model, 0, 0);

  mjtNum* state = test.data->plugin_state + test.model->plugin_stateadr[0];

  // ---------------------------------------------------------------------------
  // 1. Plastic loading.
  // ---------------------------------------------------------------------------

  constexpr mjtNum e1 = 0.01;

  const mjtNum F1[9] = {
      std::sqrt(mjtNum(1.0) + mjtNum(2.0) * e1),
      0,
      0,
      0,
      std::sqrt(mjtNum(1.0) - mjtNum(2.0) * e1),
      0,
      0,
      0,
      1,
  };

  SetAffineDeformation(test.model, test.data, reference_positions, F1);

  ExpectAffineConfiguration(test.model, test.data, reference_positions, F1);

  const mjtNum trial_stress_1 =
      mjtNum(2.0) * reference.shear_modulus * std::sqrt(mjtNum(3.0)) * e1;

  ASSERT_GT(trial_stress_1, kYieldStress);

  const mjtNum radial_scale = kYieldStress / trial_stress_1;

  ASSERT_GT(radial_scale, mjtNum(0.0));
  ASSERT_LT(radial_scale, mjtNum(1.0));

  // mj_forward() has evaluated the plastic candidate, but Compute() must not
  // have committed irreversible history yet.
  for (int i = 0; i < 6; ++i) {
    ASSERT_NEAR(state[i], mjtNum(0.0), Tol());
  }

  // ---------------------------------------------------------------------------
  // 2. Commit the plastic history.
  // ---------------------------------------------------------------------------

  plasticity->Advance(test.model, test.data, 0);

  // For this traceless proportional loading:
  //
  //   E_p = (1 - radial_scale) E1
  //
  const mjtNum expected_ep = (mjtNum(1.0) - radial_scale) * e1;

  EXPECT_NEAR(state[0], expected_ep, Tol());
  EXPECT_NEAR(state[1], -expected_ep, Tol());
  EXPECT_NEAR(state[2], mjtNum(0.0), Tol());
  EXPECT_NEAR(state[3], mjtNum(0.0), Tol());
  EXPECT_NEAR(state[4], mjtNum(0.0), Tol());
  EXPECT_NEAR(state[5], mjtNum(0.0), Tol());

  std::array<mjtNum, 6> committed_state{};

  for (int i = 0; i < 6; ++i) {
    committed_state[i] = state[i];
  }

  // ---------------------------------------------------------------------------
  // 3. Elastic unloading.
  //
  // e2 is still positive, but smaller than e1.
  //
  // The elastic strain after unloading is:
  //
  //   e_elastic = e2 - e_p
  //
  // and must lie strictly inside the yield surface.
  // ---------------------------------------------------------------------------

  constexpr mjtNum e2 = 0.009;

  const mjtNum elastic_e2 = e2 - expected_ep;

  const mjtNum unloading_trial_stress = mjtNum(2.0) * reference.shear_modulus *
                                        std::sqrt(mjtNum(3.0)) *
                                        std::abs(elastic_e2);

  ASSERT_LT(unloading_trial_stress, kYieldStress);

  const mjtNum F2[9] = {
      std::sqrt(mjtNum(1.0) + mjtNum(2.0) * e2),
      0,
      0,
      0,
      std::sqrt(mjtNum(1.0) - mjtNum(2.0) * e2),
      0,
      0,
      0,
      1,
  };

  SetAffineDeformation(test.model, test.data, reference_positions, F2);

  ExpectAffineConfiguration(test.model, test.data, reference_positions, F2);

  // Compute() during mj_forward() must not alter the committed state.
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(state[i], committed_state[i], Tol());
  }

  // ---------------------------------------------------------------------------
  // 4. Check the actual force after unloading.
  //
  // Since E2 and E_p are collinear:
  //
  //   E_elastic = E2 - E_p
  //
  // and therefore, at this fixed current configuration:
  //
  //   f_passive =
  //       (elastic_e2 / e2) f_native(E2).
  // ---------------------------------------------------------------------------

  const mjtNum unloading_force_scale = elastic_e2 / e2;

  ASSERT_GT(unloading_force_scale, mjtNum(0.0));
  ASSERT_LT(unloading_force_scale, mjtNum(1.0));

  mjtNum native_norm_squared = 0;
  mjtNum passive_norm_squared = 0;

  for (int dof = 0; dof < test.model->nv; ++dof) {
    const mjtNum native_force = test.data->qfrc_spring[dof];

    const mjtNum expected_force = unloading_force_scale * native_force;

    const mjtNum passive_force = test.data->qfrc_passive[dof];

    native_norm_squared += native_force * native_force;

    passive_norm_squared += passive_force * passive_force;

    EXPECT_NEAR(passive_force, expected_force, Tol());
  }

  const mjtNum native_norm = std::sqrt(native_norm_squared);

  const mjtNum passive_norm = std::sqrt(passive_norm_squared);

  ASSERT_GT(native_norm, mjtNum(1.0));

  EXPECT_NEAR(passive_norm, unloading_force_scale * native_norm, Tol());

  EXPECT_LT(passive_norm, native_norm);

  // ---------------------------------------------------------------------------
  // 5. Commit the unloading step.
  //
  // Because unloading was elastic, E_p must remain exactly unchanged.
  // ---------------------------------------------------------------------------

  plasticity->Advance(test.model, test.data, 0);

  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(state[i], committed_state[i], Tol());
  }
}

}  // namespace
}  // namespace mujoco::plugin::plasticity