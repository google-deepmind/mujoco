#include "plugin/plasticity/j2.h"

#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "test/fixture.h"

namespace mujoco::plugin::plasticity {
namespace {

constexpr mjtNum kDoubleTolerance = 1e-12;
constexpr mjtNum kFloatTolerance = 1e-3;
constexpr mjtNum kShearModulus = 40000.0;
constexpr mjtNum kYieldStress = 250.0;

mjtNum Tol() { return MjTol(kDoubleTolerance, kFloatTolerance); }

// -----------------------------------------------------------------------------
// Helper: compare two six-component plastic strain states.
// -----------------------------------------------------------------------------

void ExpectPlasticStrainNear(const std::array<mjtNum, 6>& actual,
                             const mjtNum expected[6]) {
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(actual[i], expected[i], Tol());
  }
}

// -----------------------------------------------------------------------------
// Helper: compare two 3x3 tensors.
// -----------------------------------------------------------------------------

void ExpectTensorNear(const std::array<mjtNum, 9>& actual,
                      const std::array<mjtNum, 9>& expected) {
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(actual[i], expected[i], Tol());
  }
}

// =============================================================================
// Elastic regime.
// =============================================================================

// For uniaxial strain:
//
//   E = diag(e, 0, 0)
//
// the J2 equivalent stress is:
//
//   sigma_eq = 2 G |e|
//
// With:
//
//   e = 0.002
//   G = 40000
//
// we obtain:
//
//   sigma_eq = 160 < 250
//
// so no plastic correction must occur.
TEST(J2RadialReturnTest, ElasticStepLeavesStateUnchanged) {
  constexpr mjtNum e = 0.002;

  const std::array<mjtNum, 9> green_strain = {
      e, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  const mjtNum plastic_strain[6] = {
      0, 0, 0, 0, 0, 0,
  };

  const auto trial =
      internal::ComputeTrialState(green_strain, plastic_strain, kShearModulus);

  ASSERT_NEAR(trial.equivalent_stress, 160.0, Tol());

  const auto result = internal::ComputeRadialReturn(
      trial, plastic_strain, kShearModulus, kYieldStress);

  EXPECT_FALSE(result.yielded);

  EXPECT_NEAR(result.plastic_multiplier, 0.0, Tol());

  const mjtNum expected_plastic[6] = {
      0, 0, 0, 0, 0, 0,
  };

  ExpectPlasticStrainNear(result.plastic_strain, expected_plastic);

  ExpectTensorNear(result.deviatoric_stress, trial.deviatoric_stress);

  EXPECT_NEAR(result.equivalent_stress, trial.equivalent_stress, Tol());
}

// -----------------------------------------------------------------------------
// Exactly on the yield surface:
//
//   sigma_eq_trial = sigma_y
//
// Our convention is:
//
//   f_trial <= 0
//
// therefore a point already on the surface does not generate an additional
// plastic increment.
// -----------------------------------------------------------------------------

TEST(J2RadialReturnTest, StateExactlyAtYieldRemainsElastic) {
  constexpr mjtNum e = kYieldStress / (2.0 * kShearModulus);

  const std::array<mjtNum, 9> green_strain = {
      e, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  const mjtNum plastic_strain[6] = {
      0, 0, 0, 0, 0, 0,
  };

  const auto trial =
      internal::ComputeTrialState(green_strain, plastic_strain, kShearModulus);

  ASSERT_NEAR(trial.equivalent_stress, kYieldStress, Tol());

  // Use the computed value itself as sigma_y so that f_trial is exactly zero,
  // independently of floating-point rounding in the analytical expression.
  const auto result = internal::ComputeRadialReturn(
      trial, plastic_strain, kShearModulus, trial.equivalent_stress);

  EXPECT_FALSE(result.yielded);

  EXPECT_NEAR(result.plastic_multiplier, 0.0, Tol());

  const mjtNum expected_plastic[6] = {
      0, 0, 0, 0, 0, 0,
  };

  ExpectPlasticStrainNear(result.plastic_strain, expected_plastic);
}

// =============================================================================
// Plastic uniaxial return.
// =============================================================================

// For:
//
//   E = diag(0.01, 0, 0)
//
// we have:
//
//   sigma_eq_trial = 2 G e
//                  = 800
//
// With:
//
//   sigma_y = 250
//
// perfect J2 gives:
//
//   Delta lambda
//     = (800 - 250) / (3 * 40000)
//     = 0.004583333333...
//
// For this loading direction:
//
//   Delta E_p =
//       diag(
//           Delta lambda,
//          -Delta lambda / 2,
//          -Delta lambda / 2
//       )
//
TEST(J2RadialReturnTest, UniaxialPlasticReturnHasExpectedIncrement) {
  constexpr mjtNum e = 0.01;

  const std::array<mjtNum, 9> green_strain = {
      e, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  const mjtNum plastic_strain[6] = {
      0, 0, 0, 0, 0, 0,
  };

  const auto trial =
      internal::ComputeTrialState(green_strain, plastic_strain, kShearModulus);

  ASSERT_NEAR(trial.equivalent_stress, 800.0, Tol());

  const auto result = internal::ComputeRadialReturn(
      trial, plastic_strain, kShearModulus, kYieldStress);

  ASSERT_TRUE(result.yielded);

  const mjtNum expected_delta_lambda =
      (800.0 - kYieldStress) / (3.0 * kShearModulus);

  EXPECT_NEAR(result.plastic_multiplier, expected_delta_lambda, Tol());

  const mjtNum expected_plastic[6] = {
      expected_delta_lambda,
      mjtNum(-0.5) * expected_delta_lambda,
      mjtNum(-0.5) * expected_delta_lambda,
      0,
      0,
      0,
  };

  ExpectPlasticStrainNear(result.plastic_strain, expected_plastic);

  // Associative J2 plastic flow is incompressible.
  EXPECT_NEAR(result.plastic_strain[0] + result.plastic_strain[1] +
                  result.plastic_strain[2],
              0.0, Tol());
}

// =============================================================================
// Consistency check.
//
// This is the important non-tautological test.
//
// Instead of trusting result.equivalent_stress, recompute the complete trial
// state from the UPDATED plastic strain and verify that it actually lies on
// the yield surface.
// =============================================================================

TEST(J2RadialReturnTest, CorrectedPlasticStateReproducesYieldStress) {
  constexpr mjtNum e = 0.01;

  const std::array<mjtNum, 9> green_strain = {
      e, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  const mjtNum plastic_strain[6] = {
      0, 0, 0, 0, 0, 0,
  };

  const auto trial =
      internal::ComputeTrialState(green_strain, plastic_strain, kShearModulus);

  const auto result = internal::ComputeRadialReturn(
      trial, plastic_strain, kShearModulus, kYieldStress);

  ASSERT_TRUE(result.yielded);

  const auto corrected = internal::ComputeTrialState(
      green_strain, result.plastic_strain.data(), kShearModulus);

  // This value is independently recomputed from E - E_p.
  EXPECT_NEAR(corrected.equivalent_stress, kYieldStress, Tol());

  // The independently recomputed deviatoric stress must agree with the
  // radial-return stress.
  ExpectTensorNear(corrected.deviatoric_stress, result.deviatoric_stress);
}

// =============================================================================
// Pure shear plastic return.
//
// This specifically exercises the off-diagonal plastic-state convention:
//
//   [xx, yy, zz, xy, xz, yz]
// =============================================================================

TEST(J2RadialReturnTest, PureShearPlasticReturnIsConsistent) {
  constexpr mjtNum gamma = 0.02;
  constexpr mjtNum tensor_shear = gamma / 2.0;

  const std::array<mjtNum, 9> green_strain = {
      0, tensor_shear, 0, tensor_shear, 0, 0, 0, 0, 0,
  };

  const mjtNum plastic_strain[6] = {
      0, 0, 0, 0, 0, 0,
  };

  const auto trial =
      internal::ComputeTrialState(green_strain, plastic_strain, kShearModulus);

  const mjtNum expected_trial_stress = std::sqrt(3.0) * kShearModulus * gamma;

  ASSERT_NEAR(trial.equivalent_stress, expected_trial_stress, Tol());

  ASSERT_GT(trial.equivalent_stress, kYieldStress);

  const auto result = internal::ComputeRadialReturn(
      trial, plastic_strain, kShearModulus, kYieldStress);

  ASSERT_TRUE(result.yielded);

  // Pure xy shear must create only an xy plastic component.
  EXPECT_NEAR(result.plastic_strain[0], 0.0, Tol());

  EXPECT_NEAR(result.plastic_strain[1], 0.0, Tol());

  EXPECT_NEAR(result.plastic_strain[2], 0.0, Tol());

  EXPECT_NEAR(result.plastic_strain[4], 0.0, Tol());

  EXPECT_NEAR(result.plastic_strain[5], 0.0, Tol());

  EXPECT_GT(std::abs(result.plastic_strain[3]), 0.0);

  // Plastic flow remains traceless.
  EXPECT_NEAR(result.plastic_strain[0] + result.plastic_strain[1] +
                  result.plastic_strain[2],
              0.0, Tol());

  // Recompute from the returned plastic state.
  const auto corrected = internal::ComputeTrialState(
      green_strain, result.plastic_strain.data(), kShearModulus);

  EXPECT_NEAR(corrected.equivalent_stress, kYieldStress, Tol());

  ExpectTensorNear(corrected.deviatoric_stress, result.deviatoric_stress);
}

// =============================================================================
// Loading followed by elastic unloading.
//
// First load to:
//
//   e = 0.01
//
// which creates plastic strain.
//
// Then reduce the total strain to:
//
//   e = 0.009
//
// The unloading increment is:
//
//   Delta e = -0.001
//
// corresponding to an 80-unit reduction in equivalent stress:
//
//   2 G * 0.001 = 80
//
// so:
//
//   sigma_eq = 250 - 80 = 170
//
// which lies inside the yield surface.
//
// Therefore the historical plastic strain must remain unchanged.
// =============================================================================

TEST(J2RadialReturnTest, UnloadingDoesNotAccumulatePlasticStrain) {
  constexpr mjtNum loaded_strain = 0.01;

  const std::array<mjtNum, 9> loaded_green_strain = {
      loaded_strain, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  const mjtNum initial_plastic_strain[6] = {
      0, 0, 0, 0, 0, 0,
  };

  const auto loading_trial = internal::ComputeTrialState(
      loaded_green_strain, initial_plastic_strain, kShearModulus);

  const auto loading_result = internal::ComputeRadialReturn(
      loading_trial, initial_plastic_strain, kShearModulus, kYieldStress);

  ASSERT_TRUE(loading_result.yielded);

  // Unload slightly.
  constexpr mjtNum unloaded_strain = 0.009;

  const std::array<mjtNum, 9> unloaded_green_strain = {
      unloaded_strain, 0, 0, 0, 0, 0, 0, 0, 0,
  };

  const auto unloading_trial = internal::ComputeTrialState(
      unloaded_green_strain, loading_result.plastic_strain.data(),
      kShearModulus);

  ASSERT_NEAR(unloading_trial.equivalent_stress, 170.0, Tol());

  ASSERT_LT(unloading_trial.equivalent_stress, kYieldStress);

  const auto unloading_result = internal::ComputeRadialReturn(
      unloading_trial, loading_result.plastic_strain.data(), kShearModulus,
      kYieldStress);

  EXPECT_FALSE(unloading_result.yielded);

  EXPECT_NEAR(unloading_result.plastic_multiplier, 0.0, Tol());

  // Most important check:
  // unloading must not alter the historical plastic strain.
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(unloading_result.plastic_strain[i],
                loading_result.plastic_strain[i], Tol());
  }

  EXPECT_NEAR(unloading_result.equivalent_stress, 170.0, Tol());
}

}  // namespace
}  // namespace mujoco::plugin::plasticity