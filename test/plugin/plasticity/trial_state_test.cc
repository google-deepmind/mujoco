#include "plugin/plasticity/j2.h"

#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "test/fixture.h"

namespace mujoco::plugin::plasticity {
namespace {

constexpr mjtNum kShearModulus = 40000.0;
constexpr mjtNum kDoubleTolerance = 1e-12;
constexpr mjtNum kFloatTolerance = 1e-3;

mjtNum Tol() { return MjTol(kDoubleTolerance, kFloatTolerance); }

TEST(TrialStateTest, HydrostaticStrainDoesNotYield) {
  const std::array<mjtNum, 9> green_strain = {
      0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01,
  };

  const mjtNum plastic_strain[6] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  };

  const internal::TrialState trial =
      internal::ComputeTrialState(green_strain, plastic_strain, kShearModulus);

  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(trial.deviatoric_stress[i], 0.0, Tol());
  }

  EXPECT_NEAR(trial.equivalent_stress, 0.0, Tol());
}

TEST(TrialStateTest, UniaxialStrainHasExpectedVonMisesStress) {
  constexpr mjtNum kStrain = 0.01;

  const std::array<mjtNum, 9> green_strain = {
      kStrain, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  };

  const mjtNum plastic_strain[6] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  };

  const internal::TrialState trial =
      internal::ComputeTrialState(green_strain, plastic_strain, kShearModulus);

  // For E = diag(e, 0, 0),
  //
  // dev(E) = diag(2e/3, -e/3, -e/3),
  //
  // and s = 2 G dev(E).
  const mjtNum expected_sxx = (4.0 / 3.0) * kShearModulus * kStrain;
  const mjtNum expected_syy = -(2.0 / 3.0) * kShearModulus * kStrain;

  // The von Mises equivalent stress for this state is 2 G e.
  const mjtNum expected_sigma_eq = 2.0 * kShearModulus * kStrain;

  EXPECT_NEAR(trial.deviatoric_stress[0], expected_sxx, Tol());
  EXPECT_NEAR(trial.deviatoric_stress[4], expected_syy, Tol());
  EXPECT_NEAR(trial.deviatoric_stress[8], expected_syy, Tol());

  EXPECT_NEAR(trial.deviatoric_stress[1], 0.0, Tol());
  EXPECT_NEAR(trial.deviatoric_stress[2], 0.0, Tol());
  EXPECT_NEAR(trial.deviatoric_stress[3], 0.0, Tol());
  EXPECT_NEAR(trial.deviatoric_stress[5], 0.0, Tol());
  EXPECT_NEAR(trial.deviatoric_stress[6], 0.0, Tol());
  EXPECT_NEAR(trial.deviatoric_stress[7], 0.0, Tol());

  EXPECT_NEAR(trial.equivalent_stress, expected_sigma_eq, Tol());
}

TEST(TrialStateTest, PureShearHasExpectedVonMisesStress) {
  constexpr mjtNum kShearStrain = 0.01;

  const std::array<mjtNum, 9> green_strain = {
      0.0, kShearStrain, 0.0, kShearStrain, 0.0, 0.0, 0.0, 0.0, 0.0,
  };

  const mjtNum plastic_strain[6] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  };

  const internal::TrialState trial =
      internal::ComputeTrialState(green_strain, plastic_strain, kShearModulus);

  const mjtNum expected_shear_stress = 2.0 * kShearModulus * kShearStrain;

  // With s_xy = s_yx = tau,
  //
  // sigma_eq = sqrt(3 / 2 * 2 * tau^2) = sqrt(3) * tau.
  const mjtNum expected_sigma_eq = std::sqrt(3.0) * expected_shear_stress;

  EXPECT_NEAR(trial.deviatoric_stress[1], expected_shear_stress, Tol());
  EXPECT_NEAR(trial.deviatoric_stress[3], expected_shear_stress, Tol());

  EXPECT_NEAR(trial.deviatoric_stress[0], 0.0, Tol());
  EXPECT_NEAR(trial.deviatoric_stress[4], 0.0, Tol());
  EXPECT_NEAR(trial.deviatoric_stress[8], 0.0, Tol());

  EXPECT_NEAR(trial.equivalent_stress, expected_sigma_eq, Tol());
}

TEST(TrialStateTest, PlasticStrainIsSubtractedFromTotalStrain) {
  const std::array<mjtNum, 9> green_strain = {
      0.01, 0.002, 0.003, 0.002, -0.004, 0.005, 0.003, 0.005, -0.006,
  };

  const mjtNum plastic_strain[6] = {
      0.01, -0.004, -0.006, 0.002, 0.003, 0.005,
  };

  const internal::TrialState trial =
      internal::ComputeTrialState(green_strain, plastic_strain, kShearModulus);

  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(trial.elastic_strain[i], 0.0, Tol());
    EXPECT_NEAR(trial.deviatoric_stress[i], 0.0, Tol());
  }

  EXPECT_NEAR(trial.equivalent_stress, 0.0, Tol());
}

}  // namespace
}  // namespace mujoco::plugin::plasticity