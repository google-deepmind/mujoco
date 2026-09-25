#include "plugin/plasticity/j2.h"

#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "test/fixture.h"

#include <mujoco/mujoco.h>

namespace mujoco::plugin::plasticity {
namespace {

constexpr mjtNum kDoubleTolerance = 1e-17;
constexpr mjtNum kFloatTolerance = 1e-8;
constexpr int kPlasticStateSize = 6;

mjtNum Tol() { return MjTol(kDoubleTolerance, kFloatTolerance); }

// -----------------------------------------------------------------------------
// Register the plugin exactly once in this test executable.
//
// The test target compiles j2.cc directly, not register.cc, so the dynamic
// library initializer is not involved here.
// -----------------------------------------------------------------------------

void EnsurePluginRegistered() {
  static const bool registered = []() {
    J2::RegisterPlugin();
    return true;
  }();

  (void)registered;
}

// -----------------------------------------------------------------------------
// Minimal one-tetrahedron J2 model.
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
          poisson="0.25"/>

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

  // Populate flexvert_xpos and run the plugin once at rest.
  mj_forward(result.model, result.data);

  return result;
}

// -----------------------------------------------------------------------------
// Save the current rest positions of all flex vertices.
// -----------------------------------------------------------------------------

std::array<mjtNum, 12> SaveFlexPositions(const mjModel* model,
                                         const mjData* data) {
  std::array<mjtNum, 12> result{};

  EXPECT_EQ(model->flex_vertnum[0], 4);

  const int vert_adr = model->flex_vertadr[0];

  for (int vertex = 0; vertex < 4; ++vertex) {
    for (int axis = 0; axis < 3; ++axis) {
      result[3 * vertex + axis] =
          data->flexvert_xpos[3 * (vert_adr + vertex) + axis];
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
// Apply:
//
//   F = diag(stretch_x, 1, 1)
//
// directly to flexvert_xpos, using vertex 0 as the affine origin.
//
// This is intentionally a test-only manipulation. It lets us test the plugin
// constitutive/state logic without involving the flex DOF integration.
// -----------------------------------------------------------------------------

void ApplyUniaxialStretch(const mjModel* model, mjData* data,
                          const std::array<mjtNum, 12>& rest,
                          mjtNum stretch_x) {
  const int vert_adr = model->flex_vertadr[0];

  const mjtNum origin[3] = {
      rest[0],
      rest[1],
      rest[2],
  };

  for (int vertex = 0; vertex < 4; ++vertex) {
    const mjtNum reference_x = rest[3 * vertex + 0];

    const mjtNum reference_y = rest[3 * vertex + 1];

    const mjtNum reference_z = rest[3 * vertex + 2];

    mjtNum* current = data->flexvert_xpos + 3 * (vert_adr + vertex);

    current[0] = origin[0] + stretch_x * (reference_x - origin[0]);

    current[1] = reference_y;

    current[2] = reference_z;
  }
}

// -----------------------------------------------------------------------------
// Retrieve the actual J2 object created by MuJoCo's plugin init callback.
// -----------------------------------------------------------------------------

J2* GetPlugin(const mjModel* model, mjData* data) {
  EXPECT_EQ(model->nplugin, 1);

  if (model->nplugin != 1) {
    return nullptr;
  }

  return reinterpret_cast<J2*>(data->plugin_data[0]);
}

// -----------------------------------------------------------------------------
// Get this plugin instance's committed six-component state.
// -----------------------------------------------------------------------------

mjtNum* GetPluginState(const mjModel* model, mjData* data) {
  return data->plugin_state + model->plugin_stateadr[0];
}

// =============================================================================
// Test 1:
// The compiled model must allocate exactly six history variables for our one
// tetrahedron, and they must initially be zero.
// =============================================================================

TEST(J2PluginStateTest, StateLayoutIsSixValuesPerTet) {
  auto test = MakeTestModel();

  ASSERT_NE(test.model, nullptr);

  ASSERT_NE(test.data, nullptr);

  ASSERT_EQ(test.model->nflex, 1);

  ASSERT_EQ(test.model->flex_elemnum[0], 1);

  ASSERT_EQ(test.model->nplugin, 1);

  EXPECT_EQ(test.model->plugin_statenum[0], kPlasticStateSize);

  mjtNum* state = GetPluginState(test.model, test.data);

  for (int i = 0; i < kPlasticStateSize; ++i) {
    EXPECT_NEAR(state[i], 0.0, Tol());
  }

  EXPECT_NE(GetPlugin(test.model, test.data), nullptr);
}

// =============================================================================
// Test 2:
// Compute() may calculate a plastic candidate, but MUST NOT mutate the
// committed plugin_state.
// =============================================================================

TEST(J2PluginStateTest, ComputeDoesNotCommitPlasticState) {
  auto test = MakeTestModel();

  ASSERT_NE(test.model, nullptr);

  ASSERT_NE(test.data, nullptr);

  J2* plasticity = GetPlugin(test.model, test.data);

  ASSERT_NE(plasticity, nullptr);

  const auto rest = SaveFlexPositions(test.model, test.data);

  // 1% stretch gives a Green-Lagrange strain:
  //
  //   Exx = 1/2 (1.01^2 - 1)
  //       = 0.01005
  //
  // With G = 40000:
  //
  //   sigma_eq_trial = 2 G Exx
  //                  = 804
  //
  // well above sigma_y = 250.
  ApplyUniaxialStretch(test.model, test.data, rest, 1.01);

  mjtNum* state = GetPluginState(test.model, test.data);

  for (int i = 0; i < kPlasticStateSize; ++i) {
    ASSERT_NEAR(state[i], 0.0, Tol());
  }

  plasticity->Compute(test.model, test.data, 0);

  // Compute() must only populate pending_plastic_strain_.
  for (int i = 0; i < kPlasticStateSize; ++i) {
    EXPECT_NEAR(state[i], 0.0, Tol());
  }
}

// =============================================================================
// Test 3:
// Advance() commits exactly the candidate produced by Compute(), and a small
// elastic unloading step does not alter that historical plastic strain.
// =============================================================================

TEST(J2PluginStateTest, AdvanceCommitsAndUnloadingPreservesHistory) {
  auto test = MakeTestModel();

  ASSERT_NE(test.model, nullptr);

  ASSERT_NE(test.data, nullptr);

  J2* plasticity = GetPlugin(test.model, test.data);

  ASSERT_NE(plasticity, nullptr);

  const auto rest = SaveFlexPositions(test.model, test.data);

  mjtNum* state = GetPluginState(test.model, test.data);

  // ---------------------------------------------------------------------------
  // Plastic loading.
  // ---------------------------------------------------------------------------

  ApplyUniaxialStretch(test.model, test.data, rest, 1.01);

  // Independently calculate the state that SHOULD be committed.
  const auto reference = internal::BuildElementReference(test.model, 0, 0);

  const auto loaded_kinematics = internal::ComputeElementKinematics(
      test.model, test.data, 0, 0, reference);

  const mjtNum zero_plastic_strain[6] = {
      0, 0, 0, 0, 0, 0,
  };

  const auto loaded_trial =
      internal::ComputeTrialState(loaded_kinematics.green_strain,
                                  zero_plastic_strain, reference.shear_modulus);

  ASSERT_GT(loaded_trial.equivalent_stress, 250.0);

  const auto expected_loaded_state = internal::ComputeRadialReturn(
      loaded_trial, zero_plastic_strain, reference.shear_modulus, 250.0);

  ASSERT_TRUE(expected_loaded_state.yielded);

  // Plugin Compute() builds the candidate.
  plasticity->Compute(test.model, test.data, 0);

  // Still uncommitted.
  for (int i = 0; i < kPlasticStateSize; ++i) {
    ASSERT_NEAR(state[i], 0.0, Tol());
  }

  // Plugin Advance() performs the irreversible commit.
  plasticity->Advance(test.model, test.data, 0);

  for (int i = 0; i < kPlasticStateSize; ++i) {
    EXPECT_NEAR(state[i], expected_loaded_state.plastic_strain[i], Tol());
  }

  // J2 flow must remain volume preserving.
  EXPECT_NEAR(state[0] + state[1] + state[2], 0.0, Tol());

  // Save the committed historical state.
  std::array<mjtNum, 6> committed{};

  for (int i = 0; i < kPlasticStateSize; ++i) {
    committed[i] = state[i];
  }

  // ---------------------------------------------------------------------------
  // Small unloading step.
  //
  // 1.010 -> 1.009 is a sufficiently small reverse increment that the state
  // should move inside the yield surface without reverse yielding.
  // ---------------------------------------------------------------------------

  ApplyUniaxialStretch(test.model, test.data, rest, 1.009);

  const auto unloaded_kinematics = internal::ComputeElementKinematics(
      test.model, test.data, 0, 0, reference);

  const auto unloading_trial =
      internal::ComputeTrialState(unloaded_kinematics.green_strain,
                                  committed.data(), reference.shear_modulus);

  ASSERT_LT(unloading_trial.equivalent_stress, 250.0);

  // Build the next candidate.
  plasticity->Compute(test.model, test.data, 0);

  // Compute still must not modify the committed history.
  for (int i = 0; i < kPlasticStateSize; ++i) {
    EXPECT_NEAR(state[i], committed[i], Tol());
  }

  // Commit the elastic unloading candidate.
  plasticity->Advance(test.model, test.data, 0);

  // No new plastic strain may have accumulated.
  for (int i = 0; i < kPlasticStateSize; ++i) {
    EXPECT_NEAR(state[i], committed[i], Tol());
  }
}

}  // namespace
}  // namespace mujoco::plugin::plasticity