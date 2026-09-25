#include "plugin/plasticity/j2.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "test/fixture.h"

#include <mujoco/mujoco.h>

namespace mujoco::plugin::plasticity {
namespace {

constexpr mjtNum kDoubleTolerance = 1e-10;
constexpr mjtNum kFloatTolerance = 1e-3;

mjtNum Tol() { return MjTol(kDoubleTolerance, kFloatTolerance); }

constexpr int kEdges[internal::kNumTetEdges][2] = {
    {0, 1}, {1, 2}, {2, 0}, {2, 3}, {0, 3}, {1, 3},
};

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
          young="100000"
          poisson="0.25"/>
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
// Current tetrahedron positions from flexvert_xpos.
// -----------------------------------------------------------------------------

std::array<mjtNum, 12> GetCurrentPositions(const mjModel* model,
                                           const mjData* data) {
  std::array<mjtNum, 12> result{};

  const int vert_adr = model->flex_vertadr[0];

  const int elem_adr = model->flex_elemdataadr[0];

  for (int local = 0; local < 4; ++local) {
    const int vertex = vert_adr + model->flex_elem[elem_adr + local];

    for (int axis = 0; axis < 3; ++axis) {
      result[3 * local + axis] = data->flexvert_xpos[3 * vertex + axis];
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
// Apply an affine deformation F directly to four positions.
// -----------------------------------------------------------------------------

std::array<mjtNum, 12> ApplyDeformation(
    const std::array<mjtNum, 12>& reference_positions, const mjtNum F[9]) {
  std::array<mjtNum, 12> result{};

  // Vertex 0 is the affine origin.
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
      result[3 * vertex + row] = origin[row] + F[3 * row + 0] * X[0] +
                                 F[3 * row + 1] * X[1] + F[3 * row + 2] * X[2];
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
// Green-Lagrange strain:
//
//   E = 1/2 (F^T F - I)
// -----------------------------------------------------------------------------

std::array<mjtNum, 9> GreenStrain(const mjtNum F[9]) {
  std::array<mjtNum, 9> result{};

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      mjtNum value = 0;

      for (int k = 0; k < 3; ++k) {
        value += F[3 * k + row] * F[3 * k + col];
      }

      result[3 * row + col] = 0.5 * (value - (row == col ? 1.0 : 0.0));
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
// Convert full symmetric tensor into plugin plastic-state convention:
//
//   [xx, yy, zz, xy, xz, yz]
// -----------------------------------------------------------------------------

std::array<mjtNum, 6> TensorToPlasticState(
    const std::array<mjtNum, 9>& tensor) {
  return {
      tensor[0], tensor[4], tensor[8], tensor[1], tensor[2], tensor[5],
  };
}

// -----------------------------------------------------------------------------
// Reproduce native tetrahedral elastic force using the already-unpacked
// ElementReference.
//
// q_i = l_i^2 - l_i0^2
//
// r = K q
//
// native force:
//   f = -G_x^T r
// -----------------------------------------------------------------------------

std::array<mjtNum, 12> ComputeNativeVertexForces(
    const internal::ElementReference& reference,
    const std::array<mjtNum, 12>& current_positions) {
  std::array<mjtNum, 6> q{};

  for (int edge = 0; edge < internal::kNumTetEdges; ++edge) {
    const int a = kEdges[edge][0];

    const int b = kEdges[edge][1];

    mjtNum current_length_squared = 0;
    mjtNum reference_length_squared = 0;

    for (int axis = 0; axis < 3; ++axis) {
      const mjtNum dx =
          current_positions[3 * b + axis] - current_positions[3 * a + axis];

      current_length_squared += dx * dx;

      const mjtNum dX = reference.reference_edges[edge][axis];

      reference_length_squared += dX * dX;
    }

    q[edge] = current_length_squared - reference_length_squared;
  }

  std::array<mjtNum, 6> response{};

  for (int row = 0; row < internal::kNumTetEdges; ++row) {
    for (int col = 0; col < internal::kNumTetEdges; ++col) {
      response[row] +=
          reference.stiffness[internal::kNumTetEdges * row + col] * q[col];
    }
  }

  std::array<mjtNum, 12> force{};

  for (int edge = 0; edge < internal::kNumTetEdges; ++edge) {
    const int a = kEdges[edge][0];

    const int b = kEdges[edge][1];

    for (int axis = 0; axis < 3; ++axis) {
      const mjtNum gradient_a =
          current_positions[3 * a + axis] - current_positions[3 * b + axis];

      const mjtNum gradient_b = -gradient_a;

      force[3 * a + axis] -= response[edge] * gradient_a;

      force[3 * b + axis] -= response[edge] * gradient_b;
    }
  }

  return force;
}

// =============================================================================
// Zero plastic strain -> zero correction.
// =============================================================================

TEST(J2PlasticForceTest, ZeroPlasticStrainProducesZeroForce) {
  auto test = MakeTestModel();

  ASSERT_NE(test.model, nullptr);
  ASSERT_NE(test.data, nullptr);

  const auto reference = internal::BuildElementReference(test.model, 0, 0);

  const auto positions = GetCurrentPositions(test.model, test.data);

  const mjtNum plastic_strain[6] = {
      0, 0, 0, 0, 0, 0,
  };

  const auto correction = internal::ComputePlasticVertexForces(
      reference, positions, plastic_strain);

  for (mjtNum value : correction) {
    EXPECT_NEAR(value, 0.0, Tol());
  }
}

// =============================================================================
// Internal plastic correction must have zero resultant force.
// =============================================================================

TEST(J2PlasticForceTest, PlasticCorrectionHasZeroNetForce) {
  auto test = MakeTestModel();

  ASSERT_NE(test.model, nullptr);
  ASSERT_NE(test.data, nullptr);

  const auto reference = internal::BuildElementReference(test.model, 0, 0);

  const auto positions = GetCurrentPositions(test.model, test.data);

  const mjtNum plastic_strain[6] = {
      0.010, -0.005, -0.005, 0.003, -0.002, 0.001,
  };

  const auto correction = internal::ComputePlasticVertexForces(
      reference, positions, plastic_strain);

  for (int axis = 0; axis < 3; ++axis) {
    mjtNum total = 0;

    for (int vertex = 0; vertex < 4; ++vertex) {
      total += correction[3 * vertex + axis];
    }

    EXPECT_NEAR(total, 0.0, Tol());
  }
}

// =============================================================================
// Strong consistency test:
//
//   E_p = E
//
// must imply:
//
//   q_p = q
//
// and therefore:
//
//   f_native + f_plastic_correction = 0.
//
// This simultaneously checks:
//   - edge ordering,
//   - q = 2 A^T E A,
//   - stiffness layout,
//   - factor 2,
//   - current edge gradients,
//   - correction sign.
// =============================================================================

TEST(J2PlasticForceTest, PlasticStrainEqualToTotalStrainCancelsElasticForce) {
  auto test = MakeTestModel();

  ASSERT_NE(test.model, nullptr);
  ASSERT_NE(test.data, nullptr);

  const auto reference = internal::BuildElementReference(test.model, 0, 0);

  const auto reference_positions = GetCurrentPositions(test.model, test.data);

  // Deliberately combine stretch and shear so this is not merely an
  // axis-aligned special case.
  const mjtNum F[9] = {
      1.01, 0.02, 0.00, 0.00, 0.99, 0.01, 0.00, 0.00, 1.00,
  };

  const auto current_positions = ApplyDeformation(reference_positions, F);

  const auto total_strain = GreenStrain(F);

  const auto plastic_state = TensorToPlasticState(total_strain);

  const auto native_force =
      ComputeNativeVertexForces(reference, current_positions);

  const auto plastic_correction = internal::ComputePlasticVertexForces(
      reference, current_positions, plastic_state.data());

  mjtNum max_native_force = 0;

  for (int i = 0; i < 12; ++i) {
    max_native_force = std::max(max_native_force, std::abs(native_force[i]));
  }

  // Ensure this is not a vacuous zero-force test.
  ASSERT_GT(max_native_force, 1.0);

  for (int i = 0; i < 12; ++i) {
    EXPECT_NEAR(native_force[i] + plastic_correction[i], 0.0, Tol());
  }
}

}  // namespace
}  // namespace mujoco::plugin::plasticity