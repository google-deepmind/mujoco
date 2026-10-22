#include "j2.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>
#include <utility>

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::plasticity {
namespace {

// Plastic strain state stored per tetrahedron:
//
//   [ep_xx, ep_yy, ep_zz, ep_xy, ep_xz, ep_yz]
//
constexpr int kPlasticStateSizePerElement = 6;

// MuJoCo stores 21 independent coefficients for each symmetric 6x6
// tetrahedral edge stiffness matrix.
constexpr int kPackedStiffnessSize = 21;

// Tolerance for detecting degenerate reference tetrahedra.
constexpr mjtNum kReferenceTolerance = 1e-12;


// Exact tetrahedral edge ordering used by MuJoCo:
//
//   0 : (0, 1)
//   1 : (1, 2)
//   2 : (2, 0)
//   3 : (2, 3)
//   4 : (0, 3)
//   5 : (1, 3)
//
constexpr int kTetEdges[internal::kNumTetEdges][2] = {
    {0, 1}, {1, 2}, {2, 0}, {2, 3}, {0, 3}, {1, 3},
};


// -----------------------------------------------------------------------------
// Small 3x3 matrix helpers.
// Matrices are stored row-major.
// -----------------------------------------------------------------------------

mjtNum Det3(const mjtNum a[9]) {
  return a[0] * (a[4] * a[8] - a[5] * a[7]) -
         a[1] * (a[3] * a[8] - a[5] * a[6]) +
         a[2] * (a[3] * a[7] - a[4] * a[6]);
}


bool Invert3(const mjtNum a[9], mjtNum inverse[9]) {
  const mjtNum det = Det3(a);

  if (std::abs(det) < kReferenceTolerance) {
    return false;
  }

  const mjtNum inv_det = 1.0 / det;

  inverse[0] = (a[4] * a[8] - a[5] * a[7]) * inv_det;
  inverse[1] = -(a[1] * a[8] - a[2] * a[7]) * inv_det;
  inverse[2] = (a[1] * a[5] - a[2] * a[4]) * inv_det;

  inverse[3] = -(a[3] * a[8] - a[5] * a[6]) * inv_det;
  inverse[4] = (a[0] * a[8] - a[2] * a[6]) * inv_det;
  inverse[5] = -(a[0] * a[5] - a[2] * a[3]) * inv_det;

  inverse[6] = (a[3] * a[7] - a[4] * a[6]) * inv_det;
  inverse[7] = -(a[0] * a[7] - a[1] * a[6]) * inv_det;
  inverse[8] = (a[0] * a[4] - a[1] * a[3]) * inv_det;

  return true;
}


void Mul3(mjtNum result[9], const mjtNum a[9], const mjtNum b[9]) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result[3 * row + col] = a[3 * row + 0] * b[0 + col] +
                              a[3 * row + 1] * b[3 + col] +
                              a[3 * row + 2] * b[6 + col];
    }
  }
}


// Compute A^T A.
void TransposeMulSelf3(mjtNum result[9], const mjtNum a[9]) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result[3 * row + col] =
          a[row] * a[col] + a[3 + row] * a[3 + col] + a[6 + row] * a[6 + col];
    }
  }
}


// -----------------------------------------------------------------------------
// Flex / plugin helpers.
// -----------------------------------------------------------------------------

int FindAssociatedFlex(const mjModel* m, int instance) {
  int found = -1;

  for (int f = 0; f < m->nflex; ++f) {
    const int vert_adr = m->flex_vertadr[f];

    const int vert_num = m->flex_vertnum[f];

    bool associated = false;

    for (int v = 0; v < vert_num; ++v) {
      const int body_id = m->flex_vertbodyid[vert_adr + v];

      if (body_id >= 0 && body_id < m->nbody &&
          m->body_plugin[body_id] == instance) {
        associated = true;
        break;
      }
    }

    if (!associated) {
      continue;
    }

    if (found != -1) {
      return -2;
    }

    found = f;
  }

  return found;
}


std::optional<mjtNum> ParseNumericAttribute(const mjModel* m, int instance,
                                            const char* name) {
  const char* raw = mj_getPluginConfig(m, instance, name);

  if (!raw) {
    return std::nullopt;
  }

  std::string value(raw);

  value.erase(std::remove_if(value.begin(), value.end(),
                             [](unsigned char c) { return std::isspace(c); }),
              value.end());

  if (value.empty()) {
    return std::nullopt;
  }

  char* end = nullptr;

  const double parsed = std::strtod(value.c_str(), &end);

  if (end != value.c_str() + value.size()) {
    return std::nullopt;
  }

  return static_cast<mjtNum>(parsed);
}


// At nstate() time the compiled flex/body association is not fully available
// yet. Reserve enough state for all flex elements in the model.
//
// The J2 instance itself later uses only:
//
//   6 * element_count_
//
// entries from its allocated block.
int PlasticStateSize(const mjModel* m, int instance) {
  (void)instance;

  return kPlasticStateSizePerElement * m->nflexelem;
}


// Recover the physical reference-space position of a flex vertex.
//
// flex_vert0 uses MuJoCo's normalized flex reference coordinates.
// Reference differences are rescaled by 2 * flex_size.
void ReferenceVertexPosition(const mjModel* m, int flex_id, int global_vertex,
                             mjtNum position[3]) {
  for (int axis = 0; axis < 3; ++axis) {
    position[axis] = 2.0 * m->flex_size[3 * flex_id + axis] *
                     m->flex_vert0[3 * global_vertex + axis];
  }
}

}  // namespace


// =============================================================================
// Internal mathematical helpers.
// =============================================================================

namespace internal {

ElementKinematics ComputeKinematics(const mjtNum ds[9],
                                    const mjtNum inv_dm[9]) {
  ElementKinematics result{};

  // F = Ds Dm^-1
  Mul3(result.deformation_gradient.data(), ds, inv_dm);

  // J = det(F)
  result.jacobian = Det3(result.deformation_gradient.data());

  // C = F^T F
  mjtNum right_cauchy_green[9];

  TransposeMulSelf3(right_cauchy_green, result.deformation_gradient.data());

  // E = 1/2 (C - I)
  for (int i = 0; i < 9; ++i) {
    result.green_strain[i] = 0.5 * right_cauchy_green[i];
  }

  result.green_strain[0] -= 0.5;
  result.green_strain[4] -= 0.5;
  result.green_strain[8] -= 0.5;

  return result;
}


// -----------------------------------------------------------------------------
// Tensor <-> native edge representation.
// -----------------------------------------------------------------------------

std::array<mjtNum, kNumTetEdges> TensorToEdgeCoordinates(
    const std::array<std::array<mjtNum, 3>, kNumTetEdges>& reference_edges,
    const mjtNum tensor[9]) {
  std::array<mjtNum, kNumTetEdges> q{};

  for (int edge = 0; edge < kNumTetEdges; ++edge) {
    const auto& a = reference_edges[edge];

    mjtNum tensor_times_a[3] = {};

    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        tensor_times_a[row] += tensor[3 * row + col] * a[col];
      }
    }

    mjtNum quadratic = 0;

    for (int axis = 0; axis < 3; ++axis) {
      quadratic += a[axis] * tensor_times_a[axis];
    }

    // q_i = 2 A_i^T D A_i
    q[edge] = 2.0 * quadratic;
  }

  return q;
}


mjtNum EdgeQuadraticForm(
    const std::array<mjtNum, kNumTetEdges>& q,
    const std::array<mjtNum, kNumTetEdges * kNumTetEdges>& stiffness) {
  mjtNum result = 0;

  for (int row = 0; row < kNumTetEdges; ++row) {
    for (int col = 0; col < kNumTetEdges; ++col) {
      result += q[row] * stiffness[kNumTetEdges * row + col] * q[col];
    }
  }

  return result;
}


// -----------------------------------------------------------------------------
// Native shear modulus extraction.
// -----------------------------------------------------------------------------

mjtNum ExtractShearModulus(const ElementReference& reference) {
  // Traceless probe:
  //
  //   D = diag(1, -1, 0)
  //
  // with:
  //
  //   D:D = 2
  //
  const mjtNum probe[9] = {
      1, 0, 0, 0, -1, 0, 0, 0, 0,
  };

  const auto q = TensorToEdgeCoordinates(reference.reference_edges, probe);

  const mjtNum q_k_q = EdgeQuadraticForm(q, reference.stiffness);

  constexpr mjtNum probe_norm_squared = 2.0;

  // G = q^T K q / (4 V D:D)
  return q_k_q / (4.0 * reference.volume * probe_norm_squared);
}


// -----------------------------------------------------------------------------
// J2 trial state.
// -----------------------------------------------------------------------------

TrialState ComputeTrialState(const std::array<mjtNum, 9>& green_strain,
                             const mjtNum plastic_strain[6],
                             mjtNum shear_modulus) {
  TrialState result{};

  // Expand:
  //
  // [xx, yy, zz, xy, xz, yz]
  //
  // into:
  //
  // [ xx  xy  xz ]
  // [ xy  yy  yz ]
  // [ xz  yz  zz ]
  //
  const mjtNum plastic_tensor[9] = {
      plastic_strain[0], plastic_strain[3], plastic_strain[4],

      plastic_strain[3], plastic_strain[1], plastic_strain[5],

      plastic_strain[4], plastic_strain[5], plastic_strain[2],
  };

  // E_trial^e = E - E^p
  for (int i = 0; i < 9; ++i) {
    result.elastic_strain[i] = green_strain[i] - plastic_tensor[i];
  }

  const mjtNum trace = result.elastic_strain[0] + result.elastic_strain[4] +
                       result.elastic_strain[8];

  const mjtNum mean = trace / 3.0;

  std::array<mjtNum, 9> deviatoric_strain = result.elastic_strain;

  deviatoric_strain[0] -= mean;
  deviatoric_strain[4] -= mean;
  deviatoric_strain[8] -= mean;

  // s_trial = 2 G dev(E_trial^e)
  for (int i = 0; i < 9; ++i) {
    result.deviatoric_stress[i] = 2.0 * shear_modulus * deviatoric_strain[i];
  }

  // sigma_eq = sqrt(3/2 s:s)
  mjtNum stress_norm_squared = 0;

  for (int i = 0; i < 9; ++i) {
    stress_norm_squared +=
        result.deviatoric_stress[i] * result.deviatoric_stress[i];
  }

  result.equivalent_stress = std::sqrt(1.5 * stress_norm_squared);

  return result;
}


// -----------------------------------------------------------------------------
// Perfect-plastic J2 radial return.
// -----------------------------------------------------------------------------

RadialReturnResult ComputeRadialReturn(const TrialState& trial,
                                       const mjtNum plastic_strain[6],
                                       mjtNum shear_modulus,
                                       mjtNum yield_stress) {
  RadialReturnResult result{};

  // Start from committed plastic history.
  for (int i = 0; i < 6; ++i) {
    result.plastic_strain[i] = plastic_strain[i];
  }

  result.deviatoric_stress = trial.deviatoric_stress;

  result.equivalent_stress = trial.equivalent_stress;

  result.plastic_multiplier = 0.0;

  result.yielded = false;

  const mjtNum yield_function = trial.equivalent_stress - yield_stress;

  // Elastic step.
  if (yield_function <= 0.0) {
    return result;
  }

  // Defensive guards for direct helper use.
  if (trial.equivalent_stress <= kReferenceTolerance || shear_modulus <= 0.0) {
    return result;
  }

  result.yielded = true;

  // Perfect J2:
  //
  // Delta lambda =
  //   (sigma_eq_trial - sigma_y) / (3 G)
  //
  result.plastic_multiplier = yield_function / (3.0 * shear_modulus);

  // Associated flow:
  //
  // Delta E^p =
  //   Delta lambda *
  //   (3/2) *
  //   s_trial / sigma_eq_trial
  //
  const mjtNum flow_scale =
      1.5 * result.plastic_multiplier / trial.equivalent_stress;

  std::array<mjtNum, 9> plastic_increment{};

  for (int i = 0; i < 9; ++i) {
    plastic_increment[i] = flow_scale * trial.deviatoric_stress[i];
  }

  // Store independent plastic components:
  //
  // [xx, yy, zz, xy, xz, yz]
  //
  result.plastic_strain[0] += plastic_increment[0];

  result.plastic_strain[1] += plastic_increment[4];

  result.plastic_strain[2] += plastic_increment[8];

  result.plastic_strain[3] += plastic_increment[1];

  result.plastic_strain[4] += plastic_increment[2];

  result.plastic_strain[5] += plastic_increment[5];

  // Radial correction:
  //
  // s_{n+1} =
  //   (sigma_y / sigma_eq_trial) s_trial
  //
  const mjtNum radial_scale = yield_stress / trial.equivalent_stress;

  for (int i = 0; i < 9; ++i) {
    result.deviatoric_stress[i] = radial_scale * trial.deviatoric_stress[i];
  }

  result.equivalent_stress = yield_stress;

  return result;
}


// -----------------------------------------------------------------------------
// Plastic force correction.
// -----------------------------------------------------------------------------

std::array<mjtNum, kNumTetVertices * kSpatialDimension>
ComputePlasticVertexForces(
    const ElementReference& reference,
    const std::array<mjtNum, kNumTetVertices * kSpatialDimension>&
        current_positions,
    const mjtNum plastic_strain[6]) {
  std::array<mjtNum, kNumTetVertices * kSpatialDimension> force{};

  // Expand the six plastic components into a full symmetric tensor:
  //
  // [ xx  xy  xz ]
  // [ xy  yy  yz ]
  // [ xz  yz  zz ]
  //
  const mjtNum plastic_tensor[9] = {
      plastic_strain[0], plastic_strain[3], plastic_strain[4],

      plastic_strain[3], plastic_strain[1], plastic_strain[5],

      plastic_strain[4], plastic_strain[5], plastic_strain[2],
  };

  // Convert E^p into MuJoCo's native squared-edge-length coordinates:
  //
  //   q_p,i = 2 A_i^T E^p A_i
  //
  const auto plastic_edge_coordinates =
      TensorToEdgeCoordinates(reference.reference_edges, plastic_tensor);

  // r_p = K q_p
  std::array<mjtNum, kNumTetEdges> edge_response{};

  for (int row = 0; row < kNumTetEdges; ++row) {
    for (int col = 0; col < kNumTetEdges; ++col) {
      edge_response[row] += reference.stiffness[kNumTetEdges * row + col] *
                            plastic_edge_coordinates[col];
    }
  }

  // Native MuJoCo elasticity applies:
  //
  //   f_native = - G_x^T K q
  //
  // We need:
  //
  //   -G_x^T K (q - q_p)
  //
  // therefore the plugin correction is:
  //
  //   f_correction = + G_x^T K q_p
  //
  // For an edge (a,b), MuJoCo's half-gradient of squared length is:
  //
  //   vertex a : x_a - x_b
  //   vertex b : x_b - x_a
  //
  for (int edge = 0; edge < kNumTetEdges; ++edge) {
    const int a = kTetEdges[edge][0];

    const int b = kTetEdges[edge][1];

    const mjtNum response = edge_response[edge];

    for (int axis = 0; axis < kSpatialDimension; ++axis) {
      const mjtNum xa = current_positions[kSpatialDimension * a + axis];

      const mjtNum xb = current_positions[kSpatialDimension * b + axis];

      const mjtNum gradient_a = xa - xb;

      const mjtNum gradient_b = xb - xa;

      force[kSpatialDimension * a + axis] += response * gradient_a;

      force[kSpatialDimension * b + axis] += response * gradient_b;
    }
  }

  return force;
}


// =============================================================================
// MuJoCo-backed tetrahedral helpers.
// =============================================================================

ElementReference BuildElementReference(const mjModel* m, int flex_id,
                                       int element) {
  const int vert_adr = m->flex_vertadr[flex_id];

  const int elem_data_adr = m->flex_elemdataadr[flex_id];

  const int base = elem_data_adr + 4 * element;

  int vertex[4];

  for (int i = 0; i < 4; ++i) {
    vertex[i] = vert_adr + m->flex_elem[base + i];
  }

  mjtNum X[4][3];

  for (int i = 0; i < 4; ++i) {
    ReferenceVertexPosition(m, flex_id, vertex[i], X[i]);
  }

  // Dm = [X1-X0 X2-X0 X3-X0]
  const mjtNum dm[9] = {
      X[1][0] - X[0][0], X[2][0] - X[0][0], X[3][0] - X[0][0],

      X[1][1] - X[0][1], X[2][1] - X[0][1], X[3][1] - X[0][1],

      X[1][2] - X[0][2], X[2][2] - X[0][2], X[3][2] - X[0][2],
  };

  const mjtNum det_dm = Det3(dm);

  ElementReference result{};

  if (!Invert3(dm, result.inv_dm.data())) {
    mju_error(
        "J2 plasticity encountered a degenerate "
        "reference tetrahedron");
  }

  result.volume = std::abs(det_dm) / 6.0;

  // Store six reference edge vectors.
  for (int edge = 0; edge < kNumTetEdges; ++edge) {
    const int a = kTetEdges[edge][0];

    const int b = kTetEdges[edge][1];

    for (int axis = 0; axis < kSpatialDimension; ++axis) {
      result.reference_edges[edge][axis] = X[b][axis] - X[a][axis];
    }
  }

  // Unpack this element's symmetric native 6x6 stiffness matrix.
  const int stiffness_adr = m->flex_stiffnessadr[flex_id];

  if (stiffness_adr < 0) {
    mju_error("J2 plasticity requires native flex elasticity");
  }

  const mjtNum* packed =
      m->flex_stiffness + stiffness_adr + kPackedStiffnessSize * element;

  int packed_index = 0;

  for (int row = 0; row < kNumTetEdges; ++row) {
    for (int col = row; col < kNumTetEdges; ++col) {
      const mjtNum value = packed[packed_index++];

      result.stiffness[kNumTetEdges * row + col] = value;

      result.stiffness[kNumTetEdges * col + row] = value;
    }
  }

  result.shear_modulus = ExtractShearModulus(result);

  if (!(result.shear_modulus > 0)) {
    mju_error(
        "J2 plasticity failed to recover a positive "
        "shear modulus from native flex stiffness");
  }

  return result;
}


ElementKinematics ComputeElementKinematics(const mjModel* m, const mjData* d,
                                           int flex_id, int element,
                                           const ElementReference& reference) {
  const int vert_adr = m->flex_vertadr[flex_id];

  const int elem_data_adr = m->flex_elemdataadr[flex_id];

  const int base = elem_data_adr + 4 * element;

  int vertex[4];

  for (int i = 0; i < 4; ++i) {
    vertex[i] = vert_adr + m->flex_elem[base + i];
  }

  const mjtNum* x0 = d->flexvert_xpos + 3 * vertex[0];

  const mjtNum* x1 = d->flexvert_xpos + 3 * vertex[1];

  const mjtNum* x2 = d->flexvert_xpos + 3 * vertex[2];

  const mjtNum* x3 = d->flexvert_xpos + 3 * vertex[3];

  const mjtNum ds[9] = {
      x1[0] - x0[0], x2[0] - x0[0], x3[0] - x0[0],

      x1[1] - x0[1], x2[1] - x0[1], x3[1] - x0[1],

      x1[2] - x0[2], x2[2] - x0[2], x3[2] - x0[2],
  };

  return ComputeKinematics(ds, reference.inv_dm.data());
}

}  // namespace internal


// =============================================================================
// Construction.
// =============================================================================

J2::J2(const mjModel* m, int flex_id, int element_count, mjtNum yield_stress)
    : flex_id_(flex_id),
      element_count_(element_count),
      yield_stress_(yield_stress),
      reference_(element_count),
      pending_plastic_strain_(element_count) {
  for (int element = 0; element < element_count_; ++element) {
    reference_[element] = internal::BuildElementReference(m, flex_id_, element);

    pending_plastic_strain_[element].fill(0.0);
  }
}


std::optional<J2> J2::Create(const mjModel* m, mjData* d, int instance) {
  (void)d;

  const int flex_id = FindAssociatedFlex(m, instance);

  if (flex_id == -1) {
    mju_warning(
        "J2 plasticity plugin must be associated "
        "with a flexcomp");

    return std::nullopt;
  }

  if (flex_id == -2) {
    mju_warning(
        "J2 plasticity V0 supports exactly one flex "
        "per plugin instance");

    return std::nullopt;
  }

  if (m->flex_dim[flex_id] != 3) {
    mju_warning(
        "J2 plasticity V0 supports only "
        "3D tetrahedral flexes");

    return std::nullopt;
  }

  if (m->flex_interp[flex_id] != 0) {
    mju_warning(
        "J2 plasticity V0 supports only "
        "non-interpolated flexes");

    return std::nullopt;
  }

  if (m->flex_elemnum[flex_id] <= 0) {
    mju_warning(
        "J2 plasticity requires at least "
        "one tetrahedral element");

    return std::nullopt;
  }

  if (m->flex_stiffnessadr[flex_id] < 0) {
    mju_warning("J2 plasticity requires native flex elasticity");

    return std::nullopt;
  }

  const auto yield_stress = ParseNumericAttribute(m, instance, "yield");

  if (!yield_stress.has_value()) {
    mju_warning(
        "J2 plasticity requires a numeric "
        "yield attribute");

    return std::nullopt;
  }

  if (*yield_stress <= 0) {
    mju_warning("J2 plasticity requires yield > 0");

    return std::nullopt;
  }

  return J2(m, flex_id, m->flex_elemnum[flex_id], *yield_stress);
}


// =============================================================================
// Element kinematics used by the plugin.
// =============================================================================

internal::ElementKinematics J2::ComputeElementKinematics(const mjModel* m,
                                                         const mjData* d,
                                                         int element) const {
  return internal::ComputeElementKinematics(m, d, flex_id_, element,
                                            reference_[element]);
}


// =============================================================================
// State callbacks.
// =============================================================================

void J2::Reset(mjtNum* plugin_state) {
  mju_zero(plugin_state, kPlasticStateSizePerElement * element_count_);

  for (auto& plastic_strain : pending_plastic_strain_) {
    plastic_strain.fill(0.0);
  }
}


void J2::Advance(const mjModel* m, mjData* d, int instance) {
  mjtNum* plugin_state = d->plugin_state + m->plugin_stateadr[instance];

  for (int element = 0; element < element_count_; ++element) {
    mjtNum* destination = plugin_state + kPlasticStateSizePerElement * element;

    std::copy(pending_plastic_strain_[element].begin(),
              pending_plastic_strain_[element].end(), destination);
  }
}


// =============================================================================
// Passive computation.
// =============================================================================

void J2::Compute(const mjModel* m, mjData* d, int instance) {
  const mjtNum* plugin_state = d->plugin_state + m->plugin_stateadr[instance];

  // Start every forward evaluation from the committed physical history.
  //
  // Compute() may be called multiple times before Advance(), so the
  // irreversible state stored in plugin_state must remain untouched here.
  for (int element = 0; element < element_count_; ++element) {
    const mjtNum* committed_plastic_strain =
        plugin_state + kPlasticStateSizePerElement * element;

    std::copy(committed_plastic_strain,
              committed_plastic_strain + kPlasticStateSizePerElement,
              pending_plastic_strain_[element].begin());
  }

  for (int element = 0; element < element_count_; ++element) {
    const internal::ElementKinematics kinematics =
        ComputeElementKinematics(m, d, element);

    if (kinematics.jacobian <= 0) {
      mju_warning(
          "J2 plasticity encountered an "
          "inverted tetrahedron");

      continue;
    }

    const mjtNum* committed_plastic_strain =
        plugin_state + kPlasticStateSizePerElement * element;

    // -------------------------------------------------------------------------
    // Constitutive update.
    // -------------------------------------------------------------------------

    const internal::TrialState trial = internal::ComputeTrialState(
        kinematics.green_strain, committed_plastic_strain,
        reference_[element].shear_modulus);

    const internal::RadialReturnResult return_mapping =
        internal::ComputeRadialReturn(trial, committed_plastic_strain,
                                      reference_[element].shear_modulus,
                                      yield_stress_);

    // Save the candidate history.
    //
    // It is intentionally NOT written to plugin_state until Advance().
    pending_plastic_strain_[element] = return_mapping.plastic_strain;

    // -------------------------------------------------------------------------
    // Gather the current positions of this tetrahedron.
    // -------------------------------------------------------------------------

    const int vert_adr = m->flex_vertadr[flex_id_];

    const int elem_data_adr = m->flex_elemdataadr[flex_id_];

    const int element_base = elem_data_adr + 4 * element;

    std::array<mjtNum, internal::kNumTetVertices * internal::kSpatialDimension>
        current_positions{};

    int global_vertex[internal::kNumTetVertices];

    for (int local_vertex = 0; local_vertex < internal::kNumTetVertices;
         ++local_vertex) {
      const int flex_vertex = m->flex_elem[element_base + local_vertex];

      global_vertex[local_vertex] = vert_adr + flex_vertex;

      const mjtNum* position =
          d->flexvert_xpos + 3 * global_vertex[local_vertex];

      for (int axis = 0; axis < internal::kSpatialDimension; ++axis) {
        current_positions[3 * local_vertex + axis] = position[axis];
      }
    }

    // -------------------------------------------------------------------------
    // Compute plastic correction in Cartesian vertex-force coordinates.
    //
    // IMPORTANT:
    //
    // use E_p[n+1] returned by the radial return, not E_p[n].
    //
    // The native elastic force for the current configuration has already been
    // evaluated using the full deformation q.  The correction below changes:
    //
    //   -G_x^T K q
    //
    // into:
    //
    //   -G_x^T K (q - q_p)
    //
    // during THIS forward evaluation.
    // -------------------------------------------------------------------------

    const auto plastic_force = internal::ComputePlasticVertexForces(
        reference_[element], current_positions,
        return_mapping.plastic_strain.data());

    // -------------------------------------------------------------------------
    // Map Cartesian vertex forces into generalized forces.
    //
    // This mirrors the force projection used by MuJoCo's native flex
    // elasticity, except that native elasticity writes to qfrc_spring while
    // this passive plugin runs later and therefore adds directly to
    // qfrc_passive.
    // -------------------------------------------------------------------------

    for (int local_vertex = 0; local_vertex < internal::kNumTetVertices;
         ++local_vertex) {
      const int vertex = global_vertex[local_vertex];

      const int body_id = m->flex_vertbodyid[vertex];

      const mjtNum* force = plastic_force.data() + 3 * local_vertex;

      const mjtNum* position = d->flexvert_xpos + 3 * vertex;

      if (m->body_simple[body_id] != 2) {
        // General path, including pinned vertices.
        //
        // mj_applyFT performs the Cartesian-force -> generalized-force
        // projection through the body's kinematic chain.
        mj_applyFT(m, d, force, nullptr, position, body_id, d->qfrc_passive);
      } else {
        // Fast path used by native flex elasticity for simple flex bodies.
        //
        // The Cartesian force is expressed in world coordinates, while the
        // translational DOFs are represented in the body frame.
        mjtNum local_force[3];

        mju_mulMatTVec3(local_force, d->xmat + 9 * body_id, force);

        const int dof_adr = m->body_dofadr[body_id];

        const int dof_num = m->body_dofnum[body_id];

        for (int axis = 0; axis < dof_num; ++axis) {
          d->qfrc_passive[dof_adr + axis] += local_force[axis];
        }
      }
    }
  }
}

// =============================================================================
// Plugin registration.
// =============================================================================

void J2::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.plasticity.j2";

  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  static const char* attributes[] = {
      "yield",
  };

  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);

  plugin.attributes = attributes;

  plugin.nstate = +[](const mjModel* m, int instance) {
    return PlasticStateSize(m, instance);
  };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto plasticity_or_null = J2::Create(m, d, instance);

    if (!plasticity_or_null.has_value()) {
      return -1;
    }

    d->plugin_data[instance] =
        reinterpret_cast<uintptr_t>(new J2(std::move(*plasticity_or_null)));

    return 0;
  };

  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<J2*>(d->plugin_data[instance]);

    d->plugin_data[instance] = 0;
  };

  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data,
                     int instance) {
    (void)m;
    (void)instance;

    auto* plasticity = reinterpret_cast<J2*>(plugin_data);

    plasticity->Reset(plugin_state);
  };

  plugin.compute =
      +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        (void)capability_bit;

        auto* plasticity = reinterpret_cast<J2*>(d->plugin_data[instance]);

        plasticity->Compute(m, d, instance);
      };

  plugin.advance = +[](const mjModel* m, mjData* d, int instance) {
    auto* plasticity = reinterpret_cast<J2*>(d->plugin_data[instance]);

    plasticity->Advance(m, d, instance);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::plasticity