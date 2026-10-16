// Copyright 2026 The MuJoCo Metal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <metal_stdlib>
using namespace metal;

// Inspect the exponent bits because shader compilation may enable fast-math.
inline bool finite_float(float value) {
  return (as_type<uint>(value) & 0x7f800000u) != 0x7f800000u;
}

kernel void dense_spd_solve(
    device const float* mass [[buffer(0)]],
    device const float* rhs [[buffer(1)]],
    device float* factor [[buffer(2)]],
    device float* solution [[buffer(3)]],
    device int* status [[buffer(4)]],
    constant int* dims [[buffer(5)]],
    uint world [[thread_position_in_grid]]) {
  uint nv = uint(dims[0]);
  uint batch = uint(dims[1]);
  uint nrhs = uint(dims[2]);
  if (world >= batch) return;

  uint mass_base = world * nv * nv;
  uint rhs_base = world * nv * nrhs;
  uint failed = 0;

  // Zero first so every validation or factorization failure is safe to consume.
  for (uint i = 0; i < nv * nrhs; ++i) solution[rhs_base + i] = 0.0f;

  // Validate both complete inputs before starting factorization.
  float scale = 0.0f;
  for (uint i = 0; i < nv * nv; ++i) {
    float value = mass[mass_base + i];
    if (!finite_float(value)) failed = 1;
    else scale = max(scale, abs(value));
  }
  for (uint i = 0; i < nv * nrhs; ++i) {
    if (!finite_float(rhs[rhs_base + i])) failed = 1;
  }
  if (failed != 0) {
    status[world] = 1;
    return;
  }

  // Accept only roundoff-sized asymmetry, then factor the symmetric mean.
  float symmetry_tolerance = scale * 0.000003814697265625f;
  for (uint row = 0; row < nv; ++row) {
    for (uint col = row + 1; col < nv; ++col) {
      float upper = mass[mass_base + row * nv + col];
      float lower = mass[mass_base + col * nv + row];
      if (abs(upper - lower) > symmetry_tolerance) {
        failed = 2;
      }
    }
  }
  if (failed != 0) {
    status[world] = int(failed);
    return;
  }

  for (uint row = 0; row < nv; ++row) {
    for (uint col = 0; col < nv; ++col) {
      float value = mass[mass_base + row * nv + col];
      if (row != col) {
        value = 0.5f * value + 0.5f * mass[mass_base + col * nv + row];
      }
      factor[mass_base + row * nv + col] = value;
    }
  }

  // In-place lower-triangular Cholesky. A non-positive pivot is reported;
  // no diagonal shift, damping, or retry is applied.
  for (uint pivot = 0; pivot < nv; ++pivot) {
    float diagonal = factor[mass_base + pivot * nv + pivot];
    for (uint k = 0; k < pivot; ++k) {
      float value = factor[mass_base + pivot * nv + k];
      diagonal -= value * value;
    }
    if (!finite_float(diagonal)) {
      failed = 4;
      break;
    }
    if (!(diagonal > 0.0f)) {
      failed = 3;
      break;
    }
    float root = sqrt(diagonal);
    if (!finite_float(root) || !(root > 0.0f)) {
      failed = 4;
      break;
    }
    factor[mass_base + pivot * nv + pivot] = root;

    for (uint row = pivot + 1; row < nv; ++row) {
      float value = factor[mass_base + row * nv + pivot];
      for (uint k = 0; k < pivot; ++k) {
        value -= factor[mass_base + row * nv + k] *
                 factor[mass_base + pivot * nv + k];
      }
      value /= root;
      if (!finite_float(value)) {
        failed = 4;
        break;
      }
      factor[mass_base + row * nv + pivot] = value;
    }
    if (failed != 0) break;
  }

  // Forward and backward substitution for each RHS column.
  if (failed == 0) {
    for (uint col = 0; col < nrhs; ++col) {
      for (uint row = 0; row < nv; ++row) {
        float value = rhs[rhs_base + row * nrhs + col];
        for (uint k = 0; k < row; ++k) {
          value -= factor[mass_base + row * nv + k] *
                   solution[rhs_base + k * nrhs + col];
        }
        value /= factor[mass_base + row * nv + row];
        if (!finite_float(value)) {
          failed = 4;
          break;
        }
        solution[rhs_base + row * nrhs + col] = value;
      }
      if (failed != 0) break;

      for (int row = int(nv) - 1; row >= 0; --row) {
        float value = solution[rhs_base + uint(row) * nrhs + col];
        for (uint k = uint(row) + 1; k < nv; ++k) {
          value -= factor[mass_base + k * nv + uint(row)] *
                   solution[rhs_base + k * nrhs + col];
        }
        value /= factor[mass_base + uint(row) * nv + uint(row)];
        if (!finite_float(value)) {
          failed = 4;
          break;
        }
        solution[rhs_base + uint(row) * nrhs + col] = value;
      }
      if (failed != 0) break;
    }
  }

  if (failed != 0) {
    for (uint i = 0; i < nv * nrhs; ++i) solution[rhs_base + i] = 0.0f;
  }
  status[world] = int(failed);
}
