// Copyright 2026 DeepMind Technologies Limited
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

#include "test/compare_model.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <type_traits>

#include <mujoco/mjmodel.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>

namespace mujoco {
namespace {

template <typename T>
auto Compare(T val1, T val2);

auto Compare(char val1, char val2) {
  return val1 != val2;
}

auto Compare(unsigned char val1, unsigned char val2) {
  return val1 != val2;
}

auto Compare(bool val1, bool val2) {
  return val1 != val2;
}

auto Compare(mjtSize val1, mjtSize val2) {
  return val1 > val2 ? val1 - val2 : val2 - val1;
}

// The maximum spacing between a normalised floating point number x and an
// adjacent normalised number is 2 epsilon |x|; a factor 10 is added accounting
// for losses during non-idempotent operations such as vector normalizations.
template <typename T>
auto Compare(T val1, T val2) {
  using ReturnType =
      std::conditional_t<std::is_same_v<T, float>, float, double>;
  ReturnType error;
  if (std::abs(val1) <= 1 || std::abs(val2) <= 1) {
      // Absolute precision for small numbers
      error = std::abs(val1-val2);
  } else {
    // Relative precision for larger numbers
    ReturnType magnitude = std::abs(val1) + std::abs(val2);
    error = std::abs(val1/magnitude - val2/magnitude) / magnitude;
  }
  ReturnType safety_factor = 200;
  return error < safety_factor * std::numeric_limits<ReturnType>::epsilon()
             ? 0
             : error;
}

}  // namespace

mjtNum CompareModel(const mjModel* m1, const mjModel* m2,
                    std::string& field) {
  mjtNum dif, maxdif = 0.0;

  // define symbols corresponding to number of columns
  // (needed in MJMODEL_POINTERS)
  MJMODEL_POINTERS_PREAMBLE(m1);

// compare ints, exclude nbuffer because it hides the actual difference
#define X(name)                                           \
  if constexpr (std::string_view(#name) != "nbuffer") {   \
    if (m1->name != m2->name) {                           \
      maxdif = std::abs((long)m1->name - (long)m2->name); \
      field = #name;                                      \
    }                                                     \
  }
  MJMODEL_SIZES
#undef X
  if (maxdif > 0) return maxdif;

  // compare arrays, apart from bvh-related ones (which includes flex_vert0), as
  // those are sensitive to numerical differences when meshes are perfectly
  // symmetric.  Also skip flex fields derived from node local positions and
  // cell geometry that are not fully serialized to XML.
#define X(type, name, nr, nc)                                         \
  if (strncmp(#name, "bvh_", 4) &&                                    \
      strncmp(#name, "flex_vert", 9) &&                               \
      strncmp(#name, "mesh_poly", 9) &&                               \
      strcmp(#name, "flex_centered") &&                               \
      strcmp(#name, "flex_size") &&                                   \
      strcmp(#name, "flexedge_length0") &&                            \
      strcmp(#name, "flexedge_invweight0") &&                         \
      strncmp(#name, "flex_node", 9)) {                               \
    for (int r = 0; r < m1->nr; r++) {                                \
      for (int c = 0; c < nc; c++) {                                  \
        dif = Compare(m1->name[r * nc + c], m2->name[r * nc + c]);    \
        if (dif > maxdif) {                                           \
          maxdif = dif;                                               \
          field = #name;                                              \
          field += " row: " + std::to_string(r);                      \
          field += " col: " + std::to_string(c);                      \
        }                                                             \
      }                                                               \
    }                                                                 \
  }  // NOLINT
  MJMODEL_POINTERS
#undef X

  // compare fields in mjOption
  #define X(type, name, n)                                         \
    dif = Compare(m1->opt.name, m2->opt.name);                     \
    if (dif > maxdif) {maxdif = dif; field = #name;}
  #define XVEC(type, name, n)                                    \
    for (int c=0; c < n; c++) {                                  \
      dif = Compare(m1->opt.name[c], m2->opt.name[c]);           \
      if (dif > maxdif) {maxdif = dif; field = #name;} }
    MJOPTION_FIELDS
  #undef XVEC
  #undef X

  // Return largest difference and field name
  return maxdif;
}

}  // namespace mujoco
