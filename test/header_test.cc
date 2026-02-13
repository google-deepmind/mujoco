// Copyright 2021 DeepMind Technologies Limited
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

// Tests for structures in the public headers.

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

template <typename T, size_t N>
struct ArrayOrScalar {
  using type = T[N];
};
template <typename T>
struct ArrayOrScalar<T, 1> {
  using type = T;
};
template <typename T, size_t N>
using ArrayOrScalarT = typename ArrayOrScalar<T, N>::type;

// check that a vector of named pointers are ordered by address
void CheckAddressOrdering(
    const std::vector<std::pair<const void*, const char*>>& pointers,
    const char* category_name) {
  for (size_t i = 0; i < pointers.size() - 1; ++i) {
    EXPECT_LT(reinterpret_cast<uintptr_t>(pointers[i].first),
              reinterpret_cast<uintptr_t>(pointers[i + 1].first))
        << category_name << " '" << pointers[i].second
        << "' should be declared before '" << pointers[i + 1].second << "'.";
  }
}

using HeaderTest = MujocoTest;

TEST_F(HeaderTest, IntsHave4Bytes) {
  EXPECT_EQ(4, sizeof(int));
}

TEST_F(HeaderTest, IntsHaveAtLeast31Bits) {
  int shift_left_30 = 1 << 30;
  EXPECT_GT(shift_left_30, 0);
}

TEST_F(HeaderTest, EnumsAreInts) {
  EXPECT_EQ(sizeof(mjtDisableBit),      sizeof(int));
  EXPECT_EQ(sizeof(mjtEnableBit),       sizeof(int));
  EXPECT_EQ(sizeof(mjtJoint),           sizeof(int));
  EXPECT_EQ(sizeof(mjtGeom),            sizeof(int));
  EXPECT_EQ(sizeof(mjtCamLight),        sizeof(int));
  EXPECT_EQ(sizeof(mjtTexture),         sizeof(int));
  EXPECT_EQ(sizeof(mjtIntegrator),      sizeof(int));
  EXPECT_EQ(sizeof(mjtCone),            sizeof(int));
  EXPECT_EQ(sizeof(mjtJacobian),        sizeof(int));
  EXPECT_EQ(sizeof(mjtSolver),          sizeof(int));
  EXPECT_EQ(sizeof(mjtEq),              sizeof(int));
  EXPECT_EQ(sizeof(mjtWrap),            sizeof(int));
  EXPECT_EQ(sizeof(mjtTrn),             sizeof(int));
  EXPECT_EQ(sizeof(mjtDyn),             sizeof(int));
  EXPECT_EQ(sizeof(mjtGain),            sizeof(int));
  EXPECT_EQ(sizeof(mjtBias),            sizeof(int));
  EXPECT_EQ(sizeof(mjtObj),             sizeof(int));
  EXPECT_EQ(sizeof(mjtConstraint),      sizeof(int));
  EXPECT_EQ(sizeof(mjtConstraintState), sizeof(int));
  EXPECT_EQ(sizeof(mjtSensor),          sizeof(int));
  EXPECT_EQ(sizeof(mjtStage),           sizeof(int));
  EXPECT_EQ(sizeof(mjtDataType),        sizeof(int));
  EXPECT_EQ(sizeof(mjtLRMode),          sizeof(int));
  EXPECT_EQ(sizeof(mjtWarning),         sizeof(int));
  EXPECT_EQ(sizeof(mjtTimer),           sizeof(int));
  EXPECT_EQ(sizeof(mjtGridPos),         sizeof(int));
  EXPECT_EQ(sizeof(mjtFramebuffer),     sizeof(int));
  EXPECT_EQ(sizeof(mjtFontScale),       sizeof(int));
  EXPECT_EQ(sizeof(mjtFont),            sizeof(int));
  EXPECT_EQ(sizeof(mjtButton),          sizeof(int));
  EXPECT_EQ(sizeof(mjtEvent),           sizeof(int));
  EXPECT_EQ(sizeof(mjtItem),            sizeof(int));
  EXPECT_EQ(sizeof(mjtCatBit),          sizeof(int));
  EXPECT_EQ(sizeof(mjtMouse),           sizeof(int));
  EXPECT_EQ(sizeof(mjtPertBit),         sizeof(int));
  EXPECT_EQ(sizeof(mjtCamera),          sizeof(int));
  EXPECT_EQ(sizeof(mjtLabel),           sizeof(int));
  EXPECT_EQ(sizeof(mjtFrame),           sizeof(int));
  EXPECT_EQ(sizeof(mjtVisFlag),         sizeof(int));
  EXPECT_EQ(sizeof(mjtRndFlag),         sizeof(int));
  EXPECT_EQ(sizeof(mjtStereo),          sizeof(int));
}

TEST_F(HeaderTest, MjOptionFields) {
  mjOption o;
  std::vector<std::pair<const void*, const char*>> fields;

  // check that all X macros have the correct type and dim
#define XIMPL(type, name, dim)                                             \
  static_assert(                                                           \
      std::is_same_v<decltype(mjOption::name), ArrayOrScalarT<type, dim>>, \
      "incorrect type for mjOption::" #name);
#define X(type, name, dim) \
  static_assert(dim == 1, "use XVEC for non-scalar fields"); \
  XIMPL(type, name, dim)
#define XVEC(type, name, dim) \
  static_assert(dim > 1, "use X for scalar fields"); \
  XIMPL(type, name, dim)

  MJOPTION_FIELDS

#undef XVEC
#undef X
#undef XIMPL

  // check that the ordering of X macros agrees with the struct fields
#define X(type, name, dim) \
  fields.push_back({static_cast<const void*>(&o.name), #name});
#define XVEC X
  MJOPTION_FIELDS
#undef XVEC
#undef X

  CheckAddressOrdering(fields, "MJOPTION_FIELDS");

  // check that MJOPTION_FIELDS is a complete list of struct fields
  struct ExpectedMjOption {
#define XVEC(type, name, dim) type name[dim];
#define X XVEC
    MJOPTION_FIELDS
#undef X
#undef XVEC
  };
  static_assert(sizeof(mjOption) == sizeof(ExpectedMjOption));
  static_assert(alignof(mjOption) == alignof(ExpectedMjOption));
}

TEST_F(HeaderTest, MjStatisticFields) {
  mjStatistic s;
  std::vector<std::pair<const void*, const char*>> fields;

  // All fields in mjStatistic are expected to be of type mjtNum.
  using ScalarType = mjtNum;

  // check that all X macros have the correct type and dim
#define XIMPL(name, dim)                                         \
  static_assert(std::is_same_v<decltype(mjStatistic::name),      \
                               ArrayOrScalarT<ScalarType, dim>>, \
                "incorrect type for mjStatistic::" #name);
#define X(name, dim)                                         \
  static_assert(dim == 1, "use XVEC for non-scalar fields"); \
  XIMPL(name, dim)
#define XVEC(name, dim)                                      \
  static_assert(dim > 1, "use X for scalar fields");         \
  XIMPL(name, dim)

  MJSTATISTIC_FIELDS

#undef XVEC
#undef X
#undef XIMPL

  // check that the ordering of X macros agrees with the struct fields
#define X(name, dim) \
  fields.push_back({static_cast<const void*>(&s.name), #name});
#define XVEC X
  MJSTATISTIC_FIELDS
#undef XVEC
#undef X

  CheckAddressOrdering(fields, "MJSTATISTIC_FIELDS");

  // check that MJSTATISTIC_FIELDS is a complete list of struct fields
  struct ExpectedMjStatistic {
#define XVEC(name, dim) ScalarType name[dim];
#define X XVEC
    MJSTATISTIC_FIELDS;
#undef XVEC
#undef X
  };
  static_assert(sizeof(mjStatistic) == sizeof(ExpectedMjStatistic));
  static_assert(alignof(mjStatistic) == alignof(ExpectedMjStatistic));
}

TEST_F(HeaderTest, MjVisualFields) {
  mjVisual v;
  std::vector<std::pair<const void*, const char*>> fields;

  // All member fields in quality, map, scale, and rgba have the same type.
  using QualityMemberType = int;
  using MapMemberType = float;
  using ScaleMemberType = float;
  using RgbaMemberType = float[4];

  // check that all X macros have the correct type and dim
#define X(type, name)                                      \
  static_assert(                                           \
      std::is_same_v<decltype(v.global.name), type>,       \
      "incorrect type for mjVisual::global::" #name);

  MJVISUAL_GLOBAL_FIELDS

#undef X

#define X(name)                                                     \
  static_assert(                                                    \
      std::is_same_v<decltype(v.quality.name), QualityMemberType>,  \
      "incorrect type for mjVisual::quality::" #name);

  MJVISUAL_QUALITY_FIELDS

#undef X

#define XIMPL(type, name, dim)                                               \
  static_assert(                                                             \
      std::is_same_v<decltype(v.headlight.name), ArrayOrScalarT<type, dim>>, \
      "incorrect type for mjVisual::headlight::" #name);
#define X(type, name, dim)                                   \
  static_assert(dim == 1, "use XVEC for non-scalar fields"); \
  XIMPL(type, name, dim)
#define XVEC(type, name, dim)                                                \
  static_assert(dim > 1, "use X for scalar fields");                      \
  XIMPL(type, name, dim)

  MJVISUAL_HEADLIGHT_FIELDS

#undef X
#undef XVEC

#define X(name)                                            \
  static_assert(                                           \
      std::is_same_v<decltype(v.map.name), MapMemberType>, \
      "incorrect type for mjVisual::map::" #name);

  MJVISUAL_MAP_FIELDS

#undef X

#define X(name)                                                \
  static_assert(                                               \
      std::is_same_v<decltype(v.scale.name), ScaleMemberType>, \
      "incorrect type for mjVisual::scale::" #name);

  MJVISUAL_SCALE_FIELDS

#undef X

#define X(name)                                              \
  static_assert(                                             \
      std::is_same_v<decltype(v.rgba.name), RgbaMemberType>, \
      "incorrect type for mjVisual::rgba::" #name);

  MJVISUAL_RGBA_FIELDS

#undef X

  // check that the ordering of X macros agrees with the struct fields
#define X(type, name) \
  fields.push_back({static_cast<const void*>(&v.global.name), #name});
  MJVISUAL_GLOBAL_FIELDS
#undef X
#define X(name) \
  fields.push_back({static_cast<const void*>(&v.quality.name), #name});
  MJVISUAL_QUALITY_FIELDS
#undef X
#define X(type, name, dim) \
  fields.push_back({static_cast<const void*>(&v.headlight.name), #name});
#define XVEC X
  MJVISUAL_HEADLIGHT_FIELDS
#undef XVEC
#undef X
#define X(name) \
  fields.push_back({static_cast<const void*>(&v.map.name), #name});
  MJVISUAL_MAP_FIELDS
#undef X
#define X(name) \
  fields.push_back({static_cast<const void*>(&v.scale.name), #name});
  MJVISUAL_SCALE_FIELDS
#undef X
#define X(name) \
  fields.push_back({static_cast<const void*>(&v.rgba.name), #name});
  MJVISUAL_RGBA_FIELDS
#undef X

  CheckAddressOrdering(fields, "MJVISUAL_FIELDS");

  // check that MJVISUAL_FIELDS is a complete list of fields
  struct ExpectedMjVisual {
    struct {
#define X(type, name) type name;
    MJVISUAL_GLOBAL_FIELDS;
#undef X
    } global;
    struct {
#define X(name) QualityMemberType name;
      MJVISUAL_QUALITY_FIELDS;
#undef X
    } quality;
    struct {
#define X(type, name, dim) type name[dim];
#define XVEC X
      MJVISUAL_HEADLIGHT_FIELDS;
#undef XVEC
#undef X
    } headlight;
    struct {
#define X(name) MapMemberType name;
      MJVISUAL_MAP_FIELDS;
#undef X
    } map;
    struct {
#define X(name) ScaleMemberType name;
      MJVISUAL_SCALE_FIELDS;
#undef X
    } scale;
    struct {
#define X(name) RgbaMemberType name;
      MJVISUAL_RGBA_FIELDS;
#undef X
    } rgba;
  };

  static_assert(sizeof(mjVisual) == sizeof(ExpectedMjVisual));
  static_assert(alignof(mjVisual) == alignof(ExpectedMjVisual));

  static_assert(sizeof(mjVisual::global) == sizeof(ExpectedMjVisual::global));
  static_assert(alignof(decltype(mjVisual::global)) ==
                alignof(decltype(ExpectedMjVisual::global)));

  static_assert(sizeof(mjVisual::quality) == sizeof(ExpectedMjVisual::quality));
  static_assert(alignof(decltype(mjVisual::quality)) ==
                alignof(decltype(ExpectedMjVisual::quality)));

  static_assert(sizeof(mjVisual::headlight) ==
                sizeof(ExpectedMjVisual::headlight));
  static_assert(alignof(decltype(mjVisual::headlight)) ==
                alignof(decltype(ExpectedMjVisual::headlight)));

  static_assert(sizeof(mjVisual::map) == sizeof(ExpectedMjVisual::map));
  static_assert(alignof(decltype(mjVisual::map)) ==
                alignof(decltype(ExpectedMjVisual::map)));

  static_assert(sizeof(mjVisual::scale) == sizeof(ExpectedMjVisual::scale));
  static_assert(alignof(decltype(mjVisual::scale)) ==
                alignof(decltype(ExpectedMjVisual::scale)));

  static_assert(sizeof(mjVisual::rgba) == sizeof(ExpectedMjVisual::rgba));
  static_assert(alignof(decltype(mjVisual::rgba)) ==
                alignof(decltype(ExpectedMjVisual::rgba)));
}

TEST_F(HeaderTest, MjModelIntsOrdered) {
  mjModel m;
  std::vector<std::pair<const void*, const char*>> ints;

#define X(name) ints.push_back({static_cast<const void*>(&m.name), #name});
  MJMODEL_SIZES
#undef X

  CheckAddressOrdering(ints, "MJMODEL_INT");
}

TEST_F(HeaderTest, MjModelPointersOrdered) {
  mjModel m;
  std::vector<std::pair<const void*, const char*>> pointers;

#define X(type, name, dim1, dim2) \
  pointers.push_back({static_cast<const void*>(&m.name), #name});
#define XNV X
  MJMODEL_POINTERS
#undef XNV
#undef X

  CheckAddressOrdering(pointers, "MJMODEL_POINTER");
}


TEST_F(HeaderTest, MjDataPointersOrdered) {
  mjData d;
  std::vector<std::pair<const void*, const char*>> pointers;

#define X(type, name, dim1, dim2) \
  pointers.push_back({static_cast<const void*>(&d.name), #name});
#define XNV X
  MJDATA_POINTERS
#undef XNV
#undef X

  CheckAddressOrdering(pointers, "mjData pointer");
}

TEST_F(HeaderTest, MjDataArenaPointersSolverOrdered) {
  mjData d;
  std::vector<std::pair<const void*, const char*>> pointers;

#define X(type, name, dim1, dim2) \
  pointers.push_back({static_cast<const void*>(&d.name), #name});
#define XNV X
#undef MJ_D
#define MJ_D(n) 0
  MJDATA_ARENA_POINTERS_SOLVER
#undef MJ_D
#define MJ_D(n) n
#undef XNV
#undef X

  CheckAddressOrdering(pointers, "MJDATA_ARENA_POINTERS_SOLVER");
}

TEST_F(HeaderTest, MjDataArenaPointersDualOrdered) {
  mjData d;
  std::vector<std::pair<const void*, const char*>> pointers;

#define X(type, name, dim1, dim2) \
  pointers.push_back({static_cast<const void*>(&d.name), #name});
#define XNV X
#undef MJ_D
#define MJ_D(n) 0
  MJDATA_ARENA_POINTERS_DUAL
#undef MJ_D
#define MJ_D(n) n
#undef XNV
#undef X

  CheckAddressOrdering(pointers, "MJDATA_ARENA_POINTERS_DUAL");
}

TEST_F(HeaderTest, MjDataArenaPointersIslandOrdered) {
  mjData d;
  std::vector<std::pair<const void*, const char*>> pointers;

#define X(type, name, dim1, dim2) \
  pointers.push_back({static_cast<const void*>(&d.name), #name});
#define XNV X
#undef MJ_D
#define MJ_D(n) 0
  MJDATA_ARENA_POINTERS_ISLAND
#undef MJ_D
#define MJ_D(n) n
#undef XNV
#undef X

  CheckAddressOrdering(pointers, "MJDATA_ARENA_POINTERS_ISLAND");
}

TEST_F(HeaderTest, MjDataScalarsOrdered) {
  mjData d;
  std::vector<std::pair<const void*, const char*>> scalars;

#define X(type, name) \
  scalars.push_back({static_cast<const void*>(&d.name), #name});
  MJDATA_SCALAR
#undef X
  CheckAddressOrdering(scalars, "mjData scalar");
}

TEST_F(HeaderTest, MjDataVectorsOrdered) {
  mjData d;
  std::vector<std::pair<const void*, const char*>> vectors;

#define X(type, name, dim1, dim2) \
  vectors.push_back({static_cast<const void*>(&d.name), #name});
  MJDATA_VECTOR
#undef X
  CheckAddressOrdering(vectors, "mjData vector");
}

}  // namespace
}  // namespace mujoco
