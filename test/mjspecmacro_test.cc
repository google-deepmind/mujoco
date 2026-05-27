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

// Tests for X macros in mjspecmacro.h.

#include <cstddef>
#include <type_traits>

#include <gtest/gtest.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjspecmacro.h>

namespace mujoco {
namespace {

namespace expected {
#define X(type, name, dim) type name;
#define XVEC(type, name, dim) type name[dim];

struct mjsElement { MJSELEMENT_FIELDS };
struct mjsCompiler { MJSCOMPILER_FIELDS };
struct mjSpec { MJSPEC_FIELDS };
struct mjsOrientation { MJSORIENTATION_FIELDS };
struct mjsPlugin { MJSPLUGIN_FIELDS };
struct mjsBody { MJSBODY_FIELDS };
struct mjsFrame { MJSFRAME_FIELDS };
struct mjsJoint { MJSJOINT_FIELDS };
struct mjsGeom { MJSGEOM_FIELDS };
struct mjsSite { MJSSITE_FIELDS };
struct mjsCamera { MJSCAMERA_FIELDS };
struct mjsLight { MJSLIGHT_FIELDS };
struct mjsFlex { MJSFLEX_FIELDS };
struct mjsMesh { MJSMESH_FIELDS };
struct mjsHField { MJSHFIELD_FIELDS };
struct mjsSkin { MJSSKIN_FIELDS };
struct mjsTexture { MJSTEXTURE_FIELDS };
struct mjsMaterial { MJSMATERIAL_FIELDS };
struct mjsPair { MJSPAIR_FIELDS };
struct mjsExclude { MJSEXCLUDE_FIELDS };
struct mjsEquality { MJSEQUALITY_FIELDS };
struct mjsTendon { MJSTENDON_FIELDS };
struct mjsWrap { MJSWRAP_FIELDS };
struct mjsActuator { MJSACTUATOR_FIELDS };
struct mjsSensor { MJSSENSOR_FIELDS };
struct mjsNumeric { MJSNUMERIC_FIELDS };
struct mjsText { MJSTEXT_FIELDS };
struct mjsTuple { MJSTUPLE_FIELDS };
struct mjsKey { MJSKEY_FIELDS };
struct mjsDefault { MJSDEFAULT_FIELDS };

#undef XVEC
#undef X
}  // namespace expected

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

#define XIMPL(type, name, dim)                                                 \
  static_assert(                                                               \
      std::is_same_v<decltype(RealStruct::name), ArrayOrScalarT<type, dim>>,   \
      "Type mismatch for " #name);                                             \
  static_assert(offsetof(RealStruct, name) == offsetof(ExpectedStruct, name),  \
                "Offset mismatch for " #name);

#define X(type, name, dim)                                                     \
  static_assert(dim == 1, "use XVEC for non-scalar fields");                   \
  XIMPL(type, name, dim)

#define XVEC(type, name, dim)                                                  \
  static_assert(dim > 1, "use X for scalar fields");                           \
  XIMPL(type, name, dim)

#define CHECK_STRUCT(Name, Macro)                                              \
  struct Check##Name {                                                         \
    using RealStruct = Name;                                                   \
    using ExpectedStruct = expected::Name;                                     \
    static_assert(sizeof(RealStruct) == sizeof(ExpectedStruct),                \
                  "sizeof mismatch for " #Name);                               \
    static_assert(alignof(RealStruct) == alignof(ExpectedStruct),              \
                  "alignof mismatch for " #Name);                              \
    Macro                                                                      \
  };

CHECK_STRUCT(mjsElement, MJSELEMENT_FIELDS)
CHECK_STRUCT(mjsCompiler, MJSCOMPILER_FIELDS)
CHECK_STRUCT(mjSpec, MJSPEC_FIELDS)
CHECK_STRUCT(mjsOrientation, MJSORIENTATION_FIELDS)
CHECK_STRUCT(mjsPlugin, MJSPLUGIN_FIELDS)
CHECK_STRUCT(mjsBody, MJSBODY_FIELDS)
CHECK_STRUCT(mjsFrame, MJSFRAME_FIELDS)
CHECK_STRUCT(mjsJoint, MJSJOINT_FIELDS)
CHECK_STRUCT(mjsGeom, MJSGEOM_FIELDS)
CHECK_STRUCT(mjsSite, MJSSITE_FIELDS)
CHECK_STRUCT(mjsCamera, MJSCAMERA_FIELDS)
CHECK_STRUCT(mjsLight, MJSLIGHT_FIELDS)
CHECK_STRUCT(mjsFlex, MJSFLEX_FIELDS)
CHECK_STRUCT(mjsMesh, MJSMESH_FIELDS)
CHECK_STRUCT(mjsHField, MJSHFIELD_FIELDS)
CHECK_STRUCT(mjsSkin, MJSSKIN_FIELDS)
CHECK_STRUCT(mjsTexture, MJSTEXTURE_FIELDS)
CHECK_STRUCT(mjsMaterial, MJSMATERIAL_FIELDS)
CHECK_STRUCT(mjsPair, MJSPAIR_FIELDS)
CHECK_STRUCT(mjsExclude, MJSEXCLUDE_FIELDS)
CHECK_STRUCT(mjsEquality, MJSEQUALITY_FIELDS)
CHECK_STRUCT(mjsTendon, MJSTENDON_FIELDS)
CHECK_STRUCT(mjsWrap, MJSWRAP_FIELDS)
CHECK_STRUCT(mjsActuator, MJSACTUATOR_FIELDS)
CHECK_STRUCT(mjsSensor, MJSSENSOR_FIELDS)
CHECK_STRUCT(mjsNumeric, MJSNUMERIC_FIELDS)
CHECK_STRUCT(mjsText, MJSTEXT_FIELDS)
CHECK_STRUCT(mjsTuple, MJSTUPLE_FIELDS)
CHECK_STRUCT(mjsKey, MJSKEY_FIELDS)
CHECK_STRUCT(mjsDefault, MJSDEFAULT_FIELDS)

#undef CHECK_STRUCT
#undef XVEC
#undef X
#undef XIMPL

TEST(MjspecmacroTest, CompileTimeChecks) {
  // Verifies that the test binary compiled successfully and all static_asserts passed.
  EXPECT_TRUE(true);
}

}  // namespace
}  // namespace mujoco
