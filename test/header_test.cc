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
#include <utility>
#include <vector>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

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

TEST_F(HeaderTest, MjOptionFloatsOrdered) {
  mjOption o;
  std::vector<std::pair<const void*, const char*>> floats;

#define X(type, name) \
  floats.push_back({static_cast<const void*>(&o.name), #name});
  MJOPTION_FLOATS
#undef X

  CheckAddressOrdering(floats, "MJOPTION_FLOATS");
}

TEST_F(HeaderTest, MjOptionVectorsOrdered) {
  mjOption o;
  std::vector<std::pair<const void*, const char*>> vectors;

#define X(name, dim) \
  vectors.push_back({static_cast<const void*>(&o.name), #name});
  MJOPTION_VECTORS
#undef X

  CheckAddressOrdering(vectors, "MJOPTION_VECTORS");
}

TEST_F(HeaderTest, MjModelIntsOrdered) {
  mjModel m;
  std::vector<std::pair<const void*, const char*>> ints;

#define X(name) ints.push_back({static_cast<const void*>(&m.name), #name});
  MJMODEL_INTS
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
