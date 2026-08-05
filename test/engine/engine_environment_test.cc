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

// Tests for engine/engine_environment.c.

#include "src/engine/engine_environment.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_derivative.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;
using EnvironmentTest = MujocoTest;

// one ulp below x: the largest height still strictly under the boundary x
mjtNum Below(mjtNum x) {
  return std::nextafter(x, -std::numeric_limits<mjtNum>::infinity());
}

// one ulp above x
mjtNum Above(mjtNum x) {
  return std::nextafter(x, std::numeric_limits<mjtNum>::infinity());
}

// linear scan over the same nlayer-1 interior boundaries mj_envLayer searches,
// with the same tie convention: a height on a boundary opens the upper layer
int LinearScanLayer(const mjModel* m, mjtNum z) {
  int nbound = (int)m->nlayer - 1;
  if (nbound <= 0) {
    return 0;
  }
  for (int i=0; i < nbound; i++) {
    if (z < m->layer_height[i]) {
      return i;
    }
  }
  return nbound;
}

// three layers, boundaries at -1.5 and 2.25; layer_height[2] is the unbounded
// top and must never be read
static constexpr char kThreeLayerXml[] = R"(
<mujoco>
  <option>
    <layer height="-1.5" density="1" viscosity="2" gravity="0 0 -1" wind="1 0 0"/>
    <layer height="2.25" density="3" viscosity="4" gravity="0 0 -2" wind="0 1 0"/>
    <layer height="9"    density="5" viscosity="6" gravity="0 0 -3" wind="0 0 1"/>
  </option>
</mujoco>
)";

// two identical spheres: body 0 sits in a dense layer, body 1 in a vacuum
// layer. the global medium is empty, so any fluid force at all proves the
// layer field reached the physics rather than mjOption.
std::string LayeredSpheresXml(const char* fluidshape, const char* integrator) {
  return std::string(R"(
<mujoco>
  <option density="0" viscosity="0" gravity="0 0 0" integrator=")") +
         integrator + R"(">
    <layer height="0" density="1000" viscosity="1"/>
    <layer height="1"/>
  </option>
  <worldbody>
    <body name="dense" pos="0 0 -1">
      <freejoint/>
      <geom type="sphere" size=".1" fluidshape=")" + fluidshape + R"("/>
    </body>
    <body name="vacuum" pos="0 0 2">
      <freejoint/>
      <geom type="sphere" size=".1" fluidshape=")" + fluidshape + R"("/>
    </body>
  </worldbody>
</mujoco>
)";
}

TEST_F(EnvironmentTest, BoundaryHeightOpensTheUpperLayer) {
  char error[1024] = {};
  MjModelPtr model = LoadModelFromString(kThreeLayerXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  ASSERT_EQ(model->nlayer, 3);
  ASSERT_EQ(model->layer_height[0], -1.5);
  ASSERT_EQ(model->layer_height[1], 2.25);

  // exactly on a boundary is the upper layer, one ulp below is the lower one
  EXPECT_EQ(mj_envLayer(model.get(), model->layer_height[0]), 1);
  EXPECT_EQ(mj_envLayer(model.get(), Below(model->layer_height[0])), 0);
  EXPECT_EQ(mj_envLayer(model.get(), Above(model->layer_height[0])), 1);
  EXPECT_EQ(mj_envLayer(model.get(), model->layer_height[1]), 2);
  EXPECT_EQ(mj_envLayer(model.get(), Below(model->layer_height[1])), 1);
  EXPECT_EQ(mj_envLayer(model.get(), Above(model->layer_height[1])), 2);

  // layer_height[nlayer-1] is the unbounded top and is never read
  EXPECT_EQ(mj_envLayer(model.get(), model->layer_height[2]), 2);
  EXPECT_EQ(mj_envLayer(model.get(), Below(model->layer_height[2])), 2);
  EXPECT_EQ(mj_envLayer(model.get(), Above(model->layer_height[2])), 2);

  // unbounded below and above
  EXPECT_EQ(mj_envLayer(model.get(), -1e12), 0);
  EXPECT_EQ(mj_envLayer(model.get(), 1e12), 2);
}

TEST_F(EnvironmentTest, BinarySearchMatchesLinearScan) {
  constexpr uint32_t kSeed = 0x1a7e5eedu;
  uint32_t state = kSeed;
  auto next = [&state]() {
    state = state * 1664525u + 1013904223u;
    return state;
  };

  int mismatches = 0;
  for (int nlayer = 1; nlayer <= 64; nlayer++) {
    // strictly ascending heights, all exact binary fractions so that the xml
    // round trip cannot perturb the boundaries under test
    std::vector<double> heights(nlayer);
    double height = 0.125 * (int)(next() % 512) - 32.0;
    for (int i=0; i < nlayer; i++) {
      heights[i] = height;
      height += 0.125 * (1 + next() % 32);
    }

    std::string xml = "<mujoco><option>";
    char buf[128];
    for (int i=0; i < nlayer; i++) {
      std::snprintf(buf, sizeof(buf), "<layer height=\"%.17g\"/>", heights[i]);
      xml += buf;
    }
    xml += "</option></mujoco>";

    char error[1024] = {};
    MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    ASSERT_EQ(model->nlayer, nlayer);
    for (int i=0; i < nlayer; i++) {
      ASSERT_EQ(model->layer_height[i], (mjtNum)heights[i]);
    }

    // every boundary at ulp resolution, plus interior, far above and far below
    std::vector<mjtNum> queries;
    for (int i=0; i < nlayer; i++) {
      mjtNum h = model->layer_height[i];
      queries.push_back(h);
      queries.push_back(Below(h));
      queries.push_back(Above(h));
      queries.push_back(h - 0.0625);
      queries.push_back(h + 0.0625);
    }
    queries.push_back(-1e12);
    queries.push_back(1e12);
    for (int i=0; i < 32; i++) {
      queries.push_back(0.125 * (int)(next() % 2048) - 128.0);
    }

    for (mjtNum z : queries) {
      int expected = LinearScanLayer(model.get(), z);
      int actual = mj_envLayer(model.get(), z);
      if (actual != expected) {
        mismatches++;
        ADD_FAILURE() << "seed=" << kSeed << " nlayer=" << nlayer
                      << " z=" << z << " binary=" << actual
                      << " linear=" << expected;
      }
    }
  }
  EXPECT_EQ(mismatches, 0);
}

TEST_F(EnvironmentTest, SingleMediumCopiesOptionBitIdentically) {
  static constexpr char kNoLayer[] = R"(
<mujoco>
  <option gravity="1 2 3" wind=".5 .25 .125" density="7" viscosity="11"/>
</mujoco>
)";
  static constexpr char kOneLayer[] = R"(
<mujoco>
  <option gravity="1 2 3" wind=".5 .25 .125" density="7" viscosity="11">
    <layer height="0" gravity="-4 -5 -6" wind="9 8 7" density="13" viscosity="17"/>
  </option>
</mujoco>
)";
  const char* xmls[] = {kNoLayer, kOneLayer};
  for (int nlayer = 0; nlayer < 2; nlayer++) {
    char error[1024] = {};
    MjModelPtr model = LoadModelFromString(xmls[nlayer], error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    ASSERT_EQ(model->nlayer, nlayer);

    for (mjtNum z : {(mjtNum)-1e6, (mjtNum)-1, (mjtNum)0, (mjtNum)1,
                     (mjtNum)1e6}) {
      const mjtNum pos[3] = {0.25, -0.5, z};
      mjEnv env;
      mj_envSample(model.get(), pos, &env);

      SCOPED_TRACE(::testing::Message() << "nlayer=" << nlayer << " z=" << z);

      // exact equality, not near: the legacy path must copy mjOption verbatim
      EXPECT_EQ(env.density, model->opt.density);
      EXPECT_EQ(env.viscosity, model->opt.viscosity);
      for (int k=0; k < 3; k++) {
        EXPECT_EQ(env.gravity[k], model->opt.gravity[k]);
        EXPECT_EQ(env.wind[k], model->opt.wind[k]);
      }
      EXPECT_EQ(mj_envLayer(model.get(), z), 0);
    }
  }
}

TEST_F(EnvironmentTest, SampleReturnsLayerValuesExactly) {
  char error[1024] = {};
  MjModelPtr model = LoadModelFromString(kThreeLayerXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  ASSERT_EQ(model->nlayer, 3);

  auto expect_layer = [&](mjtNum z, int i) {
    const mjtNum pos[3] = {3, -7, z};
    mjEnv env;
    mj_envSample(model.get(), pos, &env);

    SCOPED_TRACE(::testing::Message() << "z=" << z << " layer=" << i);
    EXPECT_EQ(mj_envLayer(model.get(), z), i);
    EXPECT_EQ(env.density, model->layer_density[i]);
    EXPECT_EQ(env.viscosity, model->layer_viscosity[i]);
    for (int k=0; k < 3; k++) {
      EXPECT_EQ(env.gravity[k], model->layer_gravity[3*i+k]);
      EXPECT_EQ(env.wind[k], model->layer_wind[3*i+k]);
    }
  };

  // interior of each layer
  expect_layer(-10, 0);
  expect_layer(0, 1);
  expect_layer(5, 2);

  // boundaries, at ulp resolution
  expect_layer(model->layer_height[0], 1);
  expect_layer(Below(model->layer_height[0]), 0);
  expect_layer(model->layer_height[1], 2);
  expect_layer(Below(model->layer_height[1]), 1);

  // the field is stratified along +z: x and y do not enter the lookup
  mjEnv left, right;
  const mjtNum pos_left[3] = {-1e6, 1e6, 0};
  const mjtNum pos_right[3] = {1e6, -1e6, 0};
  mj_envSample(model.get(), pos_left, &left);
  mj_envSample(model.get(), pos_right, &right);
  EXPECT_EQ(left.density, right.density);
  EXPECT_EQ(left.viscosity, right.viscosity);
  for (int k=0; k < 3; k++) {
    EXPECT_EQ(left.gravity[k], right.gravity[k]);
    EXPECT_EQ(left.wind[k], right.wind[k]);
  }
}

TEST_F(EnvironmentTest, FluidForceFollowsTheLayerField) {
  for (const char* fluidshape : {"none", "ellipsoid"}) {
    char error[1024] = {};
    MjModelPtr model = LoadModelFromString(
        LayeredSpheresXml(fluidshape, "Euler"), error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    ASSERT_EQ(model->nlayer, 2);
    ASSERT_EQ(model->nv, 12);
    ASSERT_EQ(model->opt.density, 0);
    ASSERT_EQ(model->opt.viscosity, 0);

    MjDataPtr data = MakeData(model);
    for (int i=0; i < model->nv; i++) {
      data->qvel[i] = 1;
    }
    mj_forward(model.get(), data.get());

    SCOPED_TRACE(::testing::Message() << "fluidshape=" << fluidshape);

    // the sphere in the dense layer is dragged, though mjOption is empty
    EXPECT_GT(mju_norm(data->qfrc_fluid, 6), 0);

    // the sphere in the vacuum layer feels exactly nothing
    for (int i=6; i < 12; i++) {
      EXPECT_EQ(data->qfrc_fluid[i], 0);
    }
  }
}

TEST_F(EnvironmentTest, FluidDerivativeFollowsTheLayerField) {
  for (const char* fluidshape : {"none", "ellipsoid"}) {
    char error[1024] = {};
    MjModelPtr model = LoadModelFromString(
        LayeredSpheresXml(fluidshape, "implicitfast"), error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    ASSERT_EQ(model->opt.integrator, mjINT_IMPLICITFAST);
    ASSERT_EQ(model->nv, 12);

    MjDataPtr data = MakeData(model);
    for (int i=0; i < model->nv; i++) {
      data->qvel[i] = 1;
    }
    mj_forward(model.get(), data.get());

    // no damping anywhere in this model, so every non-zero left in qDeriv by
    // mjd_passive_vel comes from the fluid model
    mju_zero(data->qDeriv, model->nD);
    mjd_passive_vel(model.get(), data.get());

    SCOPED_TRACE(::testing::Message() << "fluidshape=" << fluidshape);

    // the dense body's rows are populated: force and derivative agree on where
    // the body is
    for (int i=0; i < 6; i++) {
      EXPECT_GT(mju_norm(data->qDeriv + model->D_rowadr[i],
                         model->D_rownnz[i]), 0)
          << "dof " << i;
    }

    // the vacuum body's rows are exactly zero
    for (int i=6; i < 12; i++) {
      for (int k=0; k < model->D_rownnz[i]; k++) {
        EXPECT_EQ(data->qDeriv[model->D_rowadr[i] + k], 0)
            << "dof " << i << " entry " << k;
      }
    }
  }
}

TEST_F(EnvironmentTest, ModelWithoutLayerElementHasNoLayers) {
  static constexpr char xml[] = R"(
<mujoco>
  <option density="1.2" viscosity="1.8e-5"/>
  <worldbody>
    <body>
      <freejoint/>
      <geom type="sphere" size=".1"/>
    </body>
  </worldbody>
</mujoco>
)";
  char error[1024] = {};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  EXPECT_EQ(model->nlayer, 0);
  EXPECT_EQ(mj_envLayer(model.get(), 0), 0);
  EXPECT_EQ(mj_envLayer(model.get(), -1e9), 0);
  EXPECT_EQ(mj_envLayer(model.get(), 1e9), 0);
}

// ------------------------------ adversarial ---------------------------------

// the memcmp comparisons below treat mjEnv as eight numbers and nothing else
static_assert(sizeof(mjEnv) == 8*sizeof(mjtNum),
              "mjEnv has padding; the memcmp comparisons compare it too");

constexpr mjtNum kInf = std::numeric_limits<mjtNum>::infinity();
constexpr mjtNum kMax = std::numeric_limits<mjtNum>::max();
constexpr mjtNum kDenorm = std::numeric_limits<mjtNum>::denorm_min();
constexpr mjtNum kNan = std::numeric_limits<mjtNum>::quiet_NaN();

// a value that appears nowhere in any model below, used to poison memory that
// a correct implementation must overwrite or must never read
constexpr mjtNum kPoison = -98765.4321;

// a model whose only content is the given layer boundaries. %.17g round trips a
// double exactly; the round trip is asserted rather than assumed
MjModelPtr LoadLayers(const std::vector<mjtNum>& heights) {
  std::string xml = "<mujoco><option>";
  char buf[128];
  for (mjtNum h : heights) {
    std::snprintf(buf, sizeof(buf), "<layer height=\"%.17g\"/>", (double)h);
    xml += buf;
  }
  xml += "</option></mujoco>";

  char error[1024] = {};
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  if (model == nullptr) {
    ADD_FAILURE() << error << "\nxml: " << xml;
    return model;
  }
  EXPECT_EQ(model->nlayer, (mjtSize)heights.size());
  for (size_t i=0; i < heights.size(); i++) {
    EXPECT_EQ(model->layer_height[i], heights[i])
        << "boundary " << i << " did not round trip through xml";
  }
  return model;
}

// ordered query heights that separate implementations: both ends of the format,
// both signed zeros, the smallest and largest magnitudes, in ascending order.
// NaN is deliberately absent -- it is unordered, so "first i with z < h[i]" and
// "last i with h[i] <= z" are not the same question for it, and the linear scan
// and the binary search disagree by construction (see AdversarialNonFinite*)
std::vector<mjtNum> OrderedExtremeQueries() {
  return {-kInf, -kMax, -1e300, -1.0, -kDenorm, -0.0,
          0.0,   kDenorm, 1.0,  1e300, kMax,    kInf};
}

// the above plus every boundary of m at ulp resolution
std::vector<mjtNum> BoundaryQueries(const mjModel* m) {
  std::vector<mjtNum> queries = OrderedExtremeQueries();
  for (int i=0; i < (int)m->nlayer; i++) {
    mjtNum h = m->layer_height[i];
    queries.push_back(Below(h));
    queries.push_back(h);
    queries.push_back(Above(h));
  }
  return queries;
}

// index of the first mjtNum in env that memcmp says is still poison, or -1
int FirstUnwrittenField(const mjEnv& env, unsigned char fill) {
  mjEnv poison;
  std::memset(&poison, fill, sizeof(poison));
  const auto* got = reinterpret_cast<const unsigned char*>(&env);
  const auto* want = reinterpret_cast<const unsigned char*>(&poison);
  for (int k=0; k < 8; k++) {
    if (!std::memcmp(got + k*sizeof(mjtNum), want + k*sizeof(mjtNum),
                     sizeof(mjtNum))) {
      return k;
    }
  }
  return -1;
}

TEST_F(EnvironmentTest, AdversarialNonFiniteQueryHeights) {
  char error[1024] = {};
  MjModelPtr model = LoadModelFromString(kThreeLayerXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  ASSERT_EQ(model->nlayer, 3);

  // +inf is above every boundary: the unbounded top layer
  EXPECT_EQ(mj_envLayer(model.get(), kInf), 2);

  // -inf is below every boundary: layer 0
  EXPECT_EQ(mj_envLayer(model.get(), -kInf), 0);

  // every comparison against NaN is false, so the search always takes the upper
  // branch and collapses onto layer 0, for either sign of NaN
  EXPECT_EQ(mj_envLayer(model.get(), kNan), 0);
  EXPECT_EQ(mj_envLayer(model.get(), -kNan), 0);

  // whatever it returns, it is a legal index into the five layer arrays
  for (mjtNum z : {kInf, -kInf, kNan, -kNan}) {
    int i = mj_envLayer(model.get(), z);
    EXPECT_GE(i, 0) << "z=" << z;
    EXPECT_LT(i, (int)model->nlayer) << "z=" << z;
  }

  // a NaN query still writes all four fields: fill the struct with a poison
  // byte pattern and require every one of its eight numbers to be overwritten
  for (mjtNum z : {kNan, -kNan, kInf, -kInf}) {
    mjEnv env;
    std::memset(&env, 0xa5, sizeof(env));
    const mjtNum pos[3] = {0, 0, z};
    mj_envSample(model.get(), pos, &env);

    SCOPED_TRACE(::testing::Message() << "z=" << z);
    EXPECT_EQ(FirstUnwrittenField(env, 0xa5), -1) << "field left as poison";

    int i = mj_envLayer(model.get(), z);
    EXPECT_EQ(env.density, model->layer_density[i]);
    EXPECT_EQ(env.viscosity, model->layer_viscosity[i]);
    for (int k=0; k < 3; k++) {
      EXPECT_EQ(env.gravity[k], model->layer_gravity[3*i+k]);
      EXPECT_EQ(env.wind[k], model->layer_wind[3*i+k]);
    }
  }
}

TEST_F(EnvironmentTest, AdversarialLayerIndexAlwaysInRange) {
  constexpr uint32_t kSeed = 0x5eed1de3u;
  uint32_t state = kSeed;
  auto next = [&state]() {
    state = state * 1664525u + 1013904223u;
    return state;
  };

  for (int nlayer = 0; nlayer <= 48; nlayer++) {
    std::vector<mjtNum> heights(nlayer);
    mjtNum height = 0.125 * (int)(next() % 512) - 32.0;
    for (int i=0; i < nlayer; i++) {
      heights[i] = height;
      height += 0.125 * (1 + next() % 32);
    }
    MjModelPtr model = LoadLayers(heights);
    ASSERT_THAT(model.get(), NotNull());

    std::vector<mjtNum> queries = BoundaryQueries(model.get());
    queries.push_back(kNan);
    queries.push_back(-kNan);
    for (int i=0; i < 16; i++) {
      queries.push_back(0.125 * (int)(next() % 2048) - 128.0);
    }

    // nlayer == 0 is the single-medium sentinel: layer 0 is the only legal index
    int nvalid = nlayer > 0 ? nlayer : 1;
    for (mjtNum z : queries) {
      int i = mj_envLayer(model.get(), z);
      ASSERT_GE(i, 0) << "seed=" << kSeed << " nlayer=" << nlayer << " z=" << z;
      ASSERT_LT(i, nvalid) << "seed=" << kSeed << " nlayer=" << nlayer
                           << " z=" << z;

      // the index is used unchecked to read five arrays, so sample as well: an
      // out-of-range index is then a heap error under asan, not just a number
      mjEnv env;
      const mjtNum pos[3] = {0, 0, z};
      mj_envSample(model.get(), pos, &env);
    }
  }
}

TEST_F(EnvironmentTest, AdversarialMonotonicity) {
  constexpr uint32_t kSeed = 0x0ff5e7edu;
  uint32_t state = kSeed;
  auto next = [&state]() {
    state = state * 1664525u + 1013904223u;
    return state;
  };

  for (int trial = 0; trial < 8; trial++) {
    int nlayer = 1 + next() % 40;
    std::vector<mjtNum> heights(nlayer);
    mjtNum height = 0.125 * (int)(next() % 512) - 32.0;
    for (int i=0; i < nlayer; i++) {
      heights[i] = height;
      height += 0.125 * (1 + next() % 24);
    }
    MjModelPtr model = LoadLayers(heights);
    ASSERT_THAT(model.get(), NotNull());

    // boundaries at ulp resolution, the ordered extremes, and random heights:
    // a broken comparison can match a linear scan on random points and still
    // invert on two adjacent ones
    std::vector<mjtNum> queries = BoundaryQueries(model.get());
    for (int i=0; i < 64; i++) {
      queries.push_back(0.125 * (int)(next() % 2048) - 128.0);
    }
    std::sort(queries.begin(), queries.end());

    int prev = mj_envLayer(model.get(), queries.front());
    for (mjtNum z : queries) {
      int cur = mj_envLayer(model.get(), z);
      ASSERT_LE(prev, cur) << "seed=" << kSeed << " trial=" << trial
                           << " nlayer=" << nlayer << " z=" << z
                           << " dropped from layer " << prev << " to " << cur;
      prev = cur;
    }

    // and the last query, the largest height, is in the top layer
    EXPECT_EQ(prev, nlayer - 1);
  }
}

TEST_F(EnvironmentTest, AdversarialRepeatableAndStateless) {
  char error[1024] = {};
  MjModelPtr model = LoadModelFromString(kThreeLayerXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  // a second, unrelated model: different layer count, different boundary, and
  // bodies to step
  MjModelPtr other = LoadModelFromString(
      LayeredSpheresXml("ellipsoid", "Euler"), error, sizeof(error));
  ASSERT_THAT(other.get(), NotNull()) << error;
  MjDataPtr data = MakeData(other);

  std::vector<mjtNum> queries = BoundaryQueries(model.get());
  queries.push_back(kNan);

  // baseline, forward
  std::vector<int> layers(queries.size());
  std::vector<mjEnv> envs(queries.size());
  for (size_t k=0; k < queries.size(); k++) {
    std::memset(&envs[k], 0, sizeof(mjEnv));
    const mjtNum pos[3] = {0, 0, queries[k]};
    layers[k] = mj_envLayer(model.get(), queries[k]);
    mj_envSample(model.get(), pos, &envs[k]);
  }

  auto check = [&](const char* pass, size_t k) {
    const mjtNum pos[3] = {0, 0, queries[k]};
    mjEnv env;
    std::memset(&env, 0, sizeof(env));
    mj_envSample(model.get(), pos, &env);
    EXPECT_EQ(mj_envLayer(model.get(), queries[k]), layers[k])
        << pass << " z=" << queries[k];
    EXPECT_EQ(std::memcmp(&env, &envs[k], sizeof(mjEnv)), 0)
        << pass << " z=" << queries[k];
  };

  // reverse
  for (size_t k = queries.size(); k-- > 0;) {
    check("reverse", k);
  }

  // interleaved with a different model, and with stepping it. the interleaved
  // query is at the same height and a different layer count on purpose: any
  // lookup keyed on z alone, or warm started from the previous call, answers
  // the second query with the first model's index
  for (size_t k=0; k < queries.size(); k++) {
    mj_step(other.get(), data.get());
    mjEnv scratch;
    const mjtNum elsewhere[3] = {1, 2, queries[k]};
    mj_envSample(other.get(), elsewhere, &scratch);
    mj_envLayer(other.get(), queries[k]);
    check("interleaved", k);
  }

  // after a hundred steps of the other model
  for (int i=0; i < 100; i++) {
    mj_step(other.get(), data.get());
  }
  for (size_t k=0; k < queries.size(); k++) {
    check("after stepping", k);
  }
}

TEST_F(EnvironmentTest, AdversarialExtremeAndDegenerateHeights) {
  const mjtNum one_up = Above((mjtNum)1);
  const std::vector<std::vector<mjtNum>> sets = {
      // boundary at +0, and at -0: the last entry is the unbounded top
      {0.0, 1.0},
      {-0.0, 1.0},
      // denormal boundaries
      {0.0, kDenorm, 2*kDenorm, 4*kDenorm},
      // boundaries one ulp apart
      {1.0, one_up, Above(one_up), Above(Above(one_up))},
      // very large and very small magnitudes
      {-1e300, -1e-300, 1e-300, 1e300},
      // a mix of all of them
      {-1e300, -1.0, -kDenorm, 0.0, kDenorm, 1.0, one_up, 1e300},
  };

  for (const std::vector<mjtNum>& heights : sets) {
    MjModelPtr model = LoadLayers(heights);
    ASSERT_THAT(model.get(), NotNull());

    for (mjtNum z : BoundaryQueries(model.get())) {
      SCOPED_TRACE(::testing::Message()
                   << "nlayer=" << model->nlayer << " h0=" << heights[0]
                   << " z=" << z);
      EXPECT_EQ(mj_envLayer(model.get(), z), LinearScanLayer(model.get(), z));
    }
  }

  // -0.0 <= 0.0 and 0.0 <= -0.0 are both true, so the sign of a zero boundary
  // and the sign of a zero query are both invisible to the search
  for (mjtNum boundary : {(mjtNum)0.0, (mjtNum)-0.0}) {
    MjModelPtr model = LoadLayers({boundary, 1.0});
    ASSERT_THAT(model.get(), NotNull());
    ASSERT_EQ(std::signbit(model->layer_height[0]), std::signbit(boundary))
        << "the sign of the zero boundary did not survive the xml round trip";
    EXPECT_EQ(mj_envLayer(model.get(), 0.0), 1);
    EXPECT_EQ(mj_envLayer(model.get(), -0.0), 1);
    EXPECT_EQ(mj_envLayer(model.get(), Below((mjtNum)0)), 0);
    EXPECT_EQ(mj_envLayer(model.get(), -kDenorm), 0);
    EXPECT_EQ(mj_envLayer(model.get(), kDenorm), 1);
  }
}

TEST_F(EnvironmentTest, AdversarialSingleMediumPurity) {
  static constexpr char kNoLayer[] = R"(
<mujoco>
  <option gravity="1 2 3" wind=".5 .25 .125" density="7" viscosity="11"/>
</mujoco>
)";
  static constexpr char kOneLayer[] = R"(
<mujoco>
  <option gravity="1 2 3" wind=".5 .25 .125" density="7" viscosity="11">
    <layer height="0" gravity="-4 -5 -6" wind="9 8 7" density="13" viscosity="17"/>
  </option>
</mujoco>
)";
  const char* xmls[] = {kNoLayer, kOneLayer};
  for (int nlayer = 0; nlayer < 2; nlayer++) {
    char error[1024] = {};
    MjModelPtr model = LoadModelFromString(xmls[nlayer], error, sizeof(error));
    ASSERT_THAT(model.get(), NotNull()) << error;
    ASSERT_EQ(model->nlayer, nlayer);

    // a single-layer model must not consult its layer arrays at all: fill them
    // with a value that appears nowhere in mjOption, and with a NaN boundary
    if (nlayer == 1) {
      model->layer_height[0] = kNan;
      model->layer_density[0] = kPoison;
      model->layer_viscosity[0] = kPoison;
      for (int k=0; k < 3; k++) {
        model->layer_gravity[k] = kPoison;
        model->layer_wind[k] = kPoison;
      }
    }

    for (mjtNum z : {kInf, -kInf, kNan, -kNan, (mjtNum)1e300, (mjtNum)-1e300,
                     kMax, -kMax, kDenorm, -kDenorm, (mjtNum)0.0,
                     (mjtNum)-0.0}) {
      const mjtNum pos[3] = {0.25, -0.5, z};
      mjEnv env;
      std::memset(&env, 0xa5, sizeof(env));
      mj_envSample(model.get(), pos, &env);

      SCOPED_TRACE(::testing::Message() << "nlayer=" << nlayer << " z=" << z);

      // exact equality on the raw doubles, not near
      EXPECT_EQ(env.density, model->opt.density);
      EXPECT_EQ(env.viscosity, model->opt.viscosity);
      for (int k=0; k < 3; k++) {
        EXPECT_EQ(env.gravity[k], model->opt.gravity[k]);
        EXPECT_EQ(env.wind[k], model->opt.wind[k]);
      }

      // every field written, and no poison anywhere in the output
      EXPECT_EQ(FirstUnwrittenField(env, 0xa5), -1);
      EXPECT_NE(env.density, kPoison);
      EXPECT_NE(env.viscosity, kPoison);
      for (int k=0; k < 3; k++) {
        EXPECT_NE(env.gravity[k], kPoison);
        EXPECT_NE(env.wind[k], kPoison);
      }
      EXPECT_EQ(mj_envLayer(model.get(), z), 0);
    }
  }
}

TEST_F(EnvironmentTest, AdversarialIdempotentSampling) {
  char error[1024] = {};
  MjModelPtr model = LoadModelFromString(kThreeLayerXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;

  std::vector<mjtNum> queries = BoundaryQueries(model.get());
  queries.push_back(kNan);
  queries.push_back(-kNan);

  for (mjtNum z : queries) {
    const mjtNum pos[3] = {-2, 0.5, z};

    // opposite fills: a byte the sample fails to write cannot agree
    mjEnv first, second;
    std::memset(&first, 0x00, sizeof(first));
    std::memset(&second, 0xff, sizeof(second));
    mj_envSample(model.get(), pos, &first);
    mj_envSample(model.get(), pos, &second);

    SCOPED_TRACE(::testing::Message() << "z=" << z);
    EXPECT_EQ(std::memcmp(&first, &second, sizeof(mjEnv)), 0);

    // sampling on top of an already sampled struct changes nothing
    mjEnv again = first;
    mj_envSample(model.get(), pos, &again);
    EXPECT_EQ(std::memcmp(&again, &first, sizeof(mjEnv)), 0);
  }
}

}  // namespace
}  // namespace mujoco
