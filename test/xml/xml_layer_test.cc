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

// Tests for the <option><layer> element: parsing, validation and writing.

#include <cstddef>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;
using LayerTest = MujocoTest;

// three layers; the second inherits density and wind from the global medium,
// so the writer must omit exactly those two attributes for it
static constexpr char kThreeLayerXml[] = R"(
<mujoco>
  <option density="1.2" viscosity="1.8e-5" wind="1 2 3">
    <layer height="-1.5" gravity="0 0 -9" density="1000" viscosity="1e-3"
           wind="0 0 0"/>
    <layer height="2.25" viscosity="5e-5"/>
    <layer height="3" gravity="0 0 0" density="0" viscosity="0" wind="0 0 0"/>
  </option>
</mujoco>
)";

// every field of every layer of a model loaded from kThreeLayerXml
void ExpectThreeLayers(const mjModel* m) {
  ASSERT_EQ(m->nlayer, 3);

  // layer 0: all attributes given
  EXPECT_EQ(m->layer_height[0], -1.5);
  EXPECT_EQ(m->layer_gravity[2], -9);
  EXPECT_EQ(m->layer_density[0], 1000);
  EXPECT_EQ(m->layer_viscosity[0], 1e-3);
  EXPECT_EQ(m->layer_wind[0], 0);

  // layer 1: density, gravity and wind fall back to the global medium
  EXPECT_EQ(m->layer_height[1], 2.25);
  EXPECT_EQ(m->layer_gravity[5], m->opt.gravity[2]);
  EXPECT_EQ(m->layer_density[1], 1.2);
  EXPECT_EQ(m->layer_viscosity[1], 5e-5);
  EXPECT_EQ(m->layer_wind[3], 1);
  EXPECT_EQ(m->layer_wind[4], 2);
  EXPECT_EQ(m->layer_wind[5], 3);

  // layer 2: the unbounded top, vacuum
  EXPECT_EQ(m->layer_gravity[8], 0);
  EXPECT_EQ(m->layer_density[2], 0);
  EXPECT_EQ(m->layer_viscosity[2], 0);
  EXPECT_EQ(m->layer_wind[6], 0);
}

TEST_F(LayerTest, LayersRoundTripThroughXml) {
  char error[1024] = {};
  MjModelPtr model = LoadModelFromString(kThreeLayerXml, error, sizeof(error));
  ASSERT_THAT(model.get(), NotNull()) << error;
  ExpectThreeLayers(model.get());

  // every layer is written back out, and only the layers
  std::string saved = SaveAndReadXml(model.get());
  int written = 0;
  for (size_t p = saved.find("<layer"); p != std::string::npos;
       p = saved.find("<layer", p + 1)) {
    written++;
  }
  EXPECT_EQ(written, 3) << saved;

  MjModelPtr reloaded = LoadModelFromString(saved, error, sizeof(error));
  ASSERT_THAT(reloaded.get(), NotNull()) << error;
  ExpectThreeLayers(reloaded.get());
}

TEST_F(LayerTest, RejectsUnsortedAndDuplicateLayerHeights) {
  static constexpr char kUnsorted[] = R"(
<mujoco>
  <option>
    <layer height="1"/>
    <layer height="0"/>
    <layer height="2"/>
  </option>
</mujoco>
)";
  static constexpr char kDuplicate[] = R"(
<mujoco>
  <option>
    <layer height="0"/>
    <layer height="0"/>
  </option>
</mujoco>
)";
  for (const char* xml : {kUnsorted, kDuplicate}) {
    char error[1024] = {};
    MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
    EXPECT_THAT(model.get(), IsNull());
    EXPECT_THAT(error, HasSubstr("layer heights must be strictly ascending"));
  }
}

TEST_F(LayerTest, ModelWithoutLayerElementHasNoLayers) {
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
}

TEST_F(LayerTest, AddLayerFromSpec) {
  mjSpec* spec = mj_makeSpec();
  spec->option.density = 1.2;
  spec->option.viscosity = 1.8e-5;

  // a new layer copies the global medium
  mjsLayer* bottom = mjs_addLayer(spec);
  ASSERT_THAT(bottom, NotNull());
  EXPECT_EQ(bottom->density, 1.2);
  EXPECT_EQ(bottom->viscosity, 1.8e-5);
  bottom->height = -1;
  bottom->density = 1000;

  mjsLayer* top = mjs_addLayer(spec);
  ASSERT_THAT(top, NotNull());
  top->height = 1;
  top->wind[0] = 7;

  // adding did not invalidate the pointer handed out earlier
  EXPECT_EQ(mjs_getLayer(spec, 0), bottom);
  EXPECT_EQ(mjs_getLayer(spec, 1), top);
  EXPECT_THAT(mjs_getLayer(spec, 2), IsNull());
  EXPECT_THAT(mjs_getLayer(spec, -1), IsNull());

  mjModel* model = mj_compile(spec, nullptr);
  ASSERT_THAT(model, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(model->nlayer, 2);
  EXPECT_EQ(model->layer_height[0], -1);
  EXPECT_EQ(model->layer_density[0], 1000);
  EXPECT_EQ(model->layer_viscosity[0], 1.8e-5);
  EXPECT_EQ(model->layer_density[1], 1.2);
  EXPECT_EQ(model->layer_wind[3], 7);
  mj_deleteModel(model);

  mjs_deleteLayers(spec);
  EXPECT_THAT(mjs_getLayer(spec, 0), IsNull());
  model = mj_compile(spec, nullptr);
  ASSERT_THAT(model, NotNull()) << mjs_getError(spec);
  EXPECT_EQ(model->nlayer, 0);
  mj_deleteModel(model);

  mj_deleteSpec(spec);
}

}  // namespace
}  // namespace mujoco
