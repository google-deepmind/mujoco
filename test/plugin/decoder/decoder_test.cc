// Copyright 2025 DeepMind Technologies Limited
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

// Tests for decoder plugins.

#include <string.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// A simple mjSpec with one body and one geom.
static mjSpec* MakeSimpleSpec() {
  mjSpec* s = mj_makeSpec();
  mjsBody* world = mjs_findBody(s, "world");
  mjsBody* body = mjs_addBody(world, nullptr);
  mjsGeom* geom = mjs_addGeom(body, nullptr);
  geom->size[0] = 1.0;
  geom->size[1] = 1.0;
  geom->size[2] = 1.0;
  return s;
}

// Always returns a simple mjSpec, ignoring the resource.
mjSpec* FakeDecode(mjResource* resource) { return MakeSimpleSpec(); }

// Can decode any resource that has a .fakeformat extension.
int FakeCanDecode(const mjResource* resource) {
  const char* ext = strrchr(resource->name, '.');
  if (ext) {
    return strcmp(ext, ".fakeformat") == 0 ||
           strcmp(ext, ".alsoFakeFormat") == 0;
  }
  return 0;
}

mjpDecoder FakeDecoder() {
  mjpDecoder decoder;
  mjp_defaultDecoder(&decoder);
  decoder.content_type = "model/fakeformat";
  decoder.extension = ".fakeformat|.alsoFakeFormat";
  decoder.can_decode = FakeCanDecode;
  decoder.decode = FakeDecode;
  return decoder;
}

using DecoderPluginTest = MujocoTest;

TEST_F(DecoderPluginTest, CanDecode) {
  mjpDecoder decoder = FakeDecoder();
  mjp_registerDecoder(&decoder);

  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <model name="fakeformat" file="dummy.fakeformat"/>
      <model name="also_fakeformat" file="dummy.alsoFakeFormat"/>
    </asset>
    <worldbody>
      <attach model="fakeformat" prefix="test"/>
    </worldbody>
  </mujoco>
  )";

  // Check referencing a resource via XML invokes the decoder.
  char error[1024];
  mjSpec* spec =
      mj_parseXMLString(xml, nullptr, error, sizeof(error));
  mjModel* model = mj_compile(spec, nullptr);
  ASSERT_THAT(model, testing::NotNull()) << error;
  EXPECT_EQ(model->nbody, 2);  // world + included body
  EXPECT_EQ(model->ngeom, 1);
  mj_deleteModel(model);
  mj_deleteSpec(spec);

  // Check mj_parse with extension .fakeformat
  spec =
      mj_parse("dummy.fakeformat", nullptr, nullptr, error, sizeof(error));
  model = mj_compile(spec, nullptr);
  EXPECT_EQ(model->nbody, 2);  // world + included body
  EXPECT_EQ(model->ngeom, 1);
  mj_deleteModel(model);
  mj_deleteSpec(spec);

  // Check mj_parse with extension .alsoFakeFormat
  spec =
      mj_parse("dummy.alsoFakeFormat", nullptr, nullptr, error, sizeof(error));
  model = mj_compile(spec, nullptr);
  EXPECT_EQ(model->nbody, 2);  // world + included body
  EXPECT_EQ(model->ngeom, 1);
  mj_deleteModel(model);
  mj_deleteSpec(spec);

  // Check mj_parse with content_type
  spec = mj_parse("dummy.fakeformat", "model/fakeformat", nullptr, error,
                  sizeof(error));
  model = mj_compile(spec, nullptr);
  EXPECT_EQ(model->nbody, 2);  // world + included body
  EXPECT_EQ(model->ngeom, 1);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

}  // namespace
}  // namespace mujoco
