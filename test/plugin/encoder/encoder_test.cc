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

// Tests for encoder plugins.

#include <cstdio>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

struct FakeEncoderOutput {
  int nbody;
  int ngeom;
  int njnt;
  char resource_name[512];
};

int FakeEncode(const mjSpec* s, const mjModel* m, const mjVFS* vfs,
               mjResource* resource) {
  auto* output = new FakeEncoderOutput;
  output->nbody = m->nbody;
  output->ngeom = m->ngeom;
  output->njnt = m->njnt;
  std::snprintf(output->resource_name, sizeof(output->resource_name), "%s",
                resource->name);
  resource->data = output;
  return sizeof(FakeEncoderOutput);
}

void CloseResource(mjResource* resource) {
  delete static_cast<FakeEncoderOutput*>(resource->data);
  delete resource;
}

mjpEncoder FakeEncoder() {
  mjpEncoder encoder;
  mjp_defaultEncoder(&encoder);
  encoder.content_type = "model/fakeformat";
  encoder.extension = ".fakeformat|.alsoFakeFormat";
  encoder.encode = FakeEncode;
  encoder.close_resource = CloseResource;
  return encoder;
}

using EncoderPluginTest = MujocoTest;

TEST_F(EncoderPluginTest, RegisterAndFindByExtension) {
  mjpEncoder encoder = FakeEncoder();
  mjp_registerEncoder(&encoder);

  const mjpEncoder* found = mjp_findEncoder("output.fakeformat", nullptr);
  ASSERT_THAT(found, testing::NotNull());
  EXPECT_EQ(found->encode, FakeEncode);
}

TEST_F(EncoderPluginTest, FindByAlternateExtension) {
  const mjpEncoder* found = mjp_findEncoder("output.alsoFakeFormat", nullptr);
  ASSERT_THAT(found, testing::NotNull());
  EXPECT_EQ(found->encode, FakeEncode);
}

TEST_F(EncoderPluginTest, FindByContentType) {
  const mjpEncoder* found = mjp_findEncoder(nullptr, "model/fakeformat");
  ASSERT_THAT(found, testing::NotNull());
  EXPECT_EQ(found->encode, FakeEncode);
}

TEST_F(EncoderPluginTest, FindUnknownExtensionReturnsNull) {
  const mjpEncoder* found = mjp_findEncoder("output.unknown", nullptr);
  EXPECT_THAT(found, testing::IsNull());
}

TEST_F(EncoderPluginTest, DefaultEncoderIsZeroed) {
  mjpEncoder encoder;
  mjp_defaultEncoder(&encoder);
  EXPECT_EQ(encoder.content_type, nullptr);
  EXPECT_EQ(encoder.extension, nullptr);
  EXPECT_EQ(encoder.encode, nullptr);
}

TEST_F(EncoderPluginTest, EncodeModel) {
  mjSpec* spec = mj_makeSpec();
  mjsBody* world = mjs_findBody(spec, "world");
  mjsBody* body = mjs_addBody(world, nullptr);
  mjsGeom* geom = mjs_addGeom(body, nullptr);
  geom->size[0] = 1.0;
  geom->size[1] = 1.0;
  geom->size[2] = 1.0;

  mjModel* model = mj_compile(spec, nullptr);
  ASSERT_THAT(model, testing::NotNull());

  const mjpEncoder* found = mjp_findEncoder("output.fakeformat", nullptr);
  ASSERT_THAT(found, testing::NotNull());

  mjResource resource = {};
  resource.name = const_cast<char*>("output.fakeformat");

  int result = found->encode(spec, model, nullptr, &resource);
  EXPECT_GT(result, 0);

  auto* output = static_cast<FakeEncoderOutput*>(resource.data);
  ASSERT_THAT(output, testing::NotNull());
  EXPECT_EQ(output->nbody, 2);
  EXPECT_EQ(output->ngeom, 1);
  EXPECT_EQ(output->njnt, 0);
  EXPECT_STREQ(output->resource_name, "output.fakeformat");

  delete output;
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}

}  // namespace
}  // namespace mujoco
