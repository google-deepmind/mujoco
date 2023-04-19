// Copyright 2023 DeepMind Technologies Limited
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

// Tests for engine/engine_resource.c

#include <cstring>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "src/cc/array_safety.h"
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_plugin.h"
#include "src/engine/engine_resource.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::StrEq;

using ResourceTest = MujocoTest;

int open_nop(mjResource* resource) {
  return 1;
}

int open_str(mjResource* resource) {
  if (std::strcmp(resource->name, "str://file")) {
    return 0;
  }

  resource->data = mju_malloc(100*sizeof(char));
  std::strcpy((char*) resource->data, "Hello World");
  return 1;
}

int read_nop(mjResource* resource, const void** buffer) {
  return 0;
}

int read_str(mjResource* resource, const void** buffer) {
  *buffer = resource->data;
  return std::strlen((const char*) resource->data) + 1;
}

void close_nop(mjResource* resource) {
}

void close_str(mjResource* resource) {
  mju_free(resource->data);
}

TEST_F(ResourceTest, RegisterProviderSuccess) {
  mjpResourceProvider provider = {"myprefix", open_nop, read_nop, close_nop,
                                  nullptr};

  int count1 = mjp_resourceProviderCount();
  int i = mjp_registerResourceProvider(&provider);
  int count2 = mjp_resourceProviderCount();

  EXPECT_GT(i, 0);
  EXPECT_EQ(count1+1, count2);
}

TEST_F(ResourceTest, RegisterProviderMissingCallbacks) {
  mjpResourceProvider provider = {"myprefix", nullptr, nullptr, nullptr,
                                  nullptr};

  // install warning handler
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
    util::strcpy_arr(warning, msg);
  };

  int i = mjp_registerResourceProvider(&provider);

  // warning message related to missing callbacks
  EXPECT_THAT(warning, HasSubstr("callback"));
  EXPECT_LT(i, 1);
}

TEST_F(ResourceTest, RegisterProviderMissingPrefix) {
  mjpResourceProvider provider = {"", open_nop, read_nop, close_nop, nullptr};

  // install warning handler
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
    util::strcpy_arr(warning, msg);
  };

  int i = mjp_registerResourceProvider(&provider);

  // warning message related to missing prefix
  EXPECT_THAT(warning, HasSubstr("prefix"));
  EXPECT_LT(i, 1);
}

TEST_F(ResourceTest, RegisterProviderSubPrefix) {
  mjpResourceProvider provider = {"prefix", open_nop, read_nop, close_nop,
                                  nullptr};

  mjpResourceProvider provider2 = {"pre", open_nop, read_nop, close_nop,
                                   nullptr};

  // install warning handler
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
    util::strcpy_arr(warning, msg);
  };

  int i = mjp_registerResourceProvider(&provider);
  int j = mjp_registerResourceProvider(&provider2);


  // warning message related to an error
  EXPECT_THAT(warning, HasSubstr("cannot be register"));
  EXPECT_GT(i, 0);
  EXPECT_LT(j, 1);
}

TEST_F(ResourceTest, RegisterProviderSuperPrefix) {
  mjpResourceProvider provider = {"prefix", open_nop, read_nop, close_nop,
                                  nullptr};

  mjpResourceProvider provider2 = {"prefix2", open_nop, read_nop, close_nop,
                                   nullptr};

  // install warning handler
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
    util::strcpy_arr(warning, msg);
  };

  int i = mjp_registerResourceProvider(&provider);
  int j = mjp_registerResourceProvider(&provider2);


  // warning message related to an error
  EXPECT_THAT(warning, HasSubstr("cannot be register"));
  EXPECT_GT(i, 0);
  EXPECT_LT(j, 1);
}

TEST_F(ResourceTest, RegisterProviderSame) {
  mjpResourceProvider provider = {"prefix", open_nop, read_nop, close_nop,
                                  nullptr};

  mjpResourceProvider provider2 = {"prefix", open_nop, read_nop, close_nop,
                                   nullptr};

  int i1 = mjp_registerResourceProvider(&provider);
  int count1 = mjp_resourceProviderCount();

  int i2 = mjp_registerResourceProvider(&provider2);
  int count2 = mjp_resourceProviderCount();

  EXPECT_EQ(i1, i2);
  EXPECT_EQ(count1, count2);

}

TEST_F(ResourceTest, GeneralTest) {
  mjpResourceProvider provider = {"str://", open_str, read_str, close_str,
                                  nullptr};

  // register resource provider
  int i = mjp_registerResourceProvider(&provider);
  EXPECT_GT(i, 0);

  // open resource
  mjResource* resource = mju_openResource("str://file", 0);
  ASSERT_THAT(resource,  NotNull());

  const char* buffer = NULL;
  int bytes = mju_readResource(resource, (const void**) &buffer);
  EXPECT_EQ(bytes, std::strlen("Hello World") + 1);
  EXPECT_THAT(buffer, StrEq("Hello World"));

  mju_closeResource(resource);
}

TEST_F(ResourceTest, GeneralTestFailure) {
  mjpResourceProvider provider = {"str://", open_str, read_str, close_str,
                                  nullptr};

  // register resource provider
  int i = mjp_registerResourceProvider(&provider);
  EXPECT_GT(i, 0);


  // install warning handler
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
    util::strcpy_arr(warning, msg);
  };

  // open resource
  mjResource* resource = mju_openResource("str://notfound", 0);
  ASSERT_THAT(resource,  IsNull());

  EXPECT_THAT(warning, HasSubstr("could not open"));
}

}  // namespace
}  // namespace mujoco
