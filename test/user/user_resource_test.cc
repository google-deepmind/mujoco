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

// Tests for user/user_resource.cc

#include <array>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "src/cc/array_safety.h"
#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_plugin.h"
#include "src/engine/engine_util_misc.h"
#include "src/user/user_resource.h"
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
  if (std::strcmp(resource->name, "str:file")) {
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
  mjpResourceProvider provider = {
    "my-prefix.123+45", open_nop, read_nop, close_nop
  };

  int count1 = mjp_resourceProviderCount();
  int i = mjp_registerResourceProvider(&provider);
  int count2 = mjp_resourceProviderCount();

  EXPECT_GT(i, 0);
  EXPECT_EQ(count2 - count1, 1);
}

TEST_F(ResourceTest, RegisterProviderMultipleSuccess) {
  mjpResourceProvider provider = {
    "my-prefix.123+44", open_nop, read_nop, close_nop
  };

  mjpResourceProvider provider2 = {
    "my-prefix.123+46", open_nop, read_nop, close_nop
  };


  mjpResourceProvider provider3 = {
    "my-prefix.123+41", open_nop, read_nop, close_nop
  };

  int count1 = mjp_resourceProviderCount();
  int i = mjp_registerResourceProvider(&provider);
  int i2 = mjp_registerResourceProvider(&provider2);
  int i3 = mjp_registerResourceProvider(&provider3);
  int count2 = mjp_resourceProviderCount();

  EXPECT_GT(i, 0);
  EXPECT_GT(i2, 0);
  EXPECT_GT(i3, 0);
  EXPECT_EQ(count2 - count1, 3);
}

TEST_F(ResourceTest, RegisterProviderMissingCallbacks) {
  mjpResourceProvider provider = {
    "myprefix"
  };

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
  mjpResourceProvider provider = {
    "", open_nop, read_nop, close_nop
  };

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

TEST_F(ResourceTest, RegisterProviderInvalidPrefix1) {
  mjpResourceProvider provider = {
    "1invalid", open_nop, read_nop, close_nop
  };

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

TEST_F(ResourceTest, RegisterProviderInvalidPrefix2) {
  mjpResourceProvider provider = {
    "invalid:", open_nop, read_nop, close_nop
  };

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

TEST_F(ResourceTest, RegisterProviderSame) {
  mjpResourceProvider provider = {
    "prefix", open_nop, read_nop, close_nop
  };

  mjpResourceProvider provider2 = {
    "prefix", open_nop, read_nop, close_nop
  };

  int i1 = mjp_registerResourceProvider(&provider);
  int count1 = mjp_resourceProviderCount();

  int i2 = mjp_registerResourceProvider(&provider2);
  int count2 = mjp_resourceProviderCount();

  EXPECT_EQ(i1, i2);
  EXPECT_EQ(count1, count2);
}

TEST_F(ResourceTest, RegisterProviderSameCase) {
  mjpResourceProvider provider = {
    "prefix", open_nop, read_nop, close_nop
  };

  mjpResourceProvider provider2 = {
    "PREFIX", open_nop, read_nop, close_nop
  };

  int i1 = mjp_registerResourceProvider(&provider);
  int count1 = mjp_resourceProviderCount();

  int i2 = mjp_registerResourceProvider(&provider2);
  int count2 = mjp_resourceProviderCount();

  EXPECT_EQ(i1, i2);
  EXPECT_EQ(count1, count2);
}

TEST_F(ResourceTest, GeneralTest) {
  mjpResourceProvider provider = {
    "str", open_str, read_str, close_str
  };

  // register resource provider
  int i = mjp_registerResourceProvider(&provider);
  EXPECT_GT(i, 0);

  // open resource
  mjResource* resource = mju_openResource("", "str:file", nullptr, nullptr, 0);
  ASSERT_THAT(resource,  NotNull());

  const char* buffer = NULL;
  int bytes = mju_readResource(resource, (const void**) &buffer);
  EXPECT_EQ(bytes, std::strlen("Hello World") + 1);
  EXPECT_THAT(buffer, StrEq("Hello World"));

  mju_closeResource(resource);
}

TEST_F(ResourceTest, GeneralFailureTest) {
  mjpResourceProvider provider = {
    "str", open_str, read_str, close_str
  };

  // register resource provider
  int i = mjp_registerResourceProvider(&provider);
  EXPECT_GT(i, 0);

  static std::array<char, 1024> error;

  // open resource
  mjResource* resource = mju_openResource("", "str:notfound", nullptr,
                                          error.data(), error.size());
  ASSERT_THAT(resource, IsNull());

  EXPECT_THAT(error.data(), HasSubstr("could not open"));
}

TEST_F(ResourceTest, NameWithValidPrefix) {
  mjpResourceProvider provider = {
    "nop", open_nop, read_nop, close_nop
  };

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
  mjResource* resource = mju_openResource("", "nop:found", nullptr, nullptr, 0);
  ASSERT_THAT(resource, NotNull());
  mju_closeResource(resource);
}

TEST_F(ResourceTest, NameWithUpperCasePrefix) {
  mjpResourceProvider provider = {
    "nop", open_nop, read_nop, close_nop,
  };

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
  mjResource* resource = mju_openResource("", "NOP:found", nullptr, nullptr, 0);
  ASSERT_THAT(resource, NotNull());
  mju_closeResource(resource);
}

TEST_F(ResourceTest, NameWithInvalidPrefix) {
  mjpResourceProvider provider = {
    "nop", open_nop, read_nop, close_nop
  };

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
  mjResource* resource = mju_openResource("", "nopfound", nullptr, nullptr, 0);
  ASSERT_THAT(resource, IsNull());
}

TEST_F(ResourceTest, OSFilesystemTimestamps) {
  time_t t;

  // some random file
  const char* const file = "engine/testdata/collision_box/boxbox_deep.xml";
  const std::string xml_path = GetTestDataFilePath(file);

  mjResource* resource = mju_openResource("", xml_path.c_str(), nullptr,
                                          nullptr, 0);
  mju_decodeBase64((uint8_t*) &t, resource->timestamp);

  // equal timestamps
  EXPECT_EQ(mju_isModifiedResource(resource, resource->timestamp), 0);

  std::array<char, 512> test_timestamp;

  // older resource timestamp
  t++;
  mju_encodeBase64(test_timestamp.data(), (uint8_t*) &t, sizeof(time_t));
  EXPECT_EQ(mju_isModifiedResource(resource, test_timestamp.data()), -1);


  // newer resource timestamp
  t -= 2;
  mju_encodeBase64(test_timestamp.data(), (uint8_t*) &t, sizeof(time_t));
  EXPECT_EQ(mju_isModifiedResource(resource, test_timestamp.data()), 1);

  mju_closeResource(resource);
}

}  // namespace
}  // namespace mujoco
