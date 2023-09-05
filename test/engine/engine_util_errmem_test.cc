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

// Tests for engine/engine_util_errmem.c.

#include <cstring>
#include <string>

#include <gtest/gtest.h>
#include "src/engine/engine_util_errmem.h"

namespace mujoco {
namespace {

constexpr int kBufferSize = 1024;

char* ErrorMessageBuffer() {
  static char error_message[kBufferSize] = "";
  return error_message;
}

char* WarningMessageBuffer() {
  static char warning_message[kBufferSize] = "";
  return warning_message;
}

void MjErrorHandler(const char* msg) {
  if (strnlen(msg, kBufferSize) == kBufferSize) {
    FAIL() << "mju_user_error message exceeds maximum length of "
           << kBufferSize;
  }
  strncpy(ErrorMessageBuffer(), msg, kBufferSize);
}

void MjWarningHandler(const char* msg) {
  if (strnlen(msg, kBufferSize) == kBufferSize) {
    FAIL() << "mju_user_warning message exceeds maximum length of "
           << kBufferSize;
  }
  strncpy(WarningMessageBuffer(), msg, kBufferSize);
}

void ClearErrorMessage() { ErrorMessageBuffer()[0] = '\0'; }

void ClearWarningMessage() { WarningMessageBuffer()[0] = '\0'; }

class MujocoErrorAndWarningTest : public ::testing::Test {
 public:
  MujocoErrorAndWarningTest() {
    mju_user_error = MjErrorHandler;
    mju_user_warning = MjWarningHandler;
  }

  ~MujocoErrorAndWarningTest() {
    mju_user_error = nullptr;
    mju_user_warning = nullptr;
  }
};

TEST_F(MujocoErrorAndWarningTest, MjuErrorI) {
  std::string format_string = "%010d";
  while (format_string.length() < 2 * kBufferSize) {
    format_string += 'x';
  }

  std::string expected_message = "0123456789";
  while (expected_message.length() < kBufferSize - 1) {
    expected_message += 'x';
  }

  ClearErrorMessage();
  mju_error_i(format_string.c_str(), 123456789);
  EXPECT_EQ(std::string(ErrorMessageBuffer()), expected_message);
}

TEST_F(MujocoErrorAndWarningTest, MjuWarningI) {
  std::string format_string = "%010d";
  while (format_string.length() < 2 * kBufferSize) {
    format_string += 'x';
  }

  std::string expected_message = "0123456789";
  while (expected_message.length() < kBufferSize - 1) {
    expected_message += 'x';
  }

  ClearWarningMessage();
  mju_warning_i(format_string.c_str(), 123456789);
  EXPECT_EQ(std::string(WarningMessageBuffer()), expected_message);
}

TEST_F(MujocoErrorAndWarningTest, MjuErrorS) {
  std::string format_string = "% 9s";
  while (format_string.length() < 2 * kBufferSize) {
    format_string += 'z';
  }

  std::string expected_message = "   foobar";
  while (expected_message.length() < kBufferSize - 1) {
    expected_message += 'z';
  }

  ClearErrorMessage();
  mju_error_s(format_string.c_str(), "foobar");
  EXPECT_EQ(std::string(ErrorMessageBuffer()), expected_message);
}

TEST_F(MujocoErrorAndWarningTest, MjuWarningS) {
  std::string format_string = "% 9s";
  while (format_string.length() < 2 * kBufferSize) {
    format_string += 'z';
  }

  std::string expected_message = "   foobar";
  while (expected_message.length() < kBufferSize - 1) {
    expected_message += 'z';
  }

  ClearWarningMessage();
  mju_warning_s(format_string.c_str(), "foobar");
  EXPECT_EQ(std::string(WarningMessageBuffer()), expected_message);
}

TEST_F(MujocoErrorAndWarningTest, MjuErrorInternal) {
  ClearErrorMessage();
  mjERROR("foobar %d", 123);
  std::string funcname(__func__);
  ASSERT_TRUE(funcname.length());
  EXPECT_EQ(std::string(ErrorMessageBuffer()), funcname + ": foobar 123");
}

}  // namespace
}  // namespace mujoco
