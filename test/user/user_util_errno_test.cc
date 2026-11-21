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

// Tests for user/user_util.cc with allocations that set errno on success, as
// C11 7.5p3 permits. This replaces the global operator new, hence its own
// binary.

#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <new>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "src/user/user_util.h"
#include "test/fixture.h"

namespace {

// when true, successful allocations set errno
bool alloc_sets_errno = false;

void* Alloc(std::size_t size) {
  void* ptr = std::malloc(size ? size : 1);
  if (ptr && alloc_sets_errno) errno = ENOMEM;
  return ptr;
}

}  // namespace

// replace every form paired with operator delete(void*), since sanitizer
// runtimes define the nothrow forms themselves; array and aligned forms keep
// their default pairing
void* operator new(std::size_t size) {
  void* ptr = Alloc(size);
  if (!ptr) std::abort();
  return ptr;
}

void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  return Alloc(size);
}

void operator delete(void* ptr) noexcept { std::free(ptr); }

void operator delete(void* ptr, std::size_t) noexcept { std::free(ptr); }

void operator delete(void* ptr, const std::nothrow_t&) noexcept {
  std::free(ptr);
}

namespace mujoco {
namespace {

using ::testing::ElementsAre;
using user::StringToVector;

using UserUtilErrnoTest = MujocoTest;

// parse s into v while allocations set errno, return errno after the call
int ParseErrno(const std::string& s, std::vector<double>* v) {
  alloc_sets_errno = true;
  *v = StringToVector<double>(s);
  int err = errno;
  alloc_sets_errno = false;
  return err;
}

TEST_F(UserUtilErrnoTest, StringToVectorEmpty) {
  std::vector<double> v;
  EXPECT_EQ(ParseErrno("", &v), 0);
  EXPECT_THAT(v, ElementsAre());
}

TEST_F(UserUtilErrnoTest, StringToVectorSpaces) {
  std::vector<double> v;
  EXPECT_EQ(ParseErrno("   ", &v), 0);
  EXPECT_THAT(v, ElementsAre());
}

TEST_F(UserUtilErrnoTest, StringToVectorTrailingSpaces) {
  std::vector<double> v;
  EXPECT_EQ(ParseErrno(" 1.5 2 3     ", &v), 0);
  EXPECT_THAT(v, ElementsAre(1.5, 2, 3));
}

TEST_F(UserUtilErrnoTest, StringToVectorNan) {
  MockWarningHandler warning_handler;
  warning_handler.ExpectWarnings();
  std::vector<double> v;
  EXPECT_EQ(ParseErrno("1 nan 2", &v), EDOM);
  EXPECT_EQ(v.size(), 3);
}

TEST_F(UserUtilErrnoTest, StringToVectorInvalidNumber) {
  std::vector<double> v;
  EXPECT_EQ(ParseErrno("1 0.1.2.3", &v), EINVAL);
  EXPECT_THAT(v, ElementsAre(1));
}

}  // namespace
}  // namespace mujoco
