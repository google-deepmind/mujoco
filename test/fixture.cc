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

#include "test/fixture.h"

#include <filesystem>
#include <fstream>
#include <memory>
#include <string_view>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/base/const_init.h>
#include <absl/strings/str_cat.h>
#include <absl/synchronization/mutex.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

namespace mujoco {
namespace {

ABSL_CONST_INIT static absl::Mutex handlers_mutex(absl::kConstInit);
static int guard_count ABSL_GUARDED_BY(handlers_mutex) = 0;

void default_mj_error_handler(const char* msg) {
  FAIL() << "mju_user_error: " << msg;
}

void default_mj_warning_handler(const char* msg) {
  ADD_FAILURE() << "mju_user_warning: " << msg;
}
}  // namespace

MujocoErrorTestGuard::MujocoErrorTestGuard() {
  absl::MutexLock lock(&handlers_mutex);
  if (++guard_count == 1) {
    mju_user_error = default_mj_error_handler;
    mju_user_warning = default_mj_warning_handler;
  }
}

MujocoErrorTestGuard::~MujocoErrorTestGuard() {
  absl::MutexLock lock(&handlers_mutex);
  if (--guard_count == 0) {
    mju_user_error = nullptr;
    mju_user_warning = nullptr;
  }
}

const std::string GetTestDataFilePath(std::string_view path) {
  return std::string(path);
}

const std::string GetModelPath(std::string_view path) {
  return absl::StrCat("../model/", path);
}

mjModel* LoadModelFromString(std::string_view xml, char* error,
                             int error_size) {
  static constexpr char file[] = "filename.xml";
  // mjVFS structs need to be allocated on the heap, because it's ~2MB
  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_makeEmptyFileVFS(vfs.get(), file, xml.size());
  int file_idx = mj_findFileVFS(vfs.get(), file);
  memcpy(vfs->filedata[file_idx], xml.data(), xml.size());
  mjModel* m = mj_loadXML(file, vfs.get(), error, error_size);
  mj_deleteFileVFS(vfs.get(), file);
  return m;
}

const std::string GetFileContents(const char* path) {
  std::ifstream ifs;
  ifs.open(path, std::ifstream::in);
  EXPECT_FALSE(ifs.fail());
  std::ostringstream sstream;
  sstream << ifs.rdbuf();
  return sstream.str();
}

const std::string SaveAndReadXml(const mjModel* model) {
  EXPECT_THAT(model, testing::NotNull());

  constexpr int kMaxPathLen = 1024;
  std::string path_template =
      std::filesystem::temp_directory_path().append("tmp.XXXXXX").string();
  EXPECT_LT(path_template.size(), kMaxPathLen);

  char filepath[kMaxPathLen];
  mju_strncpy(filepath, path_template.c_str(), path_template.size() + 1);

#if defined(_POSIX_VERSION) && _POSIX_VERSION >= 200112L
  int fd = mkstemp(filepath);
  EXPECT_NE(fd, -1) << std::strerror(errno);
#elif defined(_WIN32)
  EXPECT_NE(_mktemp_s(filepath), EINVAL);
#endif

  mj_saveLastXML(filepath, model, nullptr, 0);
  std::string contents = GetFileContents(filepath);

#if defined(_POSIX_VERSION) && _POSIX_VERSION >= 200112L
  close(fd);
#endif
  std::remove(filepath);

  return contents;
}

}  // namespace mujoco
