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

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>

#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/base/const_init.h>
#include <absl/strings/str_cat.h>
#include <absl/strings/str_join.h>
#include <absl/synchronization/mutex.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

namespace mujoco {
namespace {

using ::testing::NotNull;

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
                             int error_size, mjVFS* vfs) {
  // register string resource provider if not registered before
  if (mjp_getResourceProvider("LoadModelFromString:") == nullptr) {
    mjpResourceProvider resourceProvider;
    mjp_defaultResourceProvider(&resourceProvider);
    resourceProvider.prefix = "LoadModelFromString";
    resourceProvider.open = +[](mjResource* resource) {
      resource->data = &(resource->name[strlen("LoadModelFromString:")]);
      return 1;
    };
    resourceProvider.read =
        +[](mjResource* resource, const void** buffer) {
          *buffer = resource->data;
          return (int) strlen((const char*) resource->data);
        };
    resourceProvider.close = +[](mjResource* resource) {};
    mjp_registerResourceProvider(&resourceProvider);
  }
  std::string xml2 = {xml.begin(), xml.end()};
  std::string str = "LoadModelFromString:" +  xml2;
  return mj_loadXML(str.c_str(), vfs, error, error_size);
}

static void AssertModelNotNull(mjModel* model,
                                  const std::array<char, 1024>& error) {
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
}

mjModel* LoadModelFromPath(const char* model_path) {
  const std::string xml_path = GetModelPath(model_path);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(
    xml_path.c_str(), nullptr, error.data(), error.size());
  AssertModelNotNull(model, error);
  return model;
}

std::string GetFileContents(const char* path) {
  std::ifstream ifs;
  ifs.open(path, std::ifstream::in);
  EXPECT_FALSE(ifs.fail());
  std::ostringstream sstream;
  sstream << ifs.rdbuf();
  return sstream.str();
}

std::string SaveAndReadXml(const mjModel* model) {
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

std::vector<mjtNum> GetCtrlNoise(const mjModel* m, int nsteps,
                                 mjtNum ctrlnoise) {
  std::vector<mjtNum> ctrl;
  for (int step=0; step < nsteps; step++) {
    for (int i = 0; i < m->nu; i++) {
      mjtNum center = 0.0;
      mjtNum radius = 1.0;
      mjtNum* range = m->actuator_ctrlrange + 2 * i;
      if (m->actuator_ctrllimited[i]) {
        center = (range[1] + range[0]) / 2;
        radius = (range[1] - range[0]) / 2;
      }
      radius *= ctrlnoise;
      ctrl.push_back(center + radius * (2 * mju_Halton(step, i+2) - 1));
    }
  }
  return ctrl;
}

MockFilesystem::MockFilesystem(std::string unit_test_name) {
  prefix_ = absl::StrCat("MjMock.", unit_test_name);
  dir_ = "/";
  if (mjp_getResourceProvider(prefix_.c_str()) != nullptr) {
    return;
  }

  mjpResourceProvider resourceProvider;
  mjp_defaultResourceProvider(&resourceProvider);
  resourceProvider.prefix = prefix_.c_str();
  resourceProvider.data = (void *) this;

  resourceProvider.open = +[](mjResource* resource) {
    MockFilesystem *fs = static_cast<MockFilesystem*>(resource->provider->data);
    std::string filename = fs->StripPrefix(resource->name);
    return fs->FileExists(filename) ? 1 : 0;
  };

  resourceProvider.read =+[](mjResource* resource, const void** buffer) {
    MockFilesystem *fs = static_cast<MockFilesystem*>(resource->provider->data);
    std::string filename = fs->StripPrefix(resource->name);
    return (int) fs->GetFile(filename, (const unsigned char**) buffer);
  };

  resourceProvider.getdir = +[](mjResource* resource, const char** dir,
                                int* ndir) {
    MockFilesystem *fs = static_cast<MockFilesystem*>(resource->provider->data);
    *dir = resource->name;

    // find last directory path separator
    int length = fs->Prefix().size() + 1;
    for (int i = length; resource->name[i]; ++i) {
     if (resource->name[i] == '/' || resource->name[i] == '\\') {
        length = i + 1;
      }
    }
    *ndir = length;
  };

  resourceProvider.close = +[](mjResource* resource) {};
  mjp_registerResourceProvider(&resourceProvider);
}

bool MockFilesystem::AddFile(std::string filename, const unsigned char* data,
                         std::size_t ndata) {
  std::string fullfilename = PathReduce(dir_, filename);
  auto [it, inserted] = filenames_.insert(fullfilename);
  if (inserted) {
    data_[fullfilename] = std::vector(data, data + ndata);
  }
  return inserted;
}

bool MockFilesystem::FileExists(const std::string& filename) {
  std::string fullfilename = PathReduce(dir_, filename);
  return filenames_.find(fullfilename) != filenames_.end();
}

std::size_t MockFilesystem::GetFile(const std::string& filename,
                                const unsigned char** buffer) const {
  std::string fullfilename = PathReduce(dir_, filename);
  auto it = data_.find(fullfilename);
  if (it == data_.end()) {
    return 0;
  }
  *buffer = it->second.data();
  return it->second.size();
}

void MockFilesystem::ChangeDirectory(std::string dir) {
  if (dir.empty()) {
    return;
  }

  dir_ = PathReduce(dir_, dir);
  if (dir_.back() != '/') {
    dir_ = absl::StrCat(dir_, "/");
  }
}

std::string MockFilesystem::FullPath(const std::string& path) const {
  return absl::StrCat(prefix_, ":", PathReduce(dir_, path));
}

std::string MockFilesystem::StripPrefix(const char* path) const {
  return &path[prefix_.size() + 1];
}

std::string MockFilesystem::PathReduce(const std::string& current_dir,
                                       const std::string& path) {
  std::stringstream stream;
  if (!path.empty() && path[0] != '/') {
    stream = std::stringstream(absl::StrCat(current_dir, path));
  } else {
    stream = std::stringstream(path);
  }

  std::string temp;
  std::vector<std::string> dirs;
  while (std::getline(stream, temp, '/')) {
    if (temp == ".." && !dirs.empty()) {
      dirs.pop_back();
      continue;
    }

    if (temp != "." && !temp.empty()) {
      dirs.push_back(temp);
    }
  }
  if (dirs.empty()) {
    return "/";
  }

  return absl::StrJoin(dirs, "/");
}

}  // namespace mujoco
