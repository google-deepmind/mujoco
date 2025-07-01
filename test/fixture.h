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

#ifndef MUJOCO_TEST_FIXTURE_H_
#define MUJOCO_TEST_FIXTURE_H_

#include <csetjmp>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>
#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

extern "C" {
MJAPI void _mjPRIVATE__set_tls_error_fn(decltype(mju_user_error));
MJAPI decltype(mju_user_error)  _mjPRIVATE__get_tls_error_fn();
}

namespace mujoco {

// Installs and uninstalls error callbacks on MuJoCo that fail the currently
// running test if triggered. Prefer the use of MujocoTest, unless using a
// test fixture is not possible.
class MujocoErrorTestGuard {
 public:
  // Sets up error and warning callbacks on MuJoCo that will fail the test if
  // triggered.
  MujocoErrorTestGuard();

  // Clears up the callbacks from the constructor.
  ~MujocoErrorTestGuard();
};

// A test fixture which simplifies writing tests for the MuJoCo C API.
// By default, any MuJoCo operation which triggers a warning or error will
// trigger a test failure.
class MujocoTest : public ::testing::Test {
 public:
  ~MujocoTest() { mj_freeLastXML(); }

 private:
  MujocoErrorTestGuard error_guard;
};

template <typename Return, typename... Args>
auto MjuErrorMessageFrom(Return (*func)(Args...)) {
  thread_local std::jmp_buf current_jmp_buf;
  thread_local char err_msg[1000];

  auto* old_error_handler = _mjPRIVATE__get_tls_error_fn();
  auto* new_error_handler = +[](const char* msg) -> void {
    std::strncpy(err_msg, msg, sizeof(err_msg));
    std::longjmp(current_jmp_buf, 1);
  };

  return [func, old_error_handler,
          new_error_handler](Args... args) -> std::string {
    if (setjmp(current_jmp_buf) == 0) {
      err_msg[0] = '\0';
      _mjPRIVATE__set_tls_error_fn(new_error_handler);
      func(args...);
    }

    _mjPRIVATE__set_tls_error_fn(old_error_handler);
    return err_msg;
  };
}

// Returns a path to a data file, under the mujoco/test directory.
const std::string GetTestDataFilePath(std::string_view path);

// Returns a path to a data file, under the mujoco/model directory.
const std::string GetModelPath(std::string_view path);

// Returns a newly-allocated mjModel, loaded from the contents of xml.
// On failure returns nullptr and populates the error array if present.
mjModel* LoadModelFromString(std::string_view xml, char* error = nullptr,
                             int error_size = 0, mjVFS* vfs = nullptr);

// Returns a newly-allocated mjModel, loaded from the contents in model_path.
// On failure it asserts that model is null.
mjModel* LoadModelFromPath(const char* model_path);

// Returns a string loaded from first saving the model given an input.
std::string SaveAndReadXml(const mjModel* model);

// Returns a string loaded from first saving the spec given an input.
std::string SaveAndReadXml(const mjSpec* spec);

// Adds control noise.
std::vector<mjtNum> GetCtrlNoise(const mjModel* m, int nsteps,
                                 mjtNum ctrlnoise = 0.01);

// Compares all fields of two mjModels.
// Returns the name of the different field and the max difference.
mjtNum CompareModel(const mjModel* m1, const mjModel* m2, std::string& field);

// Returns a vector containing the elements of the array.
template <typename T>
std::vector<T> AsVector(const T* array, int n) {
  return std::vector<T>(array, array + n);
}

// Prints a matrix to stderr, useful for debugging.
inline void PrintMatrix(const mjtNum* mat, int nrow, int ncol, int p = 5,
                        std::string_view name = "") {
  std::cerr.precision(p);
  std::cerr << name << "\n";
  for (int r = 0; r < nrow; r++) {
    for (int c = 0; c < ncol; c++) {
      mjtNum val = mat[c + r*ncol];
      if (val) {
        std::cerr << std::fixed << std::setw(5 + p) << val << " ";
      } else {
        // don't print exact zeros
        std::cerr << std::string(6 + p, ' ');
      }
    }
    std::cerr << "\n";
  }
}

// Installs a mock filesystem via a resource provider. To obtain thread safety,
// each filesystem is scoped for individual unit tests with destructive
// operations not permitted.
class MockFilesystem {
 public:
  // constructs mock filesystem.  A unique name (normally the unit test name)
  // should be passed in.
  MockFilesystem(std::string unit_test_name);

  // Move and copy operations are forbidden.
  MockFilesystem(MockFilesystem&& other) = delete;
  MockFilesystem& operator=(MockFilesystem&& other) = delete;
  MockFilesystem(const MockFilesystem& other) = delete;
  MockFilesystem& operator=(const MockFilesystem& other) = delete;

  // Returns the prefix registered for the resource provider.
  const std::string& Prefix() const { return prefix_; }

  // Adds file to the current directory. Returns false if file already exists.
  bool AddFile(std::string filename, const unsigned char* data,
               std::size_t ndata);

  // Returns true if mock filesystem has file.
  bool FileExists(const std::string& filename);

  // Change the current directory.
  void ChangeDirectory(std::string dir);

  // Helper functions for resource provider callbacks.
  std::size_t GetFile(const std::string& filename,
                      const unsigned char** buffer) const;
  std::string FullPath(const std::string& path) const;


 private:
  std::string StripPrefix(const char* path) const;
  static std::string PathReduce(const std::string& current_dir,
                                const std::string& path);

  absl::flat_hash_set<std::string> filenames_;
  absl::flat_hash_map<std::string, std::vector<unsigned char>> data_;
  std::string prefix_;
  std::string dir_;  // current directory
};

// Installs all plugins
class PluginTest : public MujocoTest {
 public:
  // load plugin library
  PluginTest() : MujocoTest() {
    mj_loadAllPluginLibraries(
      std::string(std::getenv("MUJOCO_PLUGIN_DIR")).c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
  }
};

}  // namespace mujoco
#endif  // MUJOCO_TEST_FIXTURE_H_
