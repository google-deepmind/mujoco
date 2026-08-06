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
#include <cstdio>   // IWYU pragma: keep
#include <cstdlib>  // IWYU pragma: keep
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>  // IWYU pragma: keep
#include <string>
#include <string_view>
#include <vector>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

extern "C" {
MJAPI mjfLogHandler _mjPRIVATE_setTlsLogHandler(mjfLogHandler handler);
}

namespace mujoco {

struct MjModelDeleter {
  void operator()(mjModel* m) const {
    mj_deleteModel(m);
  }
};
using MjModelPtr = std::unique_ptr<mjModel, MjModelDeleter>;

struct MjDataDeleter {
  void operator()(mjData* d) const {
    mj_deleteData(d);
  }
};
using MjDataPtr = std::unique_ptr<mjData, MjDataDeleter>;

// Runtime scale factor for test tolerances, controlled by MJTOL_SCALE env var.
// Set MJTOL_SCALE=0 to run tests with zero tolerance and see actual residuals.
inline mjtNum MjTolScale() {
  static const mjtNum scale = []() {
    const char* env = std::getenv("MJTOL_SCALE");
    return env ? std::strtod(env, nullptr) : 1.0;
  }();
  return scale;
}

// Precision-aware GMock matcher. Use instead of DoubleNear/FloatNear.
// Under double builds, uses double_tol. Under float builds, uses float_tol.
// Scaled by MJTOL_SCALE env var (default 1.0).
template <typename T1, typename T2>
inline auto MjNear(T1 double_tol, T2 float_tol) {
#ifdef mjUSESINGLE
  return ::testing::FloatNear(static_cast<float>(float_tol) * MjTolScale());
#else
  return ::testing::DoubleNear(static_cast<double>(double_tol) * MjTolScale());
#endif
}

// Precision-aware GMock matcher (3-arg version).
// Under double builds, matches near target with double_tol.
// Under float builds, matches near target with float_tol.
// Scaled by MJTOL_SCALE env var (default 1.0).
template <typename T1, typename T2, typename T3>
inline auto MjNear(T1 target, T2 double_tol, T3 float_tol) {
#ifdef mjUSESINGLE
  return ::testing::FloatNear(static_cast<float>(target),
                              static_cast<float>(float_tol) * MjTolScale());
#else
  return ::testing::DoubleNear(static_cast<double>(target),
                               static_cast<double>(double_tol) * MjTolScale());
#endif
}
// Precision-aware tolerance for EXPECT_NEAR.
// Scaled by MJTOL_SCALE env var (default 1.0).
template <typename T1, typename T2>
inline mjtNum MjTol(T1 double_tol, T2 float_tol) {
#ifdef mjUSESINGLE
  return static_cast<mjtNum>(float_tol) * MjTolScale();
#else
  return static_cast<mjtNum>(double_tol) * MjTolScale();
#endif
}

// Precision-aware equality assertion: 4 ULPs in either precision.
#ifdef mjUSESINGLE
#define EXPECT_MJTNUM_EQ(a, b) EXPECT_FLOAT_EQ(a, b)
#else
#define EXPECT_MJTNUM_EQ(a, b) EXPECT_DOUBLE_EQ(a, b)
#endif

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

// Mock handler for capturing and verifying mju_warning logs.
class MockWarningHandler {
 public:
  // Constructor that registers this handler as the active one.
  MockWarningHandler();
  // Destructor that restores the previously active handler.
  ~MockWarningHandler();

  // Mock method called when a warning is intercepted.
  MOCK_METHOD(void, Warn, (const std::string& msg));

  // Allow any number of warnings (if empty) or expect at least one warning
  // containing the specified substring (if non-empty).
  void ExpectWarnings(std::string_view substring = "");

  // Returns the thread-local active mock warning handler.
  static MockWarningHandler* GetActive();

 private:
  static thread_local MockWarningHandler* active_handler;
  MockWarningHandler* prev_ = nullptr;
};

// A test fixture which simplifies writing tests for the MuJoCo C API.
// By default, any MuJoCo operation which triggers a warning or error will
// trigger a test failure.
class MujocoTest : public ::testing::Test {
 public:
  MujocoTest() {
    static std::once_flag flag;
    std::call_once(flag, []() {
      const char* plugin_dir = std::getenv("MUJOCO_PLUGIN_DIR");
      if (plugin_dir) {
        mj_loadAllPluginLibraries(
          plugin_dir, +[](const char* filename, int first, int count) {
            std::printf("Plugins registered by library '%s':\n", filename);
            for (int i = first; i < first + count; ++i) {
              std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
            }
          });
      }
    });
  }
  ~MujocoTest() { mj_freeLastXML(); }

 protected:
  MockWarningHandler mock_warning_handler;

 private:
  MujocoErrorTestGuard error_guard;
};

template <typename Return, typename... Args>
auto MjuErrorMessageFrom(Return (*func)(Args...)) {
  thread_local std::jmp_buf current_jmp_buf;
  thread_local char err_msg[2048];

  auto new_error_handler = +[](const mjLogMessage* msg) -> void {
    if (msg->level != mjLOG_ERROR) return;
    std::snprintf(err_msg, sizeof(err_msg), "%s", msg->subject);
    std::longjmp(current_jmp_buf, 1);
  };

  return [func, new_error_handler](Args... args) -> std::string {
    auto old_handler = _mjPRIVATE_setTlsLogHandler(new_error_handler);
    if (setjmp(current_jmp_buf) == 0) {
      err_msg[0] = '\0';
      func(args...);
    }

    _mjPRIVATE_setTlsLogHandler(old_handler);
    return err_msg;
  };
}

// Returns a path to a data file, under the mujoco/test directory.
const std::string GetTestDataFilePath(std::string_view path);

// Returns a path to a data file, under the mujoco/model directory.
const std::string GetModelPath(std::string_view path);

// Returns a newly-allocated mjModel, loaded from the contents of xml.
// On failure returns nullptr and populates the error array if present.
MjModelPtr LoadModelFromString(std::string_view xml, char* error = nullptr,
                             int error_size = 0, mjVFS* vfs = nullptr);

// Returns a newly-allocated mjData, initialized using model.
MjDataPtr MakeData(const MjModelPtr& model);

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
      mjtNum val = mat[c + r * ncol];
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

}  // namespace mujoco
#endif  // MUJOCO_TEST_FIXTURE_H_
