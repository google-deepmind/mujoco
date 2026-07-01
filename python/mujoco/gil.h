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

#ifndef MUJOCO_PYTHON_GIL_H_
#define MUJOCO_PYTHON_GIL_H_

#include <mutex>

namespace mujoco::python {

// A scoped lock that acquires a std::mutex in free-threaded Python builds
// (Py_GIL_DISABLED) and does nothing in standard GIL builds.
//
// In standard builds the GIL already serializes access to Python state, so
// adding a C++ mutex would introduce unnecessary overhead and deadlock risk
// (two locks held simultaneously). In free-threaded builds the GIL is absent,
// so an explicit mutex is required to protect shared mutable C++ state.
//
// Usage:
//   static std::mutex my_mutex;
//   {
//     MutexLockIfGilDisabled lock(my_mutex);
//     // ... access shared state ...
//   }
#ifdef Py_GIL_DISABLED
class MutexLockIfGilDisabled {
 public:
  explicit MutexLockIfGilDisabled(std::mutex& mtx) : lock_(mtx) {}
  MutexLockIfGilDisabled(const MutexLockIfGilDisabled&) = delete;
  MutexLockIfGilDisabled& operator=(const MutexLockIfGilDisabled&) = delete;

 private:
  std::lock_guard<std::mutex> lock_;
};
#else
class MutexLockIfGilDisabled {
 public:
  explicit MutexLockIfGilDisabled(std::mutex& /*unused*/) {}
  MutexLockIfGilDisabled(const MutexLockIfGilDisabled&) = delete;
  MutexLockIfGilDisabled& operator=(const MutexLockIfGilDisabled&) = delete;
};
#endif

inline std::mutex& GetCallbackMutex() {
  static std::mutex mtx;
  return mtx;
}

}  // namespace mujoco::python

#endif  // MUJOCO_PYTHON_GIL_H_
