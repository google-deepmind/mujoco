// Copyright 2024 DeepMind Technologies Limited
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

#include "private.h"
#include "raw.h"
#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

namespace py = ::pybind11;

namespace mujoco::python {

template <typename T>
class MjsElementRef {
 public:
  MjsElementRef() : ptr_(nullptr), element_(nullptr) {}
  explicit MjsElementRef(T* ptr) : ptr_(ptr), element_(GetElement(ptr)) {
    if (element_) {
      _mjPRIVATE_addRefElement(element_);
    }
  }
  MjsElementRef(const MjsElementRef& other)
      : ptr_(other.ptr_), element_(other.element_) {
    if (element_) {
      _mjPRIVATE_addRefElement(element_);
    }
  }
  MjsElementRef(MjsElementRef&& other) noexcept
      : ptr_(other.ptr_), element_(other.element_) {
    other.ptr_ = nullptr;
    other.element_ = nullptr;
  }
  MjsElementRef& operator=(const MjsElementRef& other) {
    if (this != &other) {
      if (other.element_) {
        _mjPRIVATE_addRefElement(other.element_);
      }
      if (element_) {
        _mjPRIVATE_releaseElement(element_);
      }
      ptr_ = other.ptr_;
      element_ = other.element_;
    }
    return *this;
  }
  MjsElementRef& operator=(MjsElementRef&& other) noexcept {
    if (this != &other) {
      if (element_) {
        _mjPRIVATE_releaseElement(element_);
      }
      ptr_ = other.ptr_;
      element_ = other.element_;
      other.ptr_ = nullptr;
      other.element_ = nullptr;
    }
    return *this;
  }
  ~MjsElementRef() {
    if (element_) {
      _mjPRIVATE_releaseElement(element_);
    }
  }
  T* get() const { return ptr_; }

 private:
  static raw::MjsElement* GetElement(raw::MjsElement* ptr) { return ptr; }
  template <typename U>
  static raw::MjsElement* GetElement(U* ptr) {
    return ptr ? ptr->element : nullptr;
  }

  T* ptr_;
  raw::MjsElement* element_;
};

struct MjSpec {
  MjSpec();
  MjSpec(raw::MjSpec* ptr, const py::dict& assets_ = {});

  // copy constructor and assignment
  MjSpec(const MjSpec& other);
  MjSpec& operator=(const MjSpec& other);

  // move constructor and move assignment
  MjSpec(MjSpec&& other);
  MjSpec& operator=(MjSpec&& other);
  ~MjSpec();

  raw::MjModel* Compile(mjVFS* vfs = nullptr);

  raw::MjSpec* ptr;
  py::dict assets;
  bool override_assets = true;
  MjSpec* parent = nullptr;
};
}  // namespace mujoco::python

PYBIND11_DECLARE_HOLDER_TYPE(T, mujoco::python::MjsElementRef<T>, true);
