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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_MODEL_HOLDER_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_MODEL_HOLDER_H_

#include <memory>
#include <cstddef>
#include <span>
#include <string_view>

#include <mujoco/mujoco.h>

namespace mujoco::platform {

// Container for storing an mjModel as well as its associated mjData. Also, if
// applicable, stores the mjSpec and mjVFS from which the model was constructed.
class ModelHolder {
 public:
  ModelHolder(const ModelHolder&) = delete;
  ModelHolder& operator=(const ModelHolder&) = delete;
  ~ModelHolder();

  // Creates an mjModel from an existing mjSpec. This class takes ownership of
  // the spec and will `mj_deleteSpec` it on destruction.
  static std::unique_ptr<ModelHolder> FromSpec(mjSpec* spec);

  // Creates an mjModel by attempting to open and parse the give file.
  static std::unique_ptr<ModelHolder> FromFile(std::string_view filepath);

  // Creates an mjModel by attempting to decode the given buffer.
  static std::unique_ptr<ModelHolder> FromBuffer(
      std::span<const std::byte> buffer, std::string_view content_type,
      std::string_view filename);

  // Accessors to the MuJoCo structures managed by this object.
  mjVFS* vfs() { return &vfs_; }
  mjSpec* spec() { return spec_; }
  mjData* data() { return data_; }
  mjModel* model() { return model_; }

  // Returns true if the holder holds a valid model (and data).
  bool ok() const { return error_[0] == 0 && model_ && data_; }

  // Returns the error message if the model failed to load.
  std::string_view error() const { return error_; }

 private:
  ModelHolder() = default;
  void InitFromSpec(mjSpec* spec);
  void InitFromFile(std::string_view filepath);
  void InitFromBuffer(std::span<const std::byte> buffer,
                      std::string_view content_type,
                      std::string_view filename);
  void PostInit();
  void SetLoadError(std::string_view error);

  mjVFS vfs_;
  mjSpec* spec_ = nullptr;
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  char error_[1000] = "";
};
}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_MODEL_HOLDER_H_
