// Copyright 2025 DeepMind Technologies Limited
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

// Standalone functions used by Simulate.
#ifndef MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_HELPERS_H_
#define MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_HELPERS_H_

#include <cstddef>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <mujoco/mjrender.h>
#include <mujoco/mujoco.h>

namespace mujoco::toolbox {

// Function signature for loading assets from a given path.
using LoadAssetFn = std::function<std::vector<std::byte>(std::string_view)>;

// Save/load for simple ascii files.
void SaveText(const std::string& contents, const std::string& filename);
std::string LoadText(const std::string& filename);

// Exports the given color buffer to a webp file.
void SaveColorToWebp(int width, int height, const unsigned char* data,
                     const std::string& filename);

// Exports the given depth buffer to a webp file.
void SaveDepthToWebp(int width, int height, const float* data,
                     const std::string& filename);

// Exports the current state of the mjrContext to a webp file.
void SaveScreenshotToWebp(int width, int height, mjrContext* con,
                          const std::string& filename);

// Loads a MuJoCo model from the given file.
mjModel* LoadMujocoModel(const std::string& model_file, const mjVFS* vfs);

// Returns a pointer to the value of the given field in the given data.
// Returns nullptr if the field is not found or the index is out of bounds.
const void* GetValue(const mjModel* model, const mjData* data,
                     const char* field, int index);

// Returns an XML string representation of the camera.
std::string CameraToString(const mjData* data, const mjvCamera* camera);

// Returns an XML string representation of current data keyframe.
std::string KeyframeToString(const mjModel* model, const mjData* data,
                             bool full_precision = false);

}  // namespace mujoco::toolbox

#endif  // MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_HELPERS_H_
