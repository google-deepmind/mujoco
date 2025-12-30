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
#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HELPERS_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HELPERS_H_

#include <cstddef>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco::platform {

// Function signature for loading assets from a given path.
using LoadAssetFn = std::function<std::vector<std::byte>(std::string_view)>;

// Save/load for simple ascii files.
void SaveText(const std::string& contents, const std::string& filename);
std::string LoadText(const std::string& filename);

// Attempts to find a file with the given name by recursively searching the
// given search paths.
std::string ResolveFile(const std::string& filename,
                        const std::vector<std::string>& search_paths);

// Exports the given image (assumed to be RGB888) to a webp file.
void SaveToWebp(int width, int height, const std::byte* data,
                const std::string& filename);

// Returns a pointer to the value of the given field in the given data.
// Returns nullptr if the field is not found or the index is out of bounds.
const void* GetValue(const mjModel* model, const mjData* data,
                     const char* field, int index);

// Returns an XML string representation of the camera.
std::string CameraToString(const mjData* data, const mjvCamera* camera);

// Returns an XML string representation of current data keyframe.
std::string KeyframeToString(const mjModel* model, const mjData* data,
                             bool full_precision = false);

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HELPERS_H_
