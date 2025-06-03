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

#include "third_party/mujoco/src/experimental/usd/utils.h"

#include <mujoco/mujoco.h>
#include <pxr/usd/sdf/path.h>

namespace mujoco {
namespace usd {

constexpr const char* kUsdPrimPathKey = "usd_primpath";

void SetUsdPrimPathUserValue(mjsElement* element,
                             const pxr::SdfPath& prim_path) {
  // The value is a pointer to a newly allocated SdfPath, which will be deleted
  // when the mjsElement is deleted.
  const pxr::SdfPath* usd_primpath = new pxr::SdfPath(prim_path);
  mjs_setUserValueWithCleanup(
      element, kUsdPrimPathKey, usd_primpath,
      [](const void* data) { delete static_cast<const pxr::SdfPath*>(data); });
}

pxr::SdfPath GetUsdPrimPathUserValue(mjsElement* element) {
  const void* user_data = mjs_getUserValue(element, kUsdPrimPathKey);
  if (user_data) {
    return *static_cast<const pxr::SdfPath*>(user_data);
  }
  return pxr::SdfPath();
}

}  // namespace usd
}  // namespace mujoco
