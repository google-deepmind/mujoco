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

#ifndef MUJOCO_SRC_EXPERIMENTAL_USD_COLOR_UTILS_H_
#define MUJOCO_SRC_EXPERIMENTAL_USD_COLOR_UTILS_H_

#include <cmath>

namespace mujoco::usd {

// USD color attributes use linear RGB; MuJoCo rgba uses sRGB.
inline float LinearToSrgb(float value) {
  return value <= 0.0031308 ? 12.92 * value
                            : 1.055 * std::pow(value, 1.0 / 2.4) - 0.055;
}

inline float SrgbToLinear(float value) {
  return value <= 0.04045 ? value / 12.92
                          : std::pow((value + 0.055) / 1.055, 2.4);
}

}  // namespace mujoco::usd

#endif  // MUJOCO_SRC_EXPERIMENTAL_USD_COLOR_UTILS_H_
