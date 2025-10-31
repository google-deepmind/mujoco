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

#include "wasm/unpack.h"

#include <cstddef>
#include <cstring>
#include <string>

namespace mujoco::wasm {

std::string StripWrapperSuffix(const char* func) {
  const char* suffix = "_wrapper";
  size_t name_len = strlen(func);
  size_t suffix_len = strlen(suffix);

  if (name_len >= suffix_len &&
      strcmp(func + name_len - suffix_len, suffix) == 0) {
    return std::string(func, name_len - suffix_len);
  }
  return std::string(func);
}

}  // namespace mujoco::wasm
