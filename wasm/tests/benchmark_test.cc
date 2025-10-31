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

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <algorithm>

#include <mujoco/mujoco.h>
#include "wasm/unpack.h"

namespace mujoco::wasm {
using emscripten::val;

EMSCRIPTEN_DECLARE_VAL_TYPE(NumberArray);

// TODO(matijak): Add a benchmark where the buffer shared with JS and C++ is
// created in C++ rather than JS.

val BenchmarkSortNumberArray(const NumberArray& vec) {
  UNPACK_ARRAY(mjtNum, vec);
  std::sort(vec_.data(), vec_.data() + vec_.size());
  return val::array(vec_.data(), vec_.data() + vec_.size());
}

val BenchmarkSortDoubleBuffer(const val& vec) {
  UNPACK_VALUE(mjtNum, vec);
  std::sort(vec_.data(), vec_.data() + vec_.size());
  return vec;
}

EMSCRIPTEN_BINDINGS(mujoco_benchmark_functions) {
  emscripten::class_<WasmBuffer<mjtNum>>("DoubleBuffer")
      .constructor<int>()
      .class_function("FromArray", &WasmBuffer<mjtNum>::FromArray)
      .function("GetPointer", &WasmBuffer<mjtNum>::GetPointer)
      .function("GetElementCount", &WasmBuffer<mjtNum>::GetElementCount)
      .function("GetView", &WasmBuffer<mjtNum>::GetView);
  emscripten::function("SortNumberArray", &BenchmarkSortNumberArray);
  emscripten::function("SortDoubleBuffer", &BenchmarkSortDoubleBuffer);
  emscripten::register_type<NumberArray>("number[]");
}

}  // namespace mujoco::wasm
