#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <algorithm>

#include "mujoco/mujoco.h"
#include "unpack.h"

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
