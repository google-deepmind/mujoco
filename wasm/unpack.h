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

#ifndef MUJOCO_WASM_UNPACK_H_
#define MUJOCO_WASM_UNPACK_H_

#ifdef __EMSCRIPTEN__

#include <emscripten/val.h>

#include <cinttypes>  // NOLINT required for PRId64
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "engine/engine_util_errmem.h"

namespace mujoco::wasm {

// Helper to strip "_wrapper" from function names.
std::string StripWrapperSuffix(const char* func);

// Utility class to write/read from the Heap shared by C++ and Javascript
template <typename T>
class WasmBuffer {
 private:
  // Note: Embind does not support binding more than one constructor with the
  // same argument count so we bind the factory function fromArray instead
  explicit WasmBuffer(const emscripten::val& array) {
    std::vector<T> cpp_array = convertJSArrayToNumberVector<T>(array);
    bytes_.resize(cpp_array.size() * sizeof(T));
    if (cpp_array.size() > 0) {
      memcpy(bytes_.data(), cpp_array.data(), bytes_.size());
    }
  }

 public:
  // Creates a buffer with the given element count
  explicit WasmBuffer(int element_count = 0) {
    bytes_.resize(element_count * sizeof(T));
  }

  // Creates a buffer by copying data from a (typed) array
  static WasmBuffer<T> FromArray(const emscripten::val& array) {
    return WasmBuffer<T>(array);
  }

  // Returns the pointer to the data in the buffer
  uintptr_t GetPointer() { return reinterpret_cast<uintptr_t>(bytes_.data()); }

  // Returns the number of elements in the buffer
  int GetElementCount() { return bytes_.size() / sizeof(T); }

  // Returns a TypedArray view of the buffer
  emscripten::val GetView() {
    return emscripten::val(emscripten::typed_memory_view(
        bytes_.size() / sizeof(T), reinterpret_cast<const T*>(bytes_.data())));
  }

  void Zero() {
    if (!bytes_.empty()) {
      memset(bytes_.data(), 0, bytes_.size());
    }
  }

 private:
  std::vector<std::byte> bytes_;
};

template <typename T>
class UnpackedParam {
  // The C++ representation of the parameter data
  std::variant<std::monostate, std::vector<T>, std::span<T>> data_;

  // Printable representations of the param and function name used for errors
  const char* repr_;
  const char* func_;

  explicit UnpackedParam(const char* repr, const char* func)
      : data_(std::monostate{}), repr_(repr), func_(func) {}

  UnpackedParam(std::vector<T>&& array, const char* repr, const char* func)
      : data_(std::move(array)), repr_(repr), func_(func) {}

  UnpackedParam(T* data, std::size_t count, const char* repr, const char* func)
      : data_(std::span<T>(data, count)), repr_(repr), func_(func) {}

  // Returns true and raises an error if the val is null or undefined.
  static bool ErrorOnNullOrUndefined(const emscripten::val& p,
                                     const char* func,
                                    const char* expected_type) {
    if (p.isUndefined()) {
      mju_error("[%s] Invalid argument. Expected a %s, got undefined.",
                StripWrapperSuffix(func).c_str(), expected_type);
      return true;
    } else if (p.isNull()) {
      mju_error("[%s] Invalid argument. Expected a %s, got null.",
                StripWrapperSuffix(func).c_str(), expected_type);
      return true;
    }
    return false;
  }

  // Returns true if the val is null or undefined. Use when these are expected.
  static bool IsNullOrUndefined(const emscripten::val& p) {
    return p.isUndefined() || p.isNull();
  }

 public:
  // Create from a nullable Javascript val. Call via UNPACK_NULLABLE_VALUE.
  static UnpackedParam<T> FromNullableValue(const emscripten::val& p,
                                            const char* repr,
                                            const char* func) {
    if (IsNullOrUndefined(p)) {
      return UnpackedParam<T>(repr, func);
    }
    return FromValue(p, repr, func);
  }

  // Create from a nullable Javascript number[]. Call via UNPACK_NULLABLE_ARRAY.
  static UnpackedParam<T> FromNullableArray(const emscripten::val& p,
                                            const char* repr,
                                            const char* func) {
    if (IsNullOrUndefined(p)) {
      return UnpackedParam<T>(repr, func);
    }
    return UnpackedParam<T>(convertJSArrayToNumberVector<T>(p), repr, func);
  }

  // Create from a Javascript number[]. Call via UNPACK_ARRAY.
  static UnpackedParam<T> FromArray(const emscripten::val& p, const char* repr,
                                    const char* func) {
    ErrorOnNullOrUndefined(p, func, "number[]");
    return UnpackedParam<T>(convertJSArrayToNumberVector<T>(p), repr, func);
  }

  // Creates an UnpackedParam from a Javascript a TypedArray or a WasmBuffer.
  // Call via UNPACK_VALUE.
  static UnpackedParam<T> FromValue(const emscripten::val& p, const char* repr,
                                    const char* func) {
    ErrorOnNullOrUndefined(p, func, "TypedArray or WasmBuffer");

    if (!p["byteOffset"].isUndefined()) {  // Javascript TypedArray
      T* data = reinterpret_cast<T*>(p["byteOffset"].as<uintptr_t>());
      std::size_t count = p["length"].as<std::size_t>();
      return UnpackedParam<T>(data, count, repr, func);
    } else if (!p["GetPointer"].isUndefined()) {  // C++ WasmBuffer
      WasmBuffer<T>& buffer = p.as<WasmBuffer<T>&>();
      T* data = reinterpret_cast<T*>(buffer.GetPointer());
      std::size_t count = buffer.GetElementCount();
      return UnpackedParam<T>(data, count, repr, func);
    }

    // TODO(manevi): This error message is not 100% accurate, WasmBuffer class
    // isn't surfaced to JS developers
    auto param = UnpackedParam<T>(repr, func);
    mju_error(
        "[%s] Invalid argument. Expected TypedArray or WasmBuffer, got "
        "unknown type for %s.",
        param.func().c_str(), param.repr());
    return param;
  }

  // Returns true if the parameter is not null. Used in if conditions.
  explicit operator bool() const {
    return !std::holds_alternative<std::monostate>(data_);
  }

  // Returns the printable representation of the parameter for use in error
  // messages.
  const char* repr() const { return repr_; }

  // Returns the name of the function the parameter is used in.
  std::string func() const { return StripWrapperSuffix(func_); }

  // Returns the size of the parameter. Returns 0 if the parameter is null.
  std::size_t size() const {
    if (std::holds_alternative<std::vector<T>>(data_)) {
      return std::get<std::vector<T>>(data_).size();
    } else if (std::holds_alternative<std::span<T>>(data_)) {
      return std::get<std::span<T>>(data_).size();
    }
    mju_error("[%s] [%s] UnpackedParam is null", func().c_str(), repr());
    return 0;
  }

  // Returns a pointer to the data of the parameter. Returns nullptr if the
  // parameter is null.
  const T* data() const {
    if (std::holds_alternative<std::vector<T>>(data_)) {
      return std::get<std::vector<T>>(data_).data();
    } else if (std::holds_alternative<std::span<T>>(data_)) {
      return std::get<std::span<T>>(data_).data();
    }
    return nullptr;
  }

  // Returns a non-const pointer to the data of the parameter. Returns nullptr
  // if the parameter is null.
  T* data() {
    if (std::holds_alternative<std::vector<T>>(data_)) {
      return std::get<std::vector<T>>(data_).data();
    } else if (std::holds_alternative<std::span<T>>(data_)) {
      return const_cast<T*>(std::get<std::span<T>>(data_).data());
    }
    return nullptr;
  }
};

// TODO(matijak): When the bindings are fully auto-generated we could replace
// these macros with a function calls something like this:
//
//   template <typename T, typename U>
//   UnpackedParam<T> Unpack(U&& u, const char* u_name,
//     const std::source_location location = std::source_location::current()) {
//     return UnpackedParam<T>::FromValue(std::forward<U>(u), u_name,
//       location.file_name(), location.line(), location.function_name());
//   }

#define UNPACK_VALUE(T, p) \
  UnpackedParam<T> p##_ = UnpackedParam<T>::FromValue(p, #p, __func__)

#define UNPACK_ARRAY(T, p) \
  UnpackedParam<T> p##_ = UnpackedParam<T>::FromArray(p, #p, __func__)

#define UNPACK_NULLABLE_VALUE(T, p) \
  UnpackedParam<T> p##_ = UnpackedParam<T>::FromNullableValue(p, #p, __func__)

#define UNPACK_NULLABLE_ARRAY(T, p) \
  UnpackedParam<T> p##_ = UnpackedParam<T>::FromNullableArray(p, #p, __func__)

// Raises an error if x##_.size() is not equal to expr.
// Assumes UnpackedParam x##_ is defined.
#define CHECK_SIZE(x, expr)                                                   \
  if (x##_) {                                                                 \
    if (static_cast<int64_t>(x##_.size()) != static_cast<int64_t>(expr)) {    \
      mju_error("[%s] %s must have size %" PRId64 ", got %" PRId64,           \
                x##_.func().c_str(), x##_.repr(), static_cast<int64_t>(expr), \
                static_cast<int64_t>(x##_.size()));                           \
    }                                                                         \
  }

// Raises an error if x##_.size() is not equal to y##_.size().
// Assumes UnpackedParams x##_ and y##_ are defined.
#define CHECK_SIZES(x, y)                                           \
  if (x##_ && y##_) {                                               \
    if (static_cast<int64_t>(x##_.size()) !=                        \
        static_cast<int64_t>(y##_.size())) {                        \
      mju_error("[%s] %s and %s must have equal size, got %" PRId64 \
                " and %" PRId64,                                    \
                x##_.func().c_str(), x##_.repr(), y##_.repr(),      \
                static_cast<int64_t>(x##_.size()),                  \
                static_cast<int64_t>(y##_.size()));                 \
    }                                                               \
  }

// Raises an error if x##_.size() is not a perfect square.
// Assumes UnpackedParam x##_ is defined. Defines x##_sqrt as an int.
#define CHECK_PERFECT_SQUARE(x)                                    \
  const int x##_sqrt = static_cast<int>(round(sqrt(x##_.size()))); \
  if (x##_sqrt * x##_sqrt != x##_.size()) {                        \
    mjERROR("[%s] %s must be a perfect square, got %" PRId64,      \
            x##_.func().c_str(), x##_.repr(),                      \
            static_cast<int64_t>(x##_.size()));                    \
  }

// Raises an error if x##_.size() is not divisible by divisor.
// Assumes UnpackedParam x##_ is defined. Defines x##_div as an std::div_t.
#define CHECK_DIVISIBLE(x, divisor)                                        \
  const std::div_t x##_div =                                               \
      std::div(static_cast<int>(x##_.size()), static_cast<int>(divisor));  \
  if (x##_div.rem != 0) {                                                  \
    mju_error("[%s] %s must be divisible by %d, got quot=%d rem=%d",       \
              x##_.func().c_str(), x##_.repr(), static_cast<int>(divisor), \
              x##_div.quot, x##_div.rem);                                  \
  }

}  // namespace mujoco::wasm

#endif  // __EMSCRIPTEN__

#endif  // MUJOCO_WASM_UNPACK_H_
