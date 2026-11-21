# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

include(CheckCSourceCompiles)

# Assigns compiler options to the given variable based on availability of AVX.
function(get_avx_compile_options OUTPUT_VAR)
  # On RISC-V, the platform SIMD extension is RVV (the V extension) rather than AVX.
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(riscv64|RISCV64)$")
    get_rvv_compile_options(_RVV_OPTIONS)
    set("${OUTPUT_VAR}"
        ${_RVV_OPTIONS}
        PARENT_SCOPE
    )
    return()
  endif()

  message(VERBOSE "Checking if AVX is available...")

  if(MSVC)
    set(CMAKE_REQUIRED_FLAGS "/arch:AVX")
  elseif(WIN32)
    # Abseil LTS 20230125.0 assumes that AVX implies PCLMUL on Windows.
    set(CMAKE_REQUIRED_FLAGS "-mavx" "-mpclmul")
  else()
    set(CMAKE_REQUIRED_FLAGS "-mavx")
  endif()

  if(APPLE AND "x86_64" IN_LIST CMAKE_OSX_ARCHITECTURES)
    message(STATUS "Building x86_64 on macOS, forcing CAN_BUILD_AVX to TRUE.")
    set(CAN_BUILD_AVX TRUE)
  else()
    check_c_source_compiles(
      "
      #include <immintrin.h>
      int main(int argc, char* argv[]) {
        __m256d ymm;
        return 0;
      }
    "
      CAN_BUILD_AVX
    )
  endif()

  if(CAN_BUILD_AVX)
    message(VERBOSE "Checking if AVX is available... AVX available.")
    set("${OUTPUT_VAR}"
        ${CMAKE_REQUIRED_FLAGS}
        PARENT_SCOPE
    )
  else()
    message(VERBOSE "Checking if AVX is available... AVX not available.")
    set("${OUTPUT_VAR}" PARENT_SCOPE)
  endif()
endfunction()

# Assigns compiler options to the given variable for building RISC-V Vector (RVV 1.0) code.
# If the toolchain already targets V (e.g. an RVA23 default -march), no extra flags are needed.
# Otherwise, MUJOCO_RVV_MARCH (default rv64gcv) is tried. The double-precision code paths need
# the ratified intrinsics API (v0.12, GCC >= 14 / Clang >= 19) and 64-bit vector floats.
function(get_rvv_compile_options OUTPUT_VAR)
  set(MUJOCO_RVV_MARCH
      "rv64gcv"
      CACHE STRING "-march used to enable RVV when the compiler does not target V by default."
  )
  set(_RVV_TEST_SOURCE
      "
      #include <riscv_vector.h>
      #if !defined(__riscv_v_intrinsic) || __riscv_v_intrinsic < 12000
      #error RVV intrinsics v0.12 or later required
      #endif
      #if !defined(__riscv_v_elen_fp) || __riscv_v_elen_fp < 64
      #error 64-bit vector floating point required
      #endif
      int main(void) {
        double a[4] = {0};
        size_t vl = __riscv_vsetvl_e64m2(4);
        vfloat64m2_t v = __riscv_vle64_v_f64m2(a, vl);
        __riscv_vse64_v_f64m2(a, __riscv_vfadd_vv_f64m2(v, v, vl), vl);
        return (int)a[0];
      }
    "
  )

  message(VERBOSE "Checking if RVV is available...")
  check_c_source_compiles("${_RVV_TEST_SOURCE}" CAN_BUILD_RVV_DEFAULT)
  if(CAN_BUILD_RVV_DEFAULT)
    message(STATUS "RVV intrinsics available with the default -march.")
    set("${OUTPUT_VAR}" PARENT_SCOPE)
    return()
  endif()

  set(CMAKE_REQUIRED_FLAGS "-march=${MUJOCO_RVV_MARCH}")
  check_c_source_compiles("${_RVV_TEST_SOURCE}" CAN_BUILD_RVV_MARCH)
  if(CAN_BUILD_RVV_MARCH)
    message(STATUS "RVV intrinsics available with -march=${MUJOCO_RVV_MARCH}.")
    set("${OUTPUT_VAR}"
        "-march=${MUJOCO_RVV_MARCH}"
        PARENT_SCOPE
    )
  else()
    message(STATUS "RVV intrinsics not available.")
    set("${OUTPUT_VAR}" PARENT_SCOPE)
  endif()
endfunction()
