// Copyright 2022 DeepMind Technologies Limited
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

#ifndef MUJOCO_PYTHON_UTIL_CROSSPLATFORM_H_
#define MUJOCO_PYTHON_UTIL_CROSSPLATFORM_H_

#ifdef __has_attribute
#define MUJOCO_HAS_ATTRIBUTE(x) __has_attribute(x)
#else
#define MUJOCO_HAS_ATTRIBUTE(x) 0
#endif

#if MUJOCO_HAS_ATTRIBUTE(always_inline) || \
    (defined(__GNUC__) && !defined(__clang__))
#define MUJOCO_ALWAYS_INLINE __attribute__((always_inline))
#define MUJOCO_ALWAYS_INLINE_LAMBDA MUJOCO_ALWAYS_INLINE
#if defined(__clang__)
#define MUJOCO_ALWAYS_INLINE_LAMBDA_MUTABLE MUJOCO_ALWAYS_INLINE_LAMBDA mutable
#else
#define MUJOCO_ALWAYS_INLINE_LAMBDA_MUTABLE mutable MUJOCO_ALWAYS_INLINE_LAMBDA
#endif
#elif defined(_MSC_VER)
#define MUJOCO_ALWAYS_INLINE __forceinline
#if _MSC_VER >= 1927 && _MSVC_LANG >= 202002L
#define MUJOCO_ALWAYS_INLINE_LAMBDA [[msvc::forceinline]]
#endif
#define MUJOCO_ALWAYS_INLINE_LAMBDA_MUTABLE mutable MUJOCO_ALWAYS_INLINE_LAMBDA
#else
#define MUJOCO_ALWAYS_INLINE
#endif

#ifndef MUJOCO_ALWAYS_INLINE_LAMBDA
#define MUJOCO_ALWAYS_INLINE_LAMBDA
#endif

#ifndef MUJOCO_ALWAYS_INLINE_LAMBDA_MUTABLE
#define MUJOCO_ALWAYS_INLINE_LAMBDA_MUTABLE
#endif

#ifndef MUJOCO_DIAG_IGNORE_UNUSED_LAMBDA_CAPTURE
#if defined(__clang__)
#define MUJOCO_DIAG_IGNORE_UNUSED_LAMBDA_CAPTURE                    \
    _Pragma("clang diagnostic push")                                \
    _Pragma("clang diagnostic ignored \"-Wunused-lambda-capture\"")
#define MUJOCO_DIAG_UNIGNORE_UNUSED_LAMBDA_CAPTURE \
    _Pragma("clang diagnostic pop")
#else
#define MUJOCO_DIAG_IGNORE_UNUSED_LAMBDA_CAPTURE
#define MUJOCO_DIAG_UNIGNORE_UNUSED_LAMBDA_CAPTURE
#endif
#endif

// GCC ignores [[maybe_unused]] and emits a -Wattributes
#if defined(__GNUC__) && !defined(__clang__)
#define MUJOCO_MAYBE_UNUSED
#else
#define MUJOCO_MAYBE_UNUSED [[maybe_unused]]
#endif

#endif  // MUJOCO_PYTHON_UTIL_CROSSPLATFORM_H_
