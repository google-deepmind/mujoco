// Copyright 2021 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_ERRMEM_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_ERRMEM_H_

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

#include <mujoco/mjexport.h>
#include <mujoco/mjmacro.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef mjPRINTFLIKE
  #if defined(__GNUC__)
    #define mjPRINTFLIKE(n, m) __attribute__((format(printf, n, m)))
  #else
    #define mjPRINTFLIKE(n, m)
  #endif // __GNUC__
#endif // mjPRINTFLIKE


//------------------------------ user handlers -----------------------------------------------------

MJAPI extern void (*mju_user_error)(const char*);
MJAPI extern void (*mju_user_warning)(const char*);
MJAPI extern void* (*mju_user_malloc)(size_t);
MJAPI extern void (*mju_user_free)(void*);

// clear user handlers; restore default processing
MJAPI void mju_clearHandlers(void);

// gets/sets thread-local error/warning handlers for internal use
MJAPI void (*_mjPRIVATE__get_tls_error_fn(void))(const char*);
MJAPI void _mjPRIVATE__set_tls_error_fn(void (*h)(const char*));
MJAPI void (*_mjPRIVATE__get_tls_warning_fn(void))(const char*);
MJAPI void _mjPRIVATE__set_tls_warning_fn(void (*h)(const char*));

//------------------------------ errors and warnings -----------------------------------------------

// errors
MJAPI void mju_error(const char* msg, ...) mjPRINTFLIKE(1, 2);
MJAPI void mju_error_v(const char* msg, va_list args);
MJAPI void mju_error_i(const char* msg, int i);
MJAPI void mju_error_s(const char* msg, const char* text);

// warnings
MJAPI void mju_warning(const char* msg, ...) mjPRINTFLIKE(1, 2);
MJAPI void mju_warning_i(const char* msg, int i);
MJAPI void mju_warning_s(const char* msg, const char* text);

// write [datetime, type: message] to MUJOCO_LOG.TXT
MJAPI void mju_writeLog(const char* type, const char* msg);

//------------------------------ internal error macros --------------------------------------------

// need at least c99 or c++11
#if (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L) ||       \
    (defined(__cplusplus) && __cplusplus >= 201103L)

  // macro to get the first argument
  #define _GET_MSG(msg, ...) msg

  // helper function for the mjERROR macro
  // formats buf as '{prefix}: {msg}' and passes along to mju_error_v
  static inline void _mju_error_prefix(char *buf, size_t nbuf, const char* prefix,
                                       const char* msg, ...) mjPRINTFLIKE(4, 5);

  static inline void _mju_error_prefix(char *buf, size_t nbuf, const char* prefix,
                                       const char* msg, ...) {
    snprintf(buf, nbuf, "%s: %s", prefix, msg);
    va_list args;
    va_start(args, msg);
    mju_error_v(buf, args);
    va_end(args);
  }

  // macro to get first argument
  #define _GET_MSG(msg, ...) msg

  // internal macro to prepend the calling function name to the error message
  // standard support for variadic macros with zero arguments is only now
  // supported in C23 and C++20 so we rely on a helper function to get around this
  // in a portable way
  #define mjERROR(...) {                                                \
    char _buf[sizeof(_GET_MSG(__VA_ARGS__)) + sizeof(__func__) + 1];    \
    _mju_error_prefix(_buf, sizeof(_buf), __func__, __VA_ARGS__);       \
  }

#else
  #define mjERROR mju_error
#endif  // c99 or c++11

//------------------------------ malloc and free ---------------------------------------------------

// allocate memory; byte-align on 8; pad size to multiple of 8
MJAPI void* mju_malloc(size_t size);

// free memory with free() by default
MJAPI void mju_free(void* ptr);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_ERRMEM_H_
