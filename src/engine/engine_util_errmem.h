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
#include <string.h>

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
  #endif  // __GNUC__
#endif  // mjPRINTFLIKE


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
MJAPI void mju_error_raw(const char* msg);
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

// internal macro to prepend the calling function name to the error message
#pragma warning(disable : 4996)  // needed to use strncpy with Visual Studio
#define mjERROR(...)                                                          \
{                                                                             \
  char _errbuf[1024];                                                         \
  size_t _funclen = strlen(__func__);                                         \
  strncpy(_errbuf, __func__, sizeof(_errbuf));                                \
  snprintf(_errbuf + _funclen, sizeof(_errbuf) - _funclen, ": " __VA_ARGS__); \
  mju_error_raw(_errbuf);                                                     \
}

//------------------------------ malloc and free ---------------------------------------------------

// allocate memory; byte-align on 8; pad size to multiple of 8
MJAPI void* mju_malloc(size_t size);

// free memory with free() by default
MJAPI void mju_free(void* ptr);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_ERRMEM_H_
