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

#include "engine/engine_util_errmem.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
#include <unistd.h>
#endif

#include "engine/engine_array_safety.h"
#include "engine/engine_macro.h"

//------------------------- cross-platform aligned malloc/free -------------------------------------

static inline void* mju_alignedMalloc(size_t size, size_t align) {
#ifdef _WIN32
  return _aligned_malloc(size, align);
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
  return aligned_alloc(align, size);
#endif
}

static inline void mju_alignedFree(void* ptr) {
#ifdef _WIN32
  _aligned_free(ptr);
#else
  free(ptr);
#endif
}


//------------------------- default user handlers --------------------------------------------------

// define and clear handlers
void (*mju_user_error) (const char*) = 0;
void (*mju_user_warning) (const char*) = 0;
void* (*mju_user_malloc) (size_t) = 0;
void (*mju_user_free) (void*) = 0;


// restore default processing
void mju_clearHandlers(void) {
  mju_user_error = 0;
  mju_user_warning = 0;
  mju_user_malloc = 0;
  mju_user_free = 0;
}

//------------------------- internal-only handlers -------------------------------------------------

typedef void (*callback_fn)(const char*);

static mjTHREADLOCAL callback_fn _mjPRIVATE_tls_error_fn = NULL;
static mjTHREADLOCAL callback_fn _mjPRIVATE_tls_warning_fn = NULL;

callback_fn _mjPRIVATE__get_tls_error_fn(void) {
  return _mjPRIVATE_tls_error_fn;
}

void _mjPRIVATE__set_tls_error_fn(callback_fn h) {
  _mjPRIVATE_tls_error_fn = h;
}

callback_fn _mjPRIVATE__get_tls_warning_fn(void) {
  return _mjPRIVATE_tls_warning_fn;
}

void _mjPRIVATE__set_tls_warning_fn(callback_fn h) {
  _mjPRIVATE_tls_warning_fn = h;
}

//------------------------------ error hadling -----------------------------------------------------

// write datetime, type: message to MUJOCO_LOG.TXT
void mju_writeLog(const char* type, const char* msg) {
  time_t rawtime;
  struct tm timeinfo;
  FILE* fp = fopen("MUJOCO_LOG.TXT", "a+t");
  if (fp) {
    // get time
    time(&rawtime);

#if defined(_POSIX_C_SOURCE) || defined(__APPLE__) || defined(__STDC_VERSION_TIME_H__) || defined(__EMSCRIPTEN__) || defined(__linux__)
    localtime_r(&rawtime, &timeinfo);
#elif _MSC_VER
    localtime_s(&timeinfo, &rawtime);
#elif __STDC_LIB_EXT1__
    localtime_s(&rawtime, &timeinfo);
#else
    #error "Thread-safe version of `localtime` is not present in the standard C library"
#endif

    // write to log file
    fprintf(fp, "%s%s: %s\n\n", asctime(&timeinfo), type, msg);
    fclose(fp);
  }
}



void mju_error_raw(const char* msg) {
  if (_mjPRIVATE_tls_error_fn) {
    _mjPRIVATE_tls_error_fn(msg);
  } else if (mju_user_error) {
    mju_user_error(msg);
  } else {
    // write to log and console
    mju_writeLog("ERROR", msg);
    printf("ERROR: %s\n\n", msg);

    // exit
    exit(EXIT_FAILURE);
  }
}



void mju_error_v(const char* msg, va_list args) {
  // Format msg into errmsg
  char errmsg[1024];
  vsnprintf(errmsg, mjSIZEOFARRAY(errmsg), msg, args);
  mju_error_raw(errmsg);
}



// write message to logfile and console, pause and exit
void mju_error(const char* msg, ...) {
  va_list args;
  va_start(args, msg);
  mju_error_v(msg, args);
  va_end(args);
}



// write message to logfile and console
void mju_warning(const char* msg, ...) {
  char wrnmsg[1024];

  // Format msg into wrnmsg
  va_list args;
  va_start(args, msg);
  vsnprintf(wrnmsg, mjSIZEOFARRAY(wrnmsg), msg, args);
  va_end(args);

  if (_mjPRIVATE_tls_warning_fn) {
    _mjPRIVATE_tls_warning_fn(wrnmsg);
  } else if (mju_user_warning) {
    mju_user_warning(wrnmsg);
  } else {
    // write to log file and console
    mju_writeLog("WARNING", wrnmsg);
    printf("WARNING: %s\n\n", wrnmsg);
  }
}


// error with int argument
void mju_error_i(const char* msg, int i) {
  mju_error(msg, i);
}


// warning with int argument
void mju_warning_i(const char* msg, int i) {
  mju_warning(msg, i);
}


// error string argument
void mju_error_s(const char* msg, const char* text) {
  mju_error(msg, text);
}


// warning string argument
void mju_warning_s(const char* msg, const char* text) {
  mju_warning(msg, text);
}



//------------------------------ malloc and free ---------------------------------------------------

// allocate memory; byte-align on 64; pad size to multiple of 64
void* mju_malloc(size_t size) {
  void* ptr = 0;

  // user allocator
  if (mju_user_malloc) {
    ptr = mju_user_malloc(size);
  }

  // default allocator
  else {
    // pad size to multiple of 64
    if ((size%64)) {
      size += 64 - (size%64);
    }

    // allocate
    ptr = mju_alignedMalloc(size, 64);
  }

  // error if null pointer
  if (!ptr) {
    mju_error("Could not allocate memory");
  }

  return ptr;
}


// free memory
void mju_free(void* ptr) {
  // return if null
  if (!ptr) {
    return;
  }

  // free with user or built-in function
  if (mju_user_free) {
    mju_user_free(ptr);
  } else {
    mju_alignedFree(ptr);
  }
}
