// Copyright 2023 DeepMind Technologies Limited
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

#ifndef MUJOCO_MJMACRO_H_
#define MUJOCO_MJMACRO_H_

// include asan interface header, or provide stubs for poison/unpoison macros when not using asan
#ifdef ADDRESS_SANITIZER
  #include <sanitizer/asan_interface.h>
#elif defined(_MSC_VER)
  #define ASAN_POISON_MEMORY_REGION(addr, size)
  #define ASAN_UNPOISON_MEMORY_REGION(addr, size)
#else
  #define ASAN_POISON_MEMORY_REGION(addr, size) ((void)(addr), (void)(size))
  #define ASAN_UNPOISON_MEMORY_REGION(addr, size) ((void)(addr), (void)(size))
#endif

// max and min (use only for primitive types)
#define mjMAX(a, b) (((a) > (b)) ? (a) : (b))
#define mjMIN(a, b) (((a) < (b)) ? (a) : (b))

// mjData stack frame management
#define mjMARKSTACK   int _mark = d->pstack;
#define mjFREESTACK   d->pstack = _mark;

// return current value of mjOption enable/disable flags
#define mjDISABLED(x) (m->opt.disableflags & (x))
#define mjENABLED(x)  (m->opt.enableflags & (x))

// annotation for functions that accept printf-like variadic arguments
#ifndef mjPRINTFLIKE
  #if defined(__GNUC__)
    #define mjPRINTFLIKE(n, m) __attribute__((format(printf, n, m)))
  #else
    #define mjPRINTFLIKE(n, m)
  #endif
#endif

// implementation of mjFREESTACK when using the address sanitizer
#ifdef ADDRESS_SANITIZER
  #undef mjFREESTACK
  #define mjFREESTACK {                                          \
    d->pstack = _mark;                                           \
    ASAN_POISON_MEMORY_REGION(                                   \
        (char*)d->arena + d->parena,                             \
        (d->nstack - d->pstack) * sizeof(mjtNum) - d->parena );  \
  }
#endif

#endif  // MUJOCO_MJMACRO_H_
