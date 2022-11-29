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

#if defined(__APPLE__) && defined(__AVX__)

#include <stdio.h>
#include <string.h>
#include <sys/sysctl.h>

__attribute__((weak, visibility("default"))) void _mj_rosettaError(const char* msg) {
  fprintf(stderr, "%s\n", msg);
  __asm__ __volatile__ ("ud2");  // raises SIGILL but leave this function at the top of the stack
}

__attribute__((constructor(10000), target("no-avx"))) static void _mj_checkRosetta() {
  int is_translated = 0;
  {
    size_t len = sizeof(is_translated);
    if (sysctlbyname("sysctl.proc_translated", &is_translated, &len, NULL, 0)) {
      is_translated = 0;
    }
  }
  if (is_translated) {
    _mj_rosettaError("MuJoCo cannot be run under Rosetta 2 on an Apple Silicon machine.");
  }
}

#endif  // defined(__APPLE__) && defined(__AVX__)
