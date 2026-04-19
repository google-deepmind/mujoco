// Copyright 2026 DeepMind Technologies Limited
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

// Release-time helper: emit the auto-generated MJCF XSD to a file or stdout.
//
// Usage:
//   xmlschema                    # write raw XSD to stdout
//   xmlschema raw.xsd            # write raw XSD to file
//
// The emitted document is the "raw" schema (structure + types + enums only);
// release builds post-process it through doc/mjcf_schema_enrich.py to inject
// defaults and prose from doc/XMLreference.rst, producing doc/mjcf.xsd.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  const char* out = (argc >= 2) ? argv[1] : nullptr;

  if (!out) {
    // stdout path: ask for the size, allocate, print.
    const int n = mj_printSchemaXSD(nullptr, nullptr, 0) + 1;
    char* buf = static_cast<char*>(std::malloc(n));
    if (!buf) {
      std::fprintf(stderr, "out of memory\n");
      return EXIT_FAILURE;
    }
    mj_printSchemaXSD(nullptr, buf, n);
    std::fputs(buf, stdout);
    std::free(buf);
  } else {
    mj_printSchemaXSD(out, nullptr, 0);
    std::fprintf(stderr, "wrote %s\n", out);
  }
  return EXIT_SUCCESS;
}
