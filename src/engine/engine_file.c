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

#include "engine/engine_file.h"

#include <stdio.h>
#include <limits.h>

#include "engine/engine_util_errmem.h"

void* mju_fileToMemory(const char* filename, int* filesize) {
  // open file
  *filesize = 0;
  FILE* fp = fopen(filename, "rb");
  if (!fp) {
    return NULL;
  }

  // find size
  if (fseek(fp, 0, SEEK_END) != 0) {
    fclose(fp);
    mju_warning_s("Failed to calculate size for '%s'", filename);
    return NULL;
  }

  // ensure file size fits in int
  long long_filesize = ftell(fp);  // NOLINT(runtime/int)
  if (long_filesize > INT_MAX) {
    fclose(fp);
    mju_warning_s("File size over 2GB is not supported. File: '%s'", filename);
    return NULL;
  } else if (long_filesize < 0) {
    fclose(fp);
    mju_warning_s("Failed to calculate size for '%s'", filename);
    return NULL;
  }
  *filesize = long_filesize;

  // go back to start of file
  if (fseek(fp, 0, SEEK_SET) != 0) {
    fclose(fp);
    mju_warning_s("Read error while reading '%s'", filename);
    return NULL;
  }

  // allocate and read
  void* buffer = mju_malloc(*filesize);
  if (!buffer) {
    mju_error("mjFileToMemory: could not allocate memory");
  }
  size_t bytes_read = fread(buffer, 1, *filesize, fp);

  // check that read data matches file size
  if (bytes_read != *filesize) {  // SHOULD NOT OCCUR
    if (ferror(fp)) {
      fclose(fp);
      mju_free(buffer);
      *filesize = 0;
      mju_warning_s("Read error while reading '%s'", filename);
      return NULL;
    } else if (feof(fp)) {
      *filesize = bytes_read;
    }
  }

  // close file, return contents
  fclose(fp);
  return buffer;
}
