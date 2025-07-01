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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_PRINT_H_
#define MUJOCO_SRC_ENGINE_ENGINE_PRINT_H_

#include <stdio.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// print mjModel to text file, specifying format
// float_format must be a valid printf-style format string for a single float value
MJAPI void mj_printFormattedModel(const mjModel* m, const char* filename,
                                  const char* float_format);

// print model and option to text file
MJAPI void mj_printModel(const mjModel* m, const char* filename);


// print mjData to text file, specifying format
// float_format must be a valid printf-style format string for a single float value
MJAPI void mj_printFormattedData(const mjModel* m, const mjData* d, const char* filename,
                                 const char* float_format);

// print data to text file
MJAPI void mj_printData(const mjModel* m, const mjData* d, const char* filename);

// print sparse matrix structure
MJAPI void mj_printSparsity(const char* str, int nr, int nc, const int* rowadr, const int* diag,
                            const int* rownnz, const int* rowsuper, const int* colind, FILE* fp);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_PRINT_H_
