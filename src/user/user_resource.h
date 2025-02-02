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
// IWYU pragma: private, include "third_party/mujoco/include/mujoco.h"
// IWYU pragma: friend "third_party/mujoco/src/.*"

#ifndef MUJOCO_SRC_ENGINE_ENGINE_RESOURCE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_RESOURCE_H_

#include <cstddef>

#include <mujoco/mjexport.h>
#include <mujoco/mujoco.h>

#ifdef __cplusplus
extern "C" {
#endif

// open the given resource; if the name doesn't have a prefix matching with a
// resource provider, then the OS filesystem is used
MJAPI mjResource* mju_openResource(const char* dir, const char* name,
                                   const mjVFS* vfs, char* error, size_t nerror);

// close the given resource; no-op if resource is NULL
MJAPI void mju_closeResource(mjResource* resource);

// set buffer to bytes read from the resource and return number of bytes in buffer;
// return negative value if error
MJAPI int mju_readResource(mjResource* resource, const void** buffer);

// set for a resource with a name partitioned as {dir}{filename}, the dir and ndir pointers
MJAPI void mju_getResourceDir(mjResource* resource, const char** dir, int* ndir);

// return 0 if the resource's timestamp matches the provided timestamp
// return > 0 if the resource is younger than the given timestamp
// return < 0 if the resource is older than the given timestamp
MJAPI int mju_isModifiedResource(const mjResource* resource, const char* timestamp);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_RESOURCE_H_
