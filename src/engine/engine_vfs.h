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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_VFS_H_
#define MUJOCO_SRC_ENGINE_ENGINE_VFS_H_

#include <stddef.h>

#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>

#ifdef __cplusplus
extern "C" {
#endif

// initialize to empty (no deallocation)
MJAPI void mj_defaultVFS(mjVFS* vfs);

// add file to VFS, return 0: success, 1: full, 2: repeated name, -1: not found on disk
MJAPI int mj_addFileVFS(mjVFS* vfs, const char* directory, const char* filename);

// deprecated: use mj_copyBufferVFS
MJAPI int mj_makeEmptyFileVFS(mjVFS* vfs, const char* filename, int filesize);

// add file from buffer into VFS, return 0: success, 1: full, 2: repeated name, -1: failed to load
MJAPI int mj_copyBufferVFS(mjVFS* vfs, const char* filename, const void* buffer, int nbuffer);

// return file index in VFS, or -1 if not found in VFS
MJAPI int mj_findFileVFS(const mjVFS* vfs, const char* filename);

// delete file from VFS, return 0: success, -1: not found in VFS
MJAPI int mj_deleteFileVFS(mjVFS* vfs, const char* filename);

// delete all files from VFS
MJAPI void mj_deleteVFS(mjVFS* vfs);

// open VFS resource
MJAPI mjResource* mju_openVfsResource(const char* name, const mjVFS* vfs);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_VFS_H_
