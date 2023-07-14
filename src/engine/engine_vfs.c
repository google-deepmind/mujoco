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

#include "engine/engine_vfs.h"

#include <string.h>

#include "engine/engine_array_safety.h"
#include "engine/engine_plugin.h"
#include "engine/engine_resource.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"

// strip path prefix from filename
static void vfs_strippath(char* newname, const char* oldname) {
  int sz = strlen(oldname);

  // find last delimiter
  int i;
  for (i=sz-1; i >= 0; i--) {
    if (oldname[i] == '\\' || oldname[i] == '/') {
      break;
    }
  }

  // check resulting length
  if (sz-(i+1) >= mjMAXVFSNAME) {
    mjERROR("filename too long");
  }
  if (sz-(i+1) <= 0) {
    mjERROR("empty filename");
  }

  // copy
  mju_strncpy(newname, oldname+i+1, mjMAXVFSNAME);

  // make lowercase
  for (int j=strlen(newname)-1; j >= 0; j--) {
    if (newname[j] >= 'A' && newname[j] <= 'Z') {
      newname[j] = (char)(((int)newname[j]) +'a' - 'A');
    }
  }
}



// initialize to empty (no deallocation)
void mj_defaultVFS(mjVFS* vfs) {
  memset(vfs, 0, sizeof(mjVFS));
}



// add file to VFS, return 0: success, 1: full, 2: repeated name, -1: failed to load
int mj_addFileVFS(mjVFS* vfs, const char* directory, const char* filename) {
  // check vfs size
  if (vfs->nfile >= mjMAXVFS-1) {
    return 1;
  }

  // make full name
  char fullname[1000];
  if (mju_makefullname(fullname, sizeof(fullname), directory, filename)) {
    return -1;
  }

  // strip path
  char newname[mjMAXVFSNAME];
  vfs_strippath(newname, filename);

  // check for repeated name
  for (int i=0; i < vfs->nfile; i++) {
    if (strncmp(newname, vfs->filename[i], mjMAXVFSNAME) == 0) {
      return 2;
    }
  }

  // assign name
  mjSTRNCPY(vfs->filename[vfs->nfile], newname);

  // allocate and read
  int filesize = 0;
  vfs->filedata[vfs->nfile] = mju_fileToMemory(fullname, &filesize);
  if (!vfs->filedata[vfs->nfile]) {
    return -1;
  }
  // assign size and count
  vfs->filesize[vfs->nfile] = filesize;
  vfs->nfile++;

  return 0;
}



// make empty file in VFS, return 0: success, 1: full, 2: repeated name
int mj_makeEmptyFileVFS(mjVFS* vfs, const char* filename, int filesize) {
  // check vfs size
  if (vfs->nfile >= mjMAXVFS-1) {
    return 1;
  }

  // check filesize
  if (filesize <= 0) {
    mjERROR("expects positive filesize");
  }

  // strip path
  char newname[mjMAXVFSNAME];
  vfs_strippath(newname, filename);

  // check for repeated name
  for (int i=0; i < vfs->nfile; i++) {
    if (strncmp(newname, vfs->filename[i], mjMAXVFSNAME) == 0) {
      return 2;
    }
  }

  // assign name
  mjSTRNCPY(vfs->filename[vfs->nfile], newname);

  // allocate and clear
  vfs->filedata[vfs->nfile] = mju_malloc(filesize);
  if (!vfs->filedata[vfs->nfile]) {
    mjERROR("could not allocate memory");
  }
  memset(vfs->filedata[vfs->nfile], 0, filesize);

  // assign size and count
  vfs->filesize[vfs->nfile] = filesize;
  vfs->nfile++;

  return 0;
}



// return file index in VFS, or -1 if not found in VFS
int mj_findFileVFS(const mjVFS* vfs, const char* filename) {
  // strip path
  char newname[mjMAXVFSNAME];
  vfs_strippath(newname, filename);
  // find specific file
  for (int i=0; i < vfs->nfile; i++) {
    if (strncmp(newname, vfs->filename[i], mjMAXVFSNAME) == 0) {
      return i;
    }
  }

  return -1;
}



// delete file from VFS, return 0: success, -1: not found in VFS
int mj_deleteFileVFS(mjVFS* vfs, const char* filename) {
  // strip path
  char newname[mjMAXVFSNAME];
  vfs_strippath(newname, filename);

  // find specified file
  for (int i=0; i < vfs->nfile; i++) {
    if (strncmp(newname, vfs->filename[i], mjMAXVFSNAME) == 0) {
      // free buffer
      mju_free(vfs->filedata[i]);

      // scroll remaining files forward
      for (int j=i; j < vfs->nfile-1; j++) {
        mjSTRNCPY(vfs->filename[j], vfs->filename[j+1]);
        vfs->filesize[j] = vfs->filesize[j+1];
        vfs->filedata[j] = vfs->filedata[j+1];
      }

      // set last to 0, for style
      vfs->filename[vfs->nfile-1][0] = 0;
      vfs->filesize[vfs->nfile-1] = 0;
      vfs->filedata[vfs->nfile-1] = NULL;

      // decrease counter
      vfs->nfile--;
      return 0;
    }
  }

  return -1;
}



// delete all files from VFS
void mj_deleteVFS(mjVFS* vfs) {
  for (int i=0; i < vfs->nfile; i++) {
    mju_free(vfs->filedata[i]);
  }

  memset(vfs, 0, sizeof(mjVFS));
}



// open callback for the VFS resource provider
static int vfs_open_callback(mjResource* resource) {
  if (!resource || !resource->provider_data || !resource->name) {
    return 0;
  }

  const mjVFS* vfs = (const mjVFS*) resource->provider_data;
  return mj_findFileVFS(vfs, resource->name) >= 0;
}



// read callback for the VFS resource provider
static int vfs_read_callback(mjResource* resource, const void** buffer) {
  if (!resource || !resource->provider_data) {
    *buffer = NULL;
    return -1;
  }

  const mjVFS* vfs = (const mjVFS*) resource->provider_data;
  int i = mj_findFileVFS(vfs, resource->name);
  if (i < 0) {
    *buffer = NULL;
    return -1;
  }

  *buffer = vfs->filedata[i];
  return vfs->filesize[i];
}



// close callback for the VFS resource provider
static void vfs_close_callback(mjResource* resource) {
}



// getdir callback for the VFS resource provider
static void vfs_getdir_callback(mjResource* resource, const char** dir, int* ndir) {
  if (resource) {
    *dir = resource->name;
    *ndir = mju_dirnamelen(resource->name);
  } else {
    *dir = NULL;
    *ndir = 0;
  }
}



// registers a VFS resource provider; returns the index of the provider
int mj_registerVfsProvider(const mjVFS* vfs) {
  mjpResourceProvider provider = {
    .prefix = mjVFS_PREFIX,
    .open = &vfs_open_callback,
    .read = &vfs_read_callback,
    .close = &vfs_close_callback,
    .getdir = &vfs_getdir_callback,
    .data = (void*) vfs
  };

  return mjp_registerResourceProviderInternal(&provider);
}
