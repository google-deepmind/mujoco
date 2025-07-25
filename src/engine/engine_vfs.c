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

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "engine/engine_array_safety.h"
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



// copies data into a buffer and produces a hash of the data
static uint64_t vfs_memcpy(void* dest, const void* restrict src, size_t n) {
  uint64_t hash = 0xcbf29ce484222325;  // magic number
  uint64_t prime = 0x100000001b3;      // magic prime
  const uint8_t* bytes = (uint8_t*) src;
  uint8_t* buffer = (uint8_t*) dest;
  for (size_t i = 0; i < n; i++) {
    buffer[i] = bytes[i];

    // do FNV-1 hash
    hash |= bytes[i];
    hash *= prime;
  }
  return hash;
}



// VFS hash function implemented using the FNV-1 hash
static uint64_t vfs_hash(const void* restrict buffer, size_t n) {
  uint64_t hash = 0xcbf29ce484222325;  // magic number
  uint64_t prime = 0x100000001b3;      // magic prime
  const uint8_t* bytes = (uint8_t*) buffer;
  for (size_t i = 0; i < n; i++) {
    hash |= bytes[i];
    hash *= prime;
  }
  return hash;
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
  size_t filesize = 0;
  vfs->filedata[vfs->nfile] = mju_fileToMemory(fullname, &filesize);
  if (!vfs->filedata[vfs->nfile]) {
    return -1;
  }

  // assign size, count, and checksum
  vfs->filestamp[vfs->nfile] = vfs_hash(vfs->filedata[vfs->nfile], filesize);
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
  vfs->filestamp[vfs->nfile] = 0;

  // assign size and count
  vfs->filesize[vfs->nfile] = filesize;
  vfs->nfile++;

  return 0;
}



// add file from buffer into VFS
int mj_addBufferVFS(mjVFS* vfs, const char* name, const void* buffer, int nbuffer) {
  if (!vfs || !buffer || !name) {
    mjERROR("null pointer");
  }

  if (vfs->nfile >= mjMAXVFS-1) {
    return 1;
  }

  // check buffer size
  if (nbuffer <= 0) {
    mjERROR("expects positive buffer size");
  }

  // strip path
  char newname[mjMAXVFSNAME];
  vfs_strippath(newname, name);

  // check for repeated name
  for (int i=0; i < vfs->nfile; i++) {
    if (strncmp(newname, vfs->filename[i], mjMAXVFSNAME) == 0) {
      return 2;
    }
  }

  // assign name
  mjSTRNCPY(vfs->filename[vfs->nfile], newname);

  // allocate and clear
  vfs->filedata[vfs->nfile] = mju_malloc(nbuffer);
  if (!vfs->filedata[vfs->nfile]) {
    mjERROR("could not allocate memory");
  }
  vfs->filestamp[vfs->nfile] = vfs_memcpy(vfs->filedata[vfs->nfile], buffer, nbuffer);

  // assign size and count
  vfs->filesize[vfs->nfile] = nbuffer;
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
  if (!resource || !resource->name || !resource->data) {
    return 0;
  }

  const mjVFS* vfs = (const mjVFS*) resource->data;
  int i = mj_findFileVFS(vfs, resource->name);
  resource->timestamp[0] = '\0';
  if (i >= 0 && vfs->filestamp[i]) {
    mju_encodeBase64(resource->timestamp, (uint8_t*) &vfs->filestamp[i],
                     sizeof(uint64_t));
  }
  return i >= 0;
}



// read callback for the VFS resource provider
static int vfs_read_callback(mjResource* resource, const void** buffer) {
  if (!resource || !resource->name || !resource->data) {
    *buffer = NULL;
    return -1;
  }

  const mjVFS* vfs = (const mjVFS*) resource->data;
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


// modified callback for the VFS resource provider
// return > 0 if modified and 0 if unmodified
static int vfs_modified_callback(const mjResource* resource, const char* timestamp) {
  uint64_t filestamp;
  if (mju_isValidBase64(timestamp) > sizeof(uint64_t)) {
    return 2;  // error (assume modified)
  }

  mju_decodeBase64((uint8_t*) &filestamp, timestamp);
  if (!filestamp) return 3;  // no hash (assume modified)

  if (resource) {
    const mjVFS* vfs = (const mjVFS*) resource->data;
    int i = mj_findFileVFS(vfs, resource->name);
    if (i < 0) return 4;  // missing file (assume modified)
    if (!vfs->filestamp[i]) return 5;  // missing filestamp (assume modified)

    if (vfs->filestamp[i] == filestamp) {
      return 0;  // unmodified
    }
  }
  return 1;  // modified
}

// open VFS resource
mjResource* mju_openVfsResource(const char* name, const mjVFS* vfs) {
  if (vfs == NULL) {
    return NULL;
  }

  // VFS provider
  static struct mjpResourceProvider provider = {
    .prefix   = NULL,
    .data     = NULL,
    .open     = &vfs_open_callback,
    .read     = &vfs_read_callback,
    .close    = &vfs_close_callback,
    .getdir   = &vfs_getdir_callback,
    .modified = &vfs_modified_callback,
  };

  // create resource
  mjResource* resource = (mjResource*) mju_malloc(sizeof(mjResource));
  if (resource == NULL) {
    mjERROR("could not allocate memory");
    return NULL;
  }

  // clear out resource
  memset(resource, 0, sizeof(mjResource));

  // copy name
  resource->name = mju_malloc(sizeof(char) * (strlen(name) + 1));
  if (resource->name == NULL) {
    mju_closeResource(resource);
    mjERROR("could not allocate memory");
    return NULL;
  }
  memcpy(resource->name, name, sizeof(char) * (strlen(name) + 1));
  resource->data = (void*) vfs;

  // open resource
  resource->provider = &provider;
  if (provider.open(resource)) {
    return resource;
  }

  // not found in VFS
  mju_closeResource(resource);
  return NULL;
}
