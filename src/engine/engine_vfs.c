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
#include <stdlib.h>

#include "engine/engine_array_safety.h"
#include "engine/engine_file.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"

// strip path prefix from filename
static void vfs_strippath(char* newname, const char* oldname) {
  int i, sz = strlen(oldname);

  // find last delimiter
  for (i=sz-1; i>=0; i--) {
    if (oldname[i]=='\\' || oldname[i]=='/') {
      break;
    }
  }

  // check resulting length
  if (sz-(i+1)>=mjMAXVFSNAME) {
    mju_error("Filename too long in VFS");
  }
  if (sz-(i+1)<=0) {
    mju_error("Empty filename in VFS");
  }

  // copy
  mju_strncpy(newname, oldname+i+1, mjMAXVFSNAME);

  // make lowercase
  for (i=strlen(newname)-1; i>=0; i--) {
    if (newname[i]>='A' && newname[i]<='Z') {
      newname[i] = (char)(((int)newname[i]) +'a' - 'A');
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
  if (vfs->nfile>=mjMAXVFS-1) {
    return 1;
  }

  // make full name
  char fullname[1000];
  if (directory) {
    mjSTRNCPY(fullname, directory);
    mjSTRNCAT(fullname, filename);
  } else {
    mjSTRNCPY(fullname, filename);
  }

  // strip path
  char newname[mjMAXVFSNAME];
  vfs_strippath(newname, filename);

  // check for repeated name
  for (int i=0; i<vfs->nfile; i++) {
    if (strncmp(newname, vfs->filename[i], mjMAXVFSNAME)==0) {
      return 2;
    }
  }

  // assign name
  mjSTRNCPY(vfs->filename[vfs->nfile], newname);

  // allocate and read
  int filesize = 0;
  vfs->filedata[vfs->nfile] = mju_fileToMemory(filename, &filesize);
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
  if (vfs->nfile>=mjMAXVFS-1) {
    return 1;
  }

  // check filesize
  if (filesize<=0) {
    mju_error("mj_makeEmptyFileVFS expects positive filesize");
  }

  // strip path
  char newname[mjMAXVFSNAME];
  vfs_strippath(newname, filename);

  // check for repeated name
  for (int i=0; i<vfs->nfile; i++) {
    if (strncmp(newname, vfs->filename[i], mjMAXVFSNAME)==0) {
      return 2;
    }
  }

  // assign name
  mjSTRNCPY(vfs->filename[vfs->nfile], newname);

  // allocate and clear
  vfs->filedata[vfs->nfile] = mju_malloc(filesize);
  if (!vfs->filedata[vfs->nfile]) {
    mju_error("mj_makeEmptyFileVFS: could not allocate memory");
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
  for (int i=0; i<vfs->nfile; i++) {
    if (strncmp(newname, vfs->filename[i], mjMAXVFSNAME)==0) {
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
  for (int i=0; i<vfs->nfile; i++) {
    if (strncmp(newname, vfs->filename[i], mjMAXVFSNAME)==0) {
      // free buffer
      mju_free(vfs->filedata[i]);

      // scroll remaining files forward
      while (i<vfs->nfile-1) {
        mjSTRNCPY(vfs->filename[i], vfs->filename[i+1]);
        vfs->filesize[i] = vfs->filesize[i+1];
        vfs->filedata[i] = vfs->filedata[i+1];
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
  for (int i=0; i<vfs->nfile; i++) {
    mju_free(vfs->filedata[i]);
  }

  memset(vfs, 0, sizeof(mjVFS));
}
