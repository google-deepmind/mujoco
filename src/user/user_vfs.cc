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

#include "user/user_vfs.h"

#include <cstddef>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "engine/engine_util_misc.h"
#include "user/user_util.h"

namespace {

using mujoco::user::FilePath;
using mujoco::user::FileToMemory;

// internal struct for VFS files
struct VFSFile {
  FilePath filename;
  std::vector<uint8_t> filedata;
  std::size_t filesize;
  uint64_t filestamp;
};

// internal container class for VFS
class VFS {
 public:
  // returns true if the file exists in the VFS
  bool HasFile(const FilePath& filename) const;

  // returns inserted mjuuVFSFile if the file was added successfully. This class
  // assumes ownership of the buffer.
  VFSFile* AddFile(const FilePath& filename, std::vector<uint8_t>&& buffer,
                   uint64_t filestamp);

  // returns the internal file struct for the given filename
  const VFSFile* GetFile(const FilePath& filename) const;

  // deletes file from VFS, return 0: success, -1: not found
  int DeleteFile(const FilePath& filename);

 private:
  std::unordered_map<std::string, VFSFile> files_;
};

// returns the internal VFS class pointer from the VFS C struct
inline VFS* GetVFSImpl(const mjVFS* vfs) {
  return vfs->impl_ ? static_cast<VFS*>(vfs->impl_) : nullptr;
}

// strip path prefix from filename and make lowercase
FilePath StripPath(const char* filename) {
  return FilePath(filename).StripPath().Lower();
}

// copies data into a buffer and produces a hash of the data
uint64_t vfs_memcpy(std::vector<uint8_t>& dest, const void* src, size_t n) {
  uint64_t hash = 0xcbf29ce484222325;  // magic number
  uint64_t prime = 0x100000001b3;      // magic prime
  const uint8_t* bytes = (uint8_t*) src;
  for (size_t i = 0; i < n; i++) {
    dest.push_back(bytes[i]);

    // do FNV-1 hash
    hash |= bytes[i];
    hash *= prime;
  }
  return hash;
}

// VFS hash function implemented using the FNV-1 hash
uint64_t vfs_hash(const std::vector<uint8_t>& buffer) {
  uint64_t hash = 0xcbf29ce484222325;  // magic number
  uint64_t prime = 0x100000001b3;      // magic prime
  const uint8_t* bytes = (uint8_t*) buffer.data();
  std::size_t n = buffer.size();
  for (std::size_t i = 0; i < n; i++) {
    hash |= bytes[i];
    hash *= prime;
  }
  return hash;
}

bool VFS::HasFile(const FilePath& filename) const {
  return files_.find(filename.Str()) != files_.end();
}

VFSFile* VFS::AddFile(const FilePath& filename, std::vector<uint8_t>&& buffer,
                      uint64_t filestamp) {
  auto [it, inserted] = files_.insert({filename.Str(), VFSFile()});
  if (!inserted) {
    return nullptr;  // repeated name
  }
  it->second.filename = filename;
  it->second.filedata = buffer;
  it->second.filestamp = filestamp;
  return &(it->second);
}

const VFSFile* VFS::GetFile(const FilePath& filename) const {
  auto it = files_.find(filename.Str());
  if (it == files_.end()) {
    return nullptr;
  }
  return &it->second;
}

int VFS::DeleteFile(const FilePath& filename) {
  auto it = files_.find(filename.Str());
  if (it == files_.end()) {
    return -1;
  }
  files_.erase(it);
  return 0;
}

// open callback for the VFS resource provider
int Open(mjResource* resource) {
  if (!resource || !resource->name || !resource->data) {
    return 0;
  }

  const mjVFS* vfs = (const mjVFS*) resource->data;
  const VFS* cvfs = GetVFSImpl(vfs);
  const VFSFile* file = cvfs->GetFile(StripPath(resource->name));
  if (file == nullptr) {
    file = cvfs->GetFile(FilePath(resource->name));
    if (file == nullptr) {
      return 0;
    }
  }

  resource->data = (void*) file;
  resource->timestamp[0] = '\0';
  if (file->filestamp) {
    mju_encodeBase64(resource->timestamp, (uint8_t*) &file->filestamp,
                     sizeof(uint64_t));
  }
  return 1;
}

// read callback for the VFS resource provider
int Read(mjResource* resource, const void** buffer) {
  if (!resource || !resource->name || !resource->data) {
    *buffer = nullptr;
    return -1;
  }

  const VFSFile* file = static_cast<const VFSFile*>(resource->data);
  if (file == nullptr) {
    *buffer = nullptr;
    return -1;
  }

  *buffer = file->filedata.data();
  return file->filedata.size();
}

// close callback for the VFS resource provider
void Close(mjResource* resource) {
}

// getdir callback for the VFS resource provider
void GetDir(mjResource* resource, const char** dir, int* ndir) {
  *dir = (resource) ? resource->name : nullptr;
  *ndir = (resource) ? mjuu_dirnamelen(resource->name) : 0;
}

// modified callback for the VFS resource provider
// return > 0 if modified and 0 if unmodified
int Modified(const mjResource* resource, const char* timestamp) {
  uint64_t filestamp;
  if (mju_isValidBase64(timestamp) > sizeof(uint64_t)) {
    return 2;  // error (assume modified)
  }

  mju_decodeBase64((uint8_t*) &filestamp, timestamp);
  if (!filestamp) return 3;  // no hash (assume modified)

  if (resource) {
    const VFSFile* file = static_cast<const VFSFile*>(resource->data);
    if (file == nullptr) return 4;  // missing file (assume modified)
    if (!file->filestamp) return 5;  // missing filestamp (assume modified)

    if (file->filestamp == filestamp) {
      return 0;  // unmodified
    }
  }
  return 1;  // modified
}

}  // namespace

// initialize to empty (no deallocation)
void mj_defaultVFS(mjVFS* vfs) {
  vfs->impl_ = new VFS();
}

// add file to VFS, return 0: success, 2: repeated name, -1: failed to load
int mj_addFileVFS(mjVFS* vfs, const char* directory, const char* filename) {
  VFS* cvfs = GetVFSImpl(vfs);

  // make full name
  FilePath fullname = FilePath(directory, filename);

  // strip path
  FilePath newname = StripPath(filename);

  // check beforehand for repeated name, to avoid reading file into memory
  if (cvfs->HasFile(newname)) {
    return 2;
  }

  // allocate and read
  std::vector<uint8_t> buffer = FileToMemory(fullname.c_str());
  if (buffer.empty()) {
    return -1;
  }

  if (!cvfs->AddFile(newname, std::move(buffer), vfs_hash(buffer))) {
    return 2;  // AddFile failed, SHOULD NOT OCCUR
  }
  return 0;
}

// add file from buffer into VFS
int mj_addBufferVFS(mjVFS* vfs, const char* name, const void* buffer,
                    int nbuffer) {
  std::vector<uint8_t> inbuffer;
  VFS* cvfs = GetVFSImpl(vfs);
  VFSFile* file;
  if (!(file = cvfs->AddFile(FilePath(name), std::move(inbuffer), 0))) {
    return 2;  // AddFile failed, repeated name
  }
  file->filedata.reserve(nbuffer);
  file->filestamp = vfs_memcpy(file->filedata, buffer, nbuffer);
  return 0;
}

// delete file from VFS, return 0: success, -1: not found in VFS
int mj_deleteFileVFS(mjVFS* vfs, const char* filename) {
  VFS* cvfs = GetVFSImpl(vfs);
  if (cvfs->DeleteFile(StripPath(filename))) {
    return cvfs->DeleteFile(FilePath(filename));
  }
  return 0;
}

// delete all files from VFS
void mj_deleteVFS(mjVFS* vfs) {
  if (vfs) {
    delete GetVFSImpl(vfs);
  }
}

const mjpResourceProvider* GetVfsResourceProvider() {
  static mjpResourceProvider provider
    = { nullptr, &Open, &Read, &Close, &GetDir, &Modified, nullptr };
  return &provider;
}
