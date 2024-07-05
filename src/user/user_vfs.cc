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

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstring>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "user/user_resource.h"
#include "user/user_util.h"

namespace {

// internal struct for VFS files
struct VFSFile {
  std::string filename;
  std::unique_ptr<void, std::function<void(void*)>> filedata;
  std::size_t filesize;
  uint64_t filestamp;
};

// internal container class for VFS
class VFS {
 public:
  // returns true if the file exists in the VFS
  bool HasFile(const std::string& filename) const;

  // returns inserted mjuuVFSFile if the file was added successfully. This class
  // assumes ownership of the buffer and will free it when the VFS is deleted.
  VFSFile* AddFile(const std::string& filename, void* buffer,
                   std::size_t nbuffer, uint64_t filestamp);

  // returns the internal file struct for the given filename
  const VFSFile* GetFile(const std::string& filename) const;

  // deletes file from VFS, return 0: success, -1: not found
  int DeleteFile(const std::string& filename);

 private:
  std::unordered_map<std::string, VFSFile> files_;
};

// returns the internal VFS class pointer from the VFS C struct
inline VFS* GetVFSImpl(const mjVFS* vfs) {
  return vfs->impl_ ? static_cast<VFS*>(vfs->impl_) : nullptr;
}

// strip path prefix from filename and make lowercase
std::string StripPath(const char* name) {
  std::string newname = mjuu_strippath(name);

  // make lowercase
  std::transform(newname.begin(), newname.end(), newname.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return newname;
}

// copies data into a buffer and produces a hash of the data
uint64_t vfs_memcpy(void* dest, const void* src, size_t n) {
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
uint64_t vfs_hash(const void* buffer, size_t n) {
  uint64_t hash = 0xcbf29ce484222325;  // magic number
  uint64_t prime = 0x100000001b3;      // magic prime
  const uint8_t* bytes = (uint8_t*) buffer;
  for (size_t i = 0; i < n; i++) {
    hash |= bytes[i];
    hash *= prime;
  }
  return hash;
}

bool VFS::HasFile(const std::string& filename) const {
  return files_.find(filename) != files_.end();
}

VFSFile* VFS::AddFile(const std::string& filename, void* buffer,
                      std::size_t nbuffer, uint64_t filestamp) {
  auto [it, inserted] = files_.insert({filename, VFSFile()});
  if (!inserted) {
    return nullptr;  // repeated name
  }
  it->second.filename = filename;
  it->second.filedata = std::unique_ptr<void, void(*)(void*)>(
      buffer, [](void* b) { mju_free(b); });  // corresponding to mju_malloc
  it->second.filesize = nbuffer;
  it->second.filestamp = filestamp;
  return &(it->second);
}

const VFSFile* VFS::GetFile(const std::string& filename) const {
  auto it = files_.find(filename);
  if (it == files_.end()) {
    return nullptr;
  }
  return &it->second;
}

int VFS::DeleteFile(const std::string& filename) {
  auto it = files_.find(filename);
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
    return 0;
  }

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

  const VFS* vfs = GetVFSImpl(static_cast<const mjVFS*>(resource->data));
  const VFSFile* file = vfs->GetFile(StripPath(resource->name));
  if (file == nullptr) {
    *buffer = nullptr;
    return -1;
  }

  *buffer = file->filedata.get();
  return file->filesize;
}

// close callback for the VFS resource provider
void Close(mjResource* resource) {
}

// getdir callback for the VFS resource provider
void GetDir(mjResource* resource, const char** dir, int* ndir) {
  *dir = (resource) ? resource->name : nullptr;
  *ndir = (resource) ? mju_dirnamelen(resource->name) : 0;
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
    const VFS* cvfs = GetVFSImpl(static_cast<const mjVFS*>(resource->data));
    const VFSFile* file = cvfs->GetFile(StripPath(resource->name));
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
  std::string fullname = mjuu_combinePaths(directory, filename);

  // strip path
  std::string newname = StripPath(filename);

  // check beforehand for repeated name, to avoid reading file into memory
  if (cvfs->HasFile(newname)) {
    return 2;
  }

  // allocate and read
  size_t nbuffer = 0;
  void* buffer = mju_fileToMemory(fullname.c_str(), &nbuffer);
  if (buffer == nullptr) {
    return -1;
  }

  if (!cvfs->AddFile(newname, buffer, nbuffer, vfs_hash(buffer, nbuffer))) {
    mju_free(buffer);
    return 2;  // AddFile failed, SHOULD NOT OCCUR
  }
  return 0;
}

// add file from buffer into VFS
int mj_addBufferVFS(mjVFS* vfs, const char* name, const void* buffer,
                    int nbuffer) {
  VFS* cvfs = GetVFSImpl(vfs);

  // allocate and clear
  void* inbuffer = mju_malloc(nbuffer);
  if (buffer == nullptr) {
    mjERROR("could not allocate memory");
  }
  VFSFile* file;
  if (!(file = cvfs->AddFile(StripPath(name), inbuffer, nbuffer, 0))) {
    mju_free(inbuffer);
    return 2;  // AddFile failed, repeated name
  }
  file->filestamp = vfs_memcpy(inbuffer, buffer, nbuffer);
  return 0;
}

// delete file from VFS, return 0: success, -1: not found in VFS
int mj_deleteFileVFS(mjVFS* vfs, const char* filename) {
  VFS* cvfs = GetVFSImpl(vfs);
  return cvfs->DeleteFile(StripPath(filename));
}

// delete all files from VFS
void mj_deleteVFS(mjVFS* vfs) {
  if (vfs) {
    delete GetVFSImpl(vfs);
  }
}

// open VFS resource
mjResource* mju_openVfsResource(const char* name, const mjVFS* vfs) {
  if (vfs == nullptr) {
    return nullptr;
  }

  // VFS provider
  static struct mjpResourceProvider provider = { nullptr, &Open, &Read, &Close,
                                                &GetDir, &Modified, nullptr };

  // create resource
  mjResource* resource = (mjResource*) mju_malloc(sizeof(mjResource));
  if (resource == nullptr) {
    mjERROR("could not allocate memory");
    return nullptr;
  }

  // clear out resource
  memset(resource, 0, sizeof(mjResource));

  // copy name
  std::size_t n = std::strlen(name);
  resource->name = (char*) mju_malloc(sizeof(char) * (n + 1));
  if (resource->name == nullptr) {
    mju_closeResource(resource);
    mjERROR("could not allocate memory");
    return nullptr;
  }
  std::memcpy(resource->name, name, sizeof(char) * (n + 1));
  resource->data = (void*) vfs;

  // open resource
  resource->provider = &provider;
  if (provider.open(resource)) {
    return resource;
  }

  // not found in VFS
  mju_closeResource(resource);
  return nullptr;
}
