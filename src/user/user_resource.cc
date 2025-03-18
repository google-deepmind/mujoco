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

#include "user/user_resource.h"

#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
  #include <unistd.h>
#endif

#ifdef _WIN32
  #define stat _stat
#endif

#include <mujoco/mjplugin.h>
#include "engine/engine_plugin.h"
#include "engine/engine_util_misc.h"
#include "user/user_util.h"
#include "user/user_vfs.h"

namespace {

using mujoco::user::FileToMemory;

// file buffer used internally for the OS filesystem
struct FileSpec {
  bool is_read;                 // set to nonzero if buffer was read into
  std::vector<uint8_t> buffer;  // raw bytes from file
  time_t mtime;                 // last modified time
};

// callback for opening a resource, returns zero on failure
int FileOpen(mjResource* resource, char* error, size_t nerror) {
  resource->provider = nullptr;
  resource->data = new FileSpec;
  const char* name = resource->name;
  FileSpec* spec = (FileSpec*) resource->data;
  spec->is_read = false;
  struct stat file_stat;
  if (stat(name, &file_stat) == 0) {
    memcpy(&spec->mtime, &file_stat.st_mtime, sizeof(time_t));
  } else {
    if (error) {
      snprintf(error, nerror, "Error opening file '%s': %s", name,
               strerror(errno));
    }
    return 0;
  }
  mju_encodeBase64(resource->timestamp, (uint8_t*) &spec->mtime,
                   sizeof(time_t));
  return 1;
}

// OS filesystem read callback
int FileRead(mjResource* resource, const void** buffer) {
  FileSpec* spec = (FileSpec*) resource->data;

  // only read once from file
  if (!spec->is_read) {
    spec->buffer = FileToMemory(resource->name);
    spec->is_read = true;
  }
  *buffer = spec->buffer.data();
  return spec->buffer.size();
}

// OS filesystem close callback
void FileClose(mjResource* resource) {
  FileSpec* spec = (FileSpec*) resource->data;
  if (spec) delete spec;
}

// OS filesystem getdir callback
void FileGetDir(mjResource* resource, const char** dir, int* ndir) {
  *dir = resource->name;
  *ndir = mjuu_dirnamelen(resource->name);
}

// OS filesystem modified callback
int FileModified(const mjResource* resource, const char*timestamp) {
  if (mju_isValidBase64(timestamp) != sizeof(time_t)) {
    return 1;  // error (assume modified)
  }

  time_t time1, time2;
  mju_decodeBase64((uint8_t*) &time1, timestamp);
  time2 = ((FileSpec*) resource->data)->mtime;
  double diff = difftime(time2, time1);
  if (diff < 0) return -1;
  if (diff > 0) return  1;
  return 0;
}

}  // namespace

// open the given resource; if the name doesn't have a prefix matching with a
// resource provider, then the OS filesystem is used
mjResource* mju_openResource(const char* dir, const char* name,
                             const mjVFS* vfs, char* error, size_t nerror) {
  // no error so far
  if (error) {
    error[0] = '\0';
  }

  mjResource* resource = (mjResource*) mju_malloc(sizeof(mjResource));
  if (resource == nullptr) {
    if (error) {
      strncpy(error, "could not allocate memory", nerror);
      error[nerror - 1] = '\0';
    }
    return nullptr;
  }

  // clear out resource
  memset(resource, 0, sizeof(mjResource));

  // make space for filename
  std::string fullname = mjuu_combinePaths(dir, name);
  std::size_t n = fullname.size();
  resource->name = (char*) mju_malloc(sizeof(char) * (n + 1));
  if (resource->name == nullptr) {
    if (error) {
      strncpy(error, "could not allocate memory", nerror);
      error[nerror - 1] = '\0';
    }
    mju_closeResource(resource);
    return nullptr;
  }

  // first priority is to check the VFS
  if (vfs != nullptr) {
    memcpy(resource->name, name,
           sizeof(char) * (std::strlen(name) + 1));
    const mjpResourceProvider* provider = GetVfsResourceProvider();
    resource->data = (void*) vfs;
    resource->provider = provider;
    if (provider->open(resource)) {
      return resource;
    }
  }

  // copy full path over
  memcpy(resource->name, fullname.c_str(), sizeof(char) * (n + 1));

  // find provider based off prefix of name
  const mjpResourceProvider* provider = mjp_getResourceProvider(resource->name);
  if (provider != nullptr) {
    resource->provider = provider;
    resource->data = nullptr;
    if (provider->open(resource)) {
      return resource;
    }

    if (error) {
      snprintf(error, nerror, "could not open '%s'"
               "using a resource provider matching prefix '%s'",
               resource->name, provider->prefix);
    }

    mju_closeResource(resource);
    return nullptr;
  }

  // lastly fallback to OS filesystem
  if (FileOpen(resource, error, nerror)) {
    return resource;
  }
  mju_closeResource(resource);
  return nullptr;
}



// close the given resource; no-op if resource is NULL
void mju_closeResource(mjResource* resource) {
  if (resource == nullptr) {
    return;
  }

  // use the resource provider close callback
  const mjpResourceProvider* provider = resource->provider;
  if (provider) {
    if (provider->close) provider->close(resource);
  } else {
    FileClose(resource);  // clear OS filesystem if present
  }

  // free resource
  if (resource->name) mju_free(resource->name);
  mju_free(resource);
}



// set buffer to bytes read from the resource and return number of bytes in
// buffer; return negative value if error
int mju_readResource(mjResource* resource, const void** buffer) {
  if (resource->provider) {
    return resource->provider->read(resource, buffer);
  }

  // if provider is NULL, then OS filesystem is used
  return FileRead(resource, buffer);
}



// get directory path of resource
void mju_getResourceDir(mjResource* resource, const char** dir, int* ndir) {
  *dir = nullptr;
  *ndir = 0;

  if (resource == nullptr) {
    return;
  }

  const mjpResourceProvider* provider = resource->provider;
  if (provider) {
    if (provider->getdir) provider->getdir(resource, dir, ndir);
  } else {
    // fallback to OS filesystem
    FileGetDir(resource, dir, ndir);
  }
}



// return 0 if the resource's timestamp matches the provided timestamp
// return > 0 if the resource is younger than the given timestamp
// return < 0 if the resource is older than the given timestamp
int mju_isModifiedResource(const mjResource* resource, const char* timestamp) {
  // provider is not OS filesystem
  if (resource->provider) {
    if (resource->provider->modified) {
      return resource->provider->modified(resource, timestamp);
    }
    return 1;  // default (modified)
  }

  // fallback to OS filesystem
  return FileModified(resource, timestamp);
}
