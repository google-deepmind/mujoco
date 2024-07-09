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

#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
  #include <unistd.h>
#endif

#ifdef _WIN32
  #define stat _stat
#endif

#include <mujoco/mjplugin.h>
#include "engine/engine_plugin.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"

namespace {

// file buffer used internally for the OS filesystem
struct FileSpec {
  bool is_read;                 // set to nonzero if buffer was read into
  std::vector<uint8_t> buffer;  // raw bytes from file
  time_t mtime;                 // last modified time
};

}  // namespace

// open the given resource; if the name doesn't have a prefix matching with a
// resource provider, then the OS filesystem is used
mjResource* mju_openResource(const char* name, char* error, size_t nerror) {
  // no error so far
  if (error) {
    error[0] = '\0';
  }

  mjResource* resource = (mjResource*) mju_malloc(sizeof(mjResource));
  if (resource == nullptr) {
    mjERROR("could not allocate memory");
    return nullptr;
  }

  // clear out resource
  memset(resource, 0, sizeof(mjResource));

  // copy name
  resource->name = (char*) mju_malloc(sizeof(char) * (strlen(name) + 1));
  if (resource->name == nullptr) {
    mju_closeResource(resource);
    mjERROR("could not allocate memory");
    return nullptr;
  }
  memcpy(resource->name, name, sizeof(char) * (strlen(name) + 1));

  // find provider based off prefix of name
  const mjpResourceProvider* provider = mjp_getResourceProvider(name);
  if (provider != nullptr) {
    resource->provider = provider;
    if (provider->open(resource)) {
      return resource;
    }

    if (error) {
      snprintf(error, nerror, "could not open '%s'"
               "using a resource provider matching prefix '%s'",
               name, provider->prefix);
    }

    mju_closeResource(resource);
    return nullptr;
  }

  // lastly fallback to OS filesystem
  resource->provider = nullptr;
  resource->data = new FileSpec;
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
    mju_closeResource(resource);
    return nullptr;
  }
  mju_encodeBase64(resource->timestamp, (uint8_t*) &spec->mtime,
                   sizeof(time_t));
  return resource;
}



// close the given resource; no-op if resource is NULL
void mju_closeResource(mjResource* resource) {
  if (resource == nullptr) {
    return;
  }

  // use the resource provider close callback
  if (resource->provider && resource->provider->close) {
    resource->provider->close(resource);
  } else {
    // clear OS filesystem if present
    FileSpec* spec = (FileSpec*) resource->data;
    if (spec) {
      delete spec;
    }
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
  FileSpec* spec = (FileSpec*) resource->data;

  // only read once from file
  if (!spec->is_read) {
    spec->buffer = mju_fileToMemory(resource->name);
    spec->is_read = true;
  }
  *buffer = spec->buffer.data();
  return spec->buffer.size();
}



// get directory path of resource
void mju_getResourceDir(mjResource* resource, const char** dir, int* ndir) {
  *dir = NULL;
  *ndir = 0;

  if (resource == NULL) {
    return;
  }

  // provider is not OS filesystem
  if (resource->provider) {
    if (resource->provider->getdir) {
      resource->provider->getdir(resource, dir, ndir);
    }
  } else {
    *dir = resource->name;
    *ndir = mju_dirnamelen(resource->name);
  }
}



// return 0 if the resource's timestamp matches the provided timestamp
// return > 0 if the the resource is younger than the given timestamp
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



// get the length of the dirname portion of a given path
int mju_dirnamelen(const char* path) {
  if (!path) {
    return 0;
  }

  int pos = -1;
  for (int i = 0; path[i]; ++i) {
    if (path[i] == '/' || path[i] == '\\') {
      pos = i;
    }
  }

  return pos + 1;
}



// read file into memory buffer (allocated here with mju_malloc)
std::vector<uint8_t> mju_fileToMemory(const char* filename) {
  FILE* fp = fopen(filename, "rb");
  if (!fp) {
    return {};
  }

  // find size
  if (fseek(fp, 0, SEEK_END) != 0) {
    fclose(fp);
    mju_warning("Failed to calculate size for '%s'", filename);
    return {};
  }

  // ensure file size fits in int
  long long_filesize = ftell(fp);  // NOLINT(runtime/int)
  if (long_filesize > INT_MAX) {
    fclose(fp);
    mju_warning("File size over 2GB is not supported. File: '%s'", filename);
    return {};
  } else if (long_filesize < 0) {
    fclose(fp);
    mju_warning("Failed to calculate size for '%s'", filename);
    return {};
  }

  std::vector<uint8_t> buffer(long_filesize);

  // go back to start of file
  if (fseek(fp, 0, SEEK_SET) != 0) {
    fclose(fp);
    mju_warning("Read error while reading '%s'", filename);
    return {};
  }

  // allocate and read
  std::size_t bytes_read = fread(buffer.data(), 1, buffer.size(), fp);

  // check that read data matches file size
  if (bytes_read != buffer.size()) {  // SHOULD NOT OCCUR
    if (ferror(fp)) {
      fclose(fp);
      mju_warning("Read error while reading '%s'", filename);
      return {};
    } else if (feof(fp)) {
      buffer.resize(bytes_read);
    }
  }

  // close file, return contents
  fclose(fp);
  return buffer;
}
