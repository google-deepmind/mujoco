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

#include "engine/engine_resource.h"

#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
  #include <unistd.h>
#endif

#ifdef _WIN32
  #define stat _stat
#endif

#include <mujoco/mjplugin.h>
#include "engine/engine_plugin.h"
#include "engine/engine_util_errmem.h"

// file buffer used internally for the OS filesystem
typedef struct {
  uint8_t* buffer;  // raw bytes from file
  size_t nbuffer;   // size of buffer in bytes
  time_t mtime;     // last modified time
} file_buffer;

// open the given resource; if the name doesn't have a prefix matching with a
// resource provider, then the OS filesystem is used
mjResource* mju_openResource(const char* name, char* error, size_t error_sz) {
  // no error so far
  if (error) {
    error[0] = '\0';
  }

  mjResource* resource = (mjResource*) mju_malloc(sizeof(mjResource));
  const mjpResourceProvider* provider = NULL;
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

  // find provider based off prefix of name
  provider = mjp_getResourceProvider(name);
  if (provider != NULL) {
    resource->provider = provider;
    if (provider->open(resource)) {
      return resource;
    }

    if (error) {
      snprintf(error, error_sz, "could not open '%s'"
               "using a resource provider matching prefix '%s'",
               name, provider->prefix);
    }

    mju_closeResource(resource);
    return NULL;
  }

  // lastly fallback to OS filesystem
  resource->provider = NULL;
  resource->data = mju_malloc(sizeof(file_buffer));
  file_buffer* fb = (file_buffer*) resource->data;
  fb->buffer = mju_fileToMemory(name, &(fb->nbuffer));
  if (fb->buffer == NULL) {
    if (error) {
      snprintf(error, error_sz,
               "resource not found via provider or OS filesystem: '%s'", name);
    }
    mju_closeResource(resource);
    return NULL;
  }
  struct stat file_stat;
  if (stat(name, &file_stat) == 0) {
    memcpy(&fb->mtime, &file_stat.st_mtime, sizeof(time_t));
  } else {
    memset(&fb->mtime, 0, sizeof(time_t));
    resource->timestamp[0] = '\0';
  }
  strftime(resource->timestamp, 512, "%Y-%m-%d-%H:%M:%S", localtime(&(fb->mtime)));
  return resource;
}



// close the given resource; no-op if resource is NULL
void mju_closeResource(mjResource* resource) {
  if (resource == NULL) {
    return;
  }

  // use the resource provider close callback
  if (resource->provider && resource->provider->close) {
    resource->provider->close(resource);
  } else {
    // clear OS filesystem if present
    file_buffer* fb = (file_buffer*) resource->data;
    if (fb) {
      if (fb->buffer) mju_free(fb->buffer);
      mju_free(fb);
    }
  }

  // free resource
  if (resource->name) mju_free(resource->name);
  mju_free(resource);
}



// set buffer to bytes read from the resource and return number of bytes in buffer;
// return negative value if error
int mju_readResource(mjResource* resource, const void** buffer) {
  if (resource == NULL) {
    return 0;
  }

  if (resource->provider) {
    return resource->provider->read(resource, buffer);
  }


  // if provider is NULL, then OS filesystem is used
  const file_buffer* fb = (file_buffer*) resource->data;
  *buffer = fb->buffer;
  return fb->nbuffer;
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



// modified callback for OS filesystem
static int mju_isModifiedFile(const char* name, const file_buffer* fb) {
  if (fb != NULL) {
    struct stat file_stat;
    if (stat(name, &file_stat) == 0) {
      return difftime(fb->mtime, file_stat.st_mtime) < 0;
    }
  }
  return 1;  // modified (default)
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

  return mju_isModifiedFile(resource->name, (file_buffer*) resource->data);
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
void* mju_fileToMemory(const char* filename, size_t* filesize) {
  // open file
  *filesize = 0;
  FILE* fp = fopen(filename, "rb");
  if (!fp) {
    return NULL;
  }

  // find size
  if (fseek(fp, 0, SEEK_END) != 0) {
    fclose(fp);
    mju_warning("Failed to calculate size for '%s'", filename);
    return NULL;
  }

  // ensure file size fits in int
  long long_filesize = ftell(fp);  // NOLINT(runtime/int)
  if (long_filesize > INT_MAX) {
    fclose(fp);
    mju_warning("File size over 2GB is not supported. File: '%s'", filename);
    return NULL;
  } else if (long_filesize < 0) {
    fclose(fp);
    mju_warning("Failed to calculate size for '%s'", filename);
    return NULL;
  }
  *filesize = long_filesize;

  // go back to start of file
  if (fseek(fp, 0, SEEK_SET) != 0) {
    fclose(fp);
    mju_warning("Read error while reading '%s'", filename);
    return NULL;
  }

  // allocate and read
  void* buffer = mju_malloc(*filesize);
  if (!buffer) {
    mjERROR("could not allocate memory");
  }
  size_t bytes_read = fread(buffer, 1, *filesize, fp);

  // check that read data matches file size
  if (bytes_read != *filesize) {  // SHOULD NOT OCCUR
    if (ferror(fp)) {
      fclose(fp);
      mju_free(buffer);
      *filesize = 0;
      mju_warning("Read error while reading '%s'", filename);
      return NULL;
    } else if (feof(fp)) {
      *filesize = bytes_read;
    }
  }

  // close file, return contents
  fclose(fp);
  return buffer;
}
