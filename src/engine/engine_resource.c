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
#include <stdio.h>
#include <string.h>

#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include "engine/engine_plugin.h"
#include "engine/engine_util_errmem.h"

// file buffer used internally for the OS filesystem
typedef struct {
  void* buffer;
  int nbuffer;
} file_buffer;

// helper function to fill data from resource provider into provider
static void fillResource(const mjpResourceProvider* provider, mjResource* resource) {
  if (provider == NULL) {
    resource->read = NULL;
    resource->close = NULL;
    resource->getdir = NULL;
    resource->provider_data = NULL;
  } else {
    resource->read = provider->read;
    resource->close = provider->close;
    resource->getdir = provider->getdir;
    resource->provider_data = provider->data;
  }
}


// open the given resource; if the name doesn't have a prefix matching with a
// resource provider, then the default_provider is used
// if default_provider non-positive, then the OS filesystem is used
mjResource* mju_openResource(const char* name, int default_provider) {
  mjResource* resource = (mjResource*) mju_malloc(sizeof(mjResource));
  const mjpResourceProvider* provider = NULL;
  if (resource == NULL) {
    mjERROR("could not allocate memory");
    return NULL;
  }

  // copy name
  resource->name = mju_malloc(sizeof(char) * (strlen(name) + 1));
  if (resource->name == NULL) {
    mju_free(resource);
    mjERROR("could not allocate memory");
    return NULL;
  }
  strcpy(resource->name, name);

  // find provider based off prefix of name
  provider = mjp_getResourceProvider(name);
  if (provider != NULL) {
    fillResource(provider, resource);
    if (provider->open(resource)) {
      return resource;
    }

    mju_warning("mju_openResource: could not open resource '%s' "
                "using a resource provider matching prefix '%s'",
                name, provider->prefix);
    mju_free(resource->name);
    mju_free(resource);
    return NULL;
  }

  // fallback to default provider
  if (default_provider > 0) {
    provider = mjp_getResourceProviderAtSlot(default_provider);
    if (provider == NULL) {
      mju_warning("mju_openResource: unknown resource provider at slot %d",
                  default_provider);
      mju_free(resource->name);
      mju_free(resource);
      return NULL;
    }

    fillResource(provider, resource);
    if (provider->open(resource)) {
      return resource;
    }

    mju_warning("mju_openResource: could not open resource '%s' "
                "with default provider at slot %d",
                name, default_provider);
    mju_free(resource->name);
    mju_free(resource);
    return NULL;
  }

  // lastly fallback to OS filesystem
  else {
    fillResource(NULL, resource);
    resource->data = mju_malloc(sizeof(file_buffer));
    file_buffer* fb = (file_buffer*) resource->data;
    fb->buffer = mju_fileToMemory(name, &(fb->nbuffer));
    if (fb->buffer == NULL) {
      mju_warning("mju_openResource: unknown file '%s'", name);
      mju_free(fb);
      mju_free(resource->name);
      mju_free(resource);
      return NULL;
    }
  }
  return resource;
}



// close the given resource; no-op if resource is NULL
void mju_closeResource(mjResource* resource) {
  if (resource == NULL) {
    return;
  }

  // use the resource provider to close resource
  if (resource->close) {
    resource->close(resource);
  }

  // if provider is NULL, then OS filesystem is used
  else {
    file_buffer* fb = (file_buffer*) resource->data;
    mju_free(fb->buffer);
    mju_free(fb);
  }

  // free name and resource
  mju_free(resource->name);
  mju_free(resource);
}



// set buffer to bytes read from the resource and return number of bytes in buffer;
// return negative value if error
int mju_readResource(mjResource* resource, const void** buffer) {
  if (resource == NULL) {
    return 0;
  }

  if (resource->read) {
    return resource->read(resource, buffer);
  }


  // if provider read callback is NULL, then OS filesystem is used
  const file_buffer* fb = (file_buffer*) resource->data;
  *buffer = fb->buffer;
  return fb->nbuffer;
}



// get directory path of resource
void mju_getResourceDir(mjResource* resource, const char** dir, int* ndir) {
  *dir = NULL;
  *ndir = 0;

  if (!resource) {
    return;
  }

  // provider is not OS filesystem
  if (resource->read) {
    if (resource->getdir) {
      resource->getdir(resource, dir, ndir);
    }
  } else {
    *dir = resource->name;
    *ndir = mju_dirnamelen(resource->name);
  }
}



// get the length of the dirname portion of a given path
int mju_dirnamelen(const char* path) {
  if (!path) {
    return 0;
  }

  int pos = -1;
  for (int i = 0; path[i] && i >= 0; ++i) {
    if (path[i] == '/' || path[i] == '\\') {
      pos = i;
    }
  }

  return pos + 1;
}



// read file into memory buffer (allocated here with mju_malloc)
void* mju_fileToMemory(const char* filename, int* filesize) {
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
