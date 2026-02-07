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

#include <sys/types.h>
#include <sys/stat.h>

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <string_view>
#include <mujoco/mujoco.h>

#include <mujoco/mjplugin.h>
#include "engine/engine_plugin.h"
#include "user/user_util.h"
#include "user/user_vfs.h"

mjResource* mju_openResource(const char* dir, const char* name,
                             const mjVFS* vfs, char* error, size_t nerror) {
  // TODO: Update API to use non-const pointer. Unfortunately, while this is
  // ABI stable, it will cause compiler errors in user code that is const
  // correct.
  mjVFS* non_const_vfs = const_cast<mjVFS*>(vfs);

  // TODO: Eventually, we should make `vfs` a required argument. In the
  // meantime, when passing in a nullptr VFS, we will create a VFS dynamically
  // that self-destructs when the resource is closed (or if the resource could
  // not be opened).
  if (non_const_vfs == nullptr) {
    mjVFS* local_vfs = (mjVFS*)mju_malloc(sizeof(mjVFS));
    mj_defaultVFS(local_vfs);
    mujoco::user::VFS::Upcast(local_vfs)->SetToSelfDestruct([](mjVFS* ptr) {
      mj_deleteVFS(ptr);
      mju_free(ptr);
    });

    non_const_vfs = local_vfs;
  }

  mjResource* resource =
      mujoco::user::VFS::Upcast(non_const_vfs)->Open(dir ? dir : "", name);

  if (error) {
    if (resource) {
      error[0] = '\0';
    } else {
      std::snprintf(error, nerror, "Error opening file '%s'", name);
    }
  }

  return resource;
}

void mju_closeResource(mjResource* resource) {
  if (resource && resource->vfs) {
    mujoco::user::VFS::Upcast(resource->vfs)->Close(resource);
  }
}

int mju_readResource(mjResource* resource, const void** buffer) {
  if (resource && resource->vfs) {
    return mujoco::user::VFS::Upcast(resource->vfs)->Read(resource, buffer);
  }
  return -1;  // default (error reading bytes)
}

void mju_getResourceDir(mjResource* resource, const char** dir, int* ndir) {
  *dir = nullptr;
  *ndir = 0;

  if (resource && resource->name) {
    // ensure prefix is included even if there is no separator in the
    // resource name
    int prefix_len = 0;
    const mjpResourceProvider* provider = resource->provider;
    if (provider && provider->prefix) {
      prefix_len = strlen(provider->prefix) + 1;
    }

    *dir = resource->name;
    *ndir = prefix_len;
    for (int i = prefix_len; resource->name[i]; ++i) {
      if (resource->name[i] == '/' || resource->name[i] == '\\') {
        *ndir = i + 1;
      }
    }
  }
}

int mju_isModifiedResource(const mjResource* resource, const char* timestamp) {
  if (resource && resource->provider && resource->provider->modified) {
    return resource->provider->modified(resource, timestamp);
  }
  return 1;  // default (assume modified)
}

mjSpec* mju_decodeResource(mjResource* resource, const char* content_type, const mjVFS* vfs) {
  const mjpDecoder* decoder = nullptr;
  if (content_type) {
    decoder = mjp_findDecoder(resource, content_type);
  } else {
    decoder = mjp_findDecoder(resource, mjuu_extToContentType(resource->name).c_str());
  }
  if (!decoder) {
    mju_warning("Could not find decoder for resource '%s'", resource->name);
    return nullptr;
  }
  return decoder->decode(resource, vfs);
}
