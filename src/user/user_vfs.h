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
// IWYU pragma: private, include "third_party/mujoco/include/mujoco.h"
// IWYU pragma: friend "third_party/mujoco/src/.*"

#ifndef MUJOCO_SRC_USER_USER_VFS_H_
#define MUJOCO_SRC_USER_USER_VFS_H_

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>

#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "user/user_util.h"

namespace mujoco::user {

// Underlying Virtual File System implementation for opaque mjVFS struct.
//
// This class owns and manages all the mjResource instances that are created
// using the mju_openResource. Its main job is to find the correct
// mjpResourceProvider to handle the mju open/read/close operations. It does
// this by "mounting" mjpResourceProviders at specific paths such that any
// operation within that path will be handled by the mjpResourceProvider.
//
// Mounting can be done explicitly (using mj_mountVFS) or implicitly (using
// mjp_registerResourceProvider). If no provider is found for a given path,
// then a "default" provider is used that uses normal C file operations to
// open/read/close files.
//
// To support legacy use-cases (where the VFS is an optional argument), a
// "self-destruct" mode can be configured so that a temporary VFS instance can
// be created with a lifetime tied to the opened mjResource instance.
//
// This class itself is thread-safe, but it makes no guarantees about the
// thread-safety of the underlying mjResourceProviders.
class VFS {
 public:
  explicit VFS(mjVFS* vfs);
  ~VFS();

  VFS(const VFS&) = delete;
  VFS& operator=(const VFS&) = delete;

  // Status codes for VFS operations. Values are based on the mj_VFS APIs.
  enum Status {
    kSuccess = 0,
    kFailedToLoad = -1,
    kFailedToRead = -1,
    kNotFound = -1,
    kRepeatedName = 2,
    kInvalidVfs = -1,
    kInvalidResource = -1,
    kInvalidResourceProvider = -1,
  };

  // Opens a mjResource for the given path, or nullptr on error. If successful,
  // will invoke the 'open' callback for the mjpResourceProvider associated with
  // the path/dir.
  mjResource* Open(const char* dir, const char* name);

  // Sets `buffer` to the contents of the resource and returns the number of
  // bytes of the content. This is done by invoking the 'read' callback for the
  // mjpResourceProvider associated with the resource. Returns -1 on error.
  int Read(mjResource* resource, const void** buffer);

  // Closes the resource by invoking the 'close' callback for the
  // mjpResourceProvider associated with the resource.
  Status Close(mjResource* resource);

  // Mounts a ResourceProvider at the given path. All subsequent operations
  // under `path` will be delegated to the `provider` until it is unmounted.
  Status Mount(const FilePath& path, const mjpResourceProvider* provider);

  // Unmounts the ResourceProvider from the given path.
  Status Unmount(const FilePath& path);

  // Sets a destructor to be called when the VFS has no more open resources.
  // Assumes that `destructor` will delete `this`.
  //
  // This is useful for when you want to create a temporary VFS instance with
  // a lifetime tied to a single mjResource to be opened. The `destructor`
  // should be set to `delete this` and any other cleanup that needs to happen.
  void SetToSelfDestruct(std::function<void(mjVFS*)> destructor);

  // Converts the public C-API pointer to the internal C++ class.
  static VFS* Upcast(mjVFS* vfs);
  static const VFS* Upcast(const mjVFS* vfs);

 private:
  using ResourcePtr = std::unique_ptr<mjResource, void (*)(mjResource*)>;
  ResourcePtr CreateResource(std::string_view name,
                             const mjpResourceProvider* provider);

  // Returns a mounted mjResource* that matches the given path. If no explicitly
  // mounted mjResource* is found, returns a "default" mounting that uses the
  // C file system.
  mjResource* FindMount(const std::string& fullpath);

  // Invokes the `destructor_, but only if it has been set previously. This
  // should be only called when resources_ is empty and callers should assume
  // that `this` will be invalidated after this call.
  void MaybeSelfDestruct();

  mjVFS* self_;
  std::mutex mutex_;  // Protects open_resources_ and mounts_.
  std::unordered_map<mjResource*, ResourcePtr> open_resources_;
  std::unordered_map<std::string, ResourcePtr> mounts_;
  mjResource default_mount_;
  mjpResourceProvider default_provider_;
  std::function<void(mjVFS*)> destructor_;
};

}  // namespace mujoco::user

#endif  // MUJOCO_SRC_USER_USER_VFS_H_
