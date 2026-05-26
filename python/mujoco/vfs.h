// Copyright 2026 DeepMind Technologies Limited
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

#ifndef MUJOCO_PYTHON_VFS_H_
#define MUJOCO_PYTHON_VFS_H_

#include <memory>

#include <mujoco/mujoco.h>

namespace mujoco::python {

class MjVfs {
 public:
  MjVfs() : vfs_(new mjVFS) { mj_defaultVFS(vfs_.get()); }

  void Close() { vfs_.reset(); }

  mjVFS* get() const { return vfs_.get(); }

  bool is_open() const { return vfs_ != nullptr; }

 private:
  struct VfsDeleter {
    void operator()(mjVFS* vfs) const {
      mj_deleteVFS(vfs);
      delete vfs;
    }
  };
  std::unique_ptr<mjVFS, VfsDeleter> vfs_;
};

}  // namespace mujoco::python

#endif  // MUJOCO_PYTHON_VFS_H_
