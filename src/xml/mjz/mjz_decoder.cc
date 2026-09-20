// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <cstring>
#include <string>

#include <mujoco/mjplugin.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include "user/user_util.h"

mjPLUGIN_LIB_INIT(mjz_decoder) {
  mjpDecoder decoder;
  decoder.content_type = "application/zip";
  decoder.extension    = ".mjz|.zip";
  decoder.can_decode   = +[](const mjResource* resource) {
    if (!resource || !resource->name) { return 0; }
    const char* ext = strrchr(resource->name, '.');
    return ext ? (!strcmp(ext, ".mjz") || !strcmp(ext, ".zip")) : 0;
  };
  decoder.decode = +[](mjResource* resource, const mjVFS* vfs) -> mjSpec* {
    if (!resource || !resource->name) { return nullptr; }

    mujoco::user::FilePath path(resource->name);
    std::string            archive_path = path.Str();
    std::string            stem         = path.StripPath().StripExt().Str();

    std::array<std::string, 4> candidates = {
        archive_path + "/model.xml",
        archive_path + "/" + stem + ".xml",
        archive_path + "/" + stem + "/" + stem + ".xml",
        archive_path + "/" + stem + "/model.xml",
    };

    // If no VFS is provided, create a local one so candidate probing and parsing share
    // the same mounted archive rather than mounting/unmounting per candidate.
    mjVFS                 local_vfs;
    mujoco::user::Cleanup cleanup;
    if (!vfs) {
      mj_defaultVFS(&local_vfs);
      cleanup += [&local_vfs]() { mj_deleteVFS(&local_vfs); };
      vfs      = &local_vfs;
    }

    const std::string* found_candidate = nullptr;
    for (const auto& candidate : candidates) {
      mjResource* res = mju_openResource("", candidate.c_str(), vfs, nullptr, 0);
      if (res) {
        mju_closeResource(res);
        found_candidate = &candidate;
        break;
      }
    }

    mjSpec* spec = nullptr;
    if (found_candidate) {
      char error[1024] = "";
      spec             = mj_parseXML(found_candidate->c_str(), vfs, error, sizeof(error));
      if (!spec && error[0]) { mju_warning("%s", error); }
    }

    return spec;
  };
  mjp_registerDecoder(&decoder);
}
