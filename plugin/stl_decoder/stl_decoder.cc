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

#include <cmath>
#include <cstring>
#include <map>
#include <string_view>
#include <vector>

#include <mujoco/mjplugin.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>

namespace {

template <typename T>
void ReadFromBuffer(T* dst, const char* src) {
  std::memcpy(dst, src, sizeof(T));
}

struct Vec3Key {
  int x, y, z;
  bool operator<(const Vec3Key& o) const {
    if (x != o.x) return x < o.x;
    if (y != o.y) return y < o.y;
    return z < o.z;
  }
};

Vec3Key FloatToKey(const float v[3]) {
  int x, y, z;
  std::memcpy(&x, &v[0], sizeof(int));
  std::memcpy(&y, &v[1], sizeof(int));
  std::memcpy(&z, &v[2], sizeof(int));
  return {x, y, z};
}

mjSpec* Decode(mjResource* resource, const mjVFS* vfs) {
  const void* bytes = nullptr;
  int buffer_sz = mju_readResource(resource, &bytes);
  if (buffer_sz < 0) {
    mju_warning("stl_decoder: could not read STL file '%s'", resource->name);
    return nullptr;
  }
  if (!buffer_sz) {
    mju_warning("stl_decoder: STL file '%s' is empty", resource->name);
    return nullptr;
  }

  const char* buffer = static_cast<const char*>(bytes);

  if (buffer_sz < 84) {
    mju_warning("stl_decoder: invalid header in STL file '%s'", resource->name);
    return nullptr;
  }

  int nfaces = 0;
  ReadFromBuffer(&nfaces, buffer + 80);
  if (nfaces < 1 || nfaces > 200000) {
    mju_warning(
        "stl_decoder: number of faces should be between 1 and 200000 in STL "
        "file '%s'; perhaps this is an ASCII file?",
        resource->name);
    return nullptr;
  }

  if (nfaces * 50 != buffer_sz - 84) {
    mju_warning(
        "stl_decoder: STL file '%s' has wrong size; perhaps this is an ASCII "
        "file?",
        resource->name);
    return nullptr;
  }

  const char* stl = buffer + 84;

  std::vector<float> uservert;
  std::vector<int> userface(3 * nfaces, 0);
  std::map<Vec3Key, int> vertmap;

  for (int i = 0; i < nfaces; i++) {
    for (int j = 0; j < 3; j++) {
      float v[3];
      ReadFromBuffer(&v, stl + 50 * i + 12 * (j + 1));

      if (std::fabs(v[0]) > std::pow(2, 30) ||
          std::fabs(v[1]) > std::pow(2, 30) ||
          std::fabs(v[2]) > std::pow(2, 30)) {
        mju_warning(
            "stl_decoder: vertex in STL file '%s' "
            "exceeds maximum bounds",
            resource->name);
        return nullptr;
      }

      Vec3Key key = FloatToKey(v);
      auto [it, inserted] = vertmap.emplace(key, uservert.size() / 3);
      if (inserted) {
        uservert.push_back(v[0]);
        uservert.push_back(v[1]);
        uservert.push_back(v[2]);
      }
      userface[3 * i + j] = it->second;
    }
  }

  mjSpec* spec = mj_makeSpec();
  mjsMesh* mesh = mjs_addMesh(spec, nullptr);

  mjs_setString(mesh->file, resource->name);
  mjs_setFloat(mesh->uservert, uservert.data(), uservert.size());
  mjs_setInt(mesh->userface, userface.data(), userface.size());

  return spec;
}

int CanDecode(const mjResource* resource) {
  std::string_view name(resource->name);
  return name.ends_with(".stl") || name.ends_with(".STL");
}

}  // namespace

mjPLUGIN_LIB_INIT {
  mjpDecoder decoder;
  mjp_defaultDecoder(&decoder);
  decoder.content_type = "model/stl";
  decoder.extension = ".stl";
  decoder.decode = Decode;
  decoder.can_decode = CanDecode;
  mjp_registerDecoder(&decoder);
}
