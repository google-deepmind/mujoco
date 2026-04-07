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

#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include <mujoco/mjplugin.h>
#include <mujoco/mjspec.h>
#include <mujoco/mujoco.h>
#include <tiny_obj_loader.h>

namespace {

mjSpec* Decode(mjResource* resource, const mjVFS* vfs) {
  const void* bytes = nullptr;
  int buffer_sz = mju_readResource(resource, &bytes);
  if (buffer_sz < 0) {
    mju_warning("obj_decoder: could not read OBJ file '%s'", resource->name);
    return nullptr;
  }

  tinyobj::ObjReader obj_reader;
  const char* buffer = static_cast<const char*>(bytes);
  obj_reader.ParseFromString(std::string(buffer, buffer_sz), std::string());

  if (!obj_reader.Valid()) {
    mju_warning("obj_decoder: could not parse OBJ file '%s'", resource->name);
    return nullptr;
  }

  mjSpec* spec = mj_makeSpec();
  mjsMesh* mesh = mjs_addMesh(spec, nullptr);

  const auto& attrib = obj_reader.GetAttrib();

  std::vector<float> usernormal = attrib.normals;
  std::vector<float> usertexcoord = attrib.texcoords;
  std::vector<int> userface;
  std::vector<int> userfacenormal;
  std::vector<int> userfacetexcoord;

  if (!obj_reader.GetShapes().empty()) {
    const auto& obj_mesh = obj_reader.GetShapes()[0].mesh;

    std::vector<tinyobj::index_t> face_indices;
    for (size_t face = 0, idx = 0; idx < obj_mesh.indices.size();) {
      int nfacevert = obj_mesh.num_face_vertices[face];
      if (nfacevert < 3 || nfacevert > 4) {
        mju_warning(
            "obj_decoder: only tri or quad meshes are supported (file '%s')",
            resource->name);
        mj_deleteSpec(spec);
        return nullptr;
      }

      face_indices.push_back(obj_mesh.indices[idx]);
      face_indices.push_back(obj_mesh.indices[idx + 1]);
      face_indices.push_back(obj_mesh.indices[idx + 2]);

      if (nfacevert == 4) {
        face_indices.push_back(obj_mesh.indices[idx]);
        face_indices.push_back(obj_mesh.indices[idx + 2]);
        face_indices.push_back(obj_mesh.indices[idx + 3]);
      }
      idx += nfacevert;
      ++face;
    }

    for (const auto& mesh_index : face_indices) {
      userface.push_back(mesh_index.vertex_index);

      if (!usernormal.empty()) {
        userfacenormal.push_back(mesh_index.normal_index);
      }

      if (!usertexcoord.empty()) {
        userfacetexcoord.push_back(mesh_index.texcoord_index);
      }
    }
  }

  for (size_t i = 0; i < usertexcoord.size() / 2; i++) {
    usertexcoord[2 * i + 1] = 1 - usertexcoord[2 * i + 1];
  }

  mjs_setString(mesh->file, resource->name);
  mjs_setFloat(mesh->uservert, attrib.vertices.data(), attrib.vertices.size());
  mjs_setFloat(mesh->usernormal, usernormal.data(), usernormal.size());
  mjs_setFloat(mesh->usertexcoord, usertexcoord.data(), usertexcoord.size());
  mjs_setInt(mesh->userface, userface.data(), userface.size());
  mjs_setInt(mesh->userfacenormal, userfacenormal.data(), userfacenormal.size());
  mjs_setInt(mesh->userfacetexcoord, userfacetexcoord.data(), userfacetexcoord.size());

  return spec;
}

int CanDecode(const mjResource* resource) {
  std::string_view name(resource->name);
  return name.ends_with(".obj") || name.ends_with(".OBJ");
}

}  // namespace

mjPLUGIN_LIB_INIT {
  mjpDecoder decoder;
  mjp_defaultDecoder(&decoder);
  decoder.content_type = "model/obj";
  decoder.extension = ".obj";
  decoder.decode = Decode;
  decoder.can_decode = CanDecode;
  mjp_registerDecoder(&decoder);
}
