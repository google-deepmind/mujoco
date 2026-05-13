// Copyright 2025 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MESH_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MESH_H_

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <vector>

#include <filament/Box.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/RenderableManager.h>
#include <filament/VertexBuffer.h>
#include <math/vec4.h>
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// Owns a filament Vertex and Index buffer representing a geometry mesh.
class Mesh : public mjrMesh {
 public:
  // Creates a Mesh from the given MeshData.
  Mesh(filament::Engine* engine, const mjrMeshData& data);
  ~Mesh();

  Mesh(const Mesh&) = delete;
  Mesh& operator=(const Mesh&) = delete;

  // Returns the filament IndexBuffer for the mesh.
  filament::IndexBuffer* GetFilamentIndexBuffer() const;

  // Returns the filament VertexBuffer for the mesh.
  filament::VertexBuffer* GetFilamentVertexBuffer() const;

  // Returns the primitive type of the mesh.
  filament::RenderableManager::PrimitiveType GetPrimitiveType() const;

  // Returns the vertex attribute usages for the mesh.
  std::span<const filament::VertexAttribute> GetVertexAttributes() const;

  // Returns whether the mesh has bounds.
  bool HasBounds() const;

  // Returns the bounds of the mesh.
  filament::Box GetBounds() const;

  static Mesh* downcast(mjrMesh* mesh) {
    return static_cast<Mesh*>(mesh);
  }
  static const Mesh* downcast(const mjrMesh* mesh) {
    return static_cast<const Mesh*>(mesh);
  }

 private:
  void BuildVertexBuffer(const mjrMeshData& data);
  void BuildIndexBuffer(const mjrMeshData& data);
  void UpdateBounds(const mjrMeshData& data);

  filament::math::float4* BuildOrientationsFromNormals(
      int nvertices, const mjrVertexAttribute& normals);

  void ReleaseResources();

  filament::Engine* engine_ = nullptr;
  filament::IndexBuffer* index_buffer_ = nullptr;
  filament::VertexBuffer* vertex_buffer_ = nullptr;
  filament::RenderableManager::PrimitiveType type_ =
      filament::RenderableManager::PrimitiveType::TRIANGLES;
  std::optional<filament::Box> bounds_;
  struct SharedState {
    std::vector<std::function<void()>> callbacks;
    std::mutex mutex;
    bool called = false;
  };
  std::shared_ptr<SharedState> shared_state_;
  std::array<filament::VertexAttribute, mjMAX_VERTEX_ATTRIBUTES> attributes_;
  int num_attributes_ = 0;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MESH_H_
