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

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include <filament/Box.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/RenderableManager.h>
#include <filament/VertexBuffer.h>
#include <math/vec4.h>

// Functions for creating filament vertex and index buffers.
namespace mujoco {

// The type of data stored in an index buffer.
typedef enum mjtIndexType_ {
  mjINDEX_TYPE_USHORT = 0,
  mjINDEX_TYPE_UINT = 1,
} mjtIndexType;

// The type of primitive to be drawn by vertex data.
typedef enum mjtMeshPrimitiveType_ {
  mjPRIM_TYPE_TRIANGLES = 0,
  mjPRIM_TYPE_LINES = 1,
} mjtMeshPrimitiveType;

// The usage/purpose of an attribute of a vertex.
typedef enum mjtVertexAttributeUsage_ {
  mjVERTEX_ATTRIBUTE_POSITION = 0,
  mjVERTEX_ATTRIBUTE_NORMAL = 1,
  mjVERTEX_ATTRIBUTE_TANGENTS = 2,
  mjVERTEX_ATTRIBUTE_UV = 3,
  mjVERTEX_ATTRIBUTE_COLOR = 4,
} mjtVertexAttributeUsage;

// The data format of an attribute of a vertex.
typedef enum mjtVertexAttributeType_ {
  mjVERTEX_ATTRIBUTE_TYPE_FLOAT2 = 0,
  mjVERTEX_ATTRIBUTE_TYPE_FLOAT3 = 1,
  mjVERTEX_ATTRIBUTE_TYPE_FLOAT4 = 2,
  mjVERTEX_ATTRIBUTE_TYPE_UBYTE4 = 3,
} mjtVertexAttributeType;

// Information about a single attribute of a vertex.
struct VertexAttribute {
  // The data for the attribute.
  const void* bytes;

  // The usage/purpose of the attribute.
  mjtVertexAttributeUsage usage;

  // The data format of the attribute.
  mjtVertexAttributeType type;
};

// The binary contents of a mesh.
struct MeshData {
  // The number of vertices in the mesh. Each of the vertex arrays below is
  // assumed to have this number of elements.
  size_t nvertices;

  // The number of attributes for each vertex in the mesh.
  int nattributes;

  // Information about each attribute of a vertex in the mesh. See `interleaved`
  // for more details.
  VertexAttribute attributes[16];

  // Whether the vertex attributes are interleaved or not.
  //
  // If true, assumes that the attributes are packed in the order specified in
  // the attributes array, with no padding in-between. Additionally, the
  // `data` pointer for each attribute is assumed to point to the first element
  // of that type.
  //
  // If false, assume each attribute is stored in a separate array as defined
  // by the `data` field of the attribute.
  bool interleaved;

  // The number of indices in the mesh. The indices array is assumed to have
  // this number of elements.
  size_t nindices;

  // The indices of the mesh, stored as either ushort or uint depending on the
  // index type.
  const void* indices;

  // The type of data stored in the indices array.
  mjtIndexType index_type;

  // The type of primitive to be drawn by vertex data.
  mjtMeshPrimitiveType primitive_type;

  // Whether to compute the bounds of the mesh using the vertex positions.
  bool compute_bounds;

  // The bounds of the mesh. If bounds_min == bounds_max, then we assume that
  // that the bounds are not set (i.e. the bounds is empty).
  float bounds_min[3];
  float bounds_max[3];

  // Because rendering may be multithreaded, we cannot make assumptions about
  // when the mesh data will finish uploading to the GPU. As such, we will use
  // this callback to notify callers when it is safe to free the mesh data.
  void (*release_callback)(void* user_data);

  // User data to pass to the release callback.
  void* user_data;
};

// Initializes the MeshData to default values.
void DefaultMeshData(MeshData* data);

// Owns a Vertex and Index buffer representing a geometry mesh.
class Mesh {
 public:
  // Creates a Mesh from the given MeshData.
  Mesh(filament::Engine* engine, const MeshData& data);

  ~Mesh();

  // Returns the filament IndexBuffer for the mesh.
  filament::IndexBuffer* GetFilamentIndexBuffer() const;

  // Returns the filament VertexBuffer for the mesh.
  filament::VertexBuffer* GetFilamentVertexBuffer() const;

  // Returns the primitive type of the mesh.
  filament::RenderableManager::PrimitiveType GetPrimitiveType() const;

  // Returns whether the mesh has bounds.
  bool HasBounds() const;

  // Returns the bounds of the mesh.
  filament::Box GetBounds() const;

  Mesh(const Mesh&) = delete;
  Mesh& operator=(const Mesh&) = delete;

 private:
  void BuildVertexBuffer(const MeshData& data);
  void BuildIndexBuffer(const MeshData& data);
  void UpdateBounds(const MeshData& data);

  filament::math::float4* BuildOrientationsFromNormals(
      int nvertices, const VertexAttribute& normals);

  void ReleaseResources();

  filament::Engine* engine_ = nullptr;
  filament::IndexBuffer* index_buffer_ = nullptr;
  filament::VertexBuffer* vertex_buffer_ = nullptr;
  filament::RenderableManager::PrimitiveType type_ =
      filament::RenderableManager::PrimitiveType::TRIANGLES;
  std::optional<filament::Box> bounds_;
  std::vector<std::function<void()>> release_callbacks_;
};

using MeshPtr = std::unique_ptr<Mesh>;

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MESH_H_
