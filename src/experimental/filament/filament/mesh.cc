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

#include "experimental/filament/filament/mesh.h"

#include <cfloat>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

#include <filament/Box.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/VertexBuffer.h>
#include <backend/BufferDescriptor.h>
#include <math/TVecHelpers.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/vertex_util.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;

static filament::VertexAttribute GetUsage(const VertexAttribute& attrib) {
  switch (attrib.usage) {
    case mjVERTEX_ATTRIBUTE_POSITION:
      return filament::VertexAttribute::POSITION;
    case mjVERTEX_ATTRIBUTE_NORMAL:
      return filament::VertexAttribute::TANGENTS;
    case mjVERTEX_ATTRIBUTE_TANGENTS:
      return filament::VertexAttribute::TANGENTS;
    case mjVERTEX_ATTRIBUTE_UV:
      return filament::VertexAttribute::UV0;
    case mjVERTEX_ATTRIBUTE_COLOR:
      return filament::VertexAttribute::COLOR;
    default:
      mju_error("Unsupported vertex attribute usage: %d", attrib.usage);
      return filament::VertexAttribute::POSITION;
  }
}

static filament::VertexBuffer::AttributeType GetType(
    const VertexAttribute& attrib) {
  switch (attrib.type) {
    case mjVERTEX_ATTRIBUTE_TYPE_FLOAT2:
      return filament::VertexBuffer::AttributeType::FLOAT2;
    case mjVERTEX_ATTRIBUTE_TYPE_FLOAT3:
      return filament::VertexBuffer::AttributeType::FLOAT3;
    case mjVERTEX_ATTRIBUTE_TYPE_FLOAT4:
      return filament::VertexBuffer::AttributeType::FLOAT4;
    case mjVERTEX_ATTRIBUTE_TYPE_UBYTE4:
      return filament::VertexBuffer::AttributeType::UBYTE4;
    default:
      mju_error("Unsupported vertex attribute type: %d", attrib.type);
      return filament::VertexBuffer::AttributeType::FLOAT3;
  }
}

int VertexAttributeTypeSize(const VertexAttribute& attrib) {
  switch (attrib.type) {
    case mjVERTEX_ATTRIBUTE_TYPE_FLOAT2:
      return sizeof(float) * 2;
    case mjVERTEX_ATTRIBUTE_TYPE_FLOAT3:
      return sizeof(float) * 3;
    case mjVERTEX_ATTRIBUTE_TYPE_FLOAT4:
      return sizeof(float) * 4;
    case mjVERTEX_ATTRIBUTE_TYPE_UBYTE4:
      return sizeof(uint8_t) * 4;
    default:
      mju_error("Unsupported vertex attribute type: %d", attrib.type);
      return 0;
  }
}

// Fills an index buffer with a basic incrementing sequence.
template <typename T>
int FillSequence(std::byte* buffer, std::size_t num_bytes) {
  const T num = num_bytes / sizeof(T);
  T* ptr = reinterpret_cast<T*>(buffer);
  for (T i = 0; i < num; ++i) {
    ptr[i] = i;
  }
  return num;
}

// Initializes the MeshData to default values.
void DefaultMeshData(MeshData* data) {
  std::memset(data, 0, sizeof(MeshData));
}

Mesh::Mesh(filament::Engine* engine, const MeshData& data)
    : engine_(engine) {
  type_ = data.primitive_type == mjPRIM_TYPE_TRIANGLES
              ? filament::RenderableManager::PrimitiveType::TRIANGLES
              : filament::RenderableManager::PrimitiveType::LINES;

  // If the user has provided a release callback, then we need to ensure we
  // call is when filament is done with the mesh data.
  if (data.release_callback) {
    release_callbacks_.push_back([=]() {
      data.release_callback(data.user_data);
    });
  }

  BuildVertexBuffer(data);
  BuildIndexBuffer(data);
  UpdateBounds(data);
}

Mesh::~Mesh() {
  ReleaseResources();
  if (index_buffer_) {
    engine_->destroy(index_buffer_);
  }
  if (vertex_buffer_) {
    engine_->destroy(vertex_buffer_);
  }
}

void Mesh::BuildVertexBuffer(const MeshData& data) {
  if (data.nvertices == 0) {
    mju_error("MeshData has no vertices.");
  }

  // The filament BufferDescriptor callback for releasing the memory. We assume
  // that ReleaseResources() can be called multiple times, so we assign this
  // callback to each buffer descriptor.
  auto callback = +[](void* buffer, size_t size, void* user) {
    static_cast<Mesh*>(user)->ReleaseResources();
  };

  // Pointers to specific attributes in the mesh data, used for additional
  // validation and processing.
  const VertexAttribute* positions = nullptr;
  const VertexAttribute* normals = nullptr;
  const VertexAttribute* tangents = nullptr;
  for (int i = 0; i < data.nattributes; ++i) {
    if (data.attributes[i].usage == mjVERTEX_ATTRIBUTE_POSITION) {
      positions = &data.attributes[i];
    } else if (data.attributes[i].usage == mjVERTEX_ATTRIBUTE_NORMAL) {
      normals = &data.attributes[i];
    } else if (data.attributes[i].usage == mjVERTEX_ATTRIBUTE_TANGENTS) {
      tangents = &data.attributes[i];
    }
  }
  if (!positions) {
    mju_error("MeshData has no positions.");
  }
  if (data.attributes[0].usage != mjVERTEX_ATTRIBUTE_POSITION) {
    mju_error("Positions must be the first attribute.");
  }
  if (normals && tangents) {
    mju_error("MeshData has both normals and tangents.");
  }
  if (normals && data.interleaved) {
    // We need to build orientations from normals and so we require each
    // attribute to be in a separate buffer.
    mju_error("Cannot support normals with interleaved vertex attributes.");
  }

  // Build the vertex buffer.
  filament::VertexBuffer::Builder vb_builder;
  vb_builder.vertexCount(data.nvertices);

  if (data.interleaved) {
    // For an interleaved vertex buffer, we will create a single buffer which
    // contains the data in the order specified by the attributes array,
    // starting from the first attribute's payload.
    vb_builder.bufferCount(1);
    int total_vertex_size = 0;
    for (int i = 0; i < data.nattributes; ++i) {
      total_vertex_size += VertexAttributeTypeSize(data.attributes[i]);
    }
    const void* bytes = data.attributes[0].bytes;
    const size_t nbytes = data.nvertices * total_vertex_size;

    // We assume the buffer is tightly packed with no padding between
    // attributes. As such, the stride is equal to the total vertex size and
    // each offset is the sum of the sizes of the preceding attributes.
    int offset = 0;
    for (int i = 0; i < data.nattributes; ++i) {
      const VertexAttribute& attrib = data.attributes[i];
      const filament::VertexAttribute usage = GetUsage(attrib);
      filament::VertexBuffer::AttributeType type = GetType(attrib);
      vb_builder.attribute(usage, 0, type, offset, total_vertex_size);
      if (usage == filament::VertexAttribute::COLOR) {
        vb_builder.normalized(usage);
      }
      offset += VertexAttributeTypeSize(attrib);
    }
    vertex_buffer_ = vb_builder.build(*engine_);
    vertex_buffer_->setBufferAt(*engine_, 0, {bytes, nbytes, callback, this});
  } else {
    // For a non-interleaved vertex buffer, we assign a separate buffer to each
    // attribute.
    vb_builder.bufferCount(data.nattributes);
    for (int i = 0; i < data.nattributes; ++i) {
      const VertexAttribute& attrib = data.attributes[i];
      const filament::VertexAttribute usage = GetUsage(attrib);
      filament::VertexBuffer::AttributeType type = GetType(attrib);
      if (attrib.usage == mjVERTEX_ATTRIBUTE_NORMAL) {
        // We will replace normals with orientations.
        type = filament::VertexBuffer::AttributeType::FLOAT4;
      }
      vb_builder.attribute(usage, i, type);
      if (usage == filament::VertexAttribute::COLOR) {
        vb_builder.normalized(usage);
      }
    }
    vertex_buffer_ = vb_builder.build(*engine_);

    // Assign the individual data buffers.
    for (int i = 0; i < data.nattributes; ++i) {
      const VertexAttribute& attrib = data.attributes[i];
      const void* bytes = attrib.bytes;
      size_t nbytes = data.nvertices * VertexAttributeTypeSize(attrib);
      if (attrib.usage == mjVERTEX_ATTRIBUTE_NORMAL) {
        // Replace normals with orientations.
        nbytes = data.nvertices * sizeof(float4);
        bytes = BuildOrientationsFromNormals(data.nvertices, attrib);
      }
      vertex_buffer_->setBufferAt(*engine_, i, {bytes, nbytes, callback, this});
    }
  }
}

void Mesh::BuildIndexBuffer(const MeshData& data) {
  if (data.nindices == 0) {
    return;
  }

  const int element_size = data.index_type == mjINDEX_TYPE_USHORT
                                ? sizeof(uint16_t)
                                : sizeof(uint32_t);
  const int num_bytes = data.nindices * element_size;

  // If indices == 0 and nindices > 0, then the user is specifying that the
  // vertices are provided "in order", i.e. the indices are 0, 1, 2, 3, ...
  // In this case, we need to create the sequence of indices explicitly.
  const void* indices = data.indices;
  if (indices == nullptr) {
    std::byte* sequence = new std::byte[num_bytes];
    release_callbacks_.push_back([=]() {
      delete[] sequence;
    });

    if (data.index_type == mjINDEX_TYPE_USHORT) {
      FillSequence<uint16_t>(sequence, num_bytes);
    } else {
      FillSequence<uint32_t>(sequence, num_bytes);
    }
    indices = sequence;
  }

  filament::IndexBuffer::Builder ib_builder;
  ib_builder.indexCount(data.nindices);
  ib_builder.bufferType(data.index_type == mjINDEX_TYPE_USHORT
                            ? filament::IndexBuffer::IndexType::USHORT
                            : filament::IndexBuffer::IndexType::UINT);
  index_buffer_ = ib_builder.build(*engine_);
  // We don't worry about setting a release callback here because the release
  // callback for the vertex buffer will call release_callbacks_.
  filament::backend::BufferDescriptor desc(indices, num_bytes);
  index_buffer_->setBuffer(*engine_, std::move(desc));
}

float4* Mesh::BuildOrientationsFromNormals(int nvertices, const VertexAttribute& normals) {
  float4* orientations = new float4[nvertices];
  release_callbacks_.push_back([=]() {
    delete[] orientations;
  });
  const float* normals_ptr = reinterpret_cast<const float*>(normals.bytes);
  for (int i = 0; i < nvertices; ++i) {
    orientations[i] = CalculateOrientation(ReadFloat3(normals_ptr, i));
  }
  return orientations;
}

void Mesh::UpdateBounds(const MeshData& data) {
  float3 bounds_min = ReadFloat3(data.bounds_min);
  float3 bounds_max = ReadFloat3(data.bounds_max);
  if (bounds_min != bounds_max) {
    bounds_.emplace().set(bounds_min, bounds_max);
  } else if (data.compute_bounds) {
    bounds_min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
    bounds_max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    if (data.attributes[0].usage != mjVERTEX_ATTRIBUTE_POSITION) {
      mju_error("MeshData has no positions.");
    }
    const float* positions =
        reinterpret_cast<const float*>(data.attributes[0].bytes);

    for (int i = 0; i < data.nvertices; ++i) {
      const float3 position = ReadFloat3(positions, i);
      bounds_min = min(bounds_min, position);
      bounds_max = max(bounds_max, position);
    }
    bounds_.emplace().set(bounds_min, bounds_max);
  }
}

void Mesh::ReleaseResources() {
  for (const auto& callback : release_callbacks_) {
    callback();
  }
  release_callbacks_.clear();
}

filament::IndexBuffer* Mesh::GetFilamentIndexBuffer() const {
  return index_buffer_;
}

filament::VertexBuffer* Mesh::GetFilamentVertexBuffer() const {
  return vertex_buffer_;
}

filament::RenderableManager::PrimitiveType Mesh::GetPrimitiveType() const {
  return type_;
}

bool Mesh::HasBounds() const {
  return bounds_.has_value();
}

filament::Box Mesh::GetBounds() const {
  return bounds_.value();
}
}  // namespace mujoco
