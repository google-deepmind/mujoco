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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_BUFFER_UTIL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_BUFFER_UTIL_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <type_traits>
#include <backend/BufferDescriptor.h>
#include <filament/Box.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/RenderableManager.h>
#include <filament/VertexBuffer.h>

// Functions for creating filament vertex and index buffers.
namespace mujoco {

// Simple tuple-type of a IndexBuffer+VertexBuffer.
struct FilamentBuffers {
  filament::IndexBuffer* index_buffer = nullptr;
  filament::VertexBuffer* vertex_buffer = nullptr;
  filament::Box bounds = {{-1, -1, -1}, {1, 1, 1}};
  filament::RenderableManager::PrimitiveType type =
      filament::RenderableManager::PrimitiveType::TRIANGLES;
};

// Function that fills in the given buffer with actual data.
using FillBufferFn = std::function<void(std::byte*, std::size_t)>;

// Creates and populates a BufferDescriptor (for vertex and index buffers).
filament::backend::BufferDescriptor CreateBufferDescriptor(
    std::size_t num_bytes, const FillBufferFn& fill);

// Creates a filament::VertexBuffer based on the VertexType. The fill function
// will be used to populate the buffer.
template <typename VertexType>
filament::VertexBuffer* CreateVertexBuffer(filament::Engine* engine,
                                           std::size_t num_vertices,
                                           const FillBufferFn& fill) {
  int vertex_size = 0;
  if constexpr (VertexType::kHasPosition) {
    vertex_size += sizeof(VertexType::position);
  }
  if constexpr (VertexType::kHasPosition2d) {
    vertex_size += sizeof(VertexType::position);
  }
  if constexpr (VertexType::kHasOrientation) {
    vertex_size += sizeof(VertexType::orientation);
  }
  if constexpr (VertexType::kHasUv) {
    vertex_size += sizeof(VertexType::uv);
  }
  if constexpr (VertexType::kHasColor) {
    vertex_size += sizeof(VertexType::color);
  }

  auto builder = filament::VertexBuffer::Builder();
  builder.bufferCount(1);
  builder.vertexCount(num_vertices);

  int offset = 0;
  if constexpr (VertexType::kHasPosition) {
    builder.attribute(filament::VertexAttribute::POSITION, 0,
                      filament::VertexBuffer::AttributeType::FLOAT3, offset,
                      vertex_size);
    offset += sizeof(VertexType::position);
  }
  if constexpr (VertexType::kHasPosition2d) {
    builder.attribute(filament::VertexAttribute::POSITION, 0,
                      filament::VertexBuffer::AttributeType::FLOAT2, offset,
                      vertex_size);
    offset += sizeof(VertexType::position);
  }
  if constexpr (VertexType::kHasOrientation) {
    builder.attribute(filament::VertexAttribute::TANGENTS, 0,
                      filament::VertexBuffer::AttributeType::FLOAT4, offset,
                      vertex_size);
    offset += sizeof(VertexType::orientation);
  }
  if constexpr (VertexType::kHasUv) {
    builder.attribute(filament::VertexAttribute::UV0, 0,
                      filament::VertexBuffer::AttributeType::FLOAT2, offset,
                      vertex_size);
    offset += sizeof(VertexType::uv);
  }
  if constexpr (VertexType::kHasColor) {
    builder.attribute(filament::VertexAttribute::COLOR, 0,
                      filament::VertexBuffer::AttributeType::UBYTE4, offset,
                      vertex_size);
    builder.normalized(filament::VertexAttribute::COLOR);
    offset += sizeof(VertexType::color);
  }

  auto vb = builder.build(*engine);
  const std::size_t buffer_size = num_vertices * vertex_size;
  vb->setBufferAt(*engine, 0, CreateBufferDescriptor(buffer_size, fill));
  return vb;
}

// Creates a filament::IndexBuffer. The IndexType should be either uin16_t or
// uint32_t. The fill function will be used to populate the buffer.
template <typename IndexType>
filament::IndexBuffer* CreateIndexBuffer(filament::Engine* engine,
                                         std::size_t num_indices,
                                         const FillBufferFn& fill) {
  static_assert(std::is_same<IndexType, uint16_t>::value ||
                std::is_same<IndexType, uint32_t>::value);

  constexpr auto type = std::is_same<IndexType, uint16_t>::value
                            ? filament::IndexBuffer::IndexType::USHORT
                            : filament::IndexBuffer::IndexType::UINT;

  auto builder = filament::IndexBuffer::Builder();
  builder.bufferType(type);
  builder.indexCount(num_indices);

  auto ib = builder.build(*engine);

  const std::size_t buffer_size = num_indices * sizeof(IndexType);
  ib->setBuffer(*engine, CreateBufferDescriptor(buffer_size, fill));
  return ib;
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

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_BUFFER_UTIL_H_
