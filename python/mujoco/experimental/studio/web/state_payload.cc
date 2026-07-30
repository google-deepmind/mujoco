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

// Wire-format implementation for the web viewer's state payload (see
// state_payload.h). Compiled into BOTH sides of the wire: the state_payload
// pybind module (serializer, via state_payload_py.cc) and the wasm
// web_client (parser, via web_client_session.cc). It must therefore stay
// free of python- or browser-specific dependencies.

#include "state_payload.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco::studio {
namespace {

// Appends raw bytes to the payload buffer.
void AppendBytes(std::vector<std::byte>& buffer, const void* data,
                 size_t size) {
  const std::byte* bytes = static_cast<const std::byte*>(data);
  buffer.insert(buffer.end(), bytes, bytes + size);
}

// Appends a complete [u32 tag][u32 size][payload] block.
void AppendStateBlock(std::vector<std::byte>& buffer, uint32_t tag,
                      const void* data, size_t size) {
  StateBlockHeader block_header{tag, static_cast<uint32_t>(size)};
  AppendBytes(buffer, &block_header, sizeof(block_header));
  AppendBytes(buffer, data, size);
}

// Serializes the render state (exactly kRenderStateSize bytes) into `ptr`.
void SerializeRenderStateInto(std::byte* ptr, const mjvCamera& camera,
                              const mjvPerturb& perturb,
                              const mjvOption& vis_options, const mjOption& opt,
                              const mjVisual& vis, const mjStatistic& stat,
                              const std::vector<uint8_t>& render_flags) {
  memcpy(ptr, &camera, sizeof(mjvCamera));
  ptr += sizeof(mjvCamera);

  memcpy(ptr, &perturb, sizeof(mjvPerturb));
  ptr += sizeof(mjvPerturb);

  memcpy(ptr, &vis_options, sizeof(mjvOption));
  ptr += sizeof(mjvOption);

  memcpy(ptr, &opt, sizeof(mjOption));
  ptr += sizeof(mjOption);

  memcpy(ptr, &vis, sizeof(mjVisual));
  ptr += sizeof(mjVisual);

  memcpy(ptr, &stat, sizeof(mjStatistic));
  ptr += sizeof(mjStatistic);

  // Pack render flags (mjNRNDFLAG bytes).
  memset(ptr, 0, mjNRNDFLAG);
  for (size_t i = 0; i < mjNRNDFLAG && i < render_flags.size(); ++i) {
    ptr[i] = static_cast<std::byte>(render_flags[i]);
  }
}

}  // namespace

size_t MaxStatePayloadSize(size_t physics_bytes) {
  return sizeof(StatePayloadHeader) + 3 * sizeof(StateBlockHeader) +
         (sizeof(int32_t) + physics_bytes) + kRenderStateSize +
         kMaxExtraGeoms * sizeof(mjvGeom);
}

std::vector<std::byte> SerializeStatePayload(
    uint32_t model_crc32, int32_t physics_spec, const void* physics,
    size_t physics_bytes, const mjvCamera& camera, const mjvPerturb& perturb,
    const mjvOption& vis_options, const mjOption& opt, const mjVisual& vis,
    const mjStatistic& stat, const std::vector<uint8_t>& render_flags,
    const mjvGeom* extra_geoms, size_t extra_geom_count) {
  extra_geom_count =
      extra_geom_count > kMaxExtraGeoms ? kMaxExtraGeoms : extra_geom_count;
  std::vector<std::byte> buffer;
  buffer.reserve(MaxStatePayloadSize(physics_bytes));

  StatePayloadHeader header;
  header.nblocks = extra_geom_count > 0 ? 3 : 2;
  header.model_crc32 = model_crc32;
  AppendBytes(buffer, &header, sizeof(header));

  // Physics state: [i32 spec][mjtNum values...].
  StateBlockHeader physics_header{
      kTagPhysicsState, static_cast<uint32_t>(sizeof(int32_t) + physics_bytes)};
  AppendBytes(buffer, &physics_header, sizeof(physics_header));
  AppendBytes(buffer, &physics_spec, sizeof(int32_t));
  AppendBytes(buffer, physics, physics_bytes);

  // Render state, serialized into place.
  StateBlockHeader render_header{kTagRenderState,
                                 static_cast<uint32_t>(kRenderStateSize)};
  AppendBytes(buffer, &render_header, sizeof(render_header));
  const size_t render_offset = buffer.size();
  buffer.resize(render_offset + kRenderStateSize);
  SerializeRenderStateInto(buffer.data() + render_offset, camera, perturb,
                           vis_options, opt, vis, stat, render_flags);

  // Extra geoms (only when present).
  if (extra_geom_count > 0) {
    AppendStateBlock(buffer, kTagExtraGeoms, extra_geoms,
                     extra_geom_count * sizeof(mjvGeom));
  }

  return buffer;
}

bool ParseStatePayload(const void* data, size_t size, StatePayloadView* out) {
  const std::byte* bytes = static_cast<const std::byte*>(data);
  if (size < sizeof(StatePayloadHeader)) return false;

  StatePayloadHeader header;
  memcpy(&header, bytes, sizeof(header));
  if (header.magic != kStatePayloadMagic) return false;
  if (header.version != kStatePayloadVersion) return false;
  out->model_crc32 = header.model_crc32;

  size_t offset = sizeof(StatePayloadHeader);
  for (uint16_t i = 0; i < header.nblocks; ++i) {
    if (offset + sizeof(StateBlockHeader) > size) return false;
    StateBlockHeader block;
    memcpy(&block, bytes + offset, sizeof(block));
    offset += sizeof(StateBlockHeader);
    if (offset + block.size > size) return false;
    const std::byte* payload = bytes + offset;

    switch (block.tag) {
      case kTagPhysicsState:
        if (block.size < sizeof(int32_t)) return false;
        memcpy(&out->physics_spec, payload, sizeof(int32_t));
        out->physics = payload + sizeof(int32_t);
        out->physics_bytes = block.size - sizeof(int32_t);
        break;
      case kTagRenderState:
        if (block.size != kRenderStateSize) return false;
        out->render_state = payload;
        break;
      case kTagExtraGeoms:
        if (block.size % sizeof(mjvGeom) != 0) return false;
        out->extra_geoms = payload;
        out->extra_geom_count = block.size / sizeof(mjvGeom);
        break;
      default:
        break;  // Unknown tag: skip.
    }
    offset += block.size;
  }
  return true;
}

}  // namespace mujoco::studio
