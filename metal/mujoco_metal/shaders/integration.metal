// Copyright 2026 The MuJoCo Metal contributors
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

#include <metal_stdlib>
using namespace metal;

// Inspect exponent bits because shader compilation may enable fast-math.
inline bool finite_float(float value) {
  return (as_type<uint>(value) & 0x7f800000u) != 0x7f800000u;
}

inline float4 quat_multiply(float4 a, float4 b) {
  return float4(
      a.x * b.x - dot(a.yzw, b.yzw),
      a.x * b.yzw + b.x * a.yzw + cross(a.yzw, b.yzw));
}

inline float4 quat_normalize_stable(float4 q) {
  float scale = max(max(abs(q.x), abs(q.y)), max(abs(q.z), abs(q.w)));
  float4 scaled = q / scale;
  return scaled * rsqrt(dot(scaled, scaled));
}

inline float4 quat_increment(float3 omega, float dt, thread bool& ok) {
  float scale = max(max(abs(omega.x), abs(omega.y)), abs(omega.z));
  if (scale == 0.0f) return float4(1.0f, 0.0f, 0.0f, 0.0f);

  float3 scaled = omega / scale;
  float scaled_norm = sqrt(dot(scaled, scaled));
  float dt_scale = dt * scale;
  float angle = dt_scale * scaled_norm;
  if (!finite_float(dt_scale) || !finite_float(angle)) {
    ok = false;
    return float4(1.0f, 0.0f, 0.0f, 0.0f);
  }

  float4 increment;
  if (abs(angle) < 0.0001f) {
    // Taylor series avoids losing the vector part for very small rotations.
    float angle2 = angle * angle;
    float angle4 = angle2 * angle2;
    float scalar = 1.0f - angle2 * 0.125f + angle4 * 0.002604166666666667f;
    float vector_scale = 0.5f - angle2 * 0.020833333333333333f +
                         angle4 * 0.000260416666666667f;
    increment = float4(scalar, scaled * (dt_scale * vector_scale));
  } else {
    float half_angle = 0.5f * angle;
    float axis_scale = sin(half_angle) / scaled_norm;
    increment = float4(cos(half_angle), scaled * axis_scale);
  }
  for (uint i = 0; i < 4; ++i) {
    if (!finite_float(increment[i])) ok = false;
  }
  return increment;
}

kernel void semi_implicit_euler(
    device const float* qpos [[buffer(0)]],
    device const float* qvel [[buffer(1)]],
    device const float* qacc [[buffer(2)]],
    device const float* time [[buffer(3)]],
    device const int* solve_status [[buffer(4)]],
    device const int* joint_type [[buffer(5)]],
    device const int* joint_qposadr [[buffer(6)]],
    device const int* joint_dofadr [[buffer(7)]],
    device float* candidate_qpos [[buffer(8)]],
    device float* candidate_qvel [[buffer(9)]],
    device float* candidate_time [[buffer(10)]],
    device float* output_qpos [[buffer(11)]],
    device float* output_qvel [[buffer(12)]],
    device float* output_time [[buffer(13)]],
    device int* output_status [[buffer(14)]],
    constant int* dims [[buffer(15)]],
    constant float* timestep [[buffer(16)]],
    uint world [[thread_position_in_grid]]) {
  uint nq = uint(dims[0]);
  uint nv = uint(dims[1]);
  uint njnt = uint(dims[2]);
  uint batch = uint(dims[3]);
  if (world >= batch) return;

  uint qb = world * nq;
  uint vb = world * nv;
  for (uint i = 0; i < nq; ++i) output_qpos[qb + i] = qpos[qb + i];
  for (uint i = 0; i < nv; ++i) output_qvel[vb + i] = qvel[vb + i];
  output_time[world] = time[world];

  int prior_status = solve_status[world];
  if (prior_status != 0) {
    output_status[world] = prior_status;
    return;
  }

  // Reject any nonfinite starting value before writing a candidate.
  for (uint i = 0; i < nq; ++i) {
    if (!finite_float(qpos[qb + i])) {
      output_status[world] = 10;
      return;
    }
  }
  for (uint i = 0; i < nv; ++i) {
    if (!finite_float(qvel[vb + i]) || !finite_float(qacc[vb + i])) {
      output_status[world] = 10;
      return;
    }
  }
  if (!finite_float(time[world])) {
    output_status[world] = 10;
    return;
  }

  float dt = timestep[0];
  for (uint i = 0; i < nq; ++i) candidate_qpos[qb + i] = qpos[qb + i];
  bool valid = true;
  for (uint i = 0; i < nv; ++i) {
    float velocity = qvel[vb + i] + dt * qacc[vb + i];
    if (!finite_float(velocity)) valid = false;
    candidate_qvel[vb + i] = velocity;
  }
  float next_time = time[world] + dt;
  if (!finite_float(next_time)) valid = false;
  candidate_time[world] = next_time;
  if (!valid) {
    output_status[world] = 12;
    return;
  }

  // Apply the updated velocity using MuJoCo's joint address conventions.
  for (uint j = 0; j < njnt; ++j) {
    int type = joint_type[j];
    uint pa = uint(joint_qposadr[j]);
    uint da = uint(joint_dofadr[j]);
    if (type == 0) {
      for (uint k = 0; k < 3; ++k) {
        float value = candidate_qpos[qb + pa + k] +
                      dt * candidate_qvel[vb + da + k];
        if (!finite_float(value)) valid = false;
        candidate_qpos[qb + pa + k] = value;
      }
      pa += 3;
      da += 3;
    }

    if (type == 0 || type == 1) {
      float4 q = float4(candidate_qpos[qb + pa],
                        candidate_qpos[qb + pa + 1],
                        candidate_qpos[qb + pa + 2],
                        candidate_qpos[qb + pa + 3]);
      float scale = max(max(abs(q.x), abs(q.y)), max(abs(q.z), abs(q.w)));
      if (scale == 0.0f) {
        output_status[world] = 11;
        return;
      }
      float4 normalized = quat_normalize_stable(q);
      float3 omega = float3(candidate_qvel[vb + da],
                            candidate_qvel[vb + da + 1],
                            candidate_qvel[vb + da + 2]);
      float4 increment = quat_increment(omega, dt, valid);
      float4 next_quat = quat_normalize_stable(quat_multiply(normalized, increment));
      for (uint k = 0; k < 4; ++k) {
        if (!finite_float(next_quat[k])) valid = false;
        candidate_qpos[qb + pa + k] = next_quat[k];
      }
    } else if (type == 2 || type == 3) {
      float value = candidate_qpos[qb + pa] + dt * candidate_qvel[vb + da];
      if (!finite_float(value)) valid = false;
      candidate_qpos[qb + pa] = value;
    }
    if (!valid) {
      output_status[world] = 12;
      return;
    }
  }

  // Commit only after every coordinate, quaternion, and time value is valid.
  for (uint i = 0; i < nq; ++i) output_qpos[qb + i] = candidate_qpos[qb + i];
  for (uint i = 0; i < nv; ++i) output_qvel[vb + i] = candidate_qvel[vb + i];
  output_time[world] = candidate_time[world];
  output_status[world] = 0;
}
