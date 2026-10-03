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

inline float4 qmul(float4 a, float4 b) {
  return float4(a.x*b.x-dot(a.yzw, b.yzw),
      a.x*b.yzw+b.x*a.yzw+cross(a.yzw, b.yzw));
}
inline float3 qrot(float4 q, float3 v) {
  return v + 2.0f*cross(q.yzw, cross(q.yzw, v) + q.x*v);
}
inline float4 qnorm(float4 q) { return q * rsqrt(dot(q, q)); }
inline float4 axisq(float3 axis, float angle) {
  axis = normalize(axis);
  return float4(cos(angle*0.5f), axis*sin(angle*0.5f));
}
inline float3 load3(device const float* values, uint offset) {
  return float3(values[offset], values[offset+1], values[offset+2]);
}
inline float4 load4(device const float* values, uint offset) {
  return float4(values[offset], values[offset+1], values[offset+2], values[offset+3]);
}
inline void store3(device float* values, uint offset, float3 value) {
  values[offset] = value.x; values[offset+1] = value.y; values[offset+2] = value.z;
}
inline void store4(device float* values, uint offset, float4 value) {
  values[offset] = value.x; values[offset+1] = value.y;
  values[offset+2] = value.z; values[offset+3] = value.w;
}

// One thread computes one world's complete pose tree. This favors a simple
// correctness baseline; parallel body traversal is a separate optimization.
kernel void forward_kinematics(
    device const int* parent [[buffer(0)]],
    device const float* body_pos [[buffer(1)]],
    device const float* body_quat [[buffer(2)]],
    device const int* jnt_type [[buffer(3)]],
    device const int* jnt_qposadr [[buffer(4)]],
    device const int* jnt_bodyid [[buffer(5)]],
    device const float* jnt_pos [[buffer(6)]],
    device const float* jnt_axis [[buffer(7)]],
    device const float* qpos0 [[buffer(8)]],
    device const float* qpos [[buffer(9)]],
    device float* xpos [[buffer(10)]],
    device float* xquat [[buffer(11)]],
    device const int* geom_bodyid [[buffer(12)]],
    device const float* geom_pos [[buffer(13)]],
    device const float* geom_quat [[buffer(14)]],
    device float* geom_xpos [[buffer(15)]],
    device float* geom_xquat [[buffer(16)]],
    device const int* site_bodyid [[buffer(17)]],
    device const float* site_pos [[buffer(18)]],
    device const float* site_quat [[buffer(19)]],
    device float* site_xpos [[buffer(20)]],
    device float* site_xquat [[buffer(21)]],
    device const float* body_ipos [[buffer(22)]],
    device const float* body_iquat [[buffer(23)]],
    device float* inertial_xpos [[buffer(24)]],
    device float* inertial_xquat [[buffer(25)]],
    constant uint* dims [[buffer(26)]],
    device float* joint_anchor [[buffer(27)]],
    device float* joint_axis [[buffer(28)]],
    uint world [[thread_position_in_grid]]) {
  uint nq = dims[0], nbody = dims[1], njnt = dims[2];
  uint ngeom = dims[3], nsite = dims[4], batch = dims[5];
  if (world >= batch) return;
  uint qbase = world*nq;
  uint posebase = world*nbody;
  store3(xpos, posebase*3, float3(0));
  store4(xquat, posebase*4, float4(1, 0, 0, 0));
  for (uint b=1; b<nbody; ++b) {
    uint p = uint(parent[b]);
    float3 pos = load3(xpos, (posebase+p)*3) +
                 qrot(load4(xquat, (posebase+p)*4), load3(body_pos, b*3));
    float4 quat = qnorm(qmul(load4(xquat, (posebase+p)*4), load4(body_quat, b*4)));
    for (uint j=0; j<njnt; ++j) {
      if (uint(jnt_bodyid[j]) != b) continue;
      int typ = jnt_type[j];
      uint qa = qbase + uint(jnt_qposadr[j]);
      float3 anchor = pos + qrot(quat, load3(jnt_pos, j*3));
      float3 axis = qrot(quat, load3(jnt_axis, j*3));
      store3(joint_anchor, (world*njnt+j)*3, anchor);
      store3(joint_axis, (world*njnt+j)*3, axis);
      if (typ == 0) {
        pos = load3(qpos, qa);
        quat = qnorm(load4(qpos, qa+3));
        store3(joint_anchor, (world*njnt+j)*3, pos);
        store3(joint_axis, (world*njnt+j)*3, load3(jnt_axis, j*3));
      } else if (typ == 1) {
        quat = qnorm(qmul(quat, qnorm(load4(qpos, qa))));
        pos = anchor - qrot(quat, load3(jnt_pos, j*3));
      } else if (typ == 2) {
        pos += qrot(quat, load3(jnt_axis, j*3)*(qpos[qa]-qpos0[jnt_qposadr[j]]));
      } else if (typ == 3) {
        quat = qnorm(qmul(quat, axisq(load3(jnt_axis, j*3),
                                  qpos[qa]-qpos0[jnt_qposadr[j]])));
        pos = anchor - qrot(quat, load3(jnt_pos, j*3));
      }
    }
    store3(xpos, (posebase+b)*3, pos);
    store4(xquat, (posebase+b)*4, qnorm(quat));
  }
  for (uint g=0; g<ngeom; ++g) {
    uint b = uint(geom_bodyid[g]);
    float4 bq = load4(xquat, (posebase+b)*4);
    store3(geom_xpos, (world*ngeom+g)*3,
           load3(xpos, (posebase+b)*3)+qrot(bq, load3(geom_pos, g*3)));
    store4(geom_xquat, (world*ngeom+g)*4,
           qmul(bq, load4(geom_quat, g*4)));
  }
  for (uint s=0; s<nsite; ++s) {
    uint b = uint(site_bodyid[s]);
    float4 bq = load4(xquat, (posebase+b)*4);
    store3(site_xpos, (world*nsite+s)*3,
           load3(xpos, (posebase+b)*3)+qrot(bq, load3(site_pos, s*3)));
    store4(site_xquat, (world*nsite+s)*4,
           qmul(bq, load4(site_quat, s*4)));
  }
  for (uint b=0; b<nbody; ++b) {
    float4 bq = load4(xquat, (posebase+b)*4);
    store3(inertial_xpos, (posebase+b)*3,
           load3(xpos, (posebase+b)*3)+qrot(bq, load3(body_ipos, b*3)));
    store4(inertial_xquat, (posebase+b)*4,
           qmul(bq, load4(body_iquat, b*4)));
  }
}
