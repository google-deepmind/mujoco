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

// One launch computes one model in topological body order. This is a reference
// kernel; parallel body-level traversal and batched launch are later optimizations.
kernel void forward_kinematics(
    device const int* parent [[buffer(0)]],
    device const float3* body_pos [[buffer(1)]],
    device const float4* body_quat [[buffer(2)]],
    device const int* jnt_type [[buffer(3)]],
    device const int* jnt_qposadr [[buffer(4)]],
    device const int* jnt_bodyid [[buffer(5)]],
    device const float3* jnt_pos [[buffer(6)]],
    device const float3* jnt_axis [[buffer(7)]],
    device const float* qpos0 [[buffer(8)]],
    device const float* qpos [[buffer(9)]],
    device float3* xpos [[buffer(10)]],
    device float4* xquat [[buffer(11)]],
    constant uint& nbody [[buffer(12)]],
    constant uint& njnt [[buffer(13)]],
    uint tid [[thread_position_in_grid]]) {
  if (tid != 0) return;
  xpos[0] = float3(0);
  xquat[0] = float4(1, 0, 0, 0);
  for (uint b=1; b<nbody; ++b) {
    uint p = uint(parent[b]);
    float3 pos = xpos[p] + qrot(xquat[p], body_pos[b]);
    float4 quat = qmul(xquat[p], body_quat[b]);
    for (uint j=0; j<njnt; ++j) {
      if (uint(jnt_bodyid[j]) != b) continue;
      int typ = jnt_type[j];
      uint qa = uint(jnt_qposadr[j]);
      if (typ == 0) {
        pos = float3(qpos[qa], qpos[qa+1], qpos[qa+2]);
        quat = qnorm(float4(qpos[qa+3], qpos[qa+4], qpos[qa+5], qpos[qa+6]));
      } else if (typ == 1) {
        float3 anchor = pos + qrot(quat, jnt_pos[j]);
        float4 rot = qnorm(float4(qpos[qa], qpos[qa+1], qpos[qa+2], qpos[qa+3]));
        quat = qmul(quat, rot);
        pos = anchor - qrot(quat, jnt_pos[j]);
      } else if (typ == 2) {
        pos += qrot(quat, jnt_axis[j]*(qpos[qa]-qpos0[qa]));
      } else if (typ == 3) {
        float3 anchor = pos + qrot(quat, jnt_pos[j]);
        quat = qmul(quat, axisq(jnt_axis[j], qpos[qa]-qpos0[qa]));
        pos = anchor - qrot(quat, jnt_pos[j]);
      }
    }
    xpos[b] = pos;
    xquat[b] = quat;
  }
}
