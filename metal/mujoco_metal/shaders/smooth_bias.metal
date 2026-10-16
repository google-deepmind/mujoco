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

inline float3 cross_bias(float3 a, float3 b) { return cross(a, b); }
inline void multiply_spatial(device const float* matrix, uint offset,
                            device const float* vector, thread float* result) {
  for (uint row=0; row<6; ++row) {
    result[row] = 0.0f;
    for (uint col=0; col<6; ++col)
      result[row] += matrix[offset+row*6+col]*vector[col];
  }
}
inline void motion_cross(thread const float* a, device const float* b,
                         thread float* result) {
  float3 aw = float3(a[0],a[1],a[2]);
  float3 av = float3(a[3],a[4],a[5]);
  float3 bw = float3(b[0],b[1],b[2]);
  float3 bv = float3(b[3],b[4],b[5]);
  float3 angular = cross_bias(aw,bw);
  float3 linear = cross_bias(aw,bv)+cross_bias(av,bw);
  for (uint k=0;k<3;++k) { result[k]=angular[k]; result[k+3]=linear[k]; }
}
inline void force_cross(device const float* motion, thread const float* force,
                        thread float* result) {
  float3 w = float3(motion[0],motion[1],motion[2]);
  float3 v = float3(motion[3],motion[4],motion[5]);
  float3 tau = float3(force[0],force[1],force[2]);
  float3 f = float3(force[3],force[4],force[5]);
  float3 torque = cross_bias(w,tau)+cross_bias(v,f);
  float3 out_force = cross_bias(w,f);
  for (uint k=0;k<3;++k) { result[k]=torque[k]; result[k+3]=out_force[k]; }
}

// Newton-Euler bias pass over the same generic model mappings and local
// inertias produced by the dense mass kernel. This is inertial/gravity bias;
// passive, actuator, contact, and constraint forces remain unsupported.
kernel void smooth_bias(
    device const int* parent [[buffer(0)]],
    device const int* body_dofadr [[buffer(1)]],
    device const int* body_dofnum [[buffer(2)]],
    device const int* body_jntadr [[buffer(3)]],
    device const int* body_jntnum [[buffer(4)]],
    device const int* dof_bodyid [[buffer(5)]],
    device const int* jnt_type [[buffer(6)]],
    device const int* jnt_dofadr [[buffer(7)]],
    device const float* gravity [[buffer(8)]],
    device const float* cdof [[buffer(9)]],
    device const float* local_inertia [[buffer(10)]],
    device const int* disableflags [[buffer(11)]],
    device const float* qvel [[buffer(12)]],
    device float* cvel [[buffer(13)]],
    device float* cdof_dot [[buffer(14)]],
    device float* cacc [[buffer(15)]],
    device float* body_force [[buffer(16)]],
    device float* qfrc_bias [[buffer(17)]],
    constant uint* dims [[buffer(18)]],
    uint world [[thread_position_in_grid]]) {
  uint nbody=dims[0], nv=dims[2], batch=dims[3];
  if (world>=batch) return;
  uint vb=world*nv;
  uint body6=world*nbody*6;
  uint dof6=world*nv*6;
  uint inert=world*nbody*36;

  for (uint d=0;d<nv;++d)
    for (uint k=0;k<6;++k) cdof_dot[dof6+d*6+k]=0.0f;
  for (uint b=0;b<nbody;++b)
    for (uint k=0;k<6;++k) cvel[body6+b*6+k]=0.0f;

  // MuJoCo comVel grouping: free translation first, then the free/ball
  // angular triplet; scalar joints compute their derivative before update.
  for (uint b=1;b<nbody;++b) {
    int p=parent[b];
    float velocity[6];
    for (uint k=0;k<6;++k) velocity[k]=cvel[body6+uint(p)*6+k];
    int first=body_jntadr[b];
    int count=body_jntnum[b];
    for (int jj=0;jj<count;++jj) {
      uint j=uint(first+jj);
      uint d=uint(jnt_dofadr[j]);
      uint typ=uint(jnt_type[j]);
      if (typ==0 || typ==1) {
        uint skip=typ==0 ? 3 : 0;
        if (typ==0) {
          for (uint k=0;k<3;++k) {
            for (uint r=0;r<6;++r)
              velocity[r]+=cdof[dof6+(d+k)*6+r]*qvel[vb+d+k];
          }
        }
        float derivatives[18];
        for (uint k=0;k<3;++k)
          motion_cross(velocity,cdof+dof6+(d+skip+k)*6,derivatives+k*6);
        for (uint k=0;k<3;++k) {
          for (uint r=0;r<6;++r)
            cdof_dot[dof6+(d+skip+k)*6+r]=derivatives[k*6+r];
        }
        for (uint k=0;k<3;++k) {
          for (uint r=0;r<6;++r)
            velocity[r]+=cdof[dof6+(d+skip+k)*6+r]*qvel[vb+d+skip+k];
        }
      } else {
        float derivative[6];
        motion_cross(velocity,cdof+dof6+d*6,derivative);
        for (uint r=0;r<6;++r) {
          cdof_dot[dof6+d*6+r]=derivative[r];
          velocity[r]+=cdof[dof6+d*6+r]*qvel[vb+d];
        }
      }
    }
    for (uint k=0;k<6;++k) cvel[body6+b*6+k]=velocity[k];
  }

  // RNE at zero generalized acceleration. World acceleration is -gravity.
  for (uint b=0;b<nbody;++b) {
    for (uint k=0;k<6;++k) cacc[body6+b*6+k]=0.0f;
    for (uint k=0;k<6;++k) body_force[body6+b*6+k]=0.0f;
  }
  if ((disableflags[0] & 128)==0) {
    cacc[body6+3]=-gravity[0];
    cacc[body6+4]=-gravity[1];
    cacc[body6+5]=-gravity[2];
  }
  for (uint b=1;b<nbody;++b) {
    uint p=uint(parent[b]);
    uint first=uint(body_dofadr[b]);
    uint count=uint(body_dofnum[b]);
    for (uint k=0;k<6;++k) {
      float acceleration=cacc[body6+p*6+k];
      for (uint d=0;d<count;++d)
        acceleration+=cdof_dot[dof6+(first+d)*6+k]*qvel[vb+first+d];
      cacc[body6+b*6+k]=acceleration;
    }
    float inertial_acc[6], inertial_vel[6], coriolis[6];
    multiply_spatial(local_inertia,inert+b*36,cacc+body6+b*6,inertial_acc);
    multiply_spatial(local_inertia,inert+b*36,cvel+body6+b*6,inertial_vel);
    force_cross(cvel+body6+b*6,inertial_vel,coriolis);
    for (uint k=0;k<6;++k)
      body_force[body6+b*6+k]=inertial_acc[k]+coriolis[k];
  }
  for (int b=int(nbody)-1;b>0;--b) {
    int p=parent[b];
    if (p>0)
      for (uint k=0;k<6;++k)
        body_force[body6+uint(p)*6+k]+=body_force[body6+uint(b)*6+k];
  }
  for (uint d=0;d<nv;++d) {
    uint body=uint(dof_bodyid[d]);
    float value=0.0f;
    for (uint k=0;k<6;++k)
      value+=cdof[dof6+d*6+k]*body_force[body6+body*6+k];
    qfrc_bias[vb+d]=value;
  }
}
