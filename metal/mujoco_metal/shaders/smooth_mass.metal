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

inline float3 qrot_mass(float4 q, float3 v) {
  return v + 2.0f*cross(q.yzw, cross(q.yzw, v) + q.x*v);
}
inline float3 cross_mass(float3 a, float3 b) { return cross(a, b); }
inline void store_cdof(device float* cdof, uint base,
                       uint dof, float3 angular, float3 linear) {
  uint address = base + dof*6;
  cdof[address] = angular.x; cdof[address+1] = angular.y;
  cdof[address+2] = angular.z; cdof[address+3] = linear.x;
  cdof[address+4] = linear.y; cdof[address+5] = linear.z;
}

// Dense, dimension-derived CRBA reference kernel. One thread owns one batch
// row and uses global scratch sized by the host from nbody and nv.
kernel void dense_mass_matrix(
    device const int* parent [[buffer(0)]],
    device const int* rootid [[buffer(1)]],
    device const int* body_jntadr [[buffer(2)]],
    device const int* body_jntnum [[buffer(3)]],
    device const int* dof_parentid [[buffer(4)]],
    device const int* dof_bodyid [[buffer(5)]],
    device const int* jnt_type [[buffer(6)]],
    device const int* jnt_dofadr [[buffer(7)]],
    device const float* body_mass [[buffer(8)]],
    device const float* body_inertia [[buffer(9)]],
    device const float* dof_armature [[buffer(10)]],
    device const float* body_xquat [[buffer(11)]],
    device const float* inertial_xpos [[buffer(12)]],
    device const float* inertial_xquat [[buffer(13)]],
    device const float* joint_anchor [[buffer(14)]],
    device const float* joint_axis [[buffer(15)]],
    device float* output_M [[buffer(16)]],
    device float* root_com [[buffer(17)]],
    device float* cdof [[buffer(18)]],
    device float* crb [[buffer(19)]],
    constant uint* dims [[buffer(20)]],
    device float* local_inertia [[buffer(21)]],
    uint world [[thread_position_in_grid]]) {
  uint nbody = dims[0], njnt = dims[1], nv = dims[2], batch = dims[3];
  if (world >= batch) return;
  uint com_base = world*nbody*3;
  uint dof_base = world*nv*6;
  uint inertia_base = world*nbody*36;
  uint mass_base = world*nv*nv;

  // Center of mass for each independent articulated root.
  for (uint root=0; root<nbody; ++root) {
    float total = 0.0f;
    float3 moment = float3(0.0f);
    for (uint b=1; b<nbody; ++b) {
      if (uint(rootid[b]) == root) {
        float mass = body_mass[b];
        total += mass;
        moment += mass*float3(inertial_xpos[(world*nbody+b)*3],
                              inertial_xpos[(world*nbody+b)*3+1],
                              inertial_xpos[(world*nbody+b)*3+2]);
      }
    }
    float3 center = total > 1e-15f ? moment/total :
        float3(inertial_xpos[(world*nbody+root)*3],
               inertial_xpos[(world*nbody+root)*3+1],
               inertial_xpos[(world*nbody+root)*3+2]);
    root_com[com_base+root*3] = center.x;
    root_com[com_base+root*3+1] = center.y;
    root_com[com_base+root*3+2] = center.z;
  }

  // Initialize body spatial inertias about their articulated-root COM.
  for (uint b=0; b<nbody; ++b) {
    float mass = body_mass[b];
    float3 inertia = float3(body_inertia[b*3], body_inertia[b*3+1],
                            body_inertia[b*3+2]);
    float4 quat = float4(inertial_xquat[(world*nbody+b)*4],
                         inertial_xquat[(world*nbody+b)*4+1],
                         inertial_xquat[(world*nbody+b)*4+2],
                         inertial_xquat[(world*nbody+b)*4+3]);
    float3 axis[3] = {qrot_mass(quat, float3(1,0,0)),
                      qrot_mass(quat, float3(0,1,0)),
                      qrot_mass(quat, float3(0,0,1))};
    float3 center = float3(root_com[com_base+uint(rootid[b])*3],
                           root_com[com_base+uint(rootid[b])*3+1],
                           root_com[com_base+uint(rootid[b])*3+2]);
    float3 position = float3(inertial_xpos[(world*nbody+b)*3],
                             inertial_xpos[(world*nbody+b)*3+1],
                             inertial_xpos[(world*nbody+b)*3+2]);
    float3 r = position-center;
    float S[9] = {0,-r.z,r.y, r.z,0,-r.x, -r.y,r.x,0};
    float3x3 Irot = float3x3(0.0f);
    for (uint k=0; k<3; ++k) {
      Irot[0][0] += inertia[k]*axis[k].x*axis[k].x;
      Irot[0][1] += inertia[k]*axis[k].x*axis[k].y;
      Irot[0][2] += inertia[k]*axis[k].x*axis[k].z;
      Irot[1][0] += inertia[k]*axis[k].y*axis[k].x;
      Irot[1][1] += inertia[k]*axis[k].y*axis[k].y;
      Irot[1][2] += inertia[k]*axis[k].y*axis[k].z;
      Irot[2][0] += inertia[k]*axis[k].z*axis[k].x;
      Irot[2][1] += inertia[k]*axis[k].z*axis[k].y;
      Irot[2][2] += inertia[k]*axis[k].z*axis[k].z;
    }
    uint ib = inertia_base+b*36;
    for (uint row=0; row<6; ++row) {
      for (uint col=0; col<6; ++col) {
        crb[ib+row*6+col] = 0.0f;
      }
    }
    for (uint row=0; row<3; ++row) {
      for (uint col=0; col<3; ++col) {
        float ss = 0.0f;
        for (uint k=0; k<3; ++k) ss += S[row*3+k]*S[k*3+col];
        crb[ib+row*6+col] = Irot[row][col]-mass*ss;
        crb[ib+row*6+col+3] = mass*S[row*3+col];
        crb[ib+(row+3)*6+col] = -mass*S[row*3+col];
        crb[ib+(row+3)*6+col+3] = row == col ? mass : 0.0f;
      }
    }
    uint local_ib = (world*nbody+b)*36;
    for (uint k=0; k<36; ++k) local_inertia[local_ib+k] = crb[ib+k];
  }

  // World-oriented generalized motion axes centered on each root COM.
  for (uint d=0; d<nv; ++d)
    for (uint k=0; k<6; ++k) cdof[dof_base+d*6+k] = 0.0f;
  for (uint b=1; b<nbody; ++b) {
    int count = body_jntnum[b];
    int first = body_jntadr[b];
    if (count <= 0) continue;
    float4 bodyq = float4(body_xquat[(world*nbody+b)*4],
                          body_xquat[(world*nbody+b)*4+1],
                          body_xquat[(world*nbody+b)*4+2],
                          body_xquat[(world*nbody+b)*4+3]);
    float3 center = float3(root_com[com_base+uint(rootid[b])*3],
                           root_com[com_base+uint(rootid[b])*3+1],
                           root_com[com_base+uint(rootid[b])*3+2]);
    for (int jj=0; jj<count; ++jj) {
      uint j = uint(first+jj);
      uint dadr = uint(jnt_dofadr[j]);
      uint typ = uint(jnt_type[j]);
      float3 anchor = float3(joint_anchor[(world*njnt+j)*3],
                             joint_anchor[(world*njnt+j)*3+1],
                             joint_anchor[(world*njnt+j)*3+2]);
      float3 offset = center-anchor;
      if (typ == 0 || typ == 1) {
        uint skip = typ == 0 ? 3 : 0;
        if (typ == 0) {
          store_cdof(cdof,dof_base,dadr,float3(0),float3(1,0,0));
          store_cdof(cdof,dof_base,dadr+1,float3(0),float3(0,1,0));
          store_cdof(cdof,dof_base,dadr+2,float3(0),float3(0,0,1));
        }
        for (uint k=0; k<3; ++k) {
          float3 basis = k == 0 ? float3(1,0,0) :
                         (k == 1 ? float3(0,1,0) : float3(0,0,1));
          float3 angular = qrot_mass(bodyq,basis);
          store_cdof(cdof,dof_base,dadr+skip+k,angular,
                     cross_mass(angular,offset));
        }
      } else {
        float3 axis = float3(joint_axis[(world*njnt+j)*3],
                             joint_axis[(world*njnt+j)*3+1],
                             joint_axis[(world*njnt+j)*3+2]);
        if (typ == 3) store_cdof(cdof,dof_base,dadr,axis,cross_mass(axis,offset));
        else store_cdof(cdof,dof_base,dadr,float3(0),axis);
      }
    }
  }

  // Composite spatial inertia accumulation, child to parent.
  for (int b=int(nbody)-1; b>0; --b) {
    int p = parent[b];
    if (p > 0) {
      uint child = inertia_base+uint(b)*36;
      uint par = inertia_base+uint(p)*36;
      for (uint k=0; k<36; ++k) crb[par+k] += crb[child+k];
    }
  }

  for (uint i=0; i<nv*nv; ++i) output_M[mass_base+i] = 0.0f;
  for (uint i=0; i<nv; ++i) {
    uint body = uint(dof_bodyid[i]);
    uint ib = inertia_base+body*36;
    float projected[6];
    for (uint r=0; r<6; ++r) {
      projected[r] = 0.0f;
      for (uint c=0; c<6; ++c)
        projected[r] += crb[ib+r*6+c]*cdof[dof_base+i*6+c];
    }
    int ancestor = int(i);
    while (ancestor >= 0) {
      float value = 0.0f;
      for (uint k=0; k<6; ++k)
        value += cdof[dof_base+uint(ancestor)*6+k]*projected[k];
      output_M[mass_base+i*nv+uint(ancestor)] = value;
      output_M[mass_base+uint(ancestor)*nv+i] = value;
      ancestor = dof_parentid[ancestor];
    }
    output_M[mass_base+i*nv+i] += dof_armature[i];
  }
}
