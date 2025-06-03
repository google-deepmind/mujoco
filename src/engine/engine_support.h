// Copyright 2021 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_SUPPORT_H_
#define MUJOCO_SRC_ENGINE_ENGINE_SUPPORT_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#ifdef __cplusplus
extern "C" {
#endif

// strings
MJAPI extern const char* mjDISABLESTRING[mjNDISABLE];
MJAPI extern const char* mjENABLESTRING[mjNENABLE];
MJAPI extern const char* mjTIMERSTRING[mjNTIMER];


//-------------------------- get/set state ---------------------------------------------------------

// return size of state specification
MJAPI int mj_stateSize(const mjModel* m, unsigned int spec);

// get state
MJAPI void mj_getState(const mjModel* m, const mjData* d, mjtNum* state, unsigned int spec);

// set state
MJAPI void mj_setState(const mjModel* m, mjData* d, const mjtNum* state, unsigned int spec);

// copy current state to the k-th model keyframe
MJAPI void mj_setKeyframe(mjModel* m, const mjData* d, int k);

//-------------------------- sparse chains ---------------------------------------------------------

// merge dof chains for two bodies
int mj_mergeChain(const mjModel* m, int* chain, int b1, int b2);

// merge dof chains for two simple bodies
int mj_mergeChainSimple(const mjModel* m, int* chain, int b1, int b2);

// get body chain
int mj_bodyChain(const mjModel* m, int body, int* chain);


//-------------------------- Jacobians -------------------------------------------------------------

// compute 3/6-by-nv Jacobian of global point attached to given body
MJAPI void mj_jac(const mjModel* m, const mjData* d,
                  mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body);

// compute body frame Jacobian
MJAPI void mj_jacBody(const mjModel* m, const mjData* d,
                      mjtNum* jacp, mjtNum* jacr, int body);

// compute body center-of-mass Jacobian
MJAPI void mj_jacBodyCom(const mjModel* m, const mjData* d,
                         mjtNum* jacp, mjtNum* jacr, int body);

// compute subtree center-of-mass Jacobian
MJAPI void mj_jacSubtreeCom(const mjModel* m, mjData* d, mjtNum* jacp, int body);

// compute geom Jacobian
MJAPI void mj_jacGeom(const mjModel* m, const mjData* d,
                      mjtNum* jacp, mjtNum* jacr, int geom);

// compute site Jacobian
MJAPI void mj_jacSite(const mjModel* m, const mjData* d,
                      mjtNum* jacp, mjtNum* jacr, int site);

// compute translation Jacobian of point, and rotation Jacobian of axis
MJAPI void mj_jacPointAxis(const mjModel* m, mjData* d,
                           mjtNum* jacPoint, mjtNum* jacAxis,
                           const mjtNum point[3], const mjtNum axis[3], int body);

// compute 3/6-by-nv sparse Jacobian of global point attached to given body
void mj_jacSparse(const mjModel* m, const mjData* d,
                  mjtNum* jacp, mjtNum* jacr, const mjtNum* point, int body,
                  int NV, const int* chain);

// sparse Jacobian difference for simple body contacts
void mj_jacSparseSimple(const mjModel* m, const mjData* d,
                        mjtNum* jacdifp, mjtNum* jacdifr, const mjtNum* point,
                        int body, int flg_second, int NV, int start);

// dense or sparse Jacobian difference for two body points: pos2 - pos1, global
MJAPI int mj_jacDifPair(const mjModel* m, const mjData* d, int* chain,
                        int b1, int b2, const mjtNum pos1[3], const mjtNum pos2[3],
                        mjtNum* jac1p, mjtNum* jac2p, mjtNum* jacdifp,
                        mjtNum* jac1r, mjtNum* jac2r, mjtNum* jacdifr);

// dense or sparse weighted sum of multiple body Jacobians at same point
int mj_jacSum(const mjModel* m, mjData* d, int* chain,
              int n, const int* body, const mjtNum* weight,
              const mjtNum point[3], mjtNum* jac, int flg_rot);

// compute 3/6-by-nv Jacobian time derivative of global point attached to given body
MJAPI void mj_jacDot(const mjModel* m, const mjData* d,
                     mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body);

// compute subtree angular momentum matrix
MJAPI void mj_angmomMat(const mjModel* m, mjData* d, mjtNum* mat, int body);


//-------------------------- inertia functions -----------------------------------------------------

// convert sparse inertia matrix M into full matrix
MJAPI void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M);

// multiply vector by inertia matrix (implementation)
MJAPI void mj_mulM_impl(mjtNum* res, const mjtNum* vec, int nv, const mjtNum* M,
                        const int* Madr, const int* parentid, const int* simplenum);

// multiply vector by inertia matrix
MJAPI void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// multiply vector by (inertia matrix)^(1/2)
MJAPI void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// add inertia matrix to destination matrix
//  destination can be sparse or dense when all int* are NULL
MJAPI void mj_addM(const mjModel* m, mjData* d, mjtNum* dst,
                   int* rownnz, int* rowadr, int* colind);


//-------------------------- perturbations ---------------------------------------------------------

// apply Cartesian force and torque
MJAPI void mj_applyFT(const mjModel* m, mjData* d,
                      const mjtNum force[3], const mjtNum torque[3],
                      const mjtNum point[3], int body, mjtNum* qfrc_target);

// accumulate xfrc_applied in qfrc
void mj_xfrcAccumulate(const mjModel* m, mjData* d, mjtNum* qfrc);


//-------------------------- coordinate transformation ---------------------------------------------

// compute object 6D velocity in object-centered frame, world/local orientation
MJAPI void mj_objectVelocity(const mjModel* m, const mjData* d,
                             int objtype, int objid, mjtNum res[6], int flg_local);

// compute object 6D acceleration in object-centered frame, world/local orientation
MJAPI void mj_objectAcceleration(const mjModel* m, const mjData* d,
                                 int objtype, int objid, mjtNum res[6], int flg_local);


//-------------------------- miscellaneous ---------------------------------------------------------

// returns the smallest distance between two geoms
MJAPI mjtNum mj_geomDistance(const mjModel* m, const mjData* d, int geom1, int geom2,
                             mjtNum distmax, mjtNum fromto[6]);

// extract 6D force:torque for one contact, in contact frame
MJAPI void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum result[6]);

// compute velocity by finite-differencing two positions
MJAPI void mj_differentiatePos(const mjModel* m, mjtNum* qvel, mjtNum dt,
                               const mjtNum* qpos1, const mjtNum* qpos2);

// integrate position with given velocity
MJAPI void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt);

// normalize all quaternions in qpos-type vector
MJAPI void mj_normalizeQuat(const mjModel* m, mjtNum* qpos);

// map from body local to global Cartesian coordinates
MJAPI void mj_local2Global(mjData* d, mjtNum xpos[3], mjtNum xmat[9],
                           const mjtNum pos[3], const mjtNum quat[4],
                           int body, mjtByte sameframe);

// return 1 if actuator i is disabled, 0 otherwise
MJAPI int mj_actuatorDisabled(const mjModel* m, int i);

// sum all body masses
MJAPI mjtNum mj_getTotalmass(const mjModel* m);

// scale body masses and inertias to achieve specified total mass
MJAPI void mj_setTotalmass(mjModel* m, mjtNum newmass);

// high-level warning function: count warnings in mjData, print only the first time
MJAPI void mj_warning(mjData* d, int warning, int info);

// version number
MJAPI int mj_version(void);

// current version of MuJoCo as a null-terminated string
MJAPI const char* mj_versionString(void);
#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SUPPORT_H_
