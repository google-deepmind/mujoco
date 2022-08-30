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

#ifdef __cplusplus
extern "C" {
#endif

// strings
MJAPI extern const char* mjDISABLESTRING[mjNDISABLE];
MJAPI extern const char* mjENABLESTRING[mjNENABLE];
MJAPI extern const char* mjTIMERSTRING[mjNTIMER];


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
                  int NV, int* chain);

// sparse Jacobian difference for simple body contacts
void mj_jacSparseSimple(const mjModel* m, const mjData* d,
                        mjtNum* jacdifp, mjtNum* jacdifr, const mjtNum* point,
                        int body, int flg_second, int NV, int start);

// dense or sparse Jacobian difference for two body points: pos2 - pos1, global
MJAPI int mj_jacDifPair(const mjModel* m, const mjData* d, int* chain,
                        int b1, int b2, const mjtNum pos1[3], const mjtNum pos2[3],
                        mjtNum* jac1p, mjtNum* jac2p, mjtNum* jacdifp,
                        mjtNum* jac1r, mjtNum* jac2r, mjtNum* jacdifr);


//-------------------------- name functions --------------------------------------------------------

// get id of object with specified name; -1: not found; type is mjtObj
MJAPI int mj_name2id(const mjModel* m, int type, const char* name);

// get name of object with specified id; 0: invalid type or id; type is mjtObj
MJAPI const char* mj_id2name(const mjModel* m, int type, int id);


//-------------------------- inertia functions -----------------------------------------------------

// convert sparse inertia matrix M into full matrix
MJAPI void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M);

// multiply vector by inertia matrix
MJAPI void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// multiply vector by (inertia matrix)^(1/2)
MJAPI void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// add inertia matrix to destination matrix
//  destination can be sparse uncompressed, or dense when all int* are NULL
MJAPI void mj_addM(const mjModel* m, mjData* d, mjtNum* dst,
                   int* rownnz, int* rowadr, int* colind);

// construct sparse matrix representations matching qM
MJAPI void mj_makeMSparse(const mjModel* m, mjData* d, int *rownnz, int *rowadr, int *colind);

// set dst = qM, handle different sparsity representations
MJAPI void mj_setMSparse(const mjModel* m, mjData* d, mjtNum* dst,
                         const int *rownnz, const int *rowadr, const int *colind);


//-------------------------- perturbations ---------------------------------------------------------

// apply cartesian force and torque
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

// extract 6D force:torque for one contact, in contact frame
MJAPI void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum result[6]);

// compute velocity by finite-differencing two positions
MJAPI void mj_differentiatePos(const mjModel* m, mjtNum* qvel, mjtNum dt,
                               const mjtNum* qpos1, const mjtNum* qpos2);

// integrate position with given velocity
MJAPI void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt);

// normalize all quaterions in qpos-type vector
MJAPI void mj_normalizeQuat(const mjModel* m, mjtNum* qpos);

// map from body local to global Cartesian coordinates
MJAPI void mj_local2Global(mjData* d, mjtNum xpos[3], mjtNum xmat[9],
                           const mjtNum pos[3], const mjtNum quat[4],
                           int body, mjtByte sameframe);

// sum all body masses
MJAPI mjtNum mj_getTotalmass(const mjModel* m);

// scale body masses and inertias to achieve specified total mass
MJAPI void mj_setTotalmass(mjModel* m, mjtNum newmass);

// high-level warning function: count warnings in mjData, print only the first time
MJAPI void mj_warning(mjData* d, int warning, int info);

// version number
MJAPI int mj_version(void);

// current version of MuJoCo as a null-terminated string
MJAPI const char* mj_versionString();
#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SUPPORT_H_
