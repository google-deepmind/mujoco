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

#include "engine/engine_core_smooth.h"

#include <stddef.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_core_constraint.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"

//--------------------------- position -------------------------------------------------------------

// forward kinematics
void mj_kinematics(const mjModel* m, mjData* d) {
  int nbody = m->nbody, nsite = m->nsite, ngeom = m->ngeom;

  // set world position and orientation
  mju_zero3(d->xpos);
  mju_unit4(d->xquat);
  mju_zero3(d->xipos);
  mju_zero(d->xmat, 9);
  mju_zero(d->ximat, 9);
  d->xmat[0] = d->xmat[4] = d->xmat[8] = 1;
  d->ximat[0] = d->ximat[4] = d->ximat[8] = 1;

  // compute global cartesian positions and orientations of all bodies
  for (int i=1; i < nbody; i++) {
    mjtNum xpos[3], xquat[4];
    int jntadr = m->body_jntadr[i];
    int jntnum = m->body_jntnum[i];

    // free joint
    if (jntnum == 1 && m->jnt_type[jntadr] == mjJNT_FREE) {
      // get qpos address
      int qadr = m->jnt_qposadr[jntadr];

      // copy pos and quat from qpos
      mju_copy3(xpos, d->qpos+qadr);
      mju_copy4(xquat, d->qpos+qadr+3);
      mju_normalize4(xquat);

      // assign xanchor and xaxis
      mju_copy3(d->xanchor+3*jntadr, xpos);
      mju_copy3(d->xaxis+3*jntadr, m->jnt_axis+3*jntadr);
    }

    // regular or no joint
    else {
      int pid = m->body_parentid[i];

      // get body pos and quat: from model or mocap
      mjtNum *bodypos, *bodyquat, quat[4];
      if (m->body_mocapid[i] >= 0) {
        bodypos = d->mocap_pos + 3*m->body_mocapid[i];
        mju_copy4(quat, d->mocap_quat + 4*m->body_mocapid[i]);
        mju_normalize4(quat);
        bodyquat = quat;
      } else {
        bodypos = m->body_pos+3*i;
        bodyquat = m->body_quat+4*i;
      }

      // apply fixed translation and rotation relative to parent
      if (pid) {
        mju_mulMatVec3(xpos, d->xmat+9*pid, bodypos);
        mju_addTo3(xpos, d->xpos+3*pid);
        mju_mulQuat(xquat, d->xquat+4*pid, bodyquat);
      } else {
        // parent is the world
        mju_copy3(xpos, bodypos);
        mju_copy4(xquat, bodyquat);
      }

      // accumulate joints, compute xpos and xquat for this body
      mjtNum xanchor[3], xaxis[3];
      for (int j=0; j < jntnum; j++) {
        // get joint id, qpos address, joint type
        int jid = jntadr + j;
        int qadr = m->jnt_qposadr[jid];
        mjtJoint jtype = m->jnt_type[jid];

        // compute axis in global frame; ball jnt_axis is (0,0,1), set by compiler
        mju_rotVecQuat(xaxis, m->jnt_axis+3*jid, xquat);

        // compute anchor in global frame
        mju_rotVecQuat(xanchor, m->jnt_pos+3*jid, xquat);
        mju_addTo3(xanchor, xpos);

        // apply joint transformation
        switch (jtype) {
        case mjJNT_SLIDE:
          mju_addToScl3(xpos, xaxis, d->qpos[qadr] - m->qpos0[qadr]);
          break;

        case mjJNT_BALL:
        case mjJNT_HINGE:
          {
            // compute local quaternion rotation
            mjtNum qloc[4];
            if (jtype == mjJNT_BALL) {
              mju_copy4(qloc, d->qpos+qadr);
              mju_normalize4(qloc);
            } else {
              mju_axisAngle2Quat(qloc, m->jnt_axis+3*jid, d->qpos[qadr] - m->qpos0[qadr]);
            }

            // apply rotation
            mju_mulQuat(xquat, xquat, qloc);

            // correct for off-center rotation
            mjtNum vec[3];
            mju_rotVecQuat(vec, m->jnt_pos+3*jid, xquat);
            mju_sub3(xpos, xanchor, vec);
          }
          break;

        default:
          mjERROR("unknown joint type %d", jtype);    // SHOULD NOT OCCUR
        }

        // assign xanchor and xaxis
        mju_copy3(d->xanchor+3*jid, xanchor);
        mju_copy3(d->xaxis+3*jid, xaxis);
      }
    }

    // assign xquat and xpos, construct xmat
    mju_normalize4(xquat);
    mju_copy4(d->xquat+4*i, xquat);
    mju_copy3(d->xpos+3*i, xpos);
    mju_quat2Mat(d->xmat+9*i, xquat);
  }

  // compute/copy Cartesian positions and orientations of body inertial frames
  for (int i=1; i < nbody; i++) {
    mj_local2Global(d, d->xipos+3*i, d->ximat+9*i,
                    m->body_ipos+3*i, m->body_iquat+4*i,
                    i, m->body_sameframe[i]);
  }

  // compute/copy Cartesian positions and orientations of geoms
  for (int i=0; i < ngeom; i++) {
    mj_local2Global(d, d->geom_xpos+3*i, d->geom_xmat+9*i,
                    m->geom_pos+3*i, m->geom_quat+4*i,
                    m->geom_bodyid[i], m->geom_sameframe[i]);
  }

  // compute/copy Cartesian positions and orientations of sites
  for (int i=0; i < nsite; i++) {
    mj_local2Global(d, d->site_xpos+3*i, d->site_xmat+9*i,
                    m->site_pos+3*i, m->site_quat+4*i,
                    m->site_bodyid[i], m->site_sameframe[i]);
  }
}



// map inertias and motion dofs to global frame centered at subtree-CoM
void mj_comPos(const mjModel* m, mjData* d) {
  int nbody = m->nbody, njnt = m->njnt;
  mjtNum offset[3], axis[3];
  mj_markStack(d);
  mjtNum* mass_subtree = mjSTACKALLOC(d, m->nbody, mjtNum);

  // clear subtree
  mju_zero(mass_subtree, m->nbody);
  mju_zero(d->subtree_com, m->nbody*3);

  // backwards pass over bodies: compute subtree_com and mass_subtree
  for (int i=nbody-1; i >= 0; i--) {
    // add local info
    mju_addToScl3(d->subtree_com+3*i, d->xipos+3*i, m->body_mass[i]);
    mass_subtree[i] += m->body_mass[i];

    // add to parent, except for world
    if (i) {
      int j = m->body_parentid[i];
      mju_addTo3(d->subtree_com+3*j, d->subtree_com+3*i);
      mass_subtree[j] += mass_subtree[i];
    }

    // compute local com
    if (mass_subtree[i] < mjMINVAL) {
      mju_copy3(d->subtree_com+3*i, d->xipos+3*i);
    } else {
      mju_scl3(d->subtree_com+3*i, d->subtree_com+3*i,
               1.0/mjMAX(mjMINVAL, mass_subtree[i]));
    }
  }

  // zero out CoM frame inertia for the world body
  mju_zero(d->cinert, 10);

  // map inertias to frame centered at subtree_com
  for (int i=1; i < nbody; i++) {
    mju_sub3(offset, d->xipos+3*i, d->subtree_com+3*m->body_rootid[i]);
    mju_inertCom(d->cinert+10*i, m->body_inertia+3*i, d->ximat+9*i,
                 offset, m->body_mass[i]);
  }

  // map motion dofs to global frame centered at subtree_com
  for (int j=0; j < njnt; j++) {
    // get dof address, body index
    int da = 6*m->jnt_dofadr[j];
    int bi = m->jnt_bodyid[j];

    // compute com-anchor vector
    mju_sub3(offset, d->subtree_com+3*m->body_rootid[bi], d->xanchor+3*j);

    // create motion dof
    int skip = 0;
    switch ((mjtJoint) m->jnt_type[j]) {
    case mjJNT_FREE:
      // translation components: x, y, z in global frame
      mju_zero(d->cdof+da, 18);
      for (int i=0; i < 3; i++) {
        d->cdof[da+3+7*i] = 1;
      }

      // rotation components: same as ball
      skip = 18;
      mjFALLTHROUGH;

    case mjJNT_BALL:
      for (int i=0; i < 3; i++) {
        // I_3 rotation in child frame (assume no subsequent rotations)
        axis[0] = d->xmat[9*bi+i+0];
        axis[1] = d->xmat[9*bi+i+3];
        axis[2] = d->xmat[9*bi+i+6];

        mju_dofCom(d->cdof+da+skip+6*i, axis, offset);
      }
      break;

    case mjJNT_SLIDE:
      mju_dofCom(d->cdof+da, d->xaxis+3*j, 0);
      break;

    case mjJNT_HINGE:
      mju_dofCom(d->cdof+da, d->xaxis+3*j, offset);
      break;
    }
  }

  mj_freeStack(d);
}



// compute camera and light positions and orientations
void mj_camlight(const mjModel* m, mjData* d) {
  mjtNum pos[3], matT[9];

  // compute Cartesian positions and orientations of cameras
  for (int i=0; i < m->ncam; i++) {
    // default processing for fixed mode
    mj_local2Global(d, d->cam_xpos+3*i, d->cam_xmat+9*i,
                    m->cam_pos+3*i, m->cam_quat+4*i, m->cam_bodyid[i], 0);

    // get camera body id and target body id
    int id = m->cam_bodyid[i];
    int id1 = m->cam_targetbodyid[i];

    // adjust for mode
    switch ((mjtCamLight) m->cam_mode[i]) {
    case mjCAMLIGHT_FIXED:
      break;
    case mjCAMLIGHT_TRACK:
    case mjCAMLIGHT_TRACKCOM:
      // fixed global orientation
      mju_copy(d->cam_xmat+9*i, m->cam_mat0+9*i, 9);

      // position: track camera body
      if (m->cam_mode[i] == mjCAMLIGHT_TRACK) {
        mju_add3(d->cam_xpos+3*i, d->xpos+3*id, m->cam_pos0+3*i);
      }

      // position: track subtree com
      else {
        mju_add3(d->cam_xpos+3*i, d->subtree_com+3*id, m->cam_poscom0+3*i);
      }
      break;

    case mjCAMLIGHT_TARGETBODY:
    case mjCAMLIGHT_TARGETBODYCOM:
      // only if target body is specified
      if (id1 >= 0) {
        // get position to look at
        if (m->cam_mode[i] == mjCAMLIGHT_TARGETBODY) {
          mju_copy3(pos, d->xpos+3*id1);
        } else {
          mju_copy3(pos, d->subtree_com+3*id1);
        }

        // zaxis = -desired camera direction, in global frame
        mju_sub3(matT+6, d->cam_xpos+3*i, pos);
        mju_normalize3(matT+6);

        // xaxis: orthogonal to zaxis and to (0,0,1)
        matT[3] = 0;
        matT[4] = 0;
        matT[5] = 1;
        mju_cross(matT, matT+3, matT+6);
        mju_normalize3(matT);

        // yaxis: orthogonal to xaxis and zaxis
        mju_cross(matT+3, matT+6, matT);
        mju_normalize3(matT+3);

        // set camera frame
        mju_transpose(d->cam_xmat+9*i, matT, 3, 3);
      }
    }
  }

  // compute Cartesian positions and directions of lights
  for (int i=0; i < m->nlight; i++) {
    // default processing for fixed mode
    mj_local2Global(d, d->light_xpos+3*i, 0, m->light_pos+3*i, 0, m->light_bodyid[i], 0);
    mju_rotVecQuat(d->light_xdir+3*i, m->light_dir+3*i, d->xquat+4*m->light_bodyid[i]);

    // get light body id and target body id
    int id = m->light_bodyid[i];
    int id1 = m->light_targetbodyid[i];

    // adjust for mode
    switch ((mjtCamLight) m->light_mode[i]) {
    case mjCAMLIGHT_FIXED:
      break;
    case mjCAMLIGHT_TRACK:
    case mjCAMLIGHT_TRACKCOM:
      // fixed global orientation
      mju_copy3(d->light_xdir+3*i, m->light_dir0+3*i);

      // position: track light body
      if (m->light_mode[i] == mjCAMLIGHT_TRACK) {
        mju_add3(d->light_xpos+3*i, d->xpos+3*id, m->light_pos0+3*i);
      }

      // position: track subtree com
      else {
        mju_add3(d->light_xpos+3*i, d->subtree_com+3*id, m->light_poscom0+3*i);
      }
      break;

    case mjCAMLIGHT_TARGETBODY:
    case mjCAMLIGHT_TARGETBODYCOM:
      // only if target body is specified
      if (id1 >= 0) {
        // get position to look at
        if (m->light_mode[i] == mjCAMLIGHT_TARGETBODY) {
          mju_copy3(pos, d->xpos+3*id1);
        } else {
          mju_copy3(pos, d->subtree_com+3*id1);
        }

        // set dir
        mju_sub3(d->light_xdir+3*i, pos, d->light_xpos+3*i);
      }
    }

    // normalize dir
    mju_normalize3(d->light_xdir+3*i);
  }
}



// update dynamic BVH; leaf aabbs must be updated before call
void mj_updateDynamicBVH(const mjModel* m, mjData* d, int bvhadr, int bvhnum) {
  mj_markStack(d);
  int* modified = mjSTACKALLOC(d, bvhnum, int);
  mju_zeroInt(modified, bvhnum);

  // mark leafs as modified
  for (int i=0; i < bvhnum; i++) {
    if (m->bvh_nodeid[bvhadr+i] >= 0) {
      modified[i] = 1;
    }
  }

  // update non-leafs in backward pass (parents come before children)
  for (int i=bvhnum-1; i >= 0; i--) {
    if (m->bvh_nodeid[bvhadr+i] < 0) {
      int child1 = m->bvh_child[2*(bvhadr+i)];
      int child2 = m->bvh_child[2*(bvhadr+i)+1];

      // update if either child is modified
      if (modified[child1] || modified[child2]) {
        mjtNum* aabb = d->bvh_aabb_dyn + 6*(bvhadr - m->nbvhstatic + i);
        const mjtNum* aabb1 = d->bvh_aabb_dyn + 6*(bvhadr - m->nbvhstatic + child1);
        const mjtNum* aabb2 = d->bvh_aabb_dyn + 6*(bvhadr - m->nbvhstatic + child2);

        // compute new (min, max)
        mjtNum xmin[3], xmax[3];
        for (int k=0; k < 3; k++) {
          xmin[k] = mju_min(aabb1[k] - aabb1[k+3], aabb2[k] - aabb2[k+3]);
          xmax[k] = mju_max(aabb1[k] + aabb1[k+3], aabb2[k] + aabb2[k+3]);
        }

        // convert to (center, size)
        for (int k=0; k < 3; k++) {
          aabb[k]   = 0.5*(xmax[k]+xmin[k]);
          aabb[k+3] = 0.5*(xmax[k]-xmin[k]);
        }

        modified[i] = 1;
      }
    }
  }

  mj_freeStack(d);
}



// compute flex-related quantities
void mj_flex(const mjModel* m, mjData* d) {
  int nv = m->nv, issparse = mj_isSparse(m);
  int* rowadr = d->flexedge_J_rowadr, *rownnz = d->flexedge_J_rownnz;
  mjtNum* J = d->flexedge_J;

  // skip if no flexes
  if (!m->nflex) {
    return;
  }

  // compute Cartesian positions of flex vertices
  for (int f=0; f < m->nflex; f++) {
    int vstart = m->flex_vertadr[f];
    int vend = m->flex_vertadr[f] + m->flex_vertnum[f];
    int nstart = m->flex_nodeadr[f];
    int nend = m->flex_nodeadr[f] + m->flex_nodenum[f];

    // 0: vertices are the mesh vertices, 1: vertices are interpolated from nodal dofs
    if (m->flex_interp[f] == 0) {
      // centered: copy body position
      if (m->flex_centered[f]) {
        for (int i=vstart; i < vend; i++) {
          mju_copy3(d->flexvert_xpos+3*i, d->xpos+3*m->flex_vertbodyid[i]);
        }
      }

      // non-centered: map from local to global
      else {
        for (int i=vstart; i < vend; i++) {
          mju_mulMatVec3(d->flexvert_xpos+3*i, d->xmat+9*m->flex_vertbodyid[i], m->flex_vert+3*i);
          mju_addTo3(d->flexvert_xpos+3*i, d->xpos+3*m->flex_vertbodyid[i]);
        }
      }
    }

    // trilinear interpolation
    else {
      mjtNum nodexpos[mjMAXFLEXNODES];
      if (m->flex_centered[f]) {
        for (int i=nstart; i < nend; i++) {
          mju_copy3(nodexpos + 3*(i-nstart), d->xpos + 3*m->flex_nodebodyid[i]);
        }
      } else {
        for (int i=nstart; i < nend; i++) {
          int j = i - nstart;
          mju_mulMatVec3(nodexpos + 3*j, d->xmat + 9*m->flex_nodebodyid[i], m->flex_node + 3*i);
          mju_addTo3(nodexpos + 3*j, d->xpos + 3*m->flex_nodebodyid[i]);
        }
      }

      for (int i=vstart; i < vend; i++) {
        mju_zero3(d->flexvert_xpos+3*i);
        mjtNum* coord = m->flex_vert0 + 3*i;
        for (int j=0; j < nend-nstart; j++) {
          mjtNum coef = (j&1 ? coord[2] : 1-coord[2]) *
                        (j&2 ? coord[1] : 1-coord[1]) *
                        (j&4 ? coord[0] : 1-coord[0]);
          mju_addToScl3(d->flexvert_xpos+3*i, nodexpos+3*j, coef);
        }
      }
    }
  }

  // compute flex element aabb
  for (int f=0; f < m->nflex; f++) {
    int dim = m->flex_dim[f];

    // process elements of this flex
    int elemnum = m->flex_elemnum[f];
    for (int e=0; e < elemnum; e++) {
      const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
      const mjtNum* vert = d->flexvert_xpos + 3*m->flex_vertadr[f];

      // compute min and max along each global axis
      mjtNum xmin[3], xmax[3];
      mju_copy3(xmin, vert+3*edata[0]);
      mju_copy3(xmax, vert+3*edata[0]);
      for (int i=1; i <= dim; i++) {
        for (int j=0; j < 3; j++) {
          mjtNum value = vert[3*edata[i]+j];
          xmin[j] = mju_min(xmin[j], value);
          xmax[j] = mju_max(xmax[j], value);
        }
      }

      // compute aabb (center, size)
      int base = m->flex_elemadr[f] + e;
      d->flexelem_aabb[6*base+0] = 0.5*(xmax[0]+xmin[0]);
      d->flexelem_aabb[6*base+1] = 0.5*(xmax[1]+xmin[1]);
      d->flexelem_aabb[6*base+2] = 0.5*(xmax[2]+xmin[2]);
      d->flexelem_aabb[6*base+3] = 0.5*(xmax[0]-xmin[0]) + m->flex_radius[f];
      d->flexelem_aabb[6*base+4] = 0.5*(xmax[1]-xmin[1]) + m->flex_radius[f];
      d->flexelem_aabb[6*base+5] = 0.5*(xmax[2]-xmin[2]) + m->flex_radius[f];
    }
  }

  // update flex bhv_aabb_dyn if needed
  if (!mjDISABLED(mjDSBL_MIDPHASE)) {
    for (int f=0; f < m->nflex; f++) {
      if (m->flex_bvhadr[f] >= 0) {
        int flex_bvhadr = m->flex_bvhadr[f];
        int flex_bvhnum = m->flex_bvhnum[f];

        // copy element aabbs to bhv leaf aabbs
        for (int i=flex_bvhadr; i < flex_bvhadr+flex_bvhnum; i++) {
          if (m->bvh_nodeid[i] >= 0) {
            mju_copy(d->bvh_aabb_dyn + 6*(i - m->nbvhstatic),
                     d->flexelem_aabb + 6*(m->flex_elemadr[f] + m->bvh_nodeid[i]), 6);
          }
        }

        // update dynamic BVH
        mj_updateDynamicBVH(m, d, m->flex_bvhadr[f], m->flex_bvhnum[f]);
      }
    }
  }

  // allocate space
  mj_markStack(d);
  mjtNum* jac1 = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* jac2 = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* jacdif = mjSTACKALLOC(d, 3*nv, mjtNum);
  int* chain = issparse ? mjSTACKALLOC(d, nv, int) : NULL;

  // clear Jacobian: sparse or dense
  if (issparse) {
    mju_zeroInt(rowadr, m->nflexedge);
    mju_zeroInt(rownnz, m->nflexedge);
  } else {
    mju_zero(J, m->nflexedge*nv);
  }

  // compute lengths and Jacobians of edges
  for (int f=0; f < m->nflex; f++) {
    // skip if edges cannot generate forces
    if (m->flex_rigid[f] || m->flex_interp[f]) {
      continue;
    }

    // skip Jacobian if no built-in passive force is needed
    int skipjacobian = !m->flex_edgeequality[f] &&
                       !m->flex_edgedamping[f] &&
                       !m->flex_edgestiffness[f] &&
                       !m->flex_damping[f];

    // process edges of this flex
    int vbase = m->flex_vertadr[f];
    int ebase = m->flex_edgeadr[f];
    for (int e=0; e < m->flex_edgenum[f]; e++) {
      int v1 = m->flex_edge[2*(ebase+e)];
      int v2 = m->flex_edge[2*(ebase+e)+1];
      int b1 = m->flex_vertbodyid[vbase+v1];
      int b2 = m->flex_vertbodyid[vbase+v2];
      mjtNum* pos1 = d->flexvert_xpos + 3*(vbase+v1);
      mjtNum* pos2 = d->flexvert_xpos + 3*(vbase+v2);

      // vec = unit vector from v1 to v2, compute edge length
      mjtNum vec[3];
      mju_sub3(vec, pos2, pos1);
      d->flexedge_length[ebase+e] = mju_normalize3(vec);

      // skip Jacobian if not needed
      if (skipjacobian) {
        continue;
      }

      // sparse edge Jacobian
      if (issparse) {
        // set rowadr
        if (ebase+e > 0) {
          rowadr[ebase+e] = rowadr[ebase+e-1] + rownnz[ebase+e-1];
        }

        // get endpoint Jacobians, subtract
        int NV = mj_jacDifPair(m, d, chain, b1, b2, pos1, pos2,
                               jac1, jac2, jacdif, NULL, NULL, NULL);

        // no dofs: skip
        if (!NV) {
          continue;
        }

        // apply chain rule to compute edge Jacobian
        mju_mulMatTVec(J + rowadr[ebase+e], jacdif, vec, 3, NV);

        // copy sparsity info
        rownnz[ebase+e] = NV;
        mju_copyInt(d->flexedge_J_colind + rowadr[ebase+e], chain, NV);
      }

      // dense edge Jacobian
      else {
        // get endpoint Jacobians, subtract
        mj_jac(m, d, jac1, NULL, pos1, b1);
        mj_jac(m, d, jac2, NULL, pos2, b2);
        mju_sub(jacdif, jac2, jac1, 3*nv);

        // apply chain rule to compute edge Jacobian
        mju_mulMatTVec(J + (ebase+e)*nv, jacdif, vec, 3, nv);
      }
    }
  }

  mj_freeStack(d);
}



// compute tendon lengths and moments
void mj_tendon(const mjModel* m, mjData* d) {
  int issparse = mj_isSparse(m), nv = m->nv, nten = m->ntendon;
  int *rownnz = d->ten_J_rownnz, *rowadr = d->ten_J_rowadr, *colind = d->ten_J_colind;
  mjtNum *L = d->ten_length, *J = d->ten_J;

  if (!nten) {
    return;
  }

  // allocate stack arrays
  int *chain = NULL, *buf_ind = NULL;
  mjtNum *jac1, *jac2, *jacdif, *tmp, *sparse_buf = NULL;
  mj_markStack(d);
  jac1 = mjSTACKALLOC(d, 3*nv, mjtNum);
  jac2 = mjSTACKALLOC(d, 3*nv, mjtNum);
  jacdif = mjSTACKALLOC(d, 3*nv, mjtNum);
  tmp = mjSTACKALLOC(d, nv, mjtNum);
  if (issparse) {
    chain = mjSTACKALLOC(d, nv, int);
    buf_ind = mjSTACKALLOC(d, nv, int);
    sparse_buf = mjSTACKALLOC(d, nv, mjtNum);
  }

  // clear results
  mju_zero(L, nten);

  // clear Jacobian: sparse or dense
  if (issparse) {
    mju_zeroInt(rownnz, nten);
  } else {
    mju_zero(J, nten*nv);
  }

  // loop over tendons
  int wrapcount = 0;
  for (int i=0; i < nten; i++) {
    // initialize tendon path
    int adr = m->tendon_adr[i];
    d->ten_wrapadr[i] = wrapcount;
    d->ten_wrapnum[i] = 0;
    int tendon_num = m->tendon_num[i];

    // sparse Jacobian row init
    if (issparse) {
      rowadr[i] = (i > 0 ? rowadr[i-1] + rownnz[i-1] : 0);
    }

    // process fixed tendon
    if (m->wrap_type[adr] == mjWRAP_JOINT) {
      // process all defined joints
      for (int j=0; j < tendon_num; j++) {
        // get joint id
        int k = m->wrap_objid[adr+j];

        // add to length
        L[i] += m->wrap_prm[adr+j] * d->qpos[m->jnt_qposadr[k]];

        // add to moment
        if (issparse) {
          rownnz[i] = mju_combineSparse(J+rowadr[i], &m->wrap_prm[adr+j], 1, 1,
                                        rownnz[i], 1,
                                        colind+rowadr[i], &m->jnt_dofadr[k],
                                        sparse_buf, buf_ind);
        }

        // add to moment: dense
        else {
          J[i*nv + m->jnt_dofadr[k]] = m->wrap_prm[adr+j];
        }
      }

      continue;
    }

    // process spatial tendon
    mjtNum divisor = 1;
    int wraptype, j = 0;
    while (j < tendon_num-1) {
      // get 1st and 2nd object
      int type0 = m->wrap_type[adr+j+0];
      int type1 = m->wrap_type[adr+j+1];
      int id0 = m->wrap_objid[adr+j+0];
      int id1 = m->wrap_objid[adr+j+1];

      // pulley
      if (type0 == mjWRAP_PULLEY || type1 == mjWRAP_PULLEY) {
        // get divisor, insert obj=-2
        if (type0 == mjWRAP_PULLEY) {
          divisor = m->wrap_prm[adr+j];
          mju_zero3(d->wrap_xpos+wrapcount*3);
          d->wrap_obj[wrapcount] = -2;
          d->ten_wrapnum[i]++;
          wrapcount++;
        }

        // move to next
        j++;
        continue;
      }

      // init sequence; assume it starts with site
      mjtNum wlen = -1;
      int wrapid = -1;
      mjtNum wpnt[12];
      mju_copy3(wpnt, d->site_xpos+3*id0);
      int wbody[4];
      wbody[0] = m->site_bodyid[id0];

      // second object is geom: process site-geom-site
      if (type1 == mjWRAP_SPHERE || type1 == mjWRAP_CYLINDER) {
        // reassign, get 2nd site info
        wraptype = type1;
        wrapid = id1;
        type1 = m->wrap_type[adr+j+2];
        id1 = m->wrap_objid[adr+j+2];

        // do wrapping, possibly get 2 extra points (wlen>=0)
        int sideid = mju_round(m->wrap_prm[adr+j+1]);
        if (sideid < -1 || sideid >= m->nsite) {
          mjERROR("invalid sideid %d in wrap_prm", sideid);  // SHOULD NOT OCCUR
        }

        wlen = mju_wrap(wpnt+3, d->site_xpos+3*id0, d->site_xpos+3*id1,
                        d->geom_xpos+3*wrapid, d->geom_xmat+9*wrapid, m->geom_size[3*wrapid],
                        wraptype, (sideid >= 0 ? d->site_xpos+3*sideid : 0));
      } else {
        wraptype = mjWRAP_NONE;
      }

      // complete sequence, accumulate lengths
      if (wlen < 0) {
        mju_copy3(wpnt+3, d->site_xpos+3*id1);
        wbody[1] = m->site_bodyid[id1];
        L[i] += mju_dist3(wpnt, wpnt+3) / divisor;
      } else {
        mju_copy3(wpnt+9, d->site_xpos+3*id1);
        wbody[1] = wbody[2] = m->geom_bodyid[wrapid];
        wbody[3] = m->site_bodyid[id1];
        L[i] += (mju_dist3(wpnt, wpnt+3) + wlen + mju_dist3(wpnt+6, wpnt+9)) / divisor;
      }

      // accumulate moments if consecutive points are in different bodies
      for (int k=0; k < (wlen < 0 ? 1 : 3); k++) {
        if (wbody[k] != wbody[k+1]) {
          // get 3D position difference, normalize
          mjtNum dif[3];
          mju_sub3(dif, wpnt+3*k+3, wpnt+3*k);
          mju_normalize3(dif);

          // sparse
          if (issparse) {
            // get endpoint Jacobians, subtract
            int NV = mj_jacDifPair(m, d, chain,
                                   wbody[k], wbody[k+1], wpnt+3*k, wpnt+3*k+3,
                                   jac1, jac2, jacdif, NULL, NULL, NULL);

            // no dofs: skip
            if (!NV) {
              continue;
            }

            // apply chain rule to compute tendon Jacobian
            mju_mulMatTVec(tmp, jacdif, dif, 3, NV);

            // add to existing
            rownnz[i] = mju_combineSparse(J+rowadr[i], tmp, 1, 1/divisor,
                                          rownnz[i], NV, colind+rowadr[i],
                                          chain, sparse_buf, buf_ind);
          }

          // dense
          else {
            // get endpoint Jacobians, subtract
            mj_jac(m, d, jac1, 0, wpnt+3*k, wbody[k]);
            mj_jac(m, d, jac2, 0, wpnt+3*k+3, wbody[k+1]);
            mju_sub(jacdif, jac2, jac1, 3*nv);

            // apply chain rule to compute tendon Jacobian
            mju_mulMatTVec(tmp, jacdif, dif, 3, nv);

            // add to existing
            mju_addToScl(J + i*nv, tmp, 1/divisor, nv);
          }
        }
      }

      // assign to wrap
      mju_copy(d->wrap_xpos+wrapcount*3, wpnt, (wlen < 0 ? 3 : 9));
      d->wrap_obj[wrapcount] = -1;
      if (wlen >= 0) {
        d->wrap_obj[wrapcount+1] = d->wrap_obj[wrapcount+2] = wrapid;
      }
      d->ten_wrapnum[i] += (wlen < 0 ? 1 : 3);
      wrapcount += (wlen < 0 ? 1 : 3);

      // advance
      j += (wraptype != mjWRAP_NONE ? 2 : 1);

      // assign last site before pulley or tendon end
      if (j == tendon_num-1 || m->wrap_type[adr+j+1] == mjWRAP_PULLEY) {
        mju_copy3(d->wrap_xpos+wrapcount*3, d->site_xpos+3*id1);
        d->wrap_obj[wrapcount] = -1;
        d->ten_wrapnum[i]++;
        wrapcount++;
      }
    }
  }

  mj_freeStack(d);
}



// compute time derivative of dense tendon Jacobian for one tendon
void mj_tendonDot(const mjModel* m, mjData* d, int id, mjtNum* Jdot) {
  int nv = m->nv;

  // tendon id is invalid: return
  if (id < 0 || id >= m->ntendon) {
    return;
  }

  // clear output
  mju_zero(Jdot, nv);

  // fixed tendon has zero Jdot: return
  int adr = m->tendon_adr[id];
  if (m->wrap_type[adr] == mjWRAP_JOINT) {
    return;
  }

  // allocate stack arrays
  mj_markStack(d);
  mjtNum* jac1 = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* jac2 = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* jacdif = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* tmp = mjSTACKALLOC(d, nv, mjtNum);

  // process spatial tendon
  mjtNum divisor = 1;
  int wraptype, j = 0;
  int num = m->tendon_num[id];
  while (j < num-1) {
    // get 1st and 2nd object
    int type0 = m->wrap_type[adr+j+0];
    int type1 = m->wrap_type[adr+j+1];
    int id0 = m->wrap_objid[adr+j+0];
    int id1 = m->wrap_objid[adr+j+1];

    // pulley
    if (type0 == mjWRAP_PULLEY || type1 == mjWRAP_PULLEY) {
      // get divisor, insert obj=-2
      if (type0 == mjWRAP_PULLEY) {
        divisor = m->wrap_prm[adr+j];
      }

      // move to next
      j++;
      continue;
    }

    // init sequence; assume it starts with site
    mjtNum wpnt[6];
    mju_copy3(wpnt, d->site_xpos+3*id0);
    mjtNum vel[6];
    mj_objectVelocity(m, d, mjOBJ_SITE, id0, vel, /*flg_local=*/0);
    mjtNum wvel[6] = {vel[3], vel[4], vel[5], 0, 0, 0};
    int wbody[2];
    wbody[0] = m->site_bodyid[id0];

    // second object is geom: process site-geom-site
    if (type1 == mjWRAP_SPHERE || type1 == mjWRAP_CYLINDER) {
      // TODO(tassa) support geom wrapping (requires derivatives of mju_wrap)
      mjERROR("geom wrapping not supported");
    } else {
      wraptype = mjWRAP_NONE;
    }

    // complete sequence
    wbody[1] = m->site_bodyid[id1];
    mju_copy3(wpnt+3, d->site_xpos+3*id1);
    mj_objectVelocity(m, d, mjOBJ_SITE, id1, vel, /*flg_local=*/0);
    mju_copy3(wvel+3, vel+3);

    // accumulate moments if consecutive points are in different bodies
    if (wbody[0] != wbody[1]) {
      // dpnt = 3D position difference, normalize
      mjtNum dpnt[3];
      mju_sub3(dpnt, wpnt+3, wpnt);
      mjtNum norm = mju_normalize3(dpnt);

      // dvel = d / dt (dpnt)
      mjtNum dvel[3];
      mju_sub3(dvel, wvel+3, wvel);
      mjtNum dot = mju_dot3(dpnt, dvel);
      mju_addToScl3(dvel, dpnt, -dot);
      mju_scl3(dvel, dvel, norm > mjMINVAL ? 1/norm : 0);

      // TODO(tassa ) write sparse branch, requires mj_jacDotSparse
      // if (mj_isSparse(m)) { ... }

      // get endpoint JacobianDots, subtract
      mj_jacDot(m, d, jac1, 0, wpnt, wbody[0]);
      mj_jacDot(m, d, jac2, 0, wpnt+3, wbody[1]);
      mju_sub(jacdif, jac2, jac1, 3*nv);

      // chain rule, first term: Jdot += d/dt(jac2 - jac1) * dpnt
      mju_mulMatTVec(tmp, jacdif, dpnt, 3, nv);

      // add to existing
      mju_addToScl(Jdot, tmp, 1/divisor, nv);

      // get endpoint Jacobians, subtract
      mj_jac(m, d, jac1, 0, wpnt, wbody[0]);
      mj_jac(m, d, jac2, 0, wpnt+3, wbody[1]);
      mju_sub(jacdif, jac2, jac1, 3*nv);

      // chain rule, second term: Jdot += (jac2 - jac1) * d/dt(dpnt)
      mju_mulMatTVec(tmp, jacdif, dvel, 3, nv);

      // add to existing
      mju_addToScl(Jdot, tmp, 1/divisor, nv);
    }

    // advance
    j += (wraptype != mjWRAP_NONE ? 2 : 1);
  }

  mj_freeStack(d);
}



// compute actuator/transmission lengths and moments
void mj_transmission(const mjModel* m, mjData* d) {
  int nv = m->nv, nu = m->nu;

  // nothing to do
  if (!nu) {
    return;
  }

  // outputs
  mjtNum* length = d->actuator_length;
  mjtNum* moment = d->actuator_moment;
  int *rownnz = d->moment_rownnz;
  int *rowadr = d->moment_rowadr;
  int *colind = d->moment_colind;

  // allocate Jacbians
  mj_markStack(d);
  mjtNum* jac  = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* jacA = mjSTACKALLOC(d, 3*nv, mjtNum);
  mjtNum* jacS = mjSTACKALLOC(d, 3*nv, mjtNum);

  // define stack variables required for body transmission, don't allocate
  int issparse = mj_isSparse(m);
  mjtNum* efc_force = NULL;  // used as marker for allocation requirement
  mjtNum *moment_exclude, *jacdifp, *jac1p, *jac2p;
  int *chain;

  // define stack variables required for site transmission, don't allocate
  mjtNum *jacref = NULL, *moment_tmp = NULL;

  // compute lengths and moments
  for (int i=0; i < nu; i++) {
    rowadr[i] = i == 0 ? 0 : rowadr[i-1] + rownnz[i-1];
    int nnz, adr = rowadr[i];

    // extract info
    int id = m->actuator_trnid[2*i];
    mjtNum* gear = m->actuator_gear+6*i;

    // process according to transmission type
    switch ((mjtTrn) m->actuator_trntype[i]) {
    case mjTRN_JOINT:                   // joint
    case mjTRN_JOINTINPARENT:           // joint, force in parent frame
      // slide and hinge joint: scalar gear
      if (m->jnt_type[id] == mjJNT_SLIDE || m->jnt_type[id] == mjJNT_HINGE) {
        // sparsity
        rownnz[i] = 1;
        colind[adr] = m->jnt_dofadr[id];

        length[i] = d->qpos[m->jnt_qposadr[id]]*gear[0];
        moment[adr] = gear[0];
      }

      // ball joint: 3D wrench gear
      else if (m->jnt_type[id] == mjJNT_BALL) {
        // axis: expmap representation of quaternion
        mjtNum axis[3], quat[4];
        mju_copy4(quat, d->qpos+m->jnt_qposadr[id]);
        mju_normalize4(quat);
        mju_quat2Vel(axis, quat, 1);

        // gearAxis: rotate to parent frame if necessary
        mjtNum gearAxis[3];
        if (m->actuator_trntype[i] == mjTRN_JOINT) {
          mju_copy3(gearAxis, gear);
        } else {
          mju_negQuat(quat, quat);
          mju_rotVecQuat(gearAxis, gear, quat);
        }

        // length: axis*gearAxis
        length[i] = mju_dot3(axis, gearAxis);

        // dof start address
        int jnt_dofadr = m->jnt_dofadr[id];

        // sparsity
        for (int j = 0; j < 3; j++) {
          colind[adr+j] = jnt_dofadr + j;
        }
        rownnz[i] = 3;

        // moment: gearAxis
        mju_copy3(moment+adr, gearAxis);
      }

      // free joint: 6D wrench gear
      else {
        // cannot compute meaningful length, set to 0
        length[i] = 0;

        // gearAxis: rotate to world frame if necessary
        mjtNum gearAxis[3];
        if (m->actuator_trntype[i] == mjTRN_JOINT) {
          mju_copy3(gearAxis, gear+3);
        } else {
          mjtNum quat[4];
          mju_copy4(quat, d->qpos+m->jnt_qposadr[id]+3);
          mju_normalize4(quat);
          mju_negQuat(quat, quat);
          mju_rotVecQuat(gearAxis, gear+3, quat);
        }

        // dof start address
        int jnt_dofadr = m->jnt_dofadr[id];

        // sparsity
        for (int j = 0; j < 6; j++) {
          colind[adr+j] = jnt_dofadr + j;
        }
        rownnz[i] = 6;

        // moment: gear(tran), gearAxis
        mju_copy3(moment+adr, gear);
        mju_copy3(moment+adr+3, gearAxis);
      }
      break;

    case mjTRN_SLIDERCRANK:             // slider-crank
      {
        // get data
        int idslider = m->actuator_trnid[2*i+1];
        mjtNum rod = m->actuator_cranklength[i];
        mjtNum axis[3] = {d->site_xmat[9 * idslider + 2],
                          d->site_xmat[9 * idslider + 5],
                          d->site_xmat[9 * idslider + 8]};
        mjtNum vec[3];
        mju_sub3(vec, d->site_xpos+3*id, d->site_xpos+3*idslider);

        // compute length and determinant
        //  length = a'*v - sqrt(det);  det = (a'*v)^2 + r^2 - v'*v)
        mjtNum av = mju_dot3(vec, axis);
        mjtNum sdet, det = av*av + rod*rod - mju_dot3(vec, vec);
        int ok = 1;
        if (det <= 0) {
          ok = 0;
          sdet = 0;
          length[i] = av;
        } else {
          sdet = mju_sqrt(det);
          length[i] = av - sdet;
        }

        // compute derivatives of length w.r.t. vec and axis
        mjtNum dlda[3], dldv[3];
        if (ok) {
          mju_scl3(dldv, axis, 1-av/sdet);
          mju_scl3(dlda, vec, 1/sdet);        // use dlda as temp
          mju_addTo3(dldv, dlda);

          mju_scl3(dlda, vec, 1-av/sdet);
        } else {
          mju_copy3(dlda, vec);
          mju_copy3(dldv, axis);
        }

        // get Jacobians of axis(jacA) and vec(jac)
        mj_jacPointAxis(m, d, jacS, jacA, d->site_xpos+3*idslider,
                        axis, m->site_bodyid[idslider]);
        mj_jacSite(m, d, jac, 0, id);
        mju_subFrom(jac, jacS, 3*nv);

        // clear moment
        mju_zero(moment + adr, nv);

        // apply chain rule
        for (int j=0; j < nv; j++) {
          for (int k=0; k < 3; k++) {
            moment[adr+j] += dlda[k]*jacA[k*nv+j] + dldv[k]*jac[k*nv+j];
          }
        }

        // scale by gear ratio
        length[i] *= gear[0];
        for (int j = 0; j < nv; j++) {
          moment[adr+j] *= gear[0];
        }

        // sparsity (compress)
        nnz = 0;
        for (int j = 0; j < nv; j++) {
          if (moment[adr+j]) {
            moment[adr+nnz] = moment[adr+j];
            colind[adr+nnz] = j;
            nnz++;
          }
        }
        rownnz[i] = nnz;
      }
      break;

    case mjTRN_TENDON:                  // tendon
      length[i] = d->ten_length[id]*gear[0];

      // moment: sparse or dense
      if (issparse) {
        // sparsity
        int ten_J_rownnz = d->ten_J_rownnz[id];
        int ten_J_rowadr = d->ten_J_rowadr[id];
        rownnz[i] = ten_J_rownnz;
        mju_copyInt(colind + adr, d->ten_J_colind + ten_J_rowadr, ten_J_rownnz);

        mju_scl(moment + adr, d->ten_J + ten_J_rowadr, gear[0], ten_J_rownnz);
      } else {
        mju_scl(moment+adr, d->ten_J + id*nv, gear[0], nv);

        // sparsity (compress)
        nnz = 0;
        for (int j = 0; j < nv; j++) {
          if (moment[adr+j]) {
            moment[adr+nnz] = moment[adr+j];
            colind[adr+nnz] = j;
            nnz++;
          }
        }
        rownnz[i] = nnz;
      }
      break;

    case mjTRN_SITE:                    // site
      // get site translation (jac) and rotation (jacS) Jacobians in global frame
      mj_jacSite(m, d, jac, jacS, id);

      // clear length
      length[i] = 0;

      // reference site undefined
      if (m->actuator_trnid[2*i+1] == -1) {
        // wrench: gear expressed in global frame
        mjtNum wrench[6];
        mju_mulMatVec3(wrench, d->site_xmat+9*id, gear);      // translation
        mju_mulMatVec3(wrench+3, d->site_xmat+9*id, gear+3);  // rotation

        // moment: global Jacobian projected on wrench
        mju_mulMatTVec(moment+adr, jac, wrench, 3, nv);       // translation
        mju_mulMatTVec(jac, jacS, wrench+3, 3, nv);           // rotation
        mju_addTo(moment+adr, jac, nv);                       // add the two
      }

      // reference site defined
      else {
        int refid = m->actuator_trnid[2*i+1];
        if (!jacref) jacref = mjSTACKALLOC(d, 3*nv, mjtNum);

        // initialize last dof address for each body
        int b0 = m->body_weldid[m->site_bodyid[id]];
        int b1 = m->body_weldid[m->site_bodyid[refid]];
        int dofadr0 = m->body_dofadr[b0] + m->body_dofnum[b0] - 1;
        int dofadr1 = m->body_dofadr[b1] + m->body_dofnum[b1] - 1;

        // find common ancestral dof, if any
        int dofadr_common = -1;
        if (dofadr0 >= 0 && dofadr1 >= 0) {
          // traverse up the tree until common ancestral dof is found
          while (dofadr0 != dofadr1) {
            if (dofadr0 < dofadr1) {
              dofadr1 = m->dof_parentid[dofadr1];
            } else {
              dofadr0 = m->dof_parentid[dofadr0];
            }
            if (dofadr0 == -1 || dofadr1 == -1) {
              // reached tree root, no common ancestral dof
              break;
            }
          }

          // found common ancestral dof
          if (dofadr0 == dofadr1) {
            dofadr_common = dofadr0;
          }
        }

        // clear moment
        mju_zero(moment+adr, nv);

        // translational transmission
        if (!mju_isZero(gear, 3)) {
          // vec: site position in reference site frame
          mjtNum vec[3];
          mju_sub3(vec, d->site_xpos+3*id, d->site_xpos+3*refid);
          mju_mulMatTVec3(vec, d->site_xmat+9*refid, vec);

          // length: dot product with gear
          length[i] += mju_dot3(vec, gear);

          // jacref: global Jacobian of reference site
          mj_jacSite(m, d, jacref, NULL, refid);

          // subtract jacref from jac
          mju_subFrom(jac, jacref, 3*nv);

          // if common ancestral dof was found, clear the columns of its parental chain
          int da = dofadr_common;
          while (da >= 0) {
            jac[nv*0 + da] = 0;
            jac[nv*1 + da] = 0;
            jac[nv*2 + da] = 0;
            da = m->dof_parentid[da];
          }

          // wrench: translational gear expressed in global frame
          mjtNum wrench[6];
          mju_mulMatVec3(wrench, d->site_xmat+9*refid, gear);

          // moment: global Jacobian projected on wrench
          mju_mulMatTVec(moment+adr, jac, wrench, 3, nv);
        }

        // rotational transmission
        if (!mju_isZero(gear+3, 3)) {
          mjtNum refquat[4];

          // get site and refsite quats from parent bodies (avoiding mju_mat2Quat)
          mjtNum quat[4];
          mju_mulQuat(quat, m->site_quat+4*id, d->xquat+4*m->site_bodyid[id]);
          mju_mulQuat(refquat, m->site_quat+4*refid, d->xquat+4*m->site_bodyid[refid]);

          // convert difference to expmap (axis-angle)
          mjtNum vec[3];
          mju_subQuat(vec, quat, refquat);

          // add length: dot product with gear
          length[i] += mju_dot3(vec, gear+3);

          // jacref: global rotational Jacobian of reference site
          mj_jacSite(m, d, NULL, jacref, refid);

          // subtract jacref from jacS
          mju_subFrom(jacS, jacref, 3*nv);

          // if common ancestral dof was found, clear the columns of its parental chain
          int da = dofadr_common;
          while (da >= 0) {
            jacS[nv*0 + da] = 0;
            jacS[nv*1 + da] = 0;
            jacS[nv*2 + da] = 0;
            da = m->dof_parentid[da];
          }

          // wrench: rotational gear expressed in global frame
          mjtNum wrench[6];
          mju_mulMatVec3(wrench, d->site_xmat+9*refid, gear+3);

          // moment_tmp: global Jacobian projected on wrench, add to moment
          if (!moment_tmp) moment_tmp = mjSTACKALLOC(d, nv, mjtNum);
          mju_mulMatTVec(moment_tmp, jacS, wrench, 3, nv);
          mju_addTo(moment+adr, moment_tmp, nv);
        }
      }

      // sparsity (compress)
      nnz = 0;
      for (int j = 0; j < nv; j++) {
        if (moment[adr+j]) {
          moment[adr+nnz] = moment[adr+j];
          colind[adr+nnz] = j;
          nnz++;
        }
      }
      rownnz[i] = nnz;

      break;

    case mjTRN_BODY:                  // body (adhesive contacts)
      // cannot compute meaningful length, set to 0
      length[i] = 0;

      // clear moment
      mju_zero(moment+adr, nv);

      // moment is average of all contact normal Jacobians
      {
        // allocate stack variables for the first mjTRN_BODY
        if (!efc_force) {
          efc_force = mjSTACKALLOC(d, d->nefc, mjtNum);
          moment_exclude = mjSTACKALLOC(d, nv, mjtNum);
          jacdifp = mjSTACKALLOC(d, 3*nv, mjtNum);
          jac1p = mjSTACKALLOC(d, 3*nv, mjtNum);
          jac2p = mjSTACKALLOC(d, 3*nv, mjtNum);
          chain = issparse ? mjSTACKALLOC(d, nv, int) : NULL;
        }

        // clear efc_force and moment_exclude
        mju_zero(efc_force, d->nefc);
        mju_zero(moment_exclude, nv);

        // count all relevant contacts, accumulate Jacobians
        int counter = 0, ncon = d->ncon;
        for (int j=0; j < ncon; j++) {
          const mjContact* con = d->contact+j;

          // get geom ids
          int g1 = con->geom[0];
          int g2 = con->geom[1];

          // contact involving flex, continue
          if (g1 < 0 || g2 < 0) {
            continue;
          }

          // get body ids
          int b1 = m->geom_bodyid[g1];
          int b2 = m->geom_bodyid[g2];

          // irrelevant contact, continue
          if (b1 != id && b2 != id) {
            continue;
          }

          // mark contact normals in efc_force
          if (!con->exclude) {
            counter++;

            // condim 1 or elliptic cones: normal is in the first row
            if (con->dim == 1 || m->opt.cone == mjCONE_ELLIPTIC) {
              efc_force[con->efc_address] = 1;
            }

            // pyramidal cones: average all pyramid directions
            else {
              int npyramid = con->dim-1;  // number of frictional directions
              for (int k=0; k < 2*npyramid; k++) {
                efc_force[con->efc_address+k] = 0.5/npyramid;
              }
            }
          }

          // excluded contact in gap: get sparse or dense Jacobian, accumulate
          else if (con->exclude == 1) {
            counter++;

            // get Jacobian difference
            int NV = mj_jacDifPair(m, d, chain, b1, b2, con->pos, con->pos,
                                   jac1p, jac2p, jacdifp, NULL, NULL, NULL);

            // project Jacobian along the normal of the contact frame
            mju_mulMatMat(jac, con->frame, jacdifp, 1, 3, NV);

            // accumulate in moment_exclude
            if (issparse) {
              for (int k=0; k < NV; k++) {
                moment_exclude[chain[k]] += jac[k];
              }
            } else {
              mju_addTo(moment_exclude, jac, nv);
            }
          }
        }

        // moment is average over contact normal Jacobians, make negative for adhesion
        if (counter) {
          // accumulate active contact Jacobians into moment
          mj_mulJacTVec(m, d, moment+adr, efc_force);

          // add Jacobians from excluded contacts
          mju_addTo(moment+adr, moment_exclude, nv);

          // normalize by total contacts, flip sign
          mju_scl(moment+adr, moment+adr, -1.0/counter, nv);
        }
      }

      // sparsity (compress)
      nnz = 0;
      for (int j = 0; j < nv; j++) {
        if (moment[adr+j]) {
          moment[adr+nnz] = moment[adr+j];
          colind[adr+nnz] = j;
          nnz++;
        }
      }
      rownnz[i] = nnz;

      break;

    default:
      mjERROR("unknown transmission type %d", m->actuator_trntype[i]);  // SHOULD NOT OCCUR
    }
  }

  mj_freeStack(d);
}



//-------------------------- inertia ---------------------------------------------------------------

// add tendon armature to qM
void mj_tendonArmature(const mjModel* m, mjData* d) {
  int nv = m->nv, ntendon = m->ntendon, issparse = mj_isSparse(m);

  for (int k=0; k < ntendon; k++) {
    mjtNum armature = m->tendon_armature[k];

    if (!armature) {
      continue;
    }

    // dense
    if (!issparse) {
      mjtNum* ten_J = d->ten_J + nv*k;
      for (int i=0; i < m->nv; i++) {
        int Madr = m->dof_Madr[i];
        for (int j = i; j >= 0; j = m->dof_parentid[j]) {
          d->qM[Madr++] += armature * ten_J[j] * ten_J[i];
        }
      }
    }

    // sparse
    else {
      // get sparse info for tendon k
      int rowadr = d->ten_J_rowadr[k];
      int rownnz = d->ten_J_rownnz[k];
      const int* colind = d->ten_J_colind + rowadr;
      mjtNum* ten_J = d->ten_J + rowadr;

      // iterate forward on nonzero rows i
      for (int adr_i=0; adr_i < rownnz; adr_i++) {
        int i = colind[adr_i];
        int Madr = m->dof_Madr[i];
        int adr_j = rownnz - 1;

        // iterate backward on ancestors of i, find matching column j
        for (int j = i; j >= 0; j = m->dof_parentid[j]) {
          // reduce adr_j until column index is no bigger than j
          while (colind[adr_j] > j && adr_j >= 0) {
            adr_j--;
          }

          // found match, update qM
          if (colind[adr_j] == j) {
            d->qM[Madr++] += armature * ten_J[adr_j] * ten_J[adr_i];
          }
        }
      }
    }
  }
}



// composite rigid body inertia algorithm
void mj_crb(const mjModel* m, mjData* d) {
  mjtNum buf[6];
  mjtNum* crb = d->crb;
  int last_body = m->nbody - 1, nv = m->nv;

  // crb = cinert
  mju_copy(crb, d->cinert, 10*m->nbody);

  // backward pass over bodies, accumulate composite inertias
  for (int i=last_body; i > 0; i--) {
    if (m->body_parentid[i] > 0) {
      mju_addTo(crb+10*m->body_parentid[i], crb+10*i, 10);
    }
  }

  // clear qM
  mju_zero(d->qM, m->nM);

  // dense forward pass over dofs
  for (int i=0; i < nv; i++) {
    // process block of diagonals (simple bodies)
    if (m->dof_simplenum[i]) {
      int n = i + m->dof_simplenum[i];
      for (; i < n; i++) {
        d->qM[m->dof_Madr[i]] = m->dof_M0[i];
      }

      // finish or else fall through with next row
      if (i == nv) {
        break;
      }
    }

    // init M(i,i) with armature inertia
    int Madr_ij = m->dof_Madr[i];
    d->qM[Madr_ij] = m->dof_armature[i];

    // precompute buf = crb_body_i * cdof_i
    mju_mulInertVec(buf, crb+10*m->dof_bodyid[i], d->cdof+6*i);

    // sparse backward pass over ancestors
    for (int j=i; j >= 0; j = m->dof_parentid[j]) {
      // M(i,j) += cdof_j * (crb_body_i * cdof_i)
      d->qM[Madr_ij++] += mju_dot(d->cdof+6*j, buf, 6);
    }
  }
}



void mj_makeM(const mjModel* m, mjData* d) {
  TM_START;
  mj_crb(m, d);
  mj_tendonArmature(m, d);
  mju_gather(d->M, d->qM, d->mapM2M, m->nC);
  TM_END(mjTIMER_POS_INERTIA);
}



// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd
// (legacy implementation)
void mj_factorI_legacy(const mjModel* m, mjData* d, const mjtNum* M, mjtNum* qLD,
                       mjtNum* qLDiagInv) {
  int cnt;
  int Madr_kk, Madr_ki;
  mjtNum tmp;

  // local copies of key variables
  int* dof_Madr = m->dof_Madr;
  int* dof_parentid = m->dof_parentid;
  int nv = m->nv;

  // copy M into LD
  mju_copy(qLD, M, m->nM);

  // dense backward loop over dofs (regular only, simple diagonal already copied)
  for (int k=nv-1; k >= 0; k--) {
    // get address of M(k,k)
    Madr_kk = dof_Madr[k];

    // check for small/negative numbers on diagonal
    if (qLD[Madr_kk] < mjMINVAL) {
      mj_warning(d, mjWARN_INERTIA, k);
      qLD[Madr_kk] = mjMINVAL;
    }

    // skip the rest if simple
    if (m->dof_simplenum[k]) {
      continue;
    }

    // sparse backward loop over ancestors of k (excluding k)
    Madr_ki = Madr_kk + 1;
    int i = dof_parentid[k];
    while (i >= 0) {
      tmp = qLD[Madr_ki] / qLD[Madr_kk];          // tmp = M(k,i) / M(k,k)

      // get number of ancestors of i (including i)
      if (i < nv-1) {
        cnt = dof_Madr[i+1] - dof_Madr[i];
      } else {
        cnt = m->nM - dof_Madr[i+1];
      }

      // M(i,j) -= M(k,j) * tmp
      mju_addToScl(qLD+dof_Madr[i], qLD+Madr_ki, -tmp, cnt);

      qLD[Madr_ki] = tmp;                         // M(k,i) = tmp

      // advance to i's parent
      i = dof_parentid[i];
      Madr_ki++;
    }
  }

  // compute 1/diag(D)
  for (int i=0; i < nv; i++) {
    qLDiagInv[i] = 1.0 / qLD[dof_Madr[i]];
  }
}



// sparse L'*D*L factorizaton of the inertia matrix M, assumed spd
void mj_factorM(const mjModel* m, mjData* d) {
  TM_START;
  mju_copy(d->qLD, d->M, m->nC);
  mj_factorI(d->qLD, d->qLDiagInv, m->nv, d->M_rownnz, d->M_rowadr, d->M_colind);
  TM_ADD(mjTIMER_POS_INERTIA);
}



// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd
void mj_factorI(mjtNum* mat, mjtNum* diaginv, int nv,
                const int* rownnz, const int* rowadr, const int* colind) {
  // backward loop over rows
  for (int k=nv-1; k >= 0; k--) {
    // get row k's address, diagonal index, inverse diagonal value
    int start = rowadr[k];
    int diag = rownnz[k] - 1;
    int end = start + diag;
    mjtNum invD = 1 / mat[end];
    if (diaginv) diaginv[k] = invD;

    // update triangle above row k
    for (int adr=end - 1; adr >= start; adr--) {
      // update row i < k:  L(i, 0..i) -= L(i, 0..i) * L(k, i) / L(k, k)
      int i = colind[adr];
      mju_addToScl(mat + rowadr[i], mat + start, -mat[adr] * invD, rownnz[i]);
    }

    // update row k:  L(k, :) /= L(k, k)
    mju_scl(mat + start, mat + start, invD, diag);
  }
}



// in-place sparse backsubstitution:  x = inv(L'*D*L)*x
// (legacy implementation)
void mj_solveLD_legacy(const mjModel* m, mjtNum* restrict x, int n,
                       const mjtNum* qLD, const mjtNum* qLDiagInv) {
  // local copies of key variables
  int* dof_Madr = m->dof_Madr;
  int* dof_parentid = m->dof_parentid;
  int nv = m->nv;

  // single vector
  if (n == 1) {
    // x <- inv(L') * x; skip simple, exploit sparsity of input vector
    for (int i=nv-1; i >= 0; i--) {
      if (!m->dof_simplenum[i] && x[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        // read directly from x[i] since i cannot be a parent of itself
        while (j >= 0) {
          x[j] -= qLD[Madr_ij++]*x[i];         // x(j) -= L(i,j) * x(i)

          // advance to parent
          j = dof_parentid[j];
        }
      }
    }

    // x <- inv(D) * x
    for (int i=0; i < nv; i++) {
      x[i] *= qLDiagInv[i];  // x(i) /= L(i,i)
    }

    // x <- inv(L) * x; skip simple
    for (int i=0; i < nv; i++) {
      if (!m->dof_simplenum[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        // write directly in x[i] since i cannot be a parent of itself
        while (j >= 0) {
          x[i] -= qLD[Madr_ij++]*x[j];             // x(i) -= L(i,j) * x(j)

          // advance to parent
          j = dof_parentid[j];
        }
      }
    }
  }

  // multiple vectors
  else {
    int offset;
    mjtNum tmp;

    // x <- inv(L') * x; skip simple
    for (int i=nv-1; i >= 0; i--) {
      if (!m->dof_simplenum[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        while (j >= 0) {
          // process all vectors, exploit sparsity
          for (offset=0; offset < n*nv; offset+=nv)
            if ((tmp = x[i+offset])) {
              x[j+offset] -= qLD[Madr_ij]*tmp;  // x(j) -= L(i,j) * x(i)
            }

          // advance to parent
          Madr_ij++;
          j = dof_parentid[j];
        }
      }
    }

    // x <- inv(D) * x
    for (int i=0; i < nv; i++) {
      for (offset=0; offset < n*nv; offset+=nv) {
        x[i+offset] *= qLDiagInv[i];  // x(i) /= L(i,i)
      }
    }

    // x <- inv(L) * x; skip simple
    for (int i=0; i < nv; i++) {
      if (!m->dof_simplenum[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        tmp = x[i+offset];
        while (j >= 0) {
          // process all vectors
          for (offset=0; offset < n*nv; offset+=nv) {
            x[i+offset] -= qLD[Madr_ij]*x[j+offset];  // x(i) -= L(i,j) * x(j)
          }

          // advance to parent
          Madr_ij++;
          j = dof_parentid[j];
        }
      }
    }
  }
}



// in-place sparse backsubstitution:  x = inv(L'*D*L)*x
void mj_solveLD(mjtNum* restrict x, const mjtNum* qLD, const mjtNum* qLDiagInv, int nv, int n,
                const int* rownnz, const int* rowadr, const int* colind) {
  // x <- L^-T x
  for (int i=nv-1; i > 0; i--) {
    // skip diagonal rows
    if (rownnz[i] == 1) {
      continue;
    }

    // one vector
    if (n == 1) {
      mjtNum x_i;
      if ((x_i = x[i])) {
        int start = rowadr[i];
        int end = start + rownnz[i] - 1;
        for (int adr=start; adr < end; adr++) {
          x[colind[adr]] -= qLD[adr] * x_i;
        }
      }
    }

    // multiple vectors
    else {
      int start = rowadr[i];
      int end = start + rownnz[i] - 1;
      for (int offset=0; offset < n*nv; offset+=nv) {
        mjtNum x_i;
        if ((x_i = x[i+offset])) {
          for (int adr=start; adr < end; adr++) {
            x[offset + colind[adr]] -= qLD[adr] * x_i;
          }
        }
      }
    }
  }

  // x <- D^-1 x
  for (int i=0; i < nv; i++) {
    mjtNum invD_i = qLDiagInv[i];

    // one vector
    if (n == 1) {
      x[i] *= invD_i;
    }

    // multiple vectors
    else {
      for (int offset=0; offset < n*nv; offset+=nv) {
        x[i+offset] *= invD_i;
      }
    }
  }

  // x <- L^-1 x
  for (int i=1; i < nv; i++) {
    // skip diagonal rows
    if (rownnz[i] == 1) {
      continue;
    }

    int d;
    if ((d = rownnz[i] - 1) > 0) {
      int adr = rowadr[i];

      // one vector
      if (n == 1) {
        x[i] -= mju_dotSparse(qLD+adr, x, d, colind+adr);
      }

      // multiple vectors
      else {
        for (int offset=0; offset < n*nv; offset+=nv) {
          x[i+offset] -= mju_dotSparse(qLD+adr, x+offset, d, colind+adr);
        }
      }
    }
  }
}



// sparse backsubstitution:  x = inv(L'*D*L)*y
//  use factorization in d
void mj_solveM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n) {
  if (x != y) {
    mju_copy(x, y, n*m->nv);
  }
  mj_solveLD(x, d->qLD, d->qLDiagInv, m->nv, n,
             d->M_rownnz, d->M_rowadr, d->M_colind);
}



// half of sparse backsubstitution:  x = sqrt(inv(D))*inv(L')*y
void mj_solveM2(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y,
                const mjtNum* sqrtInvD, int n) {
  int nv = m->nv;

  // local copies of key variables
  const int* rownnz = d->M_rownnz;
  const int* rowadr = d->M_rowadr;
  const int* colind = d->M_colind;
  const int* diagnum = m->dof_simplenum;
  const mjtNum* qLD = d->qLD;

  // x = y
  mju_copy(x, y, n * nv);

  // x <- L^-T x
  for (int i=nv-1; i > 0; i--) {
    // skip diagonal rows
    if (diagnum[i]) {
      continue;
    }

    // prepare row i column address range
    int start = rowadr[i];
    int end = start + rownnz[i] - 1;

    // process all vectors
    for (int offset=0; offset < n*nv; offset+=nv) {
      mjtNum x_i;
      if ((x_i = x[i+offset])) {
        for (int adr=start; adr < end; adr++) {
          x[offset + colind[adr]] -= qLD[adr] * x_i;
        }
      }
    }
  }

  // x <- D^-1/2 x
  for (int i=0; i < nv; i++) {
    mjtNum invD_i = sqrtInvD[i];
    for (int offset=0; offset < n*nv; offset+=nv) {
      x[i+offset] *= invD_i;
    }
  }
}



//---------------------------------- velocity ------------------------------------------------------

// compute cvel, cdof_dot
void mj_comVel(const mjModel* m, mjData* d) {
  int nbody = m->nbody;

  // set world vel to 0
  mju_zero(d->cvel, 6);

  // forward pass over bodies
  for (int i=1; i < nbody; i++) {
    // get body's first dof address
    int bda = m->body_dofadr[i];

    // cvel = cvel_parent
    mjtNum cvel[6];
    mju_copy(cvel, d->cvel+6*m->body_parentid[i], 6);

    // cvel = cvel_parent + cdof * qvel,  cdofdot = cvel x cdof
    int dofnum = m->body_dofnum[i];
    mjtNum cdofdot[36];
    for (int j=0; j < dofnum; j++) {
      mjtNum tmp[6];

      // compute cvel and cdofdot
      switch ((mjtJoint) m->jnt_type[m->dof_jntid[bda+j]]) {
      case mjJNT_FREE:
        // cdofdot = 0
        mju_zero(cdofdot, 18);

        // update velocity
        mju_mulDofVec(tmp, d->cdof+6*bda, d->qvel+bda, 3);
        mju_addTo(cvel, tmp, 6);

        // continue with rotations
        j += 3;
        mjFALLTHROUGH;

      case mjJNT_BALL:
        // compute all 3 cdofdots using parent velocity
        for (int k=0; k < 3; k++) {
          mju_crossMotion(cdofdot+6*(j+k), cvel, d->cdof+6*(bda+j+k));
        }

        // update velocity
        mju_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 3);
        mju_addTo(cvel, tmp, 6);

        // adjust for 3-dof joint
        j += 2;
        break;

      default:
        // in principle we should use the new velocity to compute cdofdot,
        // but it makes no difference because crossMotion(cdof, cdof) = 0,
        // and using the old velocity may be more accurate numerically
        mju_crossMotion(cdofdot+6*j, cvel, d->cdof+6*(bda+j));

        // update velocity
        mju_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 1);
        mju_addTo(cvel, tmp, 6);
      }
    }

    // assign cvel, cdofdot
    mju_copy(d->cvel+6*i, cvel, 6);
    mju_copy(d->cdof_dot+6*bda, cdofdot, 6*dofnum);
  }
}



// subtree linear velocity and angular momentum
void mj_subtreeVel(const mjModel* m, mjData* d) {
  int nbody = m->nbody;
  mjtNum dx[3], dv[3], dp[3], dL[3];
  mj_markStack(d);
  mjtNum* body_vel = mjSTACKALLOC(d, 6*m->nbody, mjtNum);

  // bodywise quantities
  for (int i=0; i < nbody; i++) {
    // compute and save body velocity
    mj_objectVelocity(m, d, mjOBJ_BODY, i, body_vel+6*i, 0);

    // body linear momentum
    mju_scl3(d->subtree_linvel+3*i, body_vel+6*i+3, m->body_mass[i]);

    // body angular momentum
    mju_mulMatTVec3(dv, d->ximat+9*i, body_vel+6*i);
    dv[0] *= m->body_inertia[3*i];
    dv[1] *= m->body_inertia[3*i+1];
    dv[2] *= m->body_inertia[3*i+2];
    mju_mulMatVec3(d->subtree_angmom+3*i, d->ximat+9*i, dv);
  }

  // subtree linvel
  for (int i=nbody-1; i >= 0; i--) {
    // non-world: add linear momentum to parent
    if (i) {
      mju_addTo3(d->subtree_linvel+3*m->body_parentid[i], d->subtree_linvel+3*i);
    }

    // convert linear momentum to linear velocity
    mju_scl3(d->subtree_linvel+3*i, d->subtree_linvel+3*i,
             1/mjMAX(mjMINVAL, m->body_subtreemass[i]));
  }

  // subtree angmom
  for (int i=nbody-1; i > 0; i--) {
    int parent = m->body_parentid[i];

    // momentum wrt body i
    mju_sub3(dx, d->xipos+3*i, d->subtree_com+3*i);
    mju_sub3(dv, body_vel+6*i+3, d->subtree_linvel+3*i);
    mju_scl3(dp, dv, m->body_mass[i]);
    mju_cross(dL, dx, dp);

    // add to subtree i
    mju_addTo3(d->subtree_angmom+3*i, dL);

    // add to parent
    mju_addTo3(d->subtree_angmom+3*parent, d->subtree_angmom+3*i);

    // momentum wrt parent
    mju_sub3(dx, d->subtree_com+3*i, d->subtree_com+3*parent);
    mju_sub3(dv, d->subtree_linvel+3*i, d->subtree_linvel+3*parent);
    mju_scl3(dv, dv, m->body_subtreemass[i]);
    mju_cross(dL, dx, dv);

    // add to parent
    mju_addTo3(d->subtree_angmom+3*parent, dL);
  }

  mj_freeStack(d);
}


//---------------------------------- RNE -----------------------------------------------------------

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result) {
  int nbody = m->nbody, nv = m->nv;
  mjtNum tmp[6], tmp1[6];
  mj_markStack(d);
  mjtNum* loc_cacc = mjSTACKALLOC(d, m->nbody*6, mjtNum);
  mjtNum* loc_cfrc_body = mjSTACKALLOC(d, m->nbody*6, mjtNum);

  // set world acceleration to -gravity
  mju_zero(loc_cacc, 6);
  if (!mjDISABLED(mjDSBL_GRAVITY)) {
    mju_scl3(loc_cacc+3, m->opt.gravity, -1);
  }

  // forward pass over bodies: accumulate cacc, set cfrc_body
  for (int i=1; i < nbody; i++) {
    // get body's first dof address
    int bda = m->body_dofadr[i];

    // cacc = cacc_parent + cdofdot * qvel
    mju_mulDofVec(tmp, d->cdof_dot+6*bda, d->qvel+bda, m->body_dofnum[i]);
    mju_add(loc_cacc+6*i, loc_cacc+6*m->body_parentid[i], tmp, 6);

    // cacc += cdof * qacc
    if (flg_acc) {
      mju_mulDofVec(tmp, d->cdof+6*bda, d->qacc+bda, m->body_dofnum[i]);
      mju_addTo(loc_cacc+6*i, tmp, 6);
    }

    // cfrc_body = cinert * cacc + cvel x (cinert * cvel)
    mju_mulInertVec(loc_cfrc_body+6*i, d->cinert+10*i, loc_cacc+6*i);
    mju_mulInertVec(tmp, d->cinert+10*i, d->cvel+6*i);
    mju_crossForce(tmp1, d->cvel+6*i, tmp);
    mju_addTo(loc_cfrc_body+6*i, tmp1, 6);
  }

  // clear world cfrc_body, for style
  mju_zero(loc_cfrc_body, 6);

  // backward pass over bodies: accumulate cfrc_body from children
  for (int i=nbody-1; i > 0; i--)
    if (m->body_parentid[i]) {
      mju_addTo(loc_cfrc_body+6*m->body_parentid[i], loc_cfrc_body+6*i, 6);
    }

  // result = cdof * cfrc_body
  for (int i=0; i < nv; i++) {
    result[i] = mju_dot(d->cdof+6*i, loc_cfrc_body+6*m->dof_bodyid[i], 6);
  }

  mj_freeStack(d);
}



// RNE with complete data: compute cacc, cfrc_ext, cfrc_int
void mj_rnePostConstraint(const mjModel* m, mjData* d) {
  int nbody = m->nbody;
  mjtNum cfrc_com[6], cfrc[6], lfrc[6];
  mjContact* con;

  // clear cacc, set world acceleration to -gravity
  mju_zero(d->cacc, 6);
  if (!mjDISABLED(mjDSBL_GRAVITY)) {
    mju_scl3(d->cacc+3, m->opt.gravity, -1);
  }

  // cfrc_ext = perturb
  mju_zero(d->cfrc_ext, 6*nbody);
  for (int i=1; i < nbody; i++)
    if (!mju_isZero(d->xfrc_applied+6*i, 6)) {
      // rearrange as torque:force
      mju_copy3(cfrc, d->xfrc_applied+6*i+3);
      mju_copy3(cfrc+3, d->xfrc_applied+6*i);

      // map force from application point to com; both world-oriented
      mju_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[i], d->xipos+3*i, 0);

      // accumulate
      mju_addTo(d->cfrc_ext+6*i, cfrc_com, 6);
    }

  // cfrc_ext += contacts
  int ncon = d->ncon;
  for (int i=0; i < ncon; i++)
    if (d->contact[i].efc_address >= 0) {
      // get contact pointer
      con = d->contact+i;

      // skip contact involving flex
      if (con->geom[0] < 0 || con->geom[1] < 0) {
        continue;
      }

      // tmp = contact-local force:torque vector
      mj_contactForce(m, d, i, lfrc);

      // cfrc = world-oriented torque:force vector (swap in the process)
      mju_mulMatTVec3(cfrc, con->frame, lfrc+3);
      mju_mulMatTVec3(cfrc+3, con->frame, lfrc);

      // body 1
      int k;
      if ((k = m->geom_bodyid[con->geom[0]])) {
        // tmp = subtree CoM-based torque_force vector
        mju_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], con->pos, 0);

        // apply (opposite for body 1)
        mju_subFrom(d->cfrc_ext+6*k, cfrc_com, 6);
      }

      // body 2
      if ((k = m->geom_bodyid[con->geom[1]])) {
        // tmp = subtree CoM-based torque_force vector
        mju_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], con->pos, 0);

        // apply
        mju_addTo(d->cfrc_ext+6*k, cfrc_com, 6);
      }
    }

  // cfrc_ext += connect, weld, flex constraints
  int i = 0, ne = d->ne;
  while (i < ne) {
    if (d->efc_type[i] != mjCNSTR_EQUALITY)
      mjERROR("row %d of efc is not an equality constraint", i);  // SHOULD NOT OCCUR

    int id = d->efc_id[i];
    mjtNum* eq_data = m->eq_data + mjNEQDATA*id;
    mjtNum pos[3], *offset;
    int k, obj1, obj2, body_semantic;
    switch ((mjtEq) m->eq_type[id]) {
    case mjEQ_CONNECT:
    case mjEQ_WELD:
      // cfrc = world-oriented torque:force vector
      mju_copy3(cfrc + 3, d->efc_force + i);
      if (m->eq_type[id] == mjEQ_WELD) {
        mju_copy3(cfrc, d->efc_force + i + 3);
      } else {
        mju_zero3(cfrc);  // no torque from connect
      }

      body_semantic = m->eq_objtype[id] == mjOBJ_BODY;

      // body 1
      obj1 = m->eq_obj1id[id];
      k = body_semantic ? obj1 : m->site_bodyid[obj1];
      if (k) {
        offset = body_semantic ? eq_data + 3 * (m->eq_type[id] == mjEQ_WELD) :
                                 m->site_pos + 3 * obj1;

        // transform point on body1: local -> global
        mj_local2Global(d, pos, 0, offset, 0, k, 0);

        // tmp = subtree CoM-based torque_force vector
        mju_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], pos, 0);

        // apply (opposite for body 1)
        mju_addTo(d->cfrc_ext+6*k, cfrc_com, 6);
      }

      // body 2
      obj2 = m->eq_obj2id[id];
      k = body_semantic ? obj2 : m->site_bodyid[obj2];
      if (k) {
        offset = body_semantic ? eq_data + 3 * (m->eq_type[id] == mjEQ_CONNECT) :
                                 m->site_pos + 3 * obj2;

        // transform point on body2: local -> global
        mj_local2Global(d, pos, 0, offset, 0, k, 0);

        // tmp = subtree CoM-based torque_force vector
        mju_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], pos, 0);

        // apply
        mju_subFrom(d->cfrc_ext+6*k, cfrc_com, 6);
      }

      // increment rows
      i += m->eq_type[id] == mjEQ_WELD ? 6 : 3;
      break;

    case mjEQ_JOINT:
    case mjEQ_TENDON:
      // increment 1 row
      i++;
      break;

    case mjEQ_FLEX:
      // increment with number of non-rigid edges
      k = m->eq_obj1id[id];
      int flex_edgeadr = m->flex_edgeadr[k];
      int flex_edgenum = m->flex_edgenum[k];

      for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
        if (!m->flexedge_rigid[e]) {
          i++;
        }
      }
      break;

    default:
      mjERROR("unknown constraint type type %d", m->eq_type[id]);    // SHOULD NOT OCCUR
    }
  }

  // forward pass over bodies: compute cacc, cfrc_int
  mjtNum cacc[6], cfrc_body[6], cfrc_corr[6];
  mju_zero(d->cfrc_int, 6);
  for (int j=1; j < nbody; j++) {
    // get body's first dof address
    int bda = m->body_dofadr[j];

    // cacc = cacc_parent + cdofdot * qvel + cdof * qacc
    mju_mulDofVec(cacc, d->cdof_dot+6*bda, d->qvel+bda, m->body_dofnum[j]);
    mju_add(d->cacc+6*j, d->cacc+6*m->body_parentid[j], cacc, 6);
    mju_mulDofVec(cacc, d->cdof+6*bda, d->qacc+bda, m->body_dofnum[j]);
    mju_addTo(d->cacc+6*j, cacc, 6);

    // cfrc_body = cinert * cacc + cvel x (cinert * cvel)
    mju_mulInertVec(cfrc_body, d->cinert+10*j, d->cacc+6*j);
    mju_mulInertVec(cfrc_corr, d->cinert+10*j, d->cvel+6*j);
    mju_crossForce(cfrc, d->cvel+6*j, cfrc_corr);
    mju_addTo(cfrc_body, cfrc, 6);

    // set cfrc_int = cfrc_body - cfrc_ext
    mju_sub(d->cfrc_int+6*j, cfrc_body, d->cfrc_ext+6*j, 6);
  }

  // backward pass over bodies: accumulate cfrc_int from children
  for (int j=nbody-1; j > 0; j--) {
    mju_addTo(d->cfrc_int+6*m->body_parentid[j], d->cfrc_int+6*j, 6);
  }
}



// add bias force due to tendon armature
void mj_tendonBias(const mjModel* m, mjData* d, mjtNum* qfrc) {
  int ntendon = m->ntendon, nv = m->nv, issparse = mj_isSparse(m);
  mjtNum* ten_Jdot = NULL;
  mj_markStack(d);

  // add bias term due to tendon armature
  for (int i=0; i < ntendon; i++) {
    mjtNum armature = m->tendon_armature[i];

    // no armature: skip
    if (!armature) {
      continue;
    }

    // allocate if required
    if (!ten_Jdot) {
      ten_Jdot = mjSTACKALLOC(d, nv, mjtNum);
    }

    // get dense d/dt(tendon Jacobian) for tendon i
    mj_tendonDot(m, d, i, ten_Jdot);

    // add bias term:  qfrc += ten_J * armature * dot(ten_Jdot, qvel)
    mjtNum coef = armature * mju_dot(ten_Jdot, d->qvel, nv);

    if (coef) {
      // dense
      if (!issparse) {
        mju_addToScl(qfrc, d->ten_J + nv*i, coef, nv);
      }

      // sparse
      else {
        int nnz = d->ten_J_rownnz[i];
        int adr = d->ten_J_rowadr[i];
        const int* colind = d->ten_J_colind + adr;
        const mjtNum* ten_J = d->ten_J + adr;
        for (int j=0; j < nnz; j++) {
          qfrc[colind[j]] += coef * ten_J[j];
        }
      }
    }
  }

  mj_freeStack(d);
}
