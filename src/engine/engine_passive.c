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

#include "engine/engine_passive.h"

#include <stddef.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_callback.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"
#include "engine/engine_plugin.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"


//----------------------------- passive forces -----------------------------------------------------

// local edge-based vertex indexing for 2D and 3D elements, 2D and 3D elements
// have 3 and 6 edges, respectively so the missing indexes are set to 0
static const int edges[2][6][2] = {{{1, 2}, {2, 0}, {0, 1}, {0, 0}, {0, 0}, {0, 0}},
                                   {{0, 1}, {1, 2}, {2, 0}, {2, 3}, {0, 3}, {1, 3}}};

// compute gradient of squared lengths of edges belonging to a given element
static void inline GradSquaredLengths(mjtNum gradient[6][2][3],
                                      const mjtNum* xpos,
                                      const int vert[4],
                                      const int edge[6][2],
                                      int nedge) {
  for (int e = 0; e < nedge; e++) {
    for (int d = 0; d < 3; d++) {
      gradient[e][0][d] = xpos[3*vert[edge[e][0]]+d] - xpos[3*vert[edge[e][1]]+d];
      gradient[e][1][d] = xpos[3*vert[edge[e][1]]+d] - xpos[3*vert[edge[e][0]]+d];
    }
  }
}

// spring and damper forces
static void mj_springdamper(const mjModel* m, mjData* d) {
  int nv = m->nv, njnt = m->njnt, ntendon = m->ntendon;
  int issparse = mj_isSparse(m);

  // joint-level springs
  for (int i=0; i < njnt; i++) {
    mjtNum stiffness = m->jnt_stiffness[i];

    // disabled : nothing to do
    if (stiffness == 0) {
      continue;
    }

    int padr = m->jnt_qposadr[i];
    int dadr = m->jnt_dofadr[i];

    switch ((mjtJoint) m->jnt_type[i]) {
    case mjJNT_FREE:
      // apply force
      d->qfrc_spring[dadr+0] = -stiffness*(d->qpos[padr+0] - m->qpos_spring[padr+0]);
      d->qfrc_spring[dadr+1] = -stiffness*(d->qpos[padr+1] - m->qpos_spring[padr+1]);
      d->qfrc_spring[dadr+2] = -stiffness*(d->qpos[padr+2] - m->qpos_spring[padr+2]);

      // continue with rotations
      dadr += 3;
      padr += 3;
      mjFALLTHROUGH;

    case mjJNT_BALL:
      {
        // convert quaternion difference into angular "velocity"
        mjtNum dif[3], quat[4];
        mju_copy4(quat, d->qpos+padr);
        mju_normalize4(quat);
        mju_subQuat(dif, quat, m->qpos_spring + padr);

        // apply torque
        d->qfrc_spring[dadr+0] = -stiffness*dif[0];
        d->qfrc_spring[dadr+1] = -stiffness*dif[1];
        d->qfrc_spring[dadr+2] = -stiffness*dif[2];
      }
      break;

    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      // apply force or torque
      d->qfrc_spring[dadr] = -stiffness*(d->qpos[padr] - m->qpos_spring[padr]);
      break;
    }
  }

  // dof-level dampers
  for (int i=0; i < m->nv; i++) {
    mjtNum damping = m->dof_damping[i];
    if (damping != 0) {
      d->qfrc_damper[i] = -damping*d->qvel[i];
    }
  }

  // flex elasticity
  for (int f=0; f < m->nflex; f++) {
    mjtNum* k = m->flex_stiffness + 21*m->flex_elemadr[f];
    mjtNum* b = m->flex_bending + 16*m->flex_edgeadr[f];
    int dim = m->flex_dim[f];

    if (dim == 1 || m->flex_rigid[f]) {
      continue;
    }

    // add bending forces to qfrc_spring
    if (dim == 2) {
      mjtNum* xpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
      int* bodyid = m->flex_vertbodyid + m->flex_vertadr[f];

      for (int e = 0; e < m->flex_edgenum[f]; e++) {
        const int* edge = m->flex_edge + 2*(e+m->flex_edgeadr[f]);
        const int* flap = m->flex_edgeflap + 2*(e+m->flex_edgeadr[f]);
        int v[4] = {edge[0], edge[1], flap[0], flap[1]};
        if (v[3] == -1) {
          // skip boundary edges
          continue;
        }
        mjtNum force[12] = {0};
        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 4; j++) {
            for (int x = 0; x < 3; x++) {
              force[3*i+x] += b[16*e+4*i+j] * xpos[3*v[j]+x];
            }
          }
        }

        // TODO: add damping

        // insert into global force
        for (int i = 0; i < 4; i++) {
          int bid = bodyid[v[i]];
          int body_dofnum = m->body_dofnum[bid];
          int body_dofadr = m->body_dofadr[bid];
          for (int x = 0; x < body_dofnum; x++) {
            d->qfrc_spring[body_dofadr+x] -= force[3*i+x];
          }
        }
      }
    }

    if (k[0] == 0) {
      continue;
    }

    if (m->flex_interp[f]) {
      mjtNum xpos[mjMAXFLEXNODES], displ[mjMAXFLEXNODES], vel[mjMAXFLEXNODES];
      mjtNum frc[mjMAXFLEXNODES], dmp[mjMAXFLEXNODES];
      mjtNum com[3] = {0};
      mjtNum* xpos0 = m->flex_node0 + 3*m->flex_nodeadr[f];
      int* bodyid = m->flex_nodebodyid + m->flex_nodeadr[f];
      int nstart = m->flex_nodeadr[f];

      // compute positions
      if (m->flex_centered[f]) {
        for (int i=0; i < m->flex_nodenum[f]; i++) {
          mju_copy3(xpos + 3*i, d->xpos + 3*bodyid[i]);
          mju_copy3(vel + 3*i, d->qvel + m->body_dofadr[bodyid[i]]);
        }
      } else {
        mjtNum screw[6];
        for (int i=0; i < m->flex_nodenum[f]; i++) {
          mju_mulMatVec3(xpos + 3*i, d->xmat + 9*bodyid[i], m->flex_node + 3*(i+nstart));
          mju_addTo3(xpos + 3*i, d->xpos + 3*bodyid[i]);
          mj_objectVelocity(m, d, mjOBJ_BODY, bodyid[i], screw, 0);
          mju_copy3(vel + 3*i, screw + 3);
        }
      }

      // compute center of mass
      for (int i = 0; i < m->flex_nodenum[f]; i++) {
        mju_addToScl3(com, xpos+3*i, 1.0/m->flex_nodenum[f]);
      }

      // re-center positions using center of mass
      for (int i = 0; i < m->flex_nodenum[f]; i++) {
        mju_addToScl3(xpos+3*i, com, -1);
      }

      // compute the Jacobian at the center of mass
      mjtNum mat[9] = {0};
      mjtNum p[3] = {.5, .5, .5};
      mju_defGradient(mat, p, xpos, 1);

      // find rotation
      mjtNum quat[4] = {1, 0, 0, 0};
      mju_mat2Rot(quat, mat);
      mju_negQuat(quat, quat);

      // rotate vertices to quat and add reference center of mass
      for (int i = 0; i < m->flex_nodenum[f]; i++) {
        mju_rotVecQuat(xpos+3*i, xpos+3*i, quat);
        mju_addTo3(xpos+3*i, p);
        mju_rotVecQuat(vel+3*i, vel+3*i, quat);
      }

      // compute displacement
      for (int i = 0; i < m->flex_nodenum[f]; i++) {
        mju_addScl3(displ+3*i, xpos+3*i, xpos0+3*i, -1);
      }

      // compute force in the stretch frame
      mju_mulMatVec(frc, k, displ, 3*m->flex_nodenum[f], 3*m->flex_nodenum[f]);

      // compute damping force in stretch frame
      mju_mulMatVec(dmp, k, vel, 3*m->flex_nodenum[f], 3*m->flex_nodenum[f]);

      // rotate forces to global frame and add to qfrc
      mju_negQuat(quat, quat);
      for (int i = 0; i < m->flex_nodenum[f]; i++) {
        mjtNum qfrc[3], qdmp[3];
        mju_rotVecQuat(qfrc, frc+3*i, quat);
        mju_rotVecQuat(qdmp, dmp+3*i, quat);
        mju_scl3(qdmp, qdmp, m->flex_damping[f]);
        if (m->flex_centered[f]) {
          mju_addTo3(d->qfrc_spring+m->body_dofadr[bodyid[i]], qfrc);
          mju_addTo3(d->qfrc_damper+m->body_dofadr[bodyid[i]], qdmp);
        } else {
          mj_applyFT(m, d, qfrc, 0, xpos+3*i, bodyid[i], d->qfrc_spring);
          mj_applyFT(m, d, qdmp, 0, xpos+3*i, bodyid[i], d->qfrc_damper);
        }
      }

      // do not continue with the rest of the flex passive forces
      continue;
    }

    int nedge = (dim == 2) ? 3 : 6;
    int nvert = (dim == 2) ? 3 : 4;
    const int* elem = m->flex_elem + m->flex_elemdataadr[f];
    const int* edgeelem = m->flex_elemedge + m->flex_elemedgeadr[f];
    mjtNum* xpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
    mjtNum* vel = d->flexedge_velocity + m->flex_edgeadr[f];
    mjtNum* deformed = d->flexedge_length + m->flex_edgeadr[f];
    mjtNum* reference = m->flexedge_length0 + m->flex_edgeadr[f];
    int* bodyid = m->flex_vertbodyid + m->flex_vertadr[f];
    mjtNum kD = m->flex_damping[f] / m->opt.timestep;

    mj_markStack(d);
    mjtNum* qfrc = mjSTACKALLOC(d, 3*m->flex_vertnum[f], mjtNum);
    mju_zero(qfrc, 3*m->flex_vertnum[f]);

    // compute force element-by-element
    for (int t = 0; t < m->flex_elemnum[f]; t++)  {
      const int* vert = elem + (dim+1) * t;

      // compute length gradient with respect to dofs
      mjtNum gradient[6][2][3];
      GradSquaredLengths(gradient, xpos, vert, edges[dim-2], nedge);

      // we add generalized Rayleigh damping as decribed in Section 5.2 of
      // Kharevych et al., "Geometric, Variational Integrators for Computer
      // Animation" http://multires.caltech.edu/pubs/DiscreteLagrangian.pdf

      // extract elongation of edges belonging to this element
      mjtNum elongation[6];
      for (int e = 0; e < nedge; e++) {
        int idx = edgeelem[t * nedge + e];
        mjtNum previous = deformed[idx] - vel[idx] * m->opt.timestep;
        elongation[e] = deformed[idx]*deformed[idx] - reference[idx]*reference[idx] +
                       (deformed[idx]*deformed[idx] - previous*previous) * kD;
      }

      // unpack triangular representation
      mjtNum metric[36];
      int id = 0;
      for (int ed1 = 0; ed1 < nedge; ed1++) {
        for (int ed2 = ed1; ed2 < nedge; ed2++) {
          metric[nedge*ed1 + ed2] = k[21*t + id];
          metric[nedge*ed2 + ed1] = k[21*t + id++];
        }
      }

      // we now multiply the elongations by the precomputed metric tensor,
      // notice that if metric=diag(1/reference) then this would yield a
      // mass-spring model

      // compute local force
      mjtNum force[12] = {0};
      for (int ed1 = 0; ed1 < nedge; ed1++) {
        for (int ed2 = 0; ed2 < nedge; ed2++) {
          for (int i = 0; i < 2; i++) {
            for (int x = 0; x < 3; x++) {
              force[3 * edges[dim-2][ed2][i] + x] -=
                  elongation[ed1] * gradient[ed2][i][x] *
                  metric[nedge * ed1 + ed2];
            }
          }
        }
      }

      // insert into global force
      for (int i = 0; i < nvert; i++) {
        for (int x = 0; x < 3; x++) {
          qfrc[3*vert[i]+x] += force[3*i+x];
        }
      }
    }

    // insert force into qfrc_passive, straightforward for simple bodies,
    // need to distribute the force in case of pinned vertices
    for (int v = 0; v < m->flex_vertnum[f]; v++) {
      int bid = bodyid[v];
      if (m->body_simple[bid] != 2) {
        // this should only occur for pinned flex vertices
        mj_applyFT(m, d, qfrc + 3*v, 0, xpos + 3*v, bid, d->qfrc_spring);
      } else {
        int body_dofnum = m->body_dofnum[bid];
        int body_dofadr = m->body_dofadr[bid];
        for (int x = 0; x < body_dofnum; x++) {
          d->qfrc_spring[body_dofadr+x] += qfrc[3*v+x];
        }
      }
    }

    mj_freeStack(d);
  }

  // flexedge-level spring-dampers
  for (int f=0; f < m->nflex; f++) {
    mjtNum stiffness = m->flex_edgestiffness[f];
    mjtNum damping = m->flex_edgedamping[f];

    // disabled or rigid: nothing to do
    if (m->flex_rigid[f] || (stiffness == 0 && damping == 0)) {
      continue;
    }

    // process non-rigid edges of this flex (global edge index)
    int edgeend = m->flex_edgeadr[f] + m->flex_edgenum[f];
    for (int e=m->flex_edgeadr[f]; e < edgeend; e++) {
      // skip rigid
      if (m->flexedge_rigid[e]) {
        continue;
      }

      // compute spring-damper force along edge
      mjtNum frc_spring = stiffness * (m->flexedge_length0[e] - d->flexedge_length[e]);
      mjtNum frc_damper = -damping * d->flexedge_velocity[e];

      // transform to joint torque, add to qfrc_{spring, damper}: dense or sparse
      if (issparse) {
        int end = d->flexedge_J_rowadr[e] + d->flexedge_J_rownnz[e];
        for (int j=d->flexedge_J_rowadr[e]; j < end; j++) {
          int colind = d->flexedge_J_colind[j];
          mjtNum J = d->flexedge_J[j];
          d->qfrc_spring[colind] += J * frc_spring;
          d->qfrc_damper[colind] += J * frc_damper;
        }
      } else {
        if (frc_spring) mju_addToScl(d->qfrc_spring, d->flexedge_J+e*nv, frc_spring, nv);
        if (frc_damper) mju_addToScl(d->qfrc_damper, d->flexedge_J+e*nv, frc_damper, nv);
      }
    }
  }

  // tendon-level spring-dampers
  for (int i=0; i < ntendon; i++) {
    mjtNum stiffness = m->tendon_stiffness[i];
    mjtNum damping = m->tendon_damping[i];

    // disabled : nothing to do
    if (stiffness == 0 && damping == 0) {
      continue;
    }

    // compute spring force along tendon
    mjtNum length = d->ten_length[i];
    mjtNum lower = m->tendon_lengthspring[2*i];
    mjtNum upper = m->tendon_lengthspring[2*i+1];
    mjtNum frc_spring = 0;
    if (length > upper) {
      frc_spring = stiffness * (upper - length);
    } else if (length < lower) {
      frc_spring = stiffness * (lower - length);
    }

    // compute damper linear force along tendon
    mjtNum frc_damper = -damping * d->ten_velocity[i];

    // transform to joint torque, add to qfrc_{spring, damper}: dense or sparse
    if (issparse) {
      if (frc_spring || frc_damper) {
        int end = d->ten_J_rowadr[i] + d->ten_J_rownnz[i];
        for (int j=d->ten_J_rowadr[i]; j < end; j++) {
          int k = d->ten_J_colind[j];
          mjtNum J = d->ten_J[j];
          d->qfrc_spring[k] += J * frc_spring;
          d->qfrc_damper[k] += J * frc_damper;
        }
      }
    } else {
      if (frc_spring) mju_addToScl(d->qfrc_spring, d->ten_J+i*nv, frc_spring, nv);
      if (frc_damper) mju_addToScl(d->qfrc_damper, d->ten_J+i*nv, frc_damper, nv);
    }
  }
}



// body-level gravity compensation, return 1 if any, 0 otherwise
static int mj_gravcomp(const mjModel* m, mjData* d) {
  if (!m->ngravcomp || mjDISABLED(mjDSBL_GRAVITY) || mju_norm3(m->opt.gravity) == 0) {
    return 0;
  }

  int nbody = m->nbody, has_gravcomp = 0;
  mjtNum force[3], torque[3]={0};

  // apply per-body gravity compensation
  for (int i=1; i < nbody; i++) {
    if (m->body_gravcomp[i]) {
      has_gravcomp = 1;
      mju_scl3(force, m->opt.gravity, -(m->body_mass[i]*m->body_gravcomp[i]));
      mj_applyFT(m, d, force, torque, d->xipos+3*i, i, d->qfrc_gravcomp);
    }
  }

  return has_gravcomp;
}



// fluid forces
static int mj_fluid(const mjModel* m, mjData* d) {
  int nbody = m->nbody;
  int has_fluid = m->opt.viscosity > 0 || m->opt.density > 0;

  if (has_fluid) {
    for (int i=1; i < nbody; i++) {
      if (m->body_mass[i] < mjMINVAL) {
        continue;
      }

      // if any child geom uses the ellipsoid model, inertia-box model is disabled for parent body
      int use_ellipsoid_model = 0;
      int geomnum = m->body_geomnum[i];
      for (int j=0; j < geomnum && use_ellipsoid_model == 0; j++) {
        const int geomid = m->body_geomadr[i] + j;
        use_ellipsoid_model += (m->geom_fluid[mjNFLUID*geomid] > 0);
      }

      if (use_ellipsoid_model) {
        mj_ellipsoidFluidModel(m, d, i);
      } else {
        mj_inertiaBoxFluidModel(m, d, i);
      }
    }
  }

  return has_fluid;
}



// all passive forces
void mj_passive(const mjModel* m, mjData* d) {
  int nv = m->nv;

  // clear all passive force vectors
  mju_zero(d->qfrc_spring,   nv);
  mju_zero(d->qfrc_damper,   nv);
  mju_zero(d->qfrc_gravcomp, nv);
  mju_zero(d->qfrc_fluid,    nv);
  mju_zero(d->qfrc_passive,  nv);

  // disabled: return
  if (mjDISABLED(mjDSBL_PASSIVE)) {
    return;
  }

  // springs and dampers
  mj_springdamper(m, d);

  // gravity compensation
  int has_gravcomp = mj_gravcomp(m, d);

  // fluid forces
  int has_fluid = mj_fluid(m, d);

  // add passive forces into qfrc_passive
  mju_add(d->qfrc_passive, d->qfrc_spring, d->qfrc_damper, nv);
  if (has_fluid) mju_addTo(d->qfrc_passive, d->qfrc_fluid, nv);
  if (has_gravcomp) {
    int njnt = m->njnt;
    for (int i=0; i < njnt; i++) {
      // skip if gravcomp added via actuators
      if (m->jnt_actgravcomp[i]) {
        continue;
      }

      // get number of dofs for this joint
      int dofnum;
      switch (m->jnt_type[i]) {
      case mjJNT_HINGE:
      case mjJNT_SLIDE:
        dofnum = 1;
        break;

      case mjJNT_BALL:
        dofnum = 3;
        break;

      case mjJNT_FREE:
        dofnum = 6;
        break;
      }

      // add gravcomp force
      int dofadr = m->jnt_dofadr[i];
      for (int j=0; j < dofnum; j++) {
        d->qfrc_passive[dofadr+j] += d->qfrc_gravcomp[dofadr+j];
      }
    }
  }

  // user callback: add custom passive forces
  if (mjcb_passive) {
    mjcb_passive(m, d);
  }

  // plugin: add custom passive forces
  if (m->nplugin) {
    const int nslot = mjp_pluginCount();

    // iterate over plugins, call compute if type is mjPLUGIN_PASSIVE
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        mjERROR("invalid plugin slot: %d", slot);
      }
      if (plugin->capabilityflags & mjPLUGIN_PASSIVE) {
        if (!plugin->compute) {
          mjERROR("`compute` is a null function pointer for plugin at slot %d", slot);
        }
        plugin->compute(m, d, i, mjPLUGIN_PASSIVE);
      }
    }
  }
}



//---------------------------------- fluid models --------------------------------------------------

// fluid forces based on inertia-box approximation
void mj_inertiaBoxFluidModel(const mjModel* m, mjData* d, int i) {
  mjtNum lvel[6], wind[6], lwind[6], lfrc[6], bfrc[6], box[3], diam, *inertia;
  inertia = m->body_inertia + 3*i;
  box[0] = mju_sqrt(mju_max(mjMINVAL,
                            (inertia[1] + inertia[2] - inertia[0])) / m->body_mass[i] * 6.0);
  box[1] = mju_sqrt(mju_max(mjMINVAL,
                            (inertia[0] + inertia[2] - inertia[1])) / m->body_mass[i] * 6.0);
  box[2] = mju_sqrt(mju_max(mjMINVAL,
                            (inertia[0] + inertia[1] - inertia[2])) / m->body_mass[i] * 6.0);

  // map from CoM-centered to local body-centered 6D velocity
  mj_objectVelocity(m, d, mjOBJ_BODY, i, lvel, 1);

  // compute wind in local coordinates
  mju_zero(wind, 6);
  mju_copy3(wind+3, m->opt.wind);
  mju_transformSpatial(lwind, wind, 0, d->xipos+3*i,
                       d->subtree_com+3*m->body_rootid[i], d->ximat+9*i);

  // subtract translational component from body velocity
  mju_subFrom3(lvel+3, lwind+3);
  mju_zero(lfrc, 6);

  // set viscous force and torque
  if (m->opt.viscosity > 0) {
    // diameter of sphere approximation
    diam = (box[0] + box[1] + box[2])/3.0;

    // angular viscosity
    mju_scl3(lfrc, lvel, -mjPI*diam*diam*diam*m->opt.viscosity);

    // linear viscosity
    mju_scl3(lfrc+3, lvel+3, -3.0*mjPI*diam*m->opt.viscosity);
  }

  // add lift and drag force and torque
  if (m->opt.density > 0) {
    // force
    lfrc[3] -= 0.5*m->opt.density*box[1]*box[2]*mju_abs(lvel[3])*lvel[3];
    lfrc[4] -= 0.5*m->opt.density*box[0]*box[2]*mju_abs(lvel[4])*lvel[4];
    lfrc[5] -= 0.5*m->opt.density*box[0]*box[1]*mju_abs(lvel[5])*lvel[5];

    // torque
    lfrc[0] -= m->opt.density*box[0]*(box[1]*box[1]*box[1]*box[1]+box[2]*box[2]*box[2]*box[2])*
               mju_abs(lvel[0])*lvel[0]/64.0;
    lfrc[1] -= m->opt.density*box[1]*(box[0]*box[0]*box[0]*box[0]+box[2]*box[2]*box[2]*box[2])*
               mju_abs(lvel[1])*lvel[1]/64.0;
    lfrc[2] -= m->opt.density*box[2]*(box[0]*box[0]*box[0]*box[0]+box[1]*box[1]*box[1]*box[1])*
               mju_abs(lvel[2])*lvel[2]/64.0;
  }
  // rotate to global orientation: lfrc -> bfrc
  mju_mulMatVec3(bfrc, d->ximat+9*i, lfrc);
  mju_mulMatVec3(bfrc+3, d->ximat+9*i, lfrc+3);

  // apply force and torque to body com
  mj_applyFT(m, d, bfrc+3, bfrc, d->xipos+3*i, i, d->qfrc_fluid);
}



// fluid forces based on ellipsoid approximation
void mj_ellipsoidFluidModel(const mjModel* m, mjData* d, int bodyid) {
  mjtNum lvel[6], wind[6], lwind[6], lfrc[6], bfrc[6];
  mjtNum geom_interaction_coef, magnus_lift_coef, kutta_lift_coef;
  mjtNum semiaxes[3], virtual_mass[3], virtual_inertia[3];
  mjtNum blunt_drag_coef, slender_drag_coef, ang_drag_coef;

  for (int j=0; j < m->body_geomnum[bodyid]; j++) {
    const int geomid = m->body_geomadr[bodyid] + j;

    mju_geomSemiAxes(m, geomid, semiaxes);

    readFluidGeomInteraction(
      m->geom_fluid + mjNFLUID*geomid, &geom_interaction_coef,
      &blunt_drag_coef, &slender_drag_coef, &ang_drag_coef,
      &kutta_lift_coef, &magnus_lift_coef,
      virtual_mass, virtual_inertia);

    // scales all forces, read from MJCF as boolean (0.0 or 1.0)
    if (geom_interaction_coef == 0.0) {
      continue;
    }

    // map from CoM-centered to local body-centered 6D velocity
    mj_objectVelocity(m, d, mjOBJ_GEOM, geomid, lvel, 1);

    // compute wind in local coordinates
    mju_zero(wind, 6);
    mju_copy3(wind+3, m->opt.wind);
    mju_transformSpatial(lwind, wind, 0,
                         d->geom_xpos + 3*geomid,  // Frame of ref's origin.
                         d->subtree_com + 3*m->body_rootid[bodyid],
                         d->geom_xmat + 9*geomid);  // Frame of ref's orientation.

    // subtract translational component from grom velocity
    mju_subFrom3(lvel+3, lwind+3);

    // initialize viscous force and torque
    mju_zero(lfrc, 6);

    // added-mass forces and torques
    mj_addedMassForces(lvel, NULL, m->opt.density, virtual_mass, virtual_inertia, lfrc);

    // lift force orthogonal to lvel from Kutta-Joukowski theorem
    mj_viscousForces(lvel, m->opt.density, m->opt.viscosity, semiaxes, magnus_lift_coef,
                     kutta_lift_coef, blunt_drag_coef, slender_drag_coef, ang_drag_coef, lfrc);

    // scale by geom_interaction_coef (1.0 by default)
    mju_scl(lfrc, lfrc, geom_interaction_coef, 6);

    // rotate to global orientation: lfrc -> bfrc
    mju_mulMatVec3(bfrc, d->geom_xmat + 9*geomid, lfrc);
    mju_mulMatVec3(bfrc+3, d->geom_xmat + 9*geomid, lfrc+3);

    // apply force and torque to body com
    mj_applyFT(m, d, bfrc+3, bfrc,
               d->geom_xpos + 3*geomid,  // point where FT is generated
               bodyid, d->qfrc_fluid);
  }
}


// compute forces due to fluid mass moving with the body
void mj_addedMassForces(const mjtNum local_vels[6], const mjtNum local_accels[6],
                        const mjtNum fluid_density, const mjtNum virtual_mass[3],
                        const mjtNum virtual_inertia[3], mjtNum local_force[6])
{
  const mjtNum lin_vel[3] = {local_vels[3], local_vels[4], local_vels[5]};
  const mjtNum ang_vel[3] = {local_vels[0], local_vels[1], local_vels[2]};
  const mjtNum virtual_lin_mom[3] = {
    fluid_density * virtual_mass[0] * lin_vel[0],
    fluid_density * virtual_mass[1] * lin_vel[1],
    fluid_density * virtual_mass[2] * lin_vel[2]
  };
  const mjtNum virtual_ang_mom[3] = {
    fluid_density * virtual_inertia[0] * ang_vel[0],
    fluid_density * virtual_inertia[1] * ang_vel[1],
    fluid_density * virtual_inertia[2] * ang_vel[2]
  };

  // disabled due to dependency on qacc but included for completeness
  if (local_accels) {
    local_force[0] -= fluid_density * virtual_inertia[0] * local_accels[0];
    local_force[1] -= fluid_density * virtual_inertia[1] * local_accels[1];
    local_force[2] -= fluid_density * virtual_inertia[2] * local_accels[2];
    local_force[3] -= fluid_density * virtual_mass[0] * local_accels[3];
    local_force[4] -= fluid_density * virtual_mass[1] * local_accels[4];
    local_force[5] -= fluid_density * virtual_mass[2] * local_accels[5];
  }

  mjtNum added_mass_force[3], added_mass_torque1[3], added_mass_torque2[3];
  mju_cross(added_mass_force, virtual_lin_mom, ang_vel);
  mju_cross(added_mass_torque1, virtual_lin_mom, lin_vel);
  mju_cross(added_mass_torque2, virtual_ang_mom, ang_vel);

  mju_addTo3(local_force, added_mass_torque1);
  mju_addTo3(local_force, added_mass_torque2);
  mju_addTo3(local_force+3, added_mass_force);
}


// inlined helper functions
static inline mjtNum mji_pow4(const mjtNum val) {
  return (val*val)*(val*val);
}

static inline mjtNum mji_pow2(const mjtNum val) {
  return val*val;
}

static inline mjtNum mji_ellipsoid_max_moment(const mjtNum size[3], const int dir) {
  const mjtNum d0 = size[dir], d1 = size[(dir+1) % 3], d2 = size[(dir+2) % 3];
  return 8.0/15.0 * mjPI * d0 * mji_pow4(mju_max(d1, d2));
}



// lift and drag forces due to motion in the fluid
void mj_viscousForces(
  const mjtNum local_vels[6], const mjtNum fluid_density,
  const mjtNum fluid_viscosity, const mjtNum size[3],
  const mjtNum magnus_lift_coef, const mjtNum kutta_lift_coef,
  const mjtNum blunt_drag_coef, const mjtNum slender_drag_coef,
  const mjtNum ang_drag_coef, mjtNum local_force[6])
{
  const mjtNum lin_vel[3] = {local_vels[3], local_vels[4], local_vels[5]};
  const mjtNum ang_vel[3] = {local_vels[0], local_vels[1], local_vels[2]};
  const mjtNum volume = 4.0/3.0 * mjPI * size[0] * size[1] * size[2];
  const mjtNum d_max = mju_max(mju_max(size[0], size[1]), size[2]);
  const mjtNum d_min = mju_min(mju_min(size[0], size[1]), size[2]);
  const mjtNum d_mid = size[0] + size[1] + size[2] - d_max - d_min;
  const mjtNum A_max = mjPI * d_max * d_mid;

  mjtNum magnus_force[3];
  mju_cross(magnus_force, ang_vel, lin_vel);
  magnus_force[0] *= magnus_lift_coef * fluid_density * volume;
  magnus_force[1] *= magnus_lift_coef * fluid_density * volume;
  magnus_force[2] *= magnus_lift_coef * fluid_density * volume;

  // the dot product between velocity and the normal to the cross-section that
  // defines the body's projection along velocity is proj_num/sqrt(proj_denom)
  const mjtNum proj_denom = mji_pow4(size[1] * size[2]) * mji_pow2(lin_vel[0]) +
                            mji_pow4(size[2] * size[0]) * mji_pow2(lin_vel[1]) +
                            mji_pow4(size[0] * size[1]) * mji_pow2(lin_vel[2]);
  const mjtNum proj_num = mji_pow2(size[1] * size[2] * lin_vel[0]) +
                          mji_pow2(size[2] * size[0] * lin_vel[1]) +
                          mji_pow2(size[0] * size[1] * lin_vel[2]);

  // projected surface in the direction of the velocity
  const mjtNum A_proj = mjPI * mju_sqrt(proj_denom/mju_max(mjMINVAL, proj_num));

  // not-unit normal to ellipsoid's projected area in the direction of velocity
  const mjtNum norm[3] = {
    mji_pow2(size[1] * size[2]) * lin_vel[0],
    mji_pow2(size[2] * size[0]) * lin_vel[1],
    mji_pow2(size[0] * size[1]) * lin_vel[2]
  };

  // cosine between velocity and normal to the surface
  // divided by proj_denom instead of sqrt(proj_denom) to account for skipped normalization in norm
  const mjtNum cos_alpha = proj_num / mju_max(
    mjMINVAL, mju_norm3(lin_vel) * proj_denom);
  mjtNum kutta_circ[3];
  mju_cross(kutta_circ, norm, lin_vel);
  kutta_circ[0] *= kutta_lift_coef * fluid_density * cos_alpha * A_proj;
  kutta_circ[1] *= kutta_lift_coef * fluid_density * cos_alpha * A_proj;
  kutta_circ[2] *= kutta_lift_coef * fluid_density * cos_alpha * A_proj;
  mjtNum kutta_force[3];
  mju_cross(kutta_force, kutta_circ, lin_vel);

  // viscous force and torque in Stokes flow, analytical for spherical bodies
  const mjtNum eq_sphere_D = 2.0/3.0 * (size[0] + size[1] + size[2]);
  const mjtNum lin_visc_force_coef = 3.0 * mjPI * eq_sphere_D;
  const mjtNum lin_visc_torq_coef = mjPI * eq_sphere_D*eq_sphere_D*eq_sphere_D;

  // moments of inertia used to compute angular quadratic drag
  const mjtNum I_max = 8.0/15.0 * mjPI * d_mid * mji_pow4(d_max);
  const mjtNum II[3] = {
    mji_ellipsoid_max_moment(size, 0),
    mji_ellipsoid_max_moment(size, 1),
    mji_ellipsoid_max_moment(size, 2)
  };
  const mjtNum mom_visc[3] = {
    ang_vel[0] * (ang_drag_coef*II[0] + slender_drag_coef*(I_max - II[0])),
    ang_vel[1] * (ang_drag_coef*II[1] + slender_drag_coef*(I_max - II[1])),
    ang_vel[2] * (ang_drag_coef*II[2] + slender_drag_coef*(I_max - II[2]))
  };

  const mjtNum drag_lin_coef =  // linear plus quadratic
                               fluid_viscosity*lin_visc_force_coef + fluid_density*mju_norm3(lin_vel)*(
    A_proj*blunt_drag_coef + slender_drag_coef*(A_max - A_proj));
  const mjtNum drag_ang_coef =  // linear plus quadratic
                               fluid_viscosity * lin_visc_torq_coef +
                               fluid_density * mju_norm3(mom_visc);

  local_force[0] -= drag_ang_coef * ang_vel[0];
  local_force[1] -= drag_ang_coef * ang_vel[1];
  local_force[2] -= drag_ang_coef * ang_vel[2];
  local_force[3] += magnus_force[0] + kutta_force[0] - drag_lin_coef*lin_vel[0];
  local_force[4] += magnus_force[1] + kutta_force[1] - drag_lin_coef*lin_vel[1];
  local_force[5] += magnus_force[2] + kutta_force[2] - drag_lin_coef*lin_vel[2];
}



// read the geom_fluid_coefs array into its constituent parts
void readFluidGeomInteraction(const mjtNum* geom_fluid_coefs,
                              mjtNum* geom_fluid_coef,
                              mjtNum* blunt_drag_coef,
                              mjtNum* slender_drag_coef,
                              mjtNum* ang_drag_coef,
                              mjtNum* kutta_lift_coef,
                              mjtNum* magnus_lift_coef,
                              mjtNum virtual_mass[3],
                              mjtNum virtual_inertia[3]) {
  int i = 0;
  geom_fluid_coef[0]   = geom_fluid_coefs[i++];
  blunt_drag_coef[0]   = geom_fluid_coefs[i++];
  slender_drag_coef[0] = geom_fluid_coefs[i++];
  ang_drag_coef[0]     = geom_fluid_coefs[i++];
  kutta_lift_coef[0]   = geom_fluid_coefs[i++];
  magnus_lift_coef[0]  = geom_fluid_coefs[i++];
  virtual_mass[0]      = geom_fluid_coefs[i++];
  virtual_mass[1]      = geom_fluid_coefs[i++];
  virtual_mass[2]      = geom_fluid_coefs[i++];
  virtual_inertia[0]   = geom_fluid_coefs[i++];
  virtual_inertia[1]   = geom_fluid_coefs[i++];
  virtual_inertia[2]   = geom_fluid_coefs[i++];
  if (i != mjNFLUID) {
    mjERROR("wrong number of entries.");
  }
}



// write components into geom_fluid_coefs array
void writeFluidGeomInteraction (mjtNum* geom_fluid_coefs,
                                const mjtNum* geom_fluid_coef,
                                const mjtNum* blunt_drag_coef,
                                const mjtNum* slender_drag_coef,
                                const mjtNum* ang_drag_coef,
                                const mjtNum* kutta_lift_coef,
                                const mjtNum* magnus_lift_coef,
                                const mjtNum virtual_mass[3],
                                const mjtNum virtual_inertia[3]) {
  int i = 0;
  geom_fluid_coefs[i++] = geom_fluid_coef[0];
  geom_fluid_coefs[i++] = blunt_drag_coef[0];
  geom_fluid_coefs[i++] = slender_drag_coef[0];
  geom_fluid_coefs[i++] = ang_drag_coef[0];
  geom_fluid_coefs[i++] = kutta_lift_coef[0];
  geom_fluid_coefs[i++] = magnus_lift_coef[0];
  geom_fluid_coefs[i++] = virtual_mass[0];
  geom_fluid_coefs[i++] = virtual_mass[1];
  geom_fluid_coefs[i++] = virtual_mass[2];
  geom_fluid_coefs[i++] = virtual_inertia[0];
  geom_fluid_coefs[i++] = virtual_inertia[1];
  geom_fluid_coefs[i++] = virtual_inertia[2];
  if (i != mjNFLUID) {
    mjERROR("wrong number of entries.");
  }
}
