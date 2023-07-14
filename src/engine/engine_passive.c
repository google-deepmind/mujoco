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
#include "engine/engine_plugin.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"


//----------------------------- passive forces -----------------------------------------------------

// all passive forces
void mj_passive(const mjModel* m, mjData* d) {
  int issparse = mj_isSparse(m);
  int nv = m->nv;
  mjtNum dif[3], frc, stiffness, damping;

  // clear passive force
  mju_zero(d->qfrc_passive, m->nv);

  // disabled: return
  if (mjDISABLED(mjDSBL_PASSIVE)) {
    return;
  }

  // joint-level springs
  for (int i=0; i < m->njnt; i++) {
    stiffness = m->jnt_stiffness[i];

    int padr = m->jnt_qposadr[i];
    int dadr = m->jnt_dofadr[i];

    switch (m->jnt_type[i]) {
    case mjJNT_FREE:
      // apply force
      d->qfrc_passive[dadr+0] -= stiffness*(d->qpos[padr+0] - m->qpos_spring[padr+0]);
      d->qfrc_passive[dadr+1] -= stiffness*(d->qpos[padr+1] - m->qpos_spring[padr+1]);
      d->qfrc_passive[dadr+2] -= stiffness*(d->qpos[padr+2] - m->qpos_spring[padr+2]);

      // continue with rotations
      dadr += 3;
      padr += 3;
      mjFALLTHROUGH;

    case mjJNT_BALL:
      // covert quatertion difference into angular "velocity"
      mju_subQuat(dif, d->qpos + padr, m->qpos_spring + padr);

      // apply torque
      d->qfrc_passive[dadr+0] -= stiffness*dif[0];
      d->qfrc_passive[dadr+1] -= stiffness*dif[1];
      d->qfrc_passive[dadr+2] -= stiffness*dif[2];
      break;

    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      // apply force or torque
      d->qfrc_passive[dadr] -= stiffness*(d->qpos[padr] - m->qpos_spring[padr]);
      break;
    }
  }

  // dof-level dampers
  for (int i=0; i < m->nv; i++) {
    damping = m->dof_damping[i];
    d->qfrc_passive[i] -= damping*d->qvel[i];
  }

  // tendon-level spring-dampers
  for (int i=0; i < m->ntendon; i++) {
    stiffness = m->tendon_stiffness[i];
    damping = m->tendon_damping[i];

    // compute spring force along tendon
    mjtNum length = d->ten_length[i];
    mjtNum lower = m->tendon_lengthspring[2*i];
    mjtNum upper = m->tendon_lengthspring[2*i+1];
    if (length > upper) {
      frc = stiffness * (upper - length);
    } else if (length < lower) {
      frc = stiffness * (lower - length);
    } else {
      frc = 0;
    }

    // compute damper linear force along tendon
    frc -= damping * d->ten_velocity[i];

    // transform to joint torque, add to qfrc_passive: dense or sparse
    if (issparse) {
      int end = d->ten_J_rowadr[i] + d->ten_J_rownnz[i];
      for (int j=d->ten_J_rowadr[i]; j < end; j++) {
        d->qfrc_passive[d->ten_J_colind[j]] += d->ten_J[j] * frc;
      }
    } else {
      mju_addToScl(d->qfrc_passive, d->ten_J+i*nv, frc, nv);
    }
  }

  // body-level gravity compensation
  if (!mjDISABLED(mjDSBL_GRAVITY) && mju_norm3(m->opt.gravity)) {
    mjtNum force[3], torque[3]={0};

    // apply per-body gravity compensation
    for (int i=1; i < m->nbody; i++) {
      if (m->body_gravcomp[i]) {
        mju_scl3(force, m->opt.gravity, -(m->body_mass[i]*m->body_gravcomp[i]));
        mj_applyFT(m, d, force, torque, d->xipos+3*i, i, d->qfrc_passive);
      }
    }
  }

  // body-level viscosity, lift and drag
  if (m->opt.viscosity > 0 || m->opt.density > 0) {
    for (int i=1; i < m->nbody; i++) {
      if (m->body_mass[i] < mjMINVAL) {
        continue;
      }

      int use_ellipsoid_model = 0;
      // if any child geom uses the ellipsoid model, inertia-box model is disabled for parent body
      for (int j=0; j < m->body_geomnum[i] && use_ellipsoid_model == 0; j++) {
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

  // user callback: add custom passive forces
  if (mjcb_passive) {
    mjcb_passive(m, d);
  }

  // plugin
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
  mju_rotVecMat(bfrc, lfrc, d->ximat+9*i);
  mju_rotVecMat(bfrc+3, lfrc+3, d->ximat+9*i);

  // apply force and torque to body com
  mj_applyFT(m, d, bfrc+3, bfrc, d->xipos+3*i, i, d->qfrc_passive);
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
    mju_rotVecMat(bfrc, lfrc, d->geom_xmat + 9*geomid);
    mju_rotVecMat(bfrc+3, lfrc+3, d->geom_xmat + 9*geomid);

    // apply force and torque to body com
    mj_applyFT(m, d, bfrc+3, bfrc,
               d->geom_xpos + 3*geomid,  // point where FT is generated
               bodyid, d->qfrc_passive);
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
