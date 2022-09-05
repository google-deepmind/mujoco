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

#include "engine/engine_vis_visualize.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include "engine/engine_array_safety.h"
#include "engine/engine_macro.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"
#include "engine/engine_vis_init.h"

//----------------------------- utility functions and macros ---------------------------------------

static const mjtNum IDENTITY[9] = {1, 0, 0,
                                   0, 1, 0,
                                   0, 0, 1};


// copy float array
static void f2f(float* dest, const float* src, int n) {
  memcpy(dest, src, n*sizeof(float));
}



// make text label
static void makeLabel(const mjModel* m, mjtObj type, int id, char* label) {
  const char* typestr = mju_type2Str(type);
  const char* namestr = mj_id2name(m, type, id);
  char txt[100];

  // copy existing name or make numeric name
  if (namestr) {
    mjSNPRINTF(txt, "%s", namestr);
  } else if (typestr) {
    mjSNPRINTF(txt, "%s %d", typestr, id);
  } else {
    mjSNPRINTF(txt, "%d", id);
  }

  // copy result into label
  strncpy(label, txt, 99);
}



// return if there is no space in buffer
#define START                                                      \
    if ( scn->ngeom>=scn->maxgeom ) {                              \
      mj_warning(d, mjWARN_VGEOMFULL, scn->maxgeom);               \
      return;                                                      \
    } else {                                                       \
      thisgeom = scn->geoms + scn->ngeom;                          \
      mjv_initGeom(thisgeom, mjGEOM_NONE, NULL, NULL, NULL, NULL); \
      thisgeom->objtype = objtype;                                 \
      thisgeom->objid = i;                                         \
      thisgeom->category = category;                               \
      thisgeom->segid = scn->ngeom;                                \
    }


// advance counter
#define FINISH { scn->ngeom++; }



// add contact-related geoms in mjvObject
static void addContactGeom(const mjModel* m, mjData* d, const mjtByte* flags,
                           const mjvOption* vopt, mjvScene* scn) {
  int body1, body2;
  int objtype = mjOBJ_UNKNOWN, category = mjCAT_DECOR;
  mjtNum mat[9], tmp[9], vec[3], frc[3], confrc[6], axis[3];
  mjtNum framewidth, framelength, scl = m->stat.meansize;
  mjContact* con;
  mjvGeom* thisgeom;
  mjtByte split;

  // fast return if all relevant features are disabled
  if (!flags[mjVIS_CONTACTPOINT] && !flags[mjVIS_CONTACTFORCE] && vopt->frame!=mjFRAME_CONTACT) {
    return;
  }

  // loop over contacts included in impulse solver
  for (int i=0; i<d->ncon; i++) {
    // get pointer
    con = d->contact + i;

    // mat = contact rotation matrix (normal along z)
    mju_copy(tmp, con->frame+3, 6);
    mju_copy(tmp+6, con->frame, 3);
    mju_transpose(mat, tmp, 3, 3);

    // contact point
    if (flags[mjVIS_CONTACTPOINT]) {
      START
      thisgeom->type = mjGEOM_CYLINDER;
      thisgeom->size[0] = thisgeom->size[1] = m->vis.scale.contactwidth * scl;
      thisgeom->size[2] = m->vis.scale.contactheight * scl;
      mju_n2f(thisgeom->pos, con->pos, 3);
      mju_n2f(thisgeom->mat, mat, 9);

      // different colors for included and excluded contacts
      if (d->contact[i].efc_address>=0) {
        f2f(thisgeom->rgba, m->vis.rgba.contactpoint, 4);
      } else {
        f2f(thisgeom->rgba, m->vis.rgba.contactgap, 4);
      }

      FINISH
    }

    // contact frame
    if (vopt->frame==mjFRAME_CONTACT) {
      // set length and width of axis cylinders using half regular frame scaling
      framelength = m->vis.scale.framelength * scl / 2;
      framewidth = m->vis.scale.framewidth * scl / 2;

      // draw the three axes (separate geoms)
      for (int j=0; j<3; j++) {
        START

        // prepare axis
        for (int k=0; k<3; k++) {
          axis[k] = (j==k ? framelength : 0);
        }
        mju_mulMatVec(vec, mat, axis, 3, 3);

        // create a cylinder
        mjv_makeConnector(thisgeom, mjGEOM_CYLINDER, framewidth,
                          con->pos[0],
                          con->pos[1],
                          con->pos[2],
                          con->pos[0] + vec[0],
                          con->pos[1] + vec[1],
                          con->pos[2] + vec[2]);

        // set color: R, G or B depending on axis
        for (int k=0; k<3; k++) {
          thisgeom->rgba[k] = (j==k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }

    // nothing else to do for excluded contacts
    if (d->contact[i].efc_address<0) {
      continue;
    }

    // mat = contact frame rotation matrix (normal along x)
    mju_transpose(mat, con->frame, 3, 3);

    // get contact force:torque in contact frame
    mj_contactForce(m, d, i, confrc);

    // contact force
    if (flags[mjVIS_CONTACTFORCE]) {
      // get force, fill zeros if only normal
      mju_zero3(frc);
      mju_copy(frc, confrc, mjMIN(3, con->dim));
      if (mju_norm3(frc)<mjMINVAL) {
        continue;
      }

      // render combined or split
      split = (flags[mjVIS_CONTACTSPLIT] && con->dim>1);
      for (int j = (split ? 1 : 0); j < (split ? 3 : 1); j++) {
        // set vec to combined, normal or friction force, in world frame
        switch (j) {
        case 0:             // combined
          mju_mulMatVec(vec, mat, frc, 3, 3);
          break;
        case 1:             // normal
          vec[0] = mat[0]*frc[0];
          vec[1] = mat[3]*frc[0];
          vec[2] = mat[6]*frc[0];
          break;
        case 2:             // friction
          vec[0] = mat[1]*frc[1] + mat[2]*frc[2];
          vec[1] = mat[4]*frc[1] + mat[5]*frc[2];
          vec[2] = mat[7]*frc[1] + mat[8]*frc[2];
          break;
        }

        // scale vector
        mju_scl3(vec, vec, m->vis.map.force/m->stat.meanmass);

        // get body ids
        body1 = m->geom_bodyid[con->geom1];
        body2 = m->geom_bodyid[con->geom2];

        // make sure arrow points towards body with higher id
        if (body1>body2) {
          mju_scl3(vec, vec, -1);
        }

        // one-directional arrow for friction and world, symmetric otherwise
        START
        mjv_makeConnector(thisgeom,
                          body1>0 && body2>0 && !split ? mjGEOM_ARROW2 : mjGEOM_ARROW,
                          m->vis.scale.forcewidth * scl,
                          con->pos[0],
                          con->pos[1],
                          con->pos[2],
                          con->pos[0] + vec[0],
                          con->pos[1] + vec[1],
                          con->pos[2] + vec[2]);
        f2f(thisgeom->rgba, j==2 ? m->vis.rgba.contactfriction : m->vis.rgba.contactforce, 4);
        if (vopt->label==mjLABEL_CONTACTFORCE && j==(split ? 1 : 0)) {
          mjSNPRINTF(thisgeom->label, "%-.3g", mju_norm3(frc));
        }
        FINISH
      }
    }
  }
}



// copy material fields from model to visual geom
static void setMaterial(const mjModel* m, mjvGeom* geom, int matid, const float* rgba,
                        const mjtByte* flags) {
  // set material properties if given
  if (matid>=0) {
    f2f(geom->texrepeat, m->mat_texrepeat + 2*matid, 2);
    f2f(geom->rgba, m->mat_rgba + 4*matid, 4);
    geom->texuniform = m->mat_texuniform[matid];
    geom->emission = m->mat_emission[matid];
    geom->specular = m->mat_specular[matid];
    geom->shininess = m->mat_shininess[matid];
    geom->reflectance = m->mat_reflectance[matid];
  }

  // otherwise clear texrepeat
  else {
    geom->texrepeat[0] = 0;
    geom->texrepeat[1] = 0;
  }

  // use rgba if different from default, or no material given
  if (rgba[0]!=0.5f || rgba[1]!=0.5f || rgba[2]!=0.5f || rgba[3]!=1.0f || matid<0) {
    f2f(geom->rgba, rgba, 4);
  }

  // set texture
  if (flags[mjVIS_TEXTURE] && matid>=0) {
    geom->texid = m->mat_texid[matid];
  }

  // scale alpha for dynamic geoms only
  if (flags[mjVIS_TRANSPARENT] && (geom->category==mjCAT_DYNAMIC)) {
    geom->rgba[3] *= m->vis.map.alpha;
  }
}



//----------------------------- main API functions -------------------------------------------------

// set (type, size, pos, mat) connector-type geom between given points
//  assume that mjv_initGeom was already called to set all other properties
void mjv_makeConnector(mjvGeom* geom, int type, mjtNum width,
                       mjtNum a0, mjtNum a1, mjtNum a2,
                       mjtNum b0, mjtNum b1, mjtNum b2) {
  mjtNum quat[4], mat[9], dif[3] = {b0-a0, b1-a1, b2-a2};

  // require connector-compatible type
  if (type!=mjGEOM_CAPSULE && type!=mjGEOM_CYLINDER &&
      type!=mjGEOM_ARROW && type!=mjGEOM_ARROW1 && type!=mjGEOM_ARROW2
      && type!=mjGEOM_LINE) {
    mju_error_i("Invalid geom type %d for connector", type);
  }

  // assign type
  geom->type = type;

  // compute size for XYZ scaling
  geom->size[0] = geom->size[1] = (float)width;
  geom->size[2] = (float)mju_norm3(dif);

  // cylinder and capsule are centered, and size[0] is "radius"
  if (type==mjGEOM_CAPSULE || type==mjGEOM_CYLINDER) {
    geom->pos[0] = 0.5*(a0 + b0);
    geom->pos[1] = 0.5*(a1 + b1);
    geom->pos[2] = 0.5*(a2 + b2);
    geom->size[2] *= 0.5;
  }

  // arrow is not centered
  else {
    geom->pos[0] = a0;
    geom->pos[1] = a1;
    geom->pos[2] = a2;
  }

  // set mat to minimal rotation aligning b-a with z axis
  mju_quatZ2Vec(quat, dif);
  mju_quat2Mat(mat, quat);
  mju_n2f(geom->mat, mat, 9);
}



// initialize given fields when not NULL, set the rest to their default values
void mjv_initGeom(mjvGeom* geom, int type, const mjtNum* size,
                  const mjtNum* pos, const mjtNum* mat, const float* rgba) {
  // assign type
  geom->type = type;

  // set size (for XYZ scaling)
  if (size) {
    switch (type) {
      case mjGEOM_SPHERE:
        geom->size[0] = (float)size[0];
        geom->size[1] = (float)size[0];
        geom->size[2] = (float)size[0];
        break;

      case mjGEOM_CAPSULE:
        geom->size[0] = (float)size[0];
        geom->size[1] = (float)size[0];
        geom->size[2] = (float)size[1];
        break;

      case mjGEOM_CYLINDER:
        geom->size[0] = (float)size[0];
        geom->size[1] = (float)size[0];
        geom->size[2] = (float)size[1];
        break;

      default:
        mju_n2f(geom->size, size, 3);
    }
  } else {
    geom->size[0] = 0.1f;
    geom->size[1] = 0.1f;
    geom->size[2] = 0.1f;
  }

  // set pos
  if (pos) {
    mju_n2f(geom->pos, pos, 3);
  } else {
    geom->pos[0] = 0;
    geom->pos[1] = 0;
    geom->pos[2] = 0;
  }

  // set mat
  if (mat) {
    mju_n2f(geom->mat, mat, 9);
  } else {
    geom->mat[0] = 1;
    geom->mat[1] = 0;
    geom->mat[2] = 0;
    geom->mat[3] = 0;
    geom->mat[4] = 1;
    geom->mat[5] = 0;
    geom->mat[6] = 0;
    geom->mat[7] = 0;
    geom->mat[8] = 1;
  }

  // set rgba
  if (rgba) {
    f2f(geom->rgba, rgba, 4);
  } else {
    geom->rgba[0] = 0.5;
    geom->rgba[1] = 0.5;
    geom->rgba[2] = 0.5;
    geom->rgba[3] = 1;
  }

  // set defaults that cannot be assigned via this function
  geom->dataid       = -1;
  geom->texid        = -1;
  geom->texuniform   = 0;
  geom->texcoord     = 0;
  geom->texrepeat[0] = 1;
  geom->texrepeat[1] = 1;
  geom->emission     = 0;
  geom->specular     = 0.5;
  geom->shininess    = 0.5;
  geom->reflectance  = 0;
  geom->label[0]     = 0;
  geom->modelrbound  = 0;
}



// mark geom as selected
static void markselected(const mjVisual* vis, mjvGeom* geom) {
  // add emission
  geom->emission += vis->global.glow;
}



// mix colors for perturbation object
static void mixcolor(float rgba[4], const float ref[4], int flg1, int flg2) {
  rgba[0] = flg1 ? ref[0] : 0;
  if (flg2) {
    rgba[0] = mjMAX(rgba[0], ref[1]);
  }

  rgba[1] = flg1 ? ref[1] : 0;
  if (flg2) {
    rgba[1] = mjMAX(rgba[1], ref[0]);
  }

  rgba[2] = ref[2];
  rgba[3] = ref[3];
}



// a body is static if it is welded to the world and is not a mocap body
static int bodycategory(const mjModel* m, int bodyid) {
  if (m->body_weldid[bodyid]==0 && m->body_mocapid[bodyid]==-1) {
    return mjCAT_STATIC;
  } else {
    return mjCAT_DYNAMIC;
  }
}

// add abstract geoms
void mjv_addGeoms(const mjModel* m, mjData* d, const mjvOption* vopt,
                  const mjvPerturb* pert, int catmask, mjvScene* scn) {
  int objtype, category;
  mjtNum sz[3], mat[9], selpos[3];
  mjtNum catenary[3*mjNCATENARY];
  mjtNum *cur, *nxt, *xpos, *xfrc;
  mjtNum vec[3], end[3], axis[3], rod, len, det, tmp[9], quat[4];
  mjtByte broken;
  mjContact *con;
  mjvGeom* thisgeom;
  mjvPerturb localpert;
  float scl = m->stat.meansize, rgba[4];

  // make default pert if missing
  if (!pert) {
    mjv_defaultPerturb(&localpert);
    pert = &localpert;
  }

  // clear mjCAT_STATIC bit if mjVIS_STATIC is not set
  if (!vopt->flags[mjVIS_STATIC]) {
    catmask &= (~mjCAT_STATIC);
  }

  // skin
  objtype = mjOBJ_SKIN;
  category = mjCAT_DYNAMIC;
  if (vopt->flags[mjVIS_SKIN] && (category & catmask)) {
    for (int i=0; i<m->nskin; i++) {
      START

      // construct geom, pos = first bone
      mjv_initGeom(thisgeom, mjGEOM_SKIN, NULL,
                   d->xpos + 3*m->skin_bonebodyid[m->skin_boneadr[i]], NULL, NULL);

      // set material properties
      setMaterial(m, thisgeom, m->skin_matid[i], m->skin_rgba+4*i, vopt->flags);

      // glow skin if selected
      if (pert->skinselect==i) {
        markselected(&m->vis, thisgeom);
      }

      // set texcoord
      if (m->skin_texcoordadr[i]>=0) {
        thisgeom->texcoord = 1;
      }

      // skip if alpha is 0
      if (thisgeom->rgba[3]==0) {
        continue;
      }

      // vopt->label
      if (vopt->label==mjLABEL_SKIN) {
        makeLabel(m, mjOBJ_SKIN, i, thisgeom->label);
      }

      FINISH
    }
  }

  // inertia
  objtype = mjOBJ_BODY;
  if (vopt->flags[mjVIS_INERTIA]) {
    for (int i=1; i<m->nbody; i++) {
      // skip if mass too small or if this body is static and static bodies are masked
      if (m->body_mass[i]>mjMINVAL && (bodycategory(m, i) & catmask)) {
        START

        // compute sizes of equivalent box
        sz[0] = mju_sqrt((m->body_inertia[3*i+1] + m->body_inertia[3*i+2] -
                          m->body_inertia[3*i+0]) *6/m->body_mass[i]) /2;
        sz[1] = mju_sqrt((m->body_inertia[3*i+0] + m->body_inertia[3*i+2] -
                          m->body_inertia[3*i+1]) *6/m->body_mass[i]) /2;
        sz[2] = mju_sqrt((m->body_inertia[3*i+0] + m->body_inertia[3*i+1] -
                          m->body_inertia[3*i+2]) *6/m->body_mass[i]) /2;

        // scale with mass if enabled
        if (vopt->flags[mjVIS_SCLINERTIA]) {
          // density = mass / volume
          mjtNum density = m->body_mass[i] / mju_max(mjMINVAL, 8*sz[0]*sz[1]*sz[2]);

          // scale = root3(density)
          mjtNum scl = mju_pow(density*0.001, 1.0/3.0);

          // scale sizes, so that box with density of 1000 has same mass
          sz[0] *= scl;
          sz[1] *= scl;
          sz[2] *= scl;
        }

        // construct geom
        mjv_initGeom(thisgeom, mjGEOM_BOX, sz, d->xipos+3*i, d->ximat+9*i, m->vis.rgba.inertia);

        // glow
        if (pert->select==i) {
          markselected(&m->vis, thisgeom);
        }

        // vopt->label
        if (vopt->label==mjLABEL_BODY || (vopt->label==mjLABEL_SELECTION && pert->select==i)) {
          makeLabel(m, mjOBJ_BODY, i, thisgeom->label);
        }

        FINISH
      }
    }
  }

  // connector to mouse perturbation target
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_PERTOBJ] && (category & catmask) && pert->select>0) {
    int i = pert->select;

    if ((pert->active | pert->active2) & mjPERT_TRANSLATE) {
      START

      // construct geom
      sz[0] = scl * m->vis.scale.constraint;
      mjv_makeConnector(thisgeom, mjGEOM_CAPSULE, sz[0],
                        d->xipos[3*i], d->xipos[3*i+1], d->xipos[3*i+2],
                        pert->refpos[0], pert->refpos[1], pert->refpos[2]);

      // prepare color
      mixcolor(rgba, m->vis.rgba.constraint,
               (pert->active & mjPERT_TRANSLATE)>0,
               (pert->active2 & mjPERT_TRANSLATE)>0);

      f2f(thisgeom->rgba, rgba, 4);

      FINISH

      // add small sphere at end-effector
      START

      // construct geom
      sz[0] = 2*sz[0];
      sz[1] = sz[2] = sz[0];
      mju_quat2Mat(mat, pert->refquat);
      mjv_initGeom(thisgeom, mjGEOM_SPHERE, sz, pert->refpos, mat, rgba);

      FINISH
    }

    if ((pert->active | pert->active2) & mjPERT_ROTATE) {
      START

      // prepare color, use inertia color
      mixcolor(rgba, m->vis.rgba.inertia,
               (pert->active & mjPERT_ROTATE)>0,
               (pert->active2 & mjPERT_ROTATE)>0);

      // construct geom
      sz[0] = sz[1] = sz[2] = scl;
      mju_quat2Mat(mat, pert->refquat);
      mjv_initGeom(thisgeom, mjGEOM_BOX, sz, d->xipos+3*i, mat, rgba);

      FINISH
    }
  }

  // world and body frame
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if (category & catmask) {
    for (int i = (vopt->frame==mjFRAME_WORLD ? 0 : 1);
         i < (vopt->frame==mjFRAME_BODY ? m->nbody : 1);
         i++) {
      // set length(1) and width(0) of the axis cylinders
      if (i==0) {
        sz[1] = m->vis.scale.framelength * scl * 2;
        sz[0] = m->vis.scale.framewidth * scl * 2;
      } else {
        sz[1] = m->vis.scale.framelength * scl;
        sz[0] = m->vis.scale.framewidth * scl;
      }

      // skip if body is static and static bodies are masked
      if (i>0 && bodycategory(m, i) & ~catmask) {
        continue;
      }

      // draw the three axes (separate geoms)
      for (int j=0; j<3; j++) {
        START

        // prepare axis
        for (int k=0; k<3; k++) {
          axis[k] = (j==k ? sz[1] : 0);
        }
        mju_mulMatVec(vec, d->xmat+9*i, axis, 3, 3);

        // create a cylinder
        mjv_makeConnector(thisgeom, mjGEOM_CYLINDER, sz[0],
                          d->xpos[3*i+0],
                          d->xpos[3*i+1],
                          d->xpos[3*i+2],
                          d->xpos[3*i+0] + vec[0],
                          d->xpos[3*i+1] + vec[1],
                          d->xpos[3*i+2] + vec[2]);

        // set color: R, G or B depending on axis
        for (int k=0; k<3; k++) {
          thisgeom->rgba[k] = (j==k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }
  }

  // selection point
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if ((category & catmask) && pert->select>0 && vopt->flags[mjVIS_SELECT]) {
    int i=0;
    // compute selection point in world coordinates
    mju_rotVecMat(selpos, pert->localpos, d->xmat+9*pert->select);
    mju_addTo3(selpos, d->xpos+3*pert->select);

    START
    thisgeom->type = mjGEOM_SPHERE;
    thisgeom->size[0] = thisgeom->size[1] = thisgeom->size[2] = scl * m->vis.scale.selectpoint;
    mju_n2f(thisgeom->pos, selpos, 3);
    mju_n2f(thisgeom->mat, IDENTITY, 9);
    f2f(thisgeom->rgba, m->vis.rgba.selectpoint, 4);
    if (vopt->label==mjLABEL_SELPNT) {
      mjSNPRINTF(
        thisgeom->label, "%.3f %.3f %.3f (local %.3f %.3f %.3f)",
        selpos[0], selpos[1], selpos[2],
        pert->localpos[0], pert->localpos[1], pert->localpos[2]);
    }
    FINISH
  }

  // label bodies when inertia boxes are not shown
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if ((category & catmask) && (vopt->label==mjLABEL_SELECTION || vopt->label==mjLABEL_BODY) &&
      !vopt->flags[mjVIS_INERTIA]) {
    for (int i=1; i<m->nbody; i++) {
      if (vopt->label==mjLABEL_BODY || (vopt->label==mjLABEL_SELECTION && pert->select==i)) {
        // skip if body is static and static bodies are masked
        if (bodycategory(m, i) & ~catmask) {
          continue;
        }
        START

        // construct geom
        thisgeom->type = mjGEOM_LABEL;
        mju_n2f(thisgeom->pos, d->xipos+3*i, 3);
        mju_n2f(thisgeom->mat, d->ximat+9*i, 9);

        // vopt->label
        makeLabel(m, mjOBJ_BODY, i, thisgeom->label);

        FINISH
      }
    }
  }

  // joint
  objtype = mjOBJ_JOINT;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_JOINT] && (category & catmask)) {
    for (int i=0; i<m->njnt; i++) {
      if (vopt->jointgroup[mjMAX(0, mjMIN(mjNGROUP-1, m->jnt_group[i]))]) {
        // set length(1) and width(0) of the connectors
        sz[1] = m->vis.scale.jointlength * scl;
        sz[0] = m->vis.scale.jointwidth * scl;

        START

        // set type, size, pos, mat depending on joint type
        int j = m->jnt_bodyid[i];
        switch (m->jnt_type[i]) {
        case mjJNT_FREE:
          thisgeom->type = mjGEOM_BOX;
          thisgeom->size[0] = thisgeom->size[1] = thisgeom->size[2] = 0.3*sz[1];
          mju_n2f(thisgeom->pos, d->xanchor+3*i, 3);
          mju_n2f(thisgeom->mat, d->xmat+9*j, 9);
          break;

        case mjJNT_BALL:
          thisgeom->type = mjGEOM_SPHERE;
          thisgeom->size[0] = thisgeom->size[1] = thisgeom->size[2] = 0.3*sz[1];
          mju_n2f(thisgeom->pos, d->xanchor+3*i, 3);
          mju_n2f(thisgeom->mat, d->xmat+9*j, 9);
          break;

        case mjJNT_SLIDE:
        case mjJNT_HINGE:
          mjv_makeConnector(thisgeom,
                            m->jnt_type[i]==mjJNT_SLIDE ? mjGEOM_ARROW : mjGEOM_ARROW1, sz[0],
                            d->xanchor[3*i+0],
                            d->xanchor[3*i+1],
                            d->xanchor[3*i+2],
                            d->xanchor[3*i+0] + sz[1]*d->xaxis[3*i+0],
                            d->xanchor[3*i+1] + sz[1]*d->xaxis[3*i+1],
                            d->xanchor[3*i+2] + sz[1]*d->xaxis[3*i+2]);
          break;

        default:
          mju_error_i("Unknown joint type %d in mjv_visualize", m->jnt_type[i]);
        }

        f2f(thisgeom->rgba, m->vis.rgba.joint, 4);

        // vopt->label
        if (vopt->label==mjLABEL_JOINT) {
          makeLabel(m, mjOBJ_JOINT, i, thisgeom->label);
        }

        FINISH
      }
    }
  }

  // actuator
  objtype = mjOBJ_ACTUATOR;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_ACTUATOR] && (category & catmask)) {
    for (int i=0; i<m->nu; i++) {
      if (vopt->actuatorgroup[mjMAX(0, mjMIN(mjNGROUP-1, m->actuator_group[i]))]) {
        // determine extended range
        mjtNum rng[3] = {-1, 0, +1};
        mjtNum rmin = -1, rmax = 1, act;
        if (m->actuator_ctrllimited[i]) {
          rmin = m->actuator_ctrlrange[2*i];
          rmax = m->actuator_ctrlrange[2*i+1];
        } else if (vopt->flags[mjVIS_ACTIVATION] && m->actuator_actlimited[i]) {
          rmin = m->actuator_actrange[2*i];
          rmax = m->actuator_actrange[2*i+1];
        }
        if (rmin>=0) {
          rng[0] = -1;
          rng[1] = rmin;
          rng[2] = rmax;
        } else if (rmax<=0) {
          rng[0] = rmin;
          rng[1] = rmax;
          rng[2] = +1;
        } else {
          rng[0] = rmin;
          rng[1] = 0;
          rng[2] = rmax;
        }

        // adjust small ranges
        if (rng[1]-rng[0]<mjMINVAL) {
          rng[0] = rng[1] - mjMINVAL;
        }
        if (rng[2]-rng[1]<mjMINVAL) {
          rng[2] = rng[1] + mjMINVAL;
        }

        // clamp act to extended range
        if (vopt->flags[mjVIS_ACTIVATION] && m->actuator_dyntype[i]) {
          act = mju_clip(d->act[i-(m->nu-m->na)], rng[0], rng[2]);
        } else {
          act = mju_clip(d->ctrl[i], rng[0], rng[2]);
        }

        // compute interpolants
        float amin, amean, amax;
        if (act<=rng[1]) {
          amin = (rng[1]-act) / mjMAX(mjMINVAL, rng[1]-rng[0]);
          amean = 1 - amin;
          amax = 0;
        } else {
          amax = (act-rng[1]) / mjMAX(mjMINVAL, rng[2]-rng[1]);
          amean = 1 - amax;
          amin = 0;
        }

        // interpolated color
        float rgba[4];
        for (int j=0; j<4; j++) {
          rgba[j] = amin*m->vis.rgba.actuatornegative[j] +
                    amean*m->vis.rgba.actuator[j] +
                    amax*m->vis.rgba.actuatorpositive[j];
        }

        // get transmission object id
        int j = m->actuator_trnid[2*i];

        // slide and hinge joint actuators
        if (m->actuator_trntype[i]==mjTRN_JOINT ||
            m->actuator_trntype[i]==mjTRN_JOINTINPARENT ||
            m->actuator_trntype[i]==mjTRN_SITE) {
          START

          // site actuators
          if (m->actuator_trntype[i]==mjTRN_SITE) {
            // inflate sizes by 5%
            mju_scl3(sz, m->site_size+3*j, 1.05);

            // make geom
            mjv_initGeom(thisgeom,
                         m->site_type[j], sz,
                         d->site_xpos + 3*j,
                         d->site_xmat + 9*j,
                         thisgeom->rgba);
          } else if (m->jnt_type[j]==mjJNT_HINGE || m->jnt_type[j]==mjJNT_SLIDE) {
            // set length(1) and width(0) of the connectors
            sz[1] = m->vis.scale.actuatorlength * scl;
            sz[0] = m->vis.scale.actuatorwidth * scl;

            // make geom
            mjv_makeConnector(thisgeom,
                              m->jnt_type[j]==mjJNT_SLIDE ? mjGEOM_ARROW : mjGEOM_ARROW1, sz[0],
                              d->xanchor[3*j+0],
                              d->xanchor[3*j+1],
                              d->xanchor[3*j+2],
                              d->xanchor[3*j+0] + sz[1]*d->xaxis[3*j+0],
                              d->xanchor[3*j+1] + sz[1]*d->xaxis[3*j+1],
                              d->xanchor[3*j+2] + sz[1]*d->xaxis[3*j+2]);
          }

          // ball or free joint
          else if (m->jnt_type[j]==mjJNT_BALL || m->jnt_type[j]==mjJNT_FREE) {
            sz[0] = sz[1] = sz[2] = m->vis.scale.jointlength * scl * 0.33;

            // make geom
            mjv_initGeom(thisgeom,
                         m->jnt_type[j]==mjJNT_BALL ? mjGEOM_SPHERE : mjGEOM_BOX, sz,
                         d->xanchor + 3*j,
                         d->xmat + 9*m->jnt_bodyid[j],
                         thisgeom->rgba);
          }

          // set interpolated color
          f2f(thisgeom->rgba, rgba, 4);

          // vopt->label
          if (vopt->label==mjLABEL_ACTUATOR) {
            makeLabel(m, mjOBJ_ACTUATOR, i, thisgeom->label);
          }

          FINISH
        }

        // body actuators
        else if (m->actuator_trntype[i]==mjTRN_BODY) {
          // iterate over body's geoms
          int geomnum = m->body_geomnum[j];
          int geomadr = m->body_geomadr[j];
          for (int k=geomadr; k<geomadr+geomnum; k++) {
            int geomtype = m->geom_type[k];
            // add inflated geom if it is a regular primitive
            if (geomtype != mjGEOM_PLANE && geomtype != mjGEOM_HFIELD && geomtype != mjGEOM_MESH) {
              START
              // inflate sizes by 5%
              mju_scl3(sz, m->geom_size+3*k, 1.05);

              // make geom
              mjv_initGeom(thisgeom,
                           m->geom_type[k], sz,
                           d->geom_xpos + 3*k,
                           d->geom_xmat + 9*k,
                           thisgeom->rgba);

              // set interpolated color
              f2f(thisgeom->rgba, rgba, 4);

              FINISH
            }
          }
        }

        // spatial tendon actuators
        else if (m->actuator_trntype[i]==mjTRN_TENDON && d->ten_wrapnum[j]) {
          for (int k=d->ten_wrapadr[j]; k<d->ten_wrapadr[j]+d->ten_wrapnum[j]-1; k++) {
            if (d->wrap_obj[k]!=-2 && d->wrap_obj[k+1]!=-2) {
              START

              // determine width: smaller for segments inside wrapping objects
              if (d->wrap_obj[k]>=0 && d->wrap_obj[k+1]>=0) {
                sz[0] = 0.5 * m->tendon_width[j];
              } else {
                sz[0] = m->tendon_width[j];
              }

              // increase width for actuator
              sz[0] *= m->vis.map.actuatortendon;

              // construct geom
              mjv_makeConnector(thisgeom, mjGEOM_CAPSULE, sz[0],
                                d->wrap_xpos[3*k], d->wrap_xpos[3*k+1], d->wrap_xpos[3*k+2],
                                d->wrap_xpos[3*k+3], d->wrap_xpos[3*k+4], d->wrap_xpos[3*k+5]);

              // set material if given
              setMaterial(m, thisgeom, m->tendon_matid[j], m->tendon_rgba+4*j, vopt->flags);

              // set interpolated color
              f2f(thisgeom->rgba, rgba, 4);

              // vopt->label: only the first segment
              if (vopt->label==mjLABEL_ACTUATOR && k==d->ten_wrapadr[j]) {
                makeLabel(m, mjOBJ_ACTUATOR, i, thisgeom->label);
              }

              FINISH
            }
          }
        }
      }
    }
  }

  // geom
  int planeid = -1;
  for (int i=0; i<m->ngeom; i++) {
    // count planes, put current plane number in geom->dataid
    if (m->geom_type[i]==mjGEOM_PLANE) {
      planeid++;
    }

    // set type and category: geom
    objtype = mjOBJ_GEOM;
    category = bodycategory(m, m->geom_bodyid[i]);

    // skip if category is masked
    if (!(category & catmask)) {
      continue;
    }

    // get geom group and clamp
    int j = mjMAX(0, mjMIN(mjNGROUP-1, m->geom_group[i]));

    if (vopt->geomgroup[j]) {
      START

      // construct geom
      mjv_initGeom(thisgeom, m->geom_type[i], m->geom_size+3*i,
                   d->geom_xpos+3*i, d->geom_xmat+9*i, NULL);
      thisgeom->dataid = m->geom_dataid[i];

      // copy rbound from model
      thisgeom->modelrbound = (float)m->geom_rbound[i];

      // set material properties
      setMaterial(m, thisgeom, m->geom_matid[i], m->geom_rgba+4*i, vopt->flags);

      // set texcoord
      if (m->geom_type[i]==mjGEOM_MESH &&
          m->geom_dataid[i]>=0 &&
          m->mesh_texcoordadr[m->geom_dataid[i]]>=0) {
        thisgeom->texcoord = 1;
      }

      // skip if alpha is 0
      if (thisgeom->rgba[3]==0) {
        continue;
      }

      // glow geoms of selected body
      if (pert->select>0 && pert->select==m->geom_bodyid[i]) {
        markselected(&m->vis, thisgeom);
      }

      // vopt->label
      if (vopt->label==mjLABEL_GEOM) {
        makeLabel(m, mjOBJ_GEOM, i, thisgeom->label);
      }

      // mesh: 2*i is original, 2*i+1 is convex hull
      if (m->geom_type[i]==mjGEOM_MESH) {
        thisgeom->dataid *= 2;
        if (m->mesh_graphadr[m->geom_dataid[i]]>=0 && vopt->flags[mjVIS_CONVEXHULL]) {
          thisgeom->dataid += 1;
        }
      }

      // plane
      else if (m->geom_type[i]==mjGEOM_PLANE) {
        // use current planeid
        thisgeom->dataid = planeid;

        // save initial pos
        mju_copy3(tmp, d->geom_xpos+3*i);

        // re-center infinite plane
        if (m->geom_size[3*i]<=0 || m->geom_size[3*i+1]<=0) {
          // vec = headpos - geompos
          for (j=0; j<3; j++) {
            vec[j] = 0.5*(scn->camera[0].pos[j] + scn->camera[1].pos[j]) - d->geom_xpos[3*i+j];
          }

          // construct axes
          mjtNum ax[9];
          mju_transpose(ax, d->geom_xmat+9*i, 3, 3);

          // loop over (x,y)
          for (int k=0; k<2; k++) {
            if (m->geom_size[3*i+k]<=0) {
              // compute zfar
              mjtNum zfar = m->vis.map.zfar * m->stat.extent;

              // get size increment
              mjtNum sX;
              int matid = m->geom_matid[i];
              if (matid>=0 && m->mat_texrepeat[2*matid+k]>0) {
                sX = 2/m->mat_texrepeat[2*matid+k];
              } else {
                sX = 2.1*zfar/(mjMAXPLANEGRID-2);
              }

              // project on frame, round to integer increment of size
              mjtNum dX = mju_dot3(vec, ax+3*k);
              dX = 2*sX*mju_round(0.5*dX/sX);

              // translate
              mju_addToScl3(tmp, ax+3*k, dX);
            }
          }
        }

        // set final pos
        mju_n2f(thisgeom->pos, tmp, 3);
      }

      FINISH

      // set type and category: frame
      objtype = mjOBJ_UNKNOWN;
      category = mjCAT_DECOR;
      if (!(category & catmask) || vopt->frame!=mjFRAME_GEOM) {
        continue;
      }

      // construct geom frame
      objtype = mjOBJ_UNKNOWN;
      sz[0] = m->vis.scale.framewidth * scl;
      sz[1] = m->vis.scale.framelength * scl;
      for (int j=0; j<3; j++) {
        START

        // prepare axis
        for (int k=0; k<3; k++) {
          axis[k] = (j==k ? sz[1] : 0);
        }
        mju_mulMatVec(vec, d->geom_xmat+9*i, axis, 3, 3);

        // create a cylinder
        mjv_makeConnector(thisgeom, mjGEOM_CYLINDER, sz[0],
                          d->geom_xpos[3*i+0],
                          d->geom_xpos[3*i+1],
                          d->geom_xpos[3*i+2],
                          d->geom_xpos[3*i+0] + vec[0],
                          d->geom_xpos[3*i+1] + vec[1],
                          d->geom_xpos[3*i+2] + vec[2]);

        // set color: R, G or B depending on axis
        for (int k=0; k<3; k++) {
          thisgeom->rgba[k] = (j==k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }
  }

  // site
  for (int i=0; i<m->nsite; i++) {
    // set type and category
    objtype = mjOBJ_SITE;
    category = bodycategory(m, m->site_bodyid[i]);

    // skip if category is masked
    if (!(category & catmask)) {
      continue;
    }

    // show if group enabled
    if (vopt->sitegroup[mjMAX(0, mjMIN(mjNGROUP-1, m->site_group[i]))]) {
      START

      // construct geom
      mjv_initGeom(thisgeom, m->site_type[i], m->site_size+3*i,
                   d->site_xpos+3*i, d->site_xmat+9*i, NULL);

      // set material if given
      setMaterial(m, thisgeom, m->site_matid[i], m->site_rgba+4*i, vopt->flags);

      // skip if alpha is 0
      if (thisgeom->rgba[3]==0) {
        continue;
      }

      // glow
      if (pert->select>0 && pert->select==m->site_bodyid[i]) {
        markselected(&m->vis, thisgeom);
      }

      // vopt->label
      if (vopt->label==mjLABEL_SITE) {
        makeLabel(m, mjOBJ_SITE, i, thisgeom->label);
      }

      FINISH

      // set category for site frame
      category = mjCAT_DECOR;
      if (!(category & catmask) || vopt->frame!=mjFRAME_SITE) {
        continue;
      }

      // construct site frame
      objtype = mjOBJ_UNKNOWN;
      sz[0] = m->vis.scale.framewidth * scl;
      sz[1] = m->vis.scale.framelength * scl;
      for (int j=0; j<3; j++) {
        START

        // prepare axis
        for (int k=0; k<3; k++) {
          axis[k] = (j==k ? sz[1] : 0);
        }
        mju_mulMatVec(vec, d->site_xmat+9*i, axis, 3, 3);

        // create a cylinder
        mjv_makeConnector(thisgeom, mjGEOM_CYLINDER, sz[0],
                          d->site_xpos[3*i+0],
                          d->site_xpos[3*i+1],
                          d->site_xpos[3*i+2],
                          d->site_xpos[3*i+0] + vec[0],
                          d->site_xpos[3*i+1] + vec[1],
                          d->site_xpos[3*i+2] + vec[2]);

        // set color: R, G or B depending on axis
        for (int k=0; k<3; k++) {
          thisgeom->rgba[k] = (j==k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }
  }

  // cameras
  objtype = mjOBJ_CAMERA;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_CAMERA] && (category & catmask)) {
    for (int i=0; i<m->ncam; i++) {
      START

      // construct geom: camera body
      thisgeom->type = mjGEOM_BOX;
      thisgeom->size[0] = scl * m->vis.scale.camera * 1.0;
      thisgeom->size[1] = scl * m->vis.scale.camera * 0.8;
      thisgeom->size[2] = scl * m->vis.scale.camera * 0.4;
      mju_n2f(thisgeom->pos, d->cam_xpos+3*i, 3);
      mju_n2f(thisgeom->mat, d->cam_xmat+9*i, 9);
      f2f(thisgeom->rgba, m->vis.rgba.camera, 4);

      // vopt->label
      if (vopt->label==mjLABEL_CAMERA) {
        makeLabel(m, mjOBJ_CAMERA, i, thisgeom->label);
      }

      FINISH

      START

      // construct geom: lens
      thisgeom->pos[0] = (float)(d->cam_xpos[3*i] -
                                 scl*m->vis.scale.camera*0.6 * d->cam_xmat[9*i+2]);
      thisgeom->pos[1] = (float)(d->cam_xpos[3*i+1] -
                                 scl*m->vis.scale.camera*0.6 * d->cam_xmat[9*i+5]);
      thisgeom->pos[2] = (float)(d->cam_xpos[3*i+2] -
                                 scl*m->vis.scale.camera*0.6 * d->cam_xmat[9*i+8]);
      thisgeom->type = mjGEOM_CYLINDER;
      thisgeom->size[0] = scl * m->vis.scale.camera * 0.4;
      thisgeom->size[1] = scl * m->vis.scale.camera * 0.4;
      thisgeom->size[2] = scl * m->vis.scale.camera * 0.3;
      mju_n2f(thisgeom->mat, d->cam_xmat+9*i, 9);
      f2f(thisgeom->rgba, m->vis.rgba.camera, 4);
      for (int k=0; k<3; k++) {
        thisgeom->rgba[k] *= 0.5;  // make lens body darker
      }

      FINISH

      // set category for camera frame
      category = mjCAT_DECOR;
      if (!(category & catmask) || vopt->frame!=mjFRAME_CAMERA) {
        continue;
      }

      // construct camera frame
      objtype = mjOBJ_UNKNOWN;
      sz[0] = m->vis.scale.framewidth * scl;
      sz[1] = m->vis.scale.framelength * scl;
      for (int j=0; j<3; j++) {
        START

        // prepare axis
        for (int k=0; k<3; k++) {
          axis[k] = (j==k ? sz[1] : 0);
        }
        mju_mulMatVec(vec, d->cam_xmat+9*i, axis, 3, 3);

        // create a cylinder
        mjv_makeConnector(thisgeom, mjGEOM_CYLINDER, sz[0],
                          d->cam_xpos[3*i+0],
                          d->cam_xpos[3*i+1],
                          d->cam_xpos[3*i+2],
                          d->cam_xpos[3*i+0] + vec[0],
                          d->cam_xpos[3*i+1] + vec[1],
                          d->cam_xpos[3*i+2] + vec[2]);

        // set color: R, G or B depending on axis
        for (int k=0; k<3; k++) {
          thisgeom->rgba[k] = (j==k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }
  }

  // lights
  objtype = mjOBJ_LIGHT;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_LIGHT] && (category & catmask)) {
    for (int i=0; i<m->nlight; i++) {
      // make light frame
      mju_quatZ2Vec(quat, d->light_xdir+3*i);
      mju_quat2Mat(mat, quat);

      // make light position: offset backward, to avoid casting shadow
      mju_addScl3(vec, d->light_xpos+3*i, d->light_xdir+3*i, -scl * m->vis.scale.light -0.0001);

      START

      // construct geom
      thisgeom->type = mjGEOM_CYLINDER;
      thisgeom->size[0] = scl * m->vis.scale.light * 0.8;
      thisgeom->size[1] = scl * m->vis.scale.light * 0.8;
      thisgeom->size[2] = scl * m->vis.scale.light * 1.0;
      mju_n2f(thisgeom->pos, vec, 3);
      mju_n2f(thisgeom->mat, mat, 9);
      f2f(thisgeom->rgba, m->vis.rgba.light, 4);

      // vopt->label
      if (vopt->label==mjLABEL_LIGHT) {
        makeLabel(m, mjOBJ_LIGHT, i, thisgeom->label);
      }

      FINISH

      // set category for light frame
      category = mjCAT_DECOR;
      if (!(category & catmask) || vopt->frame!=mjFRAME_LIGHT) {
        continue;
      }

      // construct light frame
      objtype = mjOBJ_UNKNOWN;
      sz[0] = m->vis.scale.framewidth * scl;
      sz[1] = m->vis.scale.framelength * scl;
      for (int j=0; j<3; j++) {
        START

        // prepare axis
        for (int k=0; k<3; k++) {
          axis[k] = (j==k ? sz[1] : 0);
        }
        mju_mulMatVec(vec, mat, axis, 3, 3);

        // create a cylinder
        mjv_makeConnector(thisgeom, mjGEOM_CYLINDER, sz[0],
                          d->light_xpos[3*i+0],
                          d->light_xpos[3*i+1],
                          d->light_xpos[3*i+2],
                          d->light_xpos[3*i+0] + vec[0],
                          d->light_xpos[3*i+1] + vec[1],
                          d->light_xpos[3*i+2] + vec[2]);

        // set color: R, G or B depending on axis
        for (int k=0; k<3; k++) {
          thisgeom->rgba[k] = (j==k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }
  }

  // spatial tendons
  objtype = mjOBJ_TENDON;
  category = mjCAT_DYNAMIC;
  if (vopt->flags[mjVIS_TENDON] && (category & catmask)) {
    for (int i=0; i<m->ntendon; i++) {
      if (vopt->tendongroup[mjMAX(0, mjMIN(mjNGROUP-1, m->tendon_group[i]))]) {
        // conditions for drawing a catenary
        int draw_catenary =
            !mjDISABLED(mjDSBL_GRAVITY)           &&  // gravity enabled
            mju_norm3(m->opt.gravity) > mjMINVAL  &&  // gravity strictly nonzero
            m->tendon_num[i] == 2                 &&  // only two sites on the tendon
            m->tendon_limited[i] == 1             &&  // limited length range
            m->tendon_range[2*i] == 0             &&  // range lower-bound is 0
            m->tendon_stiffness[i] == 0           &&  // no stiffness
            m->tendon_damping[i] == 0             &&  // no damping
            m->tendon_frictionloss[i] == 0;           // no frictionloss

        // conditions not met: draw straight lines
        if (!draw_catenary) {
          for (int j=d->ten_wrapadr[i]; j<d->ten_wrapadr[i]+d->ten_wrapnum[i]-1; j++) {
            if (d->wrap_obj[j]!=-2 && d->wrap_obj[j+1]!=-2) {
              START

              // determine width: smaller for segments inside wrapping objects
              if (d->wrap_obj[j]>=0 && d->wrap_obj[j+1]>=0) {
                sz[0] = 0.5 * m->tendon_width[i];
              } else {
                sz[0] = m->tendon_width[i];
              }

              // construct geom
              mjv_makeConnector(thisgeom, mjGEOM_CAPSULE, sz[0],
                                d->wrap_xpos[3*j], d->wrap_xpos[3*j+1], d->wrap_xpos[3*j+2],
                                d->wrap_xpos[3*j+3], d->wrap_xpos[3*j+4], d->wrap_xpos[3*j+5]);

              // set material if given
              setMaterial(m, thisgeom, m->tendon_matid[i], m->tendon_rgba+4*i, vopt->flags);

              // vopt->label: only the first segment
              if (vopt->label==mjLABEL_TENDON && j==d->ten_wrapadr[i]) {
                makeLabel(m, mjOBJ_TENDON, i, thisgeom->label);
              }

              FINISH
            }
          }
        }

        // special case handling of string-like tendons under gravity
        else {
          // two hanging points: x0, x1
          mjtNum x0[3], x1[3];
          mju_copy3(x0, d->wrap_xpos + 3*d->ten_wrapadr[i]);
          mju_copy3(x1, d->wrap_xpos + 3*d->ten_wrapadr[i] + 3);

          // length of the tendon
          mjtNum length = m->tendon_range[2*i+1];

          // points along catenary path
          int npoints = mjv_catenary(x0, x1, m->opt.gravity, length, catenary);

          // draw npoints-1 segments
          for (int j=0; j<npoints-1; j++) {
            START

            sz[0] = m->tendon_width[i];

            // construct geom
            mjv_makeConnector(thisgeom, mjGEOM_CAPSULE, sz[0],
                              catenary[3*j], catenary[3*j+1], catenary[3*j+2],
                              catenary[3*j+3], catenary[3*j+4], catenary[3*j+5]);

            // set material if given
            setMaterial(m, thisgeom, m->tendon_matid[i], m->tendon_rgba+4*i, vopt->flags);

            // vopt->label: only the first segment
            if (vopt->label==mjLABEL_TENDON && npoints/2) {
              makeLabel(m, mjOBJ_TENDON, i, thisgeom->label);
            }

            FINISH
          }
        }
      }
    }
  }

  // slider-crank
  objtype = mjOBJ_ACTUATOR;
  category = mjCAT_DYNAMIC;
  if ((category & catmask)) {
    for (int i=0; i<m->nu; i++) {
      if (m->actuator_trntype[i]==mjTRN_SLIDERCRANK) {
        // get data
        int j = m->actuator_trnid[2*i];                 // crank
        int k = m->actuator_trnid[2*i+1];               // slider
        rod = m->actuator_cranklength[i];
        axis[0] = d->site_xmat[9*k+2];
        axis[1] = d->site_xmat[9*k+5];
        axis[2] = d->site_xmat[9*k+8];

        // compute crank length
        mju_sub(vec, d->site_xpos+3*j, d->site_xpos+3*k, 3);
        len = mju_dot3(vec, axis);
        det = len*len + rod*rod - mju_dot3(vec, vec);
        broken = 0;
        if (det<0) {
          det = 0;
          broken = 1;
        }
        len = len - mju_sqrt(det);

        // compute slider endpoint
        mju_scl3(end, axis, len);
        mju_addTo3(end, d->site_xpos+3*k);

        // render slider
        START
        mjv_makeConnector(thisgeom, mjGEOM_CYLINDER, scl * m->vis.scale.slidercrank,
                          d->site_xpos[3*k], d->site_xpos[3*k+1], d->site_xpos[3*k+2],
                          end[0], end[1], end[2]);
        f2f(thisgeom->rgba, m->vis.rgba.slidercrank, 4);
        if (vopt->label==mjLABEL_ACTUATOR) {
          makeLabel(m, mjOBJ_ACTUATOR, i, thisgeom->label);
        }
        FINISH

        // render crank
        START
        mjv_makeConnector(thisgeom, mjGEOM_CAPSULE, scl * m->vis.scale.slidercrank/2.0,
                          end[0], end[1], end[2],
                          d->site_xpos[3*j+0], d->site_xpos[3*j+1], d->site_xpos[3*j+2]);
        if (broken) {
          f2f(thisgeom->rgba, m->vis.rgba.crankbroken, 4);
        } else {
          f2f(thisgeom->rgba, m->vis.rgba.slidercrank, 4);
        }
        FINISH
      }
    }
  }

  // center of mass for root bodies
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_COM] && (category & catmask)) {
    for (int i=1; i<m->nbody; i++) {
      if (m->body_rootid[i]==i) {
        START
        thisgeom->type = mjGEOM_SPHERE;
        thisgeom->size[0] = thisgeom->size[1] = thisgeom->size[2] = scl * m->vis.scale.com;
        mju_n2f(thisgeom->pos, d->subtree_com+3*i, 3);
        mju_n2f(thisgeom->mat, IDENTITY, 9);
        f2f(thisgeom->rgba, m->vis.rgba.com, 4);
        FINISH
      }
    }
  }

  // auto connect
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_AUTOCONNECT] && (category & catmask)) {
    for (int i=1; i<m->nbody; i++) {
      // do not connect to world
      if (m->body_parentid[i]==0) {
        continue;
      }

      // start at body com, connect joint centers in reverse order
      cur = d->xipos+3*i;
      if (m->body_jntnum[i]) {
        for (int j=m->body_jntadr[i]+m->body_jntnum[i]-1; j>=m->body_jntadr[i]; j--) {
          START
          nxt = d->xanchor+3*j;

          // construct geom
          mjv_makeConnector(thisgeom, mjGEOM_CAPSULE, scl * m->vis.scale.connect,
                            cur[0], cur[1], cur[2],
                            nxt[0], nxt[1], nxt[2]);
          f2f(thisgeom->rgba, m->vis.rgba.connect, 4);

          FINISH
          cur = nxt;
        }
      }

      // connect first joint (or com) to parent com
      START
      nxt = d->xipos+3*m->body_parentid[i];
      mjv_makeConnector(thisgeom, mjGEOM_CAPSULE, scl * m->vis.scale.connect,
                        cur[0], cur[1], cur[2],
                        nxt[0], nxt[1], nxt[2]);
      f2f(thisgeom->rgba, m->vis.rgba.connect, 4);
      FINISH
    }
  }

  // rangefinders
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_RANGEFINDER] && (category & catmask)) {
    for (int i=0; i<m->nsensor; i++) {
      if (m->sensor_type[i]==mjSENS_RANGEFINDER) {
        // sensor data
        mjtNum dst = d->sensordata[m->sensor_adr[i]];
        int sid = m->sensor_objid[i];

        // null output: nothing to render
        if (dst<0) {
          continue;
        }

        // make ray
        START
        mjv_makeConnector(thisgeom, mjGEOM_LINE, .01,
                          d->site_xpos[3*sid],
                          d->site_xpos[3*sid+1],
                          d->site_xpos[3*sid+2],
                          d->site_xpos[3*sid]   + d->site_xmat[9*sid+2]*dst,
                          d->site_xpos[3*sid+1] + d->site_xmat[9*sid+5]*dst,
                          d->site_xpos[3*sid+2] + d->site_xmat[9*sid+8]*dst
                         );
        f2f(thisgeom->rgba, m->vis.rgba.rangefinder, 4);
        FINISH
      }
    }
  }

  // external perturbations
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  for (int i=1; i<m->nbody; i++) {
    if (!mju_isZero(d->xfrc_applied+6*i, 6) && (category & catmask)) {
      // point of application and force
      xpos = d->xipos+3*i;
      xfrc = d->xfrc_applied+6*i;

      // force perturbation
      if (vopt->flags[mjVIS_PERTFORCE] && mju_norm3(xfrc)>mjMINVAL) {
        // map force to spatial vector in world frame
        mju_scl3(vec, xfrc, m->vis.map.force/m->stat.meanmass);

        START
        mjv_makeConnector(thisgeom, mjGEOM_ARROW,
                          m->vis.scale.forcewidth * scl,
                          xpos[0],
                          xpos[1],
                          xpos[2],
                          xpos[0] + vec[0],
                          xpos[1] + vec[1],
                          xpos[2] + vec[2]);
        f2f(thisgeom->rgba, m->vis.rgba.force, 4);
        FINISH
      }
    }
  }

  // connect and distance constraints
  objtype = mjOBJ_EQUALITY;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_CONSTRAINT] && (category & catmask) && m->neq) {
    // connect or weld
    for (int i=0; i<m->neq; i++) {
      if (m->eq_active[i] && (m->eq_type[i]==mjEQ_CONNECT || m->eq_type[i]==mjEQ_WELD)) {
        // compute endpoints in global coordinates
        int j = m->eq_obj1id[i], k = m->eq_obj2id[i];
        mju_rotVecMat(vec, m->eq_data+mjNEQDATA*i+3*(m->eq_type[i]==mjEQ_WELD), d->xmat+9*j);
        mju_addTo3(vec, d->xpos+3*j);
        mju_rotVecMat(end, m->eq_data+mjNEQDATA*i+3*(m->eq_type[i]==mjEQ_CONNECT), d->xmat+9*k);
        mju_addTo3(end, d->xpos+3*k);

        // construct geom
        sz[0] = scl * m->vis.scale.constraint;

        START
        mjv_initGeom(thisgeom, mjGEOM_SPHERE, sz, vec, d->xmat+9*j, m->vis.rgba.connect);
        if (vopt->label==mjLABEL_CONSTRAINT) {
          makeLabel(m, mjOBJ_EQUALITY, i, thisgeom->label);
        }
        FINISH

        START
        mjv_initGeom(thisgeom, mjGEOM_SPHERE, sz, end, d->xmat+9*k, m->vis.rgba.constraint);
        if (vopt->label==mjLABEL_CONSTRAINT) {
          makeLabel(m, mjOBJ_EQUALITY, i, thisgeom->label);
        }
        FINISH
      }
    }

    // distance: find constraints at the end of the contact list
    int j = d->ncon-1;
    while (j>=0 && d->contact[j].exclude==3) {
      // recover constraint id
      int i = -d->contact[j].efc_address-2;

      // segment = pos +/- normal*len
      con = d->contact+j;
      len = con->dist - m->eq_data[mjNEQDATA*i];
      mju_addScl3(vec, con->pos, con->frame, 0.5*len);
      mju_addScl3(end, con->pos, con->frame, -0.5*len);

      // connect endpoints
      START

      // construct geom
      sz[0] = scl * m->vis.scale.constraint;
      mjv_makeConnector(thisgeom, mjGEOM_CAPSULE, sz[0],
                        vec[0], vec[1], vec[2],
                        end[0], end[1], end[2]);

      f2f(thisgeom->rgba, m->vis.rgba.constraint, 4);

      // vopt->label flag
      if (vopt->label==mjLABEL_CONSTRAINT) {
        makeLabel(m, mjOBJ_EQUALITY, i, thisgeom->label);
      }

      FINISH

      j--;
    }
  }

  // contact
  if (catmask & mjCAT_DECOR) {
    addContactGeom(m, d, vopt->flags, vopt, scn);
  }
}

#undef START
#undef FINISH



// make list of lights only
void mjv_makeLights(const mjModel* m, mjData* d, mjvScene* scn) {
  mjvLight* thislight;

  // clear counter
  scn->nlight = 0;

  // headlight
  if (m->vis.headlight.active) {
    // get pointer
    thislight = scn->lights;

    // set default properties
    memset(thislight, 0, sizeof(mjvLight));
    thislight->headlight = 1;
    thislight->directional = 1;
    thislight->castshadow = 0;

    // copy colors
    f2f(thislight->ambient, m->vis.headlight.ambient, 3);
    f2f(thislight->diffuse, m->vis.headlight.diffuse, 3);
    f2f(thislight->specular, m->vis.headlight.specular, 3);

    // advance counter
    scn->nlight++;
  }

  // remaining lights
  for (int i=0; i<m->nlight && scn->nlight<mjMAXLIGHT; i++) {
    if (m->light_active[i]) {
      // get pointer
      thislight = scn->lights + scn->nlight;

      // copy properties
      memset(thislight, 0, sizeof(mjvLight));
      thislight->directional = m->light_directional[i];
      thislight->castshadow = m->light_castshadow[i];
      if (!thislight->directional) {
        f2f(thislight->attenuation, m->light_attenuation+3*i, 3);
        thislight->exponent = m->light_exponent[i];
        thislight->cutoff = m->light_cutoff[i];
      }

      // copy colors
      f2f(thislight->ambient, m->light_ambient+3*i, 3);
      f2f(thislight->diffuse, m->light_diffuse+3*i, 3);
      f2f(thislight->specular, m->light_specular+3*i, 3);

      // copy position and direction
      mju_n2f(thislight->pos, d->light_xpos+3*i, 3);
      mju_n2f(thislight->dir, d->light_xdir+3*i, 3);

      // advance counter
      scn->nlight++;
    }
  }
}



// update camera only
void mjv_updateCamera(const mjModel* m, mjData* d, mjvCamera* cam, mjvScene* scn) {
  mjtNum ca, sa, ce, se, move[3], *mat;
  mjtNum headpos[3], forward[3], up[3], right[3], ipd, fovy, znear, zfar;

  // return if nothing to do
  if (!m || !cam || cam->type==mjCAMERA_USER) {
    return;
  }

  // get znear, zfar
  znear = m->vis.map.znear * m->stat.extent;
  zfar = m->vis.map.zfar * m->stat.extent;

  // get headpos, forward[3], up, right, ipd, fovy
  switch (cam->type) {
  case mjCAMERA_FREE:
  case mjCAMERA_TRACKING:
    // get global ipd and fovy
    ipd = m->vis.global.ipd;
    fovy = m->vis.global.fovy;

    // move lookat for tracking
    if (cam->type==mjCAMERA_TRACKING) {
      // get id and check
      int bid = cam->trackbodyid;
      if (bid<0 || bid>=m->nbody) {
        mju_error("Track body id is outside valid range");
      }

      // smooth tracking of subtree com
      mju_sub3(move, d->subtree_com + 3*cam->trackbodyid, cam->lookat);
      mju_addToScl3(cam->lookat, move, 0.2);  // constant ???
    }

    // compute frame
    ca = mju_cos(cam->azimuth/180.0*mjPI);
    sa = mju_sin(cam->azimuth/180.0*mjPI);
    ce = mju_cos(cam->elevation/180.0*mjPI);
    se = mju_sin(cam->elevation/180.0*mjPI);
    forward[0] = ce*ca;
    forward[1] = ce*sa;
    forward[2] = se;
    up[0] = -se*ca;
    up[1] = -se*sa;
    up[2] = ce;
    right[0] = sa;
    right[1] = -ca;
    right[2] = 0;
    mju_addScl3(headpos, cam->lookat, forward, -cam->distance);
    break;

  case mjCAMERA_FIXED: {
      // get id and check
      int cid = cam->fixedcamid;
      if (cid<0 || cid>=m->ncam) {
        mju_error("Fixed camera id is outside valid range");
      }

      // get camera-specific ipd and fovy
      ipd = m->cam_ipd[cid];
      fovy = m->cam_fovy[cid];

      // get pointer to camera orientation matrix
      mat = d->cam_xmat + 9*cid;

      // get frame
      forward[0] = -mat[2];
      forward[1] = -mat[5];
      forward[2] = -mat[8];
      up[0] = mat[1];
      up[1] = mat[4];
      up[2] = mat[7];
      right[0] = mat[0];
      right[1] = mat[3];
      right[2] = mat[6];
      mju_copy3(headpos, d->cam_xpos + 3*cid);
    }
    break;

  default:
    mju_error("Unknown camera type in mjv_updateCamera");
  }

  // compute GL cameras
  for (int view=0; view<2; view++) {
    // set frame
    for (int i=0; i<3; i++) {
      scn->camera[view].pos[i] = (float)(headpos[i] + (view ? ipd : -ipd)*0.5*right[i]);
      scn->camera[view].forward[i] = (float)forward[i];
      scn->camera[view].up[i] = (float)up[i];
    }

    // set symmetric frustum
    scn->camera[view].frustum_center = 0;
    scn->camera[view].frustum_top = (float)znear * tanf(fovy * (float)(mjPI/360.0));
    scn->camera[view].frustum_bottom = -scn->camera[view].frustum_top;
    scn->camera[view].frustum_near = (float)znear;
    scn->camera[view].frustum_far = (float)zfar;
  }

  // disable model transformation (do not clear float data; user may need it later)
  scn->enabletransform = 0;
}


// update all skins, here for backward API compatibility
void mjv_updateSkin(const mjModel* m, mjData* d, mjvScene* scn) {
  mjvOption opt;
  mjv_defaultOption(&opt);
  mjv_updateActiveSkin(m, d, scn, &opt);
  mju_warning("mjv_updateSkin is deprecated, please use mjv_updateActiveSkin.");
}


// update visible skins only
void mjv_updateActiveSkin(const mjModel* m, mjData* d, mjvScene* scn, const mjvOption* opt) {
  // process skins
  for (int i=0; i<m->nskin; i++) {
    // get info
    int vertadr = m->skin_vertadr[i];
    int vertnum = m->skin_vertnum[i];
    int faceadr = m->skin_faceadr[i];
    int facenum = m->skin_facenum[i];

    // clear positions and normals
    memset(scn->skinvert + 3*vertadr, 0, 3*vertnum*sizeof(float));
    memset(scn->skinnormal + 3*vertadr, 0, 3*vertnum*sizeof(float));

    if (opt->skingroup[m->skin_group[i]]) {
      // accumulate positions from all bones
      for (int j=m->skin_boneadr[i];
          j<m->skin_boneadr[i]+m->skin_bonenum[i];
          j++) {
        // get bind pose
        mjtNum bindpos[3] = {
          (mjtNum) m->skin_bonebindpos[3*j],
          (mjtNum) m->skin_bonebindpos[3*j+1],
          (mjtNum) m->skin_bonebindpos[3*j+2]
        };
        mjtNum bindquat[4] = {
          (mjtNum) m->skin_bonebindquat[4*j],
          (mjtNum) m->skin_bonebindquat[4*j+1],
          (mjtNum) m->skin_bonebindquat[4*j+2],
          (mjtNum) m->skin_bonebindquat[4*j+3]
        };

        // compute rotation
        int bodyid = m->skin_bonebodyid[j];
        mjtNum quat[4], quatneg[4], rotate[9];
        mju_negQuat(quatneg, bindquat);
        mju_mulQuat(quat, d->xquat+4*bodyid, quatneg);
        mju_quat2Mat(rotate, quat);

        // compute translation
        mjtNum translate[3];
        mju_rotVecMat(translate, bindpos, rotate);
        mju_sub3(translate, d->xpos+3*bodyid, translate);

        // process all bone vertices
        for (int k=m->skin_bonevertadr[j];
            k<m->skin_bonevertadr[j]+m->skin_bonevertnum[j];
            k++) {
          // vertex id and weight
          int vid = m->skin_bonevertid[k];
          float vweight = m->skin_bonevertweight[k];

          // get original position
          mjtNum pos[3] = {
            (mjtNum) m->skin_vert[3*(vertadr+vid)],
            (mjtNum) m->skin_vert[3*(vertadr+vid)+1],
            (mjtNum) m->skin_vert[3*(vertadr+vid)+2],
          };

          // transform
          mjtNum pos1[3];
          mju_rotVecMat(pos1, pos, rotate);
          mju_addTo3(pos1, translate);

          // accumulate position
          scn->skinvert[3*(vertadr+vid)] += vweight*(float)pos1[0];
          scn->skinvert[3*(vertadr+vid)+1] += vweight*(float)pos1[1];
          scn->skinvert[3*(vertadr+vid)+2] += vweight*(float)pos1[2];
        }
      }

      // compute vertex normals from face normals
      for (int k=faceadr; k<faceadr+facenum; k++) {
        // get face vertex indices
        int vid[3] = {
          m->skin_face[3*k],
          m->skin_face[3*k+1],
          m->skin_face[3*k+2]
        };

        // get triangle edges
        mjtNum vec01[3], vec02[3];
        for (int r=0; r<3; r++) {
          vec01[r] = scn->skinvert[3*(vertadr+vid[1])+r] - scn->skinvert[3*(vertadr+vid[0])+r];
          vec02[r] = scn->skinvert[3*(vertadr+vid[2])+r] - scn->skinvert[3*(vertadr+vid[0])+r];
        }

        // compute face normal
        mjtNum nrm[3];
        mju_cross(nrm, vec01, vec02);

        // add normal to each vertex with weight = area
        for (int r=0; r<3; r++) {
          for (int t=0; t<3; t++) {
            scn->skinnormal[3*(vertadr+vid[r])+t] += nrm[t];
          }
        }
      }

      // normalize normals
      for (int k=vertadr; k<vertadr+vertnum; k++) {
        float s = sqrtf(
                    scn->skinnormal[3*k]*scn->skinnormal[3*k] +
                    scn->skinnormal[3*k+1]*scn->skinnormal[3*k+1] +
                    scn->skinnormal[3*k+2]*scn->skinnormal[3*k+2]
                  );
        float scl = 1/mjMAX(mjMINVAL, s);

        scn->skinnormal[3*k] *= scl;
        scn->skinnormal[3*k+1] *= scl;
        scn->skinnormal[3*k+2] *= scl;
      }

      // inflate
      if (m->skin_inflate[i]) {
        float inflate = m->skin_inflate[i];
        for (int k=vertadr; k<vertadr+vertnum; k++) {
          scn->skinvert[3*k]   += inflate*scn->skinnormal[3*k];
          scn->skinvert[3*k+1] += inflate*scn->skinnormal[3*k+1];
          scn->skinvert[3*k+2] += inflate*scn->skinnormal[3*k+2];
        }
      }
    }
  }
}



// update entire scene
void mjv_updateScene(const mjModel* m, mjData* d, const mjvOption* opt,
                     const mjvPerturb* pert, mjvCamera* cam, int catmask, mjvScene* scn) {
  // clear geoms and add all categories
  scn->ngeom = 0;
  mjv_addGeoms(m, d, opt, pert, catmask, scn);

  // add lights
  mjv_makeLights(m, d, scn);

  // update camera
  mjv_updateCamera(m, d, cam, scn);

  // update skins
  if (opt->flags[mjVIS_SKIN]) {
    mjv_updateActiveSkin(m, d, scn, opt);
  }
}



//----------------------------------- catenary functions -------------------------------------------

// returns hyperbolic cosine and optionally computes hyperbolic sine
static inline mjtNum cosh_sinh(mjtNum x, mjtNum *sinh) {
  mjtNum expx = mju_exp(x);
  if (sinh) {
    *sinh = 0.5 * (expx - 1/expx);
  }
  return 0.5 * (expx + 1/expx);
}



// returns intercept of the catenary equation
static inline mjtNum catenary_intercept(mjtNum v, mjtNum h, mjtNum length) {
  return 1/mju_sqrt(mju_sqrt(length*length - v*v)/h - 1);
}



// returns residual of catenary equation and optionally computes its gradient w.r.t b
static inline mjtNum catenary_residual(mjtNum b, mjtNum intercept, mjtNum *grad) {
  mjtNum a = 0.5 / b;
  mjtNum sinh, cosh = cosh_sinh(a, &sinh);
  if (grad) {
    *grad = (a*cosh - sinh) * mju_pow(2*b*sinh - 1, -1.5);
  }
  return 1/mju_sqrt(2*b*sinh - 1) - intercept;
}



// convergence tolerance for catenary solver
static const mjtNum tolerance = 1e-9;



// solve trancendental catenary equation using change of variables proposed in
//   https://math.stackexchange.com/a/1002996
static inline mjtNum solve_catenary(mjtNum v, mjtNum h, mjtNum length) {
  mjtNum intercept = catenary_intercept(v, h, length);

  // initial guess using linear approximation to catenary_residual
  mjtNum b = intercept / mju_sqrt(24);

  // Newton steps to convergence (usually ~ 5 steps)
  for (int i=0; i<50; i++) {
    // get value and gradient
    mjtNum grad;
    mjtNum res = catenary_residual(b, intercept, &grad);

    if (mju_abs(res) < tolerance) {
      break;
    }

    // Newton step
    mjtNum step = -res / grad;

    // backtracking line-search is not essential but can reduce number of iterations
    for (int j=0; j<10; j++) {
      mjtNum new_res = catenary_residual(b + step, intercept, NULL);
      if (mju_abs(new_res) < mju_abs(res)) {
        break;
      } else {
        step *= 0.5;
      }
    }

    // take step
    b += step;
  }

  return b;
}



// points along catenary of given length between x0 and x1, returns number of points
int mjv_catenary(const mjtNum x0[3], const mjtNum x1[3], const mjtNum gravity[3], mjtNum length,
                 mjtNum catenary[3*mjNCATENARY]) {
  mjtNum dist = mju_dist3(x0, x1);

  // tendon is stretched longer than length: draw straight line
  if (dist > length) {
    // copy start and end points
    mju_copy3(catenary+0, x0);
    mju_copy3(catenary+3, x1);

    return 2;
  }

  // tendon is shorter than length
  else {
    // normalized up vector
    mjtNum up[3];
    mju_scl3(up, gravity, -1);
    mju_normalize3(up);

    // x0 to x1
    mjtNum x01[3];
    mju_sub3(x01, x1, x0);

    // make across orthonormal to up, points from x0 to x1
    mjtNum across[3];
    mju_copy3(across, x01);
    mjtNum tmp[3];
    mju_scl3(tmp, up, mju_dot3(up, across));
    mju_subFrom3(across, tmp);
    mjtNum norm = mju_normalize3(across);

    // if across is numerically tiny, just set to 0
    if (norm < mjMINVAL) {
      mju_zero3(across);
    }

    // extents in the suspension plane
    mjtNum h = mju_dot3(x01, across);  // horizontal suspension extent
    mjtNum v = mju_dot3(x01, up);      // vertical height difference of x1 and x0

    // near vertical tendon, use hanging bead approximation: 3 points
    if (length > 100*h) {
      // solve for location of bead hanging on tendon
      mjtNum d_up = -0.5*(mju_sqrt(length*length - h*h) - v);  // down from x0
      mjtNum d_across = h*d_up / (2*d_up - v);                 // across from x0

      // start point
      mju_copy3(catenary+0, x0);

      // midpoint: bead location
      mju_copy3(catenary+3, x0);
      mju_addToScl3(catenary+3, up, d_up);
      mju_addToScl3(catenary+3, across, d_across);

      // end point
      mju_copy3(catenary+6, x1);

      return 3;
    }

    // compute catenary: mjNCATENARY points
    else {
      // b*h: scaled catenary flatness
      mjtNum bh = solve_catenary(v, h, length) * h;

      // horizontal and vertical offsets
      mjtNum h_offset = -0.5 * (mju_log((length+v) / (length-v)) * bh - h);
      mjtNum v_offset = -cosh_sinh(h_offset / bh, NULL) * bh;

      // start point
      mju_copy3(catenary+0, x0);

      // hanging points
      for (int i=1; i<mjNCATENARY-1; i++) {
        // linearly spaced horizontal offset
        mjtNum horizontal = i*h/mjNCATENARY;
        mju_addScl3(catenary+3*i, x0, across, horizontal);

        // vertical offset, evaluate catenary values
        mjtNum vertical = bh * cosh_sinh((horizontal - h_offset) / bh, NULL) + v_offset;
        mju_addToScl3(catenary+3*i, up, vertical);
      }

      // end point
      mju_copy3(catenary+3*(mjNCATENARY-1), x1);

      return mjNCATENARY;
    }
  }

  return 0;  // SHOULD NOT OCCUR
}

