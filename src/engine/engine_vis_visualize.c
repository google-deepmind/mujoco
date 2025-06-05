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

#include <stddef.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjvisualize.h>
#include "engine/engine_array_safety.h"
#include "engine/engine_io.h"
#include "engine/engine_name.h"
#include "engine/engine_plugin.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"
#include "engine/engine_vis_init.h"
#include "engine/engine_vis_interact.h"

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
  strncpy(label, txt, 100);
  label[99] = '\0';
}



// return if there is no space in buffer
#define START                                                    \
  if ( scn->ngeom>=scn->maxgeom ) {                              \
    mj_warning(d, mjWARN_VGEOMFULL, scn->maxgeom);               \
    return;                                                      \
  } else {                                                       \
    thisgeom = scn->geoms + scn->ngeom;                          \
    memset(thisgeom, 0, sizeof(mjvGeom));                        \
    mjv_initGeom(thisgeom, mjGEOM_NONE, NULL, NULL, NULL, NULL); \
    thisgeom->objtype = objtype;                                 \
    thisgeom->objid = i;                                         \
    thisgeom->category = category;                               \
    thisgeom->segid = scn->ngeom;                                \
  }


// advance counter
#define FINISH { scn->ngeom++; }

// assign pseudo-random rgba to constraint island using Halton sequence
static void islandColor(float rgba[4], int islanddofadr) {
  rgba[0] = 0.1f + 0.9f*mju_Halton(islanddofadr + 1, 2);
  rgba[1] = 0.1f + 0.9f*mju_Halton(islanddofadr + 1, 3);
  rgba[2] = 0.1f + 0.9f*mju_Halton(islanddofadr + 1, 5);
  rgba[3] = 1;
}

// make a triangle in thisgeom at coordinates v0, v1, v2 with a given color
static void makeTriangle(mjvGeom* thisgeom, const mjtNum v0[3], const mjtNum v1[3],
                         const mjtNum v2[3], const float rgba[4]) {
  mjtNum e1[3] =  {v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
  mjtNum e2[3] =  {v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
  mjtNum normal[3];
  mju_cross(normal, e1, e2);
  mjtNum lengths[3] = {mju_normalize3(e1), mju_normalize3(e2), mju_normalize3(normal)};
  mjtNum xmat[9] = {e1[0], e2[0], normal[0],
                    e1[1], e2[1], normal[1],
                    e1[2], e2[2], normal[2]};
  mjv_initGeom(thisgeom, mjGEOM_TRIANGLE, lengths, v0, xmat, rgba);
}

// add contact-related geoms in mjvObject
static void addContactGeom(const mjModel* m, mjData* d, const mjtByte* flags,
                           const mjvOption* vopt, mjvScene* scn) {
  int objtype = mjOBJ_UNKNOWN, category = mjCAT_DECOR;
  mjtNum mat[9], tmp[9], vec[3], frc[3], confrc[6], axis[3];
  mjtNum framewidth, framelength, scl = m->stat.meansize;
  mjContact* con;
  mjvGeom* thisgeom;
  mjtByte split;

  // fast return if all relevant features are disabled
  if (!flags[mjVIS_CONTACTPOINT] && !flags[mjVIS_CONTACTFORCE] && vopt->frame != mjFRAME_CONTACT) {
    return;
  }

  // loop over contacts
  for (int i=0; i < d->ncon; i++) {
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
      float halfheight = m->vis.scale.contactheight * scl;
      float halfdepth = -con->dist / 2;
      thisgeom->size[2] = mjMAX(halfheight, halfdepth);
      mju_n2f(thisgeom->pos, con->pos, 3);
      mju_n2f(thisgeom->mat, mat, 9);

      int efc_adr = d->contact[i].efc_address;

      // override standard colors if visualizing islands
      if (vopt->flags[mjVIS_ISLAND] && d->nisland && efc_adr >= 0) {
        // set color using island's first dof
        islandColor(thisgeom->rgba, d->island_dofadr[d->efc_island[efc_adr]]);
      }

      // otherwise regular colors (different for included and excluded contacts)
      else {
        if (efc_adr >= 0) {
          f2f(thisgeom->rgba, m->vis.rgba.contactpoint, 4);
        } else {
          f2f(thisgeom->rgba, m->vis.rgba.contactgap, 4);
        }
      }

      // label contacting geom names or ids
      if (vopt->label == mjLABEL_CONTACTPOINT) {
        char contactlabel[2][48];
        for (int k=0; k < 2; k++) {
          // make geom label
          if (con->geom[k] >= 0) {
            const char* geomname = mj_id2name(m, mjOBJ_GEOM, con->geom[k]);
            if (geomname) {
              mjSNPRINTF(contactlabel[k], "%s", geomname);
            }
            else {
              mjSNPRINTF(contactlabel[k], "g%d", con->geom[k]);
            }
          }

          // make flex elem or vert label
          else {
            const char* flexname = mj_id2name(m, mjOBJ_FLEX, con->flex[k]);
            if (flexname) {
              if (con->elem[k] >= 0) {
                mjSNPRINTF(contactlabel[k], "%s.e%d", flexname, con->elem[k]);
              }
              else {
                mjSNPRINTF(contactlabel[k], "%s.v%d", flexname, con->vert[k]);
              }
            }
            else {
              if (con->elem[k] >= 0) {
                mjSNPRINTF(contactlabel[k], "f%d.e%d", con->flex[k], con->elem[k]);
              }
              else {
                mjSNPRINTF(contactlabel[k], "f%d.v%d", con->flex[k], con->vert[k]);
              }
            }
          }
        }

        mjSNPRINTF(thisgeom->label, "%s | %s", contactlabel[0], contactlabel[1]);
      }

      FINISH
    }

    // contact frame
    if (vopt->frame == mjFRAME_CONTACT) {
      // set length and width of axis cylinders using half regular frame scaling
      framelength = m->vis.scale.framelength * scl / 2;
      framewidth = m->vis.scale.framewidth * scl / 2;

      // draw the three axes (separate geoms)
      for (int j=0; j < 3; j++) {
        START

        // prepare axis
        for (int k=0; k < 3; k++) {
          axis[k] = (j == k ? framelength : 0);
        }
        mju_mulMatVec(vec, mat, axis, 3, 3);

        // create a cylinder
        mjtNum* from = con->pos;
        mjtNum to[3];
        mju_add3(to, from, vec);
        mjv_connector(thisgeom, mjGEOM_CYLINDER, framewidth, from, to);

        // set color: R, G or B depending on axis
        for (int k=0; k < 3; k++) {
          thisgeom->rgba[k] = (j == k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }

    // nothing else to do for excluded contacts
    if (d->contact[i].efc_address < 0) {
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
      if (mju_norm3(frc) < mjMINVAL) {
        continue;
      }

      // render combined or split
      split = (flags[mjVIS_CONTACTSPLIT] && con->dim > 1);
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

        // get bodyflex ids
        int bf[2];
        for (int k=0; k < 2; k++) {
          bf[k] = (con->geom[k] >= 0) ? m->geom_bodyid[con->geom[k]] :
                                        m->nbody + con->flex[k];
        }

        // make sure arrow points towards bodyflex with higher id
        if (bf[0] > bf[1]) {
          mju_scl3(vec, vec, -1);
        }

        // one-directional arrow for friction and world, symmetric otherwise
        START
        mjtNum* from = con->pos;
        mjtNum to[3];
        mju_add3(to, from, vec);
        mjv_connector(thisgeom,
                      bf[0] > 0 && bf[1] > 0 && !split ? mjGEOM_ARROW2 : mjGEOM_ARROW,
                      m->vis.scale.forcewidth * scl,from, to);
        f2f(thisgeom->rgba, j == 2 ? m->vis.rgba.contactfriction : m->vis.rgba.contactforce, 4);
        if (vopt->label == mjLABEL_CONTACTFORCE && j == (split ? 1 : 0)) {
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
  if (matid >= 0) {
    f2f(geom->rgba, m->mat_rgba + 4*matid, 4);
    geom->emission = m->mat_emission[matid];
    geom->specular = m->mat_specular[matid];
    geom->shininess = m->mat_shininess[matid];
    geom->reflectance = m->mat_reflectance[matid];
  }

  // use rgba if different from default, or no material given
  if (rgba[0] != 0.5f || rgba[1] != 0.5f || rgba[2] != 0.5f || rgba[3] != 1.0f || matid < 0) {
    f2f(geom->rgba, rgba, 4);
  }

  // set texture
  if (flags[mjVIS_TEXTURE] && matid >= 0) {
    geom->matid = matid;
  }

  // scale alpha for dynamic geoms only
  if (flags[mjVIS_TRANSPARENT] && (geom->category == mjCAT_DYNAMIC)) {
    geom->rgba[3] *= m->vis.map.alpha;
  }
}



//----------------------------- main API functions -------------------------------------------------

// set (type, size, pos, mat) connector-type geom between given points
//  assume that mjv_initGeom was already called to set all other properties
void mjv_connector(mjvGeom* geom, int type, mjtNum width,
                   const mjtNum from[3], const mjtNum to[3]) {
  mjtNum quat[4], mat[9], dif[3] = {to[0]-from[0], to[1]-from[1], to[2]-from[2]};

  // require connector-compatible type
  if (type != mjGEOM_CAPSULE && type != mjGEOM_CYLINDER &&
      type != mjGEOM_ARROW && type != mjGEOM_ARROW1 && type != mjGEOM_ARROW2
      && type != mjGEOM_LINE) {
    mjERROR("invalid geom type %d for connector", type);
  }

  // assign type
  geom->type = type;

  // compute size for XYZ scaling
  geom->size[0] = geom->size[1] = (float)width;
  geom->size[2] = (float)mju_norm3(dif);

  // cylinder and capsule are centered, and size[0] is "radius"
  if (type == mjGEOM_CAPSULE || type == mjGEOM_CYLINDER) {
    geom->pos[0] = 0.5*(from[0] + to[0]);
    geom->pos[1] = 0.5*(from[1] + to[1]);
    geom->pos[2] = 0.5*(from[2] + to[2]);
    geom->size[2] *= 0.5;
  }

  // arrow is not centered
  else {
    geom->pos[0] = from[0];
    geom->pos[1] = from[1];
    geom->pos[2] = from[2];
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
    switch ((mjtGeom) type) {
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
  geom->matid        = -1;
  geom->texcoord     = 0;
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
  if (m->body_weldid[bodyid] == 0 && m->body_mocapid[bodyid] == -1) {
    return mjCAT_STATIC;
  } else {
    return mjCAT_DYNAMIC;
  }
}



// computes the camera frustum
static void getFrustum(float zver[2], float zhor[2], float znear,
                       const float intrinsic[4], const float sensorsize[2]) {
  zhor[0] = znear / intrinsic[0] * (sensorsize[0]/2.f - intrinsic[2]);
  zhor[1] = znear / intrinsic[0] * (sensorsize[0]/2.f + intrinsic[2]);
  zver[0] = znear / intrinsic[1] * (sensorsize[1]/2.f - intrinsic[3]);
  zver[1] = znear / intrinsic[1] * (sensorsize[1]/2.f + intrinsic[3]);
}



// add abstract geoms
void mjv_addGeoms(const mjModel* m, mjData* d, const mjvOption* vopt,
                  const mjvPerturb* pert, int catmask, mjvScene* scn) {
  int objtype, category;
  mjtNum sz[3], mat[9], selpos[3];
  mjtNum *cur, *nxt, *xfrc;
  mjtNum vec[3], end[3], axis[3], rod, len, det, tmp[9], quat[4];
  mjtByte broken;
  mjvGeom* thisgeom;
  mjvPerturb localpert;
  float scl = m->stat.meansize;
  int mark_active = m->vis.global.bvactive;

  // make default pert if missing
  if (!pert) {
    mjv_defaultPerturb(&localpert);
    pert = &localpert;
  }

  // clear mjCAT_STATIC bit if mjVIS_STATIC is not set
  if (!vopt->flags[mjVIS_STATIC]) {
    catmask &= (~mjCAT_STATIC);
  }

  // flex
  objtype = mjOBJ_FLEX;
  category = mjCAT_DYNAMIC;
  if ((vopt->flags[mjVIS_FLEXVERT] || vopt->flags[mjVIS_FLEXEDGE] ||
       vopt->flags[mjVIS_FLEXFACE] || vopt->flags[mjVIS_FLEXSKIN]) &&
      (category & catmask)) {
    for (int i=0; i < m->nflex; i++) {
      if (vopt->flexgroup[mjMAX(0, mjMIN(mjNGROUP-1, m->flex_group[i]))]) {
        START

        // construct geom, pos = first vertex
        mjv_initGeom(thisgeom, mjGEOM_FLEX, NULL,
                     d->flexvert_xpos + 3*m->flex_vertadr[i], NULL, NULL);

        // size[0] = radius
        thisgeom->size[0] = m->flex_radius[i];

        // set material properties
        setMaterial(m, thisgeom, m->flex_matid[i], m->flex_rgba+4*i, vopt->flags);

        // set texcoord
        if (m->flex_texcoordadr[i] >= 0) {
          thisgeom->texcoord = 1;
        }
        else {
          thisgeom->matid = -1;
        }

        // glow flex if selected
        if (pert->flexselect == i) {
          markselected(&m->vis, thisgeom);
        }

        // skip if alpha is 0
        if (thisgeom->rgba[3] == 0) {
          continue;
        }

        // vopt->label
        if (vopt->label == mjLABEL_FLEX) {
          makeLabel(m, mjOBJ_FLEX, i, thisgeom->label);
        }

        FINISH
      }
    }
  }

  // skin
  objtype = mjOBJ_SKIN;
  category = mjCAT_DYNAMIC;
  if (vopt->flags[mjVIS_SKIN] && (category & catmask)) {
    for (int i=0; i < m->nskin; i++) {
      if (vopt->skingroup[mjMAX(0, mjMIN(mjNGROUP-1, m->skin_group[i]))]) {
        START

        // construct geom, pos = first bone
        mjv_initGeom(thisgeom, mjGEOM_SKIN, NULL,
                     d->xpos + 3*m->skin_bonebodyid[m->skin_boneadr[i]], NULL, NULL);

        // set material properties
        setMaterial(m, thisgeom, m->skin_matid[i], m->skin_rgba+4*i, vopt->flags);

        // glow skin if selected
        if (pert->skinselect == i) {
          markselected(&m->vis, thisgeom);
        }

        // set texcoord
        if (m->skin_texcoordadr[i] >= 0) {
          thisgeom->texcoord = 1;
        }

        // skip if alpha is 0
        if (thisgeom->rgba[3] == 0) {
          continue;
        }

        // vopt->label
        if (vopt->label == mjLABEL_SKIN) {
          makeLabel(m, mjOBJ_SKIN, i, thisgeom->label);
        }

        FINISH
      }
    }
  }

  // body BVH
  category = mjCAT_DECOR;
  objtype = mjOBJ_UNKNOWN;
  if (vopt->flags[mjVIS_BODYBVH]) {
    for (int i = 0; i < m->nbvhstatic; i++) {
      int isleaf = m->bvh_child[2*i] == -1 && m->bvh_child[2*i+1] == -1;
      if (scn->ngeom >= scn->maxgeom) break;
      if (m->bvh_depth[i] != vopt->bvh_depth) {
        if (!isleaf || m->bvh_depth[i] > vopt->bvh_depth) {
          continue;
        }
      }

      // find geom number
      int bodyid = 0;
      int geomid = m->bvh_nodeid[i];
      while (i >= m->body_bvhadr[bodyid] + m->body_bvhnum[bodyid]) {
        if (++bodyid >= m->nbody) {
          break;
        }
      }

      // stop after body bvh are finished
      if (bodyid >= m->nbody) {
        break;
      }

      // get xpos, xmat, size
      const mjtNum* xpos = isleaf ? d->geom_xpos + 3 * geomid : d->xipos + 3 * bodyid;
      const mjtNum* xmat = isleaf ? d->geom_xmat + 9 * geomid : d->ximat + 9 * bodyid;
      const mjtNum *size = isleaf ? m->geom_aabb + 6*geomid + 3 : m->bvh_aabb + 6*i + 3;

      // offset xpos with aabb center (not always at frame origin)
      const mjtNum *center = isleaf ? m->geom_aabb + 6*geomid : m->bvh_aabb + 6*i;
      mjtNum pos[3];
      mju_mulMatVec3(pos, xmat, center);
      mju_addTo3(pos, xpos);

      // set box color
      const float* rgba = m->vis.rgba.bv;
      if (mark_active && d->bvh_active[i]) {
        rgba = m->vis.rgba.bvactive;
      }

      START
      mjv_initGeom(thisgeom, mjGEOM_LINEBOX, size, pos, xmat, rgba);
      FINISH

    }
  }

  // flex BVH
  category = mjCAT_DECOR;
  objtype = mjOBJ_UNKNOWN;
  if (vopt->flags[mjVIS_FLEXBVH]) {
    for (int f=0; f < m->nflex; f++) {
      if (m->flex_bvhnum[f] && vopt->flexgroup[mjMAX(0, mjMIN(mjNGROUP-1, m->flex_group[f]))]) {
        for (int i=m->flex_bvhadr[f]; i < m->flex_bvhadr[f]+m->flex_bvhnum[f]; i++) {
          int isleaf = m->bvh_child[2*i] == -1 && m->bvh_child[2*i+1] == -1;
          if (scn->ngeom >= scn->maxgeom) break;
          if (m->bvh_depth[i] != vopt->bvh_depth) {
            if (!isleaf || m->bvh_depth[i] > vopt->bvh_depth) {
              continue;
            }
          }

          // get box data
          mjtNum *aabb = d->bvh_aabb_dyn + 6*(i - m->nbvhstatic);

          // set box color
          const float* rgba = m->vis.rgba.bv;
          if (mark_active && d->bvh_active[i]) {
            rgba = m->vis.rgba.bvactive;
          }

          START
          mjv_initGeom(thisgeom, mjGEOM_LINEBOX, aabb+3, aabb, NULL, rgba);
          FINISH
        }
      }

      if (!m->flex_interp[f]) {
        continue;
      }

      // control points box
      mjtNum xpos[mjMAXFLEXNODES];
      int nstart = m->flex_nodeadr[f];
      int* bodyid = m->flex_nodebodyid + m->flex_nodeadr[f];
      if (m->flex_centered[f]) {
        for (int i=0; i < m->flex_nodenum[f]; i++) {
          mju_copy3(xpos + 3*i, d->xpos + 3*bodyid[i]);
        }
      } else {
        for (int i=0; i < m->flex_nodenum[f]; i++) {
          mju_mulMatVec3(xpos + 3*i, d->xmat + 9*bodyid[i], m->flex_node + 3*(i+nstart));
          mju_addTo3(xpos + 3*i, d->xpos + 3*bodyid[i]);
        }
      }
      for (int i=0; i < 2; i++) {
        for (int j=0; j < 2; j++) {
          for (int k=0; k < 2; k++) {
            if (scn->ngeom >= scn->maxgeom) break;
            if (i == 0) {
              START
              mjv_connector(thisgeom, mjGEOM_LINE, 3, xpos+3*(4*i+2*j+k), xpos+3*(4*(i+1)+2*j+k));
              FINISH
            }
            if (j == 0) {
              START
              mjv_connector(thisgeom, mjGEOM_LINE, 3, xpos+3*(4*i+2*j+k), xpos+3*(4*i+2*(j+1)+k));
              FINISH
            }
            if (k == 0) {
              START
              mjv_connector(thisgeom, mjGEOM_LINE, 3, xpos+3*(4*i+2*j+k), xpos+3*(4*i+2*j+(k+1)));
              FINISH
            }
          }
        }
      }
    }
  }

  // mesh BVH
  category = mjCAT_DECOR;
  objtype = mjOBJ_UNKNOWN;
  if (vopt->flags[mjVIS_MESHBVH]) {
    for (int geomid = 0; geomid < m->ngeom; geomid++) {
      int meshid = m->geom_dataid[geomid];
      if (meshid == -1) {
        continue;
      }

      for (int b = 0; b < m->mesh_bvhnum[meshid]; b++) {
        int i = b + m->mesh_bvhadr[meshid];
        int isleaf = m->bvh_child[2*i] == -1 && m->bvh_child[2*i+1] == -1;
        if (scn->ngeom >= scn->maxgeom) break;
        if (m->bvh_depth[i] != vopt->bvh_depth) {
          if (!isleaf || m->bvh_depth[i] > vopt->bvh_depth) {
            continue;
          }
        }

        // box color
        const float* rgba = m->vis.rgba.bv;
        if (mark_active) {
          if (d->bvh_active[i]) {
            rgba = m->vis.rgba.bvactive;
          } else {
            // when marking active bvs, skip inactive volumes
            continue;
          }
        }

        // get xpos, xmat, size
        const mjtNum* xpos = d->geom_xpos + 3 * geomid;
        const mjtNum* xmat = d->geom_xmat + 9 * geomid;
        const mjtNum *size = m->bvh_aabb + 6*i + 3;

        // offset xpos with aabb center (not always at geom origin)
        const mjtNum *center = m->bvh_aabb + 6*i;
        mjtNum pos[3];
        mju_mulMatVec3(pos, xmat, center);
        mju_addTo3(pos, xpos);

        START
        mjv_initGeom(thisgeom, mjGEOM_LINEBOX, size, pos, xmat, rgba);
        FINISH
      }
    }
  }

  // inertia
  objtype = mjOBJ_BODY;
  if (vopt->flags[mjVIS_INERTIA]) {
    int ellipsoid = m->vis.global.ellipsoidinertia == 1;
    for (int i=1; i < m->nbody; i++) {
      // skip if mass too small or if this body is static and static bodies are masked
      if (m->body_mass[i] > mjMINVAL && (bodycategory(m, i) & catmask)) {
        START

        mjtNum Ixx = m->body_inertia[3*i+0];
        mjtNum Iyy = m->body_inertia[3*i+1];
        mjtNum Izz = m->body_inertia[3*i+2];
        mjtNum mass = m->body_mass[i];
        mjtNum scale_inertia = ellipsoid ? mju_sqrt(5) : mju_sqrt(3);

        sz[0] = mju_sqrt((Iyy + Izz - Ixx) / (2 * mass)) * scale_inertia;
        sz[1] = mju_sqrt((Ixx + Izz - Iyy) / (2 * mass)) * scale_inertia;
        sz[2] = mju_sqrt((Ixx + Iyy - Izz) / (2 * mass)) * scale_inertia;

        // scale with mass if enabled
        if (vopt->flags[mjVIS_SCLINERTIA]) {
          // density = mass / volume
          mjtNum scale_volume = ellipsoid ? 4.0/3.0*mjPI : 8.0;
          mjtNum volume = scale_volume * sz[0]*sz[1]*sz[2];
          mjtNum density = mass / mju_max(mjMINVAL, volume);

          // scale = root3(density)
          mjtNum scale = mju_pow(density*0.001, 1.0/3.0);

          // scale sizes, so that box/ellipsoid with density of 1000 has same mass
          sz[0] *= scale;
          sz[1] *= scale;
          sz[2] *= scale;
        }

        // construct geom
        mjtGeom type = ellipsoid ? mjGEOM_ELLIPSOID : mjGEOM_BOX;
        mjv_initGeom(thisgeom, type, sz, d->xipos+3*i, d->ximat+9*i, m->vis.rgba.inertia);

        // glow
        if (pert->select == i) {
          markselected(&m->vis, thisgeom);
        }

        // vopt->label
        if (vopt->label == mjLABEL_BODY ||
            (vopt->label == mjLABEL_SELECTION && pert->select == i)) {
          makeLabel(m, mjOBJ_BODY, i, thisgeom->label);
        }

        FINISH
      }
    }
  }

  // connector to mouse perturbation target
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_PERTOBJ] && (category & catmask) && pert->select > 0) {
    int i = pert->select;

    if ((pert->active | pert->active2) & mjPERT_TRANSLATE) {
      START

      // compute selection point in world coordinates
      mju_mulMatVec3(selpos, d->xmat+9*pert->select, pert->localpos);
      mju_addTo3(selpos, d->xpos+3*pert->select);

      // construct geom
      sz[0] = scl * m->vis.scale.constraint;
      mjv_connector(thisgeom, mjGEOM_CAPSULE, sz[0], selpos, pert->refselpos);

      // prepare color
      float rgba[4];
      mixcolor(rgba, m->vis.rgba.constraint,
               (pert->active & mjPERT_TRANSLATE) > 0,
               (pert->active2 & mjPERT_TRANSLATE) > 0);

      f2f(thisgeom->rgba, rgba, 4);

      FINISH

      // add small sphere at end-effector
      START

      // construct geom
        sz[0] = 2*sz[0];
      sz[1] = sz[2] = sz[0];
      mju_quat2Mat(mat, pert->refquat);
      mjv_initGeom(thisgeom, mjGEOM_SPHERE, sz, pert->refselpos, mat, rgba);

      FINISH
    }

    if ((pert->active | pert->active2) & mjPERT_ROTATE) {
      START

      // prepare color, use inertia color
      float rgba[4];
      mixcolor(rgba, m->vis.rgba.inertia,
               (pert->active & mjPERT_ROTATE) > 0,
               (pert->active2 & mjPERT_ROTATE) > 0);

      // construct geom: if body i has a collision aabb, use that
      mjtNum pos[3] = {0};
      if (m->body_bvhnum[i]) {
        mjtNum* aabb = m->bvh_aabb+6*m->body_bvhadr[i];
        mju_copy3(sz, aabb+3);
        mju_mulMatVec3(pos, d->ximat+9*i, aabb);
      }

      // otherwise box of size meansize
      else {
        sz[0] = sz[1] = sz[2] = scl;
      }
      mju_quat2Mat(mat, pert->refquat);
      mju_addTo3(pos, d->xipos+3*i);
      mjv_initGeom(thisgeom, mjGEOM_BOX, sz, pos, mat, rgba);

      FINISH
    }
  }

  // world and body frame
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if (category & catmask) {
    for (int i = (vopt->frame == mjFRAME_WORLD ? 0 : 1);
         i < (vopt->frame == mjFRAME_BODY ? m->nbody : 1);
         i++) {
      // set length(1) and width(0) of the axis cylinders
      if (i == 0) {
        sz[1] = m->vis.scale.framelength * scl * 2;
        sz[0] = m->vis.scale.framewidth * scl * 2;
      } else {
        sz[1] = m->vis.scale.framelength * scl;
        sz[0] = m->vis.scale.framewidth * scl;
      }

      // skip if body is static and static bodies are masked
      if (i > 0 && bodycategory(m, i) & ~catmask) {
        continue;
      }

      mjtNum* xmat = vopt->flags[mjVIS_INERTIA] ? d->ximat+9*i : d->xmat+9*i;
      mjtNum* xpos = vopt->flags[mjVIS_INERTIA] ? d->xipos+3*i : d->xpos+3*i;

      // draw the three axes (separate geoms)
      for (int j=0; j < 3; j++) {
        START

        // prepare axis
        for (int k=0; k < 3; k++) {
          axis[k] = (j == k ? sz[1] : 0);
        }
        mju_mulMatVec(vec, xmat, axis, 3, 3);

        // create a cylinder
        mjtNum* from = xpos;
        mjtNum to[3];
        mju_add3(to, from, vec);
        mjv_connector(thisgeom, mjGEOM_CYLINDER, sz[0], from, to);

        // set color: R, G or B depending on axis
        for (int k=0; k < 3; k++) {
          thisgeom->rgba[k] = (j == k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }
  }

  // selection point
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if ((category & catmask) && pert->select > 0 && vopt->flags[mjVIS_SELECT]) {
    int i=0;

    // compute selection point in world coordinates
    mju_mulMatVec3(selpos, d->xmat+9*pert->select, pert->localpos);
    mju_addTo3(selpos, d->xpos+3*pert->select);

    START
    thisgeom->type = mjGEOM_SPHERE;
    thisgeom->size[0] = thisgeom->size[1] = thisgeom->size[2] = scl * m->vis.scale.selectpoint;
    mju_n2f(thisgeom->pos, selpos, 3);
    mju_n2f(thisgeom->mat, IDENTITY, 9);
    f2f(thisgeom->rgba, m->vis.rgba.selectpoint, 4);
    if (vopt->label == mjLABEL_SELPNT) {
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
  if ((category & catmask) && (vopt->label == mjLABEL_SELECTION || vopt->label == mjLABEL_BODY) &&
      !vopt->flags[mjVIS_INERTIA]) {
    for (int i=1; i < m->nbody; i++) {
      if (vopt->label == mjLABEL_BODY || (vopt->label == mjLABEL_SELECTION && pert->select == i)) {
        // skip if body is static and static bodies are masked
        if (bodycategory(m, i) & ~catmask) {
          continue;
        }
        START

        // construct geom
        thisgeom->type = mjGEOM_LABEL;
        mju_n2f(thisgeom->pos, d->xpos+3*i, 3);
        mju_n2f(thisgeom->mat, d->xmat+9*i, 9);

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
    for (int i=0; i < m->njnt; i++) {
      if (vopt->jointgroup[mjMAX(0, mjMIN(mjNGROUP-1, m->jnt_group[i]))]) {
        // set length(1) and width(0) of the connectors
        sz[1] = m->vis.scale.jointlength * scl;
        sz[0] = m->vis.scale.jointwidth * scl;

        START

        // set type, size, pos, mat depending on joint type
        int j = m->jnt_bodyid[i];
        mjtNum* from;
        mjtNum to[3];
        switch ((mjtJoint) m->jnt_type[i]) {
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
          from = d->xanchor+3*i;
          mju_addScl3(to, from, d->xaxis+3*i, sz[1]);
          mjv_connector(thisgeom, m->jnt_type[i] == mjJNT_SLIDE ? mjGEOM_ARROW : mjGEOM_ARROW1,
                        sz[0], from, to);
          break;

        default:
          mjERROR("unknown joint type %d", m->jnt_type[i]);
        }

        f2f(thisgeom->rgba, m->vis.rgba.joint, 4);

        // vopt->label
        if (vopt->label == mjLABEL_JOINT) {
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
    for (int i=0; i < m->nu; i++) {
      if (vopt->actuatorgroup[mjMAX(0, mjMIN(mjNGROUP-1, m->actuator_group[i]))]) {
        // skip if disabled
        if (mj_actuatorDisabled(m, i)) {
          continue;
        }

        // determine extended range
        mjtNum rng[3] = {-1, 0, +1};
        mjtNum rmin = -1, rmax = 1, act = 0;
        if (m->actuator_ctrllimited[i]) {
          rmin = m->actuator_ctrlrange[2*i];
          rmax = m->actuator_ctrlrange[2*i+1];
        } else if (vopt->flags[mjVIS_ACTIVATION] && m->actuator_actlimited[i]) {
          rmin = m->actuator_actrange[2*i];
          rmax = m->actuator_actrange[2*i+1];
        }
        if (rmin >= 0) {
          rng[0] = -1;
          rng[1] = rmin;
          rng[2] = rmax;
        } else if (rmax <= 0) {
          rng[0] = rmin;
          rng[1] = rmax;
          rng[2] = +1;
        } else {
          rng[0] = rmin;
          rng[1] = 0;
          rng[2] = rmax;
        }

        // adjust small ranges
        if (rng[1]-rng[0] < mjMINVAL) {
          rng[0] = rng[1] - mjMINVAL;
        }
        if (rng[2]-rng[1] < mjMINVAL) {
          rng[2] = rng[1] + mjMINVAL;
        }

        // clamp act to extended range
        if (vopt->flags[mjVIS_ACTIVATION] && m->actuator_dyntype[i]) {
          act = mju_clip(d->act[m->actuator_actadr[i] + m->actuator_actnum[i] - 1], rng[0], rng[2]);
        } else {
          act = mju_clip(d->ctrl[i], rng[0], rng[2]);
        }

        // compute interpolants
        float amin, amean, amax;
        if (act <= rng[1]) {
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
        for (int j=0; j < 4; j++) {
          rgba[j] = amin*m->vis.rgba.actuatornegative[j] +
                    amean*m->vis.rgba.actuator[j] +
                    amax*m->vis.rgba.actuatorpositive[j];
        }

        // get transmission object id
        int j = m->actuator_trnid[2*i];

        // slide and hinge joint actuators
        if (m->actuator_trntype[i] == mjTRN_JOINT ||
            m->actuator_trntype[i] == mjTRN_JOINTINPARENT ||
            m->actuator_trntype[i] == mjTRN_SITE) {
          START

          // site actuators
          if (m->actuator_trntype[i] == mjTRN_SITE) {
            // inflate sizes by 5%
            mju_scl3(sz, m->site_size+3*j, 1.05);

            // make geom
            mjv_initGeom(thisgeom,
                         m->site_type[j], sz,
                         d->site_xpos + 3*j,
                         d->site_xmat + 9*j,
                         thisgeom->rgba);
          } else if (m->jnt_type[j] == mjJNT_HINGE || m->jnt_type[j] == mjJNT_SLIDE) {
            // set length(1) and width(0) of the connectors
            sz[1] = m->vis.scale.actuatorlength * scl;
            sz[0] = m->vis.scale.actuatorwidth * scl;

            // make geom
            mjtNum* from = d->xanchor + 3*j;
            mjtNum to[3];
            mju_addScl3(to, from, d->xaxis+3*j, sz[1]);
            mjv_connector(thisgeom, m->jnt_type[j] == mjJNT_SLIDE ? mjGEOM_ARROW : mjGEOM_ARROW1,
                          sz[0], from, to);
          }

          // ball or free joint
          else if (m->jnt_type[j] == mjJNT_BALL || m->jnt_type[j] == mjJNT_FREE) {
            sz[0] = sz[1] = sz[2] = m->vis.scale.jointlength * scl * 0.33;

            // make geom
            mjv_initGeom(thisgeom,
                         m->jnt_type[j] == mjJNT_BALL ? mjGEOM_SPHERE : mjGEOM_BOX, sz,
                         d->xanchor + 3*j,
                         d->xmat + 9*m->jnt_bodyid[j],
                         thisgeom->rgba);
          }

          // set interpolated color
          f2f(thisgeom->rgba, rgba, 4);

          // vopt->label
          if (vopt->label == mjLABEL_ACTUATOR) {
            makeLabel(m, mjOBJ_ACTUATOR, i, thisgeom->label);
          }

          FINISH
        }

        // body actuators
        else if (m->actuator_trntype[i] == mjTRN_BODY) {
          // iterate over body's geoms
          int geomnum = m->body_geomnum[j];
          int geomadr = m->body_geomadr[j];
          for (int k=geomadr; k < geomadr+geomnum; k++) {
            int geomtype = m->geom_type[k];
            // add inflated geom if it is a regular primitive
            if (geomtype != mjGEOM_PLANE && geomtype != mjGEOM_HFIELD &&
                geomtype != mjGEOM_MESH  && geomtype != mjGEOM_SDF) {
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
        else if (m->actuator_trntype[i] == mjTRN_TENDON && d->ten_wrapnum[j]) {
          for (int k=d->ten_wrapadr[j]; k < d->ten_wrapadr[j]+d->ten_wrapnum[j]-1; k++) {
            if (d->wrap_obj[k] != -2 && d->wrap_obj[k+1] != -2) {
              START

              // determine width: smaller for segments inside wrapping objects
              if (d->wrap_obj[k] >= 0 && d->wrap_obj[k+1] >= 0) {
                sz[0] = 0.5 * m->tendon_width[j];
              } else {
                sz[0] = m->tendon_width[j];
              }

              // increase width for actuator
              sz[0] *= m->vis.map.actuatortendon;

              // construct geom
              mjv_connector(thisgeom, mjGEOM_CAPSULE, sz[0], d->wrap_xpos+3*k, d->wrap_xpos+3*k+3);

              // set material if given
              setMaterial(m, thisgeom, m->tendon_matid[j], m->tendon_rgba+4*j, vopt->flags);

              // set interpolated color
              f2f(thisgeom->rgba, rgba, 4);

              // vopt->label: only the first segment
              if (vopt->label == mjLABEL_ACTUATOR && k == d->ten_wrapadr[j]) {
                makeLabel(m, mjOBJ_ACTUATOR, i, thisgeom->label);
              }

              FINISH
            }
          }
        }
      }
    }
  }

  // island labels
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if ((category & catmask) && (vopt->label == mjLABEL_ISLAND) && d->nisland) {
    for (int i=1; i < m->nbody; i++) {
      int weld_id = m->body_weldid[i];
      if (m->body_dofnum[weld_id]) {
        int islandid = d->dof_island[m->body_dofadr[weld_id]];
        if (islandid > -1) {
          START

          thisgeom->type = mjGEOM_LABEL;
          mju_n2f(thisgeom->pos, d->xipos+3*i, 3);
          mju_n2f(thisgeom->mat, d->ximat+9*i, 9);
          mjSNPRINTF(thisgeom->label, "%d", islandid);

          FINISH
        }
      }
    }
  }

  // geom
  int planeid = -1;
  for (int i=0; i < m->ngeom; i++) {
    // count planes, put current plane number in geom->dataid
    if (m->geom_type[i] == mjGEOM_PLANE) {
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
    int geomgroup = mjMAX(0, mjMIN(mjNGROUP-1, m->geom_group[i]));

    if (vopt->geomgroup[geomgroup]) {
      START

      // construct geom
      mjv_initGeom(thisgeom, m->geom_type[i], m->geom_size+3*i,
                   d->geom_xpos+3*i, d->geom_xmat+9*i, NULL);
      thisgeom->dataid = m->geom_dataid[i];

      // copy rbound from model
      thisgeom->modelrbound = (float)m->geom_rbound[i];

      // set material properties, override if visualizing islands
      float* rgba = m->geom_rgba+4*i;
      float rgba_island[4] = {.5, .5, .5, 1};
      int geom_matid = m->geom_matid[i];
      if (vopt->flags[mjVIS_ISLAND] && d->nisland) {
        geom_matid = -1;
        rgba = rgba_island;
        int weld_id = m->body_weldid[m->geom_bodyid[i]];
        if (m->body_dofnum[weld_id]) {
          int island = d->dof_island[m->body_dofadr[weld_id]];
          if (island > -1) {
            // color using island's first dof
            islandColor(rgba_island, d->island_dofadr[island]);
          }
        }
      }
      setMaterial(m, thisgeom, geom_matid, rgba, vopt->flags);

      // set texcoord
      if ((m->geom_type[i] == mjGEOM_MESH || m->geom_type[i] == mjGEOM_SDF) &&
          m->geom_dataid[i] >= 0 &&
          m->mesh_texcoordadr[m->geom_dataid[i]] >= 0) {
        thisgeom->texcoord = 1;
      }

      // skip if alpha is 0
      if (thisgeom->rgba[3] == 0) {
        continue;
      }

      // glow geoms of selected body
      if (pert->select > 0 && pert->select == m->geom_bodyid[i]) {
        markselected(&m->vis, thisgeom);
      }

      // vopt->label
      if (vopt->label == mjLABEL_GEOM) {
        makeLabel(m, mjOBJ_GEOM, i, thisgeom->label);
      }

      // mesh: 2*i is original, 2*i+1 is convex hull
      if (m->geom_type[i] == mjGEOM_MESH || m->geom_type[i] == mjGEOM_SDF) {
        thisgeom->dataid *= 2;
        if (m->mesh_graphadr[m->geom_dataid[i]] >= 0 && vopt->flags[mjVIS_CONVEXHULL] &&
            (m->geom_contype[i] || m->geom_conaffinity[i])) {
          thisgeom->dataid += 1;
        }
      }

      // plane
      else if (m->geom_type[i] == mjGEOM_PLANE) {
        // use current planeid
        thisgeom->dataid = planeid;

        // save initial pos
        mju_copy3(tmp, d->geom_xpos+3*i);

        // re-center infinite plane
        if (m->geom_size[3*i] <= 0 || m->geom_size[3*i+1] <= 0) {
          // vec = headpos - geompos
          for (int j=0; j < 3; j++) {
            vec[j] = 0.5*(scn->camera[0].pos[j] + scn->camera[1].pos[j]) - d->geom_xpos[3*i+j];
          }

          // construct axes
          mjtNum ax[9];
          mju_transpose(ax, d->geom_xmat+9*i, 3, 3);

          // loop over (x,y)
          for (int k=0; k < 2; k++) {
            if (m->geom_size[3*i+k] <= 0) {
              // compute zfar
              mjtNum zfar = m->vis.map.zfar * m->stat.extent;

              // get size increment
              mjtNum sX;
              int matid = m->geom_matid[i];
              if (matid >= 0 && m->mat_texrepeat[2*matid+k] > 0) {
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
      if (!(category & catmask) || vopt->frame != mjFRAME_GEOM) {
        continue;
      }

      // construct geom frame
      objtype = mjOBJ_UNKNOWN;
      sz[0] = m->vis.scale.framewidth * scl;
      sz[1] = m->vis.scale.framelength * scl;
      for (int j=0; j < 3; j++) {
        START

        // prepare axis
        for (int k=0; k < 3; k++) {
          axis[k] = (j == k ? sz[1] : 0);
        }
        mju_mulMatVec(vec, d->geom_xmat+9*i, axis, 3, 3);

        // create a cylinder
        mjtNum* from = d->geom_xpos+3*i;
        mjtNum to[3];
        mju_add3(to, from, vec);
        mjv_connector(thisgeom, mjGEOM_CYLINDER, sz[0], from, to);

        // set color: R, G or B depending on axis
        for (int k=0; k < 3; k++) {
          thisgeom->rgba[k] = (j == k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }
  }

  // site
  for (int i=0; i < m->nsite; i++) {
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
      if (thisgeom->rgba[3] == 0) {
        continue;
      }

      // glow
      if (pert->select > 0 && pert->select == m->site_bodyid[i]) {
        markselected(&m->vis, thisgeom);
      }

      // vopt->label
      if (vopt->label == mjLABEL_SITE) {
        makeLabel(m, mjOBJ_SITE, i, thisgeom->label);
      }

      FINISH

      // set category for site frame
      category = mjCAT_DECOR;
      if (!(category & catmask) || vopt->frame != mjFRAME_SITE) {
        continue;
      }

      // construct site frame
      objtype = mjOBJ_UNKNOWN;
      sz[0] = m->vis.scale.framewidth * scl;
      sz[1] = m->vis.scale.framelength * scl;
      for (int j=0; j < 3; j++) {
        START

        // prepare axis
        for (int k=0; k < 3; k++) {
          axis[k] = (j == k ? sz[1] : 0);
        }
        mju_mulMatVec(vec, d->site_xmat+9*i, axis, 3, 3);

        // create a cylinder
        mjtNum* from = d->site_xpos+3*i;
        mjtNum to[3];
        mju_add3(to, from, vec);
        mjv_connector(thisgeom, mjGEOM_CYLINDER, sz[0], from, to);

        // set color: R, G or B depending on axis
        for (int k=0; k < 3; k++) {
          thisgeom->rgba[k] = (j == k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }
  }

  // cameras and frustums
  objtype = mjOBJ_CAMERA;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_CAMERA] && (category & catmask)) {
    for (int i=0; i < m->ncam; i++) {
      // copy camera rgba
      float cam_rgba[4];
      f2f(cam_rgba, m->vis.rgba.camera, 4);

      // draw frustum if sensorsize is defined
      if (m->cam_sensorsize[2*i+1] > 0) {
        // when drawing frustum, make camera translucent
        cam_rgba[3] = 0.3;

        // locals
        const float* rgba = m->vis.rgba.frustum;
        mjtNum vnear[4][3], vfar[4][3];
        mjtNum center[3];
        mjtNum znear = m->vis.map.znear * m->stat.extent;
        mjtNum zfar = m->vis.scale.frustum * scl;
        float zver[2], zhor[2];

        // get frustum
        getFrustum(zver, zhor, znear, m->cam_intrinsic + 4*i, m->cam_sensorsize + 2*i);

        // frustum frame to convert from planes to vertex representation
        mjtNum *cam_xpos = d->cam_xpos+3*i;
        mjtNum *cam_xmat = d->cam_xmat+9*i;
        mjtNum x[] = {cam_xmat[0], cam_xmat[3], cam_xmat[6]};
        mjtNum y[] = {cam_xmat[1], cam_xmat[4], cam_xmat[7]};
        mjtNum z[] = {cam_xmat[2], cam_xmat[5], cam_xmat[8]};

        // vertices of the near plane
        mju_addScl3(center, cam_xpos, z, -znear);
        mju_addScl3(vnear[0], center, x, -zhor[0]);
        mju_addScl3(vnear[1], center, x,  zhor[1]);
        mju_addScl3(vnear[2], center, x,  zhor[1]);
        mju_addScl3(vnear[3], center, x, -zhor[0]);
        mju_addToScl3(vnear[0], y, -zver[0]);
        mju_addToScl3(vnear[1], y, -zver[0]);
        mju_addToScl3(vnear[2], y,  zver[1]);
        mju_addToScl3(vnear[3], y,  zver[1]);

        // vertices of the far plane
        zhor[0] *= zfar / znear;
        zhor[1] *= zfar / znear;
        zver[0] *= zfar / znear;
        zver[1] *= zfar / znear;
        mju_addScl3(center, cam_xpos, z, -zfar);
        mju_addScl3(vfar[0], center, x, -zhor[0]);
        mju_addScl3(vfar[1], center, x,  zhor[1]);
        mju_addScl3(vfar[2], center, x,  zhor[1]);
        mju_addScl3(vfar[3], center, x, -zhor[0]);
        mju_addToScl3(vfar[0], y, -zver[0]);
        mju_addToScl3(vfar[1], y, -zver[0]);
        mju_addToScl3(vfar[2], y,  zver[1]);
        mju_addToScl3(vfar[3], y,  zver[1]);

        // triangulation and wireframe of the frustum
        for (int e=0; e < 4; e++) {
          START
          makeTriangle(thisgeom, vnear[e], vfar[e], vnear[(e+1)%4], rgba);
          FINISH
          START
          makeTriangle(thisgeom, vfar[e], vfar[(e+1)%4], vnear[(e+1)%4], rgba);
          FINISH
          START
          mjv_connector(thisgeom, mjGEOM_LINE, 3, vnear[e], vnear[(e+1)%4]);
          f2f(thisgeom->rgba, rgba, 4);
          FINISH
          START
          mjv_connector(thisgeom, mjGEOM_LINE, 3, vfar[e], vfar[(e+1)%4]);
          f2f(thisgeom->rgba, rgba, 4);
          FINISH
          START
          mjv_connector(thisgeom, mjGEOM_LINE, 3, vnear[e], vfar[e]);
          f2f(thisgeom->rgba, rgba, 4);
          FINISH
        }
      }

      START

      // construct geom: camera body
      thisgeom->type = mjGEOM_BOX;
      thisgeom->size[0] = scl * m->vis.scale.camera * 1.0;
      thisgeom->size[1] = scl * m->vis.scale.camera * 0.8;
      thisgeom->size[2] = scl * m->vis.scale.camera * 0.4;
      mju_n2f(thisgeom->pos, d->cam_xpos+3*i, 3);
      mju_n2f(thisgeom->mat, d->cam_xmat+9*i, 9);
      f2f(thisgeom->rgba, cam_rgba, 4);

      // vopt->label
      if (vopt->label == mjLABEL_CAMERA) {
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
      f2f(thisgeom->rgba, cam_rgba, 4);
      for (int k=0; k < 3; k++) {
        thisgeom->rgba[k] *= 0.5;  // make lens body darker
      }

      FINISH

      // set category for camera frame
      category = mjCAT_DECOR;
      if (!(category & catmask) || vopt->frame != mjFRAME_CAMERA) {
        continue;
      }

      // construct camera frame
      objtype = mjOBJ_UNKNOWN;
      sz[0] = m->vis.scale.framewidth * scl;
      sz[1] = m->vis.scale.framelength * scl;
      for (int j=0; j < 3; j++) {
        START

        // prepare axis
        for (int k=0; k < 3; k++) {
          axis[k] = (j == k ? sz[1] : 0);
        }
        mju_mulMatVec(vec, d->cam_xmat+9*i, axis, 3, 3);

        // create a cylinder
        mjtNum* from = d->cam_xpos+3*i;
        mjtNum to[3];
        mju_add3(to, from, vec);
        mjv_connector(thisgeom, mjGEOM_CYLINDER, sz[0], from, to);

        // set color: R, G or B depending on axis
        for (int k=0; k < 3; k++) {
          thisgeom->rgba[k] = (j == k ? 0.9 : 0);
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
    for (int i=0; i < m->nlight; i++) {
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
      if (vopt->label == mjLABEL_LIGHT) {
        makeLabel(m, mjOBJ_LIGHT, i, thisgeom->label);
      }

      FINISH

      // set category for light frame
      category = mjCAT_DECOR;
      if (!(category & catmask) || vopt->frame != mjFRAME_LIGHT) {
        continue;
      }

      // construct light frame
      objtype = mjOBJ_UNKNOWN;
      sz[0] = m->vis.scale.framewidth * scl;
      sz[1] = m->vis.scale.framelength * scl;
      for (int j=0; j < 3; j++) {
        START

        // prepare axis
        for (int k=0; k < 3; k++) {
          axis[k] = (j == k ? sz[1] : 0);
        }
        mju_mulMatVec(vec, mat, axis, 3, 3);

        // create a cylinder
        mjtNum* from = d->light_xpos+3*i;
        mjtNum to[3];
        mju_add3(to, from, vec);
        mjv_connector(thisgeom, mjGEOM_CYLINDER, sz[0], from, to);

        // set color: R, G or B depending on axis
        for (int k=0; k < 3; k++) {
          thisgeom->rgba[k] = (j == k ? 0.9 : 0);
        }
        thisgeom->rgba[3] = 1;

        FINISH
      }
    }
  }

  // spatial tendons
  objtype = mjOBJ_TENDON;
  category = mjCAT_DYNAMIC;
  if (vopt->flags[mjVIS_TENDON] && (category & catmask) && m->ntendon) {
    // mark actuated tendons
    mj_markStack(d);
    int* tendon_actuated = mjSTACKALLOC(d, m->ntendon, int);
    mju_zeroInt(tendon_actuated, m->ntendon);
    for (int i=0; i < m->nu; i++) {
      if (m->actuator_trntype[i] == mjTRN_TENDON) {
        tendon_actuated[m->actuator_trnid[2*i]] = 1;
      }
    }

    // draw tendons
    for (int i=0; i < m->ntendon; i++) {
      if (vopt->tendongroup[mjMAX(0, mjMIN(mjNGROUP-1, m->tendon_group[i]))]) {
        // tendon has a deadband spring
        int limitedspring =
          m->tendon_stiffness[i] > 0            &&    // positive stiffness
          m->tendon_lengthspring[2*i] == 0      &&    // range lower-bound is 0
          m->tendon_lengthspring[2*i+1] > 0;          // range upper-bound is positive

        // tendon has a simple length constraint, but is currently not limited
        mjtNum ten_length = d->ten_length[i];
        mjtNum lower = m->tendon_range[2*i];
        mjtNum upper = m->tendon_range[2*i + 1];
        int limitedconstraint =
          m->tendon_stiffness[i] == 0           &&    // zero stiffness
          m->tendon_limited[i] == 1             &&    // limited length range
          lower == 0                            &&    // range lower-bound is 0
          ten_length < upper;                         // current length is smaller than upper bound

        // conditions for drawing a catenary
        int draw_catenary =
          !mjDISABLED(mjDSBL_GRAVITY)           &&    // gravity enabled
          mju_norm3(m->opt.gravity) > mjMINVAL  &&    // gravity strictly nonzero
          m->tendon_num[i] == 2                 &&    // only two sites on the tendon
          (limitedspring != limitedconstraint)  &&    // either spring or constraint length limits
          m->tendon_damping[i] == 0             &&    // no damping
          m->tendon_frictionloss[i] == 0        &&    // no frictionloss
          tendon_actuated[i] == 0;                    // no actuator

        // conditions not met: draw straight lines
        if (!draw_catenary) {
          for (int j=d->ten_wrapadr[i]; j < d->ten_wrapadr[i]+d->ten_wrapnum[i]-1; j++) {
            if (d->wrap_obj[j] != -2 && d->wrap_obj[j+1] != -2) {
              START

              // determine width: smaller for segments inside wrapping objects
              if (d->wrap_obj[j] >= 0 && d->wrap_obj[j+1] >= 0) {
                sz[0] = 0.5 * m->tendon_width[i];
              } else {
                sz[0] = m->tendon_width[i];
              }

              // construct geom
              mjv_connector(thisgeom, mjGEOM_CAPSULE, sz[0], d->wrap_xpos+3*j, d->wrap_xpos+3*j+3);

              // set material properties, override if visualizing islands
              float* rgba = m->tendon_rgba+4*i;
              float rgba_island[4] = {.5, .5, .5, 1};
              int tendon_matid = m->tendon_matid[i];
              if (vopt->flags[mjVIS_ISLAND] && d->nisland) {
                tendon_matid = -1;
                rgba = rgba_island;
                if (d->tendon_efcadr[i] != -1) {
                  // set color using island's first dof
                  int island = d->efc_island[d->tendon_efcadr[i]];
                  islandColor(rgba_island, d->island_dofadr[island]);
                }
              }
              setMaterial(m, thisgeom, tendon_matid, rgba, vopt->flags);

              // vopt->label: only the first segment
              if (vopt->label == mjLABEL_TENDON && j == d->ten_wrapadr[i]) {
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
          mjtNum length;
          if (limitedconstraint) {
            length = m->tendon_range[2*i+1];
          } else {
            length = m->tendon_lengthspring[2*i+1];
          }

          // get number of points along catenary path
          int ncatenary = m->vis.quality.numslices + 1;

          // allocate catenary
          mjtNum* catenary = mjSTACKALLOC(d, 3*ncatenary, mjtNum);

          // points along catenary path
          int npoints = mjv_catenary(x0, x1, m->opt.gravity, length, catenary, ncatenary);

          // draw npoints-1 segments
          for (int j=0; j < npoints-1; j++) {
            START

            sz[0] = m->tendon_width[i];

            // construct geom
            mjv_connector(thisgeom, mjGEOM_CAPSULE, sz[0], catenary+3*j, catenary+3*j+3);

            // set material if given
            setMaterial(m, thisgeom, m->tendon_matid[i], m->tendon_rgba+4*i, vopt->flags);

            // vopt->label: only the first segment
            if (vopt->label == mjLABEL_TENDON && npoints/2) {
              makeLabel(m, mjOBJ_TENDON, i, thisgeom->label);
            }

            FINISH
          }
        }
      }
    }
    mj_freeStack(d);
  }

  // slider-crank
  objtype = mjOBJ_ACTUATOR;
  category = mjCAT_DYNAMIC;
  if ((category & catmask)) {
    for (int i=0; i < m->nu; i++) {
      if (m->actuator_trntype[i] == mjTRN_SLIDERCRANK) {
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
        if (det < 0) {
          det = 0;
          broken = 1;
        }
        len = len - mju_sqrt(det);

        // compute slider endpoint
        mju_scl3(end, axis, len);
        mju_addTo3(end, d->site_xpos+3*k);

        // render slider
        START
        mjv_connector(thisgeom, mjGEOM_CYLINDER, scl * m->vis.scale.slidercrank,
                      d->site_xpos+3*k, end);
        f2f(thisgeom->rgba, m->vis.rgba.slidercrank, 4);
        if (vopt->label == mjLABEL_ACTUATOR) {
          makeLabel(m, mjOBJ_ACTUATOR, i, thisgeom->label);
        }
        FINISH

        // render crank
        START
        mjv_connector(thisgeom, mjGEOM_CAPSULE, scl * m->vis.scale.slidercrank/2.0,
                      end, d->site_xpos+3*j);
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
    for (int i=1; i < m->nbody; i++) {
      if (m->body_rootid[i] == i) {
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
    for (int i=1; i < m->nbody; i++) {
      // do not connect to world
      if (m->body_parentid[i] == 0) {
        continue;
      }

      // start at body com, connect joint centers in reverse order
      cur = d->xipos+3*i;
      if (m->body_jntnum[i]) {
        for (int j=m->body_jntadr[i]+m->body_jntnum[i]-1; j >= m->body_jntadr[i]; j--) {
          START
            nxt = d->xanchor+3*j;

          // construct geom
          mjv_connector(thisgeom, mjGEOM_CAPSULE, scl * m->vis.scale.connect, cur, nxt);
          f2f(thisgeom->rgba, m->vis.rgba.connect, 4);

          FINISH
            cur = nxt;
        }
      }

      // connect first joint (or com) to parent com
      START
        nxt = d->xipos+3*m->body_parentid[i];
      mjv_connector(thisgeom, mjGEOM_CAPSULE, scl * m->vis.scale.connect, cur, nxt);
      f2f(thisgeom->rgba, m->vis.rgba.connect, 4);
      FINISH
    }
  }

  // rangefinders
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  if (vopt->flags[mjVIS_RANGEFINDER] && (category & catmask)) {
    for (int i=0; i < m->nsensor; i++) {
      if (m->sensor_type[i] == mjSENS_RANGEFINDER) {
        // sensor data
        mjtNum dst = d->sensordata[m->sensor_adr[i]];
        int sid = m->sensor_objid[i];

        // null output: nothing to render
        if (dst < 0) {
          continue;
        }

        // make ray
        START
        mjtNum* from = d->site_xpos+3*sid;
        mjtNum to[3] = {from[0] + d->site_xmat[9*sid+2]*dst,
                        from[1] + d->site_xmat[9*sid+5]*dst,
                        from[2] + d->site_xmat[9*sid+8]*dst};
        mjv_connector(thisgeom, mjGEOM_LINE, 3, from, to);
        f2f(thisgeom->rgba, m->vis.rgba.rangefinder, 4);
        FINISH
      } else if (m->sensor_type[i] == mjSENS_GEOMFROMTO) {
        // sensor data
        mjtNum* fromto = d->sensordata + m->sensor_adr[i];

        // null output: nothing to render
        if (mju_isZero(fromto, 6)) {
          continue;
        }

        // make ray
        START
        mjv_connector(thisgeom, mjGEOM_LINE, 3, fromto, fromto+3);
        f2f(thisgeom->rgba, m->vis.rgba.rangefinder, 4);
        FINISH
      }
    }
  }

  // external perturbations
  objtype = mjOBJ_UNKNOWN;
  category = mjCAT_DECOR;
  for (int i=1; i < m->nbody; i++) {
    if (!mju_isZero(d->xfrc_applied+6*i, 6) && (category & catmask)) {
      // point of application and force
      mjtNum *xpos = d->xipos+3*i;
      xfrc = d->xfrc_applied+6*i;

      // force perturbation
      if (vopt->flags[mjVIS_PERTFORCE] && mju_norm3(xfrc) > mjMINVAL) {
        // map force to spatial vector in world frame
        mju_scl3(vec, xfrc, m->vis.map.force/m->stat.meanmass);

        START
        mjtNum* from = xpos;
        mjtNum to[3];
        mju_add3(to, from, vec);
        mjv_connector(thisgeom, mjGEOM_ARROW, m->vis.scale.forcewidth * scl, from, to);
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
    for (int i=0; i < m->neq; i++) {
      int is_weld = m->eq_type[i] == mjEQ_WELD;
      int is_connect = m->eq_type[i] == mjEQ_CONNECT;
      if (d->eq_active[i] && (is_connect || is_weld)) {
        // compute endpoints in global coordinates
        mjtNum *xmat_j, *xmat_k;
        int j = m->eq_obj1id[i], k = m->eq_obj2id[i];
        if (m->eq_objtype[i] == mjOBJ_SITE) {
          mju_copy3(vec, d->site_xpos+3*j);
          mju_copy3(end, d->site_xpos+3*k);
          xmat_j = d->site_xmat+9*j;
          xmat_k = d->site_xmat+9*k;
        } else {
          mju_mulMatVec3(vec, d->xmat+9*j, m->eq_data+mjNEQDATA*i+3*is_weld);
          mju_addTo3(vec, d->xpos+3*j);
          mju_mulMatVec3(end, d->xmat+9*k, m->eq_data+mjNEQDATA*i+3*is_connect);
          mju_addTo3(end, d->xpos+3*k);
          xmat_j = d->xmat+9*j;
          xmat_k = d->xmat+9*k;
        }

        // construct geom
        sz[0] = scl * m->vis.scale.constraint;

        START
        mjv_initGeom(thisgeom, mjGEOM_SPHERE, sz, vec, xmat_j, m->vis.rgba.connect);
        if (vopt->label == mjLABEL_CONSTRAINT) {
          makeLabel(m, mjOBJ_EQUALITY, i, thisgeom->label);
        }
        FINISH

        START
        mjv_initGeom(thisgeom, mjGEOM_SPHERE, sz, end, xmat_k, m->vis.rgba.constraint);
        if (vopt->label == mjLABEL_CONSTRAINT) {
          makeLabel(m, mjOBJ_EQUALITY, i, thisgeom->label);
        }
        FINISH
      }
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
void mjv_makeLights(const mjModel* m, const mjData* d, mjvScene* scn) {
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
    thislight->texid = -1;
    thislight->type = mjLIGHT_DIRECTIONAL;
    thislight->castshadow = 0;
    thislight->bulbradius = 0.02;
    thislight->intensity = 0;
    thislight->range = 10;

    // compute head position and gaze direction in model space
    mjtNum hpos[3], hfwd[3];
    mjv_cameraInModel(hpos, hfwd, NULL, scn);
    mju_n2f(thislight->pos, hpos, 3);
    mju_n2f(thislight->dir, hfwd, 3);

    // copy colors
    f2f(thislight->ambient, m->vis.headlight.ambient, 3);
    f2f(thislight->diffuse, m->vis.headlight.diffuse, 3);
    f2f(thislight->specular, m->vis.headlight.specular, 3);

    // advance counter
    scn->nlight++;
  }

  // remaining lights
  for (int i=0; i < m->nlight && scn->nlight < mjMAXLIGHT; i++) {
    if (m->light_active[i]) {
      // get pointer
      thislight = scn->lights + scn->nlight;

      // copy properties
      memset(thislight, 0, sizeof(mjvLight));
      thislight->type = m->light_type[i];
      thislight->texid = m->light_texid[i];
      thislight->castshadow = m->light_castshadow[i];
      thislight->bulbradius = m->light_bulbradius[i];
      thislight->intensity = m->light_intensity[i];
      thislight->range = m->light_range[i];
      if (thislight->type == mjLIGHT_SPOT) {
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
void mjv_updateCamera(const mjModel* m, const mjData* d, mjvCamera* cam, mjvScene* scn) {
  // return if nothing to do
  if (!m || !cam || cam->type == mjCAMERA_USER) {
    return;
  }

  // define extrinsics
  mjtNum move[3];
  mjtNum headpos[3], forward[3], up[3], right[3];

  // define intrinsics
  int cid, orthographic = 0;
  mjtNum fovy, ipd;
  float* intrinsic = NULL;
  float* sensorsize = NULL;

  // get headpos, forward, up, right, ipd, fovy, orthographic, intrinsic
  switch (cam->type) {
  case mjCAMERA_FREE:
  case mjCAMERA_TRACKING:
    // get global ipd
    ipd = m->vis.global.ipd;

    // get orthographic, fovy
    orthographic = m->vis.global.orthographic;
    fovy = m->vis.global.fovy;

    // move lookat for tracking
    if (cam->type == mjCAMERA_TRACKING) {
      // get id and check
      int bid = cam->trackbodyid;
      if (bid < 0 || bid >= m->nbody) {
        mjERROR("track body id is outside valid range");
      }

      // smooth tracking of subtree com
      mju_sub3(move, d->subtree_com + 3*cam->trackbodyid, cam->lookat);
      mju_addToScl3(cam->lookat, move, 0.2);  // constant ???
    }

    // compute frame
    mjtNum ca = mju_cos(cam->azimuth/180.0*mjPI);
    mjtNum sa = mju_sin(cam->azimuth/180.0*mjPI);
    mjtNum ce = mju_cos(cam->elevation/180.0*mjPI);
    mjtNum se = mju_sin(cam->elevation/180.0*mjPI);
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

  case mjCAMERA_FIXED:
    // get id, check range
    cid = cam->fixedcamid;
    if (cid < 0 || cid >= m->ncam) {
      mjERROR("fixed camera id is outside valid range");
    }

    // get camera-specific ipd, orthographic, fovy
    ipd = m->cam_ipd[cid];

    orthographic = m->cam_orthographic[cid];
    fovy = m->cam_fovy[cid];

    // if positive sensorsize, get sensorsize and intrinsic
    if (m->cam_sensorsize[2*cid+1]) {
      sensorsize = m->cam_sensorsize + 2*cid;
      intrinsic = m->cam_intrinsic + 4*cid;
    }

    // get pointer to camera orientation matrix
    mjtNum* mat = d->cam_xmat + 9*cid;

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
    break;

  default:
    mjERROR("unknown camera type");
  }

  // convert intrinsics to frustum parameters
  float znear = m->vis.map.znear * m->stat.extent;
  float zfar = m->vis.map.zfar * m->stat.extent;
  float zver[2], zhor[2] = {0, 0};
  if (orthographic){
    zver[0] = zver[1] = fovy / 2;
  } else {
    if (!intrinsic) {
      zver[0] = zver[1] = znear * mju_tan(fovy * mjPI/360.0);
    } else {
      getFrustum(zver, zhor, znear, intrinsic, sensorsize);
    }
  }

  // compute GL cameras
  for (int view=0; view < 2; view++) {
    // set frame
    for (int i=0; i < 3; i++) {
      scn->camera[view].pos[i] = (float)(headpos[i] + (view ? ipd : -ipd)*0.5*right[i]);
      scn->camera[view].forward[i] = (float)forward[i];
      scn->camera[view].up[i] = (float)up[i];
    }

    // set orthographic
    scn->camera[view].orthographic = orthographic;

    // set symmetric frustum using intrinsic camera matrix
    scn->camera[view].frustum_top = zver[1];
    scn->camera[view].frustum_bottom = -zver[0];
    scn->camera[view].frustum_center = (zhor[1] - zhor[0]) / 2;
    scn->camera[view].frustum_width = (zhor[1] + zhor[0]) / 2;
    scn->camera[view].frustum_near = znear;
    scn->camera[view].frustum_far = zfar;
  }

  // disable model transformation (do not clear float data; user may need it later)
  scn->enabletransform = 0;
}



// construct face, flat normals
static void makeFace(float* _face, float* _normal,  mjtNum radius, const mjtNum* vertxpos,
                     int nface, int i0, int i1, int i2) {
  float* face = _face + 9*nface;
  float* normal = _normal + 9*nface;
  const mjtNum* v0 = vertxpos + 3*i0;
  const mjtNum* v1 = vertxpos + 3*i1;
  const mjtNum* v2 = vertxpos + 3*i2;

  // compute normal
  mjtNum v01[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
  mjtNum v02[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};
  mjtNum nrm[3];
  mju_cross(nrm, v01, v02);
  mju_normalize3(nrm);

  // set vertices: offset by radius*normal
  mjtNum temp[3];
  mju_addScl3(temp, v0, nrm, radius);
  mju_n2f(face, temp, 3);
  mju_addScl3(temp, v1, nrm, radius);
  mju_n2f(face+3, temp, 3);
  mju_addScl3(temp, v2, nrm, radius);
  mju_n2f(face+6, temp, 3);

  // set normals
  mju_n2f(normal, nrm, 3);
  mju_n2f(normal+3, nrm, 3);
  mju_n2f(normal+6, nrm, 3);
}



// add face normal to vertices
static void addNormal(mjtNum* vertnorm, const mjtNum* vertxpos,
                      int i0, int i1, int i2) {
  // compute normal*area
  const mjtNum* v0 = vertxpos + 3*i0;
  const mjtNum* v1 = vertxpos + 3*i1;
  const mjtNum* v2 = vertxpos + 3*i2;
  mjtNum v01[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
  mjtNum v02[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};
  mjtNum nrm[3];
  mju_cross(nrm, v01, v02);
  mju_normalize3(nrm);

  // accumulate at each vertex
  mju_addTo3(vertnorm + 3*i0, nrm);
  mju_addTo3(vertnorm + 3*i1, nrm);
  mju_addTo3(vertnorm + 3*i2, nrm);
}



// construct face, smooth normals
static void makeSmooth(float* _face, float* _normal, mjtNum radius, mjtByte flg_flat,
                       const mjtNum* vertnorm, const mjtNum* vertxpos,
                       int nface, int i0, int i1, int i2) {
  float* face = _face + 9*nface;
  float* normal = _normal + 9*nface;
  int ind[3] = {i0, i1, i2};
  int sign = radius > 0 ? 1 : -1;

  // flat shading
  if (flg_flat) {
    // compute face normal
    const mjtNum* v0 = vertxpos + 3*i0;
    const mjtNum* v1 = vertxpos + 3*i1;
    const mjtNum* v2 = vertxpos + 3*i2;
    mjtNum v01[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
    mjtNum v02[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};
    mjtNum nrm[3];
    mju_cross(nrm, v01, v02);
    mju_normalize3(nrm);

    // set all vertex normals equal to face normal
    for (int k=0; k < 3; k++){
      normal[3*k+0] = (float) (sign*nrm[0]);
      normal[3*k+1] = (float) (sign*nrm[1]);
      normal[3*k+2] = (float) (sign*nrm[2]);
    }
  }

  // smooth shading
  else {
    for (int k=0; k < 3; k++){
      normal[3*k+0] = (float) (sign*vertnorm[3*ind[k]+0]);
      normal[3*k+1] = (float) (sign*vertnorm[3*ind[k]+1]);
      normal[3*k+2] = (float) (sign*vertnorm[3*ind[k]+2]);
    }
  }

  // set positions: vertices offset by radius*normal
  for (int k=0; k < 3; k++){
    face[3*k+0] = (float) (vertxpos[3*ind[k]+0] + radius*vertnorm[3*ind[k]+0]);
    face[3*k+1] = (float) (vertxpos[3*ind[k]+1] + radius*vertnorm[3*ind[k]+1]);
    face[3*k+2] = (float) (vertxpos[3*ind[k]+2] + radius*vertnorm[3*ind[k]+2]);
  }
}



// construct side in 2D face
static void makeSide(float* _face, float* _normal, mjtNum radius,
                     const mjtNum* vertnorm, const mjtNum* vertxpos,
                     int nface, int i0, int i1) {
  float* face = _face + 9*nface;
  float* normal = _normal + 9*nface;

  // compute normal
  const mjtNum* v0 = vertxpos + 3*i0;
  const mjtNum* v1 = vertxpos + 3*i1;
  mjtNum v01[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
  mjtNum nrm[3];
  mju_cross(nrm, v01, vertnorm+3*i1);
  if (radius < 0) {
    mju_scl3(nrm, nrm, -1);
  }
  mju_normalize3(nrm);

  // set normals
  for (int k=0; k < 3; k++){
    normal[3*k+0] = (float) nrm[0];
    normal[3*k+1] = (float) nrm[1];
    normal[3*k+2] = (float) nrm[2];
  }

  // set positions
  int ind[3] = {i0, i1, i1};
  for (int k=0; k < 3; k++){
    mjtNum sign = (k == 1 ? -1 : +1);
    face[3*k+0] = (float) (vertxpos[3*ind[k]+0] + sign*radius*vertnorm[3*ind[k]+0]);
    face[3*k+1] = (float) (vertxpos[3*ind[k]+1] + sign*radius*vertnorm[3*ind[k]+1]);
    face[3*k+2] = (float) (vertxpos[3*ind[k]+2] + sign*radius*vertnorm[3*ind[k]+2]);
  }
}



// copy texcoord for face
static void copyTex(float* dst, const float* src, int nface, int i0, int i1, int i2) {
  if (!dst || !src) {
    return;
  }

  dst[6*nface+0] = src[2*i0];
  dst[6*nface+1] = src[2*i0+1];
  dst[6*nface+2] = src[2*i1];
  dst[6*nface+3] = src[2*i1+1];
  dst[6*nface+4] = src[2*i2];
  dst[6*nface+5] = src[2*i2+1];
}



// update visible flexes only
void mjv_updateActiveFlex(const mjModel* m, mjData* d, mjvScene* scn, const mjvOption* opt) {
  // save flex visualization flags in scene (needed by renderer)
  scn->flexvertopt = opt->flags[mjVIS_FLEXVERT];
  scn->flexedgeopt = opt->flags[mjVIS_FLEXEDGE];
  scn->flexfaceopt = opt->flags[mjVIS_FLEXFACE];
  scn->flexskinopt = opt->flags[mjVIS_FLEXSKIN];

  // convert vertex positions from mjtNum to float
  for (int v=0; v < 3*m->nflexvert; v++) {
    scn->flexvert[v] = (float) d->flexvert_xpos[v];
  }

  // construct faces
  for (int f=0; f < m->nflex; f++) {
    int dim = m->flex_dim[f];
    mjtNum radius = m->flex_radius[f];
    mjtByte flg_flat = m->flex_flatskin[f];
    const mjtNum* vertxpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
    float* face = scn->flexface + 9*scn->flexfaceadr[f];
    float* normal = scn->flexnormal + 9*scn->flexfaceadr[f];
    float* texdst = m->flex_texcoordadr[f] >= 0 ?
                    scn->flextexcoord + 6*scn->flexfaceadr[f] : NULL;
    const float* texsrc = m->flex_texcoordadr[f] >= 0 ?
                          m->flex_texcoord + 2*m->flex_texcoordadr[f] : NULL;

    // 1D, or face and skin disabled: no faces
    if (dim == 1 || (!opt->flags[mjVIS_FLEXFACE] && !opt->flags[mjVIS_FLEXSKIN])) {
      scn->flexfaceused[f] = 0;
    }

    // 2D or 3D face: faces from elements, flat normals, texture
    else if (!opt->flags[mjVIS_FLEXSKIN]) {
      int nface = 0;
      for (int e=0; e < m->flex_elemnum[f]; e++) {
        // in 3D, show only elements in selected layer
        if (dim == 2 || m->flex_elemlayer[m->flex_elemadr[f]+e] == opt->flex_layer) {
          // get element data
          const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
          const int* tdata = m->flex_elemtexcoord + m->flex_elemdataadr[f] + e*(dim+1);

          // triangles: two faces per element
          if (dim == 2) {
            makeFace(face, normal, radius, vertxpos, nface, edata[0], edata[1], edata[2]);
            copyTex(texdst, texsrc, nface, tdata[0], tdata[1], tdata[2]);
            nface++;

            makeFace(face, normal, radius, vertxpos, nface, edata[0], edata[2], edata[1]);
            copyTex(texdst, texsrc, nface, tdata[0], tdata[2], tdata[1]);
            nface++;
          }

          // tetrahedra: four faces per element
          else {
            makeFace(face, normal, radius, vertxpos,
                     nface, edata[0], edata[1], edata[2]);
            copyTex(texdst, texsrc, nface, tdata[0], tdata[1], tdata[2]);
            nface++;

            makeFace(face, normal, radius, vertxpos,
                     nface, edata[0], edata[2], edata[3]);
            copyTex(texdst, texsrc, nface, tdata[0], tdata[2], tdata[3]);
            nface++;

            makeFace(face, normal, radius, vertxpos,
                     nface, edata[0], edata[3], edata[1]);
            copyTex(texdst, texsrc, nface, tdata[0], tdata[3], tdata[1]);
            nface++;

            makeFace(face, normal, radius, vertxpos,
                     nface, edata[1], edata[3], edata[2]);
            copyTex(texdst, texsrc, nface, tdata[1], tdata[3], tdata[2]);
            nface++;
          }
        }
      }

      // save face count
      scn->flexfaceused[f] = nface;
    }

    // 2D or 3D skin: faces from elements (2D) or shells (3D), smooth normals, texture
    else {
      // allocate and clear vertex normals for smoothing
      mj_markStack(d);
      mjtNum* vertnorm = mjSTACKALLOC(d, 3*m->flex_vertnum[f], mjtNum);
      mju_zero(vertnorm, 3*m->flex_vertnum[f]);

      // add vertex normals: top element sides in 2D, shell fragments in 3D
      if (dim == 2) {
        for (int e=0; e < m->flex_elemnum[f]; e++) {
          const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
          addNormal(vertnorm, vertxpos, edata[0], edata[1], edata[2]);
        }
      } else {
        for (int s=0; s < m->flex_shellnum[f]; s++) {
          const int* sdata = m->flex_shell + m->flex_shelldataadr[f] + s*dim;
          addNormal(vertnorm, vertxpos, sdata[0], sdata[1], sdata[2]);
        }
      }

      // normalize vertex normals
      for (int i=0; i < m->flex_vertnum[f]; i++) {
        mju_normalize3(vertnorm+3*i);
      }

      // create faces, offset along smoothed vertex normals, and texcoord
      int nface = 0;
      if (dim == 2) {
        for (int e=0; e < m->flex_elemnum[f]; e++) {
          const int* edata = m->flex_elem + m->flex_elemdataadr[f] + e*(dim+1);
          const int* tdata = m->flex_elemtexcoord + m->flex_elemdataadr[f] + e*(dim+1);
          makeSmooth(face, normal, radius, flg_flat, vertnorm, vertxpos,
                     nface, edata[0], edata[1], edata[2]);
          copyTex(texdst, texsrc, nface, tdata[0], tdata[1], tdata[2]);
          nface++;
          makeSmooth(face, normal, -radius, flg_flat, vertnorm, vertxpos,
                     nface, edata[0], edata[2], edata[1]);
          copyTex(texdst, texsrc, nface, tdata[0], tdata[2], tdata[1]);
          nface++;
        }
      } else {
        for (int s=0; s < m->flex_shellnum[f]; s++) {
          const int* sdata = m->flex_shell + m->flex_shelldataadr[f] + s*dim;
          makeSmooth(face, normal, radius, flg_flat, vertnorm, vertxpos,
                     nface, sdata[0], sdata[1], sdata[2]);
          copyTex(texdst, texsrc, nface, sdata[0], sdata[1], sdata[2]);
          nface++;
        }
      }

      // 2D: close sides using shell fragments
      if (dim == 2) {
        for (int s=0; s < m->flex_shellnum[f]; s++) {
          const int* sdata = m->flex_shell + m->flex_shelldataadr[f] + s*dim;
          makeSide(face, normal, radius, vertnorm, vertxpos,
                   nface, sdata[0], sdata[1]);
          copyTex(texdst, texsrc, nface, sdata[0], sdata[1], sdata[1]);
          nface++;
          makeSide(face, normal, -radius, vertnorm, vertxpos,
                   nface, sdata[1], sdata[0]);
          copyTex(texdst, texsrc, nface, sdata[1], sdata[0], sdata[0]);
          nface++;
        }
      }

      // save face count
      scn->flexfaceused[f] = nface;
      mj_freeStack(d);
    }

    // check face count, SHOULD NOT OCCUR
    if (scn->flexfaceused[f] > scn->flexfacenum[f]) {
      mju_error("too many flex faces in mjv_updateActiveFlex");
    }
  }
}



// update all skins, here for backward API compatibility
void mjv_updateSkin(const mjModel* m, const mjData* d, mjvScene* scn) {
  mjvOption opt;
  mjv_defaultOption(&opt);
  mjv_updateActiveSkin(m, d, scn, &opt);
  mju_warning("mjv_updateSkin is deprecated, please use mjv_updateActiveSkin.");
}



// update visible skins only
void mjv_updateActiveSkin(const mjModel* m, const mjData* d, mjvScene* scn, const mjvOption* opt) {
  // process skins
  for (int i=0; i < m->nskin; i++) {
    // get info
    int vertadr = m->skin_vertadr[i];
    int vertnum = m->skin_vertnum[i];
    int faceadr = m->skin_faceadr[i];
    int facenum = m->skin_facenum[i];

    // clear positions and normals
    memset(scn->skinvert + 3*vertadr, 0, 3*vertnum*sizeof(float));
    memset(scn->skinnormal + 3*vertadr, 0, 3*vertnum*sizeof(float));

    // update only if visible
    if (opt->skingroup[mjMAX(0, mjMIN(mjNGROUP-1, m->skin_group[i]))]) {
      // accumulate positions from all bones
      for (int j=m->skin_boneadr[i];
           j < m->skin_boneadr[i]+m->skin_bonenum[i];
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
        mju_mulMatVec3(translate, rotate, bindpos);
        mju_sub3(translate, d->xpos+3*bodyid, translate);

        // process all bone vertices
        for (int k=m->skin_bonevertadr[j];
             k < m->skin_bonevertadr[j]+m->skin_bonevertnum[j];
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
          mju_mulMatVec3(pos1, rotate, pos);
          mju_addTo3(pos1, translate);

          // accumulate position
          scn->skinvert[3*(vertadr+vid)] += vweight*(float)pos1[0];
          scn->skinvert[3*(vertadr+vid)+1] += vweight*(float)pos1[1];
          scn->skinvert[3*(vertadr+vid)+2] += vweight*(float)pos1[2];
        }
      }

      // compute vertex normals from face normals
      for (int k=faceadr; k < faceadr+facenum; k++) {
        // get face vertex indices
        int vid[3] = {
          m->skin_face[3*k],
          m->skin_face[3*k+1],
          m->skin_face[3*k+2]
        };

        // get triangle edges
        mjtNum vec01[3], vec02[3];
        for (int r=0; r < 3; r++) {
          vec01[r] = scn->skinvert[3*(vertadr+vid[1])+r] - scn->skinvert[3*(vertadr+vid[0])+r];
          vec02[r] = scn->skinvert[3*(vertadr+vid[2])+r] - scn->skinvert[3*(vertadr+vid[0])+r];
        }

        // compute face normal
        mjtNum nrm[3];
        mju_cross(nrm, vec01, vec02);

        // add normal to each vertex with weight = area
        for (int r=0; r < 3; r++) {
          for (int t=0; t < 3; t++) {
            scn->skinnormal[3*(vertadr+vid[r])+t] += nrm[t];
          }
        }
      }

      // normalize normals
      for (int k=vertadr; k < vertadr+vertnum; k++) {
        float s = sqrtf(
          scn->skinnormal[3*k+0]*scn->skinnormal[3*k+0] +
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
        for (int k=vertadr; k < vertadr+vertnum; k++) {
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
  // clear geoms
  scn->ngeom = 0;

  // trigger plugin visualization hooks
  if (m->nplugin) {
    const int nslot = mjp_pluginCount();
    // iterate over plugins, call visualize if defined
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        mjERROR("invalid plugin slot: %d", slot);
      }
      if (plugin->visualize) {
        plugin->visualize(m, d, opt, scn, i);
      }
    }
  }

  // add all categories
  mjv_addGeoms(m, d, opt, pert, catmask, scn);

  // update camera
  mjv_updateCamera(m, d, cam, scn);

  // add lights
  mjv_makeLights(m, d, scn);

  // update flexes
  if (opt->flags[mjVIS_FLEXVERT] || opt->flags[mjVIS_FLEXEDGE] ||
      opt->flags[mjVIS_FLEXFACE] || opt->flags[mjVIS_FLEXSKIN]) {
    mjv_updateActiveFlex(m, d, scn, opt);
  }

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
  for (int i=0; i < 50; i++) {
    // get value and gradient
    mjtNum grad;
    mjtNum res = catenary_residual(b, intercept, &grad);

    if (mju_abs(res) < tolerance) {
      break;
    }

    // Newton step
    mjtNum step = -res / grad;

    // backtracking line-search is not essential but can reduce number of iterations
    for (int j=0; j < 10; j++) {
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
                 mjtNum* catenary, int ncatenary) {
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

    // compute full catenary: ncatenary points
    else {
      // b*h: scaled catenary flatness
      mjtNum bh = solve_catenary(v, h, length) * h;

      // horizontal and vertical offsets
      mjtNum h_offset = -0.5 * (mju_log((length+v) / (length-v)) * bh - h);
      mjtNum v_offset = -cosh_sinh(h_offset / bh, NULL) * bh;

      // start point
      mju_copy3(catenary+0, x0);

      // hanging points
      for (int i=1; i < ncatenary-1; i++) {
        // linearly spaced horizontal offset
        mjtNum horizontal = i*h/ncatenary;
        mju_addScl3(catenary+3*i, x0, across, horizontal);

        // vertical offset, evaluate catenary values
        mjtNum vertical = bh * cosh_sinh((horizontal - h_offset) / bh, NULL) + v_offset;
        mju_addToScl3(catenary+3*i, up, vertical);
      }

      // end point
      mju_copy3(catenary+3*(ncatenary-1), x1);

      return ncatenary;
    }
  }

  return 0;  // SHOULD NOT OCCUR
}
