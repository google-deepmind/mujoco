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

#include "engine/engine_collision_convex.h"

#include <math.h>

#include <ccd/ccd.h>
#include <ccd/vec3.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_collision_primitive.h"
#include "engine/engine_macro.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"

// ccd center function
void mjccd_center(const void *obj, ccd_vec3_t *center) {
  const mjtCCD* ccd = (const mjtCCD*)obj;

  // return geom position
  mju_copy3(center->v, ccd->data->geom_xpos + 3*ccd->geom);
}



// ccd support function
void mjccd_support(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *vec) {
  const mjtCCD* ccd = (const mjtCCD*)obj;
  const mjModel* m = ccd->model;
  const mjData* d = ccd->data;
  int g = ccd->geom;

  float* vertdata;
  int ibest, graphadr, numvert, change, locid;
  int *vert_edgeadr, *vert_globalid, *edge_localid;
  mjtNum tmp, vdot;

  const mjtNum* size = m->geom_size+3*g;  // geom sizes
  mjtNum dir[3];                          // direction in geom local frame
  mjtNum res[3];                          // result in geom local frame

  // rotate dir to geom local frame
  mju_rotVecMatT(dir, _dir->v, d->geom_xmat+9*g);

  // compute result according to geom type
  switch (m->geom_type[g]) {
  case mjGEOM_SPHERE:
    mju_scl3(res, dir, size[0]);
    break;

  case mjGEOM_CAPSULE:
    // start with sphere
    mju_scl3(res, dir, size[0]);

    // add cylinder contribution
    res[2] += mju_sign(dir[2]) * size[1];
    break;

  case mjGEOM_ELLIPSOID:
    // find support point on unit sphere: scale dir by ellipsoid sizes and renormalize
    for (int i=0; i<3; i++) {
      res[i] = dir[i] * size[i];
    }
    mju_normalize3(res);

    // transform to ellipsoid
    for (int i=0; i<3; i++) {
      res[i] *= size[i];
    }
    break;

  case mjGEOM_CYLINDER:
    // set result in XY plane: support on circle
    tmp = mju_sqrt(dir[0]*dir[0] + dir[1]*dir[1]);
    if (tmp>mjMINVAL) {
      res[0] = dir[0]/tmp*size[0];
      res[1] = dir[1]/tmp*size[0];
    } else {
      res[0] = res[1] = 0;
    }

    // set result in Z direction
    res[2] = mju_sign(dir[2]) * size[1];
    break;

  case mjGEOM_BOX:
    for (int i=0; i<3; i++) {
      res[i] = mju_sign(dir[i]) * size[i];
    }
    break;

  case mjGEOM_MESH:
    // init search
    vertdata = m->mesh_vert + 3*m->mesh_vertadr[m->geom_dataid[g]];
    tmp = -1E+10;
    ibest = -1;

    // no graph data: exhaustive search
    if (m->mesh_graphadr[m->geom_dataid[g]]<0) {
      // search all vertices, find best
      for (int i=0; i<m->mesh_vertnum[m->geom_dataid[g]]; i++) {
        // vdot = dot(vertex, dir)
        vdot = dir[0] * (mjtNum)vertdata[3*i] +
               dir[1] * (mjtNum)vertdata[3*i+1] +
               dir[2] * (mjtNum)vertdata[3*i+2];

        // update best
        if (vdot>tmp) {
          tmp = vdot;
          ibest = i;
        }
      }

      // record best vertex index, in globalid format
      ((mjtCCD*)ccd)->meshindex = ibest;
    }

    // hill-climb using graph data
    else {
      // get info
      graphadr = m->mesh_graphadr[m->geom_dataid[g]];
      numvert = m->mesh_graph[graphadr];
      vert_edgeadr = m->mesh_graph + graphadr + 2;
      vert_globalid = m->mesh_graph + graphadr + 2 + numvert;
      edge_localid = m->mesh_graph + graphadr + 2 + 2*numvert;

      // init with first vertex in convex hull
      ibest = 0;
      tmp = dir[0] * (mjtNum)vertdata[3*vert_globalid[0]] +
            dir[1] * (mjtNum)vertdata[3*vert_globalid[0]+1] +
            dir[2] * (mjtNum)vertdata[3*vert_globalid[0]+2];

      // hill-climb until no change
      change = 1;
      while (change) {
        // look for improvement in ibest neighborhood
        change = 0;
        int i = vert_edgeadr[ibest];
        while ((locid=edge_localid[i]) >= 0) {
          // vdot = dot(vertex, dir)
          vdot = dir[0] * (mjtNum)vertdata[3*vert_globalid[locid]] +
                 dir[1] * (mjtNum)vertdata[3*vert_globalid[locid]+1] +
                 dir[2] * (mjtNum)vertdata[3*vert_globalid[locid]+2];

          // update best
          if (vdot>tmp) {
            tmp = vdot;
            ibest = locid;
            change = 1;
          }

          // advance to next edge
          i++;
        }
      }

      // record best vertex index, in locid format
      ((mjtCCD*)ccd)->meshindex = ibest;

      // map best index to globalid
      ibest = vert_globalid[ibest];
    }

    // sanity check, SHOULD NOT OCCUR
    if (ibest<0) {
      mju_warning("mesh_support could not find support vertex");
      mju_zero3(res);
    }

    // copy best vertex
    else {
      for (int i=0; i<3; i++) {
        res[i] = (mjtNum)vertdata[3*ibest + i];
      }
    }
    break;

  default:
    mju_error_i("ccd support function is undefined for geom type %d", m->geom_type[g]);
  }

  // add dir*margin/2 to result
  for (int i=0; i<3; i++) {
    res[i] += dir[i] * ccd->margin/2;
  }

  // rotate result to global frame
  mju_rotVecMat(vec->v, res, d->geom_xmat+9*g);

  // add geom position
  mju_addTo3(vec->v, d->geom_xpos+3*g);
}



// find single convex-convex collision, using libccd
static int mjc_MPRIteration(mjtCCD* obj1, mjtCCD* obj2, const ccd_t* ccd,
                            const mjModel* m, const mjData* d,
                            mjContact* con, int g1, int g2, mjtNum margin) {
  ccd_vec3_t dir, pos;
  ccd_real_t depth;
  if (ccdMPRPenetration(obj1, obj2, ccd, &depth, &dir, &pos)==0) {
    // contact is found but normal is undefined
    if (ccdVec3Eq(&dir, ccd_vec3_origin)) {
      return 0;
    }

    // fill in contact data
    con->dist = margin-depth;
    mju_copy3(con->frame, dir.v);
    mju_copy3(con->pos, pos.v);
    mju_zero3(con->frame+3);

    // fix contact frame normal
    mjc_fixNormal(m, d, con, g1, g2);

    return 1;
  }

  // no contact found
  else {
    return 0;
  }
}



// compare new contact to previous contacts, return 1 if it is far from all of them
static int mjc_isDistinctContact(mjContact* con, int ncon, mjtNum tolerance) {
  for (int i=0; i<ncon-1; i++) {
    if (mju_dist3(con[i].pos, con[ncon - 1].pos) <= tolerance) {
      return 0;
    }
  }
  return 1;
}



// in-place rotation of spatial frame around given point of origin
static void mju_rotateFrame(const mjtNum origin[3], const mjtNum rot[9],
                            mjtNum xmat[9], mjtNum xpos[3]) {
  mjtNum mat[9], vec[3], rel[3];

  // rotate frame: xmat = rot*xmat
  mju_mulMatMat(mat, rot, xmat, 3, 3, 3);
  mju_copy(xmat, mat, 9);

  // vector to rotation origin: rel = origin - xpos
  mju_sub3(rel, origin, xpos);

  // displacement of origin due to rotation: vec = rot*rel - rel
  mju_rotVecMat(vec, rel, rot);
  mju_subFrom3(vec, rel);

  // correct xpos by subtracting displacement: xpos = xpos - vec
  mju_subFrom3(xpos, vec);
}



// multi-point convex-convex collision, using libccd
int mjc_Convex(const mjModel* m, const mjData* d,
               mjContact* con, int g1, int g2, mjtNum margin) {
  ccd_t ccd;
  mjtCCD obj1 = {m, d, g1, -1, margin, {1, 0, 0, 0}};
  mjtCCD obj2 = {m, d, g2, -1, margin, {1, 0, 0, 0}};

  // init ccd structure
  ccd.first_dir = ccdFirstDirDefault;
  ccd.center1 = mjccd_center;
  ccd.center2 = mjccd_center;
  ccd.support1 = mjccd_support;
  ccd.support2 = mjccd_support;

  // set ccd paramters
  ccd.max_iterations = m->opt.mpr_iterations;
  ccd.mpr_tolerance = m->opt.mpr_tolerance;

  // find initial contact
  int ncon = mjc_MPRIteration(&obj1, &obj2, &ccd, m, d, con, g1, g2, margin);

  // look for additional contacts
  if (ncon && mjENABLED(mjENBL_MULTICCD)  // TODO(tassa) leave as bitflag or make geom attribute (?)
      && m->geom_type[g1] != mjGEOM_ELLIPSOID && m->geom_type[g1] != mjGEOM_SPHERE
      && m->geom_type[g2] != mjGEOM_ELLIPSOID && m->geom_type[g2] != mjGEOM_SPHERE) {
    // multiCCD parameters
    const mjtNum relative_tolerance = 1e-3;
    const mjtNum perturbation_angle = 1e-3;

    // save positions and orientations of g1 and g2
    mjtNum xpos1[3], xmat1[9], xpos2[3], xmat2[9];
    mju_copy3(xpos1, d->geom_xpos+3*g1);
    mju_copy(xmat1, d->geom_xmat+9*g1, 9);
    mju_copy3(xpos2, d->geom_xpos+3*g2);
    mju_copy(xmat2, d->geom_xmat+9*g2, 9);

    // complete frame of initial contact
    mjtNum frame[9];
    mju_copy(frame, con[0].frame, 9);
    mju_makeFrame(frame);

    // tolerance for determining if newly found contacts are distinct
    const mjtNum tolerance = relative_tolerance * mju_min(m->geom_rbound[g1], m->geom_rbound[g2]);

    // axes and rotation angles for perturbation test
    mjtNum* axes[2] = {frame+3, frame+6};
    mjtNum angles[2] = {-perturbation_angle, perturbation_angle};

    // rotate both geoms, search for new contacts
    for (int axis_id = 0; axis_id < 2; ++axis_id) {
      for (int angle_id = 0; angle_id < 2; ++angle_id) {
        mjtNum* axis = axes[axis_id];
        mjtNum angle = angles[angle_id];

        // make rotation matrix rot
        mjtNum quat[4], rot[9];
        mju_axisAngle2Quat(quat, axis, angle);
        mju_quat2Mat(rot, quat);

        // rotate g1 around initial contact point
        mju_rotateFrame(con[0].pos, rot, d->geom_xmat+9*g1, d->geom_xpos+3*g1);

        // inversely rotate g2 around initial contact point
        mjtNum invrot[9];
        mju_transpose(invrot, rot, 3, 3);
        mju_rotateFrame(con[0].pos, invrot, d->geom_xmat+9*g2, d->geom_xpos+3*g2);

        // search for new contact
        int new_contact = mjc_MPRIteration(&obj1, &obj2, &ccd, m, d, con+ncon, g1, g2, margin);

        // check new contact
        if (new_contact && mjc_isDistinctContact(con, ncon + 1, tolerance)) {
          // set penetration of new point to equal that of initial point
          con[ncon].dist = con[0].dist;
          // add new point
          ncon += 1;
        }

        // reset positions and orientations of g1 and g2
        mju_copy3(d->geom_xpos+3*g1, xpos1);
        mju_copy(d->geom_xmat+9*g1, xmat1, 9);
        mju_copy3(d->geom_xpos+3*g2, xpos2);
        mju_copy(d->geom_xmat+9*g2, xmat2, 9);
      }
    }
  }
  return ncon;
}



// parameters for plane-mesh extra contacts
const int maxplanemesh = 3;
const mjtNum tolplanemesh = 0.3;

// add one plane-mesh contact
static int addplanemesh(mjContact* con, const float vertex[3],
                        const mjtNum pos1[3], const mjtNum normal1[3],
                        const mjtNum pos2[3], const mjtNum mat2[9],
                        const mjtNum first[3], mjtNum rbound) {
  // compute point in global coordinates
  mjtNum pnt[3], v[3] = {vertex[0], vertex[1], vertex[2]};
  mju_rotVecMat(pnt, v, mat2);
  mju_addTo3(pnt, pos2);

  // skip if too close to first contact
  if (mju_dist3(pnt, first)<tolplanemesh*rbound) {
    return 0;
  }

  // pnt-pos difference vector
  mjtNum dif[3];
  mju_sub3(dif, pnt, pos1);

  // set distance
  con->dist = mju_dot3(normal1, dif);

  // set position
  mju_copy3(con->pos, pnt);
  mju_addToScl3(con->pos, normal1, -0.5*con->dist);

  // set frame
  mju_copy3(con->frame, normal1);
  mju_zero3(con->frame+3);

  return 1;
}



// plane-convex collision, using libccd
int mjc_PlaneConvex(const mjModel* m, const mjData* d,
                    mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO
  mjtNum dist, dif[3], normal[3] = {mat1[2], mat1[5], mat1[8]};
  ccd_vec3_t dir, vec;
  mjtCCD obj = {m, d, g2, -1, 0, {1, 0, 0, 0}};

  // get support point in -normal direction
  ccdVec3Set(&dir, -mat1[2], -mat1[5], -mat1[8]);
  mjccd_support(&obj, &dir, &vec);

  // compute normal distance, return if too far
  mju_sub3(dif, vec.v, pos1);
  dist = mju_dot3(normal, dif);
  if (dist>margin) {
    return 0;
  }

  // fill in contact data
  con->dist = dist;
  mju_copy3(con->pos, vec.v);
  mju_addToScl3(con->pos, normal, -0.5*dist);
  mju_copy3(con->frame, normal);
  mju_zero3(con->frame+3);

  //--------------- add all/connected vertices below margin
  float* vertdata;
  int graphadr, numvert, locid;
  int *vert_edgeadr, *vert_globalid, *edge_localid;
  mjtNum vdot;
  int count = 1, g = g2;

  // g is an ellipsoid: no need for further mesh-specific processing
  if (m->geom_dataid[g] == -1) {
    return count;
  }

  // init
  vertdata = m->mesh_vert + 3*m->mesh_vertadr[m->geom_dataid[g]];

  // express dir in geom local frame
  mjtNum locdir[3];
  mju_rotVecMatT(locdir, dir.v, d->geom_xmat+9*g);

  // inclusion threshold along locdir, relative to geom2 center
  mju_sub3(dif, pos2, pos1);
  mjtNum threshold = mju_dot3(normal, dif) - margin;

  // no graph data: exhaustive search
  if (m->mesh_graphadr[m->geom_dataid[g]]<0) {
    // search all vertices, find best
    for (int i=0; i<m->mesh_vertnum[m->geom_dataid[g]] && count<maxplanemesh; i++) {
      // vdot = dot(vertex, dir)
      vdot = locdir[0] * (mjtNum)vertdata[3*i] +
             locdir[1] * (mjtNum)vertdata[3*i+1] +
             locdir[2] * (mjtNum)vertdata[3*i+2];

      // detect contact, skip best
      if (vdot>threshold && i!=obj.meshindex) {
        count += addplanemesh(con+count, vertdata+3*i,
                              pos1, normal, pos2, mat2,
                              con->pos, m->geom_rbound[g2]);
      }
    }
  }

  // use graph data
  else if (obj.meshindex>=0) {
    // get info
    graphadr = m->mesh_graphadr[m->geom_dataid[g]];
    numvert = m->mesh_graph[graphadr];
    vert_edgeadr = m->mesh_graph + graphadr + 2;
    vert_globalid = m->mesh_graph + graphadr + 2 + numvert;
    edge_localid = m->mesh_graph + graphadr + 2 + 2*numvert;

    // look for contacts in ibest neighborhood
    int i = vert_edgeadr[obj.meshindex];
    while ((locid=edge_localid[i])>=0 && count<maxplanemesh) {
      // vdot = dot(vertex, dir)
      vdot = locdir[0] * (mjtNum)vertdata[3*vert_globalid[locid]] +
             locdir[1] * (mjtNum)vertdata[3*vert_globalid[locid]+1] +
             locdir[2] * (mjtNum)vertdata[3*vert_globalid[locid]+2];

      // detect contact
      if (vdot>threshold) {
        count += addplanemesh(con+count, vertdata+3*vert_globalid[locid],
                              pos1, normal, pos2, mat2,
                              con->pos, m->geom_rbound[g2]);
      }

      // advance to next edge
      i++;
    }
  }

  return count;
}



//----------------------------  heightfield collisions ---------------------------------------------

// ccd prism object type
struct _mjtPrism {
  mjtNum v[6][3];
};

typedef struct _mjtPrism mjtPrism;


// ccd prism support function
static void prism_support(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec) {
  int i, istart, ibest;
  mjtNum best, tmp;
  const mjtPrism* p = (const mjtPrism*)obj;

  // find best vertex in halfspace determined by dir.z
  istart = dir->v[2]<0 ? 0 : 3;
  ibest = istart;
  best = mju_dot3(p->v[istart], dir->v);
  for (i=istart+1; i<istart+3; i++) {
    if ((tmp = mju_dot3(p->v[i], dir->v)) > best) {
      ibest = i;
      best = tmp;
    }
  }

  // copy best point
  mju_copy3(vec->v, p->v[ibest]);
}


// ccd prism center function
static void prism_center(const void *obj, ccd_vec3_t *center) {
  const mjtPrism* p = (const mjtPrism*)obj;

  // compute mean
  mju_zero3(center->v);
  for (int i=0; i<6; i++) {
    mju_addTo3(center->v, p->v[i]);
  }
  mju_scl3(center->v, center->v, 1.0/6.0);
}


// ccd prism first dir
static void prism_firstdir(const void* o1, const void* o2, ccd_vec3_t *vec) {
  ccdVec3Set(vec, 0, 0, 1);
}


// add vertex to prism, count vertices
static void addVert(int* nvert, mjtPrism* prism, mjtNum x, mjtNum y, mjtNum z) {
  // move old data
  mju_copy3(prism->v[0], prism->v[1]);
  mju_copy3(prism->v[1], prism->v[2]);
  mju_copy3(prism->v[3], prism->v[4]);
  mju_copy3(prism->v[4], prism->v[5]);

  // add new vertex at last position
  prism->v[2][0] = prism->v[5][0] = x;
  prism->v[2][1] = prism->v[5][1] = y;
  prism->v[5][2] = z;

  // count
  (*nvert)++;
}



// entry point for heightfield collisions
int mjc_ConvexHField(const mjModel* m, const mjData* d,
                     mjContact* con, int g1, int g2, mjtNum margin) {
  mjGETINFO
  mjtNum mat[9], savemat2[9], savepos2[3], pos[3], vec[3], r2, dx, dy;
  mjtNum xmin, xmax, ymin, ymax, zmin, zmax;
  int hid = m->geom_dataid[g1];
  int nrow = m->hfield_nrow[hid];
  int ncol = m->hfield_ncol[hid];
  int r, c, dr[2], cnt, rmin, rmax, cmin, cmax, nvert;
  const float* data = m->hfield_data + m->hfield_adr[hid];
  mjtPrism prism;

  // ccd-related
  ccd_vec3_t dirccd, vecccd;
  ccd_real_t depth;
  mjtCCD obj = {m, d, g2, -1, 0, {1, 0, 0, 0}};
  ccd_t ccd;

  // point size1 to hfield size instead of geom1 size
  size1 = m->hfield_size + 4*hid;

  //------------------------------------- frame alignment, box-sphere test

  // express geom2 pos in heightfield frame
  mju_sub3(vec, pos2, pos1);
  mju_mulMatTVec(pos, mat1, vec, 3, 3);

  // get geom2 rbound
  r2 = m->geom_rbound[g2];

  // box-sphere test: horizontal plane
  for (int i=0; i<2; i++) {
    if ((size1[i] < pos[i]-r2-margin) || (-size1[i] > pos[i]+r2+margin)) {
      return 0;
    }
  }

  // box-sphere test in: vertical direction
  if (size1[2] < pos[2]-r2-margin) {   // up
    return 0;
  }
  if (-size1[3] > pos[2]+r2+margin) {  // down
    return 0;
  }

  // express geom2 mat in heightfield frame
  mju_mulMatTMat(mat, mat1, mat2, 3, 3, 3);

  //------------------------------------- AABB computation, box-box test

  // save mat2 and pos2, replace with relative frame
  mju_copy(savemat2, mat2, 9);
  mju_copy3(savepos2, pos2);
  mju_copy(mat2, mat, 9);
  mju_copy3(pos2, pos);

  // get support point in +X
  ccdVec3Set(&dirccd, 1, 0, 0);
  mjccd_support(&obj, &dirccd, &vecccd);
  xmax = vecccd.v[0];

  // get support point in -X
  ccdVec3Set(&dirccd, -1, 0, 0);
  mjccd_support(&obj, &dirccd, &vecccd);
  xmin = vecccd.v[0];

  // get support point in +Y
  ccdVec3Set(&dirccd, 0, 1, 0);
  mjccd_support(&obj, &dirccd, &vecccd);
  ymax = vecccd.v[1];

  // get support point in -Y
  ccdVec3Set(&dirccd, 0, -1, 0);
  mjccd_support(&obj, &dirccd, &vecccd);
  ymin = vecccd.v[1];

  // get support point in +Z
  ccdVec3Set(&dirccd, 0, 0, 1);
  mjccd_support(&obj, &dirccd, &vecccd);
  zmax = vecccd.v[2];

  // get support point in -Z
  ccdVec3Set(&dirccd, 0, 0, -1);
  mjccd_support(&obj, &dirccd, &vecccd);
  zmin = vecccd.v[2];

  // box-box test
  if ((xmin-margin > size1[0]) || (xmax+margin < -size1[0]) ||
      (ymin-margin > size1[1]) || (ymax+margin < -size1[1]) ||
      (zmin-margin > size1[2]) || (zmax+margin < -size1[3])) {
    // restore mat2 and pos2
    mju_copy(mat2, savemat2, 9);
    mju_copy3(pos2, savepos2);

    return 0;
  }

  // compute sub-grid bounds
  cmin = (int) floor((xmin + size1[0]) / (2*size1[0]) * (ncol-1));
  cmax = (int) ceil ((xmax + size1[0]) / (2*size1[0]) * (ncol-1));
  rmin = (int) floor((ymin + size1[1]) / (2*size1[1]) * (nrow-1));
  rmax = (int) ceil ((ymax + size1[1]) / (2*size1[1]) * (nrow-1));
  cmin = mjMAX(0, cmin);
  cmax = mjMIN(ncol-1, cmax);
  rmin = mjMAX(0, rmin);
  rmax = mjMIN(nrow-1, rmax);

  //------------------------------------- collision testing

  // init ccd structure
  ccd.first_dir = prism_firstdir;
  ccd.center1 = prism_center;
  ccd.center2 = mjccd_center;
  ccd.support1 = prism_support;
  ccd.support2 = mjccd_support;

  // set ccd parameters
  ccd.max_iterations = m->opt.mpr_iterations;
  ccd.mpr_tolerance = m->opt.mpr_tolerance;

  // geom margin needed for actual collision test
  obj.margin = margin;

  // compute real-valued grid step, and triangulation direction
  dx = (2.0*size1[0]) / (ncol-1);
  dy = (2.0*size1[1]) / (nrow-1);
  dr[0] = 1;
  dr[1] = 0;

  // set zbottom value using base size
  prism.v[0][2] = prism.v[1][2] = prism.v[2][2] = -size1[3];

  // process all prisms in sub-grid
  cnt = 0;
  for (r=rmin; r<rmax; r++) {
    nvert = 0;
    for (c=cmin; c<=cmax; c++) {
      for (int i=0; i<2; i++) {
        // send vertex to prism constructor
        addVert(&nvert, &prism, dx*c-size1[0], dy*(r+dr[i])-size1[1],
                data[(r+dr[i])*ncol+c]*size1[2]+margin);

        // check for enough vertices
        if (nvert>2) {
          // prism height test
          if (prism.v[3][2]<zmin && prism.v[4][2]<zmin && prism.v[5][2]<zmin) {
            continue;
          }

          // run MPR, save contact
          if (ccdMPRPenetration(&prism, &obj, &ccd, &depth, &dirccd, &vecccd)==0 &&
              !ccdVec3Eq(&dirccd, ccd_vec3_origin)) {
            // fill in contact data, transform to global coordinates
            con[cnt].dist = -depth;
            mju_rotVecMat(con[cnt].frame, dirccd.v, mat1);
            mju_rotVecMat(con[cnt].pos, vecccd.v, mat1);
            mju_addTo3(con[cnt].pos, pos1);
            mju_zero3(con[cnt].frame+3);

            // count, stop if max number reached
            cnt++;
            if (cnt>=mjMAXCONPAIR) {
              r = rmax+1;
              c = cmax+1;
              i = 3;
              break;
            }
          }
        }
      }
    }
  }

  // restore mat2 and pos2
  mju_copy(mat2, savemat2, 9);
  mju_copy3(pos2, savepos2);

  // fix contact normals
  for (int i=0; i<cnt; i++) {
    mjc_fixNormal(m, d, con+i, g1, g2);
  }

  return cnt;
}



//--------------------------- fix contact frame normal ---------------------------------------------

// compute normal for point outside ellipsoid, using ray-projection SQP
static int mjc_ellipsoidInside(mjtNum nrm[3], const mjtNum pos[3], const mjtNum size[3]) {
  // algorithm constants
  const int maxiter = 30;
  const mjtNum tolerance = 1e-6;

  // precompute quantities
  mjtNum S2inv[3] = {1/(size[0]*size[0]), 1/(size[1]*size[1]), 1/(size[2]*size[2])};
  mjtNum C = pos[0]*pos[0]*S2inv[0] + pos[1]*pos[1]*S2inv[1] + pos[2]*pos[2]*S2inv[2] - 1;
  if (C>0) {
    return 0;
  }

  // normalize initial normal (just in case)
  mju_normalize3(nrm);

  // main iteration
  int iter;
  for (iter=0; iter<maxiter; iter++) {
    // coefficients and determinant of quadratic
    mjtNum A = nrm[0]*nrm[0]*S2inv[0] + nrm[1]*nrm[1]*S2inv[1] + nrm[2]*nrm[2]*S2inv[2];
    mjtNum B = pos[0]*nrm[0]*S2inv[0] + pos[1]*nrm[1]*S2inv[1] + pos[2]*nrm[2]*S2inv[2];
    mjtNum det = B*B - A*C;
    if (det<mjMINVAL || A<mjMINVAL) {
      return (iter>0);
    }

    // ray intersection with ellipse: pos + x*nrm, x>=0
    mjtNum x = (-B + mju_sqrt(det))/A;
    if (x<0) {
      return (iter>0);
    }

    // new point on ellipsoid
    mjtNum pnt[3];
    mju_addScl3(pnt, pos, nrm, x);

    // normal at new point
    mjtNum newnrm[3] = {pnt[0]*S2inv[0], pnt[1]*S2inv[1], pnt[2]*S2inv[2]};
    mju_normalize3(newnrm);

    // save change and assign
    mjtNum change = mju_dist3(nrm, newnrm);
    mju_copy3(nrm, newnrm);

    // terminate if converged
    if (change<tolerance) {
      break;
    }
  }

  return 1;
}



// compute normal for point inside ellipsoid, using diagonal QCQP
static int mjc_ellipsoidOutside(mjtNum nrm[3], const mjtNum pos[3], const mjtNum size[3]) {
  // algorithm constants
  const int maxiter = 30;
  const mjtNum tolerance = 1e-6;

  // precompute quantities
  mjtNum S2[3] = {size[0]*size[0], size[1]*size[1], size[2]*size[2]};
  mjtNum PS2[3] = {pos[0]*pos[0]*S2[0], pos[1]*pos[1]*S2[1], pos[2]*pos[2]*S2[2]};

  // main iteration
  mjtNum la = 0;
  int iter;
  for (iter=0; iter<maxiter; iter++) {
    // precompute 1/(s^2+la)
    mjtNum R[3] = {1/(S2[0]+la), 1/(S2[1]+la), 1/(S2[2]+la)};

    // value
    mjtNum val = PS2[0]*R[0]*R[0] + PS2[1]*R[1]*R[1] + PS2[2]*R[2]*R[2] - 1;
    if (val<tolerance) {
      break;
    }

    // derivative
    mjtNum deriv = -2*(PS2[0]*R[0]*R[0]*R[0] + PS2[1]*R[1]*R[1]*R[1] + PS2[2]*R[2]*R[2]*R[2]);
    if (deriv>-mjMINVAL) {
      break;
    }

    // delta
    mjtNum delta = -val/deriv;
    if (delta<tolerance) {
      break;
    }

    // update
    la += delta;
  }

  // compute normal given lambda
  nrm[0] = pos[0]/(S2[0]+la);
  nrm[1] = pos[1]/(S2[1]+la);
  nrm[2] = pos[2]/(S2[2]+la);
  mju_normalize3(nrm);

  return 1;
}



// entry point
void mjc_fixNormal(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2) {
  mjtNum dst1, dst2;

  // get geom ids and types
  int gid[2] = {g1, g2};
  int type[2];
  for (int i=0; i<2; i++) {
    type[i] = m->geom_type[gid[i]];

    // set to -1 if type cannot be processed
    if (type[i]!=mjGEOM_SPHERE    &&
        type[i]!=mjGEOM_CAPSULE   &&
        type[i]!=mjGEOM_ELLIPSOID &&
        type[i]!=mjGEOM_CYLINDER) {
      type[i] = -1;
    }
  }

  // neither type can be processed: nothing to do
  if (type[0]<0 && type[1]<0) {
    return;
  }

  // init normals
  mjtNum normal[2][3] = {
    {con->frame[0], con->frame[1], con->frame[2]},
    {-con->frame[0], -con->frame[1], -con->frame[2]}
  };

  // process geoms in type range
  int processed[2] = {0, 0};
  for (int i=0; i<2; i++) {
    if (type[i]>=0) {
      // get geom mat and size
      mjtNum* mat = d->geom_xmat + 9*gid[i];
      mjtNum* size = m->geom_size + 3*gid[i];

      // map contact point and normal to local frame
      mjtNum dif[3], pos[3], nrm[3];
      mju_sub3(dif, con->pos, d->geom_xpos+3*gid[i]);
      mju_rotVecMatT(pos, dif, mat);
      mju_rotVecMatT(nrm, normal[i], mat);

      // process according to type
      switch (type[i]) {
      case mjGEOM_SPHERE:
        mju_copy3(nrm, pos);
        processed[i] = 1;
        break;

      case mjGEOM_CAPSULE:
        // Z: bottom cap
        if (pos[2]<-size[1]) {
          nrm[2] = pos[2]+size[1];
        }

        // Z: top cap
        else if (pos[2]>size[1]) {
          nrm[2] = pos[2]-size[1];
        }

        // Z: cylinder
        else {
          nrm[2] = 0;
        }

        // copy XY
        nrm[0] = pos[0];
        nrm[1] = pos[1];
        processed[i] = 1;
        break;

      case mjGEOM_ELLIPSOID:
        // guard against invalid ellipsoid size (just in case)
        if (size[0]<mjMINVAL || size[1]<mjMINVAL || size[2]<mjMINVAL) {
          break;
        }

        // compute elliptic distance^2
        dst1 = pos[0]*pos[0]/(size[0]*size[0]) +
               pos[1]*pos[1]/(size[1]*size[1]) +
               pos[2]*pos[2]/(size[2]*size[2]);

        // dispatch to inside or outside solver
        if (dst1<=1) {
          processed[i] = mjc_ellipsoidInside(nrm, pos, size);
        } else {
          processed[i] = mjc_ellipsoidOutside(nrm, pos, size);
        }
        break;

      case mjGEOM_CYLINDER:
        // skip if within 5% length of flat wall
        if (mju_abs(pos[2])>0.95*size[1]) {
          break;
        }

        // compute distances to flat and round wall
        dst1 = mju_abs(size[1]-mju_abs(pos[2]));
        dst2 = mju_abs(size[0]-mju_norm(pos, 2));

        // require 4x closer to round than flat wall
        if (dst1<0.25*dst2) {
          break;
        }

        // set normal for round wall
        nrm[0] = pos[0];
        nrm[1] = pos[1];
        nrm[2] = 0;
        processed[i] = 1;
        break;
      }

      // normalize and map normal to global frame
      if (processed[i]) {
        mju_normalize3(nrm);
        mju_rotVecMat(normal[i], nrm, mat);
      }
    }
  }

  // both processed: average
  if (processed[0] && processed[1]) {
    mju_sub3(con->frame, normal[0], normal[1]);
    mju_normalize3(con->frame);
  }

  // first processed: copy
  else if (processed[0]) {
    mju_copy3(con->frame, normal[0]);
  }

  // second processed: copy reverse
  else if (processed[1]) {
    mju_scl3(con->frame, normal[1], -1);
  }

  // clear second frame axis if processed, just in case
  if (processed[0] || processed[1]) {
    mju_zero3(con->frame+3);
  }
}
