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
//---------------------------------//

#include "engine/engine_ray.h"

#include <stddef.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjvisualize.h>
#include "engine/engine_io.h"
#include "engine/engine_plugin.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"



//---------------------------- utility functions ---------------------------------------------------

// map ray to local geom frame
static void ray_map(const mjtNum* pos, const mjtNum* mat, const mjtNum* pnt, const mjtNum* vec,
                    mjtNum* lpnt, mjtNum* lvec) {
  const mjtNum dif[3] = {pnt[0]-pos[0], pnt[1]-pos[1], pnt[2]-pos[2]};

  // lpnt = mat' * dif
  lpnt[0] = mat[0]*dif[0] + mat[3]*dif[1] + mat[6]*dif[2];
  lpnt[1] = mat[1]*dif[0] + mat[4]*dif[1] + mat[7]*dif[2];
  lpnt[2] = mat[2]*dif[0] + mat[5]*dif[1] + mat[8]*dif[2];

  // lvec = mat' * vec
  lvec[0] = mat[0]*vec[0] + mat[3]*vec[1] + mat[6]*vec[2];
  lvec[1] = mat[1]*vec[0] + mat[4]*vec[1] + mat[7]*vec[2];
  lvec[2] = mat[2]*vec[0] + mat[5]*vec[1] + mat[8]*vec[2];
}



// map to azimuth angle in spherical coordinates
static mjtNum longitude(const mjtNum vec[3]) {
  return mju_atan2(vec[1], vec[0]);
}



// map to elevation angle in spherical coordinates
static mjtNum latitude(const mjtNum vec[3]) {
  return mju_atan2(mju_sqrt(vec[0]*vec[0] + vec[1]*vec[1]), vec[2]);
}



// eliminate geom
static int ray_eliminate(const mjModel* m, const mjData* d, int geomid,
                         const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude) {
  // body exclusion
  if (m->geom_bodyid[geomid] == bodyexclude) {
    return 1;
  }

  // invisible geom exclusion
  if (m->geom_matid[geomid] < 0 && m->geom_rgba[4*geomid+3] == 0) {
    return 1;
  }

  // invisible material exclusion
  if (m->geom_matid[geomid] >= 0 && m->mat_rgba[4*m->geom_matid[geomid]+3] == 0) {
    return 1;
  }

  // static exclusion
  if (!flg_static && m->body_weldid[m->geom_bodyid[geomid]] == 0) {
    return 1;
  }

  // no geomgroup inclusion
  if (!geomgroup) {
    return 0;
  }

  // group inclusion/exclusion
  int groupid = mjMIN(mjNGROUP-1, mjMAX(0, m->geom_group[geomid]));

  return (geomgroup[groupid] == 0);
}



// compute solution from quadratic:  a*x^2 + 2*b*x + c = 0
static mjtNum ray_quad(mjtNum a, mjtNum b, mjtNum c, mjtNum* x) {
  // compute determinant and check
  mjtNum det = b*b - a*c;
  if (det < mjMINVAL) {
    x[0] = -1;
    x[1] = -1;
    return -1;
  }
  det = mju_sqrt(det);

  // compute the two solutions
  x[0] = (-b-det)/a;
  x[1] = (-b+det)/a;

  // finalize result
  if (x[0] >= 0) {
    return x[0];
  } else if (x[1] >= 0) {
    return x[1];
  } else {
    return -1;
  }
}



// intersect ray with triangle
mjtNum ray_triangle(mjtNum v[][3], const mjtNum* lpnt, const mjtNum* lvec,
                    const mjtNum* b0, const mjtNum* b1) {
  // dif = v[i] - lpnt
  mjtNum dif[3][3];
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      dif[i][j] = v[i][j] - lpnt[j];
    }
  }

  // project difference vectors in normal plane
  mjtNum planar[3][2];
  for (int i=0; i < 3; i++) {
    planar[i][0] = mju_dot3(b0, dif[i]);
    planar[i][1] = mju_dot3(b1, dif[i]);
  }

  // reject if on the same side of any coordinate axis
  if ((planar[0][0] > 0 && planar[1][0] > 0 && planar[2][0] > 0) ||
      (planar[0][0] < 0 && planar[1][0] < 0 && planar[2][0] < 0) ||
      (planar[0][1] > 0 && planar[1][1] > 0 && planar[2][1] > 0) ||
      (planar[0][1] < 0 && planar[1][1] < 0 && planar[2][1] < 0)) {
    return -1;
  }

  // determine if origin is inside planar projection of triangle
  // A = (p0-p2, p1-p2), b = -p2, solve A*t = b
  mjtNum A[4] = {planar[0][0]-planar[2][0], planar[1][0]-planar[2][0],
                 planar[0][1]-planar[2][1], planar[1][1]-planar[2][1]};
  mjtNum b[2] = {-planar[2][0], -planar[2][1]};
  mjtNum det = A[0]*A[3] - A[1]*A[2];
  if (mju_abs(det) < mjMINVAL) {
    return -1;
  }
  mjtNum t0 = (A[3]*b[0] - A[1]*b[1]) / det;
  mjtNum t1 = (-A[2]*b[0] + A[0]*b[1]) / det;

  // check if outside
  if (t0 < 0 || t1 < 0|| t0+t1 > 1) {
    return -1;
  }

  // intersect ray with plane of triangle
  mju_sub3(dif[0], v[0], v[2]);       // v0-v2
  mju_sub3(dif[1], v[1], v[2]);       // v1-v2
  mju_sub3(dif[2], lpnt, v[2]);       // lp-v2
  mjtNum nrm[3];
  mju_cross(nrm, dif[0], dif[1]);     // normal to triangle plane
  mjtNum denom = mju_dot3(lvec, nrm);
  if (mju_abs(denom) < mjMINVAL) {
    return -1;
  }

  return (-mju_dot3(dif[2], nrm) / denom);
}

//---------------------------- geom-specific intersection functions --------------------------------

// plane
static mjtNum ray_plane(const mjtNum* pos, const mjtNum* mat, const mjtNum* size,
                        const mjtNum* pnt, const mjtNum* vec) {
  // map to local frame
  mjtNum lpnt[3], lvec[3];
  ray_map(pos, mat, pnt, vec, lpnt, lvec);

  // z-vec not pointing towards front face: reject
  if (lvec[2] > -mjMINVAL) {
    return -1;
  }

  // intersection with plane
  const mjtNum x = -lpnt[2]/lvec[2];
  if (x < 0) {
    return -1;
  }
  mjtNum p0 = lpnt[0] + x*lvec[0];
  mjtNum p1 = lpnt[1] + x*lvec[1];

  // accept only within rendered rectangle
  if ((size[0] <= 0 || mju_abs(p0) <= size[0]) &&
      (size[1] <= 0 || mju_abs(p1) <= size[1])) {
    return x;
  } else {
    return -1;
  }
}



// sphere
static mjtNum ray_sphere(const mjtNum* pos, const mjtNum* mat, mjtNum dist_sqr,
                         const mjtNum* pnt, const mjtNum* vec) {
  // (x*vec+pnt-pos)'*(x*vec+pnt-pos) = size[0]*size[0]
  mjtNum dif[3] = {pnt[0]-pos[0], pnt[1]-pos[1], pnt[2]-pos[2]};
  mjtNum a = vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2];
  mjtNum b = vec[0]*dif[0] + vec[1]*dif[1] + vec[2]*dif[2];
  mjtNum c = dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2] - dist_sqr;

  // solve a*x^2 + 2*b*x + c = 0
  mjtNum xx[2];
  return ray_quad(a, b, c, xx);
}



// capsule
static mjtNum ray_capsule(const mjtNum* pos, const mjtNum* mat, const mjtNum* size,
                          const mjtNum* pnt, const mjtNum* vec) {
  // bounding sphere test
  mjtNum ssz = size[0] + size[1];
  if (ray_sphere(pos, NULL, ssz*ssz, pnt, vec) < 0) {
    return -1;
  }

  // map to local frame
  mjtNum lpnt[3], lvec[3];
  ray_map(pos, mat, pnt, vec, lpnt, lvec);

  // init solution
  mjtNum x = -1, sol, xx[2];

  // cylinder round side: (x*lvec+lpnt)'*(x*lvec+lpnt) = size[0]*size[0]
  mjtNum a = lvec[0]*lvec[0] + lvec[1]*lvec[1];
  mjtNum b = lvec[0]*lpnt[0] + lvec[1]*lpnt[1];
  mjtNum c = lpnt[0]*lpnt[0] + lpnt[1]*lpnt[1] - size[0]*size[0];

  // solve a*x^2 + 2*b*x + c = 0
  sol = ray_quad(a, b, c, xx);

  // make sure round solution is between flat sides
  if (sol >= 0 && mju_abs(lpnt[2]+sol*lvec[2]) <= size[1]) {
    if (x < 0 || sol < x) {
      x = sol;
    }
  }

  // top cap
  mjtNum ldif[3] = {lpnt[0], lpnt[1], lpnt[2]-size[1]};
  a = lvec[0]*lvec[0] + lvec[1]*lvec[1] + lvec[2]*lvec[2];
  b = lvec[0]*ldif[0] + lvec[1]*ldif[1] + lvec[2]*ldif[2];
  c = ldif[0]*ldif[0] + ldif[1]*ldif[1] + ldif[2]*ldif[2] - size[0]*size[0];
  ray_quad(a, b, c, xx);

  // accept only top half of sphere
  for (int i=0; i < 2; i++) {
    if (xx[i] >= 0 && lpnt[2]+xx[i]*lvec[2] >= size[1]) {
      if (x < 0 || xx[i] < x) {
        x = xx[i];
      }
    }
  }

  // bottom cap
  ldif[2] = lpnt[2]+size[1];
  b = lvec[0]*ldif[0] + lvec[1]*ldif[1] + lvec[2]*ldif[2];
  c = ldif[0]*ldif[0] + ldif[1]*ldif[1] + ldif[2]*ldif[2] - size[0]*size[0];
  ray_quad(a, b, c, xx);

  // accept only bottom half of sphere
  for (int i=0; i < 2; i++) {
    if (xx[i] >= 0 && lpnt[2]+xx[i]*lvec[2] <= -size[1]) {
      if (x < 0 || xx[i] < x) {
        x = xx[i];
      }
    }
  }

  return x;
}



// ellipsoid
static mjtNum ray_ellipsoid(const mjtNum* pos, const mjtNum* mat, const mjtNum* size,
                            const mjtNum* pnt, const mjtNum* vec) {
  // map to local frame
  mjtNum lpnt[3], lvec[3];
  ray_map(pos, mat, pnt, vec, lpnt, lvec);

  // invert size^2
  mjtNum s[3] = {1/(size[0]*size[0]), 1/(size[1]*size[1]), 1/(size[2]*size[2])};

  // (x*lvec+lpnt)' * diag(1./size^2) * (x*lvec+lpnt) = 1
  mjtNum a = s[0]*lvec[0]*lvec[0] + s[1]*lvec[1]*lvec[1] + s[2]*lvec[2]*lvec[2];
  mjtNum b = s[0]*lvec[0]*lpnt[0] + s[1]*lvec[1]*lpnt[1] + s[2]*lvec[2]*lpnt[2];
  mjtNum c = s[0]*lpnt[0]*lpnt[0] + s[1]*lpnt[1]*lpnt[1] + s[2]*lpnt[2]*lpnt[2] - 1;

  // solve a*x^2 + 2*b*x + c = 0
  mjtNum xx[2];
  return ray_quad(a, b, c, xx);
}



// cylinder
static mjtNum ray_cylinder(const mjtNum* pos, const mjtNum* mat, const mjtNum* size,
                           const mjtNum* pnt, const mjtNum* vec) {
  // bounding sphere test
  mjtNum ssz = size[0]*size[0] + size[1]*size[1];
  if (ray_sphere(pos, NULL, ssz, pnt, vec) < 0) {
    return -1;
  }

  // map to local frame
  mjtNum lpnt[3], lvec[3];
  ray_map(pos, mat, pnt, vec, lpnt, lvec);

  // init solution
  mjtNum x = -1, sol;

  // flat sides
  int side;
  if (mju_abs(lvec[2]) > mjMINVAL) {
    for (side=-1; side <= 1; side+=2) {
      // solution of: lpnt[2] + x*lvec[2] = side*height_size
      sol = (side*size[1]-lpnt[2])/lvec[2];

      // process if non-negative
      if (sol >= 0) {
        // intersection with horizontal face
        mjtNum p0 = lpnt[0] + sol*lvec[0];
        mjtNum p1 = lpnt[1] + sol*lvec[1];

        // accept within radius
        if (p0*p0 + p1*p1 <= size[0]*size[0]) {
          if (x < 0 || sol < x) {
            x = sol;
          }
        }
      }
    }
  }

  // (x*lvec+lpnt)'*(x*lvec+lpnt) = size[0]*size[0]
  mjtNum a = lvec[0]*lvec[0] + lvec[1]*lvec[1];
  mjtNum b = lvec[0]*lpnt[0] + lvec[1]*lpnt[1];
  mjtNum c = lpnt[0]*lpnt[0] + lpnt[1]*lpnt[1] - size[0]*size[0];

  // solve a*x^2 + 2*b*x + c = 0
  mjtNum xx[2];
  sol = ray_quad(a, b, c, xx);

  // make sure round solution is between flat sides
  if (sol >= 0 && mju_abs(lpnt[2]+sol*lvec[2]) <= size[1]) {
    if (x < 0 || sol < x) {
      x = sol;
    }
  }

  return x;
}



// box
static mjtNum ray_box(const mjtNum* pos, const mjtNum* mat, const mjtNum* size,
                      const mjtNum* pnt, const mjtNum* vec, mjtNum* all) {
  // clear all
  if (all) {
    for (int i=0; i < 6; i++) {
      all[i] = -1;
    }
  }

  // bounding sphere test
  mjtNum ssz = size[0]*size[0] + size[1]*size[1] + size[2]*size[2];
  if (ray_sphere(pos, NULL, ssz, pnt, vec) < 0) {
    return -1;
  }

  // faces
  const int iface[3][2] = {
    {1, 2},
    {0, 2},
    {0, 1}
  };

  // map to local frame
  mjtNum lpnt[3], lvec[3];
  ray_map(pos, mat, pnt, vec, lpnt, lvec);

  // init solution
  mjtNum x = -1, sol;

  // loop over axes with non-zero vec
  for (int i=0; i < 3; i++) {
    if (mju_abs(lvec[i]) > mjMINVAL) {
      for (int side=-1; side <= 1; side+=2) {
        // solution of: lpnt[i] + x*lvec[i] = side*size[i]
        sol = (side*size[i]-lpnt[i])/lvec[i];

        // process if non-negative
        if (sol >= 0) {
          // intersection with face
          mjtNum p0 = lpnt[iface[i][0]] + sol*lvec[iface[i][0]];
          mjtNum p1 = lpnt[iface[i][1]] + sol*lvec[iface[i][1]];

          // accept within rectangle
          if (mju_abs(p0) <= size[iface[i][0]] &&
              mju_abs(p1) <= size[iface[i][1]]) {
            // update
            if (x < 0 || sol < x) {
              x = sol;
            }

            // save in all
            if (all) {
              all[2*i+(side+1)/2] = sol;
            }
          }
        }
      }
    }
  }

  return x;
}



// intersect ray with hfield
mjtNum mj_rayHfield(const mjModel* m, const mjData* d, int id,
                    const mjtNum* pnt, const mjtNum* vec) {
  // check geom type
  if (m->geom_type[id] != mjGEOM_HFIELD) {
    mjERROR("geom with hfield type expected");
  }

  // hfield id and dimensions
  int hid = m->geom_dataid[id];
  int nrow = m->hfield_nrow[hid];
  int ncol = m->hfield_ncol[hid];
  const mjtNum* size = m->hfield_size + 4*hid;
  const float* data = m->hfield_data + m->hfield_adr[hid];

  // compute size and pos of base box
  mjtNum base_size[3] = {size[0], size[1], size[3]*0.5};
  mjtNum base_pos[3] = {
    d->geom_xpos[3*id]   - d->geom_xmat[9*id+2]*size[3]*0.5,
    d->geom_xpos[3*id+1] - d->geom_xmat[9*id+5]*size[3]*0.5,
    d->geom_xpos[3*id+2] - d->geom_xmat[9*id+8]*size[3]*0.5
  };

  // compute size and pos of top box
  mjtNum top_size[3] = {size[0], size[1], size[2]*0.5};
  mjtNum top_pos[3] = {
    d->geom_xpos[3*id]   + d->geom_xmat[9*id+2]*size[2]*0.5,
    d->geom_xpos[3*id+1] + d->geom_xmat[9*id+5]*size[2]*0.5,
    d->geom_xpos[3*id+2] + d->geom_xmat[9*id+8]*size[2]*0.5
  };

  // init: intersection with base box
  mjtNum x = ray_box(base_pos, d->geom_xmat+9*id, base_size, pnt, vec, NULL);

  // check top box: done if no intersection
  mjtNum all[6];
  mjtNum top_intersect = ray_box(top_pos, d->geom_xmat+9*id, top_size, pnt, vec, all);
  if (top_intersect < 0) {
    return x;
  }

  // map to local frame
  mjtNum lpnt[3], lvec[3];
  ray_map(d->geom_xpos+3*id, d->geom_xmat+9*id, pnt, vec, lpnt, lvec);

  // construct basis vectors of normal plane
  mjtNum b0[3] = {1, 1, 1}, b1[3];
  if (mju_abs(lvec[0]) >= mju_abs(lvec[1]) && mju_abs(lvec[0]) >= mju_abs(lvec[2])) {
    b0[0] = 0;
  } else if (mju_abs(lvec[1]) >= mju_abs(lvec[2])) {
    b0[1] = 0;
  } else {
    b0[2] = 0;
  }
  mju_addScl3(b1, b0, lvec, -mju_dot3(lvec, b0)/mju_dot3(lvec, lvec));
  mju_normalize3(b1);
  mju_cross(b0, b1, lvec);
  mju_normalize3(b0);

  // find ray segment intersecting top box
  mjtNum seg[2] = {0, top_intersect};
  for (int i=0; i < 6; i++) {
    if (all[i] > seg[1]) {
      seg[0] = top_intersect;
      seg[1] = all[i];
    }
  }

  // project segment endpoints in horizontal plane, discretize
  mjtNum dx = (2.0*size[0]) / (ncol-1);
  mjtNum dy = (2.0*size[1]) / (nrow-1);
  mjtNum SX[2], SY[2];
  for (int i=0; i < 2; i++) {
    SX[i] = (lpnt[0] + seg[i]*lvec[0] + size[0]) / dx;
    SY[i] = (lpnt[1] + seg[i]*lvec[1] + size[1]) / dy;
  }

  // compute ranges, with +1 padding
  int cmin = mjMAX(0, (int)mju_floor(mjMIN(SX[0], SX[1]))-1);
  int cmax = mjMIN(ncol-1, (int)mju_ceil(mjMAX(SX[0], SX[1]))+1);
  int rmin = mjMAX(0, (int)mju_floor(mjMIN(SY[0], SY[1]))-1);
  int rmax = mjMIN(nrow-1, (int)mju_ceil(mjMAX(SY[0], SY[1]))+1);

  // check triangles within bounds
  for (int r=rmin; r < rmax; r++) {
    for (int c=cmin; c < cmax; c++) {
      // first triangle
      mjtNum va[3][3] = {
        {dx*c-size[0], dy*r-size[1], data[r*ncol+c]*size[2]},
        {dx*(c+1)-size[0], dy*(r+1)-size[1], data[(r+1)*ncol+(c+1)]*size[2]},
        {dx*(c+1)-size[0], dy*r-size[1], data[r*ncol+(c+1)]*size[2]}
      };
      mjtNum sol = ray_triangle(va, lpnt, lvec, b0, b1);
      if (sol >= 0 && (x < 0 || sol < x)) {
        x = sol;
      }

      // second triangle
      mjtNum vb[3][3] = {
        {dx*c-size[0], dy*r-size[1], data[r*ncol+c]*size[2]},
        {dx*(c+1)-size[0], dy*(r+1)-size[1], data[(r+1)*ncol+(c+1)]*size[2]},
        {dx*c-size[0], dy*(r+1)-size[1], data[(r+1)*ncol+c]*size[2]}
      };
      sol = ray_triangle(vb, lpnt, lvec, b0, b1);
      if (sol >= 0 && (x < 0 || sol < x)) {
        x = sol;
      }
    }
  }

  // check viable sides of top box
  for (int i=0; i < 4; i++) {
    if (all[i] >= 0 && (all[i] < x || x < 0)) {
      // normalized height of intersection point
      mjtNum z = (lpnt[2] + all[i]*lvec[2]) / size[2];

      // rectangle points
      mjtNum y, y0, z0, z1;

      // side normal to x-axis
      if (i < 2) {
        y = (lpnt[1] + all[i]*lvec[1] + size[1]) / dy;
        y0 = mjMAX(0, mjMIN(nrow-2, mju_floor(y)));
        z0 = (mjtNum)data[mju_round(y0)*nrow + (i == 1 ? ncol-1 : 0)];
        z1 = (mjtNum)data[mju_round(y0+1)*nrow + (i == 1 ? ncol-1 : 0)];
      }

      // side normal to y-axis
      else {
        y = (lpnt[0] + all[i]*lvec[0] + size[0]) / dx;
        y0 = mjMAX(0, mjMIN(ncol-2, mju_floor(y)));
        z0 = (mjtNum)data[mju_round(y0) + (i == 3 ? (nrow-1)*ncol : 0)];
        z1 = (mjtNum)data[mju_round(y0+1) + (i == 3 ? (nrow-1)*ncol : 0)];
      }

      // check if point is below line segment
      if (z < z0*(y0+1-y) + z1*(y-y0)) {
        x = all[i];
      }
    }
  }

  return x;
}



// ray vs axis-aligned bounding box using slab method
// see Ericson, Real-time Collision Detection section 5.3.3.
int mju_raySlab(const mjtNum aabb[6], const mjtNum xpos[3],
                const mjtNum xmat[9], const mjtNum* pnt, const mjtNum* vec) {
  mjtNum tmin = 0.0, tmax = INFINITY;

  // compute min and max
  mjtNum min[3] = {aabb[0]-aabb[3], aabb[1]-aabb[4], aabb[2]-aabb[5]};
  mjtNum max[3] = {aabb[0]+aabb[3], aabb[1]+aabb[4], aabb[2]+aabb[5]};

  // compute ray in local coordinates
  mjtNum src[3], dir[3];
  ray_map(xpos, xmat, pnt, vec, src, dir);

  // check intersections
  mjtNum invdir[3] = { 1.0 / dir[0], 1.0 / dir[1], 1.0 / dir[2] };
  for (int d = 0; d < 3; ++d) {
    mjtNum t1 = (min[d] - src[d]) * invdir[d];
    mjtNum t2 = (max[d] - src[d]) * invdir[d];
    mjtNum minval = t1 < t2 ? t1 : t2;
    mjtNum maxval = t1 < t2 ? t2 : t1;
    tmin = tmin > minval ? tmin : minval;
    tmax = tmax < maxval ? tmax : maxval;
  }

  return tmin < tmax;
}

// ray vs tree intersection
mjtNum mju_rayTree(const mjModel* m, const mjData* d, int id, const mjtNum* pnt,
                   const mjtNum* vec) {
  int mark_active = m->vis.global.bvactive;
  const int meshid = m->geom_dataid[id];
  const int bvhadr = m->mesh_bvhadr[meshid];
  const int* faceid = m->bvh_nodeid + bvhadr;
  const mjtNum* bvh = m->bvh_aabb + 6*bvhadr;
  const int* child = m->bvh_child + 2*bvhadr;

  if (meshid == -1) {
    mjERROR("mesh id of geom %d is -1", meshid);  // SHOULD NOT OCCUR
  }

  // initialize stack
  int stack[mjMAXTREEDEPTH];
  int nstack = 0;
  stack[nstack] = 0;
  nstack++;

  // map to local frame
  mjtNum lpnt[3], lvec[3];
  ray_map(d->geom_xpos+3*id, d->geom_xmat+9*id, pnt, vec, lpnt, lvec);

  // construct basis vectors of normal plane
  mjtNum b0[3] = {1, 1, 1}, b1[3];
  if (mju_abs(lvec[0]) >= mju_abs(lvec[1]) && mju_abs(lvec[0]) >= mju_abs(lvec[2])) {
    b0[0] = 0;
  } else if (mju_abs(lvec[1]) >= mju_abs(lvec[2])) {
    b0[1] = 0;
  } else {
    b0[2] = 0;
  }
  mju_addScl3(b1, b0, lvec, -mju_dot3(lvec, b0)/mju_dot3(lvec, lvec));
  mju_normalize3(b1);
  mju_cross(b0, b1, lvec);
  mju_normalize3(b0);

  // init solution
  mjtNum x = -1, sol;

  while (nstack) {
    // pop from stack
    nstack--;
    int node = stack[nstack];

    // intersection test
    int intersect = mju_raySlab(bvh+6*node, d->geom_xpos+3*id, d->geom_xmat+9*id, pnt, vec);

    // if no intersection, skip
    if (!intersect) {
      continue;
    }

    // node1 is a leaf
    if (faceid[node] != -1) {
      int face = faceid[node] + m->mesh_faceadr[meshid];

      // get float vertices
      float* vf[3];
      vf[0] = m->mesh_vert + 3*(m->mesh_face[3*face+0] + m->mesh_vertadr[meshid]);
      vf[1] = m->mesh_vert + 3*(m->mesh_face[3*face+1] + m->mesh_vertadr[meshid]);
      vf[2] = m->mesh_vert + 3*(m->mesh_face[3*face+2] + m->mesh_vertadr[meshid]);

      // convert to mjtNum
      mjtNum v[3][3];
      for (int i=0; i < 3; i++) {
        for (int j=0; j < 3; j++) {
          v[i][j] = (mjtNum)vf[i][j];
        }
      }

      // solve
      sol = ray_triangle(v, lpnt, lvec, b0, b1);

      // update
      if (sol >= 0 && (x < 0 || sol < x)) {
        x = sol;
        if (mark_active) {
          d->bvh_active[node + bvhadr] = 1;
        }
      }
      continue;
    }

    // used for rendering
    if (mark_active) {
      d->bvh_active[node + bvhadr] = 1;
    }

    // add children to the stack
    for (int i=0; i < 2; i++) {
      if (child[2*node+i] != -1) {
        if (nstack >= mjMAXTREEDEPTH) {
          mjERROR("BVH stack depth exceeded in geom %d.", id);
        }
        stack[nstack] = child[2*node+i];
        nstack++;
      }
    }
  }

  return x;
}



// intersect ray with signed distance field
mjtNum ray_sdf(const mjModel* m, const mjData* d, int g,
               const mjtNum* pnt, const mjtNum* vec) {
  mjtNum distance_total = 0;
  mjtNum p[3];
  mjtNum kMinDist = 1e-7;

  // exclude using bounding box
  if (ray_box(d->geom_xpos+3*g, d->geom_xmat+9*g, m->geom_size+3*g, pnt, vec, NULL) < 0) {
    return -1;
  }

  // get sdf
  int instance = m->geom_plugin[g];
  const int nslot = mjp_pluginCount();
  const int slot = m->plugin[instance];
  const mjpPlugin* sdf = mjp_getPluginAtSlotUnsafe(slot, nslot);
  if (!sdf) mjERROR("invalid plugin slot: %d", slot);
  if (!(sdf->capabilityflags & mjPLUGIN_SDF)) {
    mjERROR("Plugin is not a sign distance field at slot %d", slot);
  }

  // reset counter
  sdf->reset(m, NULL, (void*)(d->plugin_data[instance]), instance);

  // compute transformation
  mjtNum sdf_quat[4], sdf_xmat[9], sdf_xpos[9];
  mjtNum negpos[3], negquat[4], xquat[4];
  mjtNum* xpos = d->geom_xpos + 3*g;
  mjtNum* pos = m->mesh_pos + 3*m->geom_dataid[g];
  mjtNum* quat = m->mesh_quat + 4*m->geom_dataid[g];
  mju_mat2Quat(xquat, d->geom_xmat + 9*g);
  mju_negPose(negpos, negquat, pos, quat);
  mju_mulPose(sdf_xpos, sdf_quat, xpos, xquat, negpos, negquat);
  mju_quat2Mat(sdf_xmat, sdf_quat);

  // map to local frame
  mjtNum lpnt[3], lvec[3];
  ray_map(sdf_xpos, sdf_xmat, pnt, vec, lpnt, lvec);

  // unit direction
  mju_normalize3(lvec);

  // ray marching, see e.g. https://en.wikipedia.org/wiki/Ray_marching
  for (int i=0; i < 40; i++) {
    mju_addScl3(p, lpnt, lvec, distance_total);
    mjtNum distance = sdf->sdf_distance(p, (mjData*)d, instance);
    distance_total += distance;
    if (mju_abs(distance) < kMinDist) {
      return distance_total;
    }
    if (distance > 1e6) {
      // no intersection
      break;
    }
  }

  // reset counter
  sdf->reset(m, NULL, (void*)(d->plugin_data[instance]), instance);

  return -1;
}



// intersect ray with mesh
mjtNum mj_rayMesh(const mjModel* m, const mjData* d, int id,
                  const mjtNum* pnt, const mjtNum* vec) {
  // check geom type
  if (m->geom_type[id] != mjGEOM_MESH) {
    mjERROR("geom with mesh type expected");
  }

  // bounding box test
  if (ray_box(d->geom_xpos+3*id, d->geom_xmat+9*id, m->geom_size+3*id, pnt, vec, NULL) < 0) {
    return -1;
  }

  return mju_rayTree(m, d, id, pnt, vec);
}



// intersect ray with pure geom, no meshes or hfields
mjtNum mju_rayGeom(const mjtNum* pos, const mjtNum* mat, const mjtNum* size,
                   const mjtNum* pnt, const mjtNum* vec, int geomtype) {
  switch ((mjtGeom) geomtype) {
  case mjGEOM_PLANE:
    return ray_plane(pos, mat, size, pnt, vec);

  case mjGEOM_SPHERE:
    return ray_sphere(pos, mat, size[0]*size[0], pnt, vec);

  case mjGEOM_CAPSULE:
    return ray_capsule(pos, mat, size, pnt, vec);

  case mjGEOM_ELLIPSOID:
    return ray_ellipsoid(pos, mat, size, pnt, vec);

  case mjGEOM_CYLINDER:
    return ray_cylinder(pos, mat, size, pnt, vec);

  case mjGEOM_BOX:
    return ray_box(pos, mat, size, pnt, vec, NULL);

  default:
    mjERROR("unexpected geom type %d", geomtype);
    return -1;
  }
}



// intersect ray with flex, return nearest vertex id
mjtNum mju_rayFlex(const mjModel* m, const mjData* d, int flex_layer, mjtByte flg_vert,
                   mjtByte flg_edge, mjtByte flg_face, mjtByte flg_skin, int flexid,
                   const mjtNum* pnt, const mjtNum* vec, int vertid[1]) {
  int dim = m->flex_dim[flexid];

  // compute bounding box
  mjtNum box[3][2] = {{0, 0}, {0, 0}, {0, 0}};
  mjtNum* vert = d->flexvert_xpos + 3*m->flex_vertadr[flexid];
  for (int i=0; i < m->flex_vertnum[flexid]; i++) {
    for (int j=0; j < 3; j++) {
      // update minimum along side j
      if (box[j][0] > vert[3*i+j] || i == 0) {
        box[j][0] = vert[3*i+j];
      }

      // update maximum along side j
      if (box[j][1] < vert[3*i+j] || i == 0) {
        box[j][1] = vert[3*i+j];
      }
    }
  }

  // adjust box for radius
  mjtNum radius = m->flex_radius[flexid];
  for (int j=0; j < 3; j++) {
    box[j][0] -= radius;
    box[j][1] += radius;
  }

  // construct box geom
  mjtNum pos[3], size[3], mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (int j=0; j < 3; j++) {
    pos[j] = 0.5*(box[j][0]+box[j][1]);
    size[j] = 0.5*(box[j][1]-box[j][0]);
  }

  // apply bounding-box filter
  if (ray_box(pos, mat, size, pnt, vec, NULL) < 0) {
    return -1;
  }

  // construct basis vectors of normal plane
  mjtNum b0[3] = {1, 1, 1}, b1[3];
  if (mju_abs(vec[0]) >= mju_abs(vec[1]) && mju_abs(vec[0]) >= mju_abs(vec[2])) {
    b0[0] = 0;
  } else if (mju_abs(vec[1]) >= mju_abs(vec[2])) {
    b0[1] = 0;
  } else {
    b0[2] = 0;
  }
  mju_addScl3(b1, b0, vec, -mju_dot3(vec, b0)/mju_dot3(vec, vec));
  mju_normalize3(b1);
  mju_cross(b0, b1, vec);
  mju_normalize3(b0);

  // init solution
  mjtNum x = -1;

  // check edges if rendered, or if skin
  if (flg_edge || (dim > 1 && flg_skin)) {
    int edge_end = m->flex_edgeadr[flexid]+m->flex_edgenum[flexid];
    for (int e=m->flex_edgeadr[flexid]; e < edge_end; e++) {
      // get vertices for this edge
      mjtNum* v1 = d->flexvert_xpos + 3*(m->flex_vertadr[flexid]+m->flex_edge[2*e]);
      mjtNum* v2 = d->flexvert_xpos + 3*(m->flex_vertadr[flexid]+m->flex_edge[2*e+1]);

      // construct capsule geom
      mju_add3(pos, v1, v2);
      mju_scl3(pos, pos, 0.5);
      mjtNum dif[3] = {v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2]};
      size[0] = radius;
      size[1] = 0.5*mju_normalize3(dif);
      mjtNum quat[4];
      mju_quatZ2Vec(quat, dif);
      mju_quat2Mat(mat, quat);

      // intersect ray with capsule
      mjtNum sol = mju_rayGeom(pos, mat, size, pnt, vec, mjGEOM_CAPSULE);

      // update
      if (sol >= 0 && (x < 0 || sol < x)) {
        x = sol;

        // construct intersection point
        mjtNum intersect[3];
        mju_addScl3(intersect, pnt, vec, sol);

        // find nearest vertex
        if (mju_dist3(v1, intersect) < mju_dist3(v2, intersect)) {
          *vertid = m->flex_edge[2*e];
        }
        else {
          *vertid = m->flex_edge[2*e+1];
        }
      }
    }
  }

  // check vertices if rendered (and edges not checked)
  else if (flg_vert && !(dim > 1 && flg_skin)) {
    for (int v=0; v < m->flex_vertnum[flexid]; v++) {
      // get vertex
      mjtNum* vpos = d->flexvert_xpos + 3*(m->flex_vertadr[flexid] + v);

      // construct sphere geom
      size[0] = radius;

      // intersect ray with sphere
      mjtNum sol = mju_rayGeom(vpos, NULL, size, pnt, vec, mjGEOM_SPHERE);

      // update
      if (sol >= 0 && (x < 0 || sol < x)) {
        x = sol;
        *vertid = v;
      }
    }
  }

  // check faces if rendered
  if (dim > 1 && (flg_face || flg_skin)) {
    for (int e=0; e < m->flex_elemnum[flexid]; e++) {
      // skip if 3D element is not visible
      int elayer = m->flex_elemlayer[m->flex_elemadr[flexid]+e];
      if (dim == 3 && ((flg_skin && elayer > 0) || (!flg_skin && elayer != flex_layer))) {
        continue;
      }

      // get element data
      const int* edata = m->flex_elem + m->flex_elemdataadr[flexid] + e*(dim+1);
      mjtNum* v1 = d->flexvert_xpos + 3*(m->flex_vertadr[flexid] + edata[0]);
      mjtNum* v2 = d->flexvert_xpos + 3*(m->flex_vertadr[flexid] + edata[1]);
      mjtNum* v3 = d->flexvert_xpos + 3*(m->flex_vertadr[flexid] + edata[2]);
      mjtNum* v4 = dim == 2 ? NULL : d->flexvert_xpos + 3*(m->flex_vertadr[flexid] + edata[3]);
      mjtNum* vptr[4][3] = {{v1, v2, v3}, {v1, v2, v4}, {v1, v3, v4}, {v2, v3, v4}};
      int vid[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};

      // process triangles of this element
      for (int i=0; i < (dim == 2?1:4); i++) {
        // copy vertices into triangle representation
        mjtNum v[3][3];
        for (int j=0; j < 3; j++)
          mju_copy3(v[j], vptr[i][j]);

        // intersect ray with triangle
        mjtNum sol = ray_triangle(v, pnt, vec, b0, b1);

        // update
        if (sol >= 0 && (x < 0 || sol < x)) {
          x = sol;

          // construct intersection point
          mjtNum intersect[3];
          mju_addScl3(intersect, pnt, vec, sol);

          // find nearest vertex
          mjtNum dist[3] = {
            mju_dist3(v[0], intersect),
            mju_dist3(v[1], intersect),
            mju_dist3(v[2], intersect)
          };
          if (dist[0] <= dist[1] && dist[0] <= dist[2]) {
            *vertid = edata[vid[i][0]];
          } else if (dist[1] <= dist[2]){
            *vertid = edata[vid[i][1]];
          } else {
            *vertid = edata[vid[i][2]];
          }
        }
      }
    }
  }

  return x;
}



// intersect ray with skin, return nearest vertex id
mjtNum mju_raySkin(int nface, int nvert, const int* face, const float* vert,
                   const mjtNum* pnt, const mjtNum* vec, int vertid[1]) {
  // compute bounding box
  mjtNum box[3][2] = {{0, 0}, {0, 0}, {0, 0}};
  for (int i=0; i < nvert; i++) {
    for (int j=0; j < 3; j++) {
      // update minimum along side j
      if (box[j][0] > vert[3*i+j] || i == 0) {
        box[j][0] = vert[3*i+j];
      }

      // update maximum along side j
      if (box[j][1] < vert[3*i+j] || i == 0) {
        box[j][1] = vert[3*i+j];
      }
    }
  }

  // construct box geom
  mjtNum pos[3], size[3], mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (int j=0; j < 3; j++) {
    pos[j] = 0.5*(box[j][0]+box[j][1]);
    size[j] = 0.5*(box[j][1]-box[j][0]);
  }

  // apply bounding-box filter
  if (ray_box(pos, mat, size, pnt, vec, NULL) < 0) {
    return -1;
  }

  // construct basis vectors of normal plane
  mjtNum b0[3] = {1, 1, 1}, b1[3];
  if (mju_abs(vec[0]) >= mju_abs(vec[1]) && mju_abs(vec[0]) >= mju_abs(vec[2])) {
    b0[0] = 0;
  } else if (mju_abs(vec[1]) >= mju_abs(vec[2])) {
    b0[1] = 0;
  } else {
    b0[2] = 0;
  }
  mju_addScl3(b1, b0, vec, -mju_dot3(vec, b0)/mju_dot3(vec, vec));
  mju_normalize3(b1);
  mju_cross(b0, b1, vec);
  mju_normalize3(b0);

  // init solution
  mjtNum x = -1;

  // process all faces
  for (int i=0; i < nface; i++) {
    // get float vertices
    const float* vf[3];
    vf[0] = vert + 3*(face[3*i]);
    vf[1] = vert + 3*(face[3*i+1]);
    vf[2] = vert + 3*(face[3*i+2]);

    // convert to mjtNum
    mjtNum v[3][3];
    for (int j=0; j < 3; j++) {
      for (int k=0; k < 3; k++) {
        v[j][k] = (mjtNum)vf[j][k];
      }
    }

    // solve
    mjtNum sol = ray_triangle(v, pnt, vec, b0, b1);

    // update
    if (sol >= 0 && (x < 0 || sol < x)) {
      x = sol;

      // construct intersection point
      mjtNum intersect[3];
      mju_addScl3(intersect, pnt, vec, sol);

      // find nearest vertex
      mjtNum dist = mju_dist3(intersect, v[0]);
      *vertid = face[3*i];
      for (int j=1; j < 3; j++) {
        mjtNum newdist = mju_dist3(intersect, v[j]);
        if (newdist < dist) {
          dist = newdist;
          *vertid = face[3*i+j];
        }
      }
    }
  }

  return x;
}



// return 1 if point is inside object-aligned bounding box, 0 otherwise
static int point_in_box(const mjtNum aabb[6], const mjtNum xpos[3],
                        const mjtNum xmat[9], const mjtNum pnt[3]) {
  mjtNum point[3];

  // compute point in local coordinates of the box
  mju_sub3(point, pnt, xpos);
  mju_mulMatTVec3(point, xmat, point);
  mju_subFrom3(point, aabb);

  // check intersections
  for (int j=0; j < 3; j++) {  // directions
    if (mju_abs(point[j]) > aabb[3+j]) {
      return 0;
    }
  }

  return 1;
}



//---------------------------- main entry point ----------------------------------------------------

// intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms on bodyexclude
//  return geomid and distance (x) to nearest surface, or -1 if no intersection
//  geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion
mjtNum mj_ray(const mjModel* m, const mjData* d, const mjtNum* pnt, const mjtNum* vec,
              const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude, int geomid[1]) {
  mjtNum dist, newdist;

  // check vector length
  if (mju_norm3(vec) < mjMINVAL) {
    mjERROR("vector length is too small");
  }

  // clear result
  dist = -1;
  if (geomid) *geomid = -1;

  // loop over geoms not eliminated by mask and bodyexclude
  for (int i=0; i < m->ngeom; i++) {
    if (!ray_eliminate(m, d, i, geomgroup, flg_static, bodyexclude)) {
      // handle mesh and hfield separately
      if (m->geom_type[i] == mjGEOM_MESH) {
        newdist = mj_rayMesh(m, d, i, pnt, vec);
      } else if (m->geom_type[i] == mjGEOM_HFIELD) {
        newdist = mj_rayHfield(m, d, i, pnt, vec);
      } else if (m->geom_type[i] == mjGEOM_SDF) {
        newdist = ray_sdf(m, d, i, pnt, vec);
      }

      // otherwise general dispatch
      else {
        newdist = mju_rayGeom(d->geom_xpos+3*i, d->geom_xmat+9*i,
                              m->geom_size+3*i, pnt, vec, m->geom_type[i]);
      }

      // update if closer intersection found
      if (newdist >= 0 && (newdist < dist || dist < 0)) {
        dist = newdist;
        if (geomid) *geomid = i;
      }
    }
  }

  return dist;
}


// Initializes spherical bounding angles (geom_ba) and flag vector for a given source
void mju_multiRayPrepare(const mjModel* m, const mjData* d, const mjtNum pnt[3],
                         const mjtNum* ray_xmat, const mjtByte* geomgroup, mjtByte flg_static,
                         int bodyexclude, mjtNum cutoff, mjtNum* geom_ba, int* geom_eliminate) {
  if (ray_xmat) {
    mjERROR("ray_xmat is currently unused, should be NULL");
  }

  // compute eliminate flag for all geoms
  for (int geomid=0; geomid < m->ngeom; geomid++)
    geom_eliminate[geomid] = ray_eliminate(m, d, geomid, geomgroup, flg_static, bodyexclude);

  for (int b=0; b < m->nbody; b++) {
    // skip precomputation if no bounding volume is available
    if (m->body_bvhadr[b] == -1) {
      continue;
    }

    // loop over child geoms, compute bounding angles
    for (int i=0; i < m->body_geomnum[b]; i++) {
      int g = i + m->body_geomadr[b];
      mjtNum AABB[4] = {mjMAXVAL, mjMAXVAL, -mjMAXVAL, -mjMAXVAL};
      mjtNum* aabb = m->geom_aabb + 6*g;
      mjtNum* xpos = d->geom_xpos + 3*g;
      mjtNum* xmat = d->geom_xmat + 9*g;

      // skip if eliminated by flags
      if (geom_eliminate[g]) {
        continue;
      }

      // add to geom_eliminate if distance of bounding sphere is above cutoff
      if (mju_dist3(d->geom_xpos+3*g, pnt) > cutoff+m->geom_rbound[g]) {
        geom_eliminate[g] = 1;
        continue;
      }

      if (point_in_box(aabb, xpos, xmat, pnt)) {
        (geom_ba+4*g)[0] = -mjPI;
        (geom_ba+4*g)[1] = 0;
        (geom_ba+4*g)[2] = mjPI;
        (geom_ba+4*g)[3] = mjPI;
        continue;
      }

      // loop over box vertices, compute spherical aperture
      for (int v=0; v < 8; v++) {
        mjtNum vert[3], box[3];
        vert[0] = (v&1 ? aabb[0]+aabb[3] : aabb[0]-aabb[3]);
        vert[1] = (v&2 ? aabb[1]+aabb[4] : aabb[1]-aabb[4]);
        vert[2] = (v&4 ? aabb[2]+aabb[5] : aabb[2]-aabb[5]);

        // rotate to the world frame
        mju_mulMatVec3(box, xmat, vert);
        mju_addTo3(box, xpos);

        // spherical coordinates
        mju_sub3(vert, box, pnt);
        mjtNum azimuth = longitude(vert);
        mjtNum elevation = latitude(vert);

        // update bounds
        AABB[0] = mju_min(AABB[0], azimuth);
        AABB[1] = mju_min(AABB[1], elevation);
        AABB[2] = mju_max(AABB[2], azimuth);
        AABB[3] = mju_max(AABB[3], elevation);
      }

      if (AABB[2]-AABB[0] > mjPI) {
        AABB[0] = -mjPI;
        AABB[1] = 0;
        AABB[2] =  mjPI;
        AABB[3] =  mjPI;
      }

      if (AABB[3]-AABB[1] > mjPI) {  // SHOULD NOT OCCUR
        mjERROR("discontinuity in azimuth angle");
      }

      mju_copy(geom_ba+4*g, AABB, 4);
    }
  }
}


// Performs single ray intersection
static mjtNum mju_singleRay(const mjModel* m, mjData* d, const mjtNum pnt[3], const mjtNum vec[3],
                            int* ray_eliminate, mjtNum* geom_ba, int geomid[1]) {
  mjtNum dist, newdist;

  // check vector length
  if (mju_norm3(vec) < mjMINVAL) {
    mjERROR("vector length is too small");
  }

  // clear result
  dist = -1;
  *geomid = -1;

  // get ray spherical coordinates
  mjtNum azimuth = longitude(vec);
  mjtNum elevation = latitude(vec);

  // loop over bodies not eliminated by bodyexclude
  for (int b=0; b < m->nbody; b++) {
    // exclude body using bounding sphere test
    if (m->body_bvhadr[b] != -1) {
      mjtNum* pos = m->bvh_aabb + 6*m->body_bvhadr[b];
      mjtNum center[3];
      mjtNum* size = pos + 3;
      mjtNum ssz = size[0]*size[0] + size[1]*size[1] + size[2]*size[2];
      mju_add3(center, pos, d->xipos+3*b);
      if (ray_sphere(center, NULL, ssz, pnt, vec) < 0) {
        continue;
      }
    }

    // loop over geoms if bounding sphere test fails
    for (int g=0; g < m->body_geomnum[b]; g++) {
      int i = m->body_geomadr[b] + g;
      if (ray_eliminate[i]) {
        continue;
      }

      // exclude geom using bounding angles
      if (m->body_bvhadr[b] != -1) {
        if (azimuth < (geom_ba+4*i)[0] || elevation < (geom_ba+4*i)[1] ||
            azimuth > (geom_ba+4*i)[2] || elevation > (geom_ba+4*i)[3]) {
          continue;
        }
      }

      // handle mesh and hfield separately
      if (m->geom_type[i] == mjGEOM_MESH) {
        newdist = mj_rayMesh(m, d, i, pnt, vec);
      } else if (m->geom_type[i] == mjGEOM_HFIELD) {
        newdist = mj_rayHfield(m, d, i, pnt, vec);
      } else if (m->geom_type[i] == mjGEOM_SDF) {
        newdist = ray_sdf(m, d, i, pnt, vec);
      }

      // otherwise general dispatch
      else {
        newdist = mju_rayGeom(d->geom_xpos+3*i, d->geom_xmat+9*i,
                              m->geom_size+3*i, pnt, vec, m->geom_type[i]);
      }

      // update if closer intersection found
      if (newdist >= 0 && (newdist < dist || dist < 0)) {
        dist = newdist;
        *geomid = i;
      }
    }
  }

  return dist;
}


// Performs multiple ray intersections with the precomputes bv and flags
void mj_multiRay(const mjModel* m, mjData* d, const mjtNum pnt[3], const mjtNum* vec,
                 const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude,
                 int* geomid, mjtNum* dist, int nray, mjtNum cutoff) {
  mj_markStack(d);

  // allocate source
  mjtNum* geom_ba = mjSTACKALLOC(d, 4*m->ngeom, mjtNum);
  int* geom_eliminate = mjSTACKALLOC(d, m->ngeom, int);

  // initialize source
  mju_multiRayPrepare(m, d, pnt, NULL, geomgroup, flg_static, bodyexclude,
                      cutoff, geom_ba, geom_eliminate);

  // loop over rays
  for (int i=0; i < nray; i++) {
    dist[i] = mju_singleRay(m, d, pnt, vec+3*i, geom_eliminate, geom_ba, geomid+i);
  }

  mj_freeStack(d);
}
