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

// Continuous-collision geometry for the IPC integrator; see the header for scope.

#include "engine/engine_collision_continuous.h"

#include <math.h>
#include <stddef.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>
#include "engine/engine_core_util.h"     // mj_local2Global (anchor body-local -> world)
#include "engine/engine_util_blas.h"     // mju_dot3, mju_mulMatVec3
#include "engine/engine_util_spatial.h"  // mju_cross
#include "engine/engine_util_errmem.h"   // mju_malloc, mju_free

static mjtNum g_ccd_l = 0, g_ccd_g0 = 0;  // which candidate set alpha, its rate/gap

static inline mjtNum min2(mjtNum a, mjtNum b) { return a < b ? a : b; }
static int g_ccd_cap = -1;

// point-triangle: distance, closest point cp, barycentric weights w of cp (for the barrier
// gradient).
// TODO(consolidation): same closest-point-on-triangle math as mjraw_SphereTriangle
// (engine_collision_primitive.c) and GJK's S2D -- NOT a duplicate by accident. The barrier
// gradient/Hessian needs the BARYCENTRIC weights w (to spread the reaction onto the 3 verts), which
// mjraw_SphereTriangle never forms (it finds the closest 3D point in a rotated frame, no w), and it
// early-exits on margin so it can't return the unconditional signed distance IPC uses in
// broadphase/CCD. Reuse = factor the core out of mjraw_SphereTriangle AND add a barycentric output
// (a collision-core change), not a drop-in arg add; deferred (regression-risky, small payoff).
mjtNum mjc_PtTri(const mjtNum* p, const mjtNum* a, const mjtNum* b, const mjtNum* c, mjtNum* cp,
                 mjtNum* w) {
  mjtNum ab[3], ac[3], ap[3];
  for (int k = 0; k < 3; k++) {
    ab[k] = b[k] - a[k];
    ac[k] = c[k] - a[k];
    ap[k] = p[k] - a[k];
  }
  mjtNum d1 = mju_dot3(ab, ap), d2 = mju_dot3(ac, ap);
  if (d1 <= 0 && d2 <= 0) {
    w[0] = 1;
    w[1] = 0;
    w[2] = 0;
  } else {
    mjtNum bp[3];
    for (int k = 0; k < 3; k++) bp[k] = p[k] - b[k];
    mjtNum d3 = mju_dot3(ab, bp), d4 = mju_dot3(ac, bp);
    if (d3 >= 0 && d4 <= d3) {
      w[0] = 0;
      w[1] = 1;
      w[2] = 0;
    } else {
      mjtNum vc = d1 * d4 - d3 * d2;
      if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        mjtNum t = d1 / (d1 - d3);
        w[0] = 1 - t;
        w[1] = t;
        w[2] = 0;
      } else {
        mjtNum cq[3];
        for (int k = 0; k < 3; k++) cq[k] = p[k] - c[k];
        mjtNum d5 = mju_dot3(ab, cq), d6 = mju_dot3(ac, cq);
        if (d6 >= 0 && d5 <= d6) {
          w[0] = 0;
          w[1] = 0;
          w[2] = 1;
        } else {
          mjtNum vb = d5 * d2 - d1 * d6;
          if (vb <= 0 && d2 >= 0 && d6 <= 0) {
            mjtNum t = d2 / (d2 - d6);
            w[0] = 1 - t;
            w[1] = 0;
            w[2] = t;
          } else {
            mjtNum va = d3 * d6 - d5 * d4;
            if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
              mjtNum t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
              w[0] = 0;
              w[1] = 1 - t;
              w[2] = t;
            } else {
              mjtNum den = 1.0 / (va + vb + vc), t = vb * den, u = vc * den;
              w[0] = 1 - t - u;
              w[1] = t;
              w[2] = u;
            }
          }
        }
      }
    }
  }
  for (int k = 0; k < 3; k++) cp[k] = w[0] * a[k] + w[1] * b[k] + w[2] * c[k];
  mjtNum dd[3];
  for (int k = 0; k < 3; k++) dd[k] = p[k] - cp[k];
  return sqrt(mju_dot3(dd, dd));
}

// closest distance between segment p1p2 and segment q1q2; closest points cp1, cp2 and the segment
// parameters st = {s, t} (cp1 = p1+s*(p2-p1), cp2 = q1+t*(q2-q1)).
// TODO(consolidation): same segment-segment math as mjraw_CapsuleCapsule's NON-parallel branch
// (engine_collision_primitive.c:442-470, identical x1/x2 clip). Kept separate because IPC returns
// (s,t) (mjraw_ discards them into mjraw_SphereSphere), the PARALLEL branch diverges (mjraw_ emits
// up to TWO contacts at the overlap ends -- a contact-gen stability trick -- vs IPC's one pair +
// mollifier), and mjraw_ early-exits on margin. Reuse needs a non-static closest-point primitive
// factored out, not an arg add; deferred.
// TODO(mollifier): no parallel-edge mollifier yet -- the near-parallel branch falls back to s=0
// (fine for non-parallel crossings) but gives a DISCONTINUOUS edge-edge gap gradient the Newton
// step feels. Add the Li-et-al EE mollifier (smooth vanish as |e1 x e2| -> 0) if/when near-parallel
// EE contacts (folded/stacked cloth) start to bite.
mjtNum mjc_SegSeg(const mjtNum* p1, const mjtNum* p2, const mjtNum* q1, const mjtNum* q2,
                  mjtNum* cp1, mjtNum* cp2, mjtNum* st) {
  mjtNum d1[3], d2[3], rr[3];
  for (int k = 0; k < 3; k++) {
    d1[k] = p2[k] - p1[k];
    d2[k] = q2[k] - q1[k];
    rr[k] = p1[k] - q1[k];
  }
  mjtNum a = mju_dot3(d1, d1), e = mju_dot3(d2, d2), fq = mju_dot3(d2, rr);
  mjtNum s, t;
  if (a <= 1e-12 && e <= 1e-12) {
    s = 0;
    t = 0;
  } else if (a <= 1e-12) {
    s = 0;
    t = fq / e;
  } else {
    mjtNum c = mju_dot3(d1, rr);
    if (e <= 1e-12) {
      t = 0;
      s = -c / a;
    } else {
      mjtNum b = mju_dot3(d1, d2), den = a * e - b * b;
      s = (den > 1e-12) ? (b * fq - c * e) / den
                        : 0.0;  // parallel: s=0 (mollifier territory, deferred)
      s = (s < 0) ? 0 : (s > 1 ? 1 : s);
      t = (b * s + fq) / e;
      if (t < 0) {
        t = 0;
        s = -c / a;
      } else if (t > 1) {
        t = 1;
        s = (b - c) / a;
      }
    }
  }
  s = (s < 0) ? 0 : (s > 1 ? 1 : s);
  t = (t < 0) ? 0 : (t > 1 ? 1 : t);
  st[0] = s;
  st[1] = t;
  for (int k = 0; k < 3; k++) {
    cp1[k] = p1[k] + s * d1[k];
    cp2[k] = q1[k] + t * d2[k];
  }
  mjtNum dd[3];
  for (int k = 0; k < 3; k++) dd[k] = cp1[k] - cp2[k];
  return sqrt(mju_dot3(dd, dd));
}

// closest point (out) on a convex polygon face to point p: project onto the face plane; if the
// projection is inside the polygon use it, else clamp to the nearest boundary edge. pv = the face's
// mesh-local vertex indices (nv of them) into the vertex array vbase; nrm = the face's plane
// normal. IPC-specific: helper for mjc_GeomDist's convex-mesh SDF branch; the engine has no
// point-on-convex-face util.
static void closestOnPoly(const mjtNum* p, const float* vbase, const int* pv, int nv,
                          const mjtNum* nrm, mjtNum* out) {
  const float* a0 = vbase + 3 * pv[0];
  mjtNum dpl = nrm[0] * (p[0] - a0[0]) + nrm[1] * (p[1] - a0[1]) + nrm[2] * (p[2] - a0[2]);
  mjtNum pp[3];
  for (int k = 0; k < 3; k++) pp[k] = p[k] - dpl * nrm[k];  // projection onto face plane
  int npos = 0, nneg = 0;
  for (int i = 0; i < nv; i++) {
    const float* a = vbase + 3 * pv[i];
    const float* b = vbase + 3 * pv[(i + 1) % nv];
    mjtNum e[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]},
           w[3] = {pp[0] - a[0], pp[1] - a[1], pp[2] - a[2]};
    mjtNum cr = (e[1] * w[2] - e[2] * w[1]) * nrm[0] + (e[2] * w[0] - e[0] * w[2]) * nrm[1] +
                (e[0] * w[1] - e[1] * w[0]) * nrm[2];
    if (cr > 0)
      npos++;
    else
      nneg++;
  }
  if (npos == 0 || nneg == 0) {
    for (int k = 0; k < 3; k++) out[k] = pp[k];
    return;
  }                    // inside polygon
  mjtNum best = 1e30;  // clamp to edges
  for (int i = 0; i < nv; i++) {
    const float* a = vbase + 3 * pv[i];
    const float* b = vbase + 3 * pv[(i + 1) % nv];
    mjtNum e[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]},
           w[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
    mjtNum t = (e[0] * w[0] + e[1] * w[1] + e[2] * w[2]) /
               (e[0] * e[0] + e[1] * e[1] + e[2] * e[2] + 1e-18);
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    mjtNum c[3] = {a[0] + t * e[0], a[1] + t * e[1], a[2] + t * e[2]};
    mjtNum d2 = (p[0] - c[0]) * (p[0] - c[0]) + (p[1] - c[1]) * (p[1] - c[1]) +
                (p[2] - c[2]) * (p[2] - c[2]);
    if (d2 < best) {
      best = d2;
      for (int k = 0; k < 3; k++) out[k] = c[k];
    }
  }
}

// signed distance from a static geom's surface to world point x (positive outside) + outward unit
// normal n. Closed-form for plane/sphere/capsule/box; returns +large (no contact) for other types.
// IPC-specific: the flex-vs-static-geom AL barrier needs a differentiable POINT-to-geom SDF;
// mj_geomDistance is geom-vs-geom (can't take a flex vertex) and mjc_distance/mjc_gradient need an
// mjSDF object + miss convex meshes.
mjtNum mjc_GeomDist(const mjModel* m, int gi, const mjtNum* gpos, const mjtNum* gmat,
                    const mjtNum* x, mjtNum* n, mjtNum cutoff) {
  int type = m->geom_type[gi];
  const mjtNum* size = m->geom_size + 3 * gi;
  mjtNum dx[3];
  for (int k = 0; k < 3; k++) dx[k] = x[k] - gpos[k];
  if (type == mjGEOM_MESH) {
    // point vs CONVEX-hull mesh, using MuJoCo's precomputed convex polygon faces. First the max
    // signed face-plane distance (cheap; gives the inside test and a far/cheap reject). If the
    // point is INSIDE (maxd<=0) return maxd with the max-face normal. If clearly FAR (maxd large)
    // return maxd (barrier is off there, normal direction irrelevant). Only NEAR the surface do the
    // proper point-to-convex-hull CLOSEST POINT (min over faces of the clamped projection) ->
    // correct distance AND a normal that points at the actual nearest surface (no max-plane normal
    // chatter).
    int meshid = m->geom_dataid[gi];
    const float* vbase = m->mesh_vert + 3 * m->mesh_vertadr[meshid];
    int polyadr = m->mesh_polyadr[meshid], pn = m->mesh_polynum[meshid];
    if (pn <= 0) {
      n[0] = 0;
      n[1] = 0;
      n[2] = 1;
      return 1e30;
    }
    mjtNum pl[3];
    mju_mulMatTVec3(pl, gmat, dx);  // world -> mesh-local
    mjtNum maxd = -1e30;
    const mjtNum* bestn = m->mesh_polynormal + 3 * polyadr;
    for (int p = 0; p < pn; p++) {
      const mjtNum* pnl = m->mesh_polynormal + 3 * (polyadr + p);
      const float* v0 = vbase + 3 * m->mesh_polyvert[m->mesh_polyvertadr[polyadr + p]];
      mjtNum c = pnl[0] * v0[0] + pnl[1] * v0[1] + pnl[2] * v0[2];
      mjtNum dd = pnl[0] * pl[0] + pnl[1] * pl[1] + pnl[2] * pl[2] - c;
      if (dd > maxd) {
        maxd = dd;
        bestn = pnl;
      }
    }
    if (maxd <= 0) {
      mju_mulMatVec3(n, gmat, bestn);
      return maxd;
    }  // inside: penetration
    // far reject: maxd (max signed plane distance) is a LOWER bound on the true distance for a
    // convex hull, so if it already exceeds the cutoff the closest-point search can't bring it into
    // range -- skip the O(faces) loop (this is what makes the per-vertex broadphase against a mesh
    // affordable).
    if (maxd > cutoff) {
      mju_mulMatVec3(n, gmat, bestn);
      return maxd;
    }
    mjtNum best = 1e30, bc[3] = {0, 0, 0};  // outside: true closest point
    for (int p = 0; p < pn; p++) {
      const mjtNum* pnl = m->mesh_polynormal + 3 * (polyadr + p);
      const int* pv = m->mesh_polyvert + m->mesh_polyvertadr[polyadr + p];
      int nv = m->mesh_polyvertnum[polyadr + p];
      mjtNum cc[3];
      closestOnPoly(pl, vbase, pv, nv, pnl, cc);
      mjtNum d2 = (pl[0] - cc[0]) * (pl[0] - cc[0]) + (pl[1] - cc[1]) * (pl[1] - cc[1]) +
                  (pl[2] - cc[2]) * (pl[2] - cc[2]);
      if (d2 < best) {
        best = d2;
        bc[0] = cc[0];
        bc[1] = cc[1];
        bc[2] = cc[2];
      }
    }
    mjtNum dist = sqrt(best);
    mjtNum nl[3];
    for (int k = 0; k < 3; k++) nl[k] = (dist > 1e-12) ? (pl[k] - bc[k]) / dist : bestn[k];
    mju_mulMatVec3(n, gmat, nl);  // outward normal (point - closest) -> world
    return dist;
  }
  if (type == mjGEOM_PLANE) {
    n[0] = gmat[2];
    n[1] = gmat[5];
    n[2] = gmat[8];  // local +z axis in world coords
    return mju_dot3(n, dx);
  }
  if (type == mjGEOM_SPHERE) {
    mjtNum L = sqrt(mju_dot3(dx, dx));
    if (L < 1e-12) {
      n[0] = 0;
      n[1] = 0;
      n[2] = 1;
      return -size[0];
    }
    for (int k = 0; k < 3; k++) n[k] = dx[k] / L;
    return L - size[0];
  }
  mjtNum p[3];
  mju_mulMatTVec3(p, gmat, dx);  // world -> geom-local
  mjtNum nl[3] = {0, 0, 0}, dist;
  if (type == mjGEOM_CAPSULE) {
    mjtNum hz = size[1], zc = p[2] > hz ? hz : (p[2] < -hz ? -hz : p[2]);
    mjtNum q[3] = {p[0], p[1], p[2] - zc};  // vector from nearest axis point
    mjtNum L = sqrt(mju_dot3(q, q));
    if (L < 1e-12) {
      nl[0] = 1;
    } else {
      for (int k = 0; k < 3; k++) nl[k] = q[k] / L;
    }
    dist = L - size[0];
  } else if (type == mjGEOM_BOX) {
    mjtNum q[3];
    int outside = 0;
    for (int k = 0; k < 3; k++) {
      mjtNum c = p[k] > size[k] ? size[k] : (p[k] < -size[k] ? -size[k] : p[k]);
      q[k] = p[k] - c;
      if (q[k] != 0) outside = 1;
    }
    if (outside) {
      mjtNum L = sqrt(mju_dot3(q, q));
      dist = L;
      for (int k = 0; k < 3; k++) nl[k] = q[k] / L;
    } else {  // inside: least-penetration face
      int ax = 0;
      mjtNum best = 1e30;
      for (int k = 0; k < 3; k++) {
        mjtNum pen = size[k] - (p[k] < 0 ? -p[k] : p[k]);
        if (pen < best) {
          best = pen;
          ax = k;
        }
      }
      dist = -best;
      nl[ax] = p[ax] > 0 ? 1 : -1;
    }
  } else {
    n[0] = 0;
    n[1] = 0;
    n[2] = 1;
    return 1e30;  // unsupported geom -> no barrier
  }
  mju_mulMatVec3(n, gmat, nl);  // geom-local normal -> world
  return dist;
}

// world-space VERTICES of a static geom (sharp features that can poke through a flex triangle):
// box -> 8 corners; mesh -> all its vertices; smooth/infinite geoms none. Returns the count.
int mjc_GeomVerts(const mjModel* m, int gi, const mjtNum* gpos, const mjtNum* gmat, mjtNum* out) {
  int type = m->geom_type[gi];
  if (type == mjGEOM_BOX) {
    const mjtNum* size = m->geom_size + 3 * gi;
    int n = 0;
    for (int sx = -1; sx <= 1; sx += 2)
      for (int sy = -1; sy <= 1; sy += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
          mjtNum loc[3] = {sx * size[0], sy * size[1], sz * size[2]}, wc[3];
          mju_mulMatVec3(wc, gmat, loc);
          for (int k = 0; k < 3; k++) out[3 * n + k] = gpos[k] + wc[k];
          n++;
        }
    return n;
  }
  if (type == mjGEOM_MESH) {
    int mid = m->geom_dataid[gi], nv = m->mesh_vertnum[mid];
    const float* vb = m->mesh_vert + 3 * m->mesh_vertadr[mid];
    for (int i = 0; i < nv; i++) {
      mjtNum lv[3] = {vb[3 * i], vb[3 * i + 1], vb[3 * i + 2]}, wv[3];
      mju_mulMatVec3(wv, gmat, lv);
      for (int k = 0; k < 3; k++) out[3 * i + k] = gpos[k] + wv[k];
    }
    return nv;
  }
  return 0;
}

// world-space EDGES of a static geom (a geom edge can slice through a flex triangle between flex
// vertices): box -> 12 edges; mesh -> its convex-polygon edges (deduped: each shared edge emitted
// once, by the polygon traversing it low->high index). Each edge = two endpoints. Returns count.
int mjc_GeomEdges(const mjModel* m, int gi, const mjtNum* gpos, const mjtNum* gmat, mjtNum* out) {
  int type = m->geom_type[gi];
  if (type == mjGEOM_BOX) {
    const mjtNum* size = m->geom_size + 3 * gi;
    int n = 0;
    for (int axis = 0; axis < 3; axis++) {
      int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
      for (int s1 = -1; s1 <= 1; s1 += 2)
        for (int s2 = -1; s2 <= 1; s2 += 2) {
          mjtNum lo[3], hi[3], w1[3], w2[3];
          lo[axis] = -size[axis];
          hi[axis] = size[axis];
          lo[a1] = hi[a1] = s1 * size[a1];
          lo[a2] = hi[a2] = s2 * size[a2];
          mju_mulMatVec3(w1, gmat, lo);
          mju_mulMatVec3(w2, gmat, hi);
          for (int k = 0; k < 3; k++) {
            out[6 * n + k] = gpos[k] + w1[k];
            out[6 * n + 3 + k] = gpos[k] + w2[k];
          }
          n++;
        }
    }
    return n;
  }
  if (type == mjGEOM_MESH) {
    int mid = m->geom_dataid[gi];
    const float* vb = m->mesh_vert + 3 * m->mesh_vertadr[mid];
    int pa = m->mesh_polyadr[mid], pn = m->mesh_polynum[mid], n = 0;
    for (int p = 0; p < pn; p++) {
      int adr = m->mesh_polyvertadr[pa + p], nvp = m->mesh_polyvertnum[pa + p];
      for (int j = 0; j < nvp; j++) {
        int a = m->mesh_polyvert[adr + j], b = m->mesh_polyvert[adr + (j + 1) % nvp];
        if (a >= b) continue;  // dedup: emit each hull edge once (the low->high traversal)
        mjtNum la[3] = {vb[3 * a], vb[3 * a + 1], vb[3 * a + 2]},
               lb[3] = {vb[3 * b], vb[3 * b + 1], vb[3 * b + 2]}, wa[3], wb[3];
        mju_mulMatVec3(wa, gmat, la);
        mju_mulMatVec3(wb, gmat, lb);
        for (int k = 0; k < 3; k++) {
          out[6 * n + k] = gpos[k] + wa[k];
          out[6 * n + 3 + k] = gpos[k] + wb[k];
        }
        n++;
      }
    }
    return n;
  }
  return 0;
}

mjtNum mjc_off(mjtNum ghc) { return ghc < IPC_DELTACAP ? ghc : IPC_DELTACAP; }

// gap g of a contact at configuration x, plus the barrier gradient direction n, the involved flex
// vertices idv[*nidx] and their weights cw (dg/d(vertex_p) = cw[p]*n). gv/ge are the precomputed
// world-space static-geom corners/edges. Single source of the per-type contact geometry.
// IPC-specific: the engine's collision generators only emit an mjContact (dist/normal); this
// returns gap + normal
// + barycentric weights cw (dg/dx) for the barrier gradient/Hessian, which they do not expose.
mjtNum mjc_conGap(const ipcCon* con, const mjModel* m, const mjData* d, const mjtNum* x,
                  const mjtNum* gv, const mjtNum* ge, mjtNum r, const mjtNum* rad, mjtNum* n,
                  int* idv, mjtNum* cw, int* nidx, mjtNum cutoff) {
  switch (con->type) {
    case 0: {  // vertex-triangle: flex self-contact or cross-flex (vertex v vs triangle A,B,C)
      int v = con->idx[0], A = con->idx[1], B = con->idx[2], C = con->idx[3];
      mjtNum cp[3], w[3], dd = mjc_PtTri(&x[3 * v], &x[3 * A], &x[3 * B], &x[3 * C], cp, w);
      for (int k = 0; k < 3; k++) n[k] = (x[3 * v + k] - cp[k]) / dd;
      idv[0] = v;
      idv[1] = A;
      idv[2] = B;
      idv[3] = C;
      cw[0] = 1;
      cw[1] = -w[0];
      cw[2] = -w[1];
      cw[3] = -w[2];
      *nidx = 4;
      return dd -
             (rad[v] +
              rad[A]);  // point radius + triangle (flex) radius; per-flex for cross-flex contact
    }
    case 1: {  // edge-edge contact (edge a1b1 against edge a2b2): flex self OR cross-flex
      int a1 = con->idx[0], b1 = con->idx[1], a2 = con->idx[2], b2 = con->idx[3];
      mjtNum cp1[3], cp2[3], st[2],
          dd = mjc_SegSeg(&x[3 * a1], &x[3 * b1], &x[3 * a2], &x[3 * b2], cp1, cp2, st);
      for (int k = 0; k < 3; k++) n[k] = (cp1[k] - cp2[k]) / dd;
      idv[0] = a1;
      idv[1] = b1;
      idv[2] = a2;
      idv[3] = b2;
      cw[0] = 1 - st[0];
      cw[1] = st[0];
      cw[2] = -(1 - st[1]);
      cw[3] = -st[1];
      *nidx = 4;
      return dd - (rad[a1] + rad[a2]);  // both edges' (flex) radii
    }
    case 2: {  // flex vertex v vs static geom gi surface
      int v = con->idx[0], gi = con->gi;
      mjtNum dd = mjc_GeomDist(m, gi, d->geom_xpos + 3 * gi, d->geom_xmat + 9 * gi, &x[3 * v], n,
                               cutoff + rad[v]);
      idv[0] = v;
      cw[0] = 1;
      *nidx = 1;
      return dd - rad[v];
    }
    case 3: {  // static geom corner gv[idx0] vs flex triangle A,B,C
      const mjtNum* corner = &gv[3 * con->idx[0]];
      int A = con->idx[1], B = con->idx[2], C = con->idx[3];
      mjtNum cp[3], w[3], dd = mjc_PtTri(corner, &x[3 * A], &x[3 * B], &x[3 * C], cp, w);
      for (int k = 0; k < 3; k++) n[k] = (corner[k] - cp[k]) / dd;
      idv[0] = A;
      idv[1] = B;
      idv[2] = C;
      cw[0] = -w[0];
      cw[1] = -w[1];
      cw[2] = -w[2];
      *nidx = 3;
      return dd - rad[A];  // flex triangle radius
    }
    default: {  // case 4: static geom edge ge[idx0] vs flex edge a,b
      const mjtNum* eg = &ge[6 * con->idx[0]];
      int a = con->idx[1], b = con->idx[2];
      mjtNum cp1[3], cp2[3], st[2], dd = mjc_SegSeg(eg, eg + 3, &x[3 * a], &x[3 * b], cp1, cp2, st);
      for (int k = 0; k < 3; k++) n[k] = (cp1[k] - cp2[k]) / dd;
      idv[0] = a;
      idv[1] = b;
      cw[0] = -(1 - st[1]);
      cw[1] = -st[1];
      *nidx = 2;
      return dd - rad[a];  // flex edge radius
    }
  }
}

// the flex vertices a contact involves (geom features are fixed and excluded): fills v[*nv]
void mjc_conVerts(const ipcCon* con, int* v, int* nv) {
  switch (con->type) {
    case 0:
    case 1:
      v[0] = con->idx[0];
      v[1] = con->idx[1];
      v[2] = con->idx[2];
      v[3] = con->idx[3];
      *nv = 4;
      break;
    case 2:
      v[0] = con->idx[0];
      *nv = 1;
      break;
    case 3:
      v[0] = con->idx[1];
      v[1] = con->idx[2];
      v[2] = con->idx[3];
      *nv = 3;
      break;  // flex triangle A,B,C
    default:
      v[0] = con->idx[1];
      v[1] = con->idx[2];
      *nv = 2;
      break;  // flex edge a,b
  }
}

// per-contact barrier activation distance d_hat. Never exceeds the global ghat, but shrinks to the
// thinnest participating radius: a thin flex (e.g. a drawstring sitting in a thick bag's sleeve)
// then gets a proportionally thin barrier zone instead of resting deep inside the thick neighbour's
// zone (which keeps it permanently active + ratchets kappa -> ill-conditioned). Geom features carry
// no radius (excluded by mjc_conVerts), so geom/flex and sphere/flex contacts keep the global ghat.
mjtNum mjc_conGhat(const ipcCon* con, const mjtNum* rad, mjtNum ghat) {
  int vv[4], nvv;
  mjc_conVerts(con, vv, &nvv);
  mjtNum g = ghat;
  for (int q = 0; q < nvv; q++)
    if (rad[vv[q]] < g) g = rad[vv[q]];
  return g;
}

// surface gap of a contact with its flex vertices advanced by t*dxw (geom features fixed);
// gap-only, for the CCD conservative advancement (recomputes the closest feature at the advanced
// configuration). IPC-specific: the engine has no advanced-configuration gap evaluator for
// conservative advancement (mjc_ccd is single-config).
static mjtNum conGapAdv(const ipcCon* con, const mjModel* m, const mjData* d, const mjtNum* x,
                        const mjtNum* dxw, mjtNum t, const mjtNum* gv, const mjtNum* ge, mjtNum r,
                        const mjtNum* rad, const int* fidx) {
  int v[4], nv;
  mjtNum P[4][3];
  mjc_conVerts(con, v, &nv);
  for (int q = 0; q < nv; q++) {
    int fq = fidx[v[q]];
    for (int k = 0; k < 3; k++) P[q][k] = x[3 * v[q] + k] + (fq >= 0 ? t * dxw[3 * fq + k] : 0.0);
  }
  mjtNum cp[3], w[3], c1[3], c2[3], st[2];
  switch (con->type) {
    case 0:
      return mjc_PtTri(P[0], P[1], P[2], P[3], cp, w) - (rad[con->idx[0]] + rad[con->idx[1]]);
    case 1:
      return mjc_SegSeg(P[0], P[1], P[2], P[3], c1, c2, st) - (rad[con->idx[0]] + rad[con->idx[2]]);
    case 2: {
      mjtNum nn[3];
      return mjc_GeomDist(m, con->gi, d->geom_xpos + 3 * con->gi, d->geom_xmat + 9 * con->gi, P[0],
                          nn, 1e30) -
             rad[con->idx[0]];
    }
    case 3:
      return mjc_PtTri(&gv[3 * con->idx[0]], P[0], P[1], P[2], cp, w) - rad[con->idx[1]];
    default: {
      const mjtNum* eg = &ge[6 * con->idx[0]];
      return mjc_SegSeg(eg, eg + 3, P[0], P[1], c1, c2, st) - rad[con->idx[1]];
    }
  }
}

mjtNum mjc_advance(const mjModel* m, const mjData* d, const mjtNum* x, const mjtNum* dxw,
                   const mjtNum* gv, const mjtNum* ge, mjtNum r, const mjtNum* rad, int nfv,
                   const int* fidx, const ipcCon* cand, int ncand, const mjtNum* cgap,
                   const int* pt2flex, int* approut) {
  mjtNum alpha = 1.0;
  g_ccd_cap = -1;
  g_ccd_l = 0;
  g_ccd_g0 = 0;  // [TRACE] reset per call
  if (approut)
    for (int c = 0; c < ncand; c++)
      approut[c] = 0;  // Alg.3: per-pair "the proxy approaches this contact"
  for (int c = 0; c < ncand; c++) {
    const ipcCon* con = &cand[c];
    // mean-removal (don't throttle COHERENT motion) is valid ONLY for a true SAME-FLEX
    // self-contact. For INTER-flex (e.g. drawstring vs bag) the two sides move independently, so
    // removing the mean underestimates the closing speed and lets one tunnel through the other.
    // Gate it on same-flex (both sides in the same flex via pt2flex).
    int other = (con->type == 0) ? con->idx[1] : con->idx[2];
    int v[4], nv,
        self = (con->type <= 1) && (con->idx[0] < nfv) && (other < nfv) &&
               (pt2flex[con->idx[0]] == pt2flex[other]);
    mjc_conVerts(con, v, &nv);
    mjtNum dp[4][3], mean[3] = {0, 0, 0};
    for (int q = 0; q < nv; q++) {
      int fq = fidx[v[q]];
      for (int k = 0; k < 3; k++) dp[q][k] = (fq >= 0 ? dxw[3 * fq + k] : 0.0);
    }
    if (self) {
      for (int q = 0; q < nv; q++)
        for (int k = 0; k < 3; k++) mean[k] += dp[q][k];
      for (int k = 0; k < 3; k++) mean[k] /= nv;
      for (int q = 0; q < nv; q++)
        for (int k = 0; k < 3; k++) dp[q][k] -= mean[k];
    }
    mjtNum l;  // bound on the gap-shrink rate per unit alpha
    if (con->type == 0) {
      mjtNum a0 = sqrt(mju_dot3(dp[0], dp[0])), b0 = 0;
      for (int q = 1; q < 4; q++) {
        mjtNum s = sqrt(mju_dot3(dp[q], dp[q]));
        if (s > b0) b0 = s;
      }
      l = a0 + b0;
    } else if (con->type == 1) {
      mjtNum a0 = 0, b0 = 0;
      for (int q = 0; q < 2; q++) {
        mjtNum s = sqrt(mju_dot3(dp[q], dp[q]));
        if (s > a0) a0 = s;
      }
      for (int q = 2; q < 4; q++) {
        mjtNum s = sqrt(mju_dot3(dp[q], dp[q]));
        if (s > b0) b0 = s;
      }
      l = a0 + b0;
    } else if (con->type == 5) {
      l = sqrt(mju_dot3(dp[0], dp[0])) + sqrt(mju_dot3(dp[1], dp[1]));
    }  // both move
    else {
      l = 0;
      for (int q = 0; q < nv; q++) {
        mjtNum s = sqrt(mju_dot3(dp[q], dp[q]));
        if (s > l) l = s;
      }
    }
    if (l < 1e-12) continue;
    mjtNum g0 =
        cgap[c];  // true gap at x (>= standoff delta at rest, so the CCD always has room: no lock)
    if (g0 <= 0) continue;  // already at/under the surface: the AL + energy own it
    if (l <= 0.8 * g0)
      continue;  // full alpha=1 step shrinks gap by <= l, stays above the 20% floor
    if (approut)
      approut[c] =
          1;  // reaches the bisection -> the proxy closes this pair's gap this step (Alg.3 add)
    mjtNum gtarget = 0.2 * g0, t = 0;
    for (int it = 0; it < 32; it++) {
      mjtNum g = conGapAdv(con, m, d, x, dxw, t, gv, ge, r, rad, fidx);
      mjtNum room = g - gtarget;
      if (room <= 1e-9 * g0) break;
      t += room / l;
      if (t >= alpha) {
        t = alpha;
        break;
      }
    }
    if (t < alpha) {
      alpha = t;
      g_ccd_cap = c;
      g_ccd_l = l;
      g_ccd_g0 = g0;
    }  // [TRACE] record the capping pair
  }
  return alpha;
}

// append a candidate contact if its gap at x is below the (margin-inflated) detection threshold
static void addCand(ipcCon con, const mjModel* m, const mjData* d, const mjtNum* x,
                    const mjtNum* gv, const mjtNum* ge, mjtNum r, const mjtNum* rad, mjtNum thresh,
                    const mjtNum* dfrom, const mjtNum* dto, mjtNum ghat, ipcCon* cand, int* nc,
                    int candmax) {
  if (*nc >= candmax) return;
  mjtNum n[3], cw[4];
  int idv[4], nidx;
  mjtNum g = mjc_conGap(&con, m, d, x, gv, ge, r, rad, n, idv, cw, &nidx, thresh);
  if (g <= 0 || g >= thresh) return;
  // closing-bound prune: over the step the gap changes by at most |sum_p cw[p]*(dto-dfrom)[idv[p]]|
  // (Cauchy-Schwarz, |n|=1), so a pair beyond its per-contact ghc + that bound cannot become active
  // this step -> drop it. Replaces the crude GLOBAL 4*maxdisp band (which inflated by the fastest
  // vertex anywhere, flooding correlated bulk motion like a settling bag+string). No-tunnel safe:
  // the per-outer re-query at xfree (dfrom=xfree, dto=x) recaptures any pair whose closest feature
  // flips under the inner step.
  mjtNum rel[3] = {0, 0, 0};
  for (int p = 0; p < nidx; p++) {
    int vp = idv[p];
    if (vp < 0) continue;
    for (int c = 0; c < 3; c++) rel[c] += cw[p] * (dto[3 * vp + c] - dfrom[3 * vp + c]);
  }
  if (g < mjc_conGhat(&con, rad, ghat) + sqrt(mju_dot3(rel, rel))) cand[(*nc)++] = con;
}

// descend flex f's element BVH (built and AABB-refreshed by mj_flex at the step's xold), collecting
// the leaf element ids whose (radius-inflated) node AABB overlaps the query box [c +/- h]. Replaces
// the hand-rolled uniform spatial hash: same "nearby elements" query, but the engine's hierarchy.
// The node AABBs already include flex_radius, so a query half of thresh is a conservative superset
// (no pair within thresh is missed; the narrowphase then filters). stack must hold flex_bvhnum
// ints.
static int bvhBox(const mjModel* m, const mjData* d, int f, const mjtNum* c, const mjtNum* h,
                  int* stack, int* out, int maxout) {
  int bvhadr = m->flex_bvhadr[f];
  if (bvhadr < 0) return 0;
  const int* child = m->bvh_child + 2 * bvhadr;
  const int* nodeid = m->bvh_nodeid + bvhadr;
  const mjtNum* aabb = d->bvh_aabb_dyn + 6 * (bvhadr - m->nbvhstatic);
  int ns = 0, nout = 0;
  stack[ns++] = 0;
  while (ns) {
    int node = stack[--ns];
    const mjtNum* na = aabb + 6 * node;  // [center(3), halfsize(3)]
    if (mju_abs(na[0] - c[0]) > na[3] + h[0] || mju_abs(na[1] - c[1]) > na[4] + h[1] ||
        mju_abs(na[2] - c[2]) > na[5] + h[2])
      continue;  // box-box separation -> prune
    int c0 = child[2 * node], c1 = child[2 * node + 1];
    if (c0 < 0 && c1 < 0) {
      if (nout < maxout) out[nout++] = nodeid[node];
    }  // leaf -> element id
    else {
      if (c0 >= 0) stack[ns++] = c0;
      if (c1 >= 0) stack[ns++] = c1;
    }
  }
  return nout;
}

// build the candidate-contact list once per step, gated by a velocity-aware threshold so any pair
// that could close within the step is captured (the Newton loop then only re-tests candidates).
// Flex-flex and geom-feature-vs-flex pairs are found by querying the flex element BVH (bvhBox).
int mjc_candidates(const mjModel* m, const mjData* d, const mjtNum* x, const mjtNum* gv,
                   const mjtNum* ge, int ngv, int nge, mjtNum r, const mjtNum* rad, mjtNum thresh,
                   mjtNum threshGeom, mjtNum maxdisp, const mjtNum* dfrom, const mjtNum* dto,
                   mjtNum ghat, int nfv, int npt, const int* fidx, const int* flist,
                   const int* fxadr, int nfd, const int* pt2flex, ipcCon* cand, int candmax) {
  (void)
      npt;  // increment A: candidates are flex-only (npt==nfv); rigid bodies carry no hard contact
  int nc = 0;
  for (int gi = 0; gi < m->ngeom; gi++) {  // free point (flex vert) vs STATIC geom
    if (m->geom_contype[gi] == 0 && m->geom_conaffinity[gi] == 0) continue;  // skip non-colliding
    if (m->body_weldid[m->geom_bodyid[gi]] != 0)
      continue;  // STATIC geoms ONLY: a MOVABLE rigid geom (ball/limb) is a soft type-3 contact,
                 // never a frozen penetration-free obstacle
    // geom-level cull: bound the points by the geom's true world AABB (the rotated local geom_aabb)
    // + per-point margin, before the per-face distance. Much tighter than the bounding sphere for
    // the elongated convex-decomposition slabs. Planes are infinite -> no cull (their distance is
    // O(1) anyway).
    int isplane = (m->geom_type[gi] == mjGEOM_PLANE);
    mjtNum wc[3], wh[3];
    if (!isplane) {
      const mjtNum* la = m->geom_aabb + 6 * gi;
      const mjtNum* gp = d->geom_xpos + 3 * gi;
      const mjtNum* gR = d->geom_xmat + 9 * gi;
      mju_mulMatVec3(wc, gR, la);
      for (int k = 0; k < 3; k++) wc[k] += gp[k];  // world AABB center
      for (int k = 0; k < 3; k++)
        wh[k] = mju_abs(gR[3 * k]) * la[3] + mju_abs(gR[3 * k + 1]) * la[4] +
                mju_abs(gR[3 * k + 2]) * la[5];
    }
    for (int v = 0; v < nfv; v++) {  // flex verts only (rigid bodies carry no hard barrier)
      if (fidx[v] < 0) continue;
      mjtNum marg = thresh + rad[v];
      if (!isplane) {  // world-AABB cull
        if (mju_abs(x[3 * v] - wc[0]) > wh[0] + marg ||
            mju_abs(x[3 * v + 1] - wc[1]) > wh[1] + marg ||
            mju_abs(x[3 * v + 2] - wc[2]) > wh[2] + marg)
          continue;
      }
      ipcCon con = {2, {v, 0, 0, 0}, gi};
      addCand(con, m, d, x, gv, ge, r, rad, thresh, dfrom, dto, ghat, cand, &nc, candmax);
    }
  }

  // (Rigid sphere-sphere and sphere-vs-flex HARD contacts removed in increment A: rigid bodies live
  // in their own solver block and carry no AL/CCD contact yet. Soft flex-rigid contact is a
  // separate next increment.)

  // ---- BVH-based candidates over ALL dim-2 flexes: geom-feature and flex-vs-flex VT/EE.
  // Each query is against one flex's element BVH (bvhBox); triangle/edge vertices are mapped
  // from the queried flex's local indices to the combined free-point space (fxadr[k] + local).
  // flex-vs-flex contact is SELF when the querying vertex/edge is in the queried flex (gated by
  // that flex's selfcollide) and INTER-FLEX otherwise (always on). Scratch buffers are sized for
  // the largest flex. ----
  int maxbvh = 1, maxel = 1, maxen = 1;
  for (int k = 0; k < nfd; k++) {
    int fk = flist[k];
    if (m->flex_bvhnum[fk] > maxbvh) maxbvh = m->flex_bvhnum[fk];
    if (m->flex_elemnum[fk] > maxel) maxel = m->flex_elemnum[fk];
    if (m->flex_edgenum[fk] > maxen) maxen = m->flex_edgenum[fk];
  }
  int* stk = (int*)mju_malloc(maxbvh * sizeof(int));
  int* outel = (int*)mju_malloc(maxel * sizeof(int));
  int* stampG = (int*)mju_malloc(maxen * sizeof(int));
  for (int e = 0; e < maxen; e++) stampG[e] = -1;
  int qid = 0;

  for (int k = 0; k < nfd; k++) {  // query flex fk's element BVH
    int fk = flist[k];
    int ne_k = m->flex_elemnum[fk], ea_k = m->flex_edgeadr[fk], off_k = fxadr[k];
    if (m->flex_bvhadr[fk] < 0 || ne_k == 0) continue;
    const int* el_k = m->flex_elem + m->flex_elemdataadr[fk];
    const int* eme_k = m->flex_elemedge + m->flex_elemedgeadr[fk];
    mjtNum rk = m->flex_radius[fk];
    int doself_k = (m->flex_selfcollide[fk] != mjFLEXSELF_NONE);

    // (rigid sphere-vs-flex-triangle hard contact removed in increment A -- see note above.)
    // geom-corner vs flex triangle (type 3); geom is static (one-sided) -> tighter threshGeom (the
    // convex-decomposition bin's ~1600 edges otherwise overflow candmax and drop the bag-bin
    // contacts).
    mjtNum qhvG[3] = {threshGeom + rk, threshGeom + rk, threshGeom + rk};
    for (int c = 0; c < ngv; c++) {
      int n = bvhBox(m, d, fk, &gv[3 * c], qhvG, stk, outel, ne_k);
      for (int i = 0; i < n; i++) {
        int e = outel[i];
        ipcCon con = {
            3, {c, off_k + el_k[3 * e], off_k + el_k[3 * e + 1], off_k + el_k[3 * e + 2]}, -1};
        addCand(con, m, d, x, gv, ge, r, rad, threshGeom, dfrom, dto, ghat, cand, &nc, candmax);
      }
    }
    // geom-edge vs flex edge (type 4); dedup the shared triangle edges per query via stampG.
    for (int c = 0; c < nge; c++) {
      const mjtNum* p0 = &ge[6 * c];
      const mjtNum* p1 = &ge[6 * c + 3];
      mjtNum qc[3], qh[3];
      for (int kk = 0; kk < 3; kk++) {
        qc[kk] = 0.5 * (p0[kk] + p1[kk]);
        qh[kk] = 0.5 * mju_abs(p1[kk] - p0[kk]) + threshGeom + rk;
      }
      int n = bvhBox(m, d, fk, qc, qh, stk, outel, ne_k);
      qid++;
      for (int i = 0; i < n; i++) {
        int e = outel[i];
        for (int j = 0; j < 3; j++) {
          int e2 = eme_k[3 * e + j];
          if (stampG[e2] == qid) continue;
          stampG[e2] = qid;
          ipcCon con = {4,
                        {c, off_k + m->flex_edge[2 * (ea_k + e2)],
                         off_k + m->flex_edge[2 * (ea_k + e2) + 1], 0},
                        -1};
          addCand(con, m, d, x, gv, ge, r, rad, threshGeom, dfrom, dto, ghat, cand, &nc, candmax);
        }
      }
    }
    // flex vertex vs flex triangle (type 0): self (same flex, gated by selfcollide) + inter-flex
    // (always). Asymmetric (vert vs tri), so all verts query every flex's BVH -- both directions
    // are distinct contacts.
    for (int v = 0; v < nfv; v++) {
      int kv = pt2flex[v];
      if (kv == k && !doself_k) continue;  // self-contact disabled for this flex
      mjtNum thv =
          3.0 * min2(r, min2(rad[v], rk)) + 4.0 * maxdisp;  // per-pair band: thinner flex sets it
      mjtNum qh[3] = {thv + rad[v], thv + rad[v], thv + rad[v]};
      int n = bvhBox(m, d, fk, &x[3 * v], qh, stk, outel, ne_k);
      for (int i = 0; i < n; i++) {
        int e = outel[i];
        int A = off_k + el_k[3 * e], B = off_k + el_k[3 * e + 1], C = off_k + el_k[3 * e + 2];
        if (kv == k && (v == A || v == B || v == C)) continue;  // skip the self-adjacent triangle
        ipcCon con = {0, {v, A, B, C}, -1};
        addCand(con, m, d, x, gv, ge, r, rad, thv, dfrom, dto, ghat, cand, &nc, candmax);
      }
    }
    // flex edge vs flex edge (type 1): symmetric, so canonical -- querying flex kj <= k, and e2 >
    // e1 within a flex. Self (kj==k) gated by selfcollide; inter-flex (kj<k) always.
    for (int kj = 0; kj <= k; kj++) {
      int self = (kj == k);
      if (self && !doself_k) continue;
      int fj = flist[kj], ea_j = m->flex_edgeadr[fj], en_j = m->flex_edgenum[fj], off_j = fxadr[kj];
      for (int e1 = 0; e1 < en_j; e1++) {
        int a1 = off_j + m->flex_edge[2 * (ea_j + e1)],
            b1 = off_j + m->flex_edge[2 * (ea_j + e1) + 1];
        mjtNum the = 3.0 * min2(r, min2(rad[a1], rk)) + 4.0 * maxdisp;  // per-pair band
        mjtNum qc[3], qh[3];
        for (int kk = 0; kk < 3; kk++) {
          qc[kk] = 0.5 * (x[3 * a1 + kk] + x[3 * b1 + kk]);
          qh[kk] = 0.5 * mju_abs(x[3 * a1 + kk] - x[3 * b1 + kk]) + the + rad[a1];
        }
        int n = bvhBox(m, d, fk, qc, qh, stk, outel, ne_k);
        qid++;
        for (int i = 0; i < n; i++) {
          int e = outel[i];
          for (int j = 0; j < 3; j++) {
            int e2 = eme_k[3 * e + j];
            if (self && e2 <= e1) continue;  // canonical within a flex
            if (stampG[e2] == qid) continue;
            stampG[e2] = qid;
            int a2 = off_k + m->flex_edge[2 * (ea_k + e2)],
                b2 = off_k + m->flex_edge[2 * (ea_k + e2) + 1];
            if (a1 == a2 || a1 == b2 || b1 == a2 || b1 == b2)
              continue;  // shared vertex -> adjacent, skip
            ipcCon con = {1, {a1, b1, a2, b2}, -1};
            addCand(con, m, d, x, gv, ge, r, rad, the, dfrom, dto, ghat, cand, &nc, candmax);
          }
        }
      }
    }
  }
  mju_free(stk);
  mju_free(outel);
  mju_free(stampG);
  return nc;
}
