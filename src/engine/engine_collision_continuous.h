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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_COLLISION_CONTINUOUS_H_
#define MUJOCO_SRC_ENGINE_ENGINE_COLLISION_CONTINUOUS_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// Continuous-collision geometry for the IPC integrator (engine_ipc): differentiable
// vertex-triangle / edge-edge / vertex-geom distance kernels with closest-point barycentrics,
// flex BVH traversal and trajectory-margin candidate generation, contact linearization, and
// the conservative CCD advance. Kept apart from the discrete collision pipeline
// (engine_collision_*): those generate contact points; these price gaps along trajectories.

// Contact layers: the gap is the MIDSURFACE distance (mjc_conGap); a pair RESTS at
// gap = r1 + r2 (mjc_conGhat) -- the flex radius is not a physical thickness but the rest
// offset, the whole skin below it is the soft recoverable band. The only hard invariant is
// that midsurfaces never cross: the CCD advance (mjc_advance) floors every committed gap at
// IPC_DMIN > 0, which also keeps the distance kernels' normals well-conditioned.
#define IPC_DMIN 1e-5  // absolute commit floor on the midsurface gap (10 um)

// one active contact. type: 0 vertex-triangle self, 1 edge-edge self, 2 flex-vertex vs geom
// surface, 3 geom-corner vs flex-triangle, 4 geom-edge vs flex-edge. idx/gi meaning per type
// (see mjc_conGap). The geom side is static, so its features (gv/ge) are precomputed once per step.
// lam = AL multiplier (rides in copies). The contact is LINEARIZED at the intersection-free state
// xfree each outer iter (paper Eq.10): ld0 = gap(xfree), ln = normal, lcw[liv] = dg/dx weights at
// the involved free pts. Then c(x) = ld0 + sum_p lcw[p]*ln.(x[liv[p]]-xfree[liv[p]]) - delta is
// LINEAR in x -> exact constant contact Hessian (mu*grad d grad d^T, no grad^2 d) -> the inner
// Newton converges in ~1 step.
typedef struct {
  int type;
  int idx[4];
  int gi;
  mjtNum lam;
  mjtNum ld0, ln[3], lcw[4];
  int liv[4], lniv;
  int cnt;
  mjtNum s;
} ipcCon;  // cnt = active-set state machine (0.3^c stiffness decay +
// aging); s = materialized AL slack for the d0 bake (slack update -> assemble -> lambda update
// un-bake)

mjtNum mjc_conGap(const ipcCon* con, const mjModel* m, const mjData* d, const mjtNum* x,
                  const mjtNum* gv, const mjtNum* ge, mjtNum r, const mjtNum* rad, mjtNum* n,
                  int* idv, mjtNum* cw, int* nidx, mjtNum cutoff);
void mjc_conVerts(const ipcCon* con, int* v, int* nv);
mjtNum mjc_conGhat(const ipcCon* con, const mjtNum* rad);
mjtNum mjc_advance(const mjModel* m, const mjData* d, const mjtNum* x, const mjtNum* dxw,
                   const mjtNum* gv, const mjtNum* ge, mjtNum r, const mjtNum* rad, int nfv,
                   const int* fidx, const ipcCon* cand, int ncand, const mjtNum* cgap,
                   const int* pt2flex, int* approut);
int mjc_candidates(const mjModel* m, const mjData* d, const mjtNum* x, const mjtNum* gv,
                   const mjtNum* ge, int ngv, int nge, mjtNum r, const mjtNum* rad, mjtNum thresh,
                   mjtNum threshGeom, mjtNum maxdisp, const mjtNum* dfrom, const mjtNum* dto,
                   mjtNum ghat, int nfv, int npt, const int* fidx, const int* flist,
                   const int* fxadr, int nfd, const int* pt2flex, ipcCon* cand, int candmax);

// distance kernels exported (MJAPI) for unit tests; internal, not a supported API.
// mjc_PtTri: point-triangle distance (closest point cp and barycentric weights w).
// mjc_SegSeg: segment-segment distance (closest points and line parameters st).
// mjc_GeomDist: signed distance (+ outward unit normal n) from geom gi's surface, at pose
// gpos/gmat, to world point x; early-out beyond distmax.
// mjc_GeomVerts / mjc_GeomEdges: world-space sharp vertices / edges of geom gi at pose
// gpos/gmat (out sized by the caller); return the count.
MJAPI mjtNum mjc_PtTri(const mjtNum* p, const mjtNum* a, const mjtNum* b, const mjtNum* c,
                       mjtNum* cp, mjtNum* w);
MJAPI mjtNum mjc_SegSeg(const mjtNum* p1, const mjtNum* p2, const mjtNum* q1, const mjtNum* q2,
                        mjtNum* cp1, mjtNum* cp2, mjtNum* st);
MJAPI mjtNum mjc_GeomDist(const mjModel* m, int gi, const mjtNum* gpos, const mjtNum* gmat,
                          const mjtNum* x, mjtNum* n, mjtNum distmax);
MJAPI int mjc_GeomVerts(const mjModel* m, int gi, const mjtNum* gpos, const mjtNum* gmat,
                        mjtNum* out);
MJAPI int mjc_GeomEdges(const mjModel* m, int gi, const mjtNum* gpos, const mjtNum* gmat,
                        mjtNum* out);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_CONTINUOUS_H_
