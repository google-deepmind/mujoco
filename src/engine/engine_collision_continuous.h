// Copyright 2026 DeepMind Technologies Limited
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

// Continuous collision for deformables. The discrete pipeline (engine_collision_*) generates
// contact points at a configuration; this module prices gaps along trajectories: differentiable
// vertex-triangle / edge-edge / vertex-geom distance kernels with closest-point barycentrics,
// swept-volume candidate generation over the flex BVH, per-pair gap evaluation with the gradient's
// vertex weights, and a conservative advancement (CCD) that bounds each pair's time of impact.
// A consumer supplies two lengths:
//   cap   -- the standoff ceiling: a pair's rest gap is min(band, cap), so thin participants keep
//            a proportionally thin skin and thick ones do not carry a fat layer;
//   band  -- the detection reach: how early the broad phase starts tracking a pair. Not a physical
//            length; wider costs candidates, narrower hands pairs to the solver later.
// FLEX-FLEX pairs (types 0/1) measure their gap at the MIDSURFACE: mjc_pairGap does not subtract
// the radii for those types, because where the mesh geometry is tighter than the combined radii
// (a string threaded through a hem) a skin-to-skin gap is permanently negative and the pair would
// be discarded as invalid -- no CCD coverage, so the region could tunnel. The broad phase adds the
// radii back into its reach so detection range is unchanged (see addCand).

// One candidate contact pair: the geometric identity only. type 0: flex vertex vs flex triangle,
// 1: flex edge vs flex edge, 2: flex vertex vs geom surface, 3: geom corner vs flex triangle,
// 4: geom edge vs flex edge. idx holds free-point indices (types 0/1: all four; type 2: idx[0];
// types 3/4: the flex-side points), gi the geom for types 2-4. Solver state (multipliers, ages)
// and any cached linearization of the gap belong to the consumer, not to this struct.
typedef struct {
  int type;    // pair type, 0-4 as above
  int idx[4];  // participant free-point indices, meaning per type (see mjc_pairGap)
  int gi;      // geom id, types 2-4 only
} mjcPair;

// the standoff of a pair whose detection band is `band`: min(band, cap)
mjtNum mjc_standoff(mjtNum band, mjtNum cap);

// per-pair detection band: min over the pair's flex radii and `band` (see the header note on
// midsurface gaps for why the radii enter the band and not the gap)
mjtNum mjc_pairBand(const mjcPair* pair, const mjtNum* rad, mjtNum band);

// the involved free-point indices of a pair (up to 4), for iterating its vertices
void mjc_pairVerts(const mjcPair* pair, int* v, int* nv);

// Gap of a pair at configuration x, plus the gradient's direction n and its vertex weights:
// dg/d(vertex idv[p]) = cw[p]*n, p < *nidx. gv/ge are the precomputed world-space geom corners and
// edges (mjc_GeomVerts/mjc_GeomEdges), rad the per-free-point radii. Early-out beyond cutoff.
// Exported for tests; internal, not a supported API.
MJAPI mjtNum mjc_pairGap(const mjcPair* pair, const mjModel* m, const mjData* d, const mjtNum* x,
                         const mjtNum* gv, const mjtNum* ge, const mjtNum* rad, mjtNum* n,
                         int* idv, mjtNum* cw, int* nidx, mjtNum cutoff);

// Swept candidate generation: all pairs whose gap can enter the detection band along the segment
// dfrom -> dto, gathered over the flex BVH (self and cross-flex) and the geom features. thresh /
// threshGeom bound the flex-flex / flex-geom reach, maxdisp the per-vertex motion the collar must
// absorb, ghat the detection band. Returns the number of candidates written to cand (at most
// candmax). Exported for tests; internal, not a supported API.
MJAPI int mjc_candidates(const mjModel* m, const mjData* d, const mjtNum* x, const mjtNum* gv,
                         const mjtNum* ge, int ngv, int nge, const mjtNum* rad, mjtNum thresh,
                         mjtNum threshGeom, mjtNum maxdisp, const mjtNum* dfrom, const mjtNum* dto,
                         mjtNum ghat, int nfv, int npt, const int* fidx, const int* flist,
                         const int* fxadr, int nfd, const int* pt2flex, mjcPair* cand, int candmax);

// Conservative advancement: the largest alpha in [0, 1] such that moving the free points from x by
// alpha*dxw keeps every candidate's gap above a fraction of its value at x (no pair's gap is
// closed by more than 80%), so the advanced configuration stays intersection-free. cgap holds each
// candidate's gap at x (from mjc_pairGap). Optional outputs: approut[c] = 1 if the full step
// closes candidate c into its active zone; toiout[c] = candidate c's own time of impact (1 if it
// does not collide this step). Exported for tests; internal, not a supported API.
MJAPI mjtNum mjc_advance(const mjModel* m, const mjData* d, const mjtNum* x, const mjtNum* dxw,
                         const mjtNum* gv, const mjtNum* ge, const mjtNum* rad, int nfv,
                         const int* fidx, const mjcPair* cand, int ncand, const mjtNum* cgap,
                         const int* pt2flex, int* approut, mjtNum* toiout);

// Distance kernels, exported for tests; internal, not a supported API.
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
