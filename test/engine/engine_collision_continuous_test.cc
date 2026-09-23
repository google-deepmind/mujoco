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

// Tests for engine/engine_collision_continuous.c: the distance kernels, the
// per-pair gap and its gradient (finite-difference checked), the swept
// candidate generation, and the conservative advancement. Everything is
// exercised directly on hand-built pairs or tiny flex models; no contact solver
// is involved.

#include "src/engine/engine_collision_continuous.h"

#include <cmath>
#include <cstdio>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjtype.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// convenience shims over the MJAPI geometry kernels (pose taken from d,
// scratch dropped)
static mjtNum PtTri(const mjtNum* p, const mjtNum* a, const mjtNum* b,
                    const mjtNum* c) {
  mjtNum cp[3], w[3];
  return mjc_PtTri(p, a, b, c, cp, w);
}
static mjtNum SegSeg(const mjtNum* p1, const mjtNum* p2, const mjtNum* q1,
                     const mjtNum* q2) {
  mjtNum cp1[3], cp2[3], st[2];
  return mjc_SegSeg(p1, p2, q1, q2, cp1, cp2, st);
}
static mjtNum GeomDist(const mjModel* m, const mjData* d, int gi,
                       const mjtNum* x, mjtNum* n) {
  return mjc_GeomDist(m, gi, d->geom_xpos + 3 * gi, d->geom_xmat + 9 * gi, x, n,
                      1e30);
}
static int GeomVerts(const mjModel* m, const mjData* d, int gi, mjtNum* out) {
  return mjc_GeomVerts(m, gi, d->geom_xpos + 3 * gi, d->geom_xmat + 9 * gi,
                       out);
}
static int GeomEdges(const mjModel* m, const mjData* d, int gi, mjtNum* out) {
  return mjc_GeomEdges(m, gi, d->geom_xpos + 3 * gi, d->geom_xmat + 9 * gi,
                       out);
}

using ::testing::NotNull;
using ContinuousCollisionTest = MujocoTest;

static mjModel* Load(const char* xml) {
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  EXPECT_THAT(model.get(), NotNull()) << error;
  return model.release();
}

// id of the first geom in the model
static int FirstGeom(const mjModel* m) { return 0; }

// ----------------------------- element distances -----------------------------

// point-triangle distance: interior (perpendicular), edge region, vertex region
TEST_F(ContinuousCollisionTest, PointTriangleDistance) {
  mjtNum a[3] = {0, 0, 0}, b[3] = {1, 0, 0}, c[3] = {0, 1, 0};

  mjtNum p_above[3] = {0.2, 0.2, 0.5};  // over the interior
  EXPECT_NEAR(PtTri(p_above, a, b, c), 0.5, MjTol(1e-12, 1e-5));

  mjtNum p_edge[3] = {-1, 0.5, 0};  // nearest the x=0 edge
  EXPECT_NEAR(PtTri(p_edge, a, b, c), 1.0, MjTol(1e-12, 1e-5));

  mjtNum p_vert[3] = {-3, -4, 0};  // nearest vertex a
  EXPECT_NEAR(PtTri(p_vert, a, b, c), 5.0, MjTol(1e-12, 1e-5));

  mjtNum p_on[3] = {0.25, 0.25, 0};  // on the triangle
  EXPECT_NEAR(PtTri(p_on, a, b, c), 0.0, MjTol(1e-12, 1e-5));
}

// segment-segment distance: perpendicular crossing, collinear gap, parallel
// offset
TEST_F(ContinuousCollisionTest, SegmentSegmentDistance) {
  mjtNum p1[3] = {-1, 0, 0}, p2[3] = {1, 0, 0};

  // perpendicular, 0.3 above
  mjtNum q1[3] = {0, -1, 0.3}, q2[3] = {0, 1, 0.3};
  EXPECT_NEAR(SegSeg(p1, p2, q1, q2), 0.3, MjTol(1e-12, 1e-5));

  mjtNum r1[3] = {2, 0, 0}, r2[3] = {3, 0, 0};  // collinear, gap 1
  EXPECT_NEAR(SegSeg(p1, p2, r1, r2), 1.0, MjTol(1e-12, 1e-5));

  mjtNum s1[3] = {-1, 0, 0.5}, s2[3] = {1, 0, 0.5};  // parallel, 0.5 above
  EXPECT_NEAR(SegSeg(p1, p2, s1, s2), 0.5, MjTol(1e-12, 1e-5));
}

// ------------------------------- geom distance -------------------------------

constexpr char kPrimitivesXml[] = R"(
<mujoco>
  <worldbody>
    <geom name="box" type="box" size="0.1 0.2 0.3" pos="0 0 0"/>
    <geom name="sphere" type="sphere" size="0.1" pos="1 0 0"/>
    <geom name="plane" type="plane" size="0 0 1" pos="0 0 -1"/>
  </worldbody>
</mujoco>
)";

TEST_F(ContinuousCollisionTest, GeomDistance) {
  mjModel* m = Load(kPrimitivesXml);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);
  int box = mj_name2id(m, mjOBJ_GEOM, "box");
  int sphere = mj_name2id(m, mjOBJ_GEOM, "sphere");
  int plane = mj_name2id(m, mjOBJ_GEOM, "plane");
  mjtNum n[3];

  // box (half-extent 0.1 in x): point on +x at 0.5 -> surface distance 0.4,
  // normal +x
  mjtNum px[3] = {0.5, 0, 0};
  EXPECT_NEAR(GeomDist(m, d, box, px, n), 0.4, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(n[0], 1, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(n[1], 0, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(n[2], 0, MjTol(1e-12, 1e-5));

  // interior point -> negative signed distance
  mjtNum pc[3] = {0, 0, 0};
  EXPECT_LT(GeomDist(m, d, box, pc, n), 0);

  // sphere radius 0.1 at (1,0,0): point at (1.3,0,0) -> 0.2, normal +x
  mjtNum ps[3] = {1.3, 0, 0};
  EXPECT_NEAR(GeomDist(m, d, sphere, ps, n), 0.2, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(n[0], 1, MjTol(1e-12, 1e-5));

  // plane at z=-1: point at z=0 -> 1.0, normal +z
  mjtNum pp[3] = {0.3, -0.2, 0};
  EXPECT_NEAR(GeomDist(m, d, plane, pp, n), 1.0, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(n[2], 1, MjTol(1e-12, 1e-5));

  mj_deleteData(d);
  mj_deleteModel(m);
}

// ---------------------------- geom sharp features ----------------------------

// a box exposes its 8 corners (at +/-size) and 12 edges
TEST_F(ContinuousCollisionTest, BoxFeatures) {
  constexpr char xml[] = R"(
  <mujoco><worldbody>
    <geom type="box" size="0.1 0.2 0.3"/>
  </worldbody></mujoco>)";
  mjModel* m = Load(xml);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  mjtNum verts[8 * 3], edges[12 * 6];
  int nv = GeomVerts(m, d, FirstGeom(m), verts);
  int ne = GeomEdges(m, d, FirstGeom(m), edges);
  EXPECT_EQ(nv, 8);
  EXPECT_EQ(ne, 12);
  for (int i = 0; i < nv; i++) {
    EXPECT_NEAR(std::fabs(verts[3 * i + 0]), 0.1, MjTol(1e-12, 1e-5));
    EXPECT_NEAR(std::fabs(verts[3 * i + 1]), 0.2, MjTol(1e-12, 1e-5));
    EXPECT_NEAR(std::fabs(verts[3 * i + 2]), 0.3, MjTol(1e-12, 1e-5));
  }
  // every box edge has unit length along exactly one axis (here 0.2, 0.4, or
  // 0.6)
  for (int i = 0; i < ne; i++) {
    mjtNum dx = edges[6 * i + 3] - edges[6 * i + 0];
    mjtNum dy = edges[6 * i + 4] - edges[6 * i + 1];
    mjtNum dz = edges[6 * i + 5] - edges[6 * i + 2];
    mjtNum len = std::sqrt(dx * dx + dy * dy + dz * dz);
    EXPECT_TRUE(std::fabs(len - 0.2) < MjTol(1e-12, 1e-5) ||
                std::fabs(len - 0.4) < MjTol(1e-12, 1e-5) ||
                std::fabs(len - 0.6) < MjTol(1e-12, 1e-5))
        << "edge " << i << " length " << len;
  }
  mj_deleteData(d);
  mj_deleteModel(m);
}

// a convex mesh exposes its vertices and its (deduplicated) hull edges;
// a tetrahedron has 4 and 6
TEST_F(ContinuousCollisionTest, MeshFeatures) {
  constexpr char xml[] = R"(
  <mujoco>
    <asset><mesh name="tet" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/></asset>
    <worldbody><geom type="mesh" mesh="tet"/></worldbody>
  </mujoco>)";
  mjModel* m = Load(xml);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  mjtNum verts[64 * 3], edges[256 * 6];
  int nv = GeomVerts(m, d, FirstGeom(m), verts);
  int ne = GeomEdges(m, d, FirstGeom(m), edges);
  EXPECT_EQ(nv, 4);  // tetrahedron vertices
  EXPECT_EQ(ne, 6);  // tetrahedron edges (each shared hull edge emitted once)
  mj_deleteData(d);
  mj_deleteModel(m);
}

// --------------------------------- pair gap ----------------------------------

// vertex-triangle pair: the gap is the point-triangle distance (midsurface:
// radii not subtracted), and (n, cw) is its exact gradient, checked by central
// differences at every involved vertex
TEST_F(ContinuousCollisionTest, PairGapVertexTriangleGradient) {
  mjModel* m = Load(kPrimitivesXml);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  // free points: vertex 0 above the interior of triangle (1, 2, 3)
  mjtNum x[12] = {0.2, 0.2, 0.5, 0, 0, 0, 1, 0, 0, 0, 1, 0};
  mjtNum radii[4] = {0.005, 0.005, 0.005, 0.005};
  mjcFlexPair pair;
  pair.type = mjcFLEX_VERT_TRI;
  pair.idx[0] = 0;
  pair.idx[1] = 1;
  pair.idx[2] = 2;
  pair.idx[3] = 3;
  pair.g = -1;

  mjtNum n[3], cw[4];
  int idv[4], nidx = 0;
  mjtNum g = mjc_pairGap(&pair, m, d, x, nullptr, nullptr, radii, n, idv, cw,
                         &nidx, 1e30);
  // midsurface distance, radii not subtracted
  EXPECT_NEAR(g, 0.5, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(n[0], 0, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(n[1], 0, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(std::fabs(n[2]), 1, MjTol(1e-12, 1e-5));
  EXPECT_GT(nidx, 0);

  // dg/d(vertex idv[p]) = cw[p]*n, by central differences
  mjtNum eps = MjEps(1e-6, 1e-3);
  for (int p = 0; p < nidx; p++) {
    for (int k = 0; k < 3; k++) {
      mjtNum saved = x[3 * idv[p] + k];
      x[3 * idv[p] + k] = saved + eps;
      mjtNum gp = mjc_pairGap(&pair, m, d, x, nullptr, nullptr, radii, n, idv,
                              cw, &nidx, 1e30);
      x[3 * idv[p] + k] = saved - eps;
      mjtNum gm = mjc_pairGap(&pair, m, d, x, nullptr, nullptr, radii, n, idv,
                              cw, &nidx, 1e30);
      x[3 * idv[p] + k] = saved;
      mjtNum g0 = mjc_pairGap(&pair, m, d, x, nullptr, nullptr, radii, n, idv,
                              cw, &nidx, 1e30);
      EXPECT_NEAR(cw[p] * n[k], (gp - gm) / (2 * eps), MjTol(1e-6, 1e-3))
          << "gradient mismatch at involved vertex " << p << " axis " << k
          << " (gap " << g0 << ")";
    }
  }
  mj_deleteData(d);
  mj_deleteModel(m);
}

// ------------------------- conservative advancement --------------------------

// a vertex sweeping through a triangle: the advance caps alpha so the gap keeps
// 20% of its value, reports the pair's own time of impact, and flags it as
// approaching; motion away is uncapped
TEST_F(ContinuousCollisionTest, AdvanceCapsCrossing) {
  mjModel* m = Load(kPrimitivesXml);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  mjtNum x[12] = {0.2, 0.2, 0.5, 0, 0, 0, 1, 0, 0, 0, 1, 0};
  mjtNum radii[4] = {0.005, 0.005, 0.005, 0.005};
  // cross-flex pair: no coherent-motion mean removal
  int pt2flex[4] = {0, 1, 1, 1};
  mjcFlexPair cand;
  cand.type = mjcFLEX_VERT_TRI;
  cand.idx[0] = 0;
  cand.idx[1] = 1;
  cand.idx[2] = 2;
  cand.idx[3] = 3;
  cand.g = -1;

  mjtNum n[3], cw[4];
  int idv[4], nidx = 0;
  mjtNum cgap[1];
  cgap[0] = mjc_pairGap(&cand, m, d, x, nullptr, nullptr, radii, n, idv, cw,
                        &nidx, 1e30);
  ASSERT_NEAR(cgap[0], 0.5, MjTol(1e-12, 1e-5));

  // vertex 0 moves straight down by 1: the full step would end 0.5 below the
  // triangle
  mjtNum dxw[12] = {0, 0, -1};
  int appr[1];
  mjtNum toi[1];
  mjtNum alpha = mjc_advance(m, d, x, dxw, nullptr, nullptr, radii, 4, &cand, 1,
                             cgap, pt2flex, appr, toi);
  // the advance stops when the gap has dropped to 20% of its value:
  // alpha = (0.5 - 0.1)/1 = 0.4
  EXPECT_NEAR(alpha, 0.4, 1e-3);
  EXPECT_LT(toi[0], 1.0);
  EXPECT_EQ(appr[0], 1);

  // moving away at speed 1: the closing-rate bound is conservative (it does not
  // project onto the normal), so the pair still reaches the bisection and is
  // flagged approaching -- but the actual gap grows along the path, so the
  // advance is uncapped and there is no impact
  mjtNum dxw_up[12] = {0, 0, +1};
  alpha = mjc_advance(m, d, x, dxw_up, nullptr, nullptr, radii, 4, &cand, 1,
                      cgap, pt2flex, appr, toi);
  EXPECT_NEAR(alpha, 1.0, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(toi[0], 1.0, MjTol(1e-12, 1e-5));

  // slow motion (well under 80% of the gap): absorbed by the 20% floor without
  // any bisection, whatever its direction
  mjtNum dxw_slow[12] = {0, 0, -0.1};
  alpha = mjc_advance(m, d, x, dxw_slow, nullptr, nullptr, radii, 4, &cand, 1,
                      cgap, pt2flex, appr, toi);
  EXPECT_NEAR(alpha, 1.0, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(toi[0], 1.0, MjTol(1e-12, 1e-5));
  EXPECT_EQ(appr[0], 0);

  mj_deleteData(d);
  mj_deleteModel(m);
}

// --------------------------- candidate generation ----------------------------

// two stacked 2x2 cloths, the upper one dz above the lower
static constexpr char kTwoClothsXml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="lower" type="grid" dim="2" count="2 2 1"
                spacing="0.05 0.05 1" radius="0.005" mass="0.05" pos="0 0 0.5"/>
      <flexcomp name="upper" type="grid" dim="2" count="2 2 1"
                spacing="0.05 0.05 1" radius="0.005" mass="0.05" pos="0 0 %g"/>
    </worldbody>
  </mujoco>)";

// the candidate pairs of the two cloths at their current positions: a static
// query (no sweep) with reach 3*band, band 3 mm; all are flex-flex pairs
static int TwoClothCandidates(const mjModel* m, mjData* d) {
  // free-point arrays over the two dim-2 flexes, in flex order
  int nfd = m->nflex;
  EXPECT_EQ(nfd, 2);
  int flist[2], fxadr[2], nfv = 0;
  for (int k = 0; k < nfd; k++) {
    flist[k] = k;
    fxadr[k] = nfv;
    nfv += m->flex_vertnum[k];
  }
  EXPECT_EQ(nfv, 8);
  mjtNum x[8 * 3], radii[8];
  int fidx[8], pt2flex[8];
  for (int k = 0; k < nfd; k++) {
    for (int v = 0; v < m->flex_vertnum[k]; v++) {
      int pt = fxadr[k] + v, vg = m->flex_vertadr[k] + v;
      for (int c = 0; c < 3; c++) x[3 * pt + c] = d->flexvert_xpos[3 * vg + c];
      radii[pt] = m->flex_radius[k];
      fidx[pt] = pt;
      pt2flex[pt] = k;
    }
  }
  mjtNum band = 0.003;
  mjcFlexPair* cand = nullptr;
  int ncand = mjc_candidates(m, d, x, nullptr, nullptr, nullptr, nullptr, 0, 0,
                             radii, 3 * band, 3 * band, x, x, band, nfv, nfv,
                             fidx, nullptr, flist, fxadr, nfd, pt2flex, &cand);
  for (int c = 0; c < ncand; c++) {
    EXPECT_TRUE(cand[c].type == mjcFLEX_VERT_TRI ||
                cand[c].type == mjcFLEX_EDGE_EDGE)
        << "flex-flex pair types only";
  }
  return ncand;
}

// the swept broad phase finds cross-flex pairs when the cloths are within the
// detection reach and none when they are far apart
TEST_F(ContinuousCollisionTest, CandidatesFindApproachingPairs) {
  for (mjtNum dz : {0.002, 0.5}) {
    char xml_filled[1024];
    snprintf(xml_filled, sizeof(xml_filled), kTwoClothsXml, 0.5 + dz);
    mjModel* m = Load(xml_filled);
    mjData* d = mj_makeData(m);
    mj_forward(m, d);
    int ncand = TwoClothCandidates(m, d);
    if (dz < 0.01) {
      EXPECT_GT(ncand, 0) << "2 mm apart, within reach: pairs expected";
    } else {
      EXPECT_EQ(ncand, 0) << "0.5 m apart, beyond reach: no pairs expected";
    }
    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

// the broad phase applies the native collision filtering: no pairs with the
// contact flag disabled, and the contype/conaffinity rule between the flexes
TEST_F(ContinuousCollisionTest, CandidatesFollowCollisionFiltering) {
  char xml_filled[1024];
  snprintf(xml_filled, sizeof(xml_filled), kTwoClothsXml, 0.502);
  mjModel* m = Load(xml_filled);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);
  int lower = mj_name2id(m, mjOBJ_FLEX, "lower");
  int upper = mj_name2id(m, mjOBJ_FLEX, "upper");
  EXPECT_GT(TwoClothCandidates(m, d), 0) << "default masks: pairs expected";

  // contact disabled
  m->opt.disableflags |= mjDSBL_CONTACT;
  EXPECT_EQ(TwoClothCandidates(m, d), 0) << "contact disabled";
  m->opt.disableflags &= ~mjDSBL_CONTACT;

  // the upper's masks zero
  m->flex_contype[upper] = 0;
  m->flex_conaffinity[upper] = 0;
  EXPECT_EQ(TwoClothCandidates(m, d), 0) << "masks zero";

  // type 2 does not meet the lower's affinity 1, and the lower's type 1 does
  // not meet affinity 2
  m->flex_contype[upper] = 2;
  m->flex_conaffinity[upper] = 2;
  EXPECT_EQ(TwoClothCandidates(m, d), 0) << "incompatible masks";

  // one side's type meeting the other's affinity is enough
  m->flex_conaffinity[lower] = 3;
  EXPECT_GT(TwoClothCandidates(m, d), 0) << "compatible masks";

  mj_deleteData(d);
  mj_deleteModel(m);
}

// the candidates of all the model's flexes for the sweep that moves every point
// of flex `mover` by dz in z (base reach 3*band, band 3 mm, no geom features),
// and in *alpha the fraction of that sweep the CCD allows over them
static int SweptCandidates(const mjModel* m, mjData* d, int mover, mjtNum dz,
                           mjtNum* alpha) {
  int nfd = m->nflex, nfv = 0;
  std::vector<int> flist(nfd), fxadr(nfd);
  for (int k = 0; k < nfd; k++) {
    flist[k] = k;
    fxadr[k] = nfv;
    nfv += m->flex_vertnum[k];
  }
  std::vector<mjtNum> x(3 * nfv), dto(3 * nfv), dxw(3 * nfv), radii(nfv);
  std::vector<int> fidx(nfv), pt2flex(nfv);
  for (int k = 0; k < nfd; k++) {
    for (int v = 0; v < m->flex_vertnum[k]; v++) {
      int pt = fxadr[k] + v, vg = m->flex_vertadr[k] + v;
      for (int c = 0; c < 3; c++) {
        x[3 * pt + c] = d->flexvert_xpos[3 * vg + c];
        dto[3 * pt + c] = x[3 * pt + c] + (k == mover && c == 2 ? dz : 0);
        dxw[3 * pt + c] = dto[3 * pt + c] - x[3 * pt + c];
      }
      radii[pt] = m->flex_radius[k];
      fidx[pt] = pt;
      pt2flex[pt] = k;
    }
  }
  mjtNum band = 0.003;
  mjcFlexPair* cand = nullptr;
  int ncand = mjc_candidates(
      m, d, x.data(), nullptr, nullptr, nullptr, nullptr, 0, 0, radii.data(),
      3 * band, 3 * band, x.data(), dto.data(), band, nfv, nfv, fidx.data(),
      nullptr, flist.data(), fxadr.data(), nfd, pt2flex.data(), &cand);
  std::vector<mjtNum> cgap(ncand > 0 ? ncand : 1);
  for (int c = 0; c < ncand; c++) {
    mjtNum n[3], cw[4];
    int idv[4], nidx;
    cgap[c] = mjc_pairGap(&cand[c], m, d, x.data(), nullptr, nullptr,
                          radii.data(), n, idv, cw, &nidx, 1e30);
  }
  *alpha = mjc_advance(m, d, x.data(), dxw.data(), nullptr, nullptr,
                       radii.data(), nfv, cand, ncand, cgap.data(),
                       pt2flex.data(), nullptr, nullptr);
  return ncand;
}

// the sweep extends the reach: a pair whose gap at the query configuration is
// far outside the band is a candidate when the sweep crosses it, and the CCD
// then bounds the motion. Static queries of the same configurations find
// nothing
TEST_F(ContinuousCollisionTest, CandidatesCoverTheSweep) {
  // a 2x2 cloth 5 cm above the floor, swept 10 cm down through it
  constexpr char kFloor[] = R"(
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="0 0 1"/>
      <flexcomp name="cloth" type="grid" dim="2" count="2 2 1"
                spacing="0.05 0.05 1" radius="0.005" mass="0.05" pos="0 0 0.05"/>
    </worldbody>
  </mujoco>)";
  {
    mjModel* m = Load(kFloor);
    mjData* d = mj_makeData(m);
    mj_forward(m, d);
    mjtNum alpha;
    EXPECT_EQ(SweptCandidates(m, d, 0, 0, &alpha), 0)
        << "at rest: beyond reach";
    EXPECT_EQ(alpha, 1);
    EXPECT_EQ(SweptCandidates(m, d, 0, -0.1, &alpha), 4)
        << "every vertex crosses the floor";
    EXPECT_LT(alpha, 1);
    mj_deleteData(d);
    mj_deleteModel(m);
  }
  // two 2x2 cloths 5 cm apart, the upper swept 10 cm down through the lower
  {
    char xml_filled[1024];
    snprintf(xml_filled, sizeof(xml_filled), kTwoClothsXml, 0.55);
    mjModel* m = Load(xml_filled);
    mjData* d = mj_makeData(m);
    mj_forward(m, d);
    int upper = mj_name2id(m, mjOBJ_FLEX, "upper");
    mjtNum alpha;
    EXPECT_EQ(SweptCandidates(m, d, upper, 0, &alpha), 0)
        << "at rest: beyond reach";
    EXPECT_EQ(alpha, 1);
    EXPECT_GT(SweptCandidates(m, d, upper, -0.1, &alpha), 0)
        << "the sweep crosses the lower cloth";
    EXPECT_LT(alpha, 1);
    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

// a sphere and a capsule expose their centre and axis as features carrying the
// geom's radius: the corner-vs-triangle gap is the centre's distance to the
// triangle less the radius
TEST_F(ContinuousCollisionTest, SmoothGeomFeatures) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom name="ball" type="sphere" size="0.1" pos="0 0 0.5"/>
      <geom name="pill" type="capsule" size="0.05 0.2" pos="1 0 0" euler="0 90 0"/>
      <flexcomp name="cloth" type="grid" dim="2" count="2 2 1"
                spacing="0.05 0.05 1" radius="0.005" mass="0.05" pos="0 0 0"/>
    </worldbody>
  </mujoco>)";
  mjModel* m = Load(xml);
  mjData* d = mj_makeData(m);
  mj_forward(m, d);
  int ball = mj_name2id(m, mjOBJ_GEOM, "ball");
  int pill = mj_name2id(m, mjOBJ_GEOM, "pill");
  mjtNum verts[2 * 3], edges[6];

  // the sphere: one feature at its centre, no edge
  EXPECT_EQ(GeomVerts(m, d, ball, verts), 1);
  EXPECT_EQ(GeomEdges(m, d, ball, edges), 0);
  EXPECT_NEAR(verts[2], 0.5, MjTol(1e-12, 1e-5));

  // the capsule: its two axis endpoints, half-length 0.2 along its axis (x
  // after the rotation), and the axis as one edge
  EXPECT_EQ(GeomVerts(m, d, pill, verts), 2);
  EXPECT_EQ(GeomEdges(m, d, pill, edges), 1);
  EXPECT_NEAR(std::fabs(verts[0] - 1), 0.2, MjTol(1e-9, 1e-5));
  EXPECT_NEAR(std::fabs(verts[3] - 1), 0.2, MjTol(1e-9, 1e-5));
  EXPECT_NEAR(std::fabs(edges[3] - edges[0]), 0.4, MjTol(1e-9, 1e-5));

  // the sphere's centre against the cloth's first triangle (the cloth lies in
  // the plane z=0 under the centre): distance 0.5 less the radius 0.1
  GeomVerts(m, d, ball, verts);
  mjtNum x[4 * 3], radii[4] = {0.005, 0.005, 0.005, 0.005};
  for (int v = 0; v < 4; v++)
    for (int c = 0; c < 3; c++) x[3 * v + c] = d->flexvert_xpos[3 * v + c];
  const int* el = m->flex_elem + m->flex_elemdataadr[0];
  mjcFlexPair pair = {mjcGEOM_CORNER_TRI, {0, el[0], el[1], el[2]}, ball};
  mjtNum n[3], cw[4];
  int idv[4], nidx;
  mjtNum g = mjc_pairGap(&pair, m, d, x, verts, nullptr, radii, n, idv, cw,
                         &nidx, 1e30);
  EXPECT_NEAR(g, 0.4, MjTol(1e-9, 1e-5));
  EXPECT_EQ(nidx, 3);
  EXPECT_NEAR(n[2], 1,
              MjTol(1e-9, 1e-5));  // from the triangle up to the centre

  mj_deleteData(d);
  mj_deleteModel(m);
}

// the closing-bound prune stays conservative under deformation: the closest
// points of a pair can move to another region of the features during the step,
// so the bound must cover every pair of vertices, not the closest points'
// weights at the query configuration. A triangle turned a quarter turn about an
// in-plane axis through the closest point to a vertex above it leaves that
// closest point in place, so the weighted bound reads zero, and sweeps the
// triangle through the vertex
TEST_F(ContinuousCollisionTest, CandidatesCoverARotatingTriangle) {
  constexpr char kXml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="tri" type="direct" dim="2" point="0 0 0  1 0 0  0 1 0" element="0 1 2"
                radius="0.001" mass="0.03"/>
      <flexcomp name="pt" type="direct" dim="2" point="0.3 0.3 0.05  -1.7 -1.7 3  -1.7 0.3 3"
                element="0 1 2" radius="0.001" mass="0.03"/>
    </worldbody>
  </mujoco>)";
  mjModel* m = Load(kXml);
  ASSERT_THAT(m, NotNull());
  mjData* d = mj_makeData(m);
  mj_forward(m, d);
  int nfd = m->nflex, nfv = 0;
  std::vector<int> flist(nfd), fxadr(nfd);
  for (int k = 0; k < nfd; k++) {
    flist[k] = k;
    fxadr[k] = nfv;
    nfv += m->flex_vertnum[k];
  }
  ASSERT_EQ(nfv, 6);
  std::vector<mjtNum> x(3 * nfv), dto(3 * nfv), dxw(3 * nfv), radii(nfv);
  std::vector<int> fidx(nfv), pt2flex(nfv);
  // the triangle turns about the axis through Q = (0.3, 0.3, 0), the closest
  // point to the vertex P = (0.3, 0.3, 0.05), along (1, -1, 0): it ends
  // vertical in the plane x + y = 0.6 with P inside
  // (a literal root: MSVC has no M_SQRT1_2 without _USE_MATH_DEFINES)
  const mjtNum r2 = mju_sqrt(0.5);
  const mjtNum Q[3] = {0.3, 0.3, 0}, ax[3] = {r2, -r2, 0};
  for (int k = 0; k < nfd; k++) {
    for (int v = 0; v < m->flex_vertnum[k]; v++) {
      int pt = fxadr[k] + v, vg = m->flex_vertadr[k] + v;
      mju_copy3(&x[3 * pt], d->flexvert_xpos + 3 * vg);
      if (k == 0) {
        mjtNum r[3], axr[3];
        mju_sub3(r, &x[3 * pt], Q);
        mju_cross(axr, ax, r);
        mjtNum along = mju_dot3(ax, r);
        for (int c = 0; c < 3; c++)
          dto[3 * pt + c] = Q[c] + axr[c] + along * ax[c];
      } else {
        mju_copy3(&dto[3 * pt], &x[3 * pt]);
      }
      mju_sub3(&dxw[3 * pt], &dto[3 * pt], &x[3 * pt]);
      radii[pt] = m->flex_radius[k];
      fidx[pt] = pt;
      pt2flex[pt] = k;
    }
  }
  mjtNum band = 0.003;
  mjcFlexPair* cand = nullptr;
  int ncand = mjc_candidates(
      m, d, x.data(), nullptr, nullptr, nullptr, nullptr, 0, 0, radii.data(),
      3 * band, 3 * band, x.data(), dto.data(), band, nfv, nfv, fidx.data(),
      nullptr, flist.data(), fxadr.data(), nfd, pt2flex.data(), &cand);
  int found = 0;
  for (int c = 0; c < ncand; c++) {
    if (cand[c].type == mjcFLEX_VERT_TRI && cand[c].idx[0] == 3) found++;
  }
  EXPECT_EQ(found, 1)
      << "the vertex the triangle sweeps through is a candidate among "
      << ncand;
  std::vector<mjtNum> cgap(ncand > 0 ? ncand : 1);
  for (int c = 0; c < ncand; c++) {
    mjtNum n[3], cw[4];
    int idv[4], nidx;
    cgap[c] = mjc_pairGap(&cand[c], m, d, x.data(), nullptr, nullptr,
                          radii.data(), n, idv, cw, &nidx, 1e30);
  }
  mjtNum alpha = mjc_advance(m, d, x.data(), dxw.data(), nullptr, nullptr,
                             radii.data(), nfv, cand, ncand, cgap.data(),
                             pt2flex.data(), nullptr, nullptr);
  EXPECT_LT(alpha, 1) << "the sweep through the vertex is capped";
  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
