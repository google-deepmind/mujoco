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
  int fidx[4] = {0, 1, 2, 3};  // all points free, identity map
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
  mjtNum alpha = mjc_advance(m, d, x, dxw, nullptr, nullptr, radii, 4, fidx,
                             &cand, 1, cgap, pt2flex, appr, toi);
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
  alpha = mjc_advance(m, d, x, dxw_up, nullptr, nullptr, radii, 4, fidx, &cand,
                      1, cgap, pt2flex, appr, toi);
  EXPECT_NEAR(alpha, 1.0, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(toi[0], 1.0, MjTol(1e-12, 1e-5));

  // slow motion (well under 80% of the gap): absorbed by the 20% floor without
  // any bisection, whatever its direction
  mjtNum dxw_slow[12] = {0, 0, -0.1};
  alpha = mjc_advance(m, d, x, dxw_slow, nullptr, nullptr, radii, 4, fidx,
                      &cand, 1, cgap, pt2flex, appr, toi);
  EXPECT_NEAR(alpha, 1.0, MjTol(1e-12, 1e-5));
  EXPECT_NEAR(toi[0], 1.0, MjTol(1e-12, 1e-5));
  EXPECT_EQ(appr[0], 0);

  mj_deleteData(d);
  mj_deleteModel(m);
}

// --------------------------- candidate generation ----------------------------

// two stacked cloths: the swept broad phase finds cross-flex pairs when they
// are within the detection reach and none when they are far apart
TEST_F(ContinuousCollisionTest, CandidatesFindApproachingPairs) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <flexcomp name="lower" type="grid" dim="2" count="2 2 1"
                spacing="0.05 0.05 1" radius="0.005" mass="0.05" pos="0 0 0.5"/>
      <flexcomp name="upper" type="grid" dim="2" count="2 2 1"
                spacing="0.05 0.05 1" radius="0.005" mass="0.05" pos="0 0 %g"/>
    </worldbody>
  </mujoco>)";

  for (mjtNum dz : {0.002, 0.5}) {
    char xml_filled[1024];
    snprintf(xml_filled, sizeof(xml_filled), xml, 0.5 + dz);
    mjModel* m = Load(xml_filled);
    mjData* d = mj_makeData(m);
    mj_forward(m, d);

    // free-point arrays over the two dim-2 flexes, in flex order
    int nfd = m->nflex;
    ASSERT_EQ(nfd, 2);
    int flist[2], fxadr[2], nfv = 0;
    for (int k = 0; k < nfd; k++) {
      flist[k] = k;
      fxadr[k] = nfv;
      nfv += m->flex_vertnum[k];
    }
    ASSERT_EQ(nfv, 8);
    mjtNum x[8 * 3], radii[8];
    int fidx[8], pt2flex[8];
    for (int k = 0; k < nfd; k++) {
      for (int v = 0; v < m->flex_vertnum[k]; v++) {
        int pt = fxadr[k] + v, vg = m->flex_vertadr[k] + v;
        for (int c = 0; c < 3; c++)
          x[3 * pt + c] = d->flexvert_xpos[3 * vg + c];
        radii[pt] = m->flex_radius[k];
        fidx[pt] = pt;
        pt2flex[pt] = k;
      }
    }

    // static query (no sweep): reach = 3*band, band 3 mm
    mjtNum band = 0.003;
    mjcFlexPair cand[256];
    int ncand = mjc_candidates(m, d, x, nullptr, nullptr, 0, 0, radii, 3 * band,
                               3 * band, 0.0, x, x, band, nfv, nfv, fidx, flist,
                               fxadr, nfd, pt2flex, cand, 256);
    if (dz < 0.01) {
      EXPECT_GT(ncand, 0) << "2 mm apart, within reach: pairs expected";
      for (int c = 0; c < ncand; c++) {
        EXPECT_TRUE(cand[c].type == mjcFLEX_VERT_TRI ||
                    cand[c].type == mjcFLEX_EDGE_EDGE)
            << "flex-flex pair types only";
      }
    } else {
      EXPECT_EQ(ncand, 0) << "0.5 m apart, beyond reach: no pairs expected";
    }
    mj_deleteData(d);
    mj_deleteModel(m);
  }
}

}  // namespace
}  // namespace mujoco
