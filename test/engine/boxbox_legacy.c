// Copyright 2016 Svetoslav Kolev
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


// The box-box collider as it stood before the separating-axis rewrite (MuJoCo 3.11.1),
// preserved verbatim as a test-only fixture so the rewrite's improvements are measured
// rather than asserted. Nothing in the engine links this file; it is built only into the
// box-box tests.

#include "test/engine/boxbox_legacy.h"

#include <math.h>

#include <mujoco/mujoco.h>
#include "src/engine/engine_inline.h"
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_misc.h"

// rounding slack for the edge-edge depth bound: relative, and absolute times the sum of
// half-sizes; wide enough to cover rounding between two computations of the same overlap,
// orders of magnitude below the box-scale depths of spurious clipping artifacts
#ifdef mjUSESINGLE
  #define mjDEPTHSLACKREL 1e-4f
  #define mjDEPTHSLACKABS 1e-5f
#else
  #define mjDEPTHSLACKREL 1e-6
  #define mjDEPTHSLACKABS 1e-12
#endif


// internal box : box
static inline
int boxboxLegacyRaw(const mjModel* M, const mjData* D, mjPreContact* con, int g1, int g2, mjtNum margin) {
  const mjtNum* pos1 = D->geom_xpos + 3 * g1;
  const mjtNum* pos2 = D->geom_xpos + 3 * g2;
  const mjtNum* mat1 = D->geom_xmat + 9 * g1;
  const mjtNum* mat2 = D->geom_xmat + 9 * g2;
  const mjtNum* size1 = M->geom_size + 3 * g1;
  const mjtNum* size2 = M->geom_size + 3 * g2;

  mjtNum pos12[3], pos21[3], rot[9], rott[9], rotabs[9], rottabs[9], tmp1[3], tmp2[3], plen1[3],
         plen2[3];
  mjtNum rotmore[9], p[3], r[9], s[3], ss[3], lp[3], rt[9], points[mjMAXCONPAIR][3],
         depth[mjMAXCONPAIR], pts[6][3], ppts2[4][2], pu[4][3], axi[3][3];
  mjtNum linesu[4][6], lines[4][6], clnorm[3], rnorm[3];
  mjtNum penetration, c1, c2, c3, a, b, c, d, lx, ly, hz, l, x, y, u, v, llx, lly, innorm, margin2;
  mjtNum maxdepth;

  int i0, i1, i2;
  mjtNum f0, f1, f2;

  int i, j, q, code, q1, q2, clcorner, n, m, k;
  int cle1, cle2, in, ax1, ax2, pax1, pax2, clface, nl, nf;

  n = 0;
  code = -1;
  margin2 = margin * margin;

  mji_sub3(tmp1, pos2, pos1);
  mji_mulMatTVec3(pos21, mat1, tmp1);

  mji_sub3(tmp1, pos1, pos2);
  mji_mulMatTVec3(pos12, mat2, tmp1);

  mju_mulMatTMat3(rot, mat1, mat2);
  mju_transpose(rott, rot, 3, 3);

  for (i = 0; i < 9; i++)
    rotabs[i] = mju_abs(rot[i]);
  for (i = 0; i < 9; i++)
    rottabs[i] = mju_abs(rott[i]);

  mji_mulMatVec3(plen2, rotabs, size2);
  mji_mulMatTVec3(plen1, rotabs, size1);

  for (i = 0, penetration = margin; i < 3; i++)
    penetration += size1[i] * 3 + size2[i] * 3;

  for (i = 0; i < 3; i++) {
    c1 = -mju_abs(pos21[i]) + size1[i] + plen2[i];
    c2 = -mju_abs(pos12[i]) + size2[i] + plen1[i];

    if (c1 < -margin || c2 < -margin)
      return 0;

    if (c1 < penetration) {
      penetration = c1;
      code = i + 3 * (pos21[i] < 0) + 0;
    }
    if (c2 < penetration) {
      penetration = c2;
      code = i + 3 * (pos12[i] < 0) + 6;
    }

    // printf("%24.16e %24.16e %d         %24.16e %d \n",c1,c2,i,penetration,code);
  }

  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      mju_zero3(tmp2);
      if (i == 0) {
        tmp2[1] = -rott[3 * j + 2];
        tmp2[2] = +rott[3 * j + 1];
      } else if (i == 1) {
        tmp2[0] = +rott[3 * j + 2];
        tmp2[2] = -rott[3 * j + 0];
      } else if (i == 2) {
        tmp2[0] = -rott[3 * j + 1];
        tmp2[1] = +rott[3 * j + 0];
      }

      c1 = mju_normalize3(tmp2);


      if (c1 < mjMINVAL)
        continue;

      c2 = mju_dot3(pos21, tmp2);

      c3 = 0;

      for (k = 0; k < 3; k++)
        if (k != i)
          c3 += size1[k] * mju_abs(tmp2[k]);
      for (k = 0; k < 3; k++)
        if (k != j)
          c3 += size2[k] * rotabs[3 * i + 3 - k - j] / c1;

      c3 -= mju_abs(c2);

      if (c3 < -margin)
        return 0;



      if (c3 < penetration * (1 - 1e-12))
      {
        penetration = c3;
        for (k = cle1 = 0; k < 3; k++)
          if (k != i)
            if ((tmp2[k] > 0) ^ (c2 < 0))
              cle1 += 1 << k;
        for (k = cle2 = 0; k < 3; k++)
          if (k != j)
            if ((rot[3 * i + 3 - k - j] > 0) ^ (c2 < 0) ^ ((k - j + 3) % 3 == 1))
              cle2 += 1 << k;

        code = 12 + i * 3 + j;
        mji_copy3(clnorm, tmp2);
        in = c2 < 0;
      }

      // printf("%24.16e %d      %24.16e %d\n",c3,12+i*3+j,penetration,code);
    }
  }


  // return 0;


  // printf("%d\n",code);

  if (code == -1)
    return 0;  // shouldn't happen

  if (code >= 12)
    goto edgeedge;


  q1 = code % 6;
  q2 = code / 6;

  // printf("%d %d\n",q1,q2);

  mju_zero(rotmore, 9);
  if (q1 == 0)
    rotmore[2] = -1, rotmore[4] = +1, rotmore[6] = +1;
  else if (q1 == 1)
    rotmore[0] = +1, rotmore[5] = -1, rotmore[7] = +1;
  else if (q1 == 2)
    rotmore[0] = +1, rotmore[4] = +1, rotmore[8] = +1;
  else if (q1 == 3)
    rotmore[2] = +1, rotmore[4] = +1, rotmore[6] = -1;
  else if (q1 == 4)
    rotmore[0] = +1, rotmore[5] = +1, rotmore[7] = -1;
  else if (q1 == 5)
    rotmore[0] = -1, rotmore[4] = +1, rotmore[8] = -1;

  i0 = 0;
  i1 = 1;
  i2 = 2;
  f0 = f1 = f2 = 1;

  if (q1 == 0) {
    i0 = 2;
    f0 = -1;
    i2 = 0;
  } else if (q1 == 1) {
    i1 = 2;
    f1 = -1;
    i2 = 1;
  } else if (q1 == 2) {
  } else if (q1 == 3) {
    i0 = 2;
    i2 = 0;
    f2 = -1;
  } else if (q1 == 4) {
    i1 = 2;
    i2 = 1;
    f2 = -1;
  } else if (q1 == 5) {
    f0 = -1;
    f2 = -1;
  }


#define rotaxis(vecres, vecin) \
{                              \
  vecres[0]=vecin[i0]*f0;      \
  vecres[1]=vecin[i1]*f1;      \
  vecres[2]=vecin[i2]*f2;      \
}
#define rotmatx(matres, matin)        \
{                                     \
  mji_scl3(matres+0, matin+i0*3, f0); \
  mji_scl3(matres+3, matin+i1*3, f1); \
  mji_scl3(matres+6, matin+i2*3, f2); \
}

  if (q2) {
    mju_mulMatMatT3(r, rotmore, rot);

    // mju_mulMatVec3(p,rotmore,pos12);
    // mju_mulMatVec3(tmp1,rotmore,size2);

    rotaxis(p, pos12);
    rotaxis(tmp1, size2);

    mji_copy3(s, size1);
  } else {
    // mju_mulMatMat(r,rotmore,rot,3,3,3);

    rotmatx(r, rot);

    // mju_mulMatVec3(p,rotmore,pos21);
    // mju_mulMatVec3(tmp1,rotmore,size1);

    rotaxis(p, pos21);
    rotaxis(tmp1, size1);

    mji_copy3(s, size2);
  }

  mju_transpose(rt, r, 3, 3);

  for (i = 0; i < 3; i++)
    ss[i] = mju_abs(tmp1[i]);

  lx = ss[0];
  ly = ss[1];
  hz = ss[2];
  p[2] -= hz;

  mji_copy3(lp, p);

  for (clcorner = 0, i = 0; i < 3; i++)
    if (r[6 + i] < 0)
      clcorner += 1 << i;

  mji_addToScl3(lp, rt + 0, s[0] * ((clcorner & 1) ? 1 : -1));
  mji_addToScl3(lp, rt + 3, s[1] * ((clcorner & 2) ? 1 : -1));
  mji_addToScl3(lp, rt + 6, s[2] * ((clcorner & 4) ? 1 : -1));

  m = k = 0;
  mji_copy3(pts[m++], lp);

  for (i = 0; i < 3; i++)
    if (mju_abs(r[6 + i]) < 0.5)
      mju_scl3(pts[m++], rt + 3 * i, s[i] * ((clcorner & (1 << i)) ? -2 : 2));

  mji_add3(pts[3], pts[0], pts[1]);
  mji_add3(pts[4], pts[0], pts[2]);
  mji_add3(pts[5], pts[3], pts[2]);

  if (m > 1)
  {
    mji_copy3(lines[k] + 0, pts[0]);
    mji_copy3(lines[k++] + 3, pts[1]);
  }
  if (m > 2)
  {
    mji_copy3(lines[k] + 0, pts[0]);
    mji_copy3(lines[k++] + 3, pts[2]);
    mji_copy3(lines[k] + 0, pts[3]);
    mji_copy3(lines[k++] + 3, pts[2]);
    mji_copy3(lines[k] + 0, pts[4]);
    mji_copy3(lines[k++] + 3, pts[1]);
  }

  for (i = 0; i < k; i++) {
    for (q = 0; q < 2; q++) {
      a = lines[i][0 + q];
      b = lines[i][3 + q];
      c = lines[i][1 - q];
      d = lines[i][4 - q];

      if (mju_abs(b) > mjMINVAL) {
        for (j = -1; j <= 1; j += 2) {
          l = ss[q] * j;
          c1 = (l - a) * (1 / b);
          if (c1 < 0 || c1 > 1)
            continue;
          c2 = c + d * c1;
          if (mju_abs(c2) > ss[1 - q])
            continue;

          if (n < mjMAXCONPAIR) {
            mji_copy3(points[n], lines[i]);
            mji_addToScl3(points[n++], lines[i] + 3, c1);
          }
        }
      }
    }
  }


  a = pts[1][0];
  b = pts[2][0];
  c = pts[1][1];
  d = pts[2][1];
  c1 = a * d - b * c;


  if (m > 2) {
    for (i = 0; i < 4; i++) {
      llx = i / 2 ? lx : -lx;
      lly = i % 2 ? ly : -ly;

      x = llx - pts[0][0];
      y = lly - pts[0][1];

      u = (x * d - y * b) * (1 / c1);
      v = (y * a - x * c) * (1 / c1);
      if (u <= 0 || v <= 0 || u >= 1 || v >= 1)
        continue;

      if (n < mjMAXCONPAIR) {
        points[n][0] = llx;
        points[n][1] = lly;
        points[n][2] = (pts[0][2] + u * pts[1][2] + v * pts[2][2]);
        n++;
      }
    }
  }

  for (i = 0; i < (1 << (m - 1)); i++) {
    mji_copy3(tmp1, pts[i == 0 ? 0 : i + 2]);


    if (i)
      if (tmp1[0] <= -lx || tmp1[0] >= lx)
        continue;
    if (i)
      if (tmp1[1] <= -ly || tmp1[1] >= ly)
        continue;

    if (n < mjMAXCONPAIR) {
      mji_copy3(points[n++], tmp1);
    }
  }


  m = n;
  n = 0;

  for (i = 0; i < m; i++) {
    if (points[i][2] > margin)
      continue;
    if (n != i) mji_copy3(points[n], points[i]);

    depth[n] = points[n][2];
    points[n][2] *= 0.5;

    n++;
  }


  mju_mulMatMatT3(r, q2 ? mat2 : mat1, rotmore);
  mju_copy3(p, q2 ? pos2 : pos1);

  tmp2[0] = (q2 ? -1 : 1) * r[2];
  tmp2[1] = (q2 ? -1 : 1) * r[5];
  tmp2[2] = (q2 ? -1 : 1) * r[8];

  mji_copy3(con[0].normal, tmp2);
  mji_zero3(con[0].tangent);




  for (i = 0; i < n; i++) {
    con[i].dist = 2 * points[i][2];
    points[i][2] += hz;

    mji_mulMatVec3(tmp2, r, points[i]);
    mji_add3(con[i].pos, tmp2, p);

    if (i) {
      mji_copy3(con[i].normal, con[0].normal);
      mji_zero3(con[i].tangent);
    }
  }


  // printf("Path1:  %d\n",n);


  return n;

edgeedge:


  code -= 12;

  q1 = code / 3;
  q2 = code % 3;



  if (q2 == 0)
    ax1 = 1, ax2 = 2;
  if (q2 == 1)
    ax1 = 0, ax2 = 2;
  if (q2 == 2)
    ax1 = 1, ax2 = 0;
  if (q1 == 0)
    pax1 = 1, pax2 = 2;
  if (q1 == 1)
    pax1 = 0, pax2 = 2;
  if (q1 == 2)
    pax1 = 1, pax2 = 0;

  // printf("%lf %lf   %lf %lf\n",rot[ 3*q1+ ax1],rot [3*q1+ ax2],rott[3*q2+pax1],rott[3*q2+pax2]);
  // printf("%lf %lf\n",mju_dot3(clnorm,rott+3*ax1),mju_dot3(clnorm,rott+3*ax2));

  if (rotabs [3 * q1 + ax1] < rotabs [3 * q1 + ax2]) {
    ax1 = ax2;
    ax2 = 3 - q2 - ax1;
  }
  if (rottabs[3 * q2 + pax1] < rottabs[3 * q2 + pax2]) {
    pax1 = pax2;
    pax2 = 3 - q1 - pax1;
  }

  if (cle1 & (1 << pax2))
    clface = pax2;
  else
    clface = pax2 + 3;


  // printf("%lf - %d %d %d %d   %d %d     %d %d %d %d %d\n",
  //        penetration,cle1,cle2,code,in,q1,q2,clface,ax1,ax2,pax1,pax2);


  mju_zero(rotmore, 9);
  if (clface == 0)
    rotmore[2] = -1, rotmore[4] = +1, rotmore[6] = +1;
  else if (clface == 1)
    rotmore[0] = +1, rotmore[5] = -1, rotmore[7] = +1;
  else if (clface == 2)
    rotmore[0] = +1, rotmore[4] = +1, rotmore[8] = +1;
  else if (clface == 3)
    rotmore[2] = +1, rotmore[4] = +1, rotmore[6] = -1;
  else if (clface == 4)
    rotmore[0] = +1, rotmore[5] = +1, rotmore[7] = -1;
  else if (clface == 5)
    rotmore[0] = -1, rotmore[4] = +1, rotmore[8] = -1;


  i0 = 0;
  i1 = 1;
  i2 = 2;
  f0 = f1 = f2 = 1;

  if (clface == 0) {
    i0 = 2;
    f0 = -1;
    i2 = 0;
  } else if (clface == 1) {
    i1 = 2;
    f1 = -1;
    i2 = 1;
  } else if (clface == 2) {
  } else if (clface == 3) {
    i0 = 2;
    i2 = 0;
    f2 = -1;
  } else if (clface == 4) {
    i1 = 2;
    i2 = 1;
    f2 = -1;
  } else if (clface == 5) {
    f0 = -1;
    f2 = -1;
  }

  // mju_mulMatVec3(p,rotmore,pos21);
  // mju_mulMatVec3(rnorm,rotmore,clnorm);
  rotaxis(p, pos21);
  rotaxis(rnorm, clnorm);

  // print("rnorm",rnorm);

  // mju_mulMatMat(r,rotmore,rot,3,3,3);
  rotmatx(r, rot);

  mji_mulMatTVec3(tmp1, rotmore, size1);
  for (i = 0; i < 3; i++)
    s[i] = mju_abs(tmp1[i]);

  mju_transpose(rt, r, 3, 3);


  lx = s[0];
  ly = s[1];
  hz = s[2];
  p[2] -= hz;


  n = 0;
  mji_copy3(points[n], p);
  mji_addToScl3(points[n], rt + 3 * ax1, size2[ax1] * ((cle2 & (1 << ax1)) ? 1 : -1));
  mji_addToScl3(points[n], rt + 3 * ax2, size2[ax2] * ((cle2 & (1 << ax2)) ? 1 : -1));
  mji_copy3(points[n + 1], points[n]);
  mji_addToScl3(points[n], rt + 3 * q2, size2[q2]);
  n = 1;
  mji_addToScl3(points[n], rt + 3 * q2, -size2[q2]);
  n = 2;


  mji_copy3(points[n], p);
  mji_addToScl3(points[n], rt + 3 * ax1, size2[ax1] * ((cle2 & (1 << ax1)) ? -1 : 1));
  mji_addToScl3(points[n], rt + 3 * ax2, size2[ax2] * ((cle2 & (1 << ax2)) ? 1 : -1));
  mji_copy3(points[n + 1], points[n]);
  mji_addToScl3(points[n], rt + 3 * q2, size2[q2]);
  n = 3;
  mji_addToScl3(points[n], rt + 3 * q2, -size2[q2]);
  n = 4;


  mji_copy3(axi[0], points[0]);
  mji_sub3(axi[1], points[1], points[0]);
  mji_sub3(axi[2], points[2], points[0]);


  if (mju_abs(rnorm[2]) < mjMINVAL)
    return 0;  // shouldn't happen

  innorm = (1 / rnorm[2]) * (in ? -1 : 1);
  // printf("%lf\n",innorm);

  for (i = 0; i < 4; i++)
  {
    c1 = -points[i][2] * (1 / rnorm[2]);

    mji_copy3(pu[i], points[i]);

    mji_addToScl3(points[i], rnorm, c1);

    // ppts[i][0]=points[i][0];
    // ppts[i][1]=points[i][1];
    ppts2[i][0] = points[i][0];
    ppts2[i][1] = points[i][1];
  }


  mji_copy3(pts[0], points[0]);
  mji_sub3(pts[1], points[1], points[0]);
  mji_sub3(pts[2], points[2], points[0]);

  m = 3;
  k = 0;
  n = 0;


  if (m > 1) {
    mji_copy3(lines[k] + 0, pts[0]);
    mji_copy3(lines[k] + 3, pts[1]);
    mji_copy3(linesu[k] + 0, axi[0]);
    mji_copy3(linesu[k++] + 3, axi[1]);
  }
  if (m > 2) {
    mji_copy3(lines[k] + 0, pts[0]);
    mji_copy3(lines[k] + 3, pts[2]);
    mji_copy3(linesu[k] + 0, axi[0]);
    mji_copy3(linesu[k++] + 3, axi[2]);

    mji_add3(lines[k] + 0, pts[0], pts[1]);
    mji_copy3(lines[k] + 3, pts[2]);
    mji_add3(linesu[k] + 0, axi[0], axi[1]);
    mji_copy3(linesu[k++] + 3, axi[2]);

    mji_add3(lines[k] + 0, pts[0], pts[2]);
    mji_copy3(lines[k] + 3, pts[1]);
    mji_add3(linesu[k] + 0, axi[0], axi[2]);
    mji_copy3(linesu[k++] + 3, axi[1]);
  }

  for (i = 0; i < k; i++) {
    for (q = 0; q < 2; q++) {
      a = lines[i][0 + q];
      b = lines[i][3 + q];
      c = lines[i][1 - q];
      d = lines[i][4 - q];

      if (mju_abs(b) > mjMINVAL) {
        for (j = -1; j <= 1; j += 2) {
          if (n < mjMAXCONPAIR) {
            l = s[q] * j;
            c1 = (l - a) * (1 / b);
            if (c1 < 0 || c1 > 1)
              continue;
            c2 = c + d * c1;
            if (mju_abs(c2) > s[1 - q])
              continue;

            if ((linesu[i][2] + linesu[i][5]*c1)*innorm > margin)
              continue;

            mji_scl3(points[n], linesu[i], 0.5);
            mji_addToScl3(points[n], linesu[i] + 3, 0.5 * c1);
            points[n][0 + q] += 0.5 * l;
            points[n][1 - q] += 0.5 * c2;
            depth[n] = points[n][2] * innorm * 2;
            n++;
          }
        }
      }
    }
  }

  nl = n;

  a = pts[1][0];
  b = pts[2][0];
  c = pts[1][1];
  d = pts[2][1];
  c1 = a * d - b * c;

  for (i = 0; i < 4; i++) {
    if (n < mjMAXCONPAIR) {
      llx = i / 2 ? lx : -lx;
      lly = i % 2 ? ly : -ly;

      x = llx - pts[0][0];
      y = lly - pts[0][1];

      u = (x * d - y * b) * (1 / c1);
      v = (y * a - x * c) * (1 / c1);

      if (nl == 0) {
        if ((u < 0 || u > 1) && (v < 0 || v > 1))
          continue;
      } else {
        if ((u < 0 || u > 1 || v < 0 || v > 1))
          continue;
      }

      if (u < 0)
        u = 0;
      if (u > 1)
        u = 1;
      if (v < 0)
        v = 0;
      if (v > 1)
        v = 1;


      mji_scl3(tmp1, pu[0], 1 - u - v);
      mji_addToScl3(tmp1, pu[1], u);
      mji_addToScl3(tmp1, pu[2], v);

      points[n][0] = llx;
      points[n][1] = lly;
      points[n][2] = 0;

      mji_sub3(tmp2, points[n], tmp1);

      c1 = mju_dot3(tmp2, tmp2);
      if (tmp1[2] > 0)
        if (c1 > margin2)
          continue;

      mji_addTo3(points[n], tmp1);
      mju_scl3(points[n], points[n], 0.5);

      depth[n] = sqrt(c1) * (tmp1[2] < 0 ? -1 : 1);
      n++;
    }
  }

  nf = n;

  for (i = 0; i < 4; i++) {
    if (n < mjMAXCONPAIR) {
      x = ppts2[i][0];
      y = ppts2[i][1];

      if (nl == 0) {
        if (nf == 0) {
        } else {
          if (x < -lx || x > lx)
            if (y < -ly || y > ly)
              continue;
        }
      } else {
        if (x < -lx || x > lx || y < -ly || y > ly)
          continue;
      }

      for (c1 = 0, j = 0; j < 2; j++)
        if (ppts2[i][j] < -s[j])
          c1 += (ppts2[i][j] + s[j]) * (ppts2[i][j] + s[j]);
        else if (ppts2[i][j] > s[j])
          c1 += (ppts2[i][j] - s[j]) * (ppts2[i][j] - s[j]);

      c1 += pu[i][2] * innorm * pu[i][2] * innorm;

      if (pu[i][2] > 0)
        if (c1 > margin2)
          continue;


      tmp1[0] = ppts2[i][0] * 0.5;
      tmp1[1] = ppts2[i][1] * 0.5;
      tmp1[2] = 0;

      for (j = 0; j < 2; j++) {
        if (ppts2[i][j] < -s[j])
          tmp1[j] = -s[j] * 0.5;
        else if (ppts2[i][j] > s[j])
          tmp1[j] = +s[j] * 0.5;
      }
      mji_addToScl3(tmp1, pu[i], 0.5);
      mji_copy3(points[n], tmp1);

      depth[n] = sqrt(c1) * (pu[i][2] < 0 ? -1 : 1);
      n++;
    }
  }

  mju_mulMatMatT3(r, mat1, rotmore);

  mji_mulMatVec3(tmp1, r, rnorm);

  mji_scl3(con[0].normal, tmp1, in ? -1 : 1);
  mji_zero3(con[0].tangent);


  // no contact can be deeper than the support overlap along the separating axis: clipping
  // against a grazing face can synthesize spurious points with arbitrarily large depth;
  // the slack covers rounding error so the deepest legitimate point is never rejected
  maxdepth = mju_max(0, penetration);
  maxdepth += margin + mjDEPTHSLACKREL * maxdepth +
              mjDEPTHSLACKABS * (size1[0] + size1[1] + size1[2] +
                                 size2[0] + size2[1] + size2[2]);

  for (i = 0, m = 0; i < n; i++) {
    if (depth[i] < -maxdepth)
      continue;

    con[m].dist = depth[i];
    points[i][2] += hz;

    mji_mulMatVec3(tmp2, r, points[i]);

    mji_add3(con[m].pos, tmp2, pos1);

    mji_copy3(con[m].normal, con[0].normal);
    mji_zero3(con[m].tangent);
    m++;
  }

  return m;

#undef rotaxis
#undef rotmatx
}


// box : box
int mjc_BoxBoxLegacy(const mjModel* m, mjData* d, mjPreContact* con, int g1, int g2, mjtNum margin) {
  mjPreContact tmp[mjMAXCONPAIR];
  int num = boxboxLegacyRaw(m, d, tmp, g1, g2, margin);

  // -1: bad, 0: good
  int dupe[mjMAXCONPAIR] = {0};


  // get box info
  const mjtNum* pos1 =  d->geom_xpos + 3 * g1;
  const mjtNum* mat1 =  d->geom_xmat + 9 * g1;
  const mjtNum* size1 = m->geom_size + 3 * g1;
  const mjtNum* pos2 =  d->geom_xpos + 3 * g2;
  const mjtNum* mat2 =  d->geom_xmat + 9 * g2;
  const mjtNum* size2 = m->geom_size + 3 * g2;

  // find bad: contacts outside one of the boxes
  int nbad = 0;
  for (int i=0; i < num; i++) {
    // box sizes with margin
    mjtNum sz1[3] = {size1[0] + margin, size1[1] + margin, size1[2] + margin};
    mjtNum sz2[3] = {size2[0] + margin, size2[1] + margin, size2[2] + margin};

    // relative distance from surface (1%) outside of which box-box contacts are removed
    static mjtNum kRemoveRatio = 1.01;

    // is the contact outside: 1, inside: -1, within the removal width: 0
    int out1 = mju_outsideBox(tmp[i].pos, pos1, mat1, sz1, kRemoveRatio);
    int out2 = mju_outsideBox(tmp[i].pos, pos2, mat2, sz2, kRemoveRatio);

    // mark as bad if outside one box and not inside the other box
    if ((out1 == 1 && out2 != -1) || (out2 == 1 && out1 != -1)) {
      dupe[i] = -1;
      nbad++;
    }
  }

  // deep penetration can strand the midpoint-convention position outside both boxes; if
  // that removed every contact, restore the penetrating ones: an empty manifold for
  // overlapping boxes lets them pass through each other
  if (nbad && nbad == num) {
    for (int i=0; i < num; i++) {
      if (tmp[i].dist < 0) {
        dupe[i] = 0;
      }
    }
  }

  // find duplicates
  for (int i=0; i < num-1; i++) {
    if (dupe[i] == -1) {
      continue;  // already marked bad: skip
    }
    for (int j=i+1; j < num; j++) {
      if (dupe[j] == -1) {
        continue;  // already marked bad: skip
      }
      if (tmp[i].pos[0] == tmp[j].pos[0] &&
          tmp[i].pos[1] == tmp[j].pos[1] &&
          tmp[i].pos[2] == tmp[j].pos[2]) {
        dupe[i] = -1;
        break;
      }
    }
  }

  // consolidate good
  int ncon = 0;
  for (int j=0; j < num; j++) {
    if (dupe[j] == 0) {
      con[ncon++] = tmp[j];
      if (ncon >= 8) {
        break;
      }
    }
  }

  return ncon;
}
