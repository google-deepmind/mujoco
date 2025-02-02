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

#include <string.h>

#include <mujoco/mjmacro.h>
#include "engine/engine_collision_primitive.h"
#include "engine/engine_util_blas.h"

// hard-clamp vector to range [-limit(i), +limit(i)]
static void mju_clampVec(mjtNum* vec, const mjtNum* limit, int n)
{
  int i;

  // loop over active limits
  for (i = 0; i < n; i++) {
    if (limit[i] > 0) {
      if (vec[i] < -limit[i])
        vec[i] = -limit[i];
      else if (vec[i] > limit[i])
        vec[i] = limit[i];
    }
  }
}


// raw sphere : box
int mjraw_SphereBox(mjContact* con, mjtNum margin,
                    const mjtNum* pos1, const mjtNum* mat1, const mjtNum* size1,
                    const mjtNum* pos2, const mjtNum* mat2, const mjtNum* size2) {
  int i, k;
  mjtNum tmp[3], center[3], clamped[3], deepest[3];
  mjtNum pos[3];
  mjtNum dist, closest;

  mju_sub3(tmp, pos1, pos2);
  mju_mulMatTVec3(center, mat2, tmp);

  mju_copy(clamped, center, 3);
  mju_clampVec(clamped, size2, 3);

  mju_copy(deepest, center, 3);
  mju_sub3(tmp, clamped, center);
  dist = mju_normalize3(tmp);

  if (dist - size1[0] > margin)
    return 0;

  // sphere center inside box
  if (dist <= mjMINVAL) {
    closest = (size2[0] + size2[1] + size2[2]) * 2;

    for (i = 0; i < 6; i++) {
      if (closest > mju_abs((i % 2 ? 1 : -1)*size2[i / 2] - center[i / 2])) {
        closest = mju_abs((i % 2 ? 1 : -1) * size2[i / 2] - center[i / 2]);
        k = i;
      }
    }

    mjtNum nearest[3] = {0};
    nearest[k / 2] = (k % 2 ? -1 : 1);

    mju_copy3(pos, center);
    mju_addToScl3(pos, nearest, (size1[0] - closest) / 2);
    mju_mulMatVec3(con[0].frame, mat2, nearest);
    dist = -closest;
  } else {
    mju_addToScl3(deepest, tmp, size1[0]);
    mju_zero3(pos);
    mju_addToScl3(pos, clamped, 0.5);
    mju_addToScl3(pos, deepest, 0.5);
    mju_mulMatVec3(con[0].frame, mat2, tmp);
  }

  mju_mulMatVec3(tmp, mat2, pos);
  mju_add3(con[0].pos, tmp, pos2);
  con[0].dist = dist - size1[0];
  mju_zero3(con[0].frame + 3);

  return 1;
}



// sphere : box
int mjc_SphereBox(const mjModel* m, const mjData* d, mjContact* con,
                  int g1, int g2, mjtNum margin)
{
  mjGETINFO;

  return mjraw_SphereBox(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}



/* GENERAL THEORY OF OPERATION
   the following code is mostly for finding (line segment)/(box) collision
   after which box-sphere is called

   First the closest point to the box is found.
   Then a "sensible" second point is found if the angle
   between the segment and the box is low enough < 45

   In the comments that follow, capsule just means the capsule's line segment
   It might be hard to understand all comments but you would need
   a picture to see what is happening at each line of the code
*/

// raw capsule : box
int mjraw_CapsuleBox(mjContact* con, mjtNum margin,
                     const mjtNum* pos1, const mjtNum* mat1, const mjtNum* size1,
                     const mjtNum* pos2, const mjtNum* mat2, const mjtNum* size2) {
  mjtNum tmp1[3], tmp2[3], tmp3[3], halfaxis[3], axis[3], dif[3];
  mjtNum pos[3];          // position of capsule in box-local frame

  mjtNum halflength;      // half of capsule's length
  mjtNum bestdist;        // closest contact point distance
  mjtNum bestdistmax;     // init value for bestdist
  mjtNum bestsegmentpos;  // between -1 and 1 :  which point on the segment is closest to the box
  mjtNum secondpos;       // distance of 2nd contact position on capsule segment from the first
  mjtNum dist;
  mjtNum bestboxpos;      // closest contact point, position on the box's edge
  mjtNum mul, e1, e2, dp, de;
  // mjtNum penetration;

  mjtNum ma, mb, mc, u, v, det, x1, x2, idet;  // linelinedist temps

  int s1, s2;        // hold linelinedist info
  int i, j, c1, c2;  // temporary variables
  int cltype = -4;   // closest type
  int clface;        // closest face
  int clcorner = 0;  // closest corner (0..7 in binary)
  int cledge;        // closest edge axis
  int axisdir;       // direction of capsule axis in relation to the box
  int n;             // number of contacts
  int ax1, ax2, ax;  // axis temporaries


  halflength = size1[1];
  secondpos = -4;  // initialize to no 2nd contact (valid values are between -1 and 1)

  mju_sub3(tmp1, pos1, pos2);       // bring capsule to box-local frame (center's box is at (0,0,0))
  mju_mulMatTVec3(pos, mat2, tmp1);  // and axis parralel to world

  tmp1[0] = mat1[2];  // capsule's axis
  tmp1[1] = mat1[5];
  tmp1[2] = mat1[8];

  mju_mulMatTVec3(axis, mat2, tmp1);      // do the same for the capsule axis
  mju_scl3(halfaxis, axis, halflength);  // scale to get actual capsule half-axis

  axisdir = 0;
  if (halfaxis[0] > 0)
    axisdir += 1;
  if (halfaxis[1] > 0)
    axisdir += 2;
  if (halfaxis[2] > 0)
    axisdir += 4;

  // under this notion    "axisdir" and "7-axisdir" point in opposite directions,
  // essentially the same for a capsule

  bestdistmax = margin + 2 * (size1[0] + halflength + size2[0] + size2[1] +
                              size2[2]);  // initialize bestdist
  bestdist = bestdistmax;
  bestsegmentpos = 0;

  mju_zero3(tmp2);

  // test to see if maybe the a face of the box is closest to the capsule
  for (i = -1; i <= 1; i += 2) {
    mju_copy3(tmp1, pos);
    mju_addToScl3(tmp1, halfaxis, i);
    mju_copy3(tmp2, tmp1);

    for (c1 = 0, j = 0, c2 = -1; j < 3; j++) {
      if (tmp1[j] < -size2[j]) {
        c1++;
        c2 = j;
        tmp1[j] = -size2[j];
      } else if (tmp1[j] > size2[j]) {
        c1++;
        c2 = j;
        tmp1[j] = size2[j];
      }
    }

    if (c1 > 1)
      continue;

    mju_subFrom3(tmp1, tmp2);
    dist = mju_dot3(tmp1, tmp1);

    if (dist < bestdist) {
      bestdist = dist;
      bestsegmentpos = i;
      cltype = -2 + i;
      clface = c2;
    }
  }

  mju_zero3(tmp2);

  for (j = 0; j < 3; j++) {
    for (i = 0; i < 8; i++) {
      if ((i & (1 << j)) == 0) {
        // trick to get a corner
        tmp3[0] = ((i & 1) ? 1 : -1) * size2[0];
        tmp3[1] = ((i & 2) ? 1 : -1) * size2[1];
        tmp3[2] = ((i & 4) ? 1 : -1) * size2[2];
        tmp3[j] = 0;

        // tmp3 is the starting point on the box
        // tmp2 is the direction along the "j"-th axis
        // pos is the capsule's center
        // halfaxis is the capsule direction

        // find closest point between capsule and the edge

        mju_sub3(dif, tmp3, pos);

        ma = size2[j] * size2[j];
        mb = -size2[j] * halfaxis[j];
        mc = size1[1] * size1[1];

        u = -size2[j] * dif[j];
        v = mju_dot3(halfaxis, dif);

        det = ma * mc - mb * mb;
        if (mju_abs(det) < mjMINVAL)
          continue;
        idet = 1 / det;


        // sX : X=1 means middle of segment. X=0 or 2 one or the other end

        x1 = (mc * u - mb * v) * idet;
        x2 = (ma * v - mb * u) * idet;

        s1 = s2 = 1;

        if (x1 > 1) {
          x1 = 1;
          s1 = 2;
          x2 = (v - mb) * (1 / mc);
        } else if (x1 < -1) {
          x1 = -1;
          s1 = 0;
          x2 = (v + mb) * (1 / mc);
        }

        if (x2 > 1) {
          x2 = 1;
          s2 = 2;
          x1 = (u - mb) * (1 / ma);
          if (x1 > 1)
            x1 = 1, s1 = 2;
          else if (x1 < -1)
            x1 = -1, s1 = 0;
        } else if (x2 < -1) {
          x2 = -1;
          s2 = 0;
          x1 = (u + mb) * (1 / ma);
          if (x1 > 1)
            x1 = 1, s1 = 2;
          else if (x1 < -1)
            x1 = -1, s1 = 0;
        }

        mju_sub3(dif, tmp3, pos);

        mju_addToScl3(dif, halfaxis, -x2);
        dif[j] += size2[j] * x1;

        tmp1[2] = mju_dot3(dif, dif);

        c1 = s1 * 3 + s2;


        // the -MINVAL might not be necessary. Fixes numerical problem when axis is numerically
        // parallel to the box
        if (tmp1[2] < bestdist - mjMINVAL) {
          bestdist = tmp1[2];
          bestsegmentpos = x2;
          bestboxpos = x1;

          // c1<6 means that closest point on the box is at the lower end
          // or in the middle of the edge
          c2 = c1 / 6;

          clcorner = i + (1 << j) * c2;  // which corner is the closest
          cledge = j;   // which axis
          cltype = c1;  // save clamped info
        }
      }
    }
  }


  // penetration = -bestdist;


  for (j = 0; j < 3; j++) {
    if (j == 2) {
      typedef union {
        struct {
          mjtNum x, y;
        };
        mjtNum c[2];
      } d2;
      d2 p, s, dd /*, c, tmp1*/;
      mjtNum uu, vv, w, ee1, best /* ,e2 */, l /* , e3, e4 */;

      bestdist = bestdistmax;

      p.x = pos[0];
      p.y = pos[1];
      dd.x = halfaxis[0];
      dd.y = halfaxis[1];
      s.x = size2[0];
      s.y = size2[1];

      l = sqrt(dd.x * dd.x + dd.y * dd.y);

      uu = dd.x * s.y;
      vv = dd.y * s.x;
      w = dd.x * p.y - dd.y * p.x;


      best = -1;

      ee1 = +uu - vv;
      if ((ee1 < 0) == (w < 0)) {
        if (best < mju_abs(ee1)) {
          best = mju_abs(ee1);
          c1 = 0;
        }
      }
      ee1 = -uu - vv;
      if ((ee1 < 0) == (w < 0)) {
        if (best < mju_abs(ee1)) {
          best = mju_abs(ee1);
          c1 = 1;
        }
      }
      ee1 = +uu + vv;
      if ((ee1 < 0) == (w < 0)) {
        if (best < mju_abs(ee1)) {
          best = mju_abs(ee1);
          c1 = 2;
        }
      }
      ee1 = -uu + vv;
      if ((ee1 < 0) == (w < 0)) {
        if (best < mju_abs(ee1)) {
          best = mju_abs(ee1);
          c1 = 3;
        }
      }

      // c.x = s.x * ((c1 / 2) ? -1 : 1);
      // c.y = s.y * ((c1 % 2) ? -1 : 1);

      ee1 = mju_abs(w) / l;
      // e2 = best / l;

      // printf("%g %g      %g %g     %g %g\n",c.x,c.y,d.x,d.y,e1,e2);

      // tmp1.x = c.x - p.x;
      // tmp1.y = c.y - p.y;
      ee1 = dd.x * dd.x + dd.y * dd.y;
      // e2 = tmp1.x * d.x + tmp1.y * d.y;
      // e3 = e2 / e1;


      // printf("%g %g      %g %g     %g %g %g \n",c.x,c.y,d.x,d.y,e1,e2,e3);

      ee1 = p.x + (+s.y - p.y) / dd.y * dd.x;
      // e2 = p.x + (-s.y - p.y) / d.y * d.x;
      // e3 = p.y + (+s.x - p.x) / d.x * d.y;
      // e4 = p.y + (-s.x - p.x) / d.x * d.y;


      // printf("%g %g     %g %g\n",e1,e2,e3,e4);
    }
  }

  // goto skip;   // allow only the closest contact

  // cltype: -3 -1 : face is closest to the capsule
  // cltype: 0..8 : edge is closest to the capsule
  // cltype/3==0 means the lower corner is closest to the capsule (note that edges include corners)
  // cltype/3==2 means the upper corner is closest to the capsule (note that edges include corners)
  // cltype/3==1 means the middle of the edge is closest to the capsule
  // cltype%3==0 means the lower corner is closest to the box (note that edges include corners)
  // cltype%3==2 means the upper corner is closest to the box (note that edges include corners)
  // cltype%3==1 means the middle of the capsule is closest to the box


  // invalid type
  if (cltype == -4)
    return 0;

  if (cltype >= 0 && cltype / 3 != 1) {  // closest to a corner of the box
    c1 = axisdir ^ clcorner;

    // hack to find the relative orientation of capsule and corner
    // there are 2 cases:
    //    1: pointing to or away from the corner
    //    2: oriented along a face or an edge


    if (c1 == 0 || c1 == 7)
      goto skip;  // case 1: no chance of additional contact

    if (c1 == 1 || c1 == 2 || c1 == 4) {
      mul = 1;
      de = 1 - bestsegmentpos;
      dp = 1 + bestsegmentpos;
    }

    if (c1 == 3 || c1 == 5 || c1 == 6) {
      mul = -1;
      c1 = 7 - c1;
      dp = 1 - bestsegmentpos;
      de = 1 + bestsegmentpos;
    }

    // "de" and "dp" distance from first closest point on the capsule to both ends of it
    // mul is a direction along the capsule's axis

    if (c1 == 1)
      ax = 0, ax1 = 1, ax2 = 2;
    if (c1 == 2)
      ax = 1, ax1 = 2, ax2 = 0;
    if (c1 == 4)
      ax = 2, ax1 = 0, ax2 = 1;



    if (axis[ax]*axis[ax] > 0.5) {  // second point along the edge of the box
      secondpos = de;  // initial position from the
      e1 = 2 * size2[ax] / mju_abs(halfaxis[ax]);

      if (e1 < secondpos) {
        secondpos = e1;  // we overshoot, move back to the  other corner of the edge
      }
      secondpos *= mul;
    } else {  // second point along a face of the box
      secondpos = dp;

      // check for overshoot again

      e1 = 2 * size2[ax1] / mju_abs(halfaxis[ax1]);
      if (e1 < secondpos)
        secondpos = e1;

      e1 = 2 * size2[ax2] / mju_abs(halfaxis[ax2]);
      if (e1 < secondpos)
        secondpos = e1;

      secondpos *= -mul;
    }
  } else if (cltype >= 0 && cltype / 3 == 1) {  // we are on box's edge
    // hacks to find the relative orientation of capsule and edge
    // there are 2 cases:
    //    c1= 2^n: edge and capsule are oriented in a T configuration (no more contacts
    //    c1!=2^n: oriented in a cross X configuration

    c1 = axisdir ^ clcorner;  // same trick

    c1 &= 7 - (1 << cledge);  // even more hacks

    // printf("%d %d %d %d    %lf %lf %lf\n",
    //        axisdir,clcorner,c1,cledge,halfaxis[0],halfaxis[1],halfaxis[2]);

    if (c1 != 1 && c1 != 2 && c1 != 4)
      goto skip;


    if (cledge == 0)
      ax1 = 1, ax2 = 2;
    if (cledge == 1)
      ax1 = 2, ax2 = 0;
    if (cledge == 2)
      ax1 = 0, ax2 = 1;
    ax = cledge;


    // Then it finds with which face the capsule has a lower angle and switches the axis names

    if (mju_abs(axis[ax1]) > mju_abs(axis[ax2]))
      ax1 = ax2;
    ax2 = 3 - ax - ax1;

    // keep track of the axis orientation (mul will tell us which direction along the capsule to
    // find the second point) you can notice all other references to the axis "halfaxis" are with
    // absolute value

    if (c1 & (1 << ax2)) {
      mul = 1;
      secondpos = 1 - bestsegmentpos;
    } else {
      mul = -1;
      secondpos = 1 + bestsegmentpos;
    }


    // now we have to find out whether we point towards the opposite side or towards one of the
    // sides and also find the farthest point along the capsule that is above the box

    e1 = 2 * size2[ax2] / mju_abs(halfaxis[ax2]);
    if (e1 < secondpos)
      secondpos = e1;

    if (((axisdir & (1 << ax)) != 0) == ((c1 & (1 << ax2)) != 0))  // that is insane
      e2 = 1 - bestboxpos;
    else
      e2 = 1 + bestboxpos;

    e1 = size2[ax] * e2 / mju_abs(halfaxis[ax]);

    if (e1 < secondpos)
      secondpos = e1;

    secondpos *= mul;
  } else if (cltype < 0) {
    // similarly we handle the case when one capsule's end is closest to a face of the box
    // and find where is the other end pointing to and clamping to the farthest point
    // of the capsule that's above the box

    if (clface == -1)
      goto skip;  // here the closest point is inside the box, no need for a second point
    if (cltype == -3)
      mul = 1;
    else
      mul = -1;

    secondpos = 2;

    mju_copy3(tmp1, pos);
    mju_addToScl3(tmp1, halfaxis, -mul);

    for (i = 0; i < 3; i++) {
      if (i != clface) {
        e1 = (size2[i] - tmp1[i]) / halfaxis[i] * mul;
        if (e1 > 0)
          if (e1 < secondpos)
            secondpos = e1;

        e1 = (-size2[i] - tmp1[i]) / halfaxis[i] * mul;
        if (e1 > 0)
          if (e1 < secondpos)
            secondpos = e1;
      }
    }
    secondpos *= mul;
  }


skip:

  // create sphere in original orientation at first contact point
  mju_copy3(tmp1, pos);
  mju_addToScl3(tmp1, halfaxis, bestsegmentpos);
  mju_mulMatVec3(tmp2, mat2, tmp1);
  mju_addTo3(tmp2, pos2);

  // collide with
  n = mjraw_SphereBox(con, margin, tmp2, mat1, size1, pos2, mat2, size2);


  if (secondpos > -3) {  // secondpos was modified
    mju_copy3(tmp1, pos);
    mju_addToScl3(tmp1, halfaxis, secondpos + bestsegmentpos);  // note the summation
    mju_mulMatVec3(tmp2, mat2, tmp1);
    mju_addTo3(tmp2, pos2);
    n += mjraw_SphereBox(con + n, margin, tmp2, mat1, size1, pos2, mat2, size2);
  }

  return n;
}


// capsule : box
int mjc_CapsuleBox(const mjModel* m, const mjData* d, mjContact* con,
                   int g1, int g2, mjtNum margin)
{
  mjGETINFO
  return mjraw_CapsuleBox(con, margin, pos1, mat1, size1, pos2, mat2, size2);
}



// box : box
int mjc_BoxBox(const mjModel* M, const mjData* D, mjContact* con, int g1, int g2, mjtNum margin)
{
  const mjtNum* pos1 = D->geom_xpos + 3 * g1;
  const mjtNum* mat1 = D->geom_xmat + 9 * g1;
  const mjtNum* size1 = M->geom_size + 3 * g1;
  const mjtNum* pos2 = D->geom_xpos + 3 * g2;
  const mjtNum* mat2 = D->geom_xmat + 9 * g2;
  const mjtNum* size2 = M->geom_size + 3 * g2;

  mjtNum pos12[3], pos21[3], rot[9], rott[9], rotabs[9], rottabs[9], tmp1[3], tmp2[3], plen1[3],
         plen2[3];
  mjtNum rotmore[9], p[3], r[9], s[3], ss[3], lp[3], rt[9], points[mjMAXCONPAIR][3],
         depth[mjMAXCONPAIR], pts[6][3], ppts2[4][2], pu[4][3], axi[3][3];
  mjtNum linesu[4][6], lines[4][6], clnorm[3], rnorm[3];
  mjtNum penetration, c1, c2, c3, a, b, c, d, lx, ly, hz, l, x, y, u, v, llx, lly, innorm, margin2;

  int i0, i1, i2;
  mjtNum f0, f1, f2;

  int i, j, q, code, q1, q2, clcorner, n, m, k;
  int cle1, cle2, in, ax1, ax2, pax1, pax2, clface, nl, nf;

  n = 0;
  code = -1;
  margin2 = margin * margin;

  mju_sub3(tmp1, pos2, pos1);
  mju_mulMatTVec3(pos21, mat1, tmp1);

  mju_sub3(tmp1, pos1, pos2);
  mju_mulMatTVec3(pos12, mat2, tmp1);

  mju_mulMatTMat3(rot, mat1, mat2);
  mju_transpose(rott, rot, 3, 3);

  for (i = 0; i < 9; i++)
    rotabs[i] = mju_abs(rot[i]);
  for (i = 0; i < 9; i++)
    rottabs[i] = mju_abs(rott[i]);

  mju_mulMatVec3(plen2, rotabs, size2);
  mju_mulMatTVec3(plen1, rotabs, size1);

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
        mju_copy3(clnorm, tmp2);
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
  mju_scl3(matres+0, matin+i0*3, f0); \
  mju_scl3(matres+3, matin+i1*3, f1); \
  mju_scl3(matres+6, matin+i2*3, f2); \
}

  if (q2) {
    mju_mulMatMatT3(r, rotmore, rot);

    // mju_mulMatVec3(p,rotmore,pos12);
    // mju_mulMatVec3(tmp1,rotmore,size2);

    rotaxis(p, pos12);
    rotaxis(tmp1, size2);

    mju_copy3(s, size1);
  } else {
    // mju_mulMatMat(r,rotmore,rot,3,3,3);

    rotmatx(r, rot);

    // mju_mulMatVec3(p,rotmore,pos21);
    // mju_mulMatVec3(tmp1,rotmore,size1);

    rotaxis(p, pos21);
    rotaxis(tmp1, size1);

    mju_copy3(s, size2);
  }

  mju_transpose(rt, r, 3, 3);

  for (i = 0; i < 3; i++)
    ss[i] = mju_abs(tmp1[i]);

  lx = ss[0];
  ly = ss[1];
  hz = ss[2];
  p[2] -= hz;

  mju_copy3(lp, p);

  for (clcorner = 0, i = 0; i < 3; i++)
    if (r[6 + i] < 0)
      clcorner += 1 << i;

  mju_addToScl3(lp, rt + 0, s[0] * ((clcorner & 1) ? 1 : -1));
  mju_addToScl3(lp, rt + 3, s[1] * ((clcorner & 2) ? 1 : -1));
  mju_addToScl3(lp, rt + 6, s[2] * ((clcorner & 4) ? 1 : -1));

  m = k = 0;
  mju_copy3(pts[m++], lp);

  for (i = 0; i < 3; i++)
    if (mju_abs(r[6 + i]) < 0.5)
      mju_scl3(pts[m++], rt + 3 * i, s[i] * ((clcorner & (1 << i)) ? -2 : 2));

  mju_add3(pts[3], pts[0], pts[1]);
  mju_add3(pts[4], pts[0], pts[2]);
  mju_add3(pts[5], pts[3], pts[2]);

  if (m > 1)
  {
    mju_copy3(lines[k] + 0, pts[0]);
    mju_copy3(lines[k++] + 3, pts[1]);
  }
  if (m > 2)
  {
    mju_copy3(lines[k] + 0, pts[0]);
    mju_copy3(lines[k++] + 3, pts[2]);
    mju_copy3(lines[k] + 0, pts[3]);
    mju_copy3(lines[k++] + 3, pts[2]);
    mju_copy3(lines[k] + 0, pts[4]);
    mju_copy3(lines[k++] + 3, pts[1]);
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

          mju_copy3(points[n], lines[i]);
          mju_addToScl3(points[n++], lines[i] + 3, c1);
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

      points[n][0] = llx;
      points[n][1] = lly;
      points[n][2] = (pts[0][2] + u * pts[1][2] + v * pts[2][2]);
      n++;
    }
  }

  for (i = 0; i < (1 << (m - 1)); i++) {
    mju_copy3(tmp1, pts[i == 0 ? 0 : i + 2]);


    if (i)
      if (tmp1[0] <= -lx || tmp1[0] >= lx)
        continue;
    if (i)
      if (tmp1[1] <= -ly || tmp1[1] >= ly)
        continue;

    mju_copy3(points[n++], tmp1);
  }


  m = n;
  n = 0;

  for (i = 0; i < m; i++)
  {
    if (points[i][2] > margin)
      continue;
    mju_copy3(points[n], points[i]);

    depth[n] = points[n][2];
    points[n][2] *= 0.5;

    n++;
  }


  mju_mulMatMatT3(r, q2 ? mat2 : mat1, rotmore);
  mju_copy3(p, q2 ? pos2 : pos1);

  tmp2[0] = (q2 ? -1 : 1) * r[2];
  tmp2[1] = (q2 ? -1 : 1) * r[5];
  tmp2[2] = (q2 ? -1 : 1) * r[8];

  mju_copy3(con[0].frame, tmp2);
  mju_zero3(con[0].frame + 3);




  for (i = 0; i < n; i++)
  {
    con[i].dist = points[i][2];
    points[i][2] += hz;

    mju_mulMatVec3(tmp2, r, points[i]);
    mju_add3(con[i].pos, tmp2, p);

    if (i)
      mju_copy(con[i].frame, con[0].frame, 6);
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

  mju_mulMatTVec3(tmp1, rotmore, size1);
  for (i = 0; i < 3; i++)
    s[i] = mju_abs(tmp1[i]);

  mju_transpose(rt, r, 3, 3);


  lx = s[0];
  ly = s[1];
  hz = s[2];
  p[2] -= hz;


  n = 0;
  mju_copy3(points[n], p);
  mju_addToScl3(points[n], rt + 3 * ax1, size2[ax1] * ((cle2 & (1 << ax1)) ? 1 : -1));
  mju_addToScl3(points[n], rt + 3 * ax2, size2[ax2] * ((cle2 & (1 << ax2)) ? 1 : -1));
  mju_copy3(points[n + 1], points[n]);
  mju_addToScl3(points[n], rt + 3 * q2, size2[q2]);
  n = 1;
  mju_addToScl3(points[n], rt + 3 * q2, -size2[q2]);
  n = 2;


  mju_copy3(points[n], p);
  mju_addToScl3(points[n], rt + 3 * ax1, size2[ax1] * ((cle2 & (1 << ax1)) ? -1 : 1));
  mju_addToScl3(points[n], rt + 3 * ax2, size2[ax2] * ((cle2 & (1 << ax2)) ? 1 : -1));
  mju_copy3(points[n + 1], points[n]);
  mju_addToScl3(points[n], rt + 3 * q2, size2[q2]);
  n = 3;
  mju_addToScl3(points[n], rt + 3 * q2, -size2[q2]);
  n = 4;


  mju_copy3(axi[0], points[0]);
  mju_sub3(axi[1], points[1], points[0]);
  mju_sub3(axi[2], points[2], points[0]);


  if (mju_abs(rnorm[2]) < mjMINVAL)
    return 0;  // shouldn't happen

  innorm = (1 / rnorm[2]) * (in ? -1 : 1);
  // printf("%lf\n",innorm);

  for (i = 0; i < 4; i++)
  {
    c1 = -points[i][2] * (1 / rnorm[2]);

    mju_copy3(pu[i], points[i]);

    mju_addToScl3(points[i], rnorm, c1);

    // ppts[i][0]=points[i][0];
    // ppts[i][1]=points[i][1];
    ppts2[i][0] = points[i][0];
    ppts2[i][1] = points[i][1];
  }


  mju_copy3(pts[0], points[0]);
  mju_sub3(pts[1], points[1], points[0]);
  mju_sub3(pts[2], points[2], points[0]);

  m = 3;
  k = 0;
  n = 0;


  if (m > 1) {
    mju_copy3(lines[k] + 0, pts[0]);
    mju_copy3(lines[k] + 3, pts[1]);
    mju_copy3(linesu[k] + 0, axi[0]);
    mju_copy3(linesu[k++] + 3, axi[1]);
  }
  if (m > 2) {
    mju_copy3(lines[k] + 0, pts[0]);
    mju_copy3(lines[k] + 3, pts[2]);
    mju_copy3(linesu[k] + 0, axi[0]);
    mju_copy3(linesu[k++] + 3, axi[2]);

    mju_add3(lines[k] + 0, pts[0], pts[1]);
    mju_copy3(lines[k] + 3, pts[2]);
    mju_add3(linesu[k] + 0, axi[0], axi[1]);
    mju_copy3(linesu[k++] + 3, axi[2]);

    mju_add3(lines[k] + 0, pts[0], pts[2]);
    mju_copy3(lines[k] + 3, pts[1]);
    mju_add3(linesu[k] + 0, axi[0], axi[2]);
    mju_copy3(linesu[k++] + 3, axi[1]);
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

            mju_scl3(points[n], linesu[i], 0.5);
            mju_addToScl3(points[n], linesu[i] + 3, 0.5 * c1);
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


      mju_scl3(tmp1, pu[0], 1 - u - v);
      mju_addToScl3(tmp1, pu[1], u);
      mju_addToScl3(tmp1, pu[2], v);

      points[n][0] = llx;
      points[n][1] = lly;
      points[n][2] = 0;

      mju_sub3(tmp2, points[n], tmp1);

      c1 = mju_dot3(tmp2, tmp2);
      if (tmp1[2] > 0)
        if (c1 > margin2)
          continue;

      mju_add3(points[n], points[n], tmp1);
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
      mju_addToScl3(tmp1, pu[i], 0.5);
      mju_copy3(points[n], tmp1);

      depth[n] = sqrt(c1) * (pu[i][2] < 0 ? -1 : 1);
      n++;
    }
  }

  mju_mulMatMatT3(r, mat1, rotmore);

  mju_mulMatVec3(tmp1, r, rnorm);

  mju_scl3(con[0].frame, tmp1, in ? -1 : 1);
  mju_zero3(con[0].frame + 3);


  for (i = 0; i < n; i++) {
    con[i].dist = depth[i];
    points[i][2] += hz;

    mju_mulMatVec3(tmp2, r, points[i]);

    mju_add3(con[i].pos, tmp2, pos1);

    mju_copy(con[i].frame, con[0].frame, 6);
  }

  return n;

#undef rotaxis
#undef rotmatx
}
