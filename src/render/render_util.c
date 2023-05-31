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

#include "render/render_util.h"

#include <math.h>
#include <string.h>

#include <mujoco/mujoco.h>
#include "render/glad/glad.h"

//----------------------------- utility ------------------------------------------------------------

// compute normal vector to given triangle, return length
void mjr_makeNormal(float* normal, const float* p1, const float* p2, const float* p3) {
  float v1[3] = {p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]};
  float v2[3] = {p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]};

  mjr_crossVec(normal, v1, v2);
  mjr_normalizeVec(normal);
}



// set 4 floats
void mjr_setf4(float* vec, float f0, float f1, float f2, float f3) {
  vec[0] = f0;
  vec[1] = f1;
  vec[2] = f2;
  vec[3] = f3;
}



// set 3 floats
void mjr_setf3(float* vec, float f0, float f1, float f2) {
  vec[0] = f0;
  vec[1] = f1;
  vec[2] = f2;
}



// multiply 4-by-4 matrices, column-major
void mjr_mulMat44(float* res, const float* A, const float* B) {
  for (int r=0; r < 4; r++) {
    for (int c=0; c < 4; c++) {
      res[r+4*c] = 0;
      for (int i=0; i < 4; i++) {
        res[r+4*c] += A[r+4*i]*B[i+4*c];
      }
    }
  }
}



// get row from 4-by-4 matrix, column-major
void mjr_getrow4(float* res, const float* A, int r) {
  res[0] = A[r];
  res[1] = A[r+4];
  res[2] = A[r+8];
  res[3] = A[r+12];
}



// compute vector cross-product a = b x c
void mjr_crossVec(float* a, const float* b, const float* c) {
  a[0] = b[1]*c[2] - b[2]*c[1];
  a[1] = b[2]*c[0] - b[0]*c[2];
  a[2] = b[0]*c[1] - b[1]*c[0];
}



// normalize vector
void mjr_normalizeVec(float* v) {
  float scl, len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

  if (len < 1E-10f) {
    v[0] = 0;
    v[1] = 0;
    v[2] = 1;
  } else {
    scl = 1.0f/len;
    v[0] *= scl;
    v[1] *= scl;
    v[2] *= scl;
  }
}



// construct orthogonal vector (for up-direction)
void mjr_orthoVec(float* res, const float* v) {
  float other[3] = {-1, 0, 0};

  // try cross with negative X-axis
  mjr_crossVec(res, v, other);

  // success
  if (res[0]*res[0]+res[1]*res[1]+res[2]*res[2] > 0.01) {
    mjr_normalizeVec(res);
    return;
  }

  // otherwise use positive Y-axis
  other[0] = 0;
  other[1] = 1;
  mjr_crossVec(res, v, other);
  mjr_normalizeVec(res);
}



// compute dot-product
float mjr_dotVec(const float* a, const float* b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}



// multiply 4x4 matrix by vector; column-major (OpenGL format)
void mjr_multiply4(float* res, const float* mat, const float* vec) {
  for (int i=0; i < 4; i++) {
    res[i] = 0;
    for (int j=0; j < 4; j++) {
      res[i] += mat[i+4*j]*vec[j];
    }
  }
}



// gluLookAt replacement, with forward instead of center
void mjr_lookAt(const float* eye, const float* forward, const float* up) {
  float f[3] = {forward[0], forward[1], forward[2]};
  float s[3], u[3], mat[16];

  // prepare vectors
  mjr_normalizeVec(f);
  mjr_crossVec(s, f, up);
  mjr_normalizeVec(s);
  mjr_crossVec(u, s, f);
  mjr_normalizeVec(u);

  // fill in 4x4 matrix (column-major OpenGL format)
  mat[0] = s[0];
  mat[1] = u[0];
  mat[2] = -f[0];
  mat[3] = 0;

  mat[4] = s[1];
  mat[5] = u[1];
  mat[6] = -f[1];
  mat[7] = 0;

  mat[8] = s[2];
  mat[9] = u[2];
  mat[10]= -f[2];
  mat[11]= 0;

  mat[12]= -mjr_dotVec(s, eye);
  mat[13]= -mjr_dotVec(u, eye);
  mat[14]= mjr_dotVec(f, eye);
  mat[15]= 1;

  // set matrix in OpenGL
  glMultMatrixf(mat);
}



// gluPerspective replacement
void mjr_perspective(float fovy, float aspect, float znear, float zfar) {
  double h, w;

  // compute width and height
  h = tan((double)fovy / 360.0 * mjPI) * (double)znear;
  w = h * (double)aspect;

  // make symmetric frustrum
  glFrustum(-w, w, -h, h, (double)znear, (double)zfar);
}



// set reflection matrix
void mjr_reflect(const float* pos, const float* mat) {
  float reflect[16], v[3], vvt[9];

  // copy axis
  v[0] = mat[2];
  v[1] = mat[5];
  v[2] = mat[8];

  // compute outer product v*vT
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      vvt[3*i+j] = v[i]*v[j];
    }
  }

  // construct reflection matrix
  reflect[0]  = 1-2*vvt[0];
  reflect[1]  =  -2*vvt[1];
  reflect[2]  =  -2*vvt[2];
  reflect[3]  = 0;
  reflect[4]  =  -2*vvt[3];
  reflect[5]  = 1-2*vvt[4];
  reflect[6]  =  -2*vvt[5];
  reflect[7]  = 0;
  reflect[8]  =  -2*vvt[6];
  reflect[9]  =  -2*vvt[7];
  reflect[10] = 1-2*vvt[8];
  reflect[11] = 0;
  reflect[12] = 2*mjr_dotVec(vvt, pos);
  reflect[13] = 2*mjr_dotVec(vvt+3, pos);
  reflect[14] = 2*mjr_dotVec(vvt+6, pos);
  reflect[15] = 1;

  // multiply current OpenGL matrix
  glMultMatrixf(reflect);
}



// set transformation matrix
void mjr_transform(const float* translate, const float* rotate, float scale) {
  mjtNum quat[4], mat[9];
  float glmat[16];

  // construct matrix rotation
  mju_f2n(quat, rotate, 4);
  mju_quat2Mat(mat, quat);

  // prepare transformation matrix
  glmat[0] = scale * (float)mat[0];
  glmat[1] = scale * (float)mat[3];
  glmat[2] = scale * (float)mat[6];
  glmat[3] = 0;
  glmat[4] = scale * (float)mat[1];
  glmat[5] = scale * (float)mat[4];
  glmat[6] = scale * (float)mat[7];
  glmat[7] = 0;
  glmat[8] = scale * (float)mat[2];
  glmat[9] = scale * (float)mat[5];
  glmat[10]= scale * (float)mat[8];
  glmat[11]= 0;
  glmat[12]= translate[0];
  glmat[13]= translate[1];
  glmat[14]= translate[2];
  glmat[15] = 1;

  // multiply current OpenGL matrix
  glMultMatrixf(glmat);
}



// Find first rectangle containing mouse, -1: not found.
int mjr_findRect(int x, int y, int nrect, const mjrRect* rect) {
  // scan
  for (int i=0; i < nrect; i++) {
    if (x >= rect[i].left &&
        x < rect[i].left+rect[i].width &&
        y >= rect[i].bottom &&
        y < rect[i].bottom+rect[i].height) {
      return i;
    }
  }

  // not found
  return -1;
}
