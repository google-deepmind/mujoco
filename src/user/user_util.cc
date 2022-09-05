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

#include "user/user_util.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <string>

#include <mujoco/mjtnum.h>
#include "engine/engine_macro.h"

using std::isnan;
using std::string;
using std::numeric_limits;


// set value of NAN here; needs <limits>
const double mjNAN = numeric_limits<double>::quiet_NaN();


// check if numeric variable is defined
bool mjuu_defined(const double num) {
  return !isnan(num);
}


// compute address of M[g1][g2] where M is triangular n-by-n
int mjuu_matadr(int g1, int g2, const int n) {
  if (g1<0 || g2<0 || g1>=n || g2>=n) {
    return -1;
  }

  if (g1>g2) {
    int tmp = g1;
    g1 = g2;
    g2 = tmp;
  }

  return g1*n + g2;
}


// set 4D vector
void mjuu_setvec(double* dest, const double x, const double y, const double z, const double w) {
  dest[0] = x;
  dest[1] = y;
  dest[2] = z;
  dest[3] = w;
}
void mjuu_setvec(float* dest, const double x, const double y, const double z, const double w) {
  dest[0] = (float)x;
  dest[1] = (float)y;
  dest[2] = (float)z;
  dest[3] = (float)w;
}


// set 3D vector
void mjuu_setvec(double* dest, const double x, const double y, const double z) {
  dest[0] = x;
  dest[1] = y;
  dest[2] = z;
}
void mjuu_setvec(float* dest, const double x, const double y, const double z) {
  dest[0] = (float)x;
  dest[1] = (float)y;
  dest[2] = (float)z;
}


// set 2D vector
void mjuu_setvec(double* dest, const double x, const double y) {
  dest[0] = x;
  dest[1] = y;
}


// copy double array
void mjuu_copyvec(double* dest, const double* src, const int n) {
  for (int i=0; i<n; i++) {
    dest[i] = src[i];
  }
}


// copy float array
void mjuu_copyvec(float* dest, const float* src, const int n) {
  for (int i=0; i<n; i++) {
    dest[i] = src[i];
  }
}


// zero double array
void mjuu_zerovec(double* dest, const int n) {
  for (int i=0; i<n; i++) {
    dest[i] = 0;
  }
}


// dot-product in 3D
double mjuu_dot3(const double* a, const double* b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}


// distance beween 3D points
double mjuu_dist3(const double* a, const double* b) {
  return sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
}


// L1 norm between vectors
double mjuu_L1(const double* a, const double* b, int n) {
  double res = 0;
  for (int i=0; i<n; i++) {
    res += fabs(a[i]-b[i]);
  }

  return res;
}


// normalize vector to unit length, return previous length
double mjuu_normvec(double* vec, const int n) {
  double nrm = 0;
  int i;

  for (i=0; i<n; i++) {
    nrm += vec[i]*vec[i];
  }
  if (nrm < mjEPS) {
    return 0;
  }

  nrm = sqrt(nrm);
  for (i=0; i<n; i++) {
    vec[i] /= nrm;
  }

  return nrm;
}


// convert quaternion to rotation matrix
void mjuu_quat2mat(double* res, const double* quat) {
  double q00 = quat[0]*quat[0];
  double q11 = quat[1]*quat[1];
  double q22 = quat[2]*quat[2];
  double q33 = quat[3]*quat[3];
  res[0] = q00 + q11 - q22 - q33;
  res[4] = q00 - q11 + q22 - q33;
  res[8] = q00 - q11 - q22 + q33;
  res[1] = 2*(quat[1]*quat[2] - quat[0]*quat[3]);
  res[2] = 2*(quat[1]*quat[3] + quat[0]*quat[2]);
  res[3] = 2*(quat[1]*quat[2] + quat[0]*quat[3]);
  res[5] = 2*(quat[2]*quat[3] - quat[0]*quat[1]);
  res[6] = 2*(quat[1]*quat[3] - quat[0]*quat[2]);
  res[7] = 2*(quat[2]*quat[3] + quat[0]*quat[1]);
}


// multiply two unit quaternions
void mjuu_mulquat(double* res, const double* qa, const double* qb) {
  res[0] = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
  res[1] = qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2];
  res[2] = qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1];
  res[3] = qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0];
  mjuu_normvec(res, 4);
}


// multiply vector by 3-by-3 matrix
void mjuu_mulvecmat(double* res, const double* vec, const double* mat) {
  res[0] = mat[0]*vec[0] + mat[1]*vec[1] + mat[2]*vec[2];
  res[1] = mat[3]*vec[0] + mat[4]*vec[1] + mat[5]*vec[2];
  res[2] = mat[6]*vec[0] + mat[7]*vec[1] + mat[8]*vec[2];
}


// compute res = R * M * R'
void mjuu_mulRMRT(double* res, const double* R, const double* M) {
  double tmp[9];

  // tmp = R*M
  tmp[0] = R[0]*M[0] + R[1]*M[3] + R[2]*M[6];
  tmp[1] = R[0]*M[1] + R[1]*M[4] + R[2]*M[7];
  tmp[2] = R[0]*M[2] + R[1]*M[5] + R[2]*M[8];
  tmp[3] = R[3]*M[0] + R[4]*M[3] + R[5]*M[6];
  tmp[4] = R[3]*M[1] + R[4]*M[4] + R[5]*M[7];
  tmp[5] = R[3]*M[2] + R[4]*M[5] + R[5]*M[8];
  tmp[6] = R[6]*M[0] + R[7]*M[3] + R[8]*M[6];
  tmp[7] = R[6]*M[1] + R[7]*M[4] + R[8]*M[7];
  tmp[8] = R[6]*M[2] + R[7]*M[5] + R[8]*M[8];

  // res = tmp*R'
  res[0] = tmp[0]*R[0] + tmp[1]*R[1] + tmp[2]*R[2];
  res[1] = tmp[0]*R[3] + tmp[1]*R[4] + tmp[2]*R[5];
  res[2] = tmp[0]*R[6] + tmp[1]*R[7] + tmp[2]*R[8];
  res[3] = tmp[3]*R[0] + tmp[4]*R[1] + tmp[5]*R[2];
  res[4] = tmp[3]*R[3] + tmp[4]*R[4] + tmp[5]*R[5];
  res[5] = tmp[3]*R[6] + tmp[4]*R[7] + tmp[5]*R[8];
  res[6] = tmp[6]*R[0] + tmp[7]*R[1] + tmp[8]*R[2];
  res[7] = tmp[6]*R[3] + tmp[7]*R[4] + tmp[8]*R[5];
  res[8] = tmp[6]*R[6] + tmp[7]*R[7] + tmp[8]*R[8];
}


// multiply two matrices, all 3-by-3
void mjuu_mulmat(double* res, const double* A, const double* B) {
  res[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];
  res[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];
  res[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];

  res[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6];
  res[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
  res[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];

  res[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
  res[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];
  res[8] = A[6]*B[2] + A[7]*B[5] + A[8]*B[8];
}


// transpose 3-by-3 matrix
void mjuu_transposemat(double* res, const double* mat) {
  res[0] = mat[0];
  res[3] = mat[1];
  res[6] = mat[2];

  res[1] = mat[3];
  res[4] = mat[4];
  res[7] = mat[5];

  res[2] = mat[6];
  res[5] = mat[7];
  res[8] = mat[8];
}


// convert global to local axis relative to given frame
void mjuu_localaxis(double* al, const double* ag, const double* quat) {
  double mat[9];
  double qneg[4] = {quat[0], -quat[1], -quat[2], -quat[3]};
  mjuu_quat2mat(mat, qneg);
  mjuu_mulvecmat(al, ag, mat);
}


// convert global to local position relative to given frame
void mjuu_localpos(double* pl, const double* pg, const double* pos, const double* quat) {
  double a[3] = {pg[0]-pos[0], pg[1]-pos[1], pg[2]-pos[2]};
  mjuu_localaxis(pl, a, quat);
}


// compute quaternion rotation from parent to child
void mjuu_localquat(double* local, const double* child, const double* parent) {
  double pneg[4] = {parent[0], -parent[1], -parent[2], -parent[3]};
  mjuu_mulquat(local, pneg, child);
}


// compute vector cross-product a = b x c
void mjuu_crossvec(double* a, const double* b, const double* c) {
  a[0] = b[1]*c[2] - b[2]*c[1];
  a[1] = b[2]*c[0] - b[0]*c[2];
  a[2] = b[0]*c[1] - b[1]*c[0];
}


// compute normal vector to given triangle, return length
double mjuu_makenormal(double* normal, const float* a, const float* b, const float* c) {
  double v1[3] = {b[0]-a[0], b[1]-a[1], b[2]-a[2]};
  double v2[3] = {c[0]-a[0], c[1]-a[1], c[2]-a[2]};
  double res;

  mjuu_crossvec(normal, v1, v2);
  if ((res=mjuu_normvec(normal, 3)) < mjEPS) {
    normal[0] = normal[1] = 0;
    normal[2] = 1;
  }

  return res;
}


// compute quaternion as minimal rotation from [0;0;1] to vec
void mjuu_z2quat(double* quat, const double* vec) {
  double z[3] = {0, 0, 1};
  mjuu_crossvec(quat+1, z, vec);
  double s = mjuu_normvec(quat+1, 3);
  if (s<1E-10) {
    quat[1] = 1;
    quat[2] = quat[3] = 0;
  }
  double ang = atan2(s, vec[2]);
  quat[0] = cos(ang/2);
  quat[1] *= sin(ang/2);
  quat[2] *= sin(ang/2);
  quat[3] *= sin(ang/2);
}


// compute quaternion given frame (axes are in matrix columns)
void mjuu_frame2quat(double* quat, const double* x, const double* y, const double* z) {
  const double* mat[3] = {x, y, z};  // mat[c][r] indexing

  // q0 largest
  if (mat[0][0]+mat[1][1]+mat[2][2]>0) {
    quat[0] = 0.5 * sqrt(1 + mat[0][0] + mat[1][1] + mat[2][2]);
    quat[1] = 0.25 * (mat[1][2] - mat[2][1]) / quat[0];
    quat[2] = 0.25 * (mat[2][0] - mat[0][2]) / quat[0];
    quat[3] = 0.25 * (mat[0][1] - mat[1][0]) / quat[0];
  }

  // q1 largest
  else if (mat[0][0]>mat[1][1] && mat[0][0]>mat[2][2]) {
    quat[1] = 0.5 * sqrt(1 + mat[0][0] - mat[1][1] - mat[2][2]);
    quat[0] = 0.25 * (mat[1][2] - mat[2][1]) / quat[1];
    quat[2] = 0.25 * (mat[1][0] + mat[0][1]) / quat[1];
    quat[3] = 0.25 * (mat[2][0] + mat[0][2]) / quat[1];
  }

  // q2 largest
  else if (mat[1][1]>mat[2][2]) {
    quat[2] = 0.5 * sqrt(1 - mat[0][0] + mat[1][1] - mat[2][2]);
    quat[0] = 0.25 * (mat[2][0] - mat[0][2]) / quat[2];
    quat[1] = 0.25 * (mat[1][0] + mat[0][1]) / quat[2];
    quat[3] = 0.25 * (mat[2][1] + mat[1][2]) / quat[2];
  }

  // q3 largest
  else {
    quat[3] = 0.5 * sqrt(1 - mat[0][0] - mat[1][1] + mat[2][2]);
    quat[0] = 0.25 * (mat[0][1] - mat[1][0]) / quat[3];
    quat[1] = 0.25 * (mat[2][0] + mat[0][2]) / quat[3];
    quat[2] = 0.25 * (mat[2][1] + mat[1][2]) / quat[3];
  }

  mjuu_normvec(quat, 4);
}


// invert frame transformation
void mjuu_frameinvert(double* newpos, double* newquat,
                      const double* oldpos, const double* oldquat) {
  // position
  mjuu_localaxis(newpos, oldpos, oldquat);
  newpos[0] = -newpos[0];
  newpos[1] = -newpos[1];
  newpos[2] = -newpos[2];

  // orientation
  newquat[0] = oldquat[0];
  newquat[1] = -oldquat[1];
  newquat[2] = -oldquat[2];
  newquat[3] = -oldquat[3];
}


// accumulate frame transformations (forward kinematics)
void mjuu_frameaccum(double* pos, double* quat,
                     const double* addpos, const double* addquat) {
  double mat[9], vec[3], qtmp[4];
  mjuu_quat2mat(mat, quat);
  mjuu_mulvecmat(vec, addpos, mat);
  pos[0] += vec[0];
  pos[1] += vec[1];
  pos[2] += vec[2];
  mjuu_mulquat(qtmp, quat, addquat);
  mjuu_copyvec(quat, qtmp, 4);
}


// invert frame accumulation
void mjuu_frameaccuminv(double* pos, double* quat,
                        const double* addpos, const double* addquat) {
  double mat[9], vec[3], qtmp[4];
  double qneg[4] = {addquat[0], -addquat[1], -addquat[2], -addquat[3]};
  mjuu_mulquat(qtmp, quat, qneg);
  mjuu_copyvec(quat, qtmp, 4);
  mjuu_quat2mat(mat, quat);
  mjuu_mulvecmat(vec, addpos, mat);
  pos[0] -= vec[0];
  pos[1] -= vec[1];
  pos[2] -= vec[2];
}


// convert local_inertia[3] to global_inertia[6]
void mjuu_globalinertia(double* global, const double* local, const double* quat) {
  double mat[9];
  mjuu_quat2mat(mat, quat);

  double tmp[9] = {
    mat[0]*local[0], mat[3]*local[0], mat[6]*local[0],
    mat[1]*local[1], mat[4]*local[1], mat[7]*local[1],
    mat[2]*local[2], mat[5]*local[2], mat[8]*local[2]
  };

  global[0] = mat[0]*tmp[0] + mat[1]*tmp[3] + mat[2]*tmp[6];
  global[1] = mat[3]*tmp[1] + mat[4]*tmp[4] + mat[5]*tmp[7];
  global[2] = mat[6]*tmp[2] + mat[7]*tmp[5] + mat[8]*tmp[8];
  global[3] = mat[0]*tmp[1] + mat[1]*tmp[4] + mat[2]*tmp[7];
  global[4] = mat[0]*tmp[2] + mat[1]*tmp[5] + mat[2]*tmp[8];
  global[5] = mat[3]*tmp[2] + mat[4]*tmp[5] + mat[5]*tmp[8];
}


// compute off-center correction to inertia matrix
//  mass * [y^2+z^2, -x*y, -x*z;  -x*y, x^2+z^2, -y*z;  -x*z, -y*z, x^2+y^2]
void mjuu_offcenter(double* res, const double mass, const double* vec) {
  res[0] = mass*(vec[1]*vec[1] + vec[2]*vec[2]);
  res[1] = mass*(vec[0]*vec[0] + vec[2]*vec[2]);
  res[2] = mass*(vec[0]*vec[0] + vec[1]*vec[1]);
  res[3] = -mass*vec[0]*vec[1];
  res[4] = -mass*vec[0]*vec[2];
  res[5] = -mass*vec[1]*vec[2];
}


// compute viscosity coefficients from mass and inertia
void mjuu_visccoef(double* visccoef, double mass, const double* inertia, double scl) {
  // compute equivalent box
  double equivbox[3];
  equivbox[0] = sqrt(mjMAX(mjMINVAL, (inertia[1] + inertia[2] - inertia[0])) / mass * 6.0);
  equivbox[1] = sqrt(mjMAX(mjMINVAL, (inertia[0] + inertia[2] - inertia[1])) / mass * 6.0);
  equivbox[2] = sqrt(mjMAX(mjMINVAL, (inertia[0] + inertia[1] - inertia[2])) / mass * 6.0);

  // apply formula for box (or rather cross) viscosity

  //  torque components
  visccoef[0] = scl * 4.0 / 3.0 * equivbox[0] *
                (equivbox[1]*equivbox[1]*equivbox[1] + equivbox[2]*equivbox[2]*equivbox[2]);
  visccoef[1] = scl * 4.0 / 3.0 * equivbox[1] *
                (equivbox[0]*equivbox[0]*equivbox[0] + equivbox[2]*equivbox[2]*equivbox[2]);
  visccoef[2] = scl * 4.0 / 3.0 * equivbox[2] *
                (equivbox[0]*equivbox[0]*equivbox[0] + equivbox[1]*equivbox[1]*equivbox[1]);

  //  force components
  visccoef[3] = scl * 4*equivbox[1]*equivbox[2];
  visccoef[4] = scl * 4*equivbox[0]*equivbox[2];
  visccoef[5] = scl * 4*equivbox[0]*equivbox[1];
}


// strip directory from filename
string mjuu_strippath(string filename) {
  // find last pathsymbol
  size_t start = filename.find_last_of("/\\");

  // no path found: return original
  if (start==string::npos) {
    return filename;
  }

  // return name without path
  else {
    return filename.substr(start+1, filename.size()-start-1);
  }
}


// strip extension
string mjuu_stripext(string filename) {
  // find last dot
  size_t end = filename.find_last_of('.');

  // no path found: return original
  if (end==string::npos) {
    return filename;
  }

  // return name without extension
  else {
    return filename.substr(0, end);
  }
}


// is directory path absolute
bool mjuu_isabspath(string path) {
  // empty: not absolute
  if (path.empty()) {
    return false;
  }

  // check first char
  const char* str = path.c_str();
  if (str[0]=='\\' || str[0]=='/') {
    return true;
  }

  // find ":/" or ":\"
  if (path.find(":/")!=string::npos || path.find(":\\")!=string::npos) {
    return true;
  }

  return false;
}


// get directory path of file
string mjuu_getfiledir(string filename) {
  // no filename
  if (filename.empty()) {
    return "";
  }

  // find last pathsymbol
  size_t last = filename.find_last_of("/\\");

  // no pathsymbol: unknown dir
  if (last==string::npos) {
    return "";
  }

  // extract path from filename
  return filename.substr(0, last+1);
}


// assemble full filename
string mjuu_makefullname(string filedir, string meshdir, string filename) {
  // filename has absolute path: filename
  if (mjuu_isabspath(filename)) {
    return filename;
  }

  // meshdir has absolute path: meshdir + filename
  if (mjuu_isabspath(meshdir)) {
    return meshdir + filename;
  }

  // default
  return filedir + meshdir + filename;
}
