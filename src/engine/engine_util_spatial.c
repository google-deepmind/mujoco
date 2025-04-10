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

#include "engine/engine_util_spatial.h"

#include <mujoco/mjmodel.h>
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"


//------------------------------ quaternion operations ---------------------------------------------

// rotate vector by quaternion
void mju_rotVecQuat(mjtNum res[3], const mjtNum vec[3], const mjtNum quat[4]) {
  // zero vec: zero res
  if (vec[0] == 0 && vec[1] == 0 && vec[2] == 0) {
    mju_zero3(res);
  }

  // null quat: copy vec
  else if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
    mju_copy3(res, vec);
  }

  // regular processing
  else {
    // tmp = q_w * v + cross(q_xyz, v)
    mjtNum tmp[3] = {
      quat[0]*vec[0] + quat[2]*vec[2] - quat[3]*vec[1],
      quat[0]*vec[1] + quat[3]*vec[0] - quat[1]*vec[2],
      quat[0]*vec[2] + quat[1]*vec[1] - quat[2]*vec[0]
    };

    // res = v + 2 * cross(q_xyz, t)
    res[0] = vec[0] + 2 * (quat[2]*tmp[2] - quat[3]*tmp[1]);
    res[1] = vec[1] + 2 * (quat[3]*tmp[0] - quat[1]*tmp[2]);
    res[2] = vec[2] + 2 * (quat[1]*tmp[1] - quat[2]*tmp[0]);
  }
}



// negate quaternion
void mju_negQuat(mjtNum res[4], const mjtNum quat[4]) {
  res[0] = quat[0];
  res[1] = -quat[1];
  res[2] = -quat[2];
  res[3] = -quat[3];
}



// multiply quaternions
void mju_mulQuat(mjtNum res[4], const mjtNum qa[4], const mjtNum qb[4]) {
  mjtNum tmp[4] = {
    qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3],
    qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2],
    qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1],
    qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
  res[3] = tmp[3];
}



// multiply quaternion and axis
void mju_mulQuatAxis(mjtNum res[4], const mjtNum quat[4], const mjtNum axis[3]) {
  mjtNum tmp[4] = {
    -quat[1]*axis[0] - quat[2]*axis[1] - quat[3]*axis[2],
    quat[0]*axis[0] + quat[2]*axis[2] - quat[3]*axis[1],
    quat[0]*axis[1] + quat[3]*axis[0] - quat[1]*axis[2],
    quat[0]*axis[2] + quat[1]*axis[1] - quat[2]*axis[0]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
  res[3] = tmp[3];
}



// convert axisAngle to quaternion
void mju_axisAngle2Quat(mjtNum res[4], const mjtNum axis[3], mjtNum angle) {
  // zero angle: null quat
  if (angle == 0) {
    res[0] = 1;
    res[1] = 0;
    res[2] = 0;
    res[3] = 0;
  }

  // regular processing
  else {
    mjtNum s = mju_sin(angle*0.5);
    res[0] = mju_cos(angle*0.5);
    res[1] = axis[0]*s;
    res[2] = axis[1]*s;
    res[3] = axis[2]*s;
  }
}



// convert quaternion (corresponding to orientation difference) to 3D velocity
void mju_quat2Vel(mjtNum res[3], const mjtNum quat[4], mjtNum dt) {
  mjtNum axis[3] = {quat[1], quat[2], quat[3]};
  mjtNum sin_a_2 = mju_normalize3(axis);
  mjtNum speed = 2 * mju_atan2(sin_a_2, quat[0]);

  // when axis-angle is larger than pi, rotation is in the opposite direction
  if (speed > mjPI) {
    speed -= 2*mjPI;
  }
  speed /= dt;

  mju_scl3(res, axis, speed);
}



// Subtract quaternions, express as 3D velocity: qb*quat(res) = qa.
void mju_subQuat(mjtNum res[3], const mjtNum qa[4], const mjtNum qb[4]) {
  // qdif = neg(qb)*qa
  mjtNum qneg[4], qdif[4];
  mju_negQuat(qneg, qb);
  mju_mulQuat(qdif, qneg, qa);

  // convert to 3D velocity
  mju_quat2Vel(res, qdif, 1);
}



// convert quaternion to 3D rotation matrix
void mju_quat2Mat(mjtNum res[9], const mjtNum quat[4]) {
  // null quat: identity
  if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
    res[0] = 1;
    res[1] = 0;
    res[2] = 0;
    res[3] = 0;
    res[4] = 1;
    res[5] = 0;
    res[6] = 0;
    res[7] = 0;
    res[8] = 1;
  }

  // regular processing
  else {
    const mjtNum q00 = quat[0]*quat[0];
    const mjtNum q01 = quat[0]*quat[1];
    const mjtNum q02 = quat[0]*quat[2];
    const mjtNum q03 = quat[0]*quat[3];
    const mjtNum q11 = quat[1]*quat[1];
    const mjtNum q12 = quat[1]*quat[2];
    const mjtNum q13 = quat[1]*quat[3];
    const mjtNum q22 = quat[2]*quat[2];
    const mjtNum q23 = quat[2]*quat[3];
    const mjtNum q33 = quat[3]*quat[3];

    res[0] = q00 + q11 - q22 - q33;
    res[4] = q00 - q11 + q22 - q33;
    res[8] = q00 - q11 - q22 + q33;

    res[1] = 2*(q12 - q03);
    res[2] = 2*(q13 + q02);
    res[3] = 2*(q12 + q03);
    res[5] = 2*(q23 - q01);
    res[6] = 2*(q13 - q02);
    res[7] = 2*(q23 + q01);
  }
}



// convert 3D rotation matrix to quaternion
void mju_mat2Quat(mjtNum quat[4], const mjtNum mat[9]) {
  // q0 largest
  if (mat[0]+mat[4]+mat[8] > 0) {
    quat[0] = 0.5 * mju_sqrt(1 + mat[0] + mat[4] + mat[8]);
    quat[1] = 0.25 * (mat[7] - mat[5]) / quat[0];
    quat[2] = 0.25 * (mat[2] - mat[6]) / quat[0];
    quat[3] = 0.25 * (mat[3] - mat[1]) / quat[0];
  }

  // q1 largest
  else if (mat[0] > mat[4] && mat[0] > mat[8]) {
    quat[1] = 0.5 * mju_sqrt(1 + mat[0] - mat[4] - mat[8]);
    quat[0] = 0.25 * (mat[7] - mat[5]) / quat[1];
    quat[2] = 0.25 * (mat[1] + mat[3]) / quat[1];
    quat[3] = 0.25 * (mat[2] + mat[6]) / quat[1];
  }

  // q2 largest
  else if (mat[4] > mat[8]) {
    quat[2] = 0.5 * mju_sqrt(1 - mat[0] + mat[4] - mat[8]);
    quat[0] = 0.25 * (mat[2] - mat[6]) / quat[2];
    quat[1] = 0.25 * (mat[1] + mat[3]) / quat[2];
    quat[3] = 0.25 * (mat[5] + mat[7]) / quat[2];
  }

  // q3 largest
  else {
    quat[3] = 0.5 * mju_sqrt(1 - mat[0] - mat[4] + mat[8]);
    quat[0] = 0.25 * (mat[3] - mat[1]) / quat[3];
    quat[1] = 0.25 * (mat[2] + mat[6]) / quat[3];
    quat[2] = 0.25 * (mat[5] + mat[7]) / quat[3];
  }

  mju_normalize4(quat);
}



// time-derivative of quaternion, given 3D rotational velocity
void mju_derivQuat(mjtNum res[4], const mjtNum quat[4], const mjtNum vel[3]) {
  res[0] = 0.5*(-vel[0]*quat[1] - vel[1]*quat[2] - vel[2]*quat[3]);
  res[1] = 0.5*( vel[0]*quat[0] + vel[1]*quat[3] - vel[2]*quat[2]);
  res[2] = 0.5*(-vel[0]*quat[3] + vel[1]*quat[0] + vel[2]*quat[1]);
  res[3] = 0.5*( vel[0]*quat[2] - vel[1]*quat[1] + vel[2]*quat[0]);
}



// integrate quaternion given 3D angular velocity
void mju_quatIntegrate(mjtNum quat[4], const mjtNum vel[3], mjtNum scale) {
  mjtNum angle, tmp[4], qrot[4];

  // form local rotation quaternion, apply
  mju_copy3(tmp, vel);
  angle = scale * mju_normalize3(tmp);
  mju_axisAngle2Quat(qrot, tmp, angle);
  mju_normalize4(quat);
  mju_mulQuat(quat, quat, qrot);
}



// compute quaternion performing rotation from z-axis to given vector
void mju_quatZ2Vec(mjtNum quat[4], const mjtNum vec[3]) {
  mjtNum axis[3], a, vn[3] = {vec[0], vec[1], vec[2]}, z[3] = {0, 0, 1};

  // set default result to no-rotation quaternion
  quat[0] = 1;
  mju_zero3(quat+1);

  // normalize vector; if too small, no rotation
  if (mju_normalize3(vn) < mjMINVAL) {
    return;
  }

  // compute angle and axis
  mju_cross(axis, z, vn);
  a = mju_normalize3(axis);

  // almost parallel
  if (mju_abs(a) < mjMINVAL) {
    // opposite: 180 deg rotation around x axis
    if (mju_dot3(vn, z) < 0) {
      quat[0] = 0;
      quat[1] = 1;
    }

    return;
  }

  // make quaternion from angle and axis
  a = mju_atan2(a, mju_dot3(vn, z));
  mju_axisAngle2Quat(quat, axis, a);
}



// extract 3D rotation from an arbitrary 3x3 matrix
static const mjtNum rotEPS = 1e-9;
int mju_mat2Rot(mjtNum quat[4], const mjtNum mat[9]) {
  // Müller, Matthias, Jan Bender, Nuttapong Chentanez, and Miles Macklin. "A
  // robust method to extract the rotational part of deformations." In
  // Proceedings of the 9th International Conference on Motion in Games, pp.
  // 55-60. 2016.

  int iter;
  mjtNum col1_mat[3] = {mat[0], mat[3], mat[6]};
  mjtNum col2_mat[3] = {mat[1], mat[4], mat[7]};
  mjtNum col3_mat[3] = {mat[2], mat[5], mat[8]};
  for (iter = 0; iter < 500; iter++) {
    mjtNum rot[9];
    mju_quat2Mat(rot, quat);
    mjtNum col1_rot[3] = {rot[0], rot[3], rot[6]};
    mjtNum col2_rot[3] = {rot[1], rot[4], rot[7]};
    mjtNum col3_rot[3] = {rot[2], rot[5], rot[8]};
    mjtNum omega[3], vec1[3], vec2[3], vec3[3];
    mju_cross(vec1, col1_rot, col1_mat);
    mju_cross(vec2, col2_rot, col2_mat);
    mju_cross(vec3, col3_rot, col3_mat);
    mju_add3(omega, vec1, vec2);
    mju_addTo3(omega, vec3);
    mju_scl3(omega, omega, 1.0 / (mju_abs(mju_dot3(col1_rot, col1_mat) +
                                          mju_dot3(col2_rot, col2_mat) +
                                          mju_dot3(col3_rot, col3_mat)) + mjMINVAL));
    mjtNum w = mju_normalize3(omega);
    if (w < rotEPS) {
      break;
    }
    mjtNum qrot[4];
    mju_axisAngle2Quat(qrot, omega, w);
    mju_mulQuat(quat, qrot, quat);
    mju_normalize4(quat);
  }
  return iter;
}



//------------------------------ pose operations (quat, pos) ---------------------------------------

// multiply two poses
void mju_mulPose(mjtNum posres[3], mjtNum quatres[4],
                 const mjtNum pos1[3], const mjtNum quat1[4],
                 const mjtNum pos2[3], const mjtNum quat2[4]) {
  // quatres = quat1*quat2
  mju_mulQuat(quatres, quat1, quat2);
  mju_normalize4(quatres);

  // posres = quat1*pos2 + pos1
  mju_rotVecQuat(posres, pos2, quat1);
  mju_addTo3(posres, pos1);
}



// negate pose
void mju_negPose(mjtNum posres[3], mjtNum quatres[4], const mjtNum pos[3], const mjtNum quat[4]) {
  // qres = neg(quat)
  mju_negQuat(quatres, quat);

  // pres = -neg(quat)*pos
  mju_rotVecQuat(posres, pos, quatres);
  mju_scl3(posres, posres, -1);
}



// transform vector by pose
void mju_trnVecPose(mjtNum res[3], const mjtNum pos[3], const mjtNum quat[4], const mjtNum vec[3]) {
  // res = quat*vec + pos
  mju_rotVecQuat(res, vec, quat);
  mju_addTo3(res, pos);
}



//------------------------------ spatial algebra ---------------------------------------------------

// vector cross-product, 3D
void mju_cross(mjtNum res[3], const mjtNum a[3], const mjtNum b[3]) {
  mjtNum tmp[3] = {
    a[1]*b[2] - a[2]*b[1],
    a[2]*b[0] - a[0]*b[2],
    a[0]*b[1] - a[1]*b[0]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
}



// cross-product for motion vector
void mju_crossMotion(mjtNum res[6], const mjtNum vel[6], const mjtNum v[6]) {
  res[0] = -vel[2]*v[1] + vel[1]*v[2];
  res[1] =  vel[2]*v[0] - vel[0]*v[2];
  res[2] = -vel[1]*v[0] + vel[0]*v[1];
  res[3] = -vel[2]*v[4] + vel[1]*v[5];
  res[4] =  vel[2]*v[3] - vel[0]*v[5];
  res[5] = -vel[1]*v[3] + vel[0]*v[4];

  res[3] += -vel[5]*v[1] + vel[4]*v[2];
  res[4] +=  vel[5]*v[0] - vel[3]*v[2];
  res[5] += -vel[4]*v[0] + vel[3]*v[1];
}



// cross-product for force vectors
void mju_crossForce(mjtNum res[6], const mjtNum vel[6], const mjtNum f[6]) {
  res[0] = -vel[2]*f[1] + vel[1]*f[2];
  res[1] =  vel[2]*f[0] - vel[0]*f[2];
  res[2] = -vel[1]*f[0] + vel[0]*f[1];
  res[3] = -vel[2]*f[4] + vel[1]*f[5];
  res[4] =  vel[2]*f[3] - vel[0]*f[5];
  res[5] = -vel[1]*f[3] + vel[0]*f[4];

  res[0] += -vel[5]*f[4] + vel[4]*f[5];
  res[1] +=  vel[5]*f[3] - vel[3]*f[5];
  res[2] += -vel[4]*f[3] + vel[3]*f[4];
}



// express inertia in com-based frame
void mju_inertCom(mjtNum res[10], const mjtNum inert[3], const mjtNum mat[9],
                  const mjtNum dif[3], mjtNum mass) {
  // tmp = diag(inert) * mat'  (mat is local-to-global rotation)
  mjtNum tmp[9] = {mat[0]*inert[0], mat[3]*inert[0], mat[6]*inert[0],
                   mat[1]*inert[1], mat[4]*inert[1], mat[7]*inert[1],
                   mat[2]*inert[2], mat[5]*inert[2], mat[8]*inert[2]};

  // res_rot = mat * diag(inert) * mat'
  res[0] = mat[0]*tmp[0] + mat[1]*tmp[3] + mat[2]*tmp[6];
  res[1] = mat[3]*tmp[1] + mat[4]*tmp[4] + mat[5]*tmp[7];
  res[2] = mat[6]*tmp[2] + mat[7]*tmp[5] + mat[8]*tmp[8];
  res[3] = mat[0]*tmp[1] + mat[1]*tmp[4] + mat[2]*tmp[7];
  res[4] = mat[0]*tmp[2] + mat[1]*tmp[5] + mat[2]*tmp[8];
  res[5] = mat[3]*tmp[2] + mat[4]*tmp[5] + mat[5]*tmp[8];

  // res_rot -= mass * dif_cross * dif_cross
  res[0] += mass*(dif[1]*dif[1] + dif[2]*dif[2]);
  res[1] += mass*(dif[0]*dif[0] + dif[2]*dif[2]);
  res[2] += mass*(dif[0]*dif[0] + dif[1]*dif[1]);
  res[3] -= mass*dif[0]*dif[1];
  res[4] -= mass*dif[0]*dif[2];
  res[5] -= mass*dif[1]*dif[2];

  // res_tran = mass * dif
  res[6] = mass*dif[0];
  res[7] = mass*dif[1];
  res[8] = mass*dif[2];

  // res_mass = mass
  res[9] = mass;
}



// multiply 6D vector (rotation, translation) by 6D inertia matrix
void mju_mulInertVec(mjtNum res[6], const mjtNum i[10], const mjtNum v[6]) {
  res[0] = i[0]*v[0] + i[3]*v[1] + i[4]*v[2] - i[8]*v[4] + i[7]*v[5];
  res[1] = i[3]*v[0] + i[1]*v[1] + i[5]*v[2] + i[8]*v[3] - i[6]*v[5];
  res[2] = i[4]*v[0] + i[5]*v[1] + i[2]*v[2] - i[7]*v[3] + i[6]*v[4];
  res[3] = i[8]*v[1] - i[7]*v[2] + i[9]*v[3];
  res[4] = i[6]*v[2] - i[8]*v[0] + i[9]*v[4];
  res[5] = i[7]*v[0] - i[6]*v[1] + i[9]*v[5];
}



// express motion axis in com-based frame
void mju_dofCom(mjtNum res[6], const mjtNum axis[3], const mjtNum offset[3]) {
  // hinge
  if (offset) {
    mju_copy3(res, axis);
    mju_cross(res+3, axis, offset);
  }

  // slide
  else {
    mju_zero3(res);
    mju_copy3(res+3, axis);
  }
}



// multiply dof matrix (6-by-n, transposed) by vector (n-by-1)
void mju_mulDofVec(mjtNum* res, const mjtNum* dof, const mjtNum* vec, int n) {
  if (n == 1) {
    mju_scl(res, dof, vec[0], 6);
  } else if (n <= 0) {
    mju_zero(res, 6);
  } else {
    mju_mulMatTVec(res, dof, vec, n, 6);
  }
}



// transform 6D motion or force vector between frames
//  rot is 3-by-3 matrix; flg_force determines vector type (motion or force)
void mju_transformSpatial(mjtNum res[6], const mjtNum vec[6], int flg_force,
                          const mjtNum newpos[3], const mjtNum oldpos[3],
                          const mjtNum rotnew2old[9]) {
  mjtNum cros[3], dif[3], tran[6];

  // apply translation
  mju_copy(tran, vec, 6);
  mju_sub3(dif, newpos, oldpos);
  if (flg_force) {
    mju_cross(cros, dif, vec+3);
    mju_sub3(tran, vec, cros);
  } else {
    mju_cross(cros, dif, vec);
    mju_sub3(tran+3, vec+3, cros);
  }

  // apply rotation if provided
  if (rotnew2old) {
    mju_mulMatTVec3(res, rotnew2old, tran);
    mju_mulMatTVec3(res+3, rotnew2old, tran+3);
  }

  // otherwise copy
  else {
    mju_copy(res, tran, 6);
  }
}



// make 3D frame given X axis (normal) and possibly Y axis (tangent 1)
void mju_makeFrame(mjtNum frame[9]) {
  mjtNum tmp[3];

  // normalize xaxis
  if (mju_normalize3(frame) < 0.5) {
    mjERROR("xaxis of contact frame undefined");
  }

  // if yaxis undefined, set yaxis to (0,1,0) if possible, otherwise (0,0,1)
  if (mju_dot3(frame+3, frame+3) < 0.25) {
    mju_zero3(frame+3);

    if (frame[1] < 0.5 && frame[1] > -0.5) {
      frame[4] = 1;
    } else {
      frame[5] = 1;
    }
  }

  // make yaxis orthogonal to xaxis
  mju_scl3(tmp, frame, mju_dot3(frame, frame+3));
  mju_subFrom3(frame+3, tmp);
  mju_normalize3(frame+3);

  // zaxis = cross(xaxis, yaxis)
  mju_cross(frame+6, frame, frame+3);
}



// convert sequence of Euler angles (radians) to quaternion
// seq[0,1,2] must be in 'xyzXYZ', lower/upper-case mean intrinsic/extrinsic rotations
void mju_euler2Quat(mjtNum quat[4], const mjtNum euler[3], const char* seq) {
  if (strnlen(seq, 4) != 3) {
    mjERROR("seq must contain exactly 3 characters");
  }

  // init
  mjtNum tmp[4] = {1, 0, 0, 0};

  // loop over euler angles, accumulate rotations
  for (int i=0; i<3; i++) {
    // construct quaternion rotation
    mjtNum rot[4] = {cos(euler[i]/2), 0, 0, 0};
    mjtNum sa = sin(euler[i]/2);
    if (seq[i]=='x' || seq[i]=='X') {
      rot[1] = sa;
    } else if (seq[i]=='y' || seq[i]=='Y') {
      rot[2] = sa;
    } else if (seq[i]=='z' || seq[i]=='Z') {
      rot[3] = sa;
    } else {
      mjERROR("seq[%d] is '%c', should be one of x, y, z, X, Y, Z", i, seq[i]);
    }

    // accumulate rotation
    if (seq[i]=='x' || seq[i]=='y' || seq[i]=='z') {
      mju_mulQuat(tmp, tmp, rot);  // moving axes: post-multiply
    } else {
      mju_mulQuat(tmp, rot, tmp);  // fixed axes: pre-multiply
    }
  }

  mju_copy4(quat, tmp);
}
