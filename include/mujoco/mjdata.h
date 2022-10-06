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

#ifndef MUJOCO_MJDATA_H_
#define MUJOCO_MJDATA_H_

#include <stddef.h>
#include <stdint.h>

#include <mujoco/mjtnum.h>
#include <mujoco/mjmodel.h>

//---------------------------------- primitive types (mjt) -----------------------------------------

typedef enum mjtWarning_ {        // warning types
  mjWARN_INERTIA      = 0,        // (near) singular inertia matrix
  mjWARN_CONTACTFULL,             // too many contacts in contact list
  mjWARN_CNSTRFULL,               // too many constraints
  mjWARN_VGEOMFULL,               // too many visual geoms
  mjWARN_BADQPOS,                 // bad number in qpos
  mjWARN_BADQVEL,                 // bad number in qvel
  mjWARN_BADQACC,                 // bad number in qacc
  mjWARN_BADCTRL,                 // bad number in ctrl

  mjNWARNING                      // number of warnings
} mjtWarning;


typedef enum mjtTimer_ {
  // main api
  mjTIMER_STEP        = 0,        // step
  mjTIMER_FORWARD,                // forward
  mjTIMER_INVERSE,                // inverse

  // breakdown of step/forward
  mjTIMER_POSITION,               // fwdPosition
  mjTIMER_VELOCITY,               // fwdVelocity
  mjTIMER_ACTUATION,              // fwdActuation
  mjTIMER_ACCELERATION,           // fwdAcceleration
  mjTIMER_CONSTRAINT,             // fwdConstraint

  // breakdown of fwdPosition
  mjTIMER_POS_KINEMATICS,         // kinematics, com, tendon, transmission
  mjTIMER_POS_INERTIA,            // inertia computations
  mjTIMER_POS_COLLISION,          // collision detection
  mjTIMER_POS_MAKE,               // make constraints
  mjTIMER_POS_PROJECT,            // project constraints

  mjNTIMER                        // number of timers
} mjtTimer;


//---------------------------------- mjContact -----------------------------------------------------

struct mjContact_ {               // result of collision detection functions
  // contact parameters set by geom-specific collision detector
  mjtNum dist;                    // distance between nearest points; neg: penetration
  mjtNum pos[3];                  // position of contact point: midpoint between geoms
  mjtNum frame[9];                // normal is in [0-2]

  // contact parameters set by mj_collideGeoms
  mjtNum includemargin;           // include if dist<includemargin=margin-gap
  mjtNum friction[5];             // tangent1, 2, spin, roll1, 2
  mjtNum solref[mjNREF];          // constraint solver reference
  mjtNum solimp[mjNIMP];          // constraint solver impedance

  // internal storage used by solver
  mjtNum mu;                      // friction of regularized cone, set by mj_makeConstraint
  mjtNum H[36];                   // cone Hessian, set by mj_updateConstraint

  // contact descriptors set by mj_collideGeoms
  int dim;                        // contact space dimensionality: 1, 3, 4 or 6
  int geom1;                      // id of geom 1
  int geom2;                      // id of geom 2

  // flag set by mj_fuseContact or mj_instantianteEquality
  int exclude;                    // 0: include, 1: in gap, 2: fused, 3: equality, 4: no dofs

  // address computed by mj_instantiateContact
  int efc_address;                // address in efc; -1: not included, -2-i: distance constraint i
};
typedef struct mjContact_ mjContact;


//---------------------------------- diagnostics ---------------------------------------------------

struct mjWarningStat_ {           // warning statistics
  int lastinfo;                   // info from last warning
  int number;                     // how many times was warning raised
};
typedef struct mjWarningStat_ mjWarningStat;


struct mjTimerStat_ {             // timer statistics
  mjtNum duration;                // cumulative duration
  int number;                     // how many times was timer called
};
typedef struct mjTimerStat_ mjTimerStat;


struct mjSolverStat_ {            // per-iteration solver statistics
  mjtNum improvement;             // cost reduction, scaled by 1/trace(M(qpos0))
  mjtNum gradient;                // gradient norm (primal only, scaled)
  mjtNum lineslope;               // slope in linesearch
  int nactive;                    // number of active constraints
  int nchange;                    // number of constraint state changes
  int neval;                      // number of cost evaluations in line search
  int nupdate;                    // number of Cholesky updates in line search
};
typedef struct mjSolverStat_ mjSolverStat;


//---------------------------------- mjData --------------------------------------------------------

struct mjData_ {
  // constant sizes
  int nstack;                     // number of mjtNums that can fit in the arena+stack space
  int nbuffer;                    // size of main buffer in bytes
  int nplugin;                    // number of plugin instances

  // stack pointer
  size_t pstack;                  // first available mjtNum address in stack

  // arena pointer
  size_t parena;                  // first available byte in arena

  // memory utilization stats
  int maxuse_stack;               // maximum stack allocation
  size_t maxuse_arena;            // maximum arena allocation
  int maxuse_con;                 // maximum number of contacts
  int maxuse_efc;                 // maximum number of scalar constraints

  // diagnostics
  mjWarningStat warning[mjNWARNING]; // warning statistics
  mjTimerStat timer[mjNTIMER];    // timer statistics
  mjSolverStat solver[mjNSOLVER]; // solver statistics per iteration
  int solver_iter;                // number of solver iterations
  int solver_nnz;                 // number of non-zeros in Hessian or efc_AR
  mjtNum solver_fwdinv[2];        // forward-inverse comparison: qfrc, efc

  // variable sizes
  int ne;                         // number of equality constraints
  int nf;                         // number of friction constraints
  int nefc;                       // number of constraints
  int ncon;                       // number of detected contacts

  // global properties
  mjtNum time;                    // simulation time
  mjtNum energy[2];               // potential, kinetic energy

  //-------------------------------- end of info header

  // buffers
  void*     buffer;               // main buffer; all pointers point in it           (nbuffer bytes)
  void*     arena;                // arena+stack buffer                (nstack*sizeof(mjtNum) bytes)

  //-------------------------------- main inputs and outputs of the computation

  // state
  mjtNum*   qpos;                 // position                                 (nq x 1)
  mjtNum*   qvel;                 // velocity                                 (nv x 1)
  mjtNum*   act;                  // actuator activation                      (na x 1)
  mjtNum*   qacc_warmstart;       // acceleration used for warmstart          (nv x 1)
  mjtNum*   plugin_state;         // plugin state                             (npluginstate x 1)

  // control
  mjtNum*   ctrl;                 // control                                  (nu x 1)
  mjtNum*   qfrc_applied;         // applied generalized force                (nv x 1)
  mjtNum*   xfrc_applied;         // applied Cartesian force/torque           (nbody x 6)

  // mocap data
  mjtNum*   mocap_pos;            // positions of mocap bodies                (nmocap x 3)
  mjtNum*   mocap_quat;           // orientations of mocap bodies             (nmocap x 4)

  // dynamics
  mjtNum*   qacc;                 // acceleration                             (nv x 1)
  mjtNum*   act_dot;              // time-derivative of actuator activation   (na x 1)

  // user data
  mjtNum*   userdata;             // user data, not touched by engine         (nuserdata x 1)

  // sensors
  mjtNum*   sensordata;           // sensor data array                        (nsensordata x 1)

  // plugins
  int*          plugin;           // copy of m->plugin, required for deletion (nplugin x 1)
  uintptr_t*    plugin_data;      // pointer to plugin-managed data structure (nplugin x 1)

  //-------------------------------- POSITION dependent

  // computed by mj_fwdPosition/mj_kinematics
  mjtNum*   xpos;                 // Cartesian position of body frame         (nbody x 3)
  mjtNum*   xquat;                // Cartesian orientation of body frame      (nbody x 4)
  mjtNum*   xmat;                 // Cartesian orientation of body frame      (nbody x 9)
  mjtNum*   xipos;                // Cartesian position of body com           (nbody x 3)
  mjtNum*   ximat;                // Cartesian orientation of body inertia    (nbody x 9)
  mjtNum*   xanchor;              // Cartesian position of joint anchor       (njnt x 3)
  mjtNum*   xaxis;                // Cartesian joint axis                     (njnt x 3)
  mjtNum*   geom_xpos;            // Cartesian geom position                  (ngeom x 3)
  mjtNum*   geom_xmat;            // Cartesian geom orientation               (ngeom x 9)
  mjtNum*   site_xpos;            // Cartesian site position                  (nsite x 3)
  mjtNum*   site_xmat;            // Cartesian site orientation               (nsite x 9)
  mjtNum*   cam_xpos;             // Cartesian camera position                (ncam x 3)
  mjtNum*   cam_xmat;             // Cartesian camera orientation             (ncam x 9)
  mjtNum*   light_xpos;           // Cartesian light position                 (nlight x 3)
  mjtNum*   light_xdir;           // Cartesian light direction                (nlight x 3)

  // computed by mj_fwdPosition/mj_comPos
  mjtNum*   subtree_com;          // center of mass of each subtree           (nbody x 3)
  mjtNum*   cdof;                 // com-based motion axis of each dof        (nv x 6)
  mjtNum*   cinert;               // com-based body inertia and mass          (nbody x 10)

  // computed by mj_fwdPosition/mj_tendon
  int*      ten_wrapadr;          // start address of tendon's path           (ntendon x 1)
  int*      ten_wrapnum;          // number of wrap points in path            (ntendon x 1)
  int*      ten_J_rownnz;         // number of non-zeros in Jacobian row      (ntendon x 1)
  int*      ten_J_rowadr;         // row start address in colind array        (ntendon x 1)
  int*      ten_J_colind;         // column indices in sparse Jacobian        (ntendon x nv)
  mjtNum*   ten_length;           // tendon lengths                           (ntendon x 1)
  mjtNum*   ten_J;                // tendon Jacobian                          (ntendon x nv)
  int*      wrap_obj;             // geom id; -1: site; -2: pulley            (nwrap*2 x 1)
  mjtNum*   wrap_xpos;            // Cartesian 3D points in all path          (nwrap*2 x 3)

  // computed by mj_fwdPosition/mj_transmission
  mjtNum*   actuator_length;      // actuator lengths                         (nu x 1)
  mjtNum*   actuator_moment;      // actuator moments                         (nu x nv)

  // computed by mj_fwdPosition/mj_crb
  mjtNum*   crb;                  // com-based composite inertia and mass     (nbody x 10)
  mjtNum*   qM;                   // total inertia (sparse)                   (nM x 1)

  // computed by mj_fwdPosition/mj_factorM
  mjtNum*   qLD;                  // L'*D*L factorization of M (sparse)       (nM x 1)
  mjtNum*   qLDiagInv;            // 1/diag(D)                                (nv x 1)
  mjtNum*   qLDiagSqrtInv;        // 1/sqrt(diag(D))                          (nv x 1)

  //-------------------------------- POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity
  mjtNum*   ten_velocity;         // tendon velocities                        (ntendon x 1)
  mjtNum*   actuator_velocity;    // actuator velocities                      (nu x 1)

  // computed by mj_fwdVelocity/mj_comVel
  mjtNum*   cvel;                 // com-based velocity [3D rot; 3D tran]     (nbody x 6)
  mjtNum*   cdof_dot;             // time-derivative of cdof                  (nv x 6)

  // computed by mj_fwdVelocity/mj_rne (without acceleration)
  mjtNum*   qfrc_bias;            // C(qpos,qvel)                             (nv x 1)

  // computed by mj_fwdVelocity/mj_passive
  mjtNum*   qfrc_passive;         // passive force                            (nv x 1)

  // computed by mj_fwdVelocity/mj_referenceConstraint
  mjtNum*   efc_vel;              // velocity in constraint space: J*qvel     (nefc x 1)
  mjtNum*   efc_aref;             // reference pseudo-acceleration            (nefc x 1)

  // computed by mj_sensorVel/mj_subtreeVel if needed
  mjtNum*   subtree_linvel;       // linear velocity of subtree com           (nbody x 3)
  mjtNum*   subtree_angmom;       // angular momentum about subtree com       (nbody x 3)

  // computed by mj_Euler
  mjtNum*   qH;                   // L'*D*L factorization of modified M       (nM x 1)
  mjtNum*   qHDiagInv;            // 1/diag(D) of modified M                  (nv x 1)

  // computed by mj_implicit
  int*      D_rownnz;             // non-zeros in each row                    (nv x 1)
  int*      D_rowadr;             // address of each row in D_colind          (nv x 1)
  int*      D_colind;             // column indices of non-zeros              (nD x 1)

  // computed by mj_implicit/mj_derivative
  mjtNum*   qDeriv;               // d (passive + actuator - bias) / d qvel   (nD x 1)

  // computed by mj_implicit/mju_factorLUSparse
  mjtNum*   qLU;                  // sparse LU of (qM - dt*qDeriv)            (nD x 1)

  //-------------------------------- POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdActuation
  mjtNum*   actuator_force;       // actuator force in actuation space        (nu x 1)
  mjtNum*   qfrc_actuator;        // actuator force                           (nv x 1)

  // computed by mj_fwdAcceleration
  mjtNum*   qfrc_smooth;          // net unconstrained force                  (nv x 1)
  mjtNum*   qacc_smooth;          // unconstrained acceleration               (nv x 1)

  // computed by mj_fwdConstraint/mj_inverse
  mjtNum*   qfrc_constraint;      // constraint force                         (nv x 1)

  // computed by mj_inverse
  mjtNum*   qfrc_inverse;         // net external force; should equal:        (nv x 1)
                                  // qfrc_applied + J'*xfrc_applied + qfrc_actuator

  // computed by mj_sensorAcc/mj_rnePostConstraint if needed; rotation:translation format
  mjtNum*   cacc;                 // com-based acceleration                   (nbody x 6)
  mjtNum*   cfrc_int;             // com-based interaction force with parent  (nbody x 6)
  mjtNum*   cfrc_ext;             // com-based external force on body         (nbody x 6)

  //-------------------------------- ARENA-ALLOCATED ARRAYS

  // computed by mj_collision
  mjContact* contact;             // list of all detected contacts            (ncon x 1)

  // computed by mj_makeConstraint
  int*      efc_type;             // constraint type (mjtConstraint)          (nefc x 1)
  int*      efc_id;               // id of object of specified type           (nefc x 1)
  int*      efc_J_rownnz;         // number of non-zeros in Jacobian row      (nefc x 1)
  int*      efc_J_rowadr;         // row start address in colind array        (nefc x 1)
  int*      efc_J_rowsuper;       // number of subsequent rows in supernode   (nefc x 1)
  int*      efc_J_colind;         // column indices in Jacobian               (nefc x nv)
  int*      efc_JT_rownnz;        // number of non-zeros in Jacobian row    T (nv x 1)
  int*      efc_JT_rowadr;        // row start address in colind array      T (nv x 1)
  int*      efc_JT_rowsuper;      // number of subsequent rows in supernode T (nv x 1)
  int*      efc_JT_colind;        // column indices in Jacobian             T (nv x nefc)
  mjtNum*   efc_J;                // constraint Jacobian                      (nefc x nv)
  mjtNum*   efc_JT;               // constraint Jacobian transposed           (nv x nefc)
  mjtNum*   efc_pos;              // constraint position (equality, contact)  (nefc x 1)
  mjtNum*   efc_margin;           // inclusion margin (contact)               (nefc x 1)
  mjtNum*   efc_frictionloss;     // frictionloss (friction)                  (nefc x 1)
  mjtNum*   efc_diagApprox;       // approximation to diagonal of A           (nefc x 1)
  mjtNum*   efc_KBIP;             // stiffness, damping, impedance, imp'      (nefc x 4)
  mjtNum*   efc_D;                // constraint mass                          (nefc x 1)
  mjtNum*   efc_R;                // inverse constraint mass                  (nefc x 1)

  // computed by mj_fwdConstraint/mj_inverse
  mjtNum*   efc_b;                // linear cost term: J*qacc_smooth - aref   (nefc x 1)
  mjtNum*   efc_force;            // constraint force in constraint space     (nefc x 1)
  int*      efc_state;            // constraint state (mjtConstraintState)    (nefc x 1)

  // computed by mj_projectConstraint
  int*      efc_AR_rownnz;        // number of non-zeros in AR                (nefc x 1)
  int*      efc_AR_rowadr;        // row start address in colind array        (nefc x 1)
  int*      efc_AR_colind;        // column indices in sparse AR              (nefc x nefc)
  mjtNum*   efc_AR;               // J*inv(M)*J' + R                          (nefc x nefc)
};
typedef struct mjData_ mjData;


//---------------------------------- callback function types ---------------------------------------

// generic MuJoCo function
typedef void (*mjfGeneric)(const mjModel* m, mjData* d);

// contact filter: 1- discard, 0- collide
typedef int (*mjfConFilt)(const mjModel* m, mjData* d, int geom1, int geom2);

// sensor simulation
typedef void (*mjfSensor)(const mjModel* m, mjData* d, int stage);

// timer
typedef mjtNum (*mjfTime)(void);

// actuator dynamics, gain, bias
typedef mjtNum (*mjfAct)(const mjModel* m, const mjData* d, int id);

// collision detection
typedef int (*mjfCollision)(const mjModel* m, const mjData* d,
                            mjContact* con, int g1, int g2, mjtNum margin);

#endif  // MUJOCO_MJDATA_H_
