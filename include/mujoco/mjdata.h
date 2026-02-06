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
#include <mujoco/mjthread.h>

//---------------------------------- primitive types (mjt) -----------------------------------------

typedef enum mjtState_ {            // state elements
  mjSTATE_TIME           = 1<<0,    // time
  mjSTATE_QPOS           = 1<<1,    // position
  mjSTATE_QVEL           = 1<<2,    // velocity
  mjSTATE_ACT            = 1<<3,    // actuator activation
  mjSTATE_HISTORY        = 1<<4,    // history buffers (control, sensor)
  mjSTATE_WARMSTART      = 1<<5,    // acceleration used for warmstart
  mjSTATE_CTRL           = 1<<6,    // control
  mjSTATE_QFRC_APPLIED   = 1<<7,    // applied generalized force
  mjSTATE_XFRC_APPLIED   = 1<<8,    // applied Cartesian force/torque
  mjSTATE_EQ_ACTIVE      = 1<<9,    // enable/disable constraints
  mjSTATE_MOCAP_POS      = 1<<10,   // positions of mocap bodies
  mjSTATE_MOCAP_QUAT     = 1<<11,   // orientations of mocap bodies
  mjSTATE_USERDATA       = 1<<12,   // user data
  mjSTATE_PLUGIN         = 1<<13,   // plugin state

  mjNSTATE               = 14,      // number of state elements

  // convenience values for commonly used state specifications
  mjSTATE_PHYSICS        = mjSTATE_QPOS | mjSTATE_QVEL | mjSTATE_ACT | mjSTATE_HISTORY,
  mjSTATE_FULLPHYSICS    = mjSTATE_TIME | mjSTATE_PHYSICS | mjSTATE_PLUGIN,
  mjSTATE_USER           = mjSTATE_CTRL | mjSTATE_QFRC_APPLIED | mjSTATE_XFRC_APPLIED |
                          mjSTATE_EQ_ACTIVE | mjSTATE_MOCAP_POS | mjSTATE_MOCAP_QUAT |
                          mjSTATE_USERDATA,
  mjSTATE_INTEGRATION    = mjSTATE_FULLPHYSICS | mjSTATE_USER | mjSTATE_WARMSTART
} mjtState;


typedef enum mjtConstraint_ {       // type of constraint
  mjCNSTR_EQUALITY       = 0,       // equality constraint
  mjCNSTR_FRICTION_DOF,             // dof friction
  mjCNSTR_FRICTION_TENDON,          // tendon friction
  mjCNSTR_LIMIT_JOINT,              // joint limit
  mjCNSTR_LIMIT_TENDON,             // tendon limit
  mjCNSTR_CONTACT_FRICTIONLESS,     // frictionless contact
  mjCNSTR_CONTACT_PYRAMIDAL,        // frictional contact, pyramidal friction cone
  mjCNSTR_CONTACT_ELLIPTIC          // frictional contact, elliptic friction cone
} mjtConstraint;


typedef enum mjtConstraintState_ {  // constraint state
  mjCNSTRSTATE_SATISFIED = 0,       // constraint satisfied, zero cost (limit, contact)
  mjCNSTRSTATE_QUADRATIC,           // quadratic cost (equality, friction, limit, contact)
  mjCNSTRSTATE_LINEARNEG,           // linear cost, negative side (friction)
  mjCNSTRSTATE_LINEARPOS,           // linear cost, positive side (friction)
  mjCNSTRSTATE_CONE                 // squared distance to cone cost (elliptic contact)
} mjtConstraintState;


typedef enum mjtWarning_ {          // warning types
  mjWARN_INERTIA         = 0,       // (near) singular inertia matrix
  mjWARN_CONTACTFULL,               // too many contacts in contact list
  mjWARN_CNSTRFULL,                 // too many constraints
  mjWARN_VGEOMFULL,                 // too many visual geoms
  mjWARN_BADQPOS,                   // bad number in qpos
  mjWARN_BADQVEL,                   // bad number in qvel
  mjWARN_BADQACC,                   // bad number in qacc
  mjWARN_BADCTRL,                   // bad number in ctrl

  mjNWARNING                        // number of warnings
} mjtWarning;


typedef enum mjtTimer_ {            // internal timers
  // main api
  mjTIMER_STEP           = 0,       // step
  mjTIMER_FORWARD,                  // forward
  mjTIMER_INVERSE,                  // inverse

  // breakdown of step/forward
  mjTIMER_POSITION,                 // fwdPosition
  mjTIMER_VELOCITY,                 // fwdVelocity
  mjTIMER_ACTUATION,                // fwdActuation
  mjTIMER_CONSTRAINT,               // fwdConstraint
  mjTIMER_ADVANCE,                  // mj_Euler, mj_implicit

  // breakdown of fwdPosition
  mjTIMER_POS_KINEMATICS,           // kinematics, com, tendon, transmission
  mjTIMER_POS_INERTIA,              // inertia computations
  mjTIMER_POS_COLLISION,            // collision detection
  mjTIMER_POS_MAKE,                 // make constraints
  mjTIMER_POS_PROJECT,              // project constraints

  // breakdown of mj_collision
  mjTIMER_COL_BROAD,                // broadphase
  mjTIMER_COL_NARROW,               // narrowphase

  mjNTIMER                          // number of timers
} mjtTimer;


typedef enum mjtSleepState_ {       // sleep state of an object
  mjS_STATIC = -1,                  // object is static
  mjS_ASLEEP = 0,                   // object is asleep
  mjS_AWAKE  = 1                    // object is awake
} mjtSleepState;


//---------------------------------- mjContact -----------------------------------------------------

struct mjContact_ {                // result of collision detection functions
  // contact parameters set by near-phase collision function
  mjtNum  dist;                    // distance between nearest points; neg: penetration
  mjtNum  pos[3];                  // position of contact point: midpoint between geoms
  mjtNum  frame[9];                // normal is in [0-2], points from geom[0] to geom[1]

  // contact parameters set by mj_collideGeoms
  mjtNum  includemargin;           // include if dist<includemargin=margin-gap
  mjtNum  friction[5];             // tangent1, 2, spin, roll1, 2
  mjtNum  solref[mjNREF];          // constraint solver reference, normal direction
  mjtNum  solreffriction[mjNREF];  // constraint solver reference, friction directions
  mjtNum  solimp[mjNIMP];          // constraint solver impedance

  // internal storage used by solver
  mjtNum  mu;                      // friction of regularized cone, set by mj_makeConstraint
  mjtNum  H[36];                   // cone Hessian, set by mj_constraintUpdate

  // contact descriptors set by mj_collideXXX
  int     dim;                     // contact space dimensionality: 1, 3, 4 or 6
  int     geom1;                   // id of geom 1; deprecated, use geom[0]
  int     geom2;                   // id of geom 2; deprecated, use geom[1]
  int     geom[2];                 // geom ids; -1 for flex
  int     flex[2];                 // flex ids; -1 for geom
  int     elem[2];                 // element ids; -1 for geom or flex vertex
  int     vert[2];                 // vertex ids;  -1 for geom or flex element

  // flag set by mj_setContact or mj_instantiateContact
  int     exclude;                 // 0: include, 1: in gap, 2: fused, 3: no dofs, 4: passive

  // address computed by mj_instantiateContact
  int     efc_address;             // address in efc; -1: not included
};
typedef struct mjContact_ mjContact;


//---------------------------------- diagnostics ---------------------------------------------------

struct mjWarningStat_ {      // warning statistics
  int     lastinfo;          // info from last warning
  int     number;            // how many times was warning raised
};
typedef struct mjWarningStat_ mjWarningStat;


struct mjTimerStat_ {        // timer statistics
  mjtNum  duration;          // cumulative duration
  int     number;            // how many times was timer called
};
typedef struct mjTimerStat_ mjTimerStat;


struct mjSolverStat_ {       // per-iteration solver statistics
  mjtNum  improvement;       // cost reduction, scaled by 1/trace(M(qpos0))
  mjtNum  gradient;          // gradient norm (primal only, scaled)
  mjtNum  lineslope;         // slope in linesearch
  int     nactive;           // number of active constraints
  int     nchange;           // number of constraint state changes
  int     neval;             // number of cost evaluations in line search
  int     nupdate;           // number of Cholesky updates in line search
};
typedef struct mjSolverStat_ mjSolverStat;


//---------------------------------- mjData --------------------------------------------------------

struct mjData_ {
  // constant sizes
  mjtSize narena;            // size of the arena in bytes (inclusive of the stack)
  mjtSize nbuffer;           // size of main buffer in bytes
  int     nplugin;           // number of plugin instances

  // stack pointer
  size_t  pstack;            // first available byte in stack (mutable)
  size_t  pbase;             // value of pstack when mj_markStack was last called (mutable)

  // arena pointer
  size_t  parena;            // first available byte in arena

  // memory utilization statistics
  mjtSize maxuse_stack;                       // maximum stack allocation in bytes (mutable)
  mjtSize maxuse_threadstack[mjMAXTHREAD];    // maximum stack allocation per thread in bytes
  mjtSize maxuse_arena;                       // maximum arena allocation in bytes
  int     maxuse_con;                         // maximum number of contacts
  int     maxuse_efc;                         // maximum number of scalar constraints

  // solver statistics
  mjSolverStat  solver[mjNISLAND*mjNSOLVER];  // solver statistics per island, per iteration
  int           solver_niter[mjNISLAND];      // number of solver iterations, per island
  int           solver_nnz[mjNISLAND];        // number of nonzeros in Hessian or efc_AR, per island
  mjtNum        solver_fwdinv[2];             // forward-inverse comparison: qfrc, efc

  // diagnostics
  mjWarningStat warning[mjNWARNING];          // warning statistics (mutable)
  mjTimerStat   timer[mjNTIMER];              // timer statistics

  // variable sizes
  int     ncon;              // number of detected contacts
  int     ne;                // number of equality constraints
  int     nf;                // number of friction constraints
  int     nl;                // number of limit constraints
  int     nefc;              // number of constraints
  int     nJ;                // number of non-zeros in constraint Jacobian
  int     nA;                // number of non-zeros in constraint inverse inertia matrix
  int     nisland;           // number of detected constraint islands
  int     nidof;             // number of dofs in all islands
  int     ntree_awake;       // number of awake trees
  int     nbody_awake;       // number of awake dynamic and static bodies
  int     nparent_awake;     // number of bodies with awake parents
  int     nv_awake;          // number of awake dofs

  // flags marking lazily evaluated stages
  mjtByte flg_energypos;     // has mj_energyPos been called
  mjtByte flg_energyvel;     // has mj_energyVel been called
  mjtByte flg_subtreevel;    // has mj_subtreeVel been called
  mjtByte flg_rnepost;       // has mj_rnePostConstraint been called

  // global properties
  mjtNum  time;              // simulation time
  mjtNum  energy[2];         // potential, kinetic energy

  //-------------------- end of info header

  // buffers
  void*   buffer;            // main buffer; all pointers point in it            (nbuffer bytes)
  void*   arena;             // arena+stack buffer                               (narena bytes)

  //-------------------- main inputs and outputs of the computation

  // state
  mjtNum* qpos;              // position                                         (nq x 1)
  mjtNum* qvel;              // velocity                                         (nv x 1)
  mjtNum* act;               // actuator activation                              (na x 1)
  mjtNum* history;           // history buffer                                   (nhistory x 1)
  mjtNum* qacc_warmstart;    // acceleration used for warmstart                  (nv x 1)
  mjtNum* plugin_state;      // plugin state                                     (npluginstate x 1)

  // control
  mjtNum* ctrl;              // control                                          (nu x 1)
  mjtNum* qfrc_applied;      // applied generalized force                        (nv x 1)
  mjtNum* xfrc_applied;      // applied Cartesian force/torque                   (nbody x 6)
  mjtByte* eq_active;        // enable/disable constraints                       (neq x 1)

  // mocap data
  mjtNum* mocap_pos;         // positions of mocap bodies                        (nmocap x 3)
  mjtNum* mocap_quat;        // orientations of mocap bodies                     (nmocap x 4)

  // dynamics
  mjtNum* qacc;              // acceleration                                     (nv x 1)
  mjtNum* act_dot;           // time-derivative of actuator activation           (na x 1)

  // user data
  mjtNum* userdata;          // user data, not touched by engine                 (nuserdata x 1)

  // sensors
  mjtNum* sensordata;        // sensor data array                                (nsensordata x 1)

  // sleep state
  int*    tree_asleep;       // <0: awake; >=0: index cycle of sleeping trees    (ntree x 1)

  // plugins
  int*       plugin;         // copy of m->plugin, required for deletion         (nplugin x 1)
  uintptr_t* plugin_data;    // pointer to plugin-managed data structure         (nplugin x 1)

  //-------------------- POSITION dependent

  // computed by mj_fwdPosition/mj_kinematics
  mjtNum* xpos;              // Cartesian position of body frame                 (nbody x 3)
  mjtNum* xquat;             // Cartesian orientation of body frame              (nbody x 4)
  mjtNum* xmat;              // Cartesian orientation of body frame              (nbody x 9)
  mjtNum* xipos;             // Cartesian position of body com                   (nbody x 3)
  mjtNum* ximat;             // Cartesian orientation of body inertia            (nbody x 9)
  mjtNum* xanchor;           // Cartesian position of joint anchor               (njnt x 3)
  mjtNum* xaxis;             // Cartesian joint axis                             (njnt x 3)
  mjtNum* geom_xpos;         // Cartesian geom position                          (ngeom x 3)
  mjtNum* geom_xmat;         // Cartesian geom orientation                       (ngeom x 9)
  mjtNum* site_xpos;         // Cartesian site position                          (nsite x 3)
  mjtNum* site_xmat;         // Cartesian site orientation                       (nsite x 9)
  mjtNum* cam_xpos;          // Cartesian camera position                        (ncam x 3)
  mjtNum* cam_xmat;          // Cartesian camera orientation                     (ncam x 9)
  mjtNum* light_xpos;        // Cartesian light position                         (nlight x 3)
  mjtNum* light_xdir;        // Cartesian light direction                        (nlight x 3)

  // computed by mj_fwdPosition/mj_comPos
  mjtNum* subtree_com;       // center of mass of each subtree                   (nbody x 3)
  mjtNum* cdof;              // com-based motion axis of each dof (rot:lin)      (nv x 6)
  mjtNum* cinert;            // com-based body inertia and mass                  (nbody x 10)

  // computed by mj_fwdPosition/mj_flex
  mjtNum* flexvert_xpos;     // Cartesian flex vertex positions                  (nflexvert x 3)
  mjtNum* flexelem_aabb;     // flex element bounding boxes (center, size)       (nflexelem x 6)
  mjtNum* flexedge_J;        // flex edge Jacobian                               (nJfe x 1)
  mjtNum* flexedge_length;   // flex edge lengths                                (nflexedge x 1)
  mjtNum* flexvert_J;        // flex vertex Jacobian                             (nJfv x 2)
  mjtNum* flexvert_length;   // flex vertex lengths                              (nflexvert x 2)
  mjtNum* bvh_aabb_dyn;      // global bounding box (center, size)               (nbvhdynamic x 6)

  // computed by mj_fwdPosition/mj_tendon
  int*    ten_wrapadr;       // start address of tendon's path                   (ntendon x 1)
  int*    ten_wrapnum;       // number of wrap points in path                    (ntendon x 1)
  int*    ten_J_rownnz;      // number of non-zeros in Jacobian row              (ntendon x 1)
  int*    ten_J_rowadr;      // row start address in colind array                (ntendon x 1)
  int*    ten_J_colind;      // column indices in sparse Jacobian                (ntendon x nv)
  mjtNum* ten_J;             // tendon Jacobian                                  (ntendon x nv)
  mjtNum* ten_length;        // tendon lengths                                   (ntendon x 1)
  int*    wrap_obj;          // geom id; -1: site; -2: pulley                    (nwrap x 2)
  mjtNum* wrap_xpos;         // Cartesian 3D points in all paths                 (nwrap x 6)

  // computed by mj_fwdPosition/mj_transmission
  mjtNum* actuator_length;   // actuator lengths                                 (nu x 1)
  int*    moment_rownnz;     // number of non-zeros in actuator_moment row       (nu x 1)
  int*    moment_rowadr;     // row start address in colind array                (nu x 1)
  int*    moment_colind;     // column indices in sparse Jacobian                (nJmom x 1)
  mjtNum* actuator_moment;   // actuator moments                                 (nJmom x 1)

  // computed by mj_fwdPosition/mj_makeM
  mjtNum* crb;               // com-based composite inertia and mass             (nbody x 10)
  mjtNum* qM;                // inertia (sparse)                                 (nM x 1)
  mjtNum* M;                 // reduced inertia (compressed sparse row)          (nC x 1)

  // computed by mj_fwdPosition/mj_factorM
  mjtNum* qLD;               // L'*D*L factorization of M (sparse)               (nC x 1)
  mjtNum* qLDiagInv;         // 1/diag(D)                                        (nv x 1)

  // computed by mj_collision/mj_collideTree
  mjtByte* bvh_active;       // was bounding volume checked for collision        (nbvh x 1)

  // computed by mj_updateSleep
  int*    tree_awake;        // is tree awake; 0: asleep; 1: awake               (ntree x 1)
  int*    body_awake;        // body sleep state (mjtSleepState)                 (nbody x 1)
  int*    body_awake_ind;    // indices of awake and static bodies               (nbody x 1)
  int*    parent_awake_ind;  // indices of bodies with awake or static parents   (nbody x 1)
  int*    dof_awake_ind;     // indices of awake dofs                            (nv x 1)

  //-------------------- POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity
  mjtNum* flexedge_velocity; // flex edge velocities                             (nflexedge x 1)
  mjtNum* ten_velocity;      // tendon velocities                                (ntendon x 1)
  mjtNum* actuator_velocity; // actuator velocities                              (nu x 1)

  // computed by mj_fwdVelocity/mj_comVel
  mjtNum* cvel;              // com-based velocity (rot:lin)                     (nbody x 6)
  mjtNum* cdof_dot;          // time-derivative of cdof (rot:lin)                (nv x 6)

  // computed by mj_fwdVelocity/mj_rne (without acceleration)
  mjtNum* qfrc_bias;         // C(qpos,qvel)                                     (nv x 1)

  // computed by mj_fwdVelocity/mj_passive
  mjtNum* qfrc_spring;       // passive spring force                             (nv x 1)
  mjtNum* qfrc_damper;       // passive damper force                             (nv x 1)
  mjtNum* qfrc_gravcomp;     // passive gravity compensation force               (nv x 1)
  mjtNum* qfrc_fluid;        // passive fluid force                              (nv x 1)
  mjtNum* qfrc_passive;      // total passive force                              (nv x 1)

  // computed by mj_sensorVel/mj_subtreeVel if needed
  mjtNum* subtree_linvel;    // linear velocity of subtree com                   (nbody x 3)
  mjtNum* subtree_angmom;    // angular momentum about subtree com               (nbody x 3)

  // computed by mj_Euler or mj_implicit
  mjtNum* qH;                // L'*D*L factorization of modified M               (nC x 1)
  mjtNum* qHDiagInv;         // 1/diag(D) of modified M                          (nv x 1)

  // computed by mj_implicit/mj_derivative
  mjtNum* qDeriv;            // d (passive + actuator - bias) / d qvel           (nD x 1)

  // computed by mj_implicit/mju_factorLUSparse
  mjtNum* qLU;               // sparse LU of (qM - dt*qDeriv)                    (nD x 1)

  //-------------------- POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdActuation
  mjtNum* actuator_force;    // actuator force in actuation space                (nu x 1)
  mjtNum* qfrc_actuator;     // actuator force                                   (nv x 1)

  // computed by mj_fwdAcceleration
  mjtNum* qfrc_smooth;       // net unconstrained force                          (nv x 1)
  mjtNum* qacc_smooth;       // unconstrained acceleration                       (nv x 1)

  // computed by mj_fwdConstraint/mj_inverse
  mjtNum* qfrc_constraint;   // constraint force                                 (nv x 1)

  // computed by mj_inverse
  mjtNum* qfrc_inverse;      // net external force; should equal:
                             // qfrc_applied + J'*xfrc_applied + qfrc_actuator   (nv x 1)

  // computed by mj_sensorAcc/mj_rnePostConstraint if needed; rotation:translation format
  mjtNum* cacc;              // com-based acceleration                           (nbody x 6)
  mjtNum* cfrc_int;          // com-based interaction force with parent          (nbody x 6)
  mjtNum* cfrc_ext;          // com-based external force on body                 (nbody x 6)

  //-------------------- arena-allocated: POSITION dependent

  // computed by mj_collision
  mjContact* contact;        // array of all detected contacts                   (ncon x 1)

  // computed by mj_makeConstraint
  int*    efc_type;          // constraint type (mjtConstraint)                  (nefc x 1)
  int*    efc_id;            // id of object of specified type                   (nefc x 1)
  int*    efc_J_rownnz;      // number of non-zeros in constraint Jacobian row   (nefc x 1)
  int*    efc_J_rowadr;      // row start address in colind array                (nefc x 1)
  int*    efc_J_rowsuper;    // number of subsequent rows in supernode           (nefc x 1)
  int*    efc_J_colind;      // column indices in constraint Jacobian            (nJ x 1)
  mjtNum* efc_J;             // constraint Jacobian                              (nJ x 1)
  mjtNum* efc_pos;           // constraint position (equality, contact)          (nefc x 1)
  mjtNum* efc_margin;        // inclusion margin (contact)                       (nefc x 1)
  mjtNum* efc_frictionloss;  // frictionloss (friction)                          (nefc x 1)
  mjtNum* efc_diagApprox;    // approximation to diagonal of A                   (nefc x 1)
  mjtNum* efc_KBIP;          // stiffness, damping, impedance, imp'              (nefc x 4)
  mjtNum* efc_D;             // constraint mass                                  (nefc x 1)
  mjtNum* efc_R;             // inverse constraint mass                          (nefc x 1)
  int*    tendon_efcadr;     // first efc address involving tendon; -1: none     (ntendon x 1)

  // computed by mj_island (island tree structure)
  int*    tree_island;       // island id of this tree; -1: none                 (ntree x 1)
  int*    island_ntree;      // number of trees in this island                   (nisland x 1)
  int*    island_itreeadr;   // island start address in itree vector             (nisland x 1)
  int*    map_itree2tree;    // map from itree to tree                           (ntree x 1)

  // computed by mj_island (island dof structure)
  int*    dof_island;        // island id of this dof; -1: none                  (nv x 1)
  int*    island_nv;         // number of dofs in this island                    (nisland x 1)
  int*    island_idofadr;    // island start address in idof vector              (nisland x 1)
  int*    island_dofadr;     // island start address in dof vector               (nisland x 1)
  int*    map_dof2idof;      // map from dof to idof                             (nv x 1)
  int*    map_idof2dof;      // map from idof to dof;  >= nidof: unconstrained   (nv x 1)

  // computed by mj_island (dofs sorted by island)
  mjtNum* ifrc_smooth;       // net unconstrained force                          (nidof x 1)
  mjtNum* iacc_smooth;       // unconstrained acceleration                       (nidof x 1)
  int*    iM_rownnz;         // inertia: non-zeros in each row                   (nidof x 1)
  int*    iM_rowadr;         // inertia: address of each row in iM_colind        (nidof x 1)
  int*    iM_colind;         // inertia: column indices of non-zeros             (nC x 1)
  mjtNum* iM;                // total inertia (sparse)                           (nC x 1)
  mjtNum* iLD;               // L'*D*L factorization of M (sparse)               (nC x 1)
  mjtNum* iLDiagInv;         // 1/diag(D)                                        (nidof x 1)
  mjtNum* iacc;              // acceleration                                     (nidof x 1)

  // computed by mj_island (island constraint structure)
  int*    efc_island;        // island id of this constraint                     (nefc x 1)
  int*    island_ne;         // number of equality constraints in island         (nisland x 1)
  int*    island_nf;         // number of friction constraints in island         (nisland x 1)
  int*    island_nefc;       // number of constraints in island                  (nisland x 1)
  int*    island_iefcadr;    // start address in iefc vector                     (nisland x 1)
  int*    map_efc2iefc;      // map from efc to iefc                             (nefc x 1)
  int*    map_iefc2efc;      // map from iefc to efc                             (nefc x 1)

  // computed by mj_island (constraints sorted by island)
  int*    iefc_type;         // constraint type (mjtConstraint)                  (nefc x 1)
  int*    iefc_id;           // id of object of specified type                   (nefc x 1)
  int*    iefc_J_rownnz;     // number of non-zeros in constraint Jacobian row   (nefc x 1)
  int*    iefc_J_rowadr;     // row start address in colind array                (nefc x 1)
  int*    iefc_J_rowsuper;   // number of subsequent rows in supernode           (nefc x 1)
  int*    iefc_J_colind;     // column indices in constraint Jacobian            (nJ x 1)
  mjtNum* iefc_J;            // constraint Jacobian                              (nJ x 1)
  mjtNum* iefc_frictionloss; // frictionloss (friction)                          (nefc x 1)
  mjtNum* iefc_D;            // constraint mass                                  (nefc x 1)
  mjtNum* iefc_R;            // inverse constraint mass                          (nefc x 1)

  // computed by mj_projectConstraint (PGS solver)
  int*    efc_AR_rownnz;     // number of non-zeros in AR                        (nefc x 1)
  int*    efc_AR_rowadr;     // row start address in colind array                (nefc x 1)
  int*    efc_AR_colind;     // column indices in sparse AR                      (nA x 1)
  mjtNum* efc_AR;            // J*inv(M)*J' + R                                  (nA x 1)

  //-------------------- arena-allocated: POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity/mj_referenceConstraint
  mjtNum* efc_vel;           // velocity in constraint space: J*qvel             (nefc x 1)
  mjtNum* efc_aref;          // reference pseudo-acceleration                    (nefc x 1)

  //-------------------- arena-allocated: POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdConstraint/mj_inverse
  mjtNum* efc_b;             // linear cost term: J*qacc_smooth - aref           (nefc x 1)
  mjtNum* iefc_aref;         // reference pseudo-acceleration                    (nefc x 1)
  int*    iefc_state;        // constraint state (mjtConstraintState)            (nefc x 1)
  mjtNum* iefc_force;        // constraint force in constraint space             (nefc x 1)
  int*    efc_state;         // constraint state (mjtConstraintState)            (nefc x 1)
  mjtNum* efc_force;         // constraint force in constraint space             (nefc x 1)
  mjtNum* ifrc_constraint;   // constraint force                                 (nidof x 1)

  // thread pool pointer
  uintptr_t threadpool;

  // compilation signature
  uint64_t  signature;       // also held by the mjSpec that compiled the model
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
