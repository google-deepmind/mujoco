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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_MUSCLE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_MUSCLE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>

#ifdef __cplusplus
extern "C" {
#endif

// Muscle-tendon actuators with a compliant tendon:
//   Millard2012: Millard et al. (2013), as implemented by OpenSim's Millard2012EquilibriumMuscle
//   Geyer2010:   Geyer and Herr (2010), with the curves of Geyer et al. (2003) as Song and Geyer
//                implement them, a fiber damping and a parallel element of its own
//
// State: act = [fiber, activation], fiber = l_ce/l_opt - 1, so a zeroed act is a fiber at its
// optimal length. The fiber advances by one backward-Euler step of the force equilibrium per
// mj_forward: act_dot[fiber] = (fiber* - fiber)/dt.
//
// Force: gain*act + bias evaluated at the solved fiber, which equals the tendon force -F_max f_T:
//   Millard2012: gain = -F_max cos(phi) f_L f_V,  bias = -F_max cos(phi) (f_P + beta v)
//   Geyer2010:   gain = -F_max f_V f_L,           bias = -F_max (f_V f_PE + beta v - f_BE)

// Millard2012 parameters, by array
typedef enum mjtMillardDyn_ {     // dynprm: activation and fiber dynamics
  mjMILLARD_TAUACT = 0,           // activation time constant (s)
  mjMILLARD_TAUDEACT,             // deactivation time constant (s)
  mjMILLARD_MINACT,               // minimum activation
  mjMILLARD_DAMPING,              // fiber damping, in F_max per normalized fiber velocity
  mjMILLARD_FV_FMAXECC,           // force-velocity: force multiplier at eccentric vmax
  mjMILLARD_FV_CONSLOPE,          // force-velocity: concentric slope near vmax
  mjMILLARD_FV_ISOSLOPE,          // force-velocity: isometric slope
  mjMILLARD_FV_ECCSLOPE,          // force-velocity: eccentric slope near vmax
  mjMILLARD_FV_CONCURV,           // force-velocity: concentric curviness
  mjMILLARD_FV_ECCCURV            // force-velocity: eccentric curviness
} mjtMillardDyn;

typedef enum mjtMillardGain_ {    // gainprm: active fiber
  mjMILLARD_FMAX = 0,             // maximum isometric force (N)
  mjMILLARD_LOPT,                 // optimal fiber length (m)
  mjMILLARD_LSLACK,               // tendon slack length (m)
  mjMILLARD_VMAX,                 // maximum contraction velocity (l_opt/s)
  mjMILLARD_PENNATION,            // pennation angle at optimal fiber length (rad)
  mjMILLARD_AFL_MIN,              // active force-length: minimum active fiber length (l_opt)
  mjMILLARD_AFL_TRANS,            // active force-length: transition fiber length (l_opt)
  mjMILLARD_AFL_MAX,              // active force-length: maximum active fiber length (l_opt)
  mjMILLARD_AFL_SLOPE             // active force-length: shallow ascending slope
} mjtMillardGain;

typedef enum mjtMillardBias_ {    // biasprm: passive elements
  mjMILLARD_PFL_STRAIN0 = 0,      // passive force-length: strain at zero force
  mjMILLARD_PFL_STRAIN1,          // passive force-length: strain at one normalized force
  mjMILLARD_PFL_KLOW,             // passive force-length: stiffness at low force
  mjMILLARD_PFL_KISO,             // passive force-length: stiffness at one normalized force
  mjMILLARD_PFL_CURV,             // passive force-length: curviness
  mjMILLARD_TFL_STRAIN1,          // tendon force-length: strain at one normalized force
  mjMILLARD_TFL_KISO,             // tendon force-length: stiffness at one normalized force
  mjMILLARD_TFL_FTOE,             // tendon force-length: normalized force at toe end
  mjMILLARD_TFL_CURV,             // tendon force-length: curviness
  mjMILLARD_RIGID                 // 1: rigid tendon (OpenSim's ignore_tendon_compliance)
} mjtMillardBias;

// Geyer2010 parameters, by array
typedef enum mjtGeyerDyn_ {       // dynprm: activation and fiber dynamics
  mjGEYER_TAUACT = 0,             // activation time constant (s)
  mjGEYER_TAUDEACT,               // deactivation time constant (s)
  mjGEYER_TAUSMOOTH,              // width of the smooth switch between the time constants
  mjGEYER_DAMPING,                // fiber damping, in F_max per normalized fiber velocity
  mjGEYER_FV_K                    // force-velocity shape factor K
} mjtGeyerDyn;

typedef enum mjtGeyerGain_ {      // gainprm: active fiber
  mjGEYER_FMAX = 0,               // maximum isometric force (N)
  mjGEYER_LOPT,                   // optimal fiber length (m)
  mjGEYER_LSLACK,                 // tendon slack length (m)
  mjGEYER_VMAX,                   // maximum contraction velocity (l_opt/s)
  mjGEYER_AFL_WIDTH,              // active force-length: width w (l_opt)
  mjGEYER_AFL_RESIDUAL            // active force-length: force at |l/l_opt - 1| = w
} mjtGeyerGain;

typedef enum mjtGeyerBias_ {      // biasprm: passive elements
  mjGEYER_PFL_REST = 0,           // parallel element: rest length (l_opt)
  mjGEYER_PFL_STRAIN,             // parallel element: reference strain
  mjGEYER_TFL_STRAIN,             // tendon: reference strain
  mjGEYER_RIGID                   // 1: rigid tendon
} mjtGeyerBias;

// normalized muscle curves
typedef enum mjtMuscleCurve_ {
  mjMUSCLECURVE_ACTIVE = 0,       // active force-length, x = l_ce/l_opt
  mjMUSCLECURVE_PASSIVE,          // passive force-length, x = l_ce/l_opt
  mjMUSCLECURVE_TENDON,           // tendon force-length, x = l_T/l_slack
  mjMUSCLECURVE_VELOCITY,         // force-velocity, x = v_ce/(v_max l_opt)
  mjMUSCLECURVE_BUFFER            // buffer element (Geyer2010 only), x = l_ce/l_opt
} mjtMuscleCurve;

// evaluate a curve of muscle model type (mjtDyn) at x, return value and optionally the slope
MJAPI mjtNum mju_muscleCurve(int type, int curve, mjtNum x, const mjtNum dynprm[mjNDYN],
                             const mjtNum gainprm[mjNGAIN], const mjtNum biasprm[mjNBIAS],
                             mjtNum* dydx);

// muscle activation and fiber velocity: act_dot of both states of actuator i
void mj_muscleActDot(const mjModel* m, mjData* d, int i, mjtNum ctrl);

// muscle gain and bias of actuator i at the fiber solved by mj_muscleActDot
void mj_muscleGainBias(const mjModel* m, const mjData* d, int i, mjtNum* gain, mjtNum* bias);

// d(actuator_force)/d(actuator_velocity) of muscle actuator i
MJAPI mjtNum mj_muscleForceVel(const mjModel* m, const mjData* d, int i);

// d(actuator_force)/d(actuator_length) of muscle actuator i, through the fiber solve
MJAPI mjtNum mj_muscleForceLen(const mjModel* m, const mjData* d, int i);

// set every muscle fiber to its isometric equilibrium at the current actuator lengths
MJAPI void mj_muscleEquilibrium(const mjModel* m, mjData* d);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_MUSCLE_H_
