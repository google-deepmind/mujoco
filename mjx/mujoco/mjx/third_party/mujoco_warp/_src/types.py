# Copyright 2025 The Newton Developers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
import dataclasses
import enum

import mujoco
import warp as wp

MJ_MINVAL = mujoco.mjMINVAL
MJ_MAXVAL = mujoco.mjMAXVAL
MJ_MINIMP = mujoco.mjMINIMP  # minimum constraint impedance
MJ_MAXIMP = mujoco.mjMAXIMP  # maximum constraint impedance
MJ_MAXCONPAIR = mujoco.mjMAXCONPAIR
MJ_MINMU = mujoco.mjMINMU  # minimum friction
# maximum size (by number of edges) of an horizon in EPA algorithm
MJ_MAX_EPAHORIZON = 12
# maximum average number of trianglarfaces EPA can insert at each iteration
MJ_MAX_EPAFACES = 5


# TODO(team): add check that all wp.launch_tiled 'block_dim' settings are configurable
@dataclasses.dataclass
class BlockDim:
  """
  Block dimension 'block_dim' settings for wp.launch_tiled.

  TODO(team): experimental and may be removed
  """

  # collision_driver
  segmented_sort: int = 128
  # derivative
  qderiv_actuator_passive_actuation: int = 64
  qderiv_actuator_passive_no_actuation: int = 256
  # forward
  euler_dense: int = 32
  actuator_velocity: int = 32
  tendon_velocity: int = 32
  # ray
  ray: int = 64
  # sensor
  contact_sort: int = 64
  energy_vel_kinetic: int = 32
  # smooth
  cholesky_factorize: int = 32
  cholesky_solve: int = 32
  cholesky_factorize_solve: int = 32
  # solver
  update_gradient_cholesky: int = 64
  # support
  mul_m_dense: int = 32


class BroadphaseType(enum.IntEnum):
  """Type of broadphase algorithm.

  Attributes:
     NXN: Broad phase checking all pairs
     SAP_TILE: Sweep and prune broad phase using tile sort
     SAP_SEGMENTED: Sweep and prune broad phase using segment sort
  """

  NXN = 0
  SAP_TILE = 1
  SAP_SEGMENTED = 2


class BroadphaseFilter(enum.IntFlag):
  """Bitmask specifying which collision functions to run during broadphase.

  Attributes:
    PLANE: collision between bounding sphere and plane.
    SPHERE: collision between bounding spheres.
    AABB: collision between axis-aligned bounding boxes.
    OBB: collision between oriented bounding boxes.
  """

  PLANE = 1
  SPHERE = 2
  AABB = 4
  OBB = 8


class CamLightType(enum.IntEnum):
  """Type of camera light.

  Attributes:
    FIXED: pos and rot fixed in body
    TRACK: pos tracks body, rot fixed in global
    TRACKCOM: pos tracks subtree com, rot fixed in body
    TARGETBODY: pos fixed in body, rot tracks target body
    TARGETBODYCOM: pos fixed in body, rot tracks target subtree com
  """

  FIXED = mujoco.mjtCamLight.mjCAMLIGHT_FIXED
  TRACK = mujoco.mjtCamLight.mjCAMLIGHT_TRACK
  TRACKCOM = mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM
  TARGETBODY = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
  TARGETBODYCOM = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODYCOM


class DataType(enum.IntFlag):
  """Sensor data types.

  Attributes:
    REAL: real values, no constraints
    POSITIVE: positive values, 0 or negative: inactive
  """

  REAL = mujoco.mjtDataType.mjDATATYPE_REAL
  POSITIVE = mujoco.mjtDataType.mjDATATYPE_POSITIVE
  # unsupported: AXIS, QUATERNION


class DisableBit(enum.IntFlag):
  """Disable default feature bitflags.

  Attributes:
    CONSTRAINT:   entire constraint solver
    EQUALITY:     equality constraints
    FRICTIONLOSS: joint and tendon frictionloss constraints
    LIMIT:        joint and tendon limit constraints
    CONTACT:      contact constraints
    SPRING:       passive spring forces
    DAMPER:       passive damper forces
    GRAVITY:      gravitational forces
    CLAMPCTRL:    clamp control to specified range
    ACTUATION:    apply actuation forces
    REFSAFE:      integrator safety: make ref[0]>=2*timestep
    EULERDAMP:    implicit damping for Euler integration
    FILTERPARENT: disable collisions between parent and child bodies
    SENSOR: sensors
  """

  CONSTRAINT = mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
  EQUALITY = mujoco.mjtDisableBit.mjDSBL_EQUALITY
  FRICTIONLOSS = mujoco.mjtDisableBit.mjDSBL_FRICTIONLOSS
  LIMIT = mujoco.mjtDisableBit.mjDSBL_LIMIT
  CONTACT = mujoco.mjtDisableBit.mjDSBL_CONTACT
  SPRING = mujoco.mjtDisableBit.mjDSBL_SPRING
  DAMPER = mujoco.mjtDisableBit.mjDSBL_DAMPER
  GRAVITY = mujoco.mjtDisableBit.mjDSBL_GRAVITY
  CLAMPCTRL = mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL
  WARMSTART = mujoco.mjtDisableBit.mjDSBL_WARMSTART
  ACTUATION = mujoco.mjtDisableBit.mjDSBL_ACTUATION
  REFSAFE = mujoco.mjtDisableBit.mjDSBL_REFSAFE
  EULERDAMP = mujoco.mjtDisableBit.mjDSBL_EULERDAMP
  FILTERPARENT = mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
  SENSOR = mujoco.mjtDisableBit.mjDSBL_SENSOR
  # unsupported: MIDPHASE


class EnableBit(enum.IntFlag):
  """Enable optional feature bitflags.

  Attributes:
    ENERGY: energy computation
    INVDISCRETE: discrete-time inverse dynamics
  """

  ENERGY = mujoco.mjtEnableBit.mjENBL_ENERGY
  INVDISCRETE = mujoco.mjtEnableBit.mjENBL_INVDISCRETE
  # unsupported: OVERRIDE, FWDINV, MULTICCD, ISLAND


class TrnType(enum.IntEnum):
  """Type of actuator transmission.

  Attributes:
    JOINT: force on joint
    JOINTINPARENT: force on joint, expressed in parent frame
    SLIDERCRANK: force via slider-crank linkage
    TENDON: force on tendon
    BODY: adhesion force on body's geoms
    SITE: force on site
  """

  JOINT = mujoco.mjtTrn.mjTRN_JOINT
  JOINTINPARENT = mujoco.mjtTrn.mjTRN_JOINTINPARENT
  SLIDERCRANK = mujoco.mjtTrn.mjTRN_SLIDERCRANK
  TENDON = mujoco.mjtTrn.mjTRN_TENDON
  BODY = mujoco.mjtTrn.mjTRN_BODY
  SITE = mujoco.mjtTrn.mjTRN_SITE


class DynType(enum.IntEnum):
  """Type of actuator dynamics.

  Attributes:
    NONE: no internal dynamics; ctrl specifies force
    INTEGRATOR: integrator: da/dt = u
    FILTER: linear filter: da/dt = (u-a) / tau
    FILTEREXACT: linear filter: da/dt = (u-a) / tau, with exact integration
    MUSCLE: piece-wise linear filter with two time constants
  """

  NONE = mujoco.mjtDyn.mjDYN_NONE
  INTEGRATOR = mujoco.mjtDyn.mjDYN_INTEGRATOR
  FILTER = mujoco.mjtDyn.mjDYN_FILTER
  FILTEREXACT = mujoco.mjtDyn.mjDYN_FILTEREXACT
  MUSCLE = mujoco.mjtDyn.mjDYN_MUSCLE
  # unsupported: USER


class GainType(enum.IntEnum):
  """Type of actuator gain.

  Attributes:
    FIXED: fixed gain
    AFFINE: const + kp*length + kv*velocity
    MUSCLE: muscle FLV curve computed by muscle_gain
  """

  FIXED = mujoco.mjtGain.mjGAIN_FIXED
  AFFINE = mujoco.mjtGain.mjGAIN_AFFINE
  MUSCLE = mujoco.mjtGain.mjGAIN_MUSCLE
  # unsupported: USER


class BiasType(enum.IntEnum):
  """Type of actuator bias.

  Attributes:
    NONE: no bias
    AFFINE: const + kp*length + kv*velocity
    MUSCLE: muscle passive force computed by muscle_bias
  """

  NONE = mujoco.mjtBias.mjBIAS_NONE
  AFFINE = mujoco.mjtBias.mjBIAS_AFFINE
  MUSCLE = mujoco.mjtBias.mjBIAS_MUSCLE
  # unsupported: USER


class JointType(enum.IntEnum):
  """Type of degree of freedom.

  Attributes:
    FREE:  global position and orientation (quat)       (7,)
    BALL:  orientation (quat) relative to parent        (4,)
    SLIDE: sliding distance along body-fixed axis       (1,)
    HINGE: rotation angle (rad) around body-fixed axis  (1,)
  """

  FREE = mujoco.mjtJoint.mjJNT_FREE
  BALL = mujoco.mjtJoint.mjJNT_BALL
  SLIDE = mujoco.mjtJoint.mjJNT_SLIDE
  HINGE = mujoco.mjtJoint.mjJNT_HINGE


class ConeType(enum.IntEnum):
  """Type of friction cone.

  Attributes:
    PYRAMIDAL: pyramidal
    ELLIPTIC: elliptic
  """

  PYRAMIDAL = mujoco.mjtCone.mjCONE_PYRAMIDAL
  ELLIPTIC = mujoco.mjtCone.mjCONE_ELLIPTIC


class IntegratorType(enum.IntEnum):
  """Integrator mode.

  Attributes:
    EULER: semi-implicit Euler
    RK4: 4th-order Runge Kutta
    IMPLICITFAST: implicit in velocity, no rne derivative
  """

  EULER = mujoco.mjtIntegrator.mjINT_EULER
  RK4 = mujoco.mjtIntegrator.mjINT_RK4
  IMPLICITFAST = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
  # unsupported: IMPLICIT


class GeomType(enum.IntEnum):
  """Type of geometry.

  Attributes:
    PLANE: plane
    HFIELD: heightfield
    SPHERE: sphere
    CAPSULE: capsule
    ELLIPSOID: ellipsoid
    CYLINDER: cylinder
    BOX: box
    MESH: mesh
    SDF: sdf
  """

  PLANE = mujoco.mjtGeom.mjGEOM_PLANE
  HFIELD = mujoco.mjtGeom.mjGEOM_HFIELD
  SPHERE = mujoco.mjtGeom.mjGEOM_SPHERE
  CAPSULE = mujoco.mjtGeom.mjGEOM_CAPSULE
  ELLIPSOID = mujoco.mjtGeom.mjGEOM_ELLIPSOID
  CYLINDER = mujoco.mjtGeom.mjGEOM_CYLINDER
  BOX = mujoco.mjtGeom.mjGEOM_BOX
  MESH = mujoco.mjtGeom.mjGEOM_MESH
  SDF = mujoco.mjtGeom.mjGEOM_SDF
  # unsupported: NGEOMTYPES, ARROW*, LINE, SKIN, LABEL, NONE


class SolverType(enum.IntEnum):
  """Constraint solver algorithm.

  Attributes:
    CG: Conjugate gradient (primal)
    NEWTON: Newton (primal)
  """

  CG = mujoco.mjtSolver.mjSOL_CG
  NEWTON = mujoco.mjtSolver.mjSOL_NEWTON
  # unsupported: PGS


class ConstraintState(enum.IntEnum):
  """State of constraint.

  Attributes:
    SATISFIED: constraint satisfied, zero cost (limit, contact)
    QUADRATIC: quadratic cost (equality, friction, limit, contact)
    LINEARNEG: linear cost, negative side (friction)
    LINEARPOS: linear cost, positive side (friction)
    CONE: square distance to cone cost (elliptic contact)
  """

  SATISFIED = mujoco.mjtConstraintState.mjCNSTRSTATE_SATISFIED
  QUADRATIC = mujoco.mjtConstraintState.mjCNSTRSTATE_QUADRATIC
  LINEARNEG = mujoco.mjtConstraintState.mjCNSTRSTATE_LINEARNEG
  LINEARPOS = mujoco.mjtConstraintState.mjCNSTRSTATE_LINEARPOS
  CONE = mujoco.mjtConstraintState.mjCNSTRSTATE_CONE


class ConstraintType(enum.IntEnum):
  """Type of constraint.

  Attributes:
    EQUALITY: equality constraint
    FRICTION_DOF: dof friction
    FRICTION_TENDON: tendon friction
    LIMIT_JOINT: joint limit
    LIMIT_TENDON: tendon limit
    CONTACT_FRICTIONLESS: frictionless contact
    CONTACT_PYRAMIDAL: frictional contact, pyramidal friction cone
    CONTACT_ELLIPTIC: frictional contact, elliptic friction cone
  """

  EQUALITY = mujoco.mjtConstraint.mjCNSTR_EQUALITY
  FRICTION_DOF = mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF
  FRICTION_TENDON = mujoco.mjtConstraint.mjCNSTR_FRICTION_TENDON
  LIMIT_JOINT = mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT
  LIMIT_TENDON = mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON
  CONTACT_FRICTIONLESS = mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS
  CONTACT_PYRAMIDAL = mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL
  CONTACT_ELLIPTIC = mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC


class SensorType(enum.IntEnum):
  """Type of sensor.

  Attributes:
    MAGNETOMETER: magnetometer
    CAMPROJECTION: camera projection
    RANGEFINDER: scalar distance to nearest geom or site along z-axis
    JOINTPOS: joint position
    TENDONPOS: scalar tendon position
    ACTUATORPOS: actuator position
    BALLQUAT: ball joint orientation
    JOINTLIMITPOS: joint limit distance-margin
    TENDONLIMITPOS: tendon limit distance-margin
    FRAMEPOS: frame position
    FRAMEXAXIS: frame x-axis
    FRAMEYAXIS: frame y-axis
    FRAMEZAXIS: frame z-axis
    FRAMEQUAT: frame orientation, represented as quaternion
    SUBTREECOM: subtree center of mass
    GEOMDIST: signed distance between two geoms
    GEOMNORMAL: normal direction between two geoms
    GEOMFROMTO: segment between two geoms
    INSIDESITE: 1 if object is inside site, 0 otherwise
    E_POTENTIAL: potential energy
    E_KINETIC: kinetic energy
    CLOCK: simulation time
    VELOCIMETER: 3D linear velocity, in local frame
    GYRO: 3D angular velocity, in local frame
    JOINTVEL: joint velocity
    TENDONVEL: scalar tendon velocity
    ACTUATORVEL: actuator velocity
    BALLANGVEL: ball joint angular velocity
    JOINTLIMITVEL: joint limit velocity
    TENDONLIMITVEL: tendon limit velocity
    FRAMELINVEL: 3D linear velocity
    FRAMEANGVEL: 3D angular velocity
    SUBTREELINVEL: subtree linear velocity
    SUBTREEANGMOM: subtree angular momentum
    TOUCH: scalar contact normal forces summed over sensor zone
    CONTACT: contacts which occurred during the simulation
    ACCELEROMETER: accelerometer
    FORCE: force
    TORQUE: torque
    ACTUATORFRC: scalar actuator force, measured at the joint
    TENDONACTFRC: scalar actuator force, measured at the tendon
    JOINTACTFRC: scalar actuator force, measured at the joint
    JOINTLIMITFRC: joint limit force
    TENDONLIMITFRC: tendon limit force
    FRAMELINACC: 3D linear acceleration
    FRAMEANGACC: 3D angular acceleration
    TACTILE: tactile sensor
  """

  MAGNETOMETER = mujoco.mjtSensor.mjSENS_MAGNETOMETER
  CAMPROJECTION = mujoco.mjtSensor.mjSENS_CAMPROJECTION
  RANGEFINDER = mujoco.mjtSensor.mjSENS_RANGEFINDER
  JOINTPOS = mujoco.mjtSensor.mjSENS_JOINTPOS
  TENDONPOS = mujoco.mjtSensor.mjSENS_TENDONPOS
  ACTUATORPOS = mujoco.mjtSensor.mjSENS_ACTUATORPOS
  BALLQUAT = mujoco.mjtSensor.mjSENS_BALLQUAT
  JOINTLIMITPOS = mujoco.mjtSensor.mjSENS_JOINTLIMITPOS
  TENDONLIMITPOS = mujoco.mjtSensor.mjSENS_TENDONLIMITPOS
  FRAMEPOS = mujoco.mjtSensor.mjSENS_FRAMEPOS
  FRAMEXAXIS = mujoco.mjtSensor.mjSENS_FRAMEXAXIS
  FRAMEYAXIS = mujoco.mjtSensor.mjSENS_FRAMEYAXIS
  FRAMEZAXIS = mujoco.mjtSensor.mjSENS_FRAMEZAXIS
  FRAMEQUAT = mujoco.mjtSensor.mjSENS_FRAMEQUAT
  SUBTREECOM = mujoco.mjtSensor.mjSENS_SUBTREECOM
  GEOMDIST = mujoco.mjtSensor.mjSENS_GEOMDIST
  GEOMNORMAL = mujoco.mjtSensor.mjSENS_GEOMNORMAL
  GEOMFROMTO = mujoco.mjtSensor.mjSENS_GEOMFROMTO
  INSIDESITE = mujoco.mjtSensor.mjSENS_INSIDESITE
  E_POTENTIAL = mujoco.mjtSensor.mjSENS_E_POTENTIAL
  E_KINETIC = mujoco.mjtSensor.mjSENS_E_KINETIC
  CLOCK = mujoco.mjtSensor.mjSENS_CLOCK
  VELOCIMETER = mujoco.mjtSensor.mjSENS_VELOCIMETER
  GYRO = mujoco.mjtSensor.mjSENS_GYRO
  JOINTVEL = mujoco.mjtSensor.mjSENS_JOINTVEL
  TENDONVEL = mujoco.mjtSensor.mjSENS_TENDONVEL
  ACTUATORVEL = mujoco.mjtSensor.mjSENS_ACTUATORVEL
  BALLANGVEL = mujoco.mjtSensor.mjSENS_BALLANGVEL
  JOINTLIMITVEL = mujoco.mjtSensor.mjSENS_JOINTLIMITVEL
  TENDONLIMITVEL = mujoco.mjtSensor.mjSENS_TENDONLIMITVEL
  FRAMELINVEL = mujoco.mjtSensor.mjSENS_FRAMELINVEL
  FRAMEANGVEL = mujoco.mjtSensor.mjSENS_FRAMEANGVEL
  SUBTREELINVEL = mujoco.mjtSensor.mjSENS_SUBTREELINVEL
  SUBTREEANGMOM = mujoco.mjtSensor.mjSENS_SUBTREEANGMOM
  TOUCH = mujoco.mjtSensor.mjSENS_TOUCH
  CONTACT = mujoco.mjtSensor.mjSENS_CONTACT
  ACCELEROMETER = mujoco.mjtSensor.mjSENS_ACCELEROMETER
  FORCE = mujoco.mjtSensor.mjSENS_FORCE
  TORQUE = mujoco.mjtSensor.mjSENS_TORQUE
  ACTUATORFRC = mujoco.mjtSensor.mjSENS_ACTUATORFRC
  TENDONACTFRC = mujoco.mjtSensor.mjSENS_TENDONACTFRC
  JOINTACTFRC = mujoco.mjtSensor.mjSENS_JOINTACTFRC
  JOINTLIMITFRC = mujoco.mjtSensor.mjSENS_JOINTLIMITFRC
  TENDONLIMITFRC = mujoco.mjtSensor.mjSENS_TENDONLIMITFRC
  FRAMELINACC = mujoco.mjtSensor.mjSENS_FRAMELINACC
  FRAMEANGACC = mujoco.mjtSensor.mjSENS_FRAMEANGACC
  TACTILE = mujoco.mjtSensor.mjSENS_TACTILE


class ObjType(enum.IntEnum):
  """Type of object.

  Attributes:
    UNKNOWN: unknown object type
    BODY: body
    XBODY: body, used to access regular frame instead of i-frame
    GEOM: geom
    SITE: site
    CAMERA: camera
  """

  UNKNOWN = mujoco.mjtObj.mjOBJ_UNKNOWN
  BODY = mujoco.mjtObj.mjOBJ_BODY
  XBODY = mujoco.mjtObj.mjOBJ_XBODY
  GEOM = mujoco.mjtObj.mjOBJ_GEOM
  SITE = mujoco.mjtObj.mjOBJ_SITE
  CAMERA = mujoco.mjtObj.mjOBJ_CAMERA


class EqType(enum.IntEnum):
  """Type of equality constraint.

  Attributes:
    CONNECT: connect two bodies at a point (ball joint)
    JOINT: couple the values of two scalar joints with cubic
    WELD: fix relative position and orientation of two bodies
    TENDON: couple the lengths of two tendons with cubic
  """

  CONNECT = mujoco.mjtEq.mjEQ_CONNECT
  WELD = mujoco.mjtEq.mjEQ_WELD
  JOINT = mujoco.mjtEq.mjEQ_JOINT
  TENDON = mujoco.mjtEq.mjEQ_TENDON
  # unsupported: FLEX, DISTANCE


class WrapType(enum.IntEnum):
  """Type of tendon wrapping object.

  Attributes:
    JOINT: constant moment arm
    PULLEY: pulley used to split tendon
    SITE: pass through site
    SPHERE: wrap around sphere
    CYLINDER: wrap around (infinite) cylinder
  """

  JOINT = mujoco.mjtWrap.mjWRAP_JOINT
  PULLEY = mujoco.mjtWrap.mjWRAP_PULLEY
  SITE = mujoco.mjtWrap.mjWRAP_SITE
  SPHERE = mujoco.mjtWrap.mjWRAP_SPHERE
  CYLINDER = mujoco.mjtWrap.mjWRAP_CYLINDER


class vec5f(wp.types.vector(length=5, dtype=float)):
  pass


class vec6f(wp.types.vector(length=6, dtype=float)):
  pass


class vec8f(wp.types.vector(length=8, dtype=float)):
  pass


class vec8i(wp.types.vector(length=8, dtype=int)):
  pass


class vec10f(wp.types.vector(length=10, dtype=float)):
  pass


class vec11f(wp.types.vector(length=11, dtype=float)):
  pass


vec5 = vec5f
vec6 = vec6f
vec10 = vec10f
vec11 = vec11f


@dataclasses.dataclass
class Option:
  """Physics options.

  Attributes:
    timestep: simulation timestep
    impratio: ratio of friction-to-normal contact impedance
    tolerance: main solver tolerance
    ls_tolerance: CG/Newton linesearch tolerance
    ccd_tolerance: convex collision detection tolerance
    gravity: gravitational acceleration
    magnetic: global magnetic flux
    integrator: integration mode (IntegratorType)
    cone: type of friction cone (ConeType)
    solver: solver algorithm (SolverType)
    iterations: number of main solver iterations
    ls_iterations: maximum number of CG/Newton linesearch iterations
    disableflags: bit flags for disabling standard features
    enableflags: bit flags for enabling optional features
    is_sparse: whether to use sparse representations
    ccd_iterations: number of iterations in convex collision detection
    ls_parallel: evaluate engine solver step sizes in parallel
    ls_parallel_min_step: minimum step size for solver linesearch
    wind: wind (for lift, drag, and viscosity)
    has_fluid: True if wind, density, or viscosity are non-zero at put_model time
    density: density of medium
    viscosity: viscosity of medium
    broadphase: broadphase type (BroadphaseType)
    broadphase_filter: broadphase filter bitflag (BroadphaseFilter)
    graph_conditional: flag to use cuda graph conditional, should be False when JAX is used
    sdf_initpoints: number of starting points for gradient descent
    sdf_iterations: max number of iterations for gradient descent
    run_collision_detection: if False, skips collision detection and allows user-populated
      contacts during the physics step (as opposed to DisableBit.CONTACT which explicitly
      zeros out the contacts at each step)
    legacy_gjk: run legacy gjk algorithm
    contact_sensor_maxmatch: max number of contacts considered by contact sensor matching criteria
                             contacts matched after this value is exceded will be ignored
  """

  timestep: wp.array(dtype=float)
  impratio: wp.array(dtype=float)
  tolerance: wp.array(dtype=float)
  ls_tolerance: wp.array(dtype=float)
  ccd_tolerance: wp.array(dtype=float)
  gravity: wp.array(dtype=wp.vec3)
  magnetic: wp.array(dtype=wp.vec3)
  integrator: int
  cone: int
  solver: int
  iterations: int
  ls_iterations: int
  disableflags: int
  enableflags: int
  is_sparse: bool
  ccd_iterations: int
  ls_parallel: bool  # warp only
  ls_parallel_min_step: float  # warp only
  wind: wp.array(dtype=wp.vec3)
  has_fluid: bool
  density: wp.array(dtype=float)
  viscosity: wp.array(dtype=float)
  broadphase: int  # warp only
  broadphase_filter: int  # warp only
  graph_conditional: bool  # warp only
  sdf_initpoints: int
  sdf_iterations: int
  run_collision_detection: bool  # warp only
  legacy_gjk: bool
  contact_sensor_maxmatch: int  # warp only


@dataclasses.dataclass
class Statistic:
  """Model statistics (in qpos0).

  Attributes:
    meaninertia: mean diagonal inertia
  """

  meaninertia: float


@dataclasses.dataclass
class Constraint:
  """Constraint data.

  Attributes:
    type: constraint type (ConstraintType)            (nworld, njmax)
    id: id of object of specific type                 (nworld, njmax)
    J: constraint Jacobian                            (nworld, njmax, nv)
    pos: constraint position (equality, contact)      (nworld, njmax)
    margin: inclusion margin (contact)                (nworld, njmax)
    D: constraint mass                                (nworld, njmax)
    vel: velocity in constraint space: J*qvel         (nworld, njmax)
    aref: reference pseudo-acceleration               (nworld, njmax)
    frictionloss: frictionloss (friction)             (nworld, njmax)
    force: constraint force in constraint space       (nworld, njmax)
    Jaref: Jac*qacc - aref                            (nworld, njmax)
    Ma: M*qacc                                        (nworld, nv)
    grad: gradient of master cost                     (nworld, nv)
    grad_dot: dot(grad, grad)                         (nworld,)
    Mgrad: M / grad                                   (nworld, nv)
    search: linesearch vector                         (nworld, nv)
    search_dot: dot(search, search)                   (nworld,)
    gauss: gauss Cost                                 (nworld,)
    cost: constraint + Gauss cost                     (nworld,)
    prev_cost: cost from previous iter                (nworld,)
    state: constraint state                           (nworld, njmax)
    mv: qM @ search                                   (nworld, nv)
    jv: efc_J @ search                                (nworld, njmax)
    quad: quadratic cost coefficients                 (nworld, njmax, 3)
    quad_gauss: quadratic cost gauss coefficients     (nworld, 3)
    h: cone hessian                                   (nworld, nv, nv)
    alpha: line search step size                      (nworld,)
    prev_grad: previous grad                          (nworld, nv)
    prev_Mgrad: previous Mgrad                        (nworld, nv)
    beta: polak-ribiere beta                          (nworld,)
    done: solver done                                 (nworld,)
    cost_candidate: costs associated with step sizes  (nworld, nlsp)
  """

  type: wp.array2d(dtype=int)
  id: wp.array2d(dtype=int)
  J: wp.array3d(dtype=float)
  pos: wp.array2d(dtype=float)
  margin: wp.array2d(dtype=float)
  D: wp.array2d(dtype=float)
  vel: wp.array2d(dtype=float)
  aref: wp.array2d(dtype=float)
  frictionloss: wp.array2d(dtype=float)
  force: wp.array2d(dtype=float)
  Jaref: wp.array2d(dtype=float)
  Ma: wp.array2d(dtype=float)
  grad: wp.array2d(dtype=float)
  cholesky_L_tmp: wp.array3d(dtype=float)
  cholesky_y_tmp: wp.array2d(dtype=float)
  grad_dot: wp.array(dtype=float)
  Mgrad: wp.array2d(dtype=float)
  search: wp.array2d(dtype=float)
  search_dot: wp.array(dtype=float)
  gauss: wp.array(dtype=float)
  cost: wp.array(dtype=float)
  prev_cost: wp.array(dtype=float)
  state: wp.array2d(dtype=int)
  mv: wp.array2d(dtype=float)
  jv: wp.array2d(dtype=float)
  quad: wp.array2d(dtype=wp.vec3)
  quad_gauss: wp.array(dtype=wp.vec3)
  h: wp.array3d(dtype=float)
  alpha: wp.array(dtype=float)
  prev_grad: wp.array2d(dtype=float)
  prev_Mgrad: wp.array2d(dtype=float)
  beta: wp.array(dtype=float)
  done: wp.array(dtype=bool)
  # linesearch
  cost_candidate: wp.array2d(dtype=float)


@dataclasses.dataclass
class TileSet:
  """Tiling configuration for decomposable block diagonal matrix.

  For non-square, non-block-diagonal tiles, use two tilesets.

  Attributes:
    adr: address of each tile in the set
    size: size of all the tiles in this set
  """

  adr: wp.array(dtype=int)
  size: int


# TODO(team): make Model/Data fields sort order match mujoco


@dataclasses.dataclass
class Model:
  """Model definition and parameters.

  Attributes:
    nq: number of generalized coordinates
    nv: number of degrees of freedom
    nu: number of actuators/controls
    na: number of activation states
    nbody: number of bodies
    njnt: number of joints
    ngeom: number of geoms
    nsite: number of sites
    ncam: number of cameras
    nlight: number of lights
    nmat: number of materials
    nexclude: number of excluded geom pairs
    neq: number of equality constraints
    nmocap: number of mocap bodies
    ngravcomp: number of bodies with nonzero gravcomp
    nM: number of non-zeros in sparse inertia matrix
    nC: number of non-zeros in sparse reduced dof-dof matrix
    ntendon: number of tendons
    nwrap: number of wrap objects in all tendon paths
    nsensor: number of sensors
    nsensordata: number of elements in sensor data vector
    nsensortaxel: number of taxels in all tactile sensors
    nmeshvert: number of vertices for all meshes
    nmeshface: number of faces for all meshes
    nmeshgraph: number of ints in mesh auxiliary data
    nmeshpoly: number of polygons in all meshes
    nmeshpolyvert: number of vertices in all polygons
    nmeshpolymap: number of polygons in vertex map
    nlsp: number of step sizes for parallel linsearch
    npair: number of predefined geom pairs
    nhfield: number of heightfields
    nhfielddata: size of elevation data
    opt: physics options
    stat: model statistics
    qpos0: qpos values at default pose                       (nworld, nq)
    qpos_spring: reference pose for springs                  (nworld, nq)
    qM_fullm_i: sparse mass matrix addressing
    qM_fullm_j: sparse mass matrix addressing
    qM_mulm_i: sparse mass matrix addressing
    qM_mulm_j: sparse mass matrix addressing
    qM_madr_ij: sparse mass matrix addressing
    M_rownnz: number of non-zeros in each row of qM          (nv,)
    M_rowadr: index of each row in qM                        (nv,)
    M_colind: column indices of non-zeros in qM              (nM,)
    mapM2M: index mapping from M (legacy) to M (CSR)         (nC)
    qM_tiles: tiling configuration
    body_tree: list of body ids by tree level
    body_parentid: id of body's parent                       (nbody,)
    body_rootid: id of root above body                       (nbody,)
    body_weldid: id of body that this body is welded to      (nbody,)
    body_mocapid: id of mocap data; -1: none                 (nbody,)
    body_jntnum: number of joints for this body              (nbody,)
    body_jntadr: start addr of joints; -1: no joints         (nbody,)
    body_dofnum: number of motion degrees of freedom         (nbody,)
    body_dofadr: start addr of dofs; -1: no dofs             (nbody,)
    body_geomnum: number of geoms                            (nbody,)
    body_geomadr: start addr of geoms; -1: no geoms          (nbody,)
    body_pos: position offset rel. to parent body            (nworld, nbody, 3)
    body_quat: orientation offset rel. to parent body        (nworld, nbody, 4)
    body_ipos: local position of center of mass              (nworld, nbody, 3)
    body_iquat: local orientation of inertia ellipsoid       (nworld, nbody, 4)
    body_mass: mass                                          (nworld, nbody,)
    body_subtreemass: mass of subtree starting at this body  (nworld, nbody,)
    subtree_mass: mass of subtree                            (nworld, nbody,)
    body_inertia: diagonal inertia in ipos/iquat frame       (nworld, nbody, 3)
    body_invweight0: mean inv inert in qpos0 (trn, rot)      (nworld, nbody, 2)
    body_contype: OR over all geom contypes                  (nbody,)
    body_conaffinity: OR over all geom conaffinities         (nbody,)
    body_gravcomp: antigravity force, units of body weight   (nworld, nbody)
    jnt_type: type of joint (JointType)                      (njnt,)
    jnt_qposadr: start addr in 'qpos' for joint's data       (njnt,)
    jnt_dofadr: start addr in 'qvel' for joint's data        (njnt,)
    jnt_bodyid: id of joint's body                           (njnt,)
    jnt_limited: does joint have limits                      (njnt,)
    jnt_actfrclimited: does joint have actuator force limits (njnt,)
    jnt_solref: constraint solver reference: limit           (nworld, njnt, mjNREF)
    jnt_solimp: constraint solver impedance: limit           (nworld, njnt, mjNIMP)
    jnt_pos: local anchor position                           (nworld, njnt, 3)
    jnt_axis: local joint axis                               (nworld, njnt, 3)
    jnt_stiffness: stiffness coefficient                     (nworld, njnt)
    jnt_range: joint limits                                  (nworld, njnt, 2)
    jnt_actfrcrange: range of total actuator force           (nworld, njnt, 2)
    jnt_margin: min distance for limit detection             (nworld, njnt)
    jnt_limited_slide_hinge_adr: limited/slide/hinge jntadr
    jnt_limited_ball_adr: limited/ball jntadr
    jnt_actgravcomp: is gravcomp force applied via actuators (njnt,)
    dof_bodyid: id of dof's body                             (nv,)
    dof_jntid: id of dof's joint                             (nv,)
    dof_parentid: id of dof's parent; -1: none               (nv,)
    dof_Madr: dof address in M-diagonal                      (nv,)
    dof_armature: dof armature inertia/mass                  (nworld, nv)
    dof_damping: damping coefficient                         (nworld, nv)
    dof_invweight0: diag. inverse inertia in qpos0           (nworld, nv)
    dof_frictionloss: dof friction loss                      (nworld, nv)
    dof_solimp: constraint solver impedance: frictionloss    (nworld, nv, NIMP)
    dof_solref: constraint solver reference: frictionloss    (nworld, nv, NREF)
    dof_tri_row: np.tril_indices                             (mjm.nv)[0]
    dof_tri_col: np.tril_indices                             (mjm.nv)[1]
    geom_type: geometric type (GeomType)                     (ngeom,)
    geom_contype: geom contact type                          (ngeom,)
    geom_conaffinity: geom contact affinity                  (ngeom,)
    geom_condim: contact dimensionality (1, 3, 4, 6)         (ngeom,)
    geom_bodyid: id of geom's body                           (ngeom,)
    geom_dataid: id of geom's mesh/hfield; -1: none          (ngeom,)
    geom_group: geom group inclusion/exclusion mask          (ngeom,)
    geom_matid: material id for rendering                    (nworld, ngeom,)
    geom_priority: geom contact priority                     (ngeom,)
    geom_solmix: mixing coef for solref/imp in geom pair     (nworld, ngeom,)
    geom_solref: constraint solver reference: contact        (nworld, ngeom, mjNREF)
    geom_solimp: constraint solver impedance: contact        (nworld, ngeom, mjNIMP)
    geom_size: geom-specific size parameters                 (ngeom, 3)
    geom_aabb: bounding box, (center, size)                  (ngeom, 6)
    geom_rbound: radius of bounding sphere                   (nworld, ngeom,)
    geom_pos: local position offset rel. to body             (nworld, ngeom, 3)
    geom_quat: local orientation offset rel. to body         (nworld, ngeom, 4)
    geom_friction: friction for (slide, spin, roll)          (nworld, ngeom, 3)
    geom_margin: detect contact if dist<margin               (nworld, ngeom,)
    geom_gap: include in solver if dist<margin-gap           (nworld, ngeom,)
    geom_rgba: rgba when material is omitted                 (nworld, ngeom, 4)
    hfield_adr: start address in hfield_data                 (nhfield,)
    hfield_nrow: number of rows in grid                      (nhfield,)
    hfield_ncol: number of columns in grid                   (nhfield,)
    hfield_size: (x, y, z_top, z_bottom)                     (nhfield, 4)
    hfield_data: elevation data                              (nhfielddata,)
    site_type: geom type for rendering (GeomType)            (nsite,)
    site_bodyid: id of site's body                           (nsite,)
    site_pos: local position offset rel. to body             (nworld, nsite, 3)
    site_quat: local orientation offset rel. to body         (nworld, nsite, 4)
    cam_mode: camera tracking mode (CamLightType)            (ncam,)
    cam_bodyid: id of camera's body                          (ncam,)
    cam_targetbodyid: id of targeted body; -1: none          (ncam,)
    cam_pos: position rel. to body frame                     (nworld, ncam, 3)
    cam_quat: orientation rel. to body frame                 (nworld, ncam, 4)
    cam_poscom0: global position rel. to sub-com in qpos0    (nworld, ncam, 3)
    cam_pos0: global position rel. to body in qpos0          (nworld, ncam, 3)
    cam_mat0: global orientation in qpos0                    (nworld, ncam, 3, 3)
    cam_fovy: y field-of-view (ortho ? len : deg)            (ncam,)
    cam_resolution: resolution: pixels [width, height]       (ncam, 2)
    cam_sensorsize: sensor size: length [width, height]      (ncam, 2)
    cam_intrinsic: [focal length; principal point]           (ncam, 4)
    light_mode: light tracking mode (CamLightType)           (nlight,)
    light_bodyid: id of light's body                         (nlight,)
    light_targetbodyid: id of targeted body; -1: none        (nlight,)
    light_type: spot, directional, etc. (mjtLightType)       (nworld, nlight)
    light_castshadow: does light cast shadows                (nworld, nlight)
    light_active: is light active                            (nworld, nlight)
    light_pos: position rel. to body frame                   (nworld, nlight, 3)
    light_dir: direction rel. to body frame                  (nworld, nlight, 3)
    light_poscom0: global position rel. to sub-com in qpos0  (nworld, nlight, 3)
    light_pos0: global position rel. to body in qpos0        (nworld, nlight, 3)
    light_dir0: global direction in qpos0                    (nlight, 3)
    mesh_vertadr: first vertex address                       (nmesh,)
    mesh_vertnum: number of vertices                         (nmesh,)
    mesh_vert: vertex positions for all meshes               (nmeshvert, 3)
    mesh_faceadr: first face address                         (nmesh,)
    mesh_face: face indices for all meshes                   (nface, 3)
    mesh_normaladr: first normal address                     (nmesh,)
    mesh_normal: normals for all meshes                      (nmeshnormal, 3)
    mesh_graphadr: graph data address; -1: no graph          (nmesh,)
    mesh_graph: convex graph data                            (nmeshgraph,)
    mesh_quat: rotation applied to asset vertices            (nmesh, 4)
    mesh_polynum: number of polygons per mesh                (nmesh,)
    mesh_polyadr: first polygon address per mesh             (nmesh,)
    mesh_polynormal: all polygon normals                     (nmeshpoly, 3)
    mesh_polyvertadr: polygon vertex start address           (nmeshpoly,)
    mesh_polyvertnum: number of vertices per polygon         (nmeshpoly,)
    mesh_polyvert: all polygon vertices                      (nmeshpolyvert,)
    mesh_polymapadr: first polygon address per vertex        (nmeshvert,)
    mesh_polymapnum: number of polygons per vertex           (nmeshvert,)
    mesh_polymap: vertex to polygon map                      (nmeshpolymap,)
    oct_aabb: octree axis-aligned bounding boxes             (noct, 6)
    oct_child: octree children                               (noct, 8)
    oct_coeff: octree interpolation coefficients             (noct, 8)
    eq_type: constraint type (EqType)                        (neq,)
    eq_obj1id: id of object 1                                (neq,)
    eq_obj2id: id of object 2                                (neq,)
    eq_objtype: type of both objects (ObjType)               (neq,)
    eq_active0: initial enable/disable constraint state      (neq,)
    eq_solref: constraint solver reference                   (nworld, neq, mjNREF)
    eq_solimp: constraint solver impedance                   (nworld, neq, mjNIMP)
    eq_data: numeric data for constraint                     (nworld, neq, mjNEQDATA)
    eq_connect_adr: eq_* addresses of type `CONNECT`
    eq_wld_adr: eq_* addresses of type `WELD`
    eq_jnt_adr: eq_* addresses of type `JOINT`
    eq_ten_adr: eq_* addresses of type `TENDON`              (<=neq,)
    actuator_moment_tiles_nv: tiling configuration
    actuator_moment_tiles_nu: tiling configuration
    actuator_trntype: transmission type (TrnType)            (nu,)
    actuator_dyntype: dynamics type (DynType)                (nu,)
    actuator_gaintype: gain type (GainType)                  (nu,)
    actuator_biastype: bias type (BiasType)                  (nu,)
    actuator_trnid: transmission id: joint, tendon, site     (nu, 2)
    actuator_actadr: first activation address; -1: stateless (nu,)
    actuator_actnum: number of activation variables          (nu,)
    actuator_ctrllimited: is control limited                 (nu,)
    actuator_forcelimited: is force limited                  (nu,)
    actuator_actlimited: is activation limited               (nu,)
    actuator_dynprm: dynamics parameters                     (nworld, nu, mjNDYN)
    actuator_gainprm: gain parameters                        (nworld, nu, mjNGAIN)
    actuator_biasprm: bias parameters                        (nworld, nu, mjNBIAS)
    actuator_actearly: step activation before force          (nu,)
    actuator_ctrlrange: range of controls                    (nworld, nu, 2)
    actuator_forcerange: range of forces                     (nworld, nu, 2)
    actuator_actrange: range of activations                  (nworld, nu, 2)
    actuator_gear: scale length and transmitted force        (nworld, nu, 6)
    actuator_cranklength: crank length for slider-crank      (nu,)
    actuator_acc0: acceleration from unit force in qpos0     (nu,)
    actuator_lengthrange: feasible actuator length range     (nu, 2)
    nxn_geom_pair: collision pair geom ids [-2, ngeom-1]     (<= ngeom * (ngeom - 1) // 2,)
    nxn_geom_pair_filtered: valid collision pair geom ids    (<= ngeom * (ngeom - 1) // 2,)
                            [-1, ngeom - 1]
    nxn_pairid: predefined pair id, -1 if not predefined,    (<= ngeom * (ngeom - 1) // 2,)
                -2 if skipped
    nxn_pairid_filtered: predefined pair id, -1 if not       (<= ngeom * (ngeom - 1) // 2,)
                         predefined
    pair_dim: contact dimensionality                         (npair,)
    pair_geom1: id of geom1                                  (npair,)
    pair_geom2: id of geom2                                  (npair,)
    pair_solref: solver reference: contact normal            (nworld, npair, mjNREF)
    pair_solreffriction: solver reference: contact friction  (nworld, npair, mjNREF)
    pair_solimp: solver impedance: contact                   (nworld, npair, mjNIMP)
    pair_margin: detect contact if dist<margin               (nworld, npair,)
    pair_gap: include in solver if dist<margin-gap           (nworld, npair,)
    pair_friction: tangent1, 2, spin, roll1, 2               (nworld, npair, 5)
    exclude_signature: body1 << 16 + body2                   (nexclude,)
    condim_max: maximum condim for geoms
    tendon_adr: address of first object in tendon's path     (ntendon,)
    tendon_num: number of objects in tendon's path           (ntendon,)
    tendon_limited: does tendon have length limits           (ntendon,)
    tendon_limited_adr: addresses for limited tendons        (<=ntendon,)
    tendon_actfrclimited: does ten have actuator force limit (ntendon,)
    tendon_solref_lim: constraint solver reference: limit    (nworld, ntendon, mjNREF)
    tendon_solimp_lim: constraint solver impedance: limit    (nworld, ntendon, mjNIMP)
    tendon_solref_fri: constraint solver reference: friction (nworld, ntendon, mjNREF)
    tendon_solimp_fri: constraint solver impedance: friction (nworld, ntendon, mjNIMP)
    tendon_range: tendon length limits                       (nworld, ntendon, 2)
    tendon_actfrcrange: range of total actuator force        (nworld, ntendon, 2)
    tendon_margin: min distance for limit detection          (nworld, ntendon)
    tendon_stiffness: stiffness coefficient                  (nworld, ntendon)
    tendon_damping: damping coefficient                      (nworld, ntendon)
    tendon_armature: inertia associated with tendon velocity (nworld, ntendon)
    tendon_frictionloss: loss due to friction                (nworld, ntendon)
    tendon_lengthspring: spring resting length range         (nworld, ntendon, 2)
    tendon_length0: tendon length in qpos0                   (nworld, ntendon)
    tendon_invweight0: inv. weight in qpos0                  (nworld, ntendon)
    wrap_objid: object id: geom, site, joint                 (nwrap,)
    wrap_prm: divisor, joint coef, or site id                (nwrap,)
    wrap_type: wrap object type (WrapType)                   (nwrap,)
    tendon_jnt_adr: joint tendon address                     (<=nwrap,)
    tendon_site_pair_adr: site pair tendon address           (<=nwrap,)
    tendon_geom_adr: geom tendon address                     (<=nwrap,)
    ten_wrapadr_site: wrap object starting address for sites (ntendon,)
    ten_wrapnum_site: number of site wrap objects per tendon (ntendon,)
    wrap_jnt_adr: addresses for joint tendon wrap object     (<=nwrap,)
    wrap_site_adr: addresses for site tendon wrap object     (<=nwrap,)
    wrap_site_pair_adr: first address for site wrap pair     (<=nwrap,)
    wrap_geom_adr: addresses for geom tendon wrap object     (<=nwrap,)
    wrap_pulley_scale: pulley scaling                        (nwrap,)
    sensor_type: sensor type (SensorType)                    (nsensor,)
    sensor_datatype: numeric data type (DataType)            (nsensor,)
    sensor_objtype: type of sensorized object (ObjType)      (nsensor,)
    sensor_objid: id of sensorized object                    (nsensor,)
    sensor_reftype: type of reference frame (ObjType)        (nsensor,)
    sensor_refid: id of reference frame; -1: global frame    (nsensor,)
    sensor_intprm: sensor parameters                         (nsensor, mjNSENS)
    sensor_dim: number of scalar outputs                     (nsensor,)
    sensor_adr: address in sensor array                      (nsensor,)
    sensor_cutoff: cutoff for real and positive; 0: ignore   (nsensor,)
    sensor_pos_adr: addresses for position sensors           (<=nsensor,)
    sensor_limitpos_adr: address for limit position sensors  (<=nsensor,)
    sensor_vel_adr: addresses for velocity sensors           (<=nsensor,)
                    (excluding limit velocity sensors)
    sensor_limitvel_adr: address for limit velocity sensors  (<=nsensor,)
    sensor_acc_adr: addresses for acceleration sensors       (<=nsensor,)
    sensor_rangefinder_adr: addresses for rangefinder sensors(<=nsensor,)
    rangefinder_sensor_adr: map sensor id to rangefinder id  (<=nsensor,)
                    (excluding touch sensors)
                    (excluding limit force sensors)
    collision_sensor_adr: map sensor id to collision id      (nsensor,)
    sensor_touch_adr: addresses for touch sensors            (<=nsensor,)
    sensor_limitfrc_adr: address for limit force sensors     (<=nsensor,)
    sensor_e_potential: evaluate energy_pos
    sensor_e_kinetic: evaluate energy_vel
    sensor_tendonactfrc_adr: address for tendonactfrc sensor (<=nsensor,)
    sensor_subtree_vel: evaluate subtree_vel
    sensor_contact_adr: addresses for contact sensors        (<=nsensor,)
    sensor_adr_to_contact_adr: map sensor adr to contact adr (nsensor,)
    sensor_rne_postconstraint: evaluate rne_postconstraint
    sensor_rangefinder_bodyid: bodyid for rangefinder        (nrangefinder,)
    plugin: globally registered plugin slot number           (nplugin,)
    plugin_attr: config attributes of geom plugin            (nplugin, 3)
    geom_plugin_index: geom index in plugin array            (ngeom, )
    mocap_bodyid: id of body for mocap                       (nmocap,)
    mat_texid: texture id for rendering                      (nworld, nmat, mjNTEXROLE)
    mat_texrepeat: texture repeat for rendering              (nworld, nmat, 2)
    mat_rgba: rgba                                           (nworld, nmat, 4)
    actuator_trntype_body_adr: addresses for actuators       (<=nu,)
                               with body transmission
    block_dim: BlockDim
    geom_pair_type_count: count of max number of each potential collision
    has_sdf_geom: whether the model contains SDF geoms
  """

  nq: int
  nv: int
  nu: int
  na: int
  nbody: int
  njnt: int
  ngeom: int
  nsite: int
  ncam: int
  nlight: int
  nmat: int
  nflex: int
  nflexvert: int
  nflexedge: int
  nflexelem: int
  nflexelemdata: int
  nexclude: int
  neq: int
  nmocap: int
  ngravcomp: int
  nM: int
  nC: int
  ntendon: int
  nwrap: int
  nsensor: int
  nsensordata: int
  nsensortaxel: int
  nmeshvert: int
  nmeshface: int
  nmeshgraph: int
  nmeshpoly: int
  nmeshpolyvert: int
  nmeshpolymap: int
  nlsp: int  # warp only
  npair: int
  nhfield: int
  nhfielddata: int
  opt: Option
  stat: Statistic
  qpos0: wp.array2d(dtype=float)
  qpos_spring: wp.array2d(dtype=float)
  qM_fullm_i: wp.array(dtype=int)  # warp only
  qM_fullm_j: wp.array(dtype=int)  # warp only
  qM_mulm_i: wp.array(dtype=int)  # warp only
  qM_mulm_j: wp.array(dtype=int)  # warp only
  qM_madr_ij: wp.array(dtype=int)  # warp only
  qLD_updates: tuple[wp.array(dtype=wp.vec3i), ...]  # warp only
  M_rownnz: wp.array(dtype=int)
  M_rowadr: wp.array(dtype=int)
  M_colind: wp.array(dtype=int)
  mapM2M: wp.array(dtype=int)
  qM_tiles: tuple[TileSet, ...]
  body_tree: tuple[wp.array(dtype=int), ...]
  body_parentid: wp.array(dtype=int)
  body_rootid: wp.array(dtype=int)
  body_weldid: wp.array(dtype=int)
  body_mocapid: wp.array(dtype=int)
  body_jntnum: wp.array(dtype=int)
  body_jntadr: wp.array(dtype=int)
  body_dofnum: wp.array(dtype=int)
  body_dofadr: wp.array(dtype=int)
  body_geomnum: wp.array(dtype=int)
  body_geomadr: wp.array(dtype=int)
  body_pos: wp.array2d(dtype=wp.vec3)
  body_quat: wp.array2d(dtype=wp.quat)
  body_ipos: wp.array2d(dtype=wp.vec3)
  body_iquat: wp.array2d(dtype=wp.quat)
  body_mass: wp.array2d(dtype=float)
  body_subtreemass: wp.array2d(dtype=float)
  subtree_mass: wp.array2d(dtype=float)
  body_inertia: wp.array2d(dtype=wp.vec3)
  body_invweight0: wp.array2d(dtype=wp.vec2)
  body_contype: wp.array(dtype=int)
  body_conaffinity: wp.array(dtype=int)
  body_gravcomp: wp.array2d(dtype=float)
  jnt_type: wp.array(dtype=int)
  jnt_qposadr: wp.array(dtype=int)
  jnt_dofadr: wp.array(dtype=int)
  jnt_bodyid: wp.array(dtype=int)
  jnt_limited: wp.array(dtype=int)
  jnt_actfrclimited: wp.array(dtype=bool)
  jnt_solref: wp.array2d(dtype=wp.vec2)
  jnt_solimp: wp.array2d(dtype=vec5)
  jnt_pos: wp.array2d(dtype=wp.vec3)
  jnt_axis: wp.array2d(dtype=wp.vec3)
  jnt_stiffness: wp.array2d(dtype=float)
  jnt_range: wp.array2d(dtype=wp.vec2)
  jnt_actfrcrange: wp.array2d(dtype=wp.vec2)
  jnt_margin: wp.array2d(dtype=float)
  jnt_limited_slide_hinge_adr: wp.array(dtype=int)  # warp only
  jnt_limited_ball_adr: wp.array(dtype=int)  # warp only
  jnt_actgravcomp: wp.array(dtype=int)
  dof_bodyid: wp.array(dtype=int)
  dof_jntid: wp.array(dtype=int)
  dof_parentid: wp.array(dtype=int)
  dof_Madr: wp.array(dtype=int)
  dof_armature: wp.array2d(dtype=float)
  dof_damping: wp.array2d(dtype=float)
  dof_invweight0: wp.array2d(dtype=float)
  dof_frictionloss: wp.array2d(dtype=float)
  dof_solimp: wp.array2d(dtype=vec5)
  dof_solref: wp.array2d(dtype=wp.vec2)
  dof_tri_row: wp.array(dtype=int)  # warp only
  dof_tri_col: wp.array(dtype=int)  # warp only
  geom_type: wp.array(dtype=int)
  geom_contype: wp.array(dtype=int)
  geom_conaffinity: wp.array(dtype=int)
  geom_condim: wp.array(dtype=int)
  geom_bodyid: wp.array(dtype=int)
  geom_dataid: wp.array(dtype=int)
  geom_group: wp.array(dtype=int)
  geom_matid: wp.array2d(dtype=int)
  geom_priority: wp.array(dtype=int)
  geom_solmix: wp.array2d(dtype=float)
  geom_solref: wp.array2d(dtype=wp.vec2)
  geom_solimp: wp.array2d(dtype=vec5)
  geom_size: wp.array2d(dtype=wp.vec3)
  geom_aabb: wp.array2d(dtype=wp.vec3)
  geom_rbound: wp.array2d(dtype=float)
  geom_pos: wp.array2d(dtype=wp.vec3)
  geom_quat: wp.array2d(dtype=wp.quat)
  geom_friction: wp.array2d(dtype=wp.vec3)
  geom_margin: wp.array2d(dtype=float)
  geom_gap: wp.array2d(dtype=float)
  geom_rgba: wp.array2d(dtype=wp.vec4)
  hfield_adr: wp.array(dtype=int)
  hfield_nrow: wp.array(dtype=int)
  hfield_ncol: wp.array(dtype=int)
  hfield_size: wp.array(dtype=wp.vec4)
  hfield_data: wp.array(dtype=float)
  site_type: wp.array(dtype=int)
  site_bodyid: wp.array(dtype=int)
  site_size: wp.array(dtype=wp.vec3)
  site_pos: wp.array2d(dtype=wp.vec3)
  site_quat: wp.array2d(dtype=wp.quat)
  cam_mode: wp.array(dtype=int)
  cam_bodyid: wp.array(dtype=int)
  cam_targetbodyid: wp.array(dtype=int)
  cam_pos: wp.array2d(dtype=wp.vec3)
  cam_quat: wp.array2d(dtype=wp.quat)
  cam_poscom0: wp.array2d(dtype=wp.vec3)
  cam_pos0: wp.array2d(dtype=wp.vec3)
  cam_mat0: wp.array2d(dtype=wp.mat33)
  cam_fovy: wp.array(dtype=float)
  cam_resolution: wp.array(dtype=wp.vec2i)
  cam_sensorsize: wp.array(dtype=wp.vec2)
  cam_intrinsic: wp.array(dtype=wp.vec4)
  light_mode: wp.array(dtype=int)
  light_bodyid: wp.array(dtype=int)
  light_targetbodyid: wp.array(dtype=int)
  light_type: wp.array2d(dtype=int)
  light_castshadow: wp.array2d(dtype=bool)
  light_active: wp.array2d(dtype=bool)
  light_pos: wp.array2d(dtype=wp.vec3)
  light_dir: wp.array2d(dtype=wp.vec3)
  light_poscom0: wp.array2d(dtype=wp.vec3)
  light_pos0: wp.array2d(dtype=wp.vec3)
  light_dir0: wp.array2d(dtype=wp.vec3)
  flex_dim: wp.array(dtype=int)
  flex_vertadr: wp.array(dtype=int)
  flex_vertnum: wp.array(dtype=int)
  flex_edgeadr: wp.array(dtype=int)
  flex_elemedgeadr: wp.array(dtype=int)
  flex_vertbodyid: wp.array(dtype=int)
  flex_edge: wp.array(dtype=wp.vec2i)
  flex_edgeflap: wp.array(dtype=wp.vec2i)
  flex_elem: wp.array(dtype=int)
  flex_elemedge: wp.array(dtype=int)
  flexedge_length0: wp.array(dtype=float)
  flex_stiffness: wp.array(dtype=float)
  flex_bending: wp.array(dtype=float)
  flex_damping: wp.array(dtype=float)
  mesh_vertadr: wp.array(dtype=int)
  mesh_vertnum: wp.array(dtype=int)
  mesh_vert: wp.array(dtype=wp.vec3)
  mesh_normaladr: wp.array(dtype=int)
  mesh_normal: wp.array(dtype=wp.vec3)
  mesh_faceadr: wp.array(dtype=int)
  mesh_face: wp.array(dtype=wp.vec3i)
  mesh_graphadr: wp.array(dtype=int)
  mesh_graph: wp.array(dtype=int)
  mesh_quat: wp.array(dtype=wp.quat)
  mesh_polynum: wp.array(dtype=int)
  mesh_polyadr: wp.array(dtype=int)
  mesh_polynormal: wp.array(dtype=wp.vec3)
  mesh_polyvertadr: wp.array(dtype=int)
  mesh_polyvertnum: wp.array(dtype=int)
  mesh_polyvert: wp.array(dtype=int)
  mesh_polymapadr: wp.array(dtype=int)
  mesh_polymapnum: wp.array(dtype=int)
  mesh_polymap: wp.array(dtype=int)
  oct_aabb: wp.array2d(dtype=wp.vec3)
  oct_child: wp.array(dtype=vec8i)
  oct_coeff: wp.array(dtype=vec8f)
  eq_type: wp.array(dtype=int)
  eq_obj1id: wp.array(dtype=int)
  eq_obj2id: wp.array(dtype=int)
  eq_objtype: wp.array(dtype=int)
  eq_active0: wp.array(dtype=bool)
  eq_solref: wp.array2d(dtype=wp.vec2)
  eq_solimp: wp.array2d(dtype=vec5)
  eq_data: wp.array2d(dtype=vec11)
  eq_connect_adr: wp.array(dtype=int)
  eq_wld_adr: wp.array(dtype=int)
  eq_jnt_adr: wp.array(dtype=int)
  eq_ten_adr: wp.array(dtype=int)
  actuator_moment_tiles_nv: tuple[TileSet, ...]
  actuator_moment_tiles_nu: tuple[TileSet, ...]
  actuator_trntype: wp.array(dtype=int)
  actuator_dyntype: wp.array(dtype=int)
  actuator_gaintype: wp.array(dtype=int)
  actuator_biastype: wp.array(dtype=int)
  actuator_trnid: wp.array(dtype=wp.vec2i)
  actuator_actadr: wp.array(dtype=int)
  actuator_actnum: wp.array(dtype=int)
  actuator_ctrllimited: wp.array(dtype=bool)
  actuator_forcelimited: wp.array(dtype=bool)
  actuator_actlimited: wp.array(dtype=bool)
  actuator_dynprm: wp.array2d(dtype=vec10f)
  actuator_gainprm: wp.array2d(dtype=vec10f)
  actuator_biasprm: wp.array2d(dtype=vec10f)
  actuator_actearly: wp.array(dtype=bool)
  actuator_ctrlrange: wp.array2d(dtype=wp.vec2)
  actuator_forcerange: wp.array2d(dtype=wp.vec2)
  actuator_actrange: wp.array2d(dtype=wp.vec2)
  actuator_gear: wp.array2d(dtype=wp.spatial_vector)
  actuator_cranklength: wp.array(dtype=float)
  actuator_acc0: wp.array(dtype=float)
  actuator_lengthrange: wp.array(dtype=wp.vec2)
  nxn_geom_pair: wp.array(dtype=wp.vec2i)  # warp only
  nxn_geom_pair_filtered: wp.array(dtype=wp.vec2i)  # warp only
  nxn_pairid: wp.array(dtype=int)  # warp only
  nxn_pairid_filtered: wp.array(dtype=int)  # warp only
  pair_dim: wp.array(dtype=int)
  pair_geom1: wp.array(dtype=int)
  pair_geom2: wp.array(dtype=int)
  pair_solref: wp.array2d(dtype=wp.vec2)
  pair_solreffriction: wp.array2d(dtype=wp.vec2)
  pair_solimp: wp.array2d(dtype=vec5)
  pair_margin: wp.array2d(dtype=float)
  pair_gap: wp.array2d(dtype=float)
  pair_friction: wp.array2d(dtype=vec5)
  exclude_signature: wp.array(dtype=int)
  condim_max: int  # warp only
  tendon_adr: wp.array(dtype=int)
  tendon_num: wp.array(dtype=int)
  tendon_limited: wp.array(dtype=int)
  tendon_limited_adr: wp.array(dtype=int)
  tendon_actfrclimited: wp.array(dtype=bool)
  tendon_solref_lim: wp.array2d(dtype=wp.vec2)
  tendon_solimp_lim: wp.array2d(dtype=vec5)
  tendon_solref_fri: wp.array2d(dtype=wp.vec2)
  tendon_solimp_fri: wp.array2d(dtype=vec5)
  tendon_range: wp.array2d(dtype=wp.vec2)
  tendon_actfrcrange: wp.array2d(dtype=wp.vec2)
  tendon_margin: wp.array2d(dtype=float)
  tendon_stiffness: wp.array2d(dtype=float)
  tendon_damping: wp.array2d(dtype=float)
  tendon_armature: wp.array2d(dtype=float)
  tendon_frictionloss: wp.array2d(dtype=float)
  tendon_lengthspring: wp.array2d(dtype=wp.vec2)
  tendon_length0: wp.array2d(dtype=float)
  tendon_invweight0: wp.array2d(dtype=float)
  wrap_objid: wp.array(dtype=int)
  wrap_prm: wp.array(dtype=float)
  wrap_type: wp.array(dtype=int)
  tendon_jnt_adr: wp.array(dtype=int)  # warp only
  tendon_site_pair_adr: wp.array(dtype=int)  # warp only
  tendon_geom_adr: wp.array(dtype=int)  # warp only
  ten_wrapadr_site: wp.array(dtype=int)  # warp only
  ten_wrapnum_site: wp.array(dtype=int)  # warp only
  wrap_jnt_adr: wp.array(dtype=int)  # warp only
  wrap_site_adr: wp.array(dtype=int)  # warp only
  wrap_site_pair_adr: wp.array(dtype=int)  # warp only
  wrap_geom_adr: wp.array(dtype=int)  # warp only
  wrap_pulley_scale: wp.array(dtype=float)  # warp only
  sensor_type: wp.array(dtype=int)
  sensor_datatype: wp.array(dtype=int)
  sensor_objtype: wp.array(dtype=int)
  sensor_objid: wp.array(dtype=int)
  sensor_reftype: wp.array(dtype=int)
  sensor_refid: wp.array(dtype=int)
  sensor_intprm: wp.array2d(dtype=int)
  sensor_dim: wp.array(dtype=int)
  sensor_adr: wp.array(dtype=int)
  sensor_cutoff: wp.array(dtype=float)
  sensor_pos_adr: wp.array(dtype=int)  # warp only
  sensor_limitpos_adr: wp.array(dtype=int)  # warp only
  sensor_vel_adr: wp.array(dtype=int)  # warp only
  sensor_limitvel_adr: wp.array(dtype=int)  # warp only
  sensor_acc_adr: wp.array(dtype=int)  # warp only
  sensor_rangefinder_adr: wp.array(dtype=int)  # warp only
  rangefinder_sensor_adr: wp.array(dtype=int)  # warp only
  collision_sensor_adr: wp.array(dtype=int)  # warp only
  sensor_touch_adr: wp.array(dtype=int)  # warp only
  sensor_limitfrc_adr: wp.array(dtype=int)  # warp only
  sensor_e_potential: bool  # warp only
  sensor_e_kinetic: bool  # warp only
  sensor_tendonactfrc_adr: wp.array(dtype=int)  # warp only
  sensor_subtree_vel: bool  # warp only
  sensor_contact_adr: wp.array(dtype=int)  # warp only
  sensor_adr_to_contact_adr: wp.array(dtype=int)  # warp only
  sensor_rne_postconstraint: bool  # warp only
  sensor_rangefinder_bodyid: wp.array(dtype=int)  # warp only
  plugin: wp.array(dtype=int)
  plugin_attr: wp.array(dtype=wp.vec3f)
  geom_plugin_index: wp.array(dtype=int)  # warp only
  mocap_bodyid: wp.array(dtype=int)  # warp only
  mat_texid: wp.array3d(dtype=int)
  mat_texrepeat: wp.array2d(dtype=wp.vec2)
  mat_rgba: wp.array2d(dtype=wp.vec4)
  actuator_trntype_body_adr: wp.array(dtype=int)  # warp only
  block_dim: BlockDim  # warp only
  geom_pair_type_count: tuple[int, ...]  # warp only
  has_sdf_geom: bool  # warp only
  taxel_vertadr: wp.array(dtype=int)  # warp only
  taxel_sensorid: wp.array(dtype=int)  # warp only


@dataclasses.dataclass
class Contact:
  """Contact data.

  Attributes:
    dist: distance between nearest points; neg: penetration
    pos: position of contact point: midpoint between geoms
    frame: normal is in [0-2], points from geom[0] to geom[1]
    includemargin: include if dist<includemargin=margin-gap
    friction: tangent1, 2, spin, roll1, 2
    solref: constraint solver reference, normal direction
    solreffriction: constraint solver reference, friction directions
    solimp: constraint solver impedance
    dim: contact space dimensionality: 1, 3, 4 or 6
    geom: geom ids; -1 for flex
    efc_address: address in efc; -1: not included
    worldid: world id
  """

  dist: wp.array(dtype=float)
  pos: wp.array(dtype=wp.vec3)
  frame: wp.array(dtype=wp.mat33)
  includemargin: wp.array(dtype=float)
  friction: wp.array(dtype=vec5)
  solref: wp.array(dtype=wp.vec2)
  solreffriction: wp.array(dtype=wp.vec2)
  solimp: wp.array(dtype=vec5)
  dim: wp.array(dtype=int)
  geom: wp.array(dtype=wp.vec2i)
  efc_address: wp.array2d(dtype=int)
  worldid: wp.array(dtype=int)


@dataclasses.dataclass
class Data:
  """Dynamic state that updates each step.

  Attributes:
    nworld: number of worlds
    nconmax: maximum number of contacts
    njmax: maximum number of constraints per world
    solver_niter: number of solver iterations                   (nworld,)
    ncon: number of detected contacts
    ne: number of equality constraints                          (nworld,)
    ne_connect: number of equality connect constraints          (nworld,)
    ne_weld: number of equality weld constraints                (nworld,)
    ne_jnt: number of equality joint constraints                (nworld,)
    ne_ten: number of equality tendon constraints               (nworld,)
    nf: number of friction constraints                          (nworld,)
    nl: number of limit constraints                             (nworld,)
    nefc: number of constraints                                 (nworld,)
    nsolving: number of unconverged worlds                      (1,)
    time: simulation time                                       (nworld,)
    energy: potential, kinetic energy                           (nworld, 2)
    qpos: position                                              (nworld, nq)
    qvel: velocity                                              (nworld, nv)
    act: actuator activation                                    (nworld, na)
    qacc_warmstart: acceleration used for warmstart             (nworld, nv)
    qacc_discrete: discrete-time acceleration                   (nworld, nv)
    ctrl: control                                               (nworld, nu)
    qfrc_applied: applied generalized force                     (nworld, nv)
    xfrc_applied: applied Cartesian force/torque                (nworld, nbody, 6)
    fluid_applied: applied fluid force/torque                   (nworld, nbody, 6)
    eq_active: enable/disable constraints                       (nworld, neq)
    mocap_pos: position of mocap bodies                         (nworld, nmocap, 3)
    mocap_quat: orientation of mocap bodies                     (nworld, nmocap, 4)
    qacc: acceleration                                          (nworld, nv)
    act_dot: time-derivative of actuator activation             (nworld, na)
    xpos: Cartesian position of body frame                      (nworld, nbody, 3)
    xquat: Cartesian orientation of body frame                  (nworld, nbody, 4)
    xmat: Cartesian orientation of body frame                   (nworld, nbody, 3, 3)
    xipos: Cartesian position of body com                       (nworld, nbody, 3)
    ximat: Cartesian orientation of body inertia                (nworld, nbody, 3, 3)
    xanchor: Cartesian position of joint anchor                 (nworld, njnt, 3)
    xaxis: Cartesian joint axis                                 (nworld, njnt, 3)
    geom_skip: skip calculating `geom_xpos` and `geom_xmat`     (ngeom,)
               during step, reuse previous value
    geom_xpos: Cartesian geom position                          (nworld, ngeom, 3)
    geom_xmat: Cartesian geom orientation                       (nworld, ngeom, 3, 3)
    site_xpos: Cartesian site position                          (nworld, nsite, 3)
    site_xmat: Cartesian site orientation                       (nworld, nsite, 3, 3)
    cam_xpos: Cartesian camera position                         (nworld, ncam, 3)
    cam_xmat: Cartesian camera orientation                      (nworld, ncam, 3, 3)
    light_xpos: Cartesian light position                        (nworld, nlight, 3)
    light_xdir: Cartesian light direction                       (nworld, nlight, 3)
    subtree_com: center of mass of each subtree                 (nworld, nbody, 3)
    cdof: com-based motion axis of each dof (rot:lin)           (nworld, nv, 6)
    cinert: com-based body inertia and mass                     (nworld, nbody, 10)
    actuator_length: actuator lengths                           (nworld, nu)
    actuator_moment: actuator moments                           (nworld, nu, nv)
    crb: com-based composite inertia and mass                   (nworld, nbody, 10)
    qM: total inertia (sparse) (nworld, 1, nM) or               (nworld, nv, nv) if dense
    qLD: L'*D*L factorization of M (sparse) (nworld, 1, nM) or  (nworld, nv, nv) if dense
    qLDiagInv: 1/diag(D)                                        (nworld, nv)
    ten_velocity: tendon velocities                             (nworld, ntendon)
    actuator_velocity: actuator velocities                      (nworld, nu)
    cvel: com-based velocity (rot:lin)                          (nworld, nbody, 6)
    cdof_dot: time-derivative of cdof (rot:lin)                 (nworld, nv, 6)
    qfrc_bias: C(qpos,qvel)                                     (nworld, nv)
    qfrc_spring: passive spring force                           (nworld, nv)
    qfrc_damper: passive damper force                           (nworld, nv)
    qfrc_gravcomp: passive gravity compensation force           (nworld, nv)
    qfrc_fluid: passive fluid force                             (nworld, nv)
    qfrc_passive: total passive force                           (nworld, nv)
    subtree_linvel: linear velocity of subtree com              (nworld, nbody, 3)
    subtree_angmom: angular momentum about subtree com          (nworld, nbody, 3)
    subtree_bodyvel: subtree body velocity (ang, vel)           (nworld, nbody, 6)
    actuator_force: actuator force in actuation space           (nworld, nu)
    qfrc_actuator: actuator force                               (nworld, nv)
    qfrc_smooth: net unconstrained force                        (nworld, nv)
    qacc_smooth: unconstrained acceleration                     (nworld, nv)
    qfrc_constraint: constraint force                           (nworld, nv)
    qfrc_inverse: net external force; should equal:             (nworld, nv)
              qfrc_applied + J.T @ xfrc_applied + qfrc_actuator
    contact: contact data
    efc: constraint data
    rne_cacc: arrays used for smooth.rne                        (nworld, nbody, 6)
    rne_cfrc: arrays used for smooth.rne                        (nworld, nbody, 6)
    qpos_t0: temporary array for rk4                            (nworld, nq)
    qvel_t0: temporary array for rk4                            (nworld, nv)
    act_t0: temporary array for rk4                             (nworld, na)
    qvel_rk: temporary array for rk4                            (nworld, nv)
    qacc_rk: temporary array for rk4                            (nworld, nv)
    act_dot_rk: temporary array for rk4                         (nworld, na)
    qfrc_integration: temporary array for integration           (nworld, nv)
    qacc_integration: temporary array for integration           (nworld, nv)
    act_vel_integration: temporary array for integration        (nworld, nu)
    qM_integration: temporary array for integration             (nworld, nv, nv) if dense
    qLD_integration: temporary array for integration            (nworld, nv, nv) if dense
    qLDiagInv_integration: temporary array for integration      (nworld, nv)
    boxes_sorted: min, max of sorted bounding boxes             (nworld, ngeom, 2)
    sap_projection_lower: broadphase context                    (nworld, ngeom, 2)
    sap_projection_upper: broadphase context                    (nworld, ngeom)
    sap_sort_index: broadphase context                          (nworld, ngeom, 2)
    sap_range: broadphase context                               (nworld, ngeom)
    sap_cumulative_sum: broadphase context                      (nworld, ngeom)
    sap_segment_index: broadphase context (requires nworld + 1) (nworld, 2)
    dyn_geom_aabb: dynamic geometry axis-aligned bounding boxes (nworld, ngeom, 2)
    collision_pair: collision pairs from broadphase             (nconmax,)
    collision_worldid: collision world ids from broadphase      (nconmax,)
    ncollision: collision count from broadphase
    epa_vert: vertices in EPA polytope in Minkowski space       (nconmax, 5 + CCDiter)
    epa_vert1: vertices in EPA polytope in geom 1 space         (nconmax, 5 + CCDiter)
    epa_vert2: vertices in EPA polytope in geom 2 space         (nconmax, 5 + CCDiter)
    epa_vert_index1: vertex indices in EPA polytope for geom 1  (nconmax, 5 + CCDiter)
    epa_vert_index2: vertex indices in EPA polytope for geom 2  (nconmax, 5 + CCDiter)
    epa_face: faces of polytope represented by three indices    (nconmax, 6 + 5 * CCDiter)
    epa_pr: projection of origin on polytope faces              (nconmax, 6 + 5 * CCDiter)
    epa_norm2: epa_pr * epa_pr                                  (nconmax, 6 + 5 * CCDiter)
    epa_index: index of face in polytope map                    (nconmax, 6 + 5 * CCDiter)
    epa_map: status of faces in polytope                        (nconmax, 6 + 5 * CCDiter)
    epa_horizon: index pair (i j) of edges on horizon           (nconmax, 2 * 12)
    multiccd_polygon: clipped contact surface                   (nconmax, 2 * max_npolygon)
    multiccd_clipped: clipped contact surface (intermediate)    (nconmax, 2 * max_npolygon)
    multiccd_pnormal: plane normal of clipping polygon          (nconmax, max_npolygon)
    multiccd_pdist: plane distance of clipping polygon          (nconmax, max_npolygon)
    multiccd_idx1: list of normal index candidates for Geom 1   (nconmax, max_meshdegree)
    multiccd_idx2: list of normal index candidates for Geom 2   (nconmax, max_meshdegree)
    multiccd_n1: list of normal candidates for Geom 1           (nconmax, max_meshdegree)
    multiccd_n2: list of normal candidates for Geom 1           (nconmax, max_meshdegree)
    multiccd_endvert: list of edge vertices candidates          (nconmax, max_meshdegree)
    multiccd_face1: contact face                                (nconmax, max_npolygon)
    multiccd_face2: contact face                                (nconmax, max_npolygon)
    cacc: com-based acceleration                                (nworld, nbody, 6)
    cfrc_int: com-based interaction force with parent           (nworld, nbody, 6)
    cfrc_ext: com-based external force on body                  (nworld, nbody, 6)
    ten_length: tendon lengths                                  (nworld, ntendon)
    ten_J: tendon Jacobian                                      (nworld, ntendon, nv)
    ten_Jdot: time derivative of tendon Jacobian                (nworld, ntendon, nv)
    ten_bias_coef: tendon bias force coefficient                (nworld, ntendon)
    ten_wrapadr: start address of tendon's path                 (nworld, ntendon)
    ten_wrapnum: number of wrap points in path                  (nworld, ntendon)
    ten_actfrc: total actuator force at tendon                  (nworld, ntendon)
    wrap_obj: geomid; -1: site; -2: pulley                      (nworld, nwrap, 2)
    wrap_xpos: Cartesian 3D points in all paths                 (nworld, nwrap, 6)
    wrap_geom_xpos: Cartesian 3D points for geom wrap points    (nworld, <=nwrap, 6)
    sensordata: sensor data array                               (nsensordata,)
    inverse_mul_m_skip: skip mul_m computation                  (nworld,)
    sensor_rangefinder_pnt: points for rangefinder              (nworld, nrangefinder, 3)
    sensor_rangefinder_vec: directions for rangefinder          (nworld, nrangefinder, 3)
    sensor_rangefinder_dist: distances for rangefinder          (nworld, nrangefinder)
    sensor_rangefinder_geomid: geomids for rangefinder          (nworld, nrangefinder)
    sensor_contact_nmatch: match count for each world-sensor    (nworld, <=nsensor)
    sensor_contact_matchid: id for matching contact             (nworld, <=nsensor, MJ_MAXCONPAIR)
    sensor_contact_criteria: critera for reduction              (nworld, <=nsensor, MJ_MAXCONPAIR)
    sensor_contact_direction: direction of contact              (nworld, <=nsensor, MJ_MAXCONPAIR)
    ray_bodyexclude: id of body to exclude from ray computation
    ray_dist: ray distance to nearest geom                      (nworld, 1)
    ray_geomid: id of geom that intersects with ray             (nworld, 1)
    energy_vel_mul_m_skip: skip mul_m computation               (nworld,)
    actuator_trntype_body_ncon: number of active contacts       (nworld, <=nu)
  """

  nworld: int  # warp only
  nconmax: int  # warp only
  njmax: int  # warp only
  solver_niter: wp.array(dtype=int)
  ncon: wp.array(dtype=int)
  ne: wp.array(dtype=int)
  ne_connect: wp.array(dtype=int)  # warp only
  ne_weld: wp.array(dtype=int)  # warp only
  ne_jnt: wp.array(dtype=int)  # warp only
  ne_ten: wp.array(dtype=int)  # warp only
  nf: wp.array(dtype=int)
  nl: wp.array(dtype=int)
  nefc: wp.array(dtype=int)
  nsolving: wp.array(dtype=int)  # warp only
  time: wp.array(dtype=float)
  energy: wp.array(dtype=wp.vec2)
  qpos: wp.array2d(dtype=float)
  qvel: wp.array2d(dtype=float)
  act: wp.array2d(dtype=float)
  qacc_warmstart: wp.array2d(dtype=float)
  qacc_discrete: wp.array2d(dtype=float)  # warp only
  ctrl: wp.array2d(dtype=float)
  qfrc_applied: wp.array2d(dtype=float)
  xfrc_applied: wp.array2d(dtype=wp.spatial_vector)
  fluid_applied: wp.array2d(dtype=wp.spatial_vector)  # warp only
  eq_active: wp.array2d(dtype=bool)
  mocap_pos: wp.array2d(dtype=wp.vec3)
  mocap_quat: wp.array2d(dtype=wp.quat)
  qacc: wp.array2d(dtype=float)
  act_dot: wp.array2d(dtype=float)
  xpos: wp.array2d(dtype=wp.vec3)
  xquat: wp.array2d(dtype=wp.quat)
  xmat: wp.array2d(dtype=wp.mat33)
  xipos: wp.array2d(dtype=wp.vec3)
  ximat: wp.array2d(dtype=wp.mat33)
  xanchor: wp.array2d(dtype=wp.vec3)
  xaxis: wp.array2d(dtype=wp.vec3)
  geom_skip: wp.array(dtype=bool)  # warp only
  geom_xpos: wp.array2d(dtype=wp.vec3)
  geom_xmat: wp.array2d(dtype=wp.mat33)
  site_xpos: wp.array2d(dtype=wp.vec3)
  site_xmat: wp.array2d(dtype=wp.mat33)
  cam_xpos: wp.array2d(dtype=wp.vec3)
  cam_xmat: wp.array2d(dtype=wp.mat33)
  light_xpos: wp.array2d(dtype=wp.vec3)
  light_xdir: wp.array2d(dtype=wp.vec3)
  subtree_com: wp.array2d(dtype=wp.vec3)
  cdof: wp.array2d(dtype=wp.spatial_vector)
  cinert: wp.array2d(dtype=vec10)
  flexvert_xpos: wp.array2d(dtype=wp.vec3)
  flexedge_length: wp.array2d(dtype=float)
  flexedge_velocity: wp.array2d(dtype=float)
  actuator_length: wp.array2d(dtype=float)
  actuator_moment: wp.array3d(dtype=float)
  crb: wp.array2d(dtype=vec10)
  qM: wp.array3d(dtype=float)
  qLD: wp.array3d(dtype=float)
  qLDiagInv: wp.array2d(dtype=float)
  ten_velocity: wp.array2d(dtype=float)
  actuator_velocity: wp.array2d(dtype=float)
  cvel: wp.array2d(dtype=wp.spatial_vector)
  cdof_dot: wp.array2d(dtype=wp.spatial_vector)
  qfrc_bias: wp.array2d(dtype=float)
  qfrc_spring: wp.array2d(dtype=float)
  qfrc_damper: wp.array2d(dtype=float)
  qfrc_gravcomp: wp.array2d(dtype=float)
  qfrc_fluid: wp.array2d(dtype=float)
  qfrc_passive: wp.array2d(dtype=float)
  subtree_linvel: wp.array2d(dtype=wp.vec3)
  subtree_angmom: wp.array2d(dtype=wp.vec3)
  subtree_bodyvel: wp.array2d(dtype=wp.spatial_vector)  # warp only
  actuator_force: wp.array2d(dtype=float)
  qfrc_actuator: wp.array2d(dtype=float)
  qfrc_smooth: wp.array2d(dtype=float)
  qacc_smooth: wp.array2d(dtype=float)
  qfrc_constraint: wp.array2d(dtype=float)
  qfrc_inverse: wp.array2d(dtype=float)
  contact: Contact
  efc: Constraint

  # RK4
  qpos_t0: wp.array2d(dtype=float)
  qvel_t0: wp.array2d(dtype=float)
  act_t0: wp.array2d(dtype=float)
  qvel_rk: wp.array2d(dtype=float)
  qacc_rk: wp.array2d(dtype=float)
  act_dot_rk: wp.array2d(dtype=float)

  # euler + implicit integration
  qfrc_integration: wp.array2d(dtype=float)
  qacc_integration: wp.array2d(dtype=float)
  act_vel_integration: wp.array2d(dtype=float)
  qM_integration: wp.array3d(dtype=float)
  qLD_integration: wp.array3d(dtype=float)
  qLDiagInv_integration: wp.array2d(dtype=float)

  # sweep-and-prune broadphase
  sap_projection_lower: wp.array3d(dtype=float)
  sap_projection_upper: wp.array2d(dtype=float)
  sap_sort_index: wp.array3d(dtype=int)
  sap_range: wp.array2d(dtype=int)
  sap_cumulative_sum: wp.array2d(dtype=int)
  sap_segment_index: wp.array2d(dtype=int)

  # collision driver
  collision_pair: wp.array(dtype=wp.vec2i)
  collision_pairid: wp.array(dtype=int)
  collision_worldid: wp.array(dtype=int)
  ncollision: wp.array(dtype=int)

  # narrowphase collision (EPA polytope)
  epa_vert: wp.array2d(dtype=wp.vec3)
  epa_vert1: wp.array2d(dtype=wp.vec3)
  epa_vert2: wp.array2d(dtype=wp.vec3)
  epa_vert_index1: wp.array2d(dtype=int)
  epa_vert_index2: wp.array2d(dtype=int)
  epa_face: wp.array2d(dtype=wp.vec3i)
  epa_pr: wp.array2d(dtype=wp.vec3)
  epa_norm2: wp.array2d(dtype=float)
  epa_index: wp.array2d(dtype=int)
  epa_map: wp.array2d(dtype=int)
  epa_horizon: wp.array2d(dtype=int)

  # narrowphase collision (multicontact)
  multiccd_polygon: wp.array2d(dtype=wp.vec3)
  multiccd_clipped: wp.array2d(dtype=wp.vec3)
  multiccd_pnormal: wp.array2d(dtype=wp.vec3)
  multiccd_pdist: wp.array2d(dtype=float)
  multiccd_idx1: wp.array2d(dtype=int)
  multiccd_idx2: wp.array2d(dtype=int)
  multiccd_n1: wp.array2d(dtype=wp.vec3)
  multiccd_n2: wp.array2d(dtype=wp.vec3)
  multiccd_endvert: wp.array2d(dtype=wp.vec3)
  multiccd_face1: wp.array2d(dtype=wp.vec3)
  multiccd_face2: wp.array2d(dtype=wp.vec3)

  # rne_postconstraint
  cacc: wp.array2d(dtype=wp.spatial_vector)
  cfrc_int: wp.array2d(dtype=wp.spatial_vector)
  cfrc_ext: wp.array2d(dtype=wp.spatial_vector)

  # tendon
  ten_length: wp.array2d(dtype=float)
  ten_J: wp.array3d(dtype=float)
  ten_Jdot: wp.array3d(dtype=float)  # warp only
  ten_bias_coef: wp.array2d(dtype=float)  # warp only
  ten_wrapadr: wp.array2d(dtype=int)
  ten_wrapnum: wp.array2d(dtype=int)
  ten_actfrc: wp.array2d(dtype=float)  # warp only
  wrap_obj: wp.array2d(dtype=wp.vec2i)
  wrap_xpos: wp.array2d(dtype=wp.spatial_vector)
  wrap_geom_xpos: wp.array2d(dtype=wp.spatial_vector)

  # sensors
  sensordata: wp.array2d(dtype=float)
  sensor_rangefinder_pnt: wp.array2d(dtype=wp.vec3)  # warp only
  sensor_rangefinder_vec: wp.array2d(dtype=wp.vec3)  # warp only
  sensor_rangefinder_dist: wp.array2d(dtype=float)  # warp only
  sensor_rangefinder_geomid: wp.array2d(dtype=int)  # warp only
  sensor_contact_nmatch: wp.array2d(dtype=int)  # warp only
  sensor_contact_matchid: wp.array3d(dtype=int)  # warp only
  sensor_contact_criteria: wp.array3d(dtype=float)  # warp only
  sensor_contact_direction: wp.array3d(dtype=float)  # warp only

  # ray
  ray_bodyexclude: wp.array(dtype=int)  # warp only
  ray_dist: wp.array2d(dtype=float)  # warp only
  ray_geomid: wp.array2d(dtype=int)  # warp only

  # mul_m
  energy_vel_mul_m_skip: wp.array(dtype=bool)
  inverse_mul_m_skip: wp.array(dtype=bool)  # warp only

  # actuator
  actuator_trntype_body_ncon: wp.array2d(dtype=int)
