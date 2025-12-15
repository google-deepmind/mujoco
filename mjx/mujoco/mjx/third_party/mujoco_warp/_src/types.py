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

TILE_SIZE_JTDAJ_SPARSE = 16
TILE_SIZE_JTDAJ_DENSE = 16


# TODO(team): add check that all wp.launch_tiled 'block_dim' settings are configurable
@dataclasses.dataclass
class BlockDim:
  """Block dimension 'block_dim' settings for wp.launch_tiled.

  TODO(team): experimental and may be removed
  """

  # collision_driver
  segmented_sort: int = 128
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
  update_gradient_cholesky_blocked: int = 32
  update_gradient_JTDAJ_sparse: int = 64
  update_gradient_JTDAJ_dense: int = 96
  linesearch_iterative: int = 64
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
    PLANE: collision between bounding sphere and plane
    SPHERE: collision between bounding spheres
    AABB: collision between axis-aligned bounding boxes
    OBB: collision between oriented bounding boxes
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
    WARMSTART:    warmstart constraint solver
    FILTERPARENT: disable collisions between parent and child bodies
    ACTUATION:    apply actuation forces
    REFSAFE:      integrator safety: make ref[0]>=2*timestep
    SENSOR:       sensors
    EULERDAMP:    implicit damping for Euler integration
    NATIVECCD:    native convex collision detection (ignored in MJWarp)
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
  FILTERPARENT = mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
  ACTUATION = mujoco.mjtDisableBit.mjDSBL_ACTUATION
  REFSAFE = mujoco.mjtDisableBit.mjDSBL_REFSAFE
  SENSOR = mujoco.mjtDisableBit.mjDSBL_SENSOR
  EULERDAMP = mujoco.mjtDisableBit.mjDSBL_EULERDAMP
  # unsupported: MIDPHASE, AUTORESET, NATIVECCD, ISLAND


class EnableBit(enum.IntFlag):
  """Enable optional feature bitflags.

  Attributes:
    ENERGY: energy computation
    INVDISCRETE: discrete-time inverse dynamics
  """

  ENERGY = mujoco.mjtEnableBit.mjENBL_ENERGY
  INVDISCRETE = mujoco.mjtEnableBit.mjENBL_INVDISCRETE
  # unsupported: OVERRIDE, FWDINV, ISLAND, MULTICCD


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
    FLEX: couple the edge lengths of a flex
  """

  CONNECT = mujoco.mjtEq.mjEQ_CONNECT
  WELD = mujoco.mjtEq.mjEQ_WELD
  JOINT = mujoco.mjtEq.mjEQ_JOINT
  TENDON = mujoco.mjtEq.mjEQ_TENDON
  FLEX = mujoco.mjtEq.mjEQ_FLEX
  # unsupported: DISTANCE


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


class State(enum.IntEnum):
  """State component elements as integer bitflags.

  Includes several convenient combinations of these flags.

  Attributes:
    TIME: time
    QPOS: position
    QVEL: velocity
    ACT: actuator activation
    WARMSTART: acceleration used for warmstart
    CTRL: control
    QFRC_APPLIED: applied generalized force
    XFRC_APPLIED: applied Cartesian force/torque
    EQ_ACTIVE: enable/disable constraints
    MOCAP_POS: positions of mocap bodies
    MOCAP_QUAT: orientations of mocap bodies
    NSTATE: number of state elements
    PHYSICS: QPOS | QVEL | ACT
    FULLPHYSICS: TIME | PHYSICS | PLUGIN
    USER: CTRL | QFRC_APPLIED | XFRC_APPLIED | EQ_ACTIVE | MOCAP_POS | MOCAP_QUAT | USERDATA
    INTEGRATION: FULLPHYSICS | USER | WARMSTART
  """

  TIME = mujoco.mjtState.mjSTATE_TIME
  QPOS = mujoco.mjtState.mjSTATE_QPOS
  QVEL = mujoco.mjtState.mjSTATE_QVEL
  ACT = mujoco.mjtState.mjSTATE_ACT
  WARMSTART = mujoco.mjtState.mjSTATE_WARMSTART
  CTRL = mujoco.mjtState.mjSTATE_CTRL
  QFRC_APPLIED = mujoco.mjtState.mjSTATE_QFRC_APPLIED
  XFRC_APPLIED = mujoco.mjtState.mjSTATE_XFRC_APPLIED
  EQ_ACTIVE = mujoco.mjtState.mjSTATE_EQ_ACTIVE
  MOCAP_POS = mujoco.mjtState.mjSTATE_MOCAP_POS
  MOCAP_QUAT = mujoco.mjtState.mjSTATE_MOCAP_QUAT
  NSTATE = mujoco.mjtState.mjNSTATE
  PHYSICS = mujoco.mjtState.mjSTATE_PHYSICS
  FULLPHYSICS = mujoco.mjtState.mjSTATE_FULLPHYSICS
  USER = mujoco.mjtState.mjSTATE_USER
  INTEGRATION = mujoco.mjtState.mjSTATE_INTEGRATION
  # unsupported: USERDATA, PLUGIN


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


class mat23f(wp.types.matrix(shape=(2, 3), dtype=float)):
  pass


class mat43f(wp.types.matrix(shape=(4, 3), dtype=float)):
  pass


class mat63f(wp.types.matrix(shape=(6, 3), dtype=float)):
  pass


vec5 = vec5f
vec6 = vec6f
vec8 = vec8f
vec10 = vec10f
vec11 = vec11f
mat23 = mat23f
mat43 = mat43f
mat63 = mat63f


def array(*args) -> wp.array:
  """A wrapper around wp.array that adds extra metadata to ease type introspection.

  Format is array(dim_1, dim_2, ..., dtype).  dim may be a constant int, or reference a size from
  Model or Data (e.g. "nq" or "nworld").  dim may also be "*", which means any nonzero size.
  """
  shape, dtype = args[:-1], args[-1]

  arr = wp.array(ndim=len(shape), dtype=dtype)
  arr.shape = shape

  return arr


@dataclasses.dataclass
class Option:
  """Physics options.

  Attributes:
    timestep: simulation timestep
    tolerance: main solver tolerance
    ls_tolerance: CG/Newton linesearch tolerance
    ccd_tolerance: convex collision detection tolerance
    density: density of medium
    viscosity: viscosity of medium
    gravity: gravitational acceleration
    wind: wind (for lift, drag, and viscosity)
    magnetic: global magnetic flux
    integrator: integration mode (IntegratorType)
    cone: type of friction cone (ConeType)
    solver: solver algorithm (SolverType)
    iterations: number of main solver iterations
    ls_iterations: maximum number of CG/Newton linesearch iterations
    ccd_iterations: number of iterations in convex collision detection
    disableflags: bit flags for disabling standard features
    enableflags: bit flags for enabling optional features
    sdf_initpoints: number of starting points for gradient descent
    sdf_iterations: max number of iterations for gradient descent

  warp only fields:
    impratio_invsqrt: ratio of friction-to-normal contact impedance (stored as inverse square root)
    is_sparse: whether to use sparse representations
    ls_parallel: evaluate engine solver step sizes in parallel
    ls_parallel_min_step: minimum step size for solver linesearch
    has_fluid: True if wind, density, or viscosity are non-zero at put_model time
    broadphase: broadphase type (BroadphaseType)
    broadphase_filter: broadphase filter bitflag (BroadphaseFilter)
    graph_conditional: flag to use cuda graph conditional
    run_collision_detection: if False, skips collision detection and allows user-populated
      contacts during the physics step (as opposed to DisableBit.CONTACT which explicitly
      zeros out the contacts at each step)
    contact_sensor_maxmatch: max number of contacts considered by contact sensor matching criteria
                             contacts matched after this value is exceded will be ignored
  """

  timestep: array("*", float)
  tolerance: array("*", float)
  ls_tolerance: array("*", float)
  ccd_tolerance: array("*", float)
  density: array("*", float)
  viscosity: array("*", float)
  gravity: array("*", wp.vec3)
  wind: array("*", wp.vec3)
  magnetic: array("*", wp.vec3)
  integrator: int
  cone: int
  solver: int
  iterations: int
  ls_iterations: int
  ccd_iterations: int
  disableflags: int
  enableflags: int
  sdf_initpoints: int
  sdf_iterations: int
  # warp only fields:
  impratio_invsqrt: array("*", float)
  is_sparse: bool
  ls_parallel: bool
  ls_parallel_min_step: float
  has_fluid: bool
  broadphase: BroadphaseType
  broadphase_filter: BroadphaseFilter
  graph_conditional: bool
  run_collision_detection: bool
  contact_sensor_maxmatch: int


@dataclasses.dataclass
class Statistic:
  """Model statistics (in qpos0).

  Attributes:
    meaninertia: mean diagonal inertia
  """

  meaninertia: float


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


@dataclasses.dataclass
class Model:
  """Model definition and parameters.

  Attributes:
    nq: number of generalized coordinates
    nv: number of degrees of freedom
    nu: number of actuators/controls
    na: number of activation states
    nbody: number of bodies
    noct: number of total octree cells in all meshes
    njnt: number of joints
    nM: number of non-zeros in sparse inertia matrix
    nC: number of non-zeros in sparse body-dof matrix
    ngeom: number of geoms
    nsite: number of sites
    ncam: number of cameras
    nlight: number of lights
    nflex: number of flexes
    nflexvert: number of vertices in all flexes
    nflexedge: number of edges in all flexes
    nflexelem: number of elements in all flexes
    nflexelemdata: number of element vertex ids in all flexes
    nflexelemedge: number of element edge ids in all flexes
    nmesh: number of meshes
    nmeshvert: number of vertices for all meshes
    nmeshnormal: number of normals in all meshes
    nmeshface: number of faces for all meshes
    nmeshgraph: number of ints in mesh auxiliary data
    nmeshpoly: number of polygons in all meshes
    nmeshpolyvert: number of vertices in all polygons
    nmeshpolymap: number of polygons in vertex map
    nhfield: number of heightfields
    nhfielddata: size of elevation data
    nmat: number of materials
    npair: number of predefined geom pairs
    nexclude: number of excluded geom pairs
    neq: number of equality constraints
    ntendon: number of tendons
    nwrap: number of wrap objects in all tendon paths
    nsensor: number of sensors
    nmocap: number of mocap bodies
    nplugin: number of plugin instances
    ngravcomp: number of bodies with nonzero gravcomp
    nsensordata: number of elements in sensor data vector
    opt: physics options
    stat: model statistics
    qpos0: qpos values at default pose                       (*, nq)
    qpos_spring: reference pose for springs                  (*, nq)
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
    body_pos: position offset rel. to parent body            (*, nbody, 3)
    body_quat: orientation offset rel. to parent body        (*, nbody, 4)
    body_ipos: local position of center of mass              (*, nbody, 3)
    body_iquat: local orientation of inertia ellipsoid       (*, nbody, 4)
    body_mass: mass                                          (*, nbody,)
    body_subtreemass: mass of subtree starting at this body  (*, nbody,)
    body_inertia: diagonal inertia in ipos/iquat frame       (*, nbody, 3)
    body_invweight0: mean inv inert in qpos0 (trn, rot)      (*, nbody, 2)
    body_gravcomp: antigravity force, units of body weight   (*, nbody)
    body_contype: OR over all geom contypes                  (nbody,)
    body_conaffinity: OR over all geom conaffinities         (nbody,)
    oct_child: octree children                               (noct, 8)
    oct_aabb: octree axis-aligned bounding boxes             (noct, 2, 3)
    oct_coeff: octree interpolation coefficients             (noct, 8)
    jnt_type: type of joint (JointType)                      (njnt,)
    jnt_qposadr: start addr in 'qpos' for joint's data       (njnt,)
    jnt_dofadr: start addr in 'qvel' for joint's data        (njnt,)
    jnt_bodyid: id of joint's body                           (njnt,)
    jnt_limited: does joint have limits                      (njnt,)
    jnt_actfrclimited: does joint have actuator force limits (njnt,)
    jnt_actgravcomp: is gravcomp force applied via actuators (njnt,)
    jnt_solref: constraint solver reference: limit           (*, njnt, mjNREF)
    jnt_solimp: constraint solver impedance: limit           (*, njnt, mjNIMP)
    jnt_pos: local anchor position                           (*, njnt, 3)
    jnt_axis: local joint axis                               (*, njnt, 3)
    jnt_stiffness: stiffness coefficient                     (*, njnt)
    jnt_range: joint limits                                  (*, njnt, 2)
    jnt_actfrcrange: range of total actuator force           (*, njnt, 2)
    jnt_margin: min distance for limit detection             (*, njnt)
    dof_bodyid: id of dof's body                             (nv,)
    dof_jntid: id of dof's joint                             (nv,)
    dof_parentid: id of dof's parent; -1: none               (nv,)
    dof_Madr: dof address in M-diagonal                      (nv,)
    dof_solref: constraint solver reference: frictionloss    (*, nv, NREF)
    dof_solimp: constraint solver impedance: frictionloss    (*, nv, NIMP)
    dof_frictionloss: dof friction loss                      (*, nv)
    dof_armature: dof armature inertia/mass                  (*, nv)
    dof_damping: damping coefficient                         (*, nv)
    dof_invweight0: diag. inverse inertia in qpos0           (*, nv)
    geom_type: geometric type (GeomType)                     (ngeom,)
    geom_contype: geom contact type                          (ngeom,)
    geom_conaffinity: geom contact affinity                  (ngeom,)
    geom_condim: contact dimensionality (1, 3, 4, 6)         (ngeom,)
    geom_bodyid: id of geom's body                           (ngeom,)
    geom_dataid: id of geom's mesh/hfield; -1: none          (ngeom,)
    geom_matid: material id for rendering                    (*, ngeom,)
    geom_group: geom group inclusion/exclusion mask          (ngeom,)
    geom_priority: geom contact priority                     (ngeom,)
    geom_solmix: mixing coef for solref/imp in geom pair     (*, ngeom,)
    geom_solref: constraint solver reference: contact        (*, ngeom, mjNREF)
    geom_solimp: constraint solver impedance: contact        (*, ngeom, mjNIMP)
    geom_size: geom-specific size parameters                 (*, ngeom, 3)
    geom_aabb: bounding box, (center, size)                  (*, ngeom, 2, 3)
    geom_rbound: radius of bounding sphere                   (*, ngeom,)
    geom_pos: local position offset rel. to body             (*, ngeom, 3)
    geom_quat: local orientation offset rel. to body         (*, ngeom, 4)
    geom_friction: friction for (slide, spin, roll)          (*, ngeom, 3)
    geom_margin: detect contact if dist<margin               (*, ngeom,)
    geom_gap: include in solver if dist<margin-gap           (*, ngeom,)
    geom_fluid: fluid interaction parameters                 (ngeom, mjNFLUID)
    geom_rgba: rgba when material is omitted                 (*, ngeom, 4)
    site_type: geom type for rendering (GeomType)            (nsite,)
    site_bodyid: id of site's body                           (nsite,)
    site_size: geom size for rendering                       (nsite, 3)
    site_pos: local position offset rel. to body             (*, nsite, 3)
    site_quat: local orientation offset rel. to body         (*, nsite, 4)
    cam_mode: camera tracking mode (CamLightType)            (ncam,)
    cam_bodyid: id of camera's body                          (ncam,)
    cam_targetbodyid: id of targeted body; -1: none          (ncam,)
    cam_pos: position rel. to body frame                     (*, ncam, 3)
    cam_quat: orientation rel. to body frame                 (*, ncam, 4)
    cam_poscom0: global position rel. to sub-com in qpos0    (*, ncam, 3)
    cam_pos0: global position rel. to body in qpos0          (*, ncam, 3)
    cam_mat0: global orientation in qpos0                    (*, ncam, 3, 3)
    cam_fovy: y field-of-view (ortho ? len : deg)            (ncam,)
    cam_resolution: resolution: pixels [width, height]       (ncam, 2)
    cam_sensorsize: sensor size: length [width, height]      (ncam, 2)
    cam_intrinsic: [focal length; principal point]           (ncam, 4)
    light_mode: light tracking mode (CamLightType)           (nlight,)
    light_bodyid: id of light's body                         (nlight,)
    light_targetbodyid: id of targeted body; -1: none        (nlight,)
    light_type: spot, directional, etc. (mjtLightType)       (*, nlight)
    light_castshadow: does light cast shadows                (*, nlight)
    light_active: is light active                            (*, nlight)
    light_pos: position rel. to body frame                   (*, nlight, 3)
    light_dir: direction rel. to body frame                  (*, nlight, 3)
    light_poscom0: global position rel. to sub-com in qpos0  (*, nlight, 3)
    light_pos0: global position rel. to body in qpos0        (*, nlight, 3)
    light_dir0: global direction in qpos0                    (*, nlight, 3)
    flex_dim: 1: lines, 2: triangles, 3: tetrahedra          (nflex,)
    flex_vertadr: first vertex address                       (nflex,)
    flex_vertnum: number of vertices                         (nflex,)
    flex_edgeadr: first edge address                         (nflex,)
    flex_edgenum: number of edges                            (nflex,)
    flex_elemadr: first element address                      (nflex,)
    flex_elemnum: number of elements                         (nflex,)
    flex_elemedgeadr: first element address                  (nflex,)
    flex_vertbodyid: vertex body ids                         (nflexvert,)
    flex_edge: edge vertex ids (2 per edge)                  (nflexedge, 2)
    flex_edgeflap: adjacent vertex ids (dim=2 only)          (nflexedge, 2)
    flex_elem: element vertex ids (dim+1 per elem)           (nflexelemdata,)
    flex_elemedge: element edge ids                          (nflexelemedge,)
    flexedge_length0: edge lengths in qpos0                  (nflexedge,)
    flexedge_invweight0: inv. inertia for the edge           (nflexedge,)
    flex_stiffness: finite element stiffness matrix          (nflexelem, 21)
    flex_bending: bending stiffness                          (nflexedge, 17)
    flex_damping: Rayleigh's damping coefficient             (nflex,)
    mesh_vertadr: first vertex address                       (nmesh,)
    mesh_vertnum: number of vertices                         (nmesh,)
    mesh_faceadr: first face address                         (nmesh,)
    mesh_normaladr: first normal address                     (nmesh,)
    mesh_graphadr: graph data address; -1: no graph          (nmesh,)
    mesh_vert: vertex positions for all meshes               (nmeshvert, 3)
    mesh_normal: normals for all meshes                      (nmeshnormal, 3)
    mesh_face: face indices for all meshes                   (nface, 3)
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
    hfield_size: (x, y, z_top, z_bottom)                     (nhfield, 4)
    hfield_nrow: number of rows in grid                      (nhfield,)
    hfield_ncol: number of columns in grid                   (nhfield,)
    hfield_adr: start address in hfield_data                 (nhfield,)
    hfield_data: elevation data                              (nhfielddata,)
    mat_texid: texture id for rendering                      (nmat, mjNTEXROLE)
    mat_texrepeat: texture repeat for rendering              (*, nmat, 2)
    mat_rgba: rgba                                           (*, nmat, 4)
    pair_dim: contact dimensionality                         (npair,)
    pair_geom1: id of geom1                                  (npair,)
    pair_geom2: id of geom2                                  (npair,)
    pair_solref: solver reference: contact normal            (*, npair, mjNREF)
    pair_solreffriction: solver reference: contact friction  (*, npair, mjNREF)
    pair_solimp: solver impedance: contact                   (*, npair, mjNIMP)
    pair_margin: detect contact if dist<margin               (*, npair,)
    pair_gap: include in solver if dist<margin-gap           (*, npair,)
    pair_friction: tangent1, 2, spin, roll1, 2               (*, npair, 5)
    exclude_signature: body1 << 16 + body2                   (nexclude,)
    eq_type: constraint type (EqType)                        (neq,)
    eq_obj1id: id of object 1                                (neq,)
    eq_obj2id: id of object 2                                (neq,)
    eq_objtype: type of both objects (ObjType)               (neq,)
    eq_active0: initial enable/disable constraint state      (neq,)
    eq_solref: constraint solver reference                   (*, neq, mjNREF)
    eq_solimp: constraint solver impedance                   (*, neq, mjNIMP)
    eq_data: numeric data for constraint                     (*, neq, mjNEQDATA)
    tendon_adr: address of first object in tendon's path     (ntendon,)
    tendon_num: number of objects in tendon's path           (ntendon,)
    tendon_limited: does tendon have length limits           (ntendon,)
    tendon_actfrclimited: does ten have actuator force limit (ntendon,)
    tendon_solref_lim: constraint solver reference: limit    (*, ntendon, mjNREF)
    tendon_solimp_lim: constraint solver impedance: limit    (*, ntendon, mjNIMP)
    tendon_solref_fri: constraint solver reference: friction (*, ntendon, mjNREF)
    tendon_solimp_fri: constraint solver impedance: friction (*, ntendon, mjNIMP)
    tendon_range: tendon length limits                       (*, ntendon, 2)
    tendon_actfrcrange: range of total actuator force        (*, ntendon, 2)
    tendon_margin: min distance for limit detection          (*, ntendon)
    tendon_stiffness: stiffness coefficient                  (*, ntendon)
    tendon_damping: damping coefficient                      (*, ntendon)
    tendon_armature: inertia associated with tendon velocity (*, ntendon)
    tendon_frictionloss: loss due to friction                (*, ntendon)
    tendon_lengthspring: spring resting length range         (*, ntendon, 2)
    tendon_length0: tendon length in qpos0                   (*, ntendon)
    tendon_invweight0: inv. weight in qpos0                  (*, ntendon)
    wrap_type: wrap object type (WrapType)                   (nwrap,)
    wrap_objid: object id: geom, site, joint                 (nwrap,)
    wrap_prm: divisor, joint coef, or site id                (nwrap,)
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
    actuator_dynprm: dynamics parameters                     (*, nu, mjNDYN)
    actuator_gainprm: gain parameters                        (*, nu, mjNGAIN)
    actuator_biasprm: bias parameters                        (*, nu, mjNBIAS)
    actuator_actearly: step activation before force          (nu,)
    actuator_ctrlrange: range of controls                    (*, nu, 2)
    actuator_forcerange: range of forces                     (*, nu, 2)
    actuator_actrange: range of activations                  (*, nu, 2)
    actuator_gear: scale length and transmitted force        (*, nu, 6)
    actuator_cranklength: crank length for slider-crank      (nu,)
    actuator_acc0: acceleration from unit force in qpos0     (nu,)
    actuator_lengthrange: feasible actuator length range     (nu, 2)
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
    plugin: globally registered plugin slot number           (nplugin,)
    plugin_attr: config attributes of geom plugin            (nplugin, 3)
    M_rownnz: number of non-zeros in each row of qM          (nv,)
    M_rowadr: index of each row in qM                        (nv,)
    M_colind: column indices of non-zeros in qM              (nM,)
    mapM2M: index mapping from M (legacy) to M (CSR)         (nC)

  warp only fields:
    nv_pad: number of degrees of freedom + padding
    nacttrnbody: number of actuators with body transmission
    nsensorcollision: number of unique collisions for
                      geom distance sensors
    nsensortaxel: number of taxels in all tactile sensors
    nsensorcontact: number of contact sensors
    nrangefinder: number of rangefinder sensors
    nmaxcondim: maximum condim for geoms
    nmaxpyramid: maximum number of pyramid directions
    nmaxpolygon: maximum number of verts per polygon
    nmaxmeshdeg: maximum number of polygons per vert
    has_sdf_geom: whether the model contains SDF geoms
    block_dim: block dim options
    body_tree: list of body ids by tree level
    mocap_bodyid: id of body for mocap                       (nmocap,)
    body_fluid_ellipsoid: does body use ellipsoid fluid      (nbody,)
    jnt_limited_slide_hinge_adr: limited/slide/hinge jntadr
    jnt_limited_ball_adr: limited/ball jntadr
    dof_tri_row: dof lower triangle row (used in solver)
    dof_tri_col: dof lower triangle col (used in solver)
    nxn_geom_pair: collision pair geom ids [-2, ngeom-1]
    nxn_geom_pair_filtered: valid collision pair geom ids
                            [-1, ngeom - 1]
    nxn_pairid: contact pair id, -1 if not predefined,
                  -2 if skipped
                collision id, else -1
    nxn_pairid_filtered: active subset of nxn_pairid
    geom_pair_type_count: count of max number of each
                          potential collision
    geom_plugin_index: geom index in plugin array            (ngeom,)
    eq_connect_adr: eq_* addresses of type `CONNECT`
    eq_wld_adr: eq_* addresses of type `WELD`
    eq_jnt_adr: eq_* addresses of type `JOINT`
    eq_ten_adr: eq_* addresses of type `TENDON`
    eq_flex_adr: eq * addresses of type `FLEX
    tendon_jnt_adr: joint tendon address
    tendon_site_pair_adr: site pair tendon address
    tendon_geom_adr: geom tendon address
    tendon_limited_adr: addresses for limited tendons
    ten_wrapadr_site: wrap object starting address for sites
    ten_wrapnum_site: number of site wrap objects per tendon
    wrap_jnt_adr: addresses for joint tendon wrap object
    wrap_site_adr: addresses for site tendon wrap object
    wrap_site_pair_adr: first address for site wrap pair
    wrap_geom_adr: addresses for geom tendon wrap object
    wrap_pulley_scale: pulley scaling                        (nwrap,)
    actuator_trntype_body_adr: addresses for actuators
                               with body transmission
    sensor_pos_adr: addresses for position sensors
    sensor_limitpos_adr: address for limit position sensors
    sensor_vel_adr: addresses for velocity sensors
                    (excluding limit velocity sensors)
    sensor_limitvel_adr: address for limit velocity sensors
    sensor_acc_adr: addresses for acceleration sensors
    sensor_rangefinder_adr: addresses for rangefinder sensors
    rangefinder_sensor_adr: map sensor id to rangefinder id
                    (excluding touch sensors)
                    (excluding limit force sensors)
    sensor_collision_start_adr: address for sensor's first
                                item in collision
    collision_sensor_adr: map sensor id to collision id      (nsensor,)
    sensor_touch_adr: addresses for touch sensors
    sensor_limitfrc_adr: address for limit force sensors
    sensor_e_potential: evaluate energy_pos
    sensor_e_kinetic: evaluate energy_vel
    sensor_tendonactfrc_adr: address for tendonactfrc sensor
    sensor_subtree_vel: evaluate subtree_vel
    sensor_contact_adr: addresses for contact sensors        (nsensorcontact,)
    sensor_adr_to_contact_adr: map sensor adr to contact adr (nsensor,)
    sensor_rne_postconstraint: evaluate rne_postconstraint
    sensor_rangefinder_bodyid: bodyid for rangefinder        (nrangefinder,)
    taxel_vertadr: tactile sensor vertex address             (nsensortaxel,)
    taxel_sensorid: address for tactile sensors
    qM_tiles: tiling configuration
    qLD_updates: tuple of index triples for sparse factorization
    qM_fullm_i: sparse mass matrix addressing
    qM_fullm_j: sparse mass matrix addressing
    qM_mulm_i: sparse matmul addressing
    qM_mulm_j: sparse matmul addressing
    qM_madr_ij: sparse matmul addressing
  """

  nq: int
  nv: int
  nu: int
  na: int
  nbody: int
  noct: int
  njnt: int
  nM: int
  nC: int
  ngeom: int
  nsite: int
  ncam: int
  nlight: int
  nflex: int
  nflexvert: int
  nflexedge: int
  nflexelem: int
  nflexelemdata: int
  nflexelemedge: int
  nmesh: int
  nmeshvert: int
  nmeshnormal: int
  nmeshface: int
  nmeshgraph: int
  nmeshpoly: int
  nmeshpolyvert: int
  nmeshpolymap: int
  nhfield: int
  nhfielddata: int
  nmat: int
  npair: int
  nexclude: int
  neq: int
  ntendon: int
  nwrap: int
  nsensor: int
  nmocap: int
  nplugin: int
  ngravcomp: int
  nsensordata: int
  opt: Option
  stat: Statistic
  qpos0: array("*", "nq", float)
  qpos_spring: array("*", "nq", float)
  body_parentid: array("nbody", int)
  body_rootid: array("nbody", int)
  body_weldid: array("nbody", int)
  body_mocapid: array("nbody", int)
  body_jntnum: array("nbody", int)
  body_jntadr: array("nbody", int)
  body_dofnum: array("nbody", int)
  body_dofadr: array("nbody", int)
  body_geomnum: array("nbody", int)
  body_geomadr: array("nbody", int)
  body_pos: array("*", "nbody", wp.vec3)
  body_quat: array("*", "nbody", wp.quat)
  body_ipos: array("*", "nbody", wp.vec3)
  body_iquat: array("*", "nbody", wp.quat)
  body_mass: array("*", "nbody", float)
  body_subtreemass: array("*", "nbody", float)
  body_inertia: array("*", "nbody", wp.vec3)
  body_invweight0: array("*", "nbody", wp.vec2)
  body_gravcomp: array("*", "nbody", float)
  body_contype: array("nbody", int)
  body_conaffinity: array("nbody", int)
  oct_child: array("noct", vec8i)
  oct_aabb: array("noct", 2, wp.vec3)
  oct_coeff: array("noct", vec8)
  jnt_type: array("njnt", int)
  jnt_qposadr: array("njnt", int)
  jnt_dofadr: array("njnt", int)
  jnt_bodyid: array("njnt", int)
  jnt_limited: array("njnt", int)
  jnt_actfrclimited: array("njnt", bool)
  jnt_actgravcomp: array("njnt", int)
  jnt_solref: array("*", "njnt", wp.vec2)
  jnt_solimp: array("*", "njnt", vec5)
  jnt_pos: array("*", "njnt", wp.vec3)
  jnt_axis: array("*", "njnt", wp.vec3)
  jnt_stiffness: array("*", "njnt", float)
  jnt_range: array("*", "njnt", wp.vec2)
  jnt_actfrcrange: array("*", "njnt", wp.vec2)
  jnt_margin: array("*", "njnt", float)
  dof_bodyid: array("nv", int)
  dof_jntid: array("nv", int)
  dof_parentid: array("nv", int)
  dof_Madr: array("nv", int)
  dof_solref: array("*", "nv", wp.vec2)
  dof_solimp: array("*", "nv", vec5)
  dof_frictionloss: array("*", "nv", float)
  dof_armature: array("*", "nv", float)
  dof_damping: array("*", "nv", float)
  dof_invweight0: array("*", "nv", float)
  geom_type: array("ngeom", int)
  geom_contype: array("ngeom", int)
  geom_conaffinity: array("ngeom", int)
  geom_condim: array("ngeom", int)
  geom_bodyid: array("ngeom", int)
  geom_dataid: array("ngeom", int)
  geom_matid: array("*", "ngeom", int)
  geom_group: array("ngeom", int)
  geom_priority: array("ngeom", int)
  geom_solmix: array("*", "ngeom", float)
  geom_solref: array("*", "ngeom", wp.vec2)
  geom_solimp: array("*", "ngeom", vec5)
  geom_size: array("*", "ngeom", wp.vec3)
  geom_aabb: array("*", "ngeom", 2, wp.vec3)
  geom_rbound: array("*", "ngeom", float)
  geom_pos: array("*", "ngeom", wp.vec3)
  geom_quat: array("*", "ngeom", wp.quat)
  geom_friction: array("*", "ngeom", wp.vec3)
  geom_margin: array("*", "ngeom", float)
  geom_gap: array("*", "ngeom", float)
  geom_fluid: array("ngeom", 12, float)
  geom_rgba: array("*", "ngeom", wp.vec4)
  site_type: array("nsite", int)
  site_bodyid: array("nsite", int)
  site_size: array("nsite", wp.vec3)
  site_pos: array("*", "nsite", wp.vec3)
  site_quat: array("*", "nsite", wp.quat)
  cam_mode: array("ncam", int)
  cam_bodyid: array("ncam", int)
  cam_targetbodyid: array("ncam", int)
  cam_pos: array("*", "ncam", wp.vec3)
  cam_quat: array("*", "ncam", wp.quat)
  cam_poscom0: array("*", "ncam", wp.vec3)
  cam_pos0: array("*", "ncam", wp.vec3)
  cam_mat0: array("*", "ncam", wp.mat33)
  cam_fovy: array("ncam", float)
  cam_resolution: array("ncam", wp.vec2i)
  cam_sensorsize: array("ncam", wp.vec2)
  cam_intrinsic: array("ncam", wp.vec4)
  light_mode: array("nlight", int)
  light_bodyid: array("nlight", int)
  light_targetbodyid: array("nlight", int)
  light_type: array("*", "nlight", int)
  light_castshadow: array("*", "nlight", bool)
  light_active: array("*", "nlight", bool)
  light_pos: array("*", "nlight", wp.vec3)
  light_dir: array("*", "nlight", wp.vec3)
  light_poscom0: array("*", "nlight", wp.vec3)
  light_pos0: array("*", "nlight", wp.vec3)
  light_dir0: array("*", "nlight", wp.vec3)
  flex_dim: array("nflex", int)
  flex_vertadr: array("nflex", int)
  flex_vertnum: array("nflex", int)
  flex_edgeadr: array("nflex", int)
  flex_edgenum: array("nflex", int)
  flex_elemadr: array("nflex", int)
  flex_elemnum: array("nflex", int)
  flex_elemedgeadr: array("nflex", int)
  flex_vertbodyid: array("nflexvert", int)
  flex_edge: array("nflexedge", wp.vec2i)
  flex_edgeflap: array("nflexedge", wp.vec2i)
  flex_elem: array("nflexelemdata", int)
  flex_elemedge: array("nflexelemedge", int)
  flexedge_length0: array("nflexedge", float)
  flexedge_invweight0: array("nflexedge", float)
  flex_stiffness: array("nflexelem", 21, float)
  flex_bending: array("nflexedge", 17, float)
  flex_damping: array("nflex", float)
  mesh_vertadr: array("nmesh", int)
  mesh_vertnum: array("nmesh", int)
  mesh_faceadr: array("nmesh", int)
  mesh_normaladr: array("nmesh", int)
  mesh_graphadr: array("nmesh", int)
  mesh_vert: array("nmeshvert", wp.vec3)
  mesh_normal: array("nmeshnormal", wp.vec3)
  mesh_face: array("nmeshface", wp.vec3i)
  mesh_graph: array("nmeshgraph", int)
  mesh_quat: array("nmesh", wp.quat)
  mesh_polynum: array("nmesh", int)
  mesh_polyadr: array("nmesh", int)
  mesh_polynormal: array("nmeshpoly", wp.vec3)
  mesh_polyvertadr: array("nmeshpoly", int)
  mesh_polyvertnum: array("nmeshpoly", int)
  mesh_polyvert: array("nmeshpolyvert", int)
  mesh_polymapadr: array("nmeshvert", int)
  mesh_polymapnum: array("nmeshvert", int)
  mesh_polymap: array("nmeshpolymap", int)
  hfield_size: array("nhfield", wp.vec4)
  hfield_nrow: array("nhfield", int)
  hfield_ncol: array("nhfield", int)
  hfield_adr: array("nhfield", int)
  hfield_data: array("nhfielddata", float)
  mat_texid: array("nmat", 10, int)
  mat_texrepeat: array("*", "nmat", wp.vec2)
  mat_rgba: array("*", "nmat", wp.vec4)
  pair_dim: array("npair", int)
  pair_geom1: array("npair", int)
  pair_geom2: array("npair", int)
  pair_solref: array("*", "npair", wp.vec2)
  pair_solreffriction: array("*", "npair", wp.vec2)
  pair_solimp: array("*", "npair", vec5)
  pair_margin: array("*", "npair", float)
  pair_gap: array("*", "npair", float)
  pair_friction: array("*", "npair", vec5)
  exclude_signature: array("nexclude", int)
  eq_type: array("neq", int)
  eq_obj1id: array("neq", int)
  eq_obj2id: array("neq", int)
  eq_objtype: array("neq", int)
  eq_active0: array("neq", bool)
  eq_solref: array("*", "neq", wp.vec2)
  eq_solimp: array("*", "neq", vec5)
  eq_data: array("*", "neq", vec11)
  tendon_adr: array("ntendon", int)
  tendon_num: array("ntendon", int)
  tendon_limited: array("ntendon", int)
  tendon_actfrclimited: array("ntendon", bool)
  tendon_solref_lim: array("*", "ntendon", wp.vec2)
  tendon_solimp_lim: array("*", "ntendon", vec5)
  tendon_solref_fri: array("*", "ntendon", wp.vec2)
  tendon_solimp_fri: array("*", "ntendon", vec5)
  tendon_range: array("*", "ntendon", wp.vec2)
  tendon_actfrcrange: array("*", "ntendon", wp.vec2)
  tendon_margin: array("*", "ntendon", float)
  tendon_stiffness: array("*", "ntendon", float)
  tendon_damping: array("*", "ntendon", float)
  tendon_armature: array("*", "ntendon", float)
  tendon_frictionloss: array("*", "ntendon", float)
  tendon_lengthspring: array("*", "ntendon", wp.vec2)
  tendon_length0: array("*", "ntendon", float)
  tendon_invweight0: array("*", "ntendon", float)
  wrap_type: array("nwrap", int)
  wrap_objid: array("nwrap", int)
  wrap_prm: array("nwrap", float)
  actuator_trntype: array("nu", int)
  actuator_dyntype: array("nu", int)
  actuator_gaintype: array("nu", int)
  actuator_biastype: array("nu", int)
  actuator_trnid: array("nu", wp.vec2i)
  actuator_actadr: array("nu", int)
  actuator_actnum: array("nu", int)
  actuator_ctrllimited: array("nu", bool)
  actuator_forcelimited: array("nu", bool)
  actuator_actlimited: array("nu", bool)
  actuator_dynprm: array("*", "nu", vec10f)
  actuator_gainprm: array("*", "nu", vec10f)
  actuator_biasprm: array("*", "nu", vec10f)
  actuator_actearly: array("nu", bool)
  actuator_ctrlrange: array("*", "nu", wp.vec2)
  actuator_forcerange: array("*", "nu", wp.vec2)
  actuator_actrange: array("*", "nu", wp.vec2)
  actuator_gear: array("*", "nu", wp.spatial_vector)
  actuator_cranklength: array("nu", float)
  actuator_acc0: array("nu", float)
  actuator_lengthrange: array("nu", wp.vec2)
  sensor_type: array("nsensor", int)
  sensor_datatype: array("nsensor", int)
  sensor_objtype: array("nsensor", int)
  sensor_objid: array("nsensor", int)
  sensor_reftype: array("nsensor", int)
  sensor_refid: array("nsensor", int)
  sensor_intprm: array("nsensor", 3, int)
  sensor_dim: array("nsensor", int)
  sensor_adr: array("nsensor", int)
  sensor_cutoff: array("nsensor", float)
  plugin: array("nplugin", int)
  plugin_attr: array("nplugin", wp.vec3f)
  M_rownnz: array("nv", int)
  M_rowadr: array("nv", int)
  M_colind: array("nC", int)
  mapM2M: array("nC", int)
  # warp only fields:
  nv_pad: int
  nacttrnbody: int
  nsensorcollision: int
  nsensortaxel: int
  nsensorcontact: int
  nrangefinder: int
  nmaxcondim: int
  nmaxpyramid: int
  nmaxpolygon: int
  nmaxmeshdeg: int
  has_sdf_geom: bool
  block_dim: BlockDim
  body_tree: tuple[wp.array(dtype=int), ...]
  mocap_bodyid: array("nmocap", int)
  body_fluid_ellipsoid: array("nbody", bool)
  jnt_limited_slide_hinge_adr: wp.array(dtype=int)
  jnt_limited_ball_adr: wp.array(dtype=int)
  dof_tri_row: wp.array(dtype=int)
  dof_tri_col: wp.array(dtype=int)
  nxn_geom_pair: wp.array(dtype=wp.vec2i)
  nxn_geom_pair_filtered: wp.array(dtype=wp.vec2i)
  nxn_pairid: wp.array(dtype=wp.vec2i)
  nxn_pairid_filtered: wp.array(dtype=wp.vec2i)
  geom_pair_type_count: tuple[int, ...]
  geom_plugin_index: array("ngeom", int)
  eq_connect_adr: wp.array(dtype=int)
  eq_wld_adr: wp.array(dtype=int)
  eq_jnt_adr: wp.array(dtype=int)
  eq_ten_adr: wp.array(dtype=int)
  eq_flex_adr: wp.array(dtype=int)
  tendon_jnt_adr: wp.array(dtype=int)
  tendon_site_pair_adr: wp.array(dtype=int)
  tendon_geom_adr: wp.array(dtype=int)
  tendon_limited_adr: wp.array(dtype=int)
  ten_wrapadr_site: wp.array(dtype=int)
  ten_wrapnum_site: wp.array(dtype=int)
  wrap_jnt_adr: wp.array(dtype=int)
  wrap_site_adr: wp.array(dtype=int)
  wrap_site_pair_adr: wp.array(dtype=int)
  wrap_geom_adr: wp.array(dtype=int)
  wrap_pulley_scale: array("nwrap", float)
  actuator_trntype_body_adr: wp.array(dtype=int)
  sensor_pos_adr: wp.array(dtype=int)
  sensor_limitpos_adr: wp.array(dtype=int)
  sensor_vel_adr: wp.array(dtype=int)
  sensor_limitvel_adr: wp.array(dtype=int)
  sensor_acc_adr: wp.array(dtype=int)
  sensor_rangefinder_adr: wp.array(dtype=int)
  rangefinder_sensor_adr: wp.array(dtype=int)
  sensor_collision_start_adr: wp.array(dtype=int)
  collision_sensor_adr: array("nsensor", int)
  sensor_touch_adr: wp.array(dtype=int)
  sensor_limitfrc_adr: wp.array(dtype=int)
  sensor_e_potential: bool
  sensor_e_kinetic: bool
  sensor_tendonactfrc_adr: wp.array(dtype=int)
  sensor_subtree_vel: bool
  sensor_contact_adr: array("nsensorcontact", int)
  sensor_adr_to_contact_adr: array("nsensor", int)
  sensor_rne_postconstraint: bool
  sensor_rangefinder_bodyid: array("nrangefinder", int)
  taxel_vertadr: array("nsensortaxel", int)
  taxel_sensorid: wp.array(dtype=int)
  qM_tiles: tuple[TileSet, ...]
  qLD_updates: tuple[wp.array(dtype=wp.vec3i), ...]
  qM_fullm_i: wp.array(dtype=int)
  qM_fullm_j: wp.array(dtype=int)
  qM_mulm_i: wp.array(dtype=int)
  qM_mulm_j: wp.array(dtype=int)
  qM_madr_ij: wp.array(dtype=int)


class ContactType(enum.IntFlag):
  """Type of contact.

  CONSTRAINT: contact for constraint solver.
  SENSOR: contact for collision sensor (GEOMDIST, GEOMNORMAL, GEOMFROMTO).
  """

  CONSTRAINT = 1
  SENSOR = 2


@dataclasses.dataclass
class Contact:
  """Contact data.

  Attributes:
    dist: distance between nearest points; neg: penetration          (naconmax,)
    pos: position of contact point: midpoint between geoms           (naconmax, 3)
    frame: normal is in [0-2], points from geom[0] to geom[1]        (naconmax, 3, 3)
    includemargin: include if dist<includemargin=margin-gap          (naconmax,)
    friction: tangent1, 2, spin, roll1, 2                            (naconmax, 5)
    solref: constraint solver reference, normal direction            (naconmax, 2)
    solreffriction: constraint solver reference, friction directions (naconmax, 2)
    solimp: constraint solver impedance                              (naconmax, 5)
    dim: contact space dimensionality: 1, 3, 4 or 6                  (naconmax,)
    geom: geom ids; -1 for flex                                      (naconmax, 2)
    efc_address: address in efc; -1: not included                    (naconmax, nmaxpyramid)
    worldid: world id                                                (naconmax,)
    type: ContactType                                                (naconmax,)
    geomcollisionid: i-th contact generated for geom                 (naconmax,)
                     helps uniquely identity contact when multiple
                     contacts are generated for geom pair
  """

  dist: array("naconmax", float)
  pos: array("naconmax", wp.vec3)
  frame: array("naconmax", wp.mat33)
  includemargin: array("naconmax", float)
  friction: array("naconmax", vec5)
  solref: array("naconmax", wp.vec2)
  solreffriction: array("naconmax", wp.vec2)
  solimp: array("naconmax", vec5)
  dim: array("naconmax", int)
  geom: array("naconmax", wp.vec2i)
  efc_address: array("naconmax", "nmaxpyramid", int)
  worldid: array("naconmax", int)
  type: array("naconmax", int)
  geomcollisionid: array("naconmax", int)


@dataclasses.dataclass
class Constraint:
  """Constraint data.

  Attributes:
    type: constraint type (ConstraintType)            (nworld, njmax)
    id: id of object of specific type                 (nworld, njmax)
    J: constraint Jacobian                            (nworld, njmax_pad, nv_pad)
    pos: constraint position (equality, contact)      (nworld, njmax)
    margin: inclusion margin (contact)                (nworld, njmax)
    D: constraint mass                                (nworld, njmax_pad)
    vel: velocity in constraint space: J*qvel         (nworld, njmax)
    aref: reference pseudo-acceleration               (nworld, njmax)
    frictionloss: frictionloss (friction)             (nworld, njmax)
    force: constraint force in constraint space       (nworld, njmax)
    Jaref: Jac*qacc - aref                            (nworld, njmax)
    Ma: M*qacc                                        (nworld, nv)
    grad: gradient of master cost                     (nworld, nv_pad)
    grad_dot: dot(grad, grad)                         (nworld,)
    Mgrad: M / grad                                   (nworld, nv_pad)
    search: linesearch vector                         (nworld, nv)
    search_dot: dot(search, search)                   (nworld,)
    gauss: Gauss Cost                                 (nworld,)
    cost: constraint + Gauss cost                     (nworld,)
    prev_cost: cost from previous iter                (nworld,)
    state: constraint state                           (nworld, njmax_pad)
    mv: qM @ search                                   (nworld, nv)
    jv: efc_J @ search                                (nworld, njmax)
    quad: quadratic cost coefficients                 (nworld, njmax, 3)
    quad_gauss: quadratic cost Gauss coefficients     (nworld, 3)
    alpha: line search step size                      (nworld,)
    prev_grad: previous grad                          (nworld, nv)
    prev_Mgrad: previous Mgrad                        (nworld, nv)
    beta: Polak-Ribiere beta                          (nworld,)
    done: solver done                                 (nworld,)
  """

  type: array("nworld", "njmax", int)
  id: array("nworld", "njmax", int)
  J: array("nworld", "njmax_pad", "nv_pad", float)
  pos: array("nworld", "njmax", float)
  margin: array("nworld", "njmax", float)
  D: array("nworld", "njmax_pad", float)
  vel: array("nworld", "njmax", float)
  aref: array("nworld", "njmax", float)
  frictionloss: array("nworld", "njmax", float)
  force: array("nworld", "njmax", float)
  Jaref: array("nworld", "njmax", float)
  Ma: array("nworld", "nv", float)
  grad: array("nworld", "nv_pad", float)
  grad_dot: array("nworld", float)
  Mgrad: array("nworld", "nv_pad", float)
  search: array("nworld", "nv", float)
  search_dot: array("nworld", float)
  gauss: array("nworld", float)
  cost: array("nworld", float)
  prev_cost: array("nworld", float)
  state: array("nworld", "njmax_pad", int)
  mv: array("nworld", "nv", float)
  jv: array("nworld", "njmax", float)
  quad: array("nworld", "njmax", wp.vec3)
  quad_gauss: array("nworld", wp.vec3)
  alpha: array("nworld", float)
  prev_grad: array("nworld", "nv", float)
  prev_Mgrad: array("nworld", "nv", float)
  beta: array("nworld", float)
  done: array("nworld", bool)


@dataclasses.dataclass
class Data:
  """Dynamic state that updates each step.

  Attributes:
    solver_niter: number of solver iterations                   (nworld,)
    ne: number of equality constraints                          (nworld,)
    nf: number of friction constraints                          (nworld,)
    nl: number of limit constraints                             (nworld,)
    nefc: number of constraints                                 (nworld,)
    time: simulation time                                       (nworld,)
    energy: potential, kinetic energy                           (nworld, 2)
    qpos: position                                              (nworld, nq)
    qvel: velocity                                              (nworld, nv)
    act: actuator activation                                    (nworld, na)
    qacc_warmstart: acceleration used for warmstart             (nworld, nv)
    ctrl: control                                               (nworld, nu)
    qfrc_applied: applied generalized force                     (nworld, nv)
    xfrc_applied: applied Cartesian force/torque                (nworld, nbody, 6)
    eq_active: enable/disable constraints                       (nworld, neq)
    mocap_pos: position of mocap bodies                         (nworld, nmocap, 3)
    mocap_quat: orientation of mocap bodies                     (nworld, nmocap, 4)
    qacc: acceleration                                          (nworld, nv)
    act_dot: time-derivative of actuator activation             (nworld, na)
    sensordata: sensor data array                               (nworld, nsensordata,)
    xpos: Cartesian position of body frame                      (nworld, nbody, 3)
    xquat: Cartesian orientation of body frame                  (nworld, nbody, 4)
    xmat: Cartesian orientation of body frame                   (nworld, nbody, 3, 3)
    xipos: Cartesian position of body com                       (nworld, nbody, 3)
    ximat: Cartesian orientation of body inertia                (nworld, nbody, 3, 3)
    xanchor: Cartesian position of joint anchor                 (nworld, njnt, 3)
    xaxis: Cartesian joint axis                                 (nworld, njnt, 3)
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
    flexvert_xpos: cartesian flex vertex positions              (nworld, nflexvert, 3)
    flexedge_J: edge length Jacobian                            (nworld, nflexedge, nv)
    flexedge_length: flex edge lengths                          (nworld, nflexedge, 1)
    ten_wrapadr: start address of tendon's path                 (nworld, ntendon)
    ten_wrapnum: number of wrap points in path                  (nworld, ntendon)
    ten_J: tendon Jacobian                                      (nworld, ntendon, nv)
    ten_length: tendon lengths                                  (nworld, ntendon)
    wrap_obj: geomid; -1: site; -2: pulley                      (nworld, nwrap, 2)
    wrap_xpos: Cartesian 3D points in all paths                 (nworld, nwrap, 6)
    actuator_length: actuator lengths                           (nworld, nu)
    actuator_moment: actuator moments                           (nworld, nu, nv)
    crb: com-based composite inertia and mass                   (nworld, nbody, 10)
    qM: total inertia                                           (nworld, nv, nv) if dense
                                                                (nworld, 1, nM) if sparse
    qLD: L'*D*L factorization of M                              (nworld, nv, nv) if dense
                                                                (nworld, 1, nC) if sparse
    qLDiagInv: 1/diag(D)                                        (nworld, nv)
    flexedge_velocity: flex edge velocities                     (nworld, nflexedge,)
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
    actuator_force: actuator force in actuation space           (nworld, nu)
    qfrc_actuator: actuator force                               (nworld, nv)
    qfrc_smooth: net unconstrained force                        (nworld, nv)
    qacc_smooth: unconstrained acceleration                     (nworld, nv)
    qfrc_constraint: constraint force                           (nworld, nv)
    qfrc_inverse: net external force; should equal:             (nworld, nv)
                  qfrc_applied + J.T @ xfrc_applied
                  + qfrc_actuator
    cacc: com-based acceleration                                (nworld, nbody, 6)
    cfrc_int: com-based interaction force with parent           (nworld, nbody, 6)
    cfrc_ext: com-based external force on body                  (nworld, nbody, 6)
    contact: contact data
    efc: constraint data

  warp only fields:
    nworld: number of worlds
    naconmax: maximum number of contacts (shared across all worlds)
    njmax: maximum number of constraints per world
    nacon: number of detected contacts (across all worlds)      (1,)
    ne_connect: number of equality connect constraints          (nworld,)
    ne_weld: number of equality weld constraints                (nworld,)
    ne_jnt: number of equality joint constraints                (nworld,)
    ne_ten: number of equality tendon constraints               (nworld,)
    ne_flex: number of flex edge equality constraints           (nworld,)
    nsolving: number of unconverged worlds                      (1,)
    subtree_bodyvel: subtree body velocity (ang, vel)           (nworld, nbody, 6)
    collision_pair: collision pairs from broadphase             (naconmax, 2)
    collision_pairid: ids from broadphase                       (naconmax, 2)
    collision_worldid: collision world ids from broadphase      (naconmax,)
    ncollision: collision count from broadphase                 (1,)
  """

  solver_niter: array("nworld", int)
  ne: array("nworld", int)
  nf: array("nworld", int)
  nl: array("nworld", int)
  nefc: array("nworld", int)
  time: array("nworld", float)
  energy: array("nworld", wp.vec2)
  qpos: array("nworld", "nq", float)
  qvel: array("nworld", "nv", float)
  act: array("nworld", "na", float)
  qacc_warmstart: array("nworld", "nv", float)
  ctrl: array("nworld", "nu", float)
  qfrc_applied: array("nworld", "nv", float)
  xfrc_applied: array("nworld", "nbody", wp.spatial_vector)
  eq_active: array("nworld", "neq", bool)
  mocap_pos: array("nworld", "nmocap", wp.vec3)
  mocap_quat: array("nworld", "nmocap", wp.quat)
  qacc: array("nworld", "nv", float)
  act_dot: array("nworld", "na", float)
  sensordata: array("nworld", "nsensordata", float)
  xpos: array("nworld", "nbody", wp.vec3)
  xquat: array("nworld", "nbody", wp.quat)
  xmat: array("nworld", "nbody", wp.mat33)
  xipos: array("nworld", "nbody", wp.vec3)
  ximat: array("nworld", "nbody", wp.mat33)
  xanchor: array("nworld", "njnt", wp.vec3)
  xaxis: array("nworld", "njnt", wp.vec3)
  geom_xpos: array("nworld", "ngeom", wp.vec3)
  geom_xmat: array("nworld", "ngeom", wp.mat33)
  site_xpos: array("nworld", "nsite", wp.vec3)
  site_xmat: array("nworld", "nsite", wp.mat33)
  cam_xpos: array("nworld", "ncam", wp.vec3)
  cam_xmat: array("nworld", "ncam", wp.mat33)
  light_xpos: array("nworld", "nlight", wp.vec3)
  light_xdir: array("nworld", "nlight", wp.vec3)
  subtree_com: array("nworld", "nbody", wp.vec3)
  cdof: array("nworld", "nv", wp.spatial_vector)
  cinert: array("nworld", "nbody", vec10)
  flexvert_xpos: array("nworld", "nflexvert", wp.vec3)
  flexedge_J: array("nworld", "nflexedge", "nv", float)
  flexedge_length: array("nworld", "nflexedge", float)
  ten_wrapadr: array("nworld", "ntendon", int)
  ten_wrapnum: array("nworld", "ntendon", int)
  ten_J: array("nworld", "ntendon", "nv", float)
  ten_length: array("nworld", "ntendon", float)
  wrap_obj: array("nworld", "nwrap", wp.vec2i)
  wrap_xpos: array("nworld", "nwrap", wp.spatial_vector)
  actuator_length: array("nworld", "nu", float)
  actuator_moment: array("nworld", "nu", "nv", float)
  crb: array("nworld", "nbody", vec10)
  qM: wp.array3d(dtype=float)
  qLD: wp.array3d(dtype=float)
  qLDiagInv: array("nworld", "nv", float)
  flexedge_velocity: array("nworld", "nflexedge", float)
  ten_velocity: array("nworld", "ntendon", float)
  actuator_velocity: array("nworld", "nu", float)
  cvel: array("nworld", "nbody", wp.spatial_vector)
  cdof_dot: array("nworld", "nv", wp.spatial_vector)
  qfrc_bias: array("nworld", "nv", float)
  qfrc_spring: array("nworld", "nv", float)
  qfrc_damper: array("nworld", "nv", float)
  qfrc_gravcomp: array("nworld", "nv", float)
  qfrc_fluid: array("nworld", "nv", float)
  qfrc_passive: array("nworld", "nv", float)
  subtree_linvel: array("nworld", "nbody", wp.vec3)
  subtree_angmom: array("nworld", "nbody", wp.vec3)
  actuator_force: array("nworld", "nu", float)
  qfrc_actuator: array("nworld", "nv", float)
  qfrc_smooth: array("nworld", "nv", float)
  qacc_smooth: array("nworld", "nv", float)
  qfrc_constraint: array("nworld", "nv", float)
  qfrc_inverse: array("nworld", "nv", float)
  cacc: array("nworld", "nbody", wp.spatial_vector)
  cfrc_int: array("nworld", "nbody", wp.spatial_vector)
  cfrc_ext: array("nworld", "nbody", wp.spatial_vector)
  contact: Contact
  efc: Constraint

  # warp only fields:
  nworld: int
  naconmax: int
  njmax: int
  nacon: array(1, int)
  ne_connect: array("nworld", int)
  ne_weld: array("nworld", int)
  ne_jnt: array("nworld", int)
  ne_ten: array("nworld", int)
  ne_flex: array("nworld", int)
  nsolving: array(1, int)
  subtree_bodyvel: array("nworld", "nbody", wp.spatial_vector)

  # warp only: collision driver
  collision_pair: array("naconmax", wp.vec2i)
  collision_pairid: array("naconmax", wp.vec2i)
  collision_worldid: array("naconmax", int)
  ncollision: array(1, int)
