# Copyright 2023 DeepMind Technologies Limited
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
"""Base types used in MJX."""

import enum
from typing import Tuple, Union
import warnings

import jax
import mujoco
from mujoco.mjx._src.dataclasses import PyTreeNode  # pylint: disable=g-importing-member
import numpy as np


class Impl(enum.Enum):
  """Implementation to use."""

  C = 'c'
  JAX = 'jax'
  WARP = 'warp'

  @classmethod
  def _missing_(cls, value):
    # This method is called only when lookup by value fails
    # (e.g., Impl('JAX') fails initially because 'JAX' != 'jax')
    if not isinstance(value, str):
      return None
    for member in cls:
      if member.value == value.lower():
        return member
    return None


class DisableBit(enum.IntFlag):
  """Disable default feature bitflags.

  Members:
    CONSTRAINT:   entire constraint solver
    EQUALITY:     equality constraints
    FRICTIONLOSS: joint and tendon frictionloss constraints
    LIMIT:        joint and tendon limit constraints
    CONTACT:      contact constraints
    PASSIVE:      passive forces
    GRAVITY:      gravitational forces
    CLAMPCTRL:    clamp control to specified range
    WARMSTART:    warmstart constraint solver
    ACTUATION:    apply actuation forces
    REFSAFE:      integrator safety: make ref[0]>=2*timestep
    SENSOR:       sensors
  """

  CONSTRAINT = mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
  EQUALITY = mujoco.mjtDisableBit.mjDSBL_EQUALITY
  FRICTIONLOSS = mujoco.mjtDisableBit.mjDSBL_FRICTIONLOSS
  LIMIT = mujoco.mjtDisableBit.mjDSBL_LIMIT
  CONTACT = mujoco.mjtDisableBit.mjDSBL_CONTACT
  PASSIVE = mujoco.mjtDisableBit.mjDSBL_PASSIVE
  GRAVITY = mujoco.mjtDisableBit.mjDSBL_GRAVITY
  CLAMPCTRL = mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL
  WARMSTART = mujoco.mjtDisableBit.mjDSBL_WARMSTART
  ACTUATION = mujoco.mjtDisableBit.mjDSBL_ACTUATION
  REFSAFE = mujoco.mjtDisableBit.mjDSBL_REFSAFE
  SENSOR = mujoco.mjtDisableBit.mjDSBL_SENSOR
  EULERDAMP = mujoco.mjtDisableBit.mjDSBL_EULERDAMP
  FILTERPARENT = mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
  # unsupported: MIDPHASE


class EnableBit(enum.IntFlag):
  """Enable optional feature bitflags.

  Members:
    INVDISCRETE: discrete-time inverse dynamics
  """

  INVDISCRETE = mujoco.mjtEnableBit.mjENBL_INVDISCRETE
  # unsupported: OVERRIDE, ENERGY, FWDINV, MULTICCD, ISLAND


class JointType(enum.IntEnum):
  """Type of degree of freedom.

  Members:
    FREE:  global position and orientation (quat)       (7,)
    BALL:  orientation (quat) relative to parent        (4,)
    SLIDE: sliding distance along body-fixed axis       (1,)
    HINGE: rotation angle (rad) around body-fixed axis  (1,)
  """

  FREE = mujoco.mjtJoint.mjJNT_FREE
  BALL = mujoco.mjtJoint.mjJNT_BALL
  SLIDE = mujoco.mjtJoint.mjJNT_SLIDE
  HINGE = mujoco.mjtJoint.mjJNT_HINGE

  def dof_width(self) -> int:
    return {0: 6, 1: 3, 2: 1, 3: 1}[self.value]

  def qpos_width(self) -> int:
    return {0: 7, 1: 4, 2: 1, 3: 1}[self.value]


class IntegratorType(enum.IntEnum):
  """Integrator mode.

  Members:
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

  Members:
    PLANE: plane
    HFIELD: height field
    SPHERE: sphere
    CAPSULE: capsule
    ELLIPSOID: ellipsoid
    CYLINDER: cylinder
    BOX: box
    MESH: mesh
    SDF: signed distance field
  """

  PLANE = mujoco.mjtGeom.mjGEOM_PLANE
  HFIELD = mujoco.mjtGeom.mjGEOM_HFIELD
  SPHERE = mujoco.mjtGeom.mjGEOM_SPHERE
  CAPSULE = mujoco.mjtGeom.mjGEOM_CAPSULE
  ELLIPSOID = mujoco.mjtGeom.mjGEOM_ELLIPSOID
  CYLINDER = mujoco.mjtGeom.mjGEOM_CYLINDER
  BOX = mujoco.mjtGeom.mjGEOM_BOX
  MESH = mujoco.mjtGeom.mjGEOM_MESH
  # unsupported: NGEOMTYPES, ARROW*, LINE, SKIN, LABEL, NONE


class ConvexMesh(PyTreeNode):
  """Geom properties for convex meshes.

  Members:
    vert: vertices of the convex mesh
    face: faces of the convex mesh
    face_normal: normal vectors for the faces
    edge: edge indexes for all edges in the convex mesh
    edge_face_normal: indexes for face normals adjacent to edges in `edge`
  """

  vert: jax.Array
  face: jax.Array
  face_normal: jax.Array
  edge: jax.Array
  edge_face_normal: jax.Array


class ConeType(enum.IntEnum):
  """Type of friction cone.

  Members:
    PYRAMIDAL: pyramidal
    ELLIPTIC: elliptic
  """

  PYRAMIDAL = mujoco.mjtCone.mjCONE_PYRAMIDAL
  ELLIPTIC = mujoco.mjtCone.mjCONE_ELLIPTIC


class JacobianType(enum.IntEnum):
  """Type of constraint Jacobian.

  Members:
    DENSE: dense
    SPARSE: sparse
    AUTO: sparse if nv>60 and device is TPU, dense otherwise
  """

  DENSE = mujoco.mjtJacobian.mjJAC_DENSE
  SPARSE = mujoco.mjtJacobian.mjJAC_SPARSE
  AUTO = mujoco.mjtJacobian.mjJAC_AUTO


class SolverType(enum.IntEnum):
  """Constraint solver algorithm.

  Members:
    CG: Conjugate gradient (primal)
    NEWTON: Newton (primal)
  """

  # unsupported: PGS
  CG = mujoco.mjtSolver.mjSOL_CG
  NEWTON = mujoco.mjtSolver.mjSOL_NEWTON


class EqType(enum.IntEnum):
  """Type of equality constraint.

  Members:
    CONNECT: connect two bodies at a point (ball joint)
    WELD: fix relative position and orientation of two bodies
    JOINT: couple the values of two scalar joints with cubic
    TENDON: couple the lengths of two tendons with cubic
  """

  CONNECT = mujoco.mjtEq.mjEQ_CONNECT
  WELD = mujoco.mjtEq.mjEQ_WELD
  JOINT = mujoco.mjtEq.mjEQ_JOINT
  TENDON = mujoco.mjtEq.mjEQ_TENDON
  # unsupported: DISTANCE


class WrapType(enum.IntEnum):
  """Type of tendon wrap object.

  Members:
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


class TrnType(enum.IntEnum):
  """Type of actuator transmission.

  Members:
    JOINT: force on joint
    JOINTINPARENT: force on joint, expressed in parent frame
    TENDON: force on tendon
    SITE: force on site
  """

  JOINT = mujoco.mjtTrn.mjTRN_JOINT
  JOINTINPARENT = mujoco.mjtTrn.mjTRN_JOINTINPARENT
  SITE = mujoco.mjtTrn.mjTRN_SITE
  TENDON = mujoco.mjtTrn.mjTRN_TENDON
  # unsupported: SLIDERCRANK, BODY


class DynType(enum.IntEnum):
  """Type of actuator dynamics.

  Members:
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

  Members:
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

  Members:
    NONE: no bias
    AFFINE: const + kp*length + kv*velocity
    MUSCLE: muscle passive force computed by muscle_bias
  """

  NONE = mujoco.mjtBias.mjBIAS_NONE
  AFFINE = mujoco.mjtBias.mjBIAS_AFFINE
  MUSCLE = mujoco.mjtBias.mjBIAS_MUSCLE
  # unsupported: USER


class ConstraintType(enum.IntEnum):
  """Type of constraint.

  Members:
    EQUALITY: equality constraint
    LIMIT_JOINT: joint limit
    LIMIT_TENDON: tendon limit
    CONTACT_FRICTIONLESS: frictionless contact
    CONTACT_PYRAMIDAL: frictional contact, pyramidal friction cone
  """

  EQUALITY = mujoco.mjtConstraint.mjCNSTR_EQUALITY
  FRICTION_DOF = mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF
  FRICTION_TENDON = mujoco.mjtConstraint.mjCNSTR_FRICTION_TENDON
  LIMIT_JOINT = mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT
  LIMIT_TENDON = mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON
  CONTACT_FRICTIONLESS = mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS
  CONTACT_PYRAMIDAL = mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL
  CONTACT_ELLIPTIC = mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC


class CamLightType(enum.IntEnum):
  """Type of camera light.

  Members:
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


class SensorType(enum.IntEnum):
  """Type of sensor.

  Members:
    MAGNETOMETER: magnetometer
    CAMPROJECTION: camera projection
    RANGEFINDER: rangefinder
    JOINTPOS: joint position
    TENDONPOS: scalar tendon position
    ACTUATORPOS: actuator position
    BALLQUAT: ball joint orientation
    FRAMEPOS: frame position
    FRAMEXAXIS: frame x-axis
    FRAMEYAXIS: frame y-axis
    FRAMEZAXIS: frame z-axis
    FRAMEQUAT: frame orientation, represented as quaternion
    SUBTREECOM: subtree centor of mass
    CLOCK: simulation time
    VELOCIMETER: 3D linear velocity, in local frame
    GYRO: 3D angular velocity, in local frame
    JOINTVEL: joint velocity
    TENDONVEL: scalar tendon velocity
    ACTUATORVEL: actuator velocity
    BALLANGVEL: ball joint angular velocity
    FRAMELINVEL: 3D linear velocity
    FRAMEANGVEL: 3D angular velocity
    SUBTREELINVEL: subtree linear velocity
    SUBTREEANGMOM: subtree angular momentum
    TOUCH: scalar contact normal forces summed over the sensor zone
    ACCELEROMETER: accelerometer
    FORCE: force
    TORQUE: torque
    ACTUATORFRC: scalar actuator force
    JOINTACTFRC: scalar actuator force, measured at the joint
    TENDONACTFRC: scalar actuator force, measured at the tendon
    FRAMELINACC: 3D linear acceleration
    FRAMEANGACC: 3D angular acceleration
  """

  MAGNETOMETER = mujoco.mjtSensor.mjSENS_MAGNETOMETER
  CAMPROJECTION = mujoco.mjtSensor.mjSENS_CAMPROJECTION
  RANGEFINDER = mujoco.mjtSensor.mjSENS_RANGEFINDER
  JOINTPOS = mujoco.mjtSensor.mjSENS_JOINTPOS
  TENDONPOS = mujoco.mjtSensor.mjSENS_TENDONPOS
  ACTUATORPOS = mujoco.mjtSensor.mjSENS_ACTUATORPOS
  BALLQUAT = mujoco.mjtSensor.mjSENS_BALLQUAT
  FRAMEPOS = mujoco.mjtSensor.mjSENS_FRAMEPOS
  FRAMEXAXIS = mujoco.mjtSensor.mjSENS_FRAMEXAXIS
  FRAMEYAXIS = mujoco.mjtSensor.mjSENS_FRAMEYAXIS
  FRAMEZAXIS = mujoco.mjtSensor.mjSENS_FRAMEZAXIS
  FRAMEQUAT = mujoco.mjtSensor.mjSENS_FRAMEQUAT
  SUBTREECOM = mujoco.mjtSensor.mjSENS_SUBTREECOM
  CLOCK = mujoco.mjtSensor.mjSENS_CLOCK
  VELOCIMETER = mujoco.mjtSensor.mjSENS_VELOCIMETER
  GYRO = mujoco.mjtSensor.mjSENS_GYRO
  JOINTVEL = mujoco.mjtSensor.mjSENS_JOINTVEL
  TENDONVEL = mujoco.mjtSensor.mjSENS_TENDONVEL
  ACTUATORVEL = mujoco.mjtSensor.mjSENS_ACTUATORVEL
  BALLANGVEL = mujoco.mjtSensor.mjSENS_BALLANGVEL
  FRAMELINVEL = mujoco.mjtSensor.mjSENS_FRAMELINVEL
  FRAMEANGVEL = mujoco.mjtSensor.mjSENS_FRAMEANGVEL
  SUBTREELINVEL = mujoco.mjtSensor.mjSENS_SUBTREELINVEL
  SUBTREEANGMOM = mujoco.mjtSensor.mjSENS_SUBTREEANGMOM
  TOUCH = mujoco.mjtSensor.mjSENS_TOUCH
  ACCELEROMETER = mujoco.mjtSensor.mjSENS_ACCELEROMETER
  FORCE = mujoco.mjtSensor.mjSENS_FORCE
  TORQUE = mujoco.mjtSensor.mjSENS_TORQUE
  ACTUATORFRC = mujoco.mjtSensor.mjSENS_ACTUATORFRC
  JOINTACTFRC = mujoco.mjtSensor.mjSENS_JOINTACTFRC
  TENDONACTFRC = mujoco.mjtSensor.mjSENS_TENDONACTFRC
  FRAMELINACC = mujoco.mjtSensor.mjSENS_FRAMELINACC
  FRAMEANGACC = mujoco.mjtSensor.mjSENS_FRAMEANGACC


class ObjType(PyTreeNode):
  """Type of object.

  Members:
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


class Statistic(PyTreeNode):
  """Model statistics (in qpos0).

  Attributes:
    meaninertia: mean diagonal inertia
    meanmass: mean body mass (not used)
    meansize: mean body size (not used)
    extent: spatial extent (not used)
    center: center of model (not used)
  """

  meaninertia: jax.Array
  meanmass: jax.Array
  meansize: jax.Array
  extent: jax.Array
  center: jax.Array


class Option(PyTreeNode):
  """Physics options."""  # fmt: skip
  timestep: jax.Array
  impratio: jax.Array
  tolerance: jax.Array
  ls_tolerance: jax.Array
  gravity: jax.Array
  wind: jax.Array
  magnetic: jax.Array
  density: jax.Array
  viscosity: jax.Array
  o_margin: jax.Array
  o_solref: jax.Array
  o_solimp: jax.Array
  o_friction: jax.Array
  integrator: IntegratorType
  cone: ConeType
  jacobian: JacobianType
  solver: SolverType
  iterations: int
  ls_iterations: int
  disableflags: DisableBit
  enableflags: int
  disableactuator: int
  sdf_initpoints: int


class OptionC(Option):
  """C-specific option."""

  apirate: jax.Array
  noslip_tolerance: jax.Array
  ccd_tolerance: jax.Array
  noslip_iterations: int
  ccd_iterations: int
  sdf_iterations: int


class OptionJAX(Option):
  """JAX-specific option."""

  has_fluid_params: bool


class ModelC(PyTreeNode):
  """CPU-specific model data."""

  nbvh: jax.Array
  nbvhstatic: jax.Array
  nbvhdynamic: jax.Array
  nflex: jax.Array
  nflexvert: jax.Array
  nflexedge: jax.Array
  nflexelem: jax.Array
  nflexelemdata: jax.Array
  nflexshelldata: jax.Array
  nflexevpair: jax.Array
  nflextexcoord: jax.Array
  nplugin: jax.Array
  ntree: jax.Array
  narena: jax.Array
  body_bvhadr: jax.Array
  body_bvhnum: jax.Array
  bvh_child: jax.Array
  bvh_nodeid: jax.Array
  bvh_aabb: jax.Array
  geom_plugin: jax.Array
  light_bodyid: jax.Array
  light_targetbodyid: jax.Array
  flex_contype: jax.Array
  flex_conaffinity: jax.Array
  flex_condim: jax.Array
  flex_priority: jax.Array
  flex_solmix: jax.Array
  flex_solref: jax.Array
  flex_solimp: jax.Array
  flex_friction: jax.Array
  flex_margin: jax.Array
  flex_gap: jax.Array
  flex_internal: jax.Array
  flex_selfcollide: jax.Array
  flex_activelayers: jax.Array
  flex_dim: jax.Array
  flex_vertadr: jax.Array
  flex_vertnum: jax.Array
  flex_edgeadr: jax.Array
  flex_edgenum: jax.Array
  flex_elemadr: jax.Array
  flex_elemnum: jax.Array
  flex_elemdataadr: jax.Array
  flex_evpairadr: jax.Array
  flex_evpairnum: jax.Array
  flex_vertbodyid: jax.Array
  flex_edge: jax.Array
  flex_elem: jax.Array
  flex_elemlayer: jax.Array
  flex_evpair: jax.Array
  flex_vert: jax.Array
  flexedge_length0: jax.Array
  flexedge_invweight0: jax.Array
  flex_radius: jax.Array
  flex_edgestiffness: jax.Array
  flex_edgedamping: jax.Array
  flex_edgeequality: jax.Array
  flex_rigid: jax.Array
  flexedge_rigid: jax.Array
  flex_centered: jax.Array
  flex_bvhadr: jax.Array
  flex_bvhnum: jax.Array
  actuator_plugin: jax.Array
  sensor_plugin: jax.Array
  plugin: jax.Array


class ModelJAX(PyTreeNode):
  """JAX-specific model data."""

  dof_hasfrictionloss: np.ndarray
  geom_rbound_hfield: np.ndarray
  mesh_convex: Tuple[ConvexMesh, ...]
  tendon_hasfrictionloss: np.ndarray
  wrap_inside_maxiter: int
  wrap_inside_tolerance: float
  wrap_inside_z_init: float
  is_wrap_inside: np.ndarray


class Model(PyTreeNode):
  """Static model of the scene that remains unchanged with each physics step."""

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
  nmesh: int
  nmeshvert: int
  nmeshnormal: int
  nmeshtexcoord: int
  nmeshface: int
  nmeshgraph: int
  nhfield: int
  nhfielddata: int
  ntex: int
  ntexdata: int
  nmat: int
  npair: int
  nexclude: int
  neq: int
  ntendon: int
  nwrap: int
  nsensor: int
  nnumeric: int
  ntuple: int
  nkey: int
  nmocap: int
  nM: int  # pylint:disable=invalid-name
  nB: int  # pylint:disable=invalid-name
  nC: int  # pylint:disable=invalid-name
  nD: int  # pylint:disable=invalid-name
  nJmom: int  # pylint:disable=invalid-name
  ngravcomp: int
  nuserdata: int
  nsensordata: int
  opt: Option
  stat: Statistic
  qpos0: jax.Array
  qpos_spring: jax.Array
  body_parentid: np.ndarray
  body_mocapid: np.ndarray
  body_rootid: np.ndarray
  body_weldid: np.ndarray
  body_jntnum: np.ndarray
  body_jntadr: np.ndarray
  body_sameframe: np.ndarray
  body_dofnum: np.ndarray
  body_dofadr: np.ndarray
  body_treeid: np.ndarray
  body_geomnum: np.ndarray
  body_geomadr: np.ndarray
  body_simple: np.ndarray
  body_pos: jax.Array
  body_quat: jax.Array
  body_ipos: jax.Array
  body_iquat: jax.Array
  body_mass: jax.Array
  body_subtreemass: jax.Array
  body_inertia: jax.Array
  body_gravcomp: jax.Array
  body_margin: np.ndarray
  body_contype: np.ndarray
  body_conaffinity: np.ndarray
  body_invweight0: jax.Array
  jnt_type: np.ndarray
  jnt_qposadr: np.ndarray
  jnt_dofadr: np.ndarray
  jnt_bodyid: np.ndarray
  jnt_limited: np.ndarray
  jnt_actfrclimited: np.ndarray
  jnt_actgravcomp: np.ndarray
  jnt_solref: jax.Array
  jnt_solimp: jax.Array
  jnt_pos: jax.Array
  jnt_axis: jax.Array
  jnt_stiffness: jax.Array
  jnt_range: jax.Array
  jnt_actfrcrange: jax.Array
  jnt_margin: jax.Array
  dof_bodyid: np.ndarray
  dof_jntid: np.ndarray
  dof_parentid: np.ndarray
  dof_treeid: np.ndarray
  dof_Madr: np.ndarray  # pylint:disable=invalid-name
  dof_simplenum: np.ndarray
  dof_solref: jax.Array
  dof_solimp: jax.Array
  dof_frictionloss: jax.Array
  dof_armature: jax.Array
  dof_damping: jax.Array
  dof_invweight0: jax.Array
  dof_M0: jax.Array  # pylint:disable=invalid-name
  geom_type: np.ndarray
  geom_contype: np.ndarray
  geom_conaffinity: np.ndarray
  geom_condim: np.ndarray
  geom_bodyid: np.ndarray
  geom_sameframe: np.ndarray
  geom_dataid: np.ndarray
  geom_group: np.ndarray
  geom_matid: jax.Array
  geom_priority: np.ndarray
  geom_solmix: jax.Array
  geom_solref: jax.Array
  geom_solimp: jax.Array
  geom_size: jax.Array
  geom_aabb: np.ndarray
  geom_rbound: jax.Array
  geom_pos: jax.Array
  geom_quat: jax.Array
  geom_friction: jax.Array
  geom_margin: jax.Array
  geom_gap: jax.Array
  geom_fluid: np.ndarray
  geom_rgba: jax.Array
  site_type: np.ndarray
  site_bodyid: np.ndarray
  site_sameframe: np.ndarray
  site_size: np.ndarray
  site_pos: jax.Array
  site_quat: jax.Array
  cam_mode: np.ndarray
  cam_bodyid: np.ndarray
  cam_targetbodyid: np.ndarray
  cam_pos: jax.Array
  cam_quat: jax.Array
  cam_poscom0: jax.Array
  cam_pos0: jax.Array
  cam_mat0: jax.Array
  cam_fovy: np.ndarray
  cam_resolution: np.ndarray
  cam_sensorsize: np.ndarray
  cam_intrinsic: np.ndarray
  light_mode: np.ndarray
  light_type: jax.Array
  light_castshadow: jax.Array
  light_pos: jax.Array
  light_dir: jax.Array
  light_poscom0: jax.Array
  light_pos0: np.ndarray
  light_dir0: np.ndarray
  light_cutoff: jax.Array
  mesh_vertadr: np.ndarray
  mesh_vertnum: np.ndarray
  mesh_faceadr: np.ndarray
  mesh_bvhadr: np.ndarray
  mesh_bvhnum: np.ndarray
  mesh_graphadr: np.ndarray
  mesh_vert: np.ndarray
  mesh_face: np.ndarray
  mesh_graph: np.ndarray
  mesh_pos: np.ndarray
  mesh_quat: np.ndarray
  mesh_texcoordadr: np.ndarray
  mesh_texcoordnum: np.ndarray
  mesh_texcoord: np.ndarray
  hfield_size: np.ndarray
  hfield_nrow: np.ndarray
  hfield_ncol: np.ndarray
  hfield_adr: np.ndarray
  hfield_data: jax.Array
  tex_type: np.ndarray
  tex_height: np.ndarray
  tex_width: np.ndarray
  tex_nchannel: np.ndarray
  tex_adr: np.ndarray
  tex_data: jax.Array
  mat_rgba: jax.Array
  mat_texid: np.ndarray
  pair_dim: np.ndarray
  pair_geom1: np.ndarray
  pair_geom2: np.ndarray
  pair_signature: np.ndarray
  pair_solref: jax.Array
  pair_solreffriction: jax.Array
  pair_solimp: jax.Array
  pair_margin: jax.Array
  pair_gap: jax.Array
  pair_friction: jax.Array
  exclude_signature: np.ndarray
  eq_type: np.ndarray
  eq_obj1id: np.ndarray
  eq_obj2id: np.ndarray
  eq_objtype: np.ndarray
  eq_active0: np.ndarray
  eq_solref: jax.Array
  eq_solimp: jax.Array
  eq_data: jax.Array
  tendon_adr: np.ndarray
  tendon_num: np.ndarray
  tendon_limited: np.ndarray
  tendon_actfrclimited: np.ndarray
  tendon_solref_lim: jax.Array
  tendon_solimp_lim: jax.Array
  tendon_solref_fri: jax.Array
  tendon_solimp_fri: jax.Array
  tendon_range: jax.Array
  tendon_actfrcrange: jax.Array
  tendon_margin: jax.Array
  tendon_stiffness: jax.Array
  tendon_damping: jax.Array
  tendon_armature: jax.Array
  tendon_frictionloss: jax.Array
  tendon_lengthspring: jax.Array
  tendon_length0: jax.Array
  tendon_invweight0: jax.Array
  wrap_type: np.ndarray
  wrap_objid: np.ndarray
  wrap_prm: np.ndarray
  actuator_trntype: np.ndarray
  actuator_dyntype: np.ndarray
  actuator_gaintype: np.ndarray
  actuator_biastype: np.ndarray
  actuator_trnid: np.ndarray
  actuator_actadr: np.ndarray
  actuator_actnum: np.ndarray
  actuator_group: np.ndarray
  actuator_ctrllimited: np.ndarray
  actuator_forcelimited: np.ndarray
  actuator_actlimited: np.ndarray
  actuator_dynprm: jax.Array
  actuator_gainprm: jax.Array
  actuator_biasprm: jax.Array
  actuator_actearly: np.ndarray
  actuator_ctrlrange: jax.Array
  actuator_forcerange: jax.Array
  actuator_actrange: jax.Array
  actuator_gear: jax.Array
  actuator_cranklength: np.ndarray
  actuator_acc0: jax.Array
  actuator_lengthrange: np.ndarray
  sensor_type: np.ndarray
  sensor_datatype: np.ndarray
  sensor_needstage: np.ndarray
  sensor_objtype: np.ndarray
  sensor_objid: np.ndarray
  sensor_reftype: np.ndarray
  sensor_refid: np.ndarray
  sensor_dim: np.ndarray
  sensor_adr: np.ndarray
  sensor_cutoff: np.ndarray
  numeric_adr: np.ndarray
  numeric_data: np.ndarray
  tuple_adr: np.ndarray
  tuple_size: np.ndarray
  tuple_objtype: np.ndarray
  tuple_objid: np.ndarray
  tuple_objprm: np.ndarray
  key_time: np.ndarray
  key_qpos: np.ndarray
  key_qvel: np.ndarray
  key_act: np.ndarray
  key_mpos: np.ndarray
  key_mquat: np.ndarray
  key_ctrl: np.ndarray
  name_bodyadr: np.ndarray
  name_jntadr: np.ndarray
  name_geomadr: np.ndarray
  name_siteadr: np.ndarray
  name_camadr: np.ndarray
  name_meshadr: np.ndarray
  name_hfieldadr: np.ndarray
  name_pairadr: np.ndarray
  name_eqadr: np.ndarray
  name_tendonadr: np.ndarray
  name_actuatoradr: np.ndarray
  name_sensoradr: np.ndarray
  name_numericadr: np.ndarray
  name_tupleadr: np.ndarray
  name_keyadr: np.ndarray
  names: bytes
  signature: np.uint64
  _sizes: jax.Array
  _impl: Union[ModelC, ModelJAX]

  @property
  def impl(self) -> Impl:
    return {
        ModelC: Impl.C,
        ModelJAX: Impl.JAX,
    }[type(self._impl)]

  def __getattr__(self, name: str):
    if name == 'value':
      # Special case for NNX, the value attribute may not exist on the parent
      # PyTreeNode, before it exists on the child PyTreeNode. Thanks NNX.
      return object.__getattribute__(self, 'value')

    try:
      impl_instsance = object.__getattribute__(self, '_impl')
      val = getattr(impl_instsance, name)
      warnings.warn(
          f'Accessing `{name}` directly from `Model` is deprecated. '
          f'Access it via `model._impl.{name}` instead.',
          DeprecationWarning,
          stacklevel=2,
      )
    except AttributeError:
      # raise the standard exception
      raise AttributeError(  # pylint: disable=raise-missing-from
          f"'{type(self).__name__}' object has no attribute '{name}'"
      )
    return val


class Contact(PyTreeNode):
  """Result of collision detection functions.

  Attributes:
    dist: distance between nearest points; neg: penetration
    pos: position of contact point: midpoint between geoms            (3,)
    frame: normal is in [0-2]                                         (9,)
    includemargin: include if dist<includemargin=margin-gap           (1,)
    friction: tangent1, 2, spin, roll1, 2                             (5,)
    solref: constraint solver reference, normal direction             (mjNREF,)
    solreffriction: constraint solver reference, friction directions  (mjNREF,)
    solimp: constraint solver impedance                               (mjNIMP,)
    dim: contact space dimensionality: 1, 3, 4, or 6
    geom1: id of geom 1; deprecated, use geom[0]
    geom2: id of geom 2; deprecated, use geom[1]
    geom: geom ids                                                    (2,)
    efc_address: address in efc; -1: not included
  """  # fmt: skip
  dist: jax.Array
  pos: jax.Array
  frame: jax.Array
  includemargin: jax.Array
  friction: jax.Array
  solref: jax.Array
  solreffriction: jax.Array
  solimp: jax.Array
  # unsupported: mu, H (calculated locally in solver.py)
  dim: np.ndarray
  geom1: jax.Array
  geom2: jax.Array
  geom: jax.Array
  # unsupported: flex, elem, vert, exclude
  efc_address: np.ndarray


class DataC(PyTreeNode):
  """C-specific data."""

  # constant sizes:
  # TODO(stunya): make these sizes jax.Array?
  ne: int
  nf: int
  nl: int
  nefc: int
  ncon: int
  # TODO(stunya): remove most of these fields
  solver_niter: jax.Array
  cdof: jax.Array
  cinert: jax.Array
  light_xpos: jax.Array
  light_xdir: jax.Array
  flexvert_xpos: jax.Array
  flexelem_aabb: jax.Array
  flexedge_J_rownnz: jax.Array  # pylint:disable=invalid-name
  flexedge_J_rowadr: jax.Array  # pylint:disable=invalid-name
  flexedge_J_colind: jax.Array  # pylint:disable=invalid-name
  flexedge_J: jax.Array  # pylint:disable=invalid-name
  flexedge_length: jax.Array
  ten_wrapadr: jax.Array
  ten_wrapnum: jax.Array
  ten_J_rownnz: jax.Array  # pylint:disable=invalid-name
  ten_J_rowadr: jax.Array  # pylint:disable=invalid-name
  ten_J_colind: jax.Array  # pylint:disable=invalid-name
  ten_J: jax.Array  # pylint:disable=invalid-name
  ten_length: jax.Array
  wrap_obj: jax.Array
  wrap_xpos: jax.Array
  actuator_length: jax.Array
  moment_rownnz: jax.Array  # pylint:disable=invalid-name
  moment_rowadr: jax.Array  # pylint:disable=invalid-name
  moment_colind: jax.Array  # pylint:disable=invalid-name
  actuator_moment: jax.Array
  crb: jax.Array
  qM: jax.Array  # pylint:disable=invalid-name
  M: jax.Array  # pylint:disable=invalid-name
  qLD: jax.Array  # pylint:disable=invalid-name
  qLDiagInv: jax.Array  # pylint:disable=invalid-name
  bvh_aabb_dyn: jax.Array
  bvh_active: jax.Array
  # position, velocity dependent:
  flexedge_velocity: jax.Array
  ten_velocity: jax.Array
  actuator_velocity: jax.Array
  cdof_dot: jax.Array
  plugin_data: jax.Array
  qH: jax.Array  # pylint:disable=invalid-name
  qHDiagInv: jax.Array  # pylint:disable=invalid-name
  B_rownnz: jax.Array  # pylint:disable=invalid-name
  B_rowadr: jax.Array  # pylint:disable=invalid-name
  B_colind: jax.Array  # pylint:disable=invalid-name
  M_rownnz: jax.Array  # pylint:disable=invalid-name
  M_rowadr: jax.Array  # pylint:disable=invalid-name
  M_colind: jax.Array  # pylint:disable=invalid-name
  mapM2M: jax.Array  # pylint:disable=invalid-name
  D_rownnz: jax.Array  # pylint:disable=invalid-name
  D_rowadr: jax.Array  # pylint:disable=invalid-name
  D_diag: jax.Array  # pylint:disable=invalid-name
  D_colind: jax.Array  # pylint:disable=invalid-name
  mapM2D: jax.Array  # pylint:disable=invalid-name
  mapD2M: jax.Array  # pylint:disable=invalid-name
  qDeriv: jax.Array  # pylint:disable=invalid-name
  qLU: jax.Array  # pylint:disable=invalid-name
  qfrc_spring: jax.Array
  qfrc_damper: jax.Array
  cacc: jax.Array
  cfrc_int: jax.Array
  cfrc_ext: jax.Array
  subtree_linvel: jax.Array
  subtree_angmom: jax.Array
  # dynamically sized arrays which are made static for the frontend JAX API
  # TODO(stunya): remove these dynamic fields entirely
  contact: Contact
  efc_type: jax.Array
  efc_J: jax.Array  # pylint:disable=invalid-name
  efc_pos: jax.Array
  efc_margin: jax.Array
  efc_frictionloss: jax.Array
  efc_D: jax.Array  # pylint:disable=invalid-name
  efc_aref: jax.Array
  efc_force: jax.Array


class DataJAX(PyTreeNode):
  """JAX-specific data."""

  ne: int
  nf: int
  nl: int
  nefc: int
  ncon: int
  solver_niter: jax.Array
  cdof: jax.Array
  cinert: jax.Array
  ten_wrapadr: jax.Array
  ten_wrapnum: jax.Array
  ten_J: jax.Array  # pylint:disable=invalid-name
  ten_length: jax.Array
  wrap_obj: jax.Array
  wrap_xpos: jax.Array
  actuator_length: jax.Array
  actuator_moment: jax.Array
  crb: jax.Array
  qM: jax.Array  # pylint:disable=invalid-name
  M: jax.Array  # pylint:disable=invalid-name
  qLD: jax.Array  # pylint:disable=invalid-name
  qLDiagInv: jax.Array  # pylint:disable=invalid-name
  ten_velocity: jax.Array
  actuator_velocity: jax.Array
  cdof_dot: jax.Array
  cacc: jax.Array
  cfrc_int: jax.Array
  cfrc_ext: jax.Array
  subtree_linvel: jax.Array
  subtree_angmom: jax.Array
  # dynamically sized data which are made static due to JAX limitations
  contact: Contact
  efc_type: jax.Array
  efc_J: jax.Array  # pylint:disable=invalid-name
  efc_pos: jax.Array
  efc_margin: jax.Array
  efc_frictionloss: jax.Array
  efc_D: jax.Array  # pylint:disable=invalid-name
  efc_aref: jax.Array
  efc_force: jax.Array


class Data(PyTreeNode):
  """Dynamic state that updates each step."""

  # global properties:
  time: jax.Array
  # state:
  qpos: jax.Array
  qvel: jax.Array
  act: jax.Array
  qacc_warmstart: jax.Array
  # control:
  ctrl: jax.Array
  qfrc_applied: jax.Array
  xfrc_applied: jax.Array
  eq_active: jax.Array
  # mocap data:
  mocap_pos: jax.Array
  mocap_quat: jax.Array
  # dynamics:
  qacc: jax.Array
  act_dot: jax.Array
  # user data:
  userdata: jax.Array
  sensordata: jax.Array
  # position dependent:
  xpos: jax.Array
  xquat: jax.Array
  xmat: jax.Array
  xipos: jax.Array
  ximat: jax.Array
  xanchor: jax.Array
  xaxis: jax.Array
  geom_xpos: jax.Array
  geom_xmat: jax.Array
  site_xpos: jax.Array
  site_xmat: jax.Array
  cam_xpos: jax.Array
  cam_xmat: jax.Array
  subtree_com: jax.Array
  cvel: jax.Array
  qfrc_bias: jax.Array
  qfrc_gravcomp: jax.Array
  qfrc_fluid: jax.Array
  qfrc_passive: jax.Array
  qfrc_actuator: jax.Array
  actuator_force: jax.Array
  qfrc_smooth: jax.Array
  qacc_smooth: jax.Array
  qfrc_constraint: jax.Array
  qfrc_inverse: jax.Array
  _impl: Union[DataC, DataJAX]

  @property
  def impl(self) -> Impl:
    return {
        DataC: Impl.C,
        DataJAX: Impl.JAX,
    }[type(self._impl)]

  def __getattr__(self, name: str):
    if name == 'value':
      # Special case for NNX, the value attribute may not exist on the parent
      # PyTreeNode, before it exists on the child PyTreeNode. Thanks NNX.
      return object.__getattribute__(self, 'value')

    try:
      impl_instsance = object.__getattribute__(self, '_impl')
      val = getattr(impl_instsance, name)
      warnings.warn(
          f'Accessing `{name}` directly from `Data` is deprecated. '
          f'Access it via `data._impl.{name}` instead.',
          DeprecationWarning,
          stacklevel=2,
      )
    except AttributeError:
      # raise the standard exception
      raise AttributeError(  # pylint: disable=raise-missing-from
          f"'{type(self).__name__}' object has no attribute '{name}'"
      )
    return val
