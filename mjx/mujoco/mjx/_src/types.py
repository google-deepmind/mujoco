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

import dataclasses
import enum
from typing import Tuple
import jax
import mujoco
from mujoco.mjx._src.dataclasses import PyTreeNode  # pylint: disable=g-importing-member
import numpy as np


def _restricted_to(platform: str):
  """Specifies whether a field exists in only MuJoCo or MJX."""
  if platform not in ('mujoco', 'mjx'):
    raise ValueError(f'unknown platform: {platform}')
  return dataclasses.field(metadata={'restricted_to': platform})


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


class Option(PyTreeNode):
  """Physics options.

  Attributes:
    timestep:          timestep
    apirate:           update rate for remote API (Hz) (not used)
    impratio:          ratio of friction-to-normal contact impedance
    tolerance:         main solver tolerance
    ls_tolerance:      CG/Newton linesearch tolerance
    noslip_tolerance:  noslip solver tolerance (not used)
    ccd_tolerance:     CCD solver tolerance (not used)
    gravity:           gravitational acceleration                 (3,)
    wind:              wind (for lift, drag and viscosity)
    magnetic:          global magnetic flux (not used)
    density:           density of medium
    viscosity:         viscosity of medium
    o_margin:          contact solver override: margin (not used)
    o_solref:          contact solver override: solref (not used)
    o_solimp:          contact solver override: solimp (not used)
    o_friction[5]:     contact solver override: friction (not used)
    has_fluid_params:  automatically set by mjx if wind/density/viscosity are
      nonzero. Not used by mj
    integrator:        integration mode
    cone:              type of friction cone
    jacobian:          matrix layout for mass matrices (dense or sparse)
                       (note that this is different from MuJoCo, where jacobian
                       specifies whether efc_J and its accompanying matrices
                       are dense or sparse.
    solver:            solver algorithm
    iterations:        number of main solver iterations
    ls_iterations:     maximum number of CG/Newton linesearch iterations
    noslip_iterations: maximum number of noslip solver iterations (not used)
    ccd_iterations:    maximum number of CCD solver iterations (not used)
    disableflags:      bit flags for disabling standard features
    enableflags:       bit flags for enabling optional features (not used)
    disableactuator:   bit flags for disabling actuators by group id (not used)
    sdf_initpoints:    number of starting points for gradient descent (not used)
    sdf_iterations:    max number of iterations for gradient descent (not used)
  """  # fmt: skip
  timestep: jax.Array
  apirate: jax.Array = _restricted_to('mujoco')
  impratio: jax.Array
  tolerance: jax.Array
  ls_tolerance: jax.Array
  noslip_tolerance: jax.Array = _restricted_to('mujoco')
  ccd_tolerance: jax.Array = _restricted_to('mujoco')
  gravity: jax.Array
  wind: jax.Array
  magnetic: jax.Array
  density: jax.Array
  viscosity: jax.Array
  o_margin: jax.Array
  o_solref: jax.Array
  o_solimp: jax.Array
  o_friction: jax.Array
  has_fluid_params: bool = _restricted_to('mjx')
  integrator: IntegratorType
  cone: ConeType
  jacobian: JacobianType
  solver: SolverType
  iterations: int
  ls_iterations: int
  noslip_iterations: int = _restricted_to('mujoco')
  ccd_iterations: int = _restricted_to('mujoco')
  disableflags: DisableBit
  enableflags: int
  disableactuator: int
  sdf_initpoints: int = _restricted_to('mujoco')
  sdf_iterations: int = _restricted_to('mujoco')


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


class Model(PyTreeNode):
  """Static model of the scene that remains unchanged with each physics step.

  Attributes:
    nq: number of generalized coordinates = dim(qpos)
    nv: number of degrees of freedom = dim(qvel)
    nu: number of actuators/controls = dim(ctrl)
    na: number of activation states = dim(act)
    nbody: number of bodies
    nbvh: number of total bounding volumes in all bodies
    nbvhstatic: number of static bounding volumes (aabb stored in mjModel)
    nbvhdynamic: number of dynamic bounding volumes (aabb stored in mjData)
    njnt: number of joints
    ngeom: number of geoms
    nsite: number of sites
    ncam: number of cameras
    nlight: number of lights
    nflex: number of flexes
    nflexvert: number of vertices in all flexes
    nflexedge: number of edges in all flexes
    nflexelem: number of elements in all flexes
    nflexelemdata: number of element vertex ids in all flexes
    nflexshelldata: number of shell fragment vertex ids in all flexes
    nflexevpair: number of element-vertex pairs in all flexes
    nflextexcoord: number of vertices with texture coordinates
    nmesh: number of meshes
    nmeshvert: number of vertices in all meshes
    nmeshnormal: number of normals in all meshes
    nmeshtexcoord: number of texcoords in all meshes
    nmeshface: number of triangular faces in all meshes
    nmeshgraph: number of ints in mesh auxiliary data
    nhfield: number of heightfields
    nhfielddata: number of data points in all heightfields
    ntex: number of textures
    ntexdata: number of bytes in texture rgb data
    nmat: number of materials
    npair: number of predefined geom pairs
    nexclude: number of excluded geom pairs
    neq: number of equality constraints
    ntendon: number of tendons
    nwrap: number of wrap objects in all tendon paths
    nsensor: number of sensors
    nnumeric: number of numeric custom fields
    ntuple: number of tuple custom fields
    nkey: number of keyframes
    nmocap: number of mocap bodies
    nM: number of non-zeros in sparse inertia matrix
    nD: number of non-zeros in sparse dof-dof matrix
    nB: number of non-zeros in sparse body-dof matrix
    nC: number of non-zeros in sparse reduced dof-dof matrix
    nD: number of non-zeros in sparse dof-dof matrix
    nJmom: number of non-zeros in sparse actuator_moment matrix
    ntree: number of kinematic trees under world body
    ngravcomp: number of bodies with nonzero gravcomp
    nuserdata: size of userdata array
    nsensordata: number of mjtNums in sensor data vector
    narena: number of bytes in the mjData arena (inclusive of stack)
    opt: physics options
    stat: model statistics
    qpos0: qpos values at default pose                        (nq,)
    qpos_spring: reference pose for springs                   (nq,)
    body_parentid: id of body's parent                        (nbody,)
    body_rootid: id of root above body                        (nbody,)
    body_weldid: id of body that this body is welded to       (nbody,)
    body_jntnum: number of joints for this body               (nbody,)
    body_jntadr: start addr of joints; -1: no joints          (nbody,)
    body_dofnum: number of motion degrees of freedom          (nbody,)
    body_dofadr: start addr of dofs; -1: no dofs              (nbody,)
    body_treeid: id of body's kinematic tree; -1: static      (nbody,)
    body_geomnum: number of geoms                             (nbody,)
    body_geomadr: start addr of geoms; -1: no geoms           (nbody,)
    body_simple: 1: diag M; 2: diag M, sliders only           (nbody,)
    body_pos: position offset rel. to parent body             (nbody, 3)
    body_quat: orientation offset rel. to parent body         (nbody, 4)
    body_ipos: local position of center of mass               (nbody, 3)
    body_iquat: local orientation of inertia ellipsoid        (nbody, 4)
    body_mass: mass                                           (nbody,)
    body_subtreemass: mass of subtree starting at this body   (nbody,)
    body_inertia: diagonal inertia in ipos/iquat frame        (nbody, 3)
    body_gravcomp: antigravity force, units of body weight    (nbody,)
    body_margin: MAX over all geom margins                    (nbody,)
    body_contype: OR over all geom contypes                   (nbody,)
    body_conaffinity: OR over all geom conaffinities          (nbody,)
    body_bvhadr: address of bvh root                          (nbody,)
    body_bvhnum: number of bounding volumes                   (nbody,)
    bvh_child: left and right children in tree                (nbvh, 2)
    bvh_nodeid: geom or elem id of node; -1: non-leaf         (nbvh,)
    bvh_aabb: local bounding box (center, size)               (nbvhstatic, 6)
    body_invweight0: mean inv inert in qpos0 (trn, rot)       (nbody, 2)
    jnt_type: type of joint (mjtJoint)                        (njnt,)
    jnt_qposadr: start addr in 'qpos' for joint's data        (njnt,)
    jnt_dofadr: start addr in 'qvel' for joint's data         (njnt,)
    jnt_bodyid: id of joint's body                            (njnt,)
    jnt_group: group for visibility                           (njnt,)
    jnt_limited: does joint have limits                       (njnt,)
    jnt_actfrclimited: does joint have actuator force limits  (njnt,)
    jnt_actgravcomp: is gravcomp force applied via actuators  (njnt,)
    jnt_solref: constraint solver reference: limit            (njnt, mjNREF)
    jnt_solimp: constraint solver impedance: limit            (njnt, mjNIMP)
    jnt_pos: local anchor position                            (njnt, 3)
    jnt_axis: local joint axis                                (njnt, 3)
    jnt_stiffness: stiffness coefficient                      (njnt,)
    jnt_range: joint limits                                   (njnt, 2)
    jnt_actfrcrange: range of total actuator force            (njnt, 2)
    jnt_margin: min distance for limit detection              (njnt,)
    dof_bodyid: id of dof's body                              (nv,)
    dof_jntid: id of dof's joint                              (nv,)
    dof_parentid: id of dof's parent; -1: none                (nv,)
    dof_treeid: id of dof's kinematic tree                    (nv,)
    dof_Madr: dof address in M-diagonal                       (nv,)
    dof_simplenum: number of consecutive simple dofs          (nv,)
    dof_solref: constraint solver reference:frictionloss      (nv, mjNREF)
    dof_solimp: constraint solver impedance:frictionloss      (nv, mjNIMP)
    dof_frictionloss: dof friction loss                       (nv,)
    dof_hasfrictionloss: dof has >0 frictionloss (MJX)        (nv,)
    dof_armature: dof armature inertia/mass                   (nv,)
    dof_damping: damping coefficient                          (nv,)
    dof_invweight0: diag. inverse inertia in qpos0            (nv,)
    dof_M0: diag. inertia in qpos0                            (nv,)
    geom_type: geometric type (mjtGeom)                       (ngeom,)
    geom_contype: geom contact type                           (ngeom,)
    geom_conaffinity: geom contact affinity                   (ngeom,)
    geom_condim: contact dimensionality (1, 3, 4, 6)          (ngeom,)
    geom_bodyid: id of geom's body                            (ngeom,)
    geom_dataid: id of geom's mesh/hfield; -1: none           (ngeom,)
    geom_group: group for visibility                          (ngeom,)
    geom_matid: material id for rendering                     (ngeom,)
    geom_priority: geom contact priority                      (ngeom,)
    geom_solmix: mixing coef for solref/imp in geom pair      (ngeom,)
    geom_solref: constraint solver reference: contact         (ngeom, mjNREF)
    geom_solimp: constraint solver impedance: contact         (ngeom, mjNIMP)
    geom_size: geom-specific size parameters                  (ngeom, 3)
    geom_aabb: bounding box, (center, size)                   (ngeom, 6)
    geom_rbound: radius of bounding sphere                    (ngeom,)
    geom_rbound_hfield: static rbound for hfield grid bounds  (ngeom,)
    geom_pos: local position offset rel. to body              (ngeom, 3)
    geom_quat: local orientation offset rel. to body          (ngeom, 4)
    geom_friction: friction for (slide, spin, roll)           (ngeom, 3)
    geom_margin: include in solver if dist<margin-gap         (ngeom,)
    geom_gap: include in solver if dist<margin-gap            (ngeom,)
    geom_rgba: rgba when material is omitted                  (ngeom, 4)
    site_bodyid: id of site's body                            (nsite,)
    site_pos: local position offset rel. to body              (nsite, 3)
    site_quat: local orientation offset rel. to body          (nsite, 4)
    cam_mode:  camera tracking mode (mjtCamLight)             (ncam,)
    cam_bodyid:  id of camera's body                          (ncam,)
    cam_targetbodyid:  id of targeted body; -1: none          (ncam,)
    cam_pos:  position rel. to body frame                     (ncam, 3)
    cam_quat:  orientation rel. to body frame                 (ncam, 4)
    cam_poscom0:  global position rel. to sub-com in qpos0    (ncam, 3)
    cam_pos0: global position rel. to body in qpos0           (ncam, 3)
    cam_mat0: global orientation in qpos0                     (ncam, 3, 3)
    cam_fovy: y field-of-view                                 (ncam,)
    cam_resolution: resolution: pixels                        (ncam, 2)
    cam_sensorsize: sensor size: length                       (ncam, 2)
    cam_intrinsic: [focal length; principal point]            (ncam, 4)
    light_mode: light tracking mode (mjtCamLight)             (nlight,)
    light_bodyid: id of light's body                          (nlight,)
    light_targetbodyid: id of targeted body; -1: none         (nlight,)
    light_directional: directional light                      (nlight,)
    light_pos: position rel. to body frame                    (nlight, 3)
    light_dir: direction rel. to body frame                   (nlight, 3)
    light_poscom0: global position rel. to sub-com in qpos0   (nlight, 3)
    light_pos0: global position rel. to body in qpos0         (nlight, 3)
    light_dir0: global direction in qpos0                     (nlight, 3)
    flex_contype: flex contact type                           (nflex,)
    flex_conaffinity: flex contact affinity                   (nflex,)
    flex_condim: contact dimensionality (1, 3, 4, 6)          (nflex,)
    flex_priority: flex contact priority                      (nflex,)
    flex_solmix: mix coef for solref/imp in contact pair      (nflex,)
    flex_solref: constraint solver reference: contact         (nflex, mjNREF)
    flex_solimp: constraint solver impedance: contact         (nflex, mjNIMP)
    flex_friction: friction for (slide, spin, roll)           (nflex,)
    flex_margin: detect contact if dist<margin                (nflex,)
    flex_gap: include in solver if dist<margin-gap            (nflex,)
    flex_internal: internal flex collision enabled            (nflex,)
    flex_selfcollide: self collision mode (mjtFlexSelf)       (nflex,)
    flex_activelayers: number of active element layers, 3D only  (nflex,)
    flex_dim: 1: lines, 2: triangles, 3: tetrahedra           (nflex,)
    flex_vertadr: first vertex address                        (nflex,)
    flex_vertnum: number of vertices                          (nflex,)
    flex_edgeadr: first edge address                          (nflex,)
    flex_edgenum: number of edges                             (nflex,)
    flex_elemadr: first element address                       (nflex,)
    flex_elemnum: number of elements                          (nflex,)
    flex_elemdataadr: first element vertex id address         (nflex,)
    flex_evpairadr: first evpair address                      (nflex,)
    flex_evpairnum: number of evpairs                         (nflex,)
    flex_vertbodyid: vertex body ids                          (nflex,)
    flex_edge: edge vertex ids (2 per edge)                   (nflexedge, 2)
    flex_elem: element vertex ids (dim+1 per elem)            (nflexelemdata,)
    flex_elemlayer: element distance from surface, 3D only    (nflexelem,)
    flex_evpair: (element, vertex) collision pairs            (nflexevpair, 2)
    flex_vert: vertex positions in local body frames          (nflexvert, 3)
    flexedge_length0: edge lengths in qpos0                   (nflexedge,)
    flexedge_invweight0: edge inv. weight in qpos0            (nflexedge,)
    flex_radius: radius around primitive element              (nflex,)
    flex_edgestiffness: edge stiffness                        (nflex,)
    flex_edgedamping: edge damping                            (nflex,)
    flex_edgeequality: is edge equality constraint defined    (nflex,)
    flex_rigid: are all verices in the same body              (nflex,)
    flexedge_rigid: are both edge vertices in same body       (nflexedge,)
    flex_centered: are all vertex coordinates (0,0,0)         (nflex,)
    flex_bvhadr: address of bvh root; -1: no bvh              (nflex,)
    flex_bvhnum: number of bounding volumes                   (nflex,)
    mesh_vertadr: first vertex address                        (nmesh,)
    mesh_vertnum: number of vertices                          (nmesh,)
    mesh_faceadr: first face address                          (nmesh,)
    mesh_bvhadr: address of bvh root                          (nmesh,)
    mesh_bvhnum: number of bvh                                (nmesh,)
    mesh_graphadr: graph data address; -1: no graph           (nmesh,)
    mesh_vert: vertex positions for all meshes                (nmeshvert, 3)
    mesh_face: vertex face data                               (nmeshface, 3)
    mesh_graph: convex graph data                             (nmeshgraph,)
    mesh_pos: translation applied to asset vertices           (nmesh, 3)
    mesh_quat: rotation applied to asset vertices             (nmesh, 4)
    mesh_convex: pre-compiled convex mesh info for MJX        (nmesh,)
    mesh_texcoordadr: texcoord data address; -1: no texcoord  (nmesh,)
    mesh_texcoordnum: number of texcoord                      (nmesh,)
    mesh_texcoord: vertex texcoords for all meshes            (nmeshtexcoord, 2)
    hfield_size: (x, y, z_top, z_bottom)                      (nhfield,)
    hfield_nrow: number of rows in grid                       (nhfield,)
    hfield_ncol: number of columns in grid                    (nhfield,)
    hfield_adr: address in hfield_data                        (nhfield,)
    hfield_data: elevation data                               (nhfielddata,)
    tex_type: texture type (mjtTexture)                       (ntex,)
    tex_height: number of rows in texture image               (ntex,)
    tex_width: number of columns in texture image             (ntex,)
    tex_nchannel: number of channels in texture image         (ntex,)
    tex_adr: start address in tex_data                        (ntex,)
    tex_data: pixel values                                    (ntexdata,)
    mat_rgba: rgba                                            (nmat, 4)
    mat_texid: indices of textures; -1: none                  (nmat, mjNTEXROLE)
    pair_dim: contact dimensionality                          (npair,)
    pair_geom1: id of geom1                                   (npair,)
    pair_geom2: id of geom2                                   (npair,)
    pair_signature: body1 << 16 + body2                       (npair,)
    pair_solref: solver reference: contact normal             (npair, mjNREF)
    pair_solreffriction: solver reference: contact friction   (npair, mjNREF)
    pair_solimp: solver impedance: contact                    (npair, mjNIMP)
    pair_margin: include in solver if dist<margin-gap         (npair,)
    pair_gap: include in solver if dist<margin-gap            (npair,)
    pair_friction: tangent1, 2, spin, roll1, 2                (npair, 5)
    exclude_signature: (body1+1) << 16 + body2+1              (nexclude,)
    eq_type: constraint type (mjtEq)                          (neq,)
    eq_obj1id: id of object 1                                 (neq,)
    eq_obj2id: id of object 2                                 (neq,)
    eq_objtype: type of both objects (mjtObj)                 (neq,)
    eq_active0: initial enable/disable constraint state       (neq,)
    eq_solref: constraint solver reference                    (neq, mjNREF)
    eq_solimp: constraint solver impedance                    (neq, mjNIMP)
    eq_data: numeric data for constraint                      (neq, mjNEQDATA)
    tendon_adr: address of first object in tendon's path      (ntendon,)
    tendon_num: number of objects in tendon's path            (ntendon,)
    tendon_limited: does tendon have length limits            (ntendon,)
    tendon_solref_lim: constraint solver reference: limit     (ntendon, mjNREF)
    tendon_solimp_lim: constraint solver impedance: limit     (ntendon, mjNIMP)
    tendon_solref_fri: constraint solver reference: friction  (ntendon, mjNREF)
    tendon_solimp_fri: constraint solver impedance: friction  (ntendon, mjNIMP)
    tendon_range: tendon length limits                        (ntendon, 2)
    tendon_margin: min distance for limit detection           (ntendon,)
    tendon_stiffness: stiffness coefficient                   (ntendon,)
    tendon_damping: damping coefficient                       (ntendon,)
    tendon_frictionloss: loss due to friction                 (ntendon,)
    tendon_lengthspring: spring resting length range          (ntendon, 2)
    tendon_length0: tendon length in qpos0                    (ntendon,)
    tendon_invweight0: inv. weight in qpos0                   (ntendon,)
    tendon_hasfrictionloss: tendon has >0 frictionloss (MJX)  (ntendon,)
    wrap_type: wrap object type (mjtWrap)                     (nwrap,)
    wrap_objid: object id: geom, site, joint                  (nwrap,)
    wrap_prm: divisor, joint coef, or site id                 (nwrap,)
    actuator_trntype: transmission type (mjtTrn)              (nu,)
    actuator_dyntype: dynamics type (mjtDyn)                  (nu,)
    actuator_gaintype: gain type (mjtGain)                    (nu,)
    actuator_biastype: bias type (mjtBias)                    (nu,)
    actuator_trnid: transmission id: joint, tendon, site      (nu, 2)
    actuator_actadr: first activation address; -1: stateless  (nu,)
    actuator_actnum: number of activation variables           (nu,)
    actuator_group: group for visibility                      (nu,)
    actuator_ctrllimited: is control limited                  (nu,)
    actuator_forcelimited: is force limited                   (nu,)
    actuator_actlimited: is activation limited                (nu,)
    actuator_dynprm: dynamics parameters                      (nu, mjNDYN)
    actuator_gainprm: gain parameters                         (nu, mjNGAIN)
    actuator_biasprm: bias parameters                         (nu, mjNBIAS)
    actuator_actearly: step activation before force           (nu,)
    actuator_ctrlrange: range of controls                     (nu, 2)
    actuator_forcerange: range of forces                      (nu, 2)
    actuator_actrange: range of activations                   (nu, 2)
    actuator_gear: scale length and transmitted force         (nu, 6)
    actuator_cranklength: crank length for slider-crank       (nu,)
    actuator_acc0: acceleration from unit force in qpos0      (nu,)
    actuator_lengthrange: feasible actuator length range      (nu, 2)
    sensor_type: sensor type (mjtSensor)                      (nsensor,)
    sensor_datatype: numeric data type (mjtDataType)          (nsensor,)
    sensor_needstage: required compute stage (mjtStage)       (nsensor,)
    sensor_objtype: type of sensorized object (mjtObj)        (nsensor,)
    sensor_objid: id of sensorized object                     (nsensor,)
    sensor_reftype: type of reference frame (mjtObj)          (nsensor,)
    sensor_refid: id of reference frame; -1: global frame     (nsensor,)
    sensor_dim: number of scalar outputs                      (nsensor,)
    sensor_adr: address in sensor array                       (nsensor,)
    sensor_cutoff: cutoff for real and positive; 0: ignore    (nsensor,)
    numeric_adr: address of field in numeric_data             (nnumeric,)
    numeric_data: array of all numeric fields                 (nnumericdata,)
    tuple_adr: address of text in text_data                   (ntuple,)
    tuple_size: number of objects in tuple                    (ntuple,)
    tuple_objtype: array of object types in all tuples        (ntupledata,)
    tuple_objid: array of object ids in all tuples            (ntupledata,)
    tuple_objprm: array of object params in all tuples        (ntupledata,)
    name_bodyadr: body name pointers                          (nbody,)
    name_jntadr: joint name pointers                          (njnt,)
    name_geomadr: geom name pointers                          (ngeom,)
    name_siteadr: site name pointers                          (nsite,)
    name_camadr: camera name pointers                         (ncam,)
    name_meshadr: mesh name pointers                          (nmesh,)
    name_pairadr: geom pair name pointers                     (npair,)
    name_eqadr: equality constraint name pointers             (neq,)
    name_tendonadr: tendon name pointers                      (ntendon,)
    name_actuatoradr: actuator name pointers                  (nu,)
    name_sensoradr: sensor name pointers                      (nsensor,)
    name_numericadr: numeric name pointers                    (nnumeric,)
    name_tupleadr: tuple name pointers                        (ntuple,)
    name_keyadr: keyframe name pointers                       (nkey,)
    names: names of all objects, 0-terminated                 (nnames,)
  """

  nq: int
  nv: int
  nu: int
  na: int
  nbody: int
  nbvh: int = _restricted_to('mujoco')
  nbvhstatic: int = _restricted_to('mujoco')
  nbvhdynamic: int = _restricted_to('mujoco')
  njnt: int
  ngeom: int
  nsite: int
  ncam: int
  nlight: int
  nflex: int = _restricted_to('mujoco')
  nflexvert: int = _restricted_to('mujoco')
  nflexedge: int = _restricted_to('mujoco')
  nflexelem: int = _restricted_to('mujoco')
  nflexelemdata: int = _restricted_to('mujoco')
  nflexshelldata: int = _restricted_to('mujoco')
  nflexevpair: int = _restricted_to('mujoco')
  nflextexcoord: int = _restricted_to('mujoco')
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
  nJmom: int
  ntree: int = _restricted_to('mujoco')
  ngravcomp: int
  nuserdata: int
  nsensordata: int
  narena: int = _restricted_to('mujoco')
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
  body_bvhadr: np.ndarray = _restricted_to('mujoco')
  body_bvhnum: np.ndarray = _restricted_to('mujoco')
  bvh_child: np.ndarray = _restricted_to('mujoco')
  bvh_nodeid: np.ndarray = _restricted_to('mujoco')
  bvh_aabb: np.ndarray = _restricted_to('mujoco')
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
  dof_hasfrictionloss: np.ndarray = _restricted_to('mjx')
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
  geom_rbound_hfield: np.ndarray = _restricted_to('mjx')
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
  light_bodyid: np.ndarray = _restricted_to('mujoco')
  light_targetbodyid: np.ndarray = _restricted_to('mujoco')
  light_directional: np.ndarray
  light_pos: jax.Array
  light_dir: jax.Array
  light_poscom0: np.ndarray = _restricted_to('mujoco')
  light_pos0: np.ndarray
  light_dir0: np.ndarray
  flex_contype: np.ndarray = _restricted_to('mujoco')
  flex_conaffinity: np.ndarray = _restricted_to('mujoco')
  flex_condim: np.ndarray = _restricted_to('mujoco')
  flex_priority: np.ndarray = _restricted_to('mujoco')
  flex_solmix: np.ndarray = _restricted_to('mujoco')
  flex_solref: np.ndarray = _restricted_to('mujoco')
  flex_solimp: np.ndarray = _restricted_to('mujoco')
  flex_friction: np.ndarray = _restricted_to('mujoco')
  flex_margin: np.ndarray = _restricted_to('mujoco')
  flex_gap: np.ndarray = _restricted_to('mujoco')
  flex_internal: np.ndarray = _restricted_to('mujoco')
  flex_selfcollide: np.ndarray = _restricted_to('mujoco')
  flex_activelayers: np.ndarray = _restricted_to('mujoco')
  flex_dim: np.ndarray = _restricted_to('mujoco')
  flex_vertadr: np.ndarray = _restricted_to('mujoco')
  flex_vertnum: np.ndarray = _restricted_to('mujoco')
  flex_edgeadr: np.ndarray = _restricted_to('mujoco')
  flex_edgenum: np.ndarray = _restricted_to('mujoco')
  flex_elemadr: np.ndarray = _restricted_to('mujoco')
  flex_elemnum: np.ndarray = _restricted_to('mujoco')
  flex_elemdataadr: np.ndarray = _restricted_to('mujoco')
  flex_evpairadr: np.ndarray = _restricted_to('mujoco')
  flex_evpairnum: np.ndarray = _restricted_to('mujoco')
  flex_vertbodyid: np.ndarray = _restricted_to('mujoco')
  flex_edge: np.ndarray = _restricted_to('mujoco')
  flex_elem: np.ndarray = _restricted_to('mujoco')
  flex_elemlayer: np.ndarray = _restricted_to('mujoco')
  flex_evpair: np.ndarray = _restricted_to('mujoco')
  flex_vert: np.ndarray = _restricted_to('mujoco')
  flexedge_length0: np.ndarray = _restricted_to('mujoco')
  flexedge_invweight0: np.ndarray = _restricted_to('mujoco')
  flex_radius: np.ndarray = _restricted_to('mujoco')
  flex_edgestiffness: np.ndarray = _restricted_to('mujoco')
  flex_edgedamping: np.ndarray = _restricted_to('mujoco')
  flex_edgeequality: np.ndarray = _restricted_to('mujoco')
  flex_rigid: np.ndarray = _restricted_to('mujoco')
  flexedge_rigid: np.ndarray = _restricted_to('mujoco')
  flex_centered: np.ndarray = _restricted_to('mujoco')
  flex_bvhadr: np.ndarray = _restricted_to('mujoco')
  flex_bvhnum: np.ndarray = _restricted_to('mujoco')
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
  mesh_convex: Tuple[ConvexMesh, ...] = _restricted_to('mjx')
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
  tendon_solref_lim: jax.Array
  tendon_solimp_lim: jax.Array
  tendon_solref_fri: jax.Array
  tendon_solimp_fri: jax.Array
  tendon_range: jax.Array
  tendon_margin: jax.Array
  tendon_stiffness: jax.Array
  tendon_damping: jax.Array
  tendon_frictionloss: jax.Array
  tendon_lengthspring: jax.Array
  tendon_length0: jax.Array
  tendon_invweight0: jax.Array
  tendon_hasfrictionloss: np.ndarray = _restricted_to('mjx')
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
  actuator_acc0: np.ndarray
  actuator_lengthrange: np.ndarray
  actuator_plugin: np.ndarray = _restricted_to('mujoco')
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
  _sizes: jax.Array


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


class Data(PyTreeNode):
  r"""\Dynamic state that updates each step.

  Attributes:
    ne: number of equality constraints
    nf: number of friction constraints
    nl: number of limit constraints
    nefc: number of constraints
    ncon: number of contacts
    solver_niter: number of solver iterations
    time: simulation time
    qpos: position                                              (nq,)
    qvel: velocity                                              (nv,)
    act: actuator activation                                    (na,)
    qacc_warmstart: acceleration used for warmstart             (nv,)
    ctrl: control                                               (nu,)
    qfrc_applied: applied generalized force                     (nv,)
    xfrc_applied: applied Cartesian force/torque                (nbody, 6)
    eq_active: enable/disable constraints                       (neq,)
    mocap_pos: positions of mocap bodies                        (nmocap x 3)
    mocap_quat: orientations of mocap bodies                    (nmocap x 4)
    qacc: acceleration                                          (nv,)
    act_dot: time-derivative of actuator activation             (na,)
    userdata: user data, not touched by engine                  (nuserdata,)
    sensordata: sensor data array                               (nsensordata,)
    xpos:  Cartesian position of body frame                     (nbody, 3)
    xquat: Cartesian orientation of body frame                  (nbody, 4)
    xmat:  Cartesian orientation of body frame                  (nbody, 3, 3)
    xipos: Cartesian position of body com                       (nbody, 3)
    ximat: Cartesian orientation of body inertia                (nbody, 3, 3)
    xanchor: Cartesian position of joint anchor                 (njnt, 3)
    xaxis: Cartesian joint axis                                 (njnt, 3)
    geom_xpos: Cartesian geom position                          (ngeom, 3)
    geom_xmat: Cartesian geom orientation                       (ngeom, 3, 3)
    site_xpos: Cartesian site position                          (nsite, 3)
    site_xmat: Cartesian site orientation                       (nsite, 9)
    cam_xpos: Cartesian camera position                         (ncam, 3)
    cam_xmat: Cartesian camera orientation                      (ncam, 3, 3)
    light_xpos: Cartesian light position                        (nlight, 3)
    light_xdir: Cartesian light direction                       (nlight, 3)
    subtree_com: center of mass of each subtree                 (nbody, 3)
    cdof: com-based motion axis of each dof                     (nv, 6)
    cinert: com-based body inertia and mass                     (nbody, 10)
    flexvert_xpos: Cartesian flex vertex positions              (nflexvert, 3)
    flexelem_aabb: flex element bounding boxes (center, size)   (nflexelem, 6)
    flexedge_J_rownnz: number of non-zeros in Jacobian row      (nflexedge,)
    flexedge_J_rowadr: row start address in colind array        (nflexedge,)
    flexedge_J_colind: column indices in sparse Jacobian        (nflexedge, nv)
    flexedge_J: flex edge Jacobian                              (nflexedge, nv)
    flexedge_length: flex edge lengths                          (nflexedge,)
    ten_wrapadr: start address of tendon's path                 (ntendon,)
    ten_wrapnum: number of wrap points in path                  (ntendon,)
    ten_J_rownnz: number of non-zeros in Jacobian row           (ntendon,)
    ten_J_rowadr: row start address in colind array             (ntendon,)
    ten_J_colind: column indices in sparse Jacobian             (ntendon, nv)
    ten_J: tendon Jacobian                                      (ntendon, nv)
    ten_length: tendon lengths                                  (ntendon,)
    wrap_obj: geom id; -1: site; -2: pulley                     (nwrap*2,)
    wrap_xpos: Cartesian 3D points in all path                  (nwrap*2, 3)
    actuator_length: actuator lengths                           (nu,)
    moment_rownnz: number of non-zeros in actuator_moment row   (nu,)
    moment_rowadr: row start address in colind array            (nu,)
    moment_colind: column indices in sparse Jacobian            (nJmom,)
    actuator_moment: actuator moments                           (nJmom,)
    crb: com-based composite inertia and mass                   (nbody, 10)
    qM: total inertia                                if sparse: (nM,)
                                                     if dense:  (nv, nv)
    qLD: L'*D*L (or Cholesky) factorization of M.    if sparse: (nM,)
                                                     if dense:  (nv, nv)
    qLDiagInv: 1/diag(D)                             if sparse: (nv,)
                                                     if dense:  (0,)
    qLDiagSqrtInv: 1/sqrt(diag(D))                              (nv,)
    bvh_aabb_dyn: global bounding box (center, size)            (nbvhdynamic, 6)
    bvh_active: volume has been added to collisions             (nbvh,)
    flexedge_velocity: flex edge velocities                     (nflexedge,)
    ten_velocity: tendon velocities                             (ntendon,)
    actuator_velocity: actuator velocities                      (nu,)
    cvel: com-based velocity [3D rot; 3D tran]                  (nbody, 6)
    cdof_dot: time-derivative of cdof                           (nv, 6)
    qfrc_bias: C(qpos,qvel)                                     (nv,)
    qfrc_spring: passive spring force                           (nv,)
    qfrc_damper: passive damper force                           (nv,)
    qfrc_gravcomp: passive gravity compensation force           (nv,)
    qfrc_fluid: passive fluid force                             (nv,)
    qfrc_passive: total passive force                           (nv,)
    subtree_linvel: linear velocity of subtree com              (nbody, 3)
    subtree_angmom: angular momentum about subtree com          (nbody, 3)
    qH: L'*D*L factorization of modified M                      (nM,)
    qHDiagInv: 1/diag(D) of modified M                          (nv,)
    B_rownnz: body-dof: non-zeros in each row                   (nbody,)
    B_rowadr: body-dof: address of each row in B_colind         (nbody,)
    B_colind: body-dof: column indices of non-zeros             (nB,)
    C_rownnz: reduced dof-dof: non-zeros in each row            (nv,)
    C_rowadr: reduced dof-dof: address of each row in C_colind  (nv,)
    C_diag: reduced dof-dof: index of diagonal element          (nv,)
    C_colind: reduced dof-dof: column indices of non-zeros      (nC,)
    mapM2C: index mapping from M to C                           (nC,)
    D_rownnz: dof-dof: non-zeros in each row                    (nv,)
    D_rowadr: dof-dof: address of each row in D_colind          (nv,)
    D_diag: dof-dof: index of diagonal element                  (nv,)
    D_colind: dof-dof: column indices of non-zeros              (nD,)
    mapM2D: index mapping from M to D                           (nD,)
    mapD2M: index mapping from D to M                           (nM,)
    qDeriv: d (passive + actuator - bias) / d qvel              (nD,)
    qLU: sparse LU of (qM - dt*qDeriv)                          (nD,)
    actuator_force: actuator force in actuation space           (nu,)
    qfrc_actuator: actuator force                               (nv,)
    qfrc_smooth: net unconstrained force                        (nv,)
    qacc_smooth: unconstrained acceleration                     (nv,)
    qfrc_constraint: constraint force                           (nv,)
    qfrc_inverse: net external force; should equal:             (nv,)
                  qfrc_applied + J'*xfrc_applied + qfrc_actuator
    cacc: com-based acceleration                                (nbody, 6)
    cfrc_int: com-based interaction force with parent           (nbody, 6)
    cfrc_ext: com-based external force on body                  (nbody, 6)
    contact: all detected contacts                              (ncon,)
    efc_type: constraint type                                   (nefc,)
    efc_J: constraint Jacobian                                  (nefc, nv)
    efc_pos: constraint position (equality, contact)            (nefc,)
    efc_margin: inclusion margin (contact)                      (nefc,)
    efc_frictionloss: frictionloss (friction)                   (nefc,)
    efc_D: constraint mass                                      (nefc,)
    efc_aref: reference pseudo-acceleration                     (nefc,)
    efc_force: constraint force in constraint space             (nefc,)
    _qM_sparse: qM in sparse representation                     (nM,)
    _qLD_sparse: qLD in sparse representation                   (nM,)
    _qLDiagInv_sparse: qLDiagInv in sparse representation       (nv,)
  """  # fmt: skip
  # constant sizes:
  ne: int
  nf: int
  nl: int
  nefc: int
  ncon: int
  # solver statistics:
  solver_niter: jax.Array
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
  light_xpos: jax.Array = _restricted_to('mujoco')
  light_xdir: jax.Array = _restricted_to('mujoco')
  subtree_com: jax.Array
  cdof: jax.Array
  cinert: jax.Array
  flexvert_xpos: jax.Array = _restricted_to('mujoco')
  flexelem_aabb: jax.Array
  flexedge_J_rownnz: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  flexedge_J_rowadr: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  flexedge_J_colind: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  flexedge_J: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  flexedge_length: jax.Array = _restricted_to('mujoco')
  ten_wrapadr: jax.Array
  ten_wrapnum: jax.Array
  ten_J_rownnz: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  ten_J_rowadr: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  ten_J_colind: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  ten_J: jax.Array  # pylint:disable=invalid-name
  ten_length: jax.Array
  wrap_obj: jax.Array
  wrap_xpos: jax.Array
  actuator_length: jax.Array
  moment_rownnz: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  moment_rowadr: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  moment_colind: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  actuator_moment: jax.Array
  crb: jax.Array
  qM: jax.Array  # pylint:disable=invalid-name
  qLD: jax.Array  # pylint:disable=invalid-name
  qLDiagInv: jax.Array  # pylint:disable=invalid-name
  qLDiagSqrtInv: jax.Array  # pylint:disable=invalid-name
  bvh_aabb_dyn: jax.Array = _restricted_to('mujoco')
  bvh_active: jax.Array = _restricted_to('mujoco')
  # position, velocity dependent:
  flexedge_velocity: jax.Array = _restricted_to('mujoco')
  ten_velocity: jax.Array
  actuator_velocity: jax.Array
  cvel: jax.Array
  cdof_dot: jax.Array
  qfrc_bias: jax.Array
  qfrc_spring: jax.Array = _restricted_to('mujoco')
  qfrc_damper: jax.Array = _restricted_to('mujoco')
  qfrc_gravcomp: jax.Array
  qfrc_fluid: jax.Array
  qfrc_passive: jax.Array
  subtree_linvel: jax.Array
  subtree_angmom: jax.Array
  qH: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  qHDiagInv: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  B_rownnz: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  B_rowadr: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  B_colind: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  C_rownnz: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  C_rowadr: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  C_diag: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  C_colind: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  mapM2C: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  D_rownnz: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  D_rowadr: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  D_diag: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  D_colind: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  mapM2D: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  mapD2M: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  qDeriv: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  qLU: jax.Array = _restricted_to('mujoco')  # pylint:disable=invalid-name
  # position, velocity, control & acceleration dependent:
  qfrc_actuator: jax.Array
  actuator_force: jax.Array
  qfrc_smooth: jax.Array
  qacc_smooth: jax.Array
  qfrc_constraint: jax.Array
  qfrc_inverse: jax.Array
  cacc: jax.Array
  cfrc_int: jax.Array
  cfrc_ext: jax.Array
  # dynamically sized
  contact: Contact
  # dynamically sized - position dependent:
  efc_type: jax.Array
  efc_J: jax.Array  # pylint:disable=invalid-name
  efc_pos: jax.Array
  efc_margin: jax.Array
  efc_frictionloss: jax.Array
  efc_D: jax.Array  # pylint:disable=invalid-name
  # dynamically sized - position & velocity dependent:
  efc_aref: jax.Array
  # dynamically sized - position, velocity, control & acceleration dependent:
  efc_force: jax.Array
  # sparse representation of qM, qLD, qLDiagInv, for compatibility with MuJoCo
  # when in dense mode
  _qM_sparse: jax.Array = _restricted_to('mjx')  # pylint:disable=invalid-name
  _qLD_sparse: jax.Array = _restricted_to('mjx')  # pylint:disable=invalid-name
  _qLDiagInv_sparse: jax.Array = _restricted_to('mjx')  # pylint:disable=invalid-name
