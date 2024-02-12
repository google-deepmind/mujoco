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
from typing import Sequence

import jax
import jax.numpy as jp
import mujoco
from mujoco.mjx._src.dataclasses import PyTreeNode  # pylint: disable=g-importing-member
import numpy as np


class DisableBit(enum.IntFlag):
  """Disable default feature bitflags.

  Attributes:
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
  """
  CONSTRAINT = mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
  EQUALITY = mujoco.mjtDisableBit.mjDSBL_EQUALITY
  LIMIT = mujoco.mjtDisableBit.mjDSBL_LIMIT
  CONTACT = mujoco.mjtDisableBit.mjDSBL_CONTACT
  PASSIVE = mujoco.mjtDisableBit.mjDSBL_PASSIVE
  GRAVITY = mujoco.mjtDisableBit.mjDSBL_GRAVITY
  CLAMPCTRL = mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL
  WARMSTART = mujoco.mjtDisableBit.mjDSBL_WARMSTART
  ACTUATION = mujoco.mjtDisableBit.mjDSBL_ACTUATION
  REFSAFE = mujoco.mjtDisableBit.mjDSBL_REFSAFE
  EULERDAMP = mujoco.mjtDisableBit.mjDSBL_EULERDAMP
  FILTERPARENT = mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
  # unsupported: FRICTIONLOSS, SENSOR, MIDPHASE


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

  def dof_width(self) -> int:
    return {0: 6, 1: 3, 2: 1, 3: 1}[self.value]

  def qpos_width(self) -> int:
    return {0: 7, 1: 4, 2: 1, 3: 1}[self.value]


class IntegratorType(enum.IntEnum):
  """Integrator mode.

  Attributes:
    EULER: semi-implicit Euler
    RK4: 4th-order Runge Kutta
  """
  EULER = mujoco.mjtIntegrator.mjINT_EULER
  RK4 = mujoco.mjtIntegrator.mjINT_RK4
  # unsupported: IMPLICIT, IMPLICITFAST


class GeomType(enum.IntEnum):
  """Type of geometry.

  Attributes:
    PLANE: plane
    HFIELD: height field
    SPHERE: sphere
    CAPSULE: capsule
    ELLIPSOID: ellipsoid
    CYLINDER: cylinder
    BOX: box
    MESH: mesh
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


class ConeType(enum.IntEnum):
  """Type of friction cone.

  Attributes:
    PYRAMIDAL: pyramidal
  """
  PYRAMIDAL = mujoco.mjtCone.mjCONE_PYRAMIDAL
  # unsupported: ELLIPTIC


class JacobianType(enum.IntEnum):
  """Type of constraint Jacobian.

  Attributes:
    DENSE: dense
    SPARSE: sparse
    AUTO: sparse if nv>60 and device is TPU, dense otherwise
  """
  DENSE = mujoco.mjtJacobian.mjJAC_DENSE
  SPARSE = mujoco.mjtJacobian.mjJAC_SPARSE
  AUTO = mujoco.mjtJacobian.mjJAC_AUTO


class SolverType(enum.IntEnum):
  """Constraint solver algorithm.

  Attributes:
    CG: Conjugate gradient (primal)
  """
  # unsupported: PGS
  CG = mujoco.mjtSolver.mjSOL_CG
  NEWTON = mujoco.mjtSolver.mjSOL_NEWTON


class EqType(enum.IntEnum):
  """Type of equality constraint.

  Attributes:
    CONNECT: connect two bodies at a point (ball joint)
    WELD: fix relative position and orientation of two bodies
    JOINT: couple the values of two scalar joints with cubic
  """
  CONNECT = mujoco.mjtEq.mjEQ_CONNECT
  WELD = mujoco.mjtEq.mjEQ_WELD
  JOINT = mujoco.mjtEq.mjEQ_JOINT
  # unsupported: TENDON, DISTANCE


class TrnType(enum.IntEnum):
  """Type of actuator transmission.

  Attributes:
    JOINT: force on joint
    SITE: force on site
  """
  JOINT = mujoco.mjtTrn.mjTRN_JOINT
  SITE = mujoco.mjtTrn.mjTRN_SITE
  # unsupported: JOINTINPARENT, SLIDERCRANK, TENDON, BODY


class DynType(enum.IntEnum):
  """Type of actuator dynamics.

  Attributes:
    NONE: no internal dynamics; ctrl specifies force
    INTEGRATOR: integrator: da/dt = u
    FILTER: linear filter: da/dt = (u-a) / tau
    FILTEREXACT: linear filter: da/dt = (u-a) / tau, with exact integration
  """
  NONE = mujoco.mjtDyn.mjDYN_NONE
  INTEGRATOR = mujoco.mjtDyn.mjDYN_INTEGRATOR
  FILTER = mujoco.mjtDyn.mjDYN_FILTER
  FILTEREXACT = mujoco.mjtDyn.mjDYN_FILTEREXACT
  # unsupported: MUSCLE, USER


class GainType(enum.IntEnum):
  """Type of actuator gain.

  Attributes:
    FIXED: fixed gain
    AFFINE: const + kp*length + kv*velocity
  """
  FIXED = mujoco.mjtGain.mjGAIN_FIXED
  AFFINE = mujoco.mjtGain.mjGAIN_AFFINE
  # unsupported: MUSCLE, USER


class BiasType(enum.IntEnum):
  """Type of actuator bias.

  Attributes:
    NONE: no bias
    AFFINE: const + kp*length + kv*velocity
  """
  NONE = mujoco.mjtBias.mjBIAS_NONE
  AFFINE = mujoco.mjtBias.mjBIAS_AFFINE
  # unsupported: MUSCLE, USER


class Option(PyTreeNode):
  """Physics options.

  Attributes:
    timestep:         timestep
    tolerance:        main solver tolerance
    ls_tolerance:     CG/Newton linesearch tolerance
    gravity:          gravitational acceleration                 (3,)
    wind:             wind (for lift, drag and viscosity)
    density:          density of medium
    viscosity:        viscosity of medium
    has_fluid_params: automatically set by mjx if wind/density/viscosity are
      nonzero. Not used by mj
    integrator:       integration mode
    cone:             type of friction cone
    jacobian:         matrix layout for mass matrices (dense or sparse)
                      (note that this is different from MuJoCo, where jacobian
                      specifies whether efc_J and its accompanying matrices
                      are dense or sparse.
    solver:           solver algorithm
    iterations:       number of main solver iterations
    ls_iterations:    maximum number of CG/Newton linesearch iterations
    disableflags:     bit flags for disabling standard features
  """
  timestep: jax.Array
  tolerance: jax.Array
  ls_tolerance: jax.Array
  # unsupported: apirate, impratio, noslip_tolerance, mpr_tolerance
  gravity: jax.Array
  wind: jax.Array
  density: jax.Array
  viscosity: jax.Array
  has_fluid_params: bool
  # unsupported: magnetic, o_margin, o_solref, o_solimp
  integrator: IntegratorType
  cone: ConeType
  jacobian: JacobianType
  solver: SolverType
  iterations: int
  ls_iterations: int
  # unsupported: noslip_iterations, mpr_iterations
  disableflags: DisableBit
  # unsupported: enableflags


class Statistic(PyTreeNode):
  """Model statistics (in qpos0).

  Attributes:
    meaninertia: mean diagonal inertia
  """
  meaninertia: jax.Array
  # unsupported: meanmass, meansize, extent, center


class Model(PyTreeNode):
  """Static model of the scene that remains unchanged with each physics step.

  Attributes:
    nq: number of generalized coordinates = dim(qpos)
    nv: number of degrees of freedom = dim(qvel)
    nu: number of actuators/controls = dim(ctrl)
    na: number of activation states = dim(act)
    nbody: number of bodies
    njnt: number of joints
    ngeom: number of geoms
    nsite: number of sites
    nmesh: number of meshes
    nmeshvert: number of vertices in all meshes
    nmeshface: number of triangular faces in all meshes
    nmat: number of materials
    npair: number of predefined geom pairs
    nexclude: number of excluded geom pairs
    neq: number of equality constraints
    nnumeric: number of numeric custom fields
    nM: number of non-zeros in sparse inertia matrix
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
    body_geomnum: number of geoms                             (nbody,)
    body_geomadr: start addr of geoms; -1: no geoms           (nbody,)
    body_pos: position offset rel. to parent body             (nbody, 3)
    body_quat: orientation offset rel. to parent body         (nbody, 4)
    body_ipos: local position of center of mass               (nbody, 3)
    body_iquat: local orientation of inertia ellipsoid        (nbody, 4)
    body_mass: mass                                           (nbody,)
    body_subtreemass: mass of subtree starting at this body   (nbody,)
    body_inertia: diagonal inertia in ipos/iquat frame        (nbody, 3)
    body_invweight0: mean inv inert in qpos0 (trn, rot)       (nbody, 2)
    jnt_type: type of joint (mjtJoint)                        (njnt,)
    jnt_qposadr: start addr in 'qpos' for joint's data        (njnt,)
    jnt_dofadr: start addr in 'qvel' for joint's data         (njnt,)
    jnt_bodyid: id of joint's body                            (njnt,)
    jnt_group: group for visibility                           (njnt,)
    jnt_limited: does joint have limits                       (njnt,)
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
    dof_Madr: dof address in M-diagonal                       (nv,)
    dof_solref: constraint solver reference:frictionloss      (nv, mjNREF)
    dof_solimp: constraint solver impedance:frictionloss      (nv, mjNIMP)
    dof_frictionloss: dof friction loss                       (nv,)
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
    geom_pos: local position offset rel. to body              (ngeom, 3)
    geom_quat: local orientation offset rel. to body          (ngeom, 4)
    geom_friction: friction for (slide, spin, roll)           (ngeom, 3)
    geom_margin: include in solver if dist<margin-gap         (ngeom,)
    geom_gap: include in solver if dist<margin-gap            (ngeom,)
    geom_rgba: rgba when material is omitted                  (ngeom, 4)
    site_bodyid: id of site's body                            (nsite,)
    site_pos: local position offset rel. to body              (nsite, 3)
    site_quat: local orientation offset rel. to body          (nsite, 4)
    mat_rgba: rgba                                            (nmat, 4)
    mesh_vertadr: first vertex address                        (nmesh x 1)
    mesh_faceadr: first face address                          (nmesh x 1)
    mesh_vert: vertex positions for all meshes                (nmeshvert, 3)
    mesh_face: vertex face data                               (nmeshface, 3)
    geom_convex_face: vertex face data, MJX only              (ngeom,)
    geom_convex_vert: vertex data, MJX only                   (ngeom,)
    geom_convex_edge: unique edge data, MJX only              (ngeom,)
    geom_convex_facenormal: normal face data, MJX only        (ngeom,)
    pair_dim: contact dimensionality                          (npair,)
    pair_geom1: id of geom1                                   (npair,)
    pair_geom2: id of geom2                                   (npair,)
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
    eq_active0: initial enable/disable constraint state       (neq,)
    eq_solref: constraint solver reference                    (neq, mjNREF)
    eq_solimp: constraint solver impedance                    (neq, mjNIMP)
    eq_data: numeric data for constraint                      (neq, mjNEQDATA)
    actuator_trntype: transmission type (mjtTrn)              (nu,)
    actuator_dyntype: dynamics type (mjtDyn)                  (nu,)
    actuator_gaintype: gain type (mjtGain)                    (nu,)
    actuator_biastype: bias type (mjtBias)                    (nu,)
    actuator_trnid: transmission id: joint, tendon, site      (nu, 2)
    actuator_actadr: first activation address; -1: stateless  (nu,)
    actuator_actnum: number of activation variables           (nu,)
    actuator_ctrllimited: is control limited                  (nu,)
    actuator_forcelimited: is force limited                   (nu,)
    actuator_actlimited: is activation limited                (nu,)
    actuator_dynprm: dynamics parameters                      (nu, mjNDYN)
    actuator_gainprm: gain parameters                         (nu, mjNGAIN)
    actuator_biasprm: bias parameters                         (nu, mjNBIAS)
    actuator_ctrlrange: range of controls                     (nu, 2)
    actuator_forcerange: range of forces                      (nu, 2)
    actuator_actrange: range of activations                   (nu, 2)
    actuator_gear: scale length and transmitted force         (nu, 6)
    numeric_adr: address of field in numeric_data             (nnumeric,)
    numeric_data: array of all numeric fields                 (nnumericdata,)
    name_numericadr: numeric name pointers                    (nnumeric,)
    names: names of all objects, 0-terminated                 (nnames,)
  """
  nq: int
  nv: int
  nu: int
  na: int
  nbody: int
  njnt: int
  ngeom: int
  nsite: int
  nmesh: int
  nmeshvert: int
  nmeshface: int
  nmat: int
  npair: int
  nexclude: int
  neq: int
  nnumeric: int
  nM: int  # pylint:disable=invalid-name
  opt: Option
  stat: Statistic
  qpos0: jax.Array
  qpos_spring: jax.Array
  body_parentid: np.ndarray
  body_rootid: np.ndarray
  body_weldid: np.ndarray
  body_jntnum: np.ndarray
  body_jntadr: np.ndarray
  body_dofnum: np.ndarray
  body_dofadr: np.ndarray
  body_geomnum: np.ndarray
  body_geomadr: np.ndarray
  body_pos: jax.Array
  body_quat: jax.Array
  body_ipos: jax.Array
  body_iquat: jax.Array
  body_mass: jax.Array
  body_subtreemass: jax.Array
  body_inertia: jax.Array
  body_invweight0: jax.Array
  jnt_type: np.ndarray
  jnt_qposadr: np.ndarray
  jnt_dofadr: np.ndarray
  jnt_bodyid: np.ndarray
  jnt_limited: np.ndarray
  jnt_actfrclimited: np.ndarray
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
  dof_Madr: np.ndarray  # pylint:disable=invalid-name
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
  geom_dataid: np.ndarray
  geom_group: np.ndarray
  geom_matid: np.ndarray
  geom_priority: np.ndarray
  geom_solmix: jax.Array
  geom_solref: jax.Array
  geom_solimp: jax.Array
  geom_size: jax.Array
  geom_pos: jax.Array
  geom_quat: jax.Array
  geom_friction: jax.Array
  geom_margin: jax.Array
  geom_gap: jax.Array
  geom_rgba: np.ndarray
  site_bodyid: np.ndarray
  site_pos: jax.Array
  site_quat: jax.Array
  mesh_vertadr: np.ndarray
  mesh_faceadr: np.ndarray
  mesh_vert: np.ndarray
  mesh_face: np.ndarray
  mat_rgba: np.ndarray
  pair_dim: np.ndarray
  pair_geom1: np.ndarray
  pair_geom2: np.ndarray
  geom_convex_face: Sequence[jax.Array]
  geom_convex_vert: Sequence[jax.Array]
  geom_convex_edge: Sequence[jax.Array]
  geom_convex_facenormal: Sequence[jax.Array]
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
  eq_active0: np.ndarray
  eq_solref: jax.Array
  eq_solimp: jax.Array
  eq_data: jax.Array
  actuator_trntype: np.ndarray
  actuator_dyntype: np.ndarray
  actuator_gaintype: np.ndarray
  actuator_biastype: np.ndarray
  actuator_trnid: np.ndarray
  actuator_actadr: np.ndarray
  actuator_actnum: np.ndarray
  actuator_ctrllimited: np.ndarray
  actuator_forcelimited: np.ndarray
  actuator_actlimited: np.ndarray
  actuator_dynprm: jax.Array
  actuator_gainprm: jax.Array
  actuator_biasprm: jax.Array
  actuator_ctrlrange: jax.Array
  actuator_forcerange: jax.Array
  actuator_actrange: jax.Array
  actuator_gear: jax.Array
  numeric_adr: np.ndarray
  numeric_data: np.ndarray
  name_numericadr: np.ndarray
  names: bytes


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
    geom1: id of geom 1
    geom2: id of geom 2
  """
  dist: jax.Array
  pos: jax.Array
  frame: jax.Array
  includemargin: jax.Array
  friction: jax.Array
  solref: jax.Array
  solreffriction: jax.Array
  solimp: jax.Array
  # unsupported: mu, H, dim
  geom1: jax.Array
  geom2: jax.Array
  # unsupported: efc_address, exclude

  @classmethod
  def zero(cls, ncon: int = 0) -> 'Contact':
    """Returns a contact filled with zeros."""
    return Contact(
        dist=jp.zeros(ncon),
        pos=jp.zeros((ncon, 3,)),
        frame=jp.zeros((ncon, 3, 3)),
        includemargin=jp.zeros(ncon),
        friction=jp.zeros((ncon, 5)),
        solref=jp.zeros((ncon, mujoco.mjNREF)),
        solreffriction=jp.zeros((ncon, mujoco.mjNREF)),
        solimp=jp.zeros((ncon, mujoco.mjNIMP,)),
        geom1=jp.zeros(ncon, dtype=jp.int32),
        geom2=jp.zeros(ncon, dtype=jp.int32),
    )


class Data(PyTreeNode):
  """Dynamic state that updates each step.

  Attributes:
    solver_niter: number of solver iterations, per island         (mjNISLAND,)
    time: simulation time
    qpos: position                                                (nq,)
    qvel: velocity                                                (nv,)
    act: actuator activation                                      (na,)
    qacc_warmstart: acceleration used for warmstart               (nv,)
    ctrl: control                                                 (nu,)
    qfrc_applied: applied generalized force                       (nv,)
    xfrc_applied: applied Cartesian force/torque                  (nbody, 6)
    eq_active: enable/disable constraints                         (neq,)
    qacc: acceleration                                            (nv,)
    act_dot: time-derivative of actuator activation               (na,)
    xpos:  Cartesian position of body frame                       (nbody, 3)
    xquat: Cartesian orientation of body frame                    (nbody, 4)
    xmat:  Cartesian orientation of body frame                    (nbody, 3, 3)
    xipos: Cartesian position of body com                         (nbody, 3)
    ximat: Cartesian orientation of body inertia                  (nbody, 3, 3)
    xanchor: Cartesian position of joint anchor                   (njnt, 3)
    xaxis: Cartesian joint axis                                   (njnt, 3)
    geom_xpos: Cartesian geom position                            (ngeom, 3)
    geom_xmat: Cartesian geom orientation                         (ngeom, 3, 3)
    site_xpos: Cartesian site position                            (nsite, 3)
    site_xmat: Cartesian site orientation                         (nsite, 9)
    subtree_com: center of mass of each subtree                   (nbody, 3)
    cdof: com-based motion axis of each dof                       (nv, 6)
    cinert: com-based body inertia and mass                       (nbody, 10)
    actuator_length: actuator lengths                             (nu,)
    actuator_moment: actuator moments                             (nu, nv)
    crb: com-based composite inertia and mass                     (nbody, 10)
    qM: total inertia                                  if sparse: (nM,)
                                                       if dense:  (nv, nv)
    qLD: L'*D*L (or Cholesky) factorization of M.      if sparse: (nM,)
                                                       if dense:  (nv, nv)
    qLDiagInv: 1/diag(D)                               if sparse: (nv,)
                                                       if dense:  (0,)
    contact: list of all detected contacts                        (ncon,)
    efc_J: constraint Jacobian                                    (nefc, nv)
    efc_frictionloss: frictionloss (friction)                     (nefc,)
    efc_D: constraint mass                                        (nefc,)
    actuator_velocity: actuator velocities                        (nu,)
    cvel: com-based velocity [3D rot; 3D tran]                    (nbody, 6)
    cdof_dot: time-derivative of cdof                             (nv, 6)
    qfrc_bias: C(qpos,qvel)                                       (nv,)
    qfrc_passive: passive force                                   (nv,)
    efc_aref: reference pseudo-acceleration                       (nefc,)
    qfrc_actuator: actuator force                                 (nv,)
    qfrc_smooth: net unconstrained force                          (nv,)
    qacc_smooth: unconstrained acceleration                       (nv,)
    qfrc_constraint: constraint force                             (nv,)
    qfrc_inverse: net external force; should equal:               (nv,)
      qfrc_applied + J'*xfrc_applied + qfrc_actuator
    efc_force: constraint force in constraint space               (nefc,)
  """
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
  # dynamics:
  qacc: jax.Array
  act_dot: jax.Array
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
  subtree_com: jax.Array
  cdof: jax.Array
  cinert: jax.Array
  crb: jax.Array
  actuator_length: jax.Array
  actuator_moment: jax.Array
  qM: jax.Array  # pylint:disable=invalid-name
  qLD: jax.Array  # pylint:disable=invalid-name
  qLDiagInv: jax.Array  # pylint:disable=invalid-name
  contact: Contact
  efc_J: jax.Array  # pylint:disable=invalid-name
  efc_frictionloss: jax.Array
  efc_D: jax.Array  # pylint:disable=invalid-name
  # position, velocity dependent:
  actuator_velocity: jax.Array
  cvel: jax.Array
  cdof_dot: jax.Array
  qfrc_bias: jax.Array
  qfrc_passive: jax.Array
  efc_aref: jax.Array
  # position, velcoity, control & acceleration dependent:
  qfrc_actuator: jax.Array
  qfrc_smooth: jax.Array
  qacc_smooth: jax.Array
  qfrc_constraint: jax.Array
  qfrc_inverse: jax.Array
  efc_force: jax.Array
