# Copyright 2026 DeepMind Technologies Limited
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

"""Model modifiers."""

from typing import Any

import mujoco
from mujoco.sysid._src.parameter import InertiaType
from mujoco.sysid._src.parameter import ModifierFn
from mujoco.sysid._src.parameter import Parameter
from mujoco.sysid._src.parameter import ParameterDict
import numpy as np


def remove_visuals(in_spec: mujoco.MjSpec) -> mujoco.MjSpec:
  """Remove visual elements from a Spec."""
  spec = in_spec.copy()
  all_geoms = spec.worldbody.find_all("geom")
  for geom in all_geoms:
    if geom.contype == 0 and geom.conaffinity == 0:
      if geom.type == mujoco.mjtGeom.mjGEOM_MESH and geom.meshname:
        meshname = geom.meshname
        mesh = spec.mesh(meshname)
        if mesh:  # multiple geoms can ref same mesh.
          spec.delete(mesh)
      spec.delete(geom)

  for mat in spec.materials:
    spec.delete(mat)
  for tex in spec.textures:
    spec.delete(tex)

  spec.compile()  # TODO(b/0): is this compile necessary?
  return spec


def _get_obj_or_raise(spec: mujoco.MjSpec, obj_type: str, obj_name: str) -> Any:
  getter = getattr(spec, obj_type, None)
  if not callable(getter):
    raise AttributeError(f"MjSpec has no method '{obj_type}'")
  obj = getter(obj_name)
  if obj is None:
    raise ValueError(f"{obj_type.capitalize()} '{obj_name}' not found in spec.")
  return obj


def apply_param_modifiers_spec(
    params: ParameterDict, spec: mujoco.MjSpec
) -> mujoco.MjSpec:
  for key in params.keys():
    param = params[key]
    if not param.frozen:
      param.apply_modifier(spec)
  return spec


def apply_param_modifiers(
    params: ParameterDict, spec: mujoco.MjSpec
) -> mujoco.MjModel:
  return apply_param_modifiers_spec(params, spec).compile()


def _infer_inertial(spec: mujoco.MjSpec, body_name: str) -> mujoco.MjsBody:
  """Override spec inertia using inferred inertia from compiled model."""
  body = _get_obj_or_raise(spec, "body", body_name)
  assert isinstance(body, mujoco.MjsBody)
  spec.compiler.inertiafromgeom = 2
  model = spec.compile()
  body.explicitinertial = True
  body.fullinertia = np.full((6, 1), np.nan)
  body.mass = model.body(body_name).mass[0]
  body.inertia = model.body(body_name).inertia
  body.ipos = model.body(body_name).ipos
  body.iquat = model.body(body_name).iquat
  return body


def is_position_actuator(actuator) -> bool:
  """Check if an actuator is a position actuator.

  This function works on both model.actuator and spec.actuator objects.

  Args:
    actuator: An actuator object from model or spec.

  Returns:
    True if the actuator is a position actuator.
  """
  return (
      actuator.gaintype == mujoco.mjtGain.mjGAIN_FIXED
      and actuator.biastype == mujoco.mjtBias.mjBIAS_AFFINE
      and actuator.dyntype
      in (mujoco.mjtDyn.mjDYN_NONE, mujoco.mjtDyn.mjDYN_FILTEREXACT)
      and actuator.gainprm[0] == -actuator.biasprm[1]
  )


def get_actuator_pd_gains(
    model: mujoco.MjModel, actuator_name: str
) -> tuple[float, float]:
  """Return the (P, D) gains of a position actuator.

  Args:
    model: MuJoCo model.
    actuator_name: Name of the actuator.
  """
  actuator_id = mujoco.mj_name2id(
      model, mujoco.mjtObj.mjOBJ_ACTUATOR.value, actuator_name
  )
  if actuator_id == -1:
    raise ValueError(f"Actuator {actuator_name} not found in model.")
  actuator = model.actuator(actuator_id)
  if not is_position_actuator(actuator):
    raise ValueError(f"Actuator {actuator_name} is not a position actuator.")
  return -actuator.biasprm[1], -actuator.biasprm[2]


def apply_pgain(
    spec: mujoco.MjSpec,
    actuator_name: str,
    value: float | np.ndarray,
) -> mujoco.MjSpec:
  """Set the proportional gain for a position actuator.

  Args:
    spec: MuJoCo model specification.
    actuator_name: Name of the actuator.
    value: Proportional gain value.
  """
  # TODO(b/0): assert scalar
  actuator = _get_obj_or_raise(spec, "actuator", actuator_name)
  assert isinstance(actuator, mujoco.MjsActuator)
  if not is_position_actuator(actuator):
    raise ValueError(f"Actuator {actuator_name} is not a position actuator.")
  actuator.gainprm[0] = value
  actuator.biasprm[1] = -value
  return spec


def apply_dgain(
    spec: mujoco.MjSpec,
    actuator_name: str,
    value: float | np.ndarray,
) -> mujoco.MjSpec:
  """Set the derivative gain for a position actuator.

  Args:
    spec: MuJoCo model specification.
    actuator_name: Name of the actuator.
    value: Derivative gain value.
  """
  # TODO(b/0): assert scalar
  actuator = _get_obj_or_raise(spec, "actuator", actuator_name)
  assert isinstance(actuator, mujoco.MjsActuator)
  if not is_position_actuator(actuator):
    raise ValueError(f"Actuator {actuator_name} is not a position actuator.")
  actuator.biasprm[2] = -value
  return spec


def apply_pdgain(
    spec: mujoco.MjSpec,
    actuator_name: str,
    value: np.ndarray,
) -> mujoco.MjSpec:
  """Set both proportional and derivative gains for a position actuator.

  Args:
    spec: MuJoCo model specification.
    actuator_name: Name of the actuator.
    value: 2-element array ``[P_gain, D_gain]``.
  """
  if value.size != 2:
    raise ValueError(f"pdgain must be a 2-element array, got {value.size}.")
  apply_pgain(spec, actuator_name, value[0])
  apply_dgain(spec, actuator_name, value[1])
  return spec


def apply_body_mass_ipos(
    spec: mujoco.MjSpec,
    body_name: str,
    mass: np.ndarray | None = None,
    ipos: np.ndarray | None = None,
    rot_inertia_scale: bool = False,
) -> mujoco.MjSpec:
  """Apply mass and center-of-mass position to a body.

  Args:
    spec: MuJoCo model specification.
    body_name: Name of the body.
    mass: Optional new mass value.
    ipos: Optional new center-of-mass position.
    rot_inertia_scale: If True, scale rotational inertia proportionally to
      mass change.
  """
  # TODO(b/0): assert mass and ipos shapes
  body = _infer_inertial(spec, body_name)
  mass_original = body.mass
  if mass is not None:
    body.mass = mass
    if rot_inertia_scale:
      scale = mass / mass_original
      body.inertia *= scale
  if ipos is not None:
    body.ipos = ipos
  return spec


def scale_body_inertia(
    spec: mujoco.MjSpec,
    body_name: str,
    value: np.ndarray,
) -> mujoco.MjSpec:
  # TODO(b/0): assert scalar
  body = _infer_inertial(spec, body_name)
  body.inertia *= value
  return spec


def pi_from_theta(theta: np.ndarray) -> np.ndarray:
  """Convert base parameters θ to inertial parameters π.

  Args:
    theta: 10-D array [alpha, d1, d2, d3, s12, s23, s13, t1, t2, t3] where:
      alpha: Scale parameter (log of U[3,3])
      [d1, d2, d3]: Log of diagonal elements
      [s12, s23, s13]: Shear parameters from upper triangle
      [t1, t2, t3]: Translation parameters from last column

  Returns:
    10-D array π = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Iyz, Ixz].
  """
  alpha, d1, d2, d3, s12, s23, s13, t1, t2, t3 = theta
  exp_alpha = np.exp(alpha)
  exp_d1 = np.exp(d1)
  exp_d2 = np.exp(d2)
  exp_d3 = np.exp(d3)
  U = np.zeros((4, 4))
  U[0, 0] = exp_d1
  U[0, 1] = s12
  U[0, 2] = s13
  U[0, 3] = t1
  U[1, 1] = exp_d2
  U[1, 2] = s23
  U[1, 3] = t2
  U[2, 2] = exp_d3
  U[2, 3] = t3
  U[3, 3] = 1
  U *= exp_alpha

  J = U @ U.T

  sigma = J[:3, :3]
  I_bar = np.trace(sigma) * np.eye(3) - sigma
  h = J[:3, 3]
  m = J[3, 3]

  return np.concatenate(([m], h, I_bar.flatten()))


def pseudoinertia_from_pi(pi: np.ndarray) -> np.ndarray:
  """Converts inertial parameters π to a 4x4 pseudoinertia matrix J.

  Args:
    pi: A 10-D array [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Iyz, Ixz].

  Returns:
    A 4x4 pseudoinertia matrix J.
  """
  m = pi[0]
  h = pi[1:4]
  I_bar = pi[4:].reshape((3, 3))

  Sigma = 0.5 * np.trace(I_bar) * np.eye(3) - I_bar

  J = np.zeros((4, 4))
  J[:3, :3] = Sigma
  J[:3, 3] = h
  J[3, :3] = h
  J[3, 3] = m

  return J


def cholesky_decompose_upper(J: np.ndarray) -> np.ndarray:
  """Perform an upper-triangular Cholesky decomposition of J.

  The returned matrix U is such that J = U @ U.T.

  Args:
    J: A 4x4 positive-definite matrix.

  Returns:
    An upper-triangular matrix U.
  """
  n = J.shape[0]
  indices = np.arange(n - 1, -1, -1)
  J_reversed = J[indices][:, indices]
  L_prime = np.linalg.cholesky(J_reversed)
  return L_prime[indices][:, indices]


def theta_from_pseudoinertia(J: np.ndarray) -> np.ndarray:
  """Extract the 10-D vector of base parameters θ from the pseudoinertia J.

  Args:
    J: A 4x4 pseudoinertia.

  Returns:
    A 10-D array θ = [alpha, d1, d2, d3, s12, s23, s13, t1, t2, t3] where:
      alpha: Scale parameter (log of U[3,3])
      [d1, d2, d3]: Log of diagonal elements
      [s12, s23, s13]: Shear parameters from upper triangle
      [t1, t2, t3]: Translation parameters from last column
  """
  # U: A 4x4 upper-triangular matrix from Cholesky decomposition
  U = cholesky_decompose_upper(J)

  # Extract exp(α) from the bottom-right element of U.
  exp_alpha = U[3, 3]
  alpha = np.log(exp_alpha)

  # Compute the d parameters from the diagonal entries (adjusted by alpha).
  d1 = np.log(U[0, 0] / exp_alpha)
  d2 = np.log(U[1, 1] / exp_alpha)
  d3 = np.log(U[2, 2] / exp_alpha)

  # Extract the shear parameters (off-diagonals in the upper triangle).
  s12 = U[0, 1] / exp_alpha
  s13 = U[0, 2] / exp_alpha
  s23 = U[1, 2] / exp_alpha

  # Extract the translation parameters (last column, except the bottom element).
  t1 = U[0, 3] / exp_alpha
  t2 = U[1, 3] / exp_alpha
  t3 = U[2, 3] / exp_alpha

  return np.array([alpha, d1, d2, d3, s12, s23, s13, t1, t2, t3])


def skew(v: np.ndarray) -> np.ndarray:
  """Skew-symmetric matrix from a length-3 vector."""
  return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def inertia_to_fullinertia(q: np.ndarray, inertia: np.ndarray) -> np.ndarray:
  """Convert diagonal inertia with quaternion to full inertia matrix."""
  xmat = np.empty(9)
  mujoco.mju_quat2Mat(xmat, q)
  R = xmat.reshape(3, 3)
  return R @ np.diag(inertia) @ R.T


def pi_from_body(spec: mujoco.MjSpec, body_name: str) -> np.ndarray:
  """Extracts the 10-D vector of inertial parameters π from a MuJoCo body.

  Args:
    spec: MuJoCo model specification object.
    body_name: Name of the body to extract parameters from.

  Returns:
    A 10-D numpy array π = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Iyz, Ixz] where:
      m: Mass of the body
      [hx, hy, hz]: First moment of mass (m * com, where com is center of mass)
      [Ixx, Iyy, Izz, Ixy, Iyz, Ixz]: Rotational inertia about the origin of the
        body-fixed reference frame.
  """
  body = _infer_inertial(spec, body_name)
  mass = body.mass
  ipos = body.ipos
  inertia = body.inertia
  iquat = body.iquat

  fullinertia = inertia_to_fullinertia(iquat, inertia)
  # Transform inertial from ipos origin to body origin.
  I_bar = fullinertia - (mass * skew(ipos) @ skew(ipos))

  return np.concatenate([[mass], mass * ipos, I_bar.flatten()])


def theta_inertia_from_body(spec: mujoco.MjSpec, body_name: str) -> np.ndarray:
  """Extract base parameters θ from a body's inertial properties."""
  pi = pi_from_body(spec, body_name)
  J = pseudoinertia_from_pi(pi)
  return theta_from_pseudoinertia(J)


def apply_body_theta_inertia(
    spec: mujoco.MjSpec,
    body_name: str,
    theta: np.ndarray,
) -> mujoco.MjSpec:
  """Apply base-parameter inertia θ to a body in the spec.

  Args:
    spec: MuJoCo model specification.
    body_name: Name of the body.
    theta: 10-element array [alpha, d1, d2, d3, s12, s23, s13, t1, t2, t3].
  """
  if theta.size != 10:
    raise ValueError(f"theta must be a 10-element array, got {theta.size}.")
  pi = pi_from_theta(theta)

  body = _infer_inertial(spec, body_name)
  body.mass = pi[0]
  body.ipos = pi[1:4] / pi[0]

  # This tells the compiler to ignore the diagonal inertia and instead
  # calculate it from the full inertia.
  body.inertia[:] = 0.0
  body.iquat[:] = np.nan

  I_bar = pi[4:].reshape((3, 3))
  skew_ipos = skew(body.ipos)
  fullinertia = I_bar + (body.mass * skew_ipos @ skew_ipos)

  # MuJoCo's ordering is: M(1,1), M(2,2), M(3,3), M(1,2), M(1,3), M(2,3) which
  # corresponds to Ixx, Iyy, Izz, Ixy, Ixz
  body.fullinertia[0] = fullinertia[0, 0]  # Ixx
  body.fullinertia[1] = fullinertia[1, 1]  # Iyy
  body.fullinertia[2] = fullinertia[2, 2]  # Izz
  body.fullinertia[3] = fullinertia[0, 1]  # Ixy
  body.fullinertia[4] = fullinertia[0, 2]  # Ixz
  body.fullinertia[5] = fullinertia[1, 2]  # Iyz

  return spec


def apply_body_inertia(spec: mujoco.MjSpec, name: str, param: Parameter):
  """Apply inertia parameters to a body based on the parameter type.

  Args:
    spec: MuJoCo model specification.
    name: Name of the body.
    param: Parameter with an ``inertia_type`` attribute.
  """
  if not hasattr(param, "inertia_type"):
    raise ValueError(
        f"Parameter {param.name} does not have inertia_type attribute."
    )

  if param.inertia_type == InertiaType.Mass:
    apply_body_mass_ipos(
        spec, name, mass=param.value, rot_inertia_scale=param.scale_rot_inertia
    )

  elif param.inertia_type == InertiaType.MassIpos:
    apply_body_mass_ipos(
        spec,
        name,
        mass=param.value[0],
        ipos=param.value[1:4],
        rot_inertia_scale=param.scale_rot_inertia,
    )

  elif param.inertia_type == InertiaType.Pseudo:
    apply_body_theta_inertia(spec, name, param.value)


def body_inertia_param(
    spec: mujoco.MjSpec,
    model: mujoco.MjModel,
    body_name: str,
    inertia_type: InertiaType = InertiaType.MassIpos,
    scale_rot_inertia: bool = False,
    mass_bound_mult: np.ndarray | None = None,
    ipos_bound_off: np.ndarray | None = None,
    stretch_bound_mult: np.ndarray | None = None,
    shear_bound_off: np.ndarray | None = None,
    param_name: str | None = None,
    modifier: ModifierFn | None = None,
) -> Parameter:
  """Creates Parameter objects for the inertia of a body in a simplified manner.

  Args:
    spec: MuJoCo model specification object.
    model: The MuJoCo model.
    body_name: Name of the body to create the parameter for.
    inertia_type: The type of inertia parameterization to use.
    scale_rot_inertia: Whether to scale the original inertia when mass changes,
      ignored with pseudo inertia.
    mass_bound_mult: Multiplicative bounds for the mass parameter.
    ipos_bound_off: Additive bounds for the ipos parameter.
    stretch_bound_mult: Multiplicative bounds for the stretch parameters in the
      pseudo-inertia parameterization.
    shear_bound_off: Additive bounds for the shear parameters in the
      pseudo-inertia parameterization.
    param_name: Optional name for the parameter.  Defaults to
      ``"{body_name}_inertia"``.
    modifier: Optional custom modifier callback.  If None, the default
      :func:`apply_body_inertia` modifier is registered on the Parameter.

  Returns:
    A Parameter configured for the body's inertia.
  """

  if mass_bound_mult is None:
    mass_bound_mult = np.array([0.1, 10.0])
  if ipos_bound_off is None:
    ipos_bound_off = np.array([-0.5, 0.5])
  if stretch_bound_mult is None:
    stretch_bound_mult = np.array([0.5, 2.0])
  if shear_bound_off is None:
    shear_bound_off = np.array([-0.5, 0.5])

  body = model.body(body_name)
  if param_name is None:
    param_name = f"{body_name}_inertia"

  if modifier is None:

    def _default_modifier(spec, param):
      return apply_body_inertia(spec, body_name, param)

    modifier = _default_modifier

  if inertia_type == InertiaType.Mass:
    param = Parameter(
        param_name,
        body.mass,
        body.mass * mass_bound_mult[0],
        body.mass * mass_bound_mult[1],
        modifier=modifier,
    )
    param.inertia_type = inertia_type
    param.scale_rot_inertia = scale_rot_inertia

  elif inertia_type == InertiaType.MassIpos:
    massipos0 = np.concatenate((body.mass, body.ipos))
    massipos_low = np.concatenate(
        (body.mass * mass_bound_mult[0], body.ipos + ipos_bound_off[0])
    )
    massipos_high = np.concatenate(
        (body.mass * mass_bound_mult[1], body.ipos + ipos_bound_off[1])
    )
    param = Parameter(
        param_name, massipos0, massipos_low, massipos_high, modifier=modifier
    )
    param.inertia_type = inertia_type
    param.scale_rot_inertia = scale_rot_inertia

  elif inertia_type == InertiaType.Pseudo:
    theta_i_0 = theta_inertia_from_body(spec, body_name)

    # mass = exp(2*alpha)
    alpha = theta_i_0[0]
    mass = np.exp(2 * alpha)
    mass_bounds = mass * mass_bound_mult
    alpha_bounds = 0.5 * np.log(mass_bounds)

    # d1, d2, d3, stretch = exp(2*d)
    # stretches body along principal axes
    d = theta_i_0[1 : 1 + 3]
    stretch = np.exp(2 * d)
    stretch_bounds = stretch[:, np.newaxis] * np.atleast_2d(stretch_bound_mult)
    d_bounds = 0.5 * np.log(stretch_bounds)

    # s12, s23, s13
    # shear the body
    s_bounds = theta_i_0[4 : 4 + 3, np.newaxis] + np.atleast_2d(shear_bound_off)

    # t1, t2, t3
    # center of mass
    t_bounds = theta_i_0[7:10, np.newaxis] + np.atleast_2d(ipos_bound_off)

    theta_bounds = np.vstack([
        alpha_bounds,
        d_bounds,
        s_bounds,
        t_bounds,
    ])
    param = Parameter(
        param_name,
        theta_i_0,
        theta_bounds[:, 0],
        theta_bounds[:, 1],
        modifier=modifier,
    )
    param.inertia_type = inertia_type

  else:
    raise ValueError(f"Unknown inertia_type: {inertia_type}")

  return param
