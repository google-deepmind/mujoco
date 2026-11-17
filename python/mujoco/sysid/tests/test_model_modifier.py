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
"""Tests for the model_modifier module."""

import mujoco
from mujoco.sysid._src import model_modifier
import numpy as np


def test_apply_pgain(arm_spec):
  """Setting a P gain correctly configures the underlying actuator parameters."""
  actuator_name = "actuator5"
  pgain_value = 74

  modified_spec = model_modifier.apply_pgain(
      arm_spec, actuator_name, pgain_value
  )
  model = modified_spec.compile()

  assert model.actuator(actuator_name).gainprm[0] == pgain_value
  assert model.actuator(actuator_name).biasprm[1] == -pgain_value


def test_apply_dgain(arm_spec):
  """Setting a D gain correctly configures the underlying actuator parameters."""
  actuator_name = "actuator5"
  dgain_value = 1.2

  modified_spec = model_modifier.apply_dgain(
      arm_spec, actuator_name, dgain_value
  )
  model = modified_spec.compile()

  assert model.actuator(actuator_name).biasprm[2] == -dgain_value


def test_apply_pdgain(arm_spec):
  """Setting P and D gains together from a single array configures both correctly."""
  actuator_name = "actuator5"
  pdgain_value = np.array([74, 1.2])

  modified_spec = model_modifier.apply_pdgain(
      arm_spec, actuator_name, pdgain_value
  )
  model = modified_spec.compile()

  assert model.actuator(actuator_name).gainprm[0] == pdgain_value[0]
  assert model.actuator(actuator_name).biasprm[1] == -pdgain_value[0]
  assert model.actuator(actuator_name).biasprm[2] == -pdgain_value[1]


def test_apply_body_mass_explicit(arm_spec):
  """Bodies with inertia defined in XML: changing mass proportionally scales inertia."""
  body_name = "link1"
  model = arm_spec.compile()
  original_mass = model.body(body_name).mass[0]
  original_inertia = model.body(body_name).inertia
  del model

  scale = 3.3
  new_mass = scale * original_mass

  modified_spec = model_modifier.apply_body_mass_ipos(
      arm_spec, body_name, mass=new_mass, rot_inertia_scale=True
  )
  model = modified_spec.compile()

  assert model.body(body_name).mass == new_mass
  np.testing.assert_allclose(
      model.body(body_name).inertia, original_inertia * scale
  )


def test_apply_body_mass_implicit(oscillator_spec):
  """Bodies with inertia inferred from geoms: changing mass proportionally scales inertia."""
  body_name = "mass"
  model = oscillator_spec.compile()
  original_mass = model.body(body_name).mass[0]
  original_inertia = model.body(body_name).inertia
  del model

  scale = 0.077
  new_mass = scale * original_mass

  modified_spec = model_modifier.apply_body_mass_ipos(
      oscillator_spec, body_name, mass=new_mass, rot_inertia_scale=True
  )
  model = modified_spec.compile()

  assert model.body(body_name).mass == new_mass
  np.testing.assert_allclose(
      model.body(body_name).inertia, original_inertia * scale
  )


def test_apply_body_theta_inertia(arm_spec):
  """Pseudo-inertia parameters compile to the corresponding full inertia."""
  body_name = "link2"
  theta = model_modifier.theta_inertia_from_body(arm_spec, body_name)
  # Perturb mass, stretch, shear and COM so the inertia is non-diagonal.
  theta += np.array([0.1, 0.2, -0.1, 0.05, 0.1, -0.2, 0.05, 0.01, -0.02, 0.03])
  pi = model_modifier.pi_from_theta(theta)

  modified_spec = model_modifier.apply_body_theta_inertia(
      arm_spec, body_name, theta
  )
  model = modified_spec.compile()

  body = model.body(body_name)
  mass = body.mass[0]
  ipos = body.ipos
  np.testing.assert_allclose(mass, pi[0])
  np.testing.assert_allclose(ipos, pi[1:4] / pi[0])
  # Rotational inertia about the body origin, in body-frame axes.
  skew_ipos = model_modifier.skew(ipos)
  inertia_origin = (
      model_modifier.inertia_to_fullinertia(body.iquat, body.inertia)
      - mass * skew_ipos @ skew_ipos
  )
  np.testing.assert_allclose(
      inertia_origin, pi[4:].reshape((3, 3)), rtol=1e-9, atol=1e-12
  )
  np.testing.assert_allclose(
      model_modifier.theta_inertia_from_body(modified_spec, body_name),
      theta,
      atol=1e-9,
  )


def test_apply_body_theta_inertia_euler():
  """Pseudo-inertia parameters apply to an inertial oriented with euler."""
  spec = mujoco.MjSpec.from_string("""
    <mujoco>
      <worldbody>
        <body name="body">
          <joint axis="0 1 0"/>
          <inertial pos="0.1 0 0" mass="1" diaginertia="0.03 0.025 0.015"
                    euler="30 40 50"/>
        </body>
      </worldbody>
    </mujoco>
  """)
  body = spec.compile().body("body")
  inertia = model_modifier.inertia_to_fullinertia(body.iquat, body.inertia)

  theta = model_modifier.theta_inertia_from_body(spec, "body")
  model = model_modifier.apply_body_theta_inertia(spec, "body", theta).compile()

  body = model.body("body")
  np.testing.assert_allclose(
      model_modifier.inertia_to_fullinertia(body.iquat, body.inertia),
      inertia,
      atol=1e-12,
  )


def test_remove_visuals(arm_spec):
  """Stripping visuals removes all textures and materials for faster compilation."""
  cleaned_spec = model_modifier.remove_visuals(arm_spec)
  assert not cleaned_spec.textures
  assert not cleaned_spec.materials


def test_apply_param_modifiers(box_spec, box_params):
  """The full modifier pipeline applies parameter callbacks and produces an updated model."""
  spec = box_spec.copy()
  original_model = spec.compile()
  original_mass = original_model.body("box").mass[0]

  # Change box_mass parameter.
  box_params["box_mass"].update_from_vector(np.array([5.3]))

  modified_model = model_modifier.apply_param_modifiers(box_params, spec)
  assert modified_model.body("box").mass[0] != original_mass
  np.testing.assert_allclose(modified_model.body("box").mass[0], 5.3, atol=1e-6)

  # Also verify apply_param_modifiers_spec returns MjSpec.
  spec2 = box_spec.copy()
  result = model_modifier.apply_param_modifiers_spec(box_params, spec2)
  assert isinstance(result, mujoco.MjSpec)
