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

import logging

import mujoco
from mujoco.sysid._src import model_modifier
from mujoco.sysid._src import parameter
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


def test_apply_param_modifiers_warns_on_none_modifier(box_spec, caplog):
  """Non-frozen parameters whose modifier is None trigger a warning naming them.

  This catches the issue #3286 footgun: a modifier-factory whose `return modifier`
  is misindented returns None, which silently no-ops at apply time and produces
  a zero-gradient optimization.
  """
  params = parameter.ParameterDict()
  params.add(parameter.Parameter("orphan_no_modifier", 1.0, 0.0, 2.0))

  with caplog.at_level(logging.WARNING, logger="absl"):
    model_modifier.apply_param_modifiers_spec(params, box_spec.copy())

  assert any(
      "orphan_no_modifier" in r.message and "modifier=None" in r.message
      for r in caplog.records
  ), "Expected a warning naming the parameter with modifier=None."

  # Dedup: a second apply on the same ParameterDict does not re-warn.
  caplog.clear()
  with caplog.at_level(logging.WARNING, logger="absl"):
    model_modifier.apply_param_modifiers_spec(params, box_spec.copy())
  assert not any("modifier=None" in r.message for r in caplog.records), (
      "Warning should fire at most once per ParameterDict instance."
  )


def test_apply_param_modifiers_silent_for_frozen_none_modifier(
    box_spec, caplog
):
  """A frozen parameter with modifier=None is legitimate and must not warn."""
  params = parameter.ParameterDict()
  params.add(
      parameter.Parameter("frozen_no_modifier", 1.0, 0.0, 2.0, frozen=True)
  )

  with caplog.at_level(logging.WARNING, logger="absl"):
    model_modifier.apply_param_modifiers_spec(params, box_spec.copy())

  assert not any("modifier=None" in r.message for r in caplog.records), (
      "Did not expect a modifier=None warning for a frozen parameter."
  )
