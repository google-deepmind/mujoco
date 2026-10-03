# Copyright 2026 The MuJoCo Metal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""CPU oracle tests for the explicitly limited smooth inertial stage."""

import mujoco
import numpy as np
import pytest

from mujoco_metal.model import load_model
from mujoco_metal.smooth import smooth_dynamics

_MIXED = """<mujoco><worldbody>
  <body pos=".2 .1 0"><freejoint/><geom type="sphere" size=".1"/>
    <body pos="0 0 .3"><joint type="hinge" axis="0 1 0" pos=".1 0 0" armature=".04" damping=".2"/>
      <geom type="box" size=".1 .2 .3"/>
      <body pos=".2 0 0"><joint type="slide" axis="1 0 0" armature=".03" damping=".4"/>
        <geom type="sphere" size=".08"/>
        <body pos=".1 0 0"><joint type="ball" pos="0 0 .1"/>
          <geom type="capsule" size=".04 .1"/></body>
      </body>
    </body>
  </body>
  <body pos="1 0 0"><freejoint/><geom type="capsule" size=".1 .2"/></body>
</worldbody></mujoco>"""


def _random_state(model, data, rng):
  data.qpos[:] = model.qpos0
  for joint, typ in enumerate(model.jnt_type):
    qa = int(model.jnt_qposadr[joint])
    if typ == int(mujoco.mjtJoint.mjJNT_FREE):
      data.qpos[qa : qa + 3] = rng.normal(size=3)
      quat = rng.normal(size=4)
      data.qpos[qa + 3 : qa + 7] = quat / np.linalg.norm(quat)
    elif typ == int(mujoco.mjtJoint.mjJNT_BALL):
      quat = rng.normal(size=4)
      data.qpos[qa : qa + 4] = quat / np.linalg.norm(quat)
    else:
      data.qpos[qa] += rng.normal()
  data.qvel[:] = rng.normal(size=model.nv)


def test_dense_mass_and_bias_match_pinned_mujoco_reference():
  model = mujoco.MjModel.from_xml_string(_MIXED)
  descriptor = load_model(model)
  data = mujoco.MjData(model)
  rng = np.random.default_rng(8201)
  for _ in range(40):
    _random_state(model, data, rng)
    mujoco.mj_forward(model, data)
    expected_mass = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, data, expected_mass)
    actual = smooth_dynamics(descriptor, data.qpos, data.qvel)
    np.testing.assert_allclose(
        actual["mass_matrix"], expected_mass, rtol=1e-10, atol=1e-10
    )
    np.testing.assert_allclose(
        actual["qfrc_bias"], data.qfrc_bias, rtol=1e-10, atol=1e-10
    )
    np.testing.assert_allclose(
        actual["mass_matrix"], actual["mass_matrix"].T, atol=1e-12
    )


def test_free_body_bias_and_gravity_match_analytic_reference():
  xml = """<mujoco><worldbody><body>
    <freejoint/><inertial pos="0 0 0" mass="2" diaginertia=".2 .3 .4"/>
  </body></worldbody></mujoco>"""
  model = mujoco.MjModel.from_xml_string(xml)
  descriptor = load_model(model)
  qpos = np.array(model.qpos0)
  qvel = np.array([0.0, 0.0, 0.0, 1.0, 2.0, 3.0])
  actual = smooth_dynamics(descriptor, qpos, qvel)
  np.testing.assert_allclose(actual["qfrc_bias"][:3], [0, 0, 19.62], atol=1e-12)
  np.testing.assert_allclose(
      actual["qfrc_bias"][3:], [0.6, -0.6, 0.2], atol=1e-12
  )


def test_stage_rejects_actuator_models_and_invalid_state():
  xml = """<mujoco><worldbody><body><joint name="hinge"/>
    <geom type="sphere" size=".1"/></body></worldbody>
    <actuator><motor joint="hinge"/></actuator></mujoco>"""
  descriptor = load_model(xml)
  with pytest.raises(ValueError, match="actuators"):
    smooth_dynamics(
        descriptor, np.zeros(descriptor.nq), np.zeros(descriptor.nv)
    )

  tendon_xml = """<mujoco><worldbody><body>
    <joint name="j"/><geom type="sphere" size=".1"/>
    </body></worldbody><tendon><fixed armature=".2">
    <joint joint="j" coef="2"/></fixed></tendon></mujoco>"""
  tendon_model = mujoco.MjModel.from_xml_string(tendon_xml)
  assert tendon_model.nu == 0 and tendon_model.ntendon == 1
  tendon_descriptor = load_model(tendon_model)
  tendon_data = mujoco.MjData(tendon_model)
  mujoco.mj_forward(tendon_model, tendon_data)
  reference_mass = np.empty((tendon_model.nv, tendon_model.nv))
  mujoco.mj_fullM(tendon_model, tendon_data, reference_mass)
  zero_xml = tendon_xml.replace('armature=".2"', 'armature="0"')
  zero_model = mujoco.MjModel.from_xml_string(zero_xml)
  zero_data = mujoco.MjData(zero_model)
  mujoco.mj_forward(zero_model, zero_data)
  zero_mass = np.empty((zero_model.nv, zero_model.nv))
  mujoco.mj_fullM(zero_model, zero_data, zero_mass)
  np.testing.assert_allclose(reference_mass - zero_mass, [[0.8]], atol=1e-12)
  with pytest.raises(ValueError, match="tendon armature"):
    smooth_dynamics(tendon_descriptor, tendon_data.qpos, tendon_data.qvel)
  actual_zero = smooth_dynamics(
      load_model(zero_model), zero_data.qpos, zero_data.qvel
  )
  np.testing.assert_allclose(actual_zero["mass_matrix"], zero_mass, atol=1e-12)

  passive = load_model(
      '<mujoco><worldbody><body><joint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  )
  with pytest.raises(ValueError, match="qvel"):
    smooth_dynamics(passive, np.array(passive.qpos0), np.array([np.nan]))


def test_deep_chain_uses_model_dimensions_for_dense_matrix():
  bodies = []
  for index in range(24):
    bodies.append(
        f'<body pos="0 0 .08"><joint type="hinge" axis="0 1 0" armature=".01"/>'
        f'<geom type="capsule" size=".025 .04"/>'
    )
  xml = (
      "<mujoco><worldbody>"
      + "".join(bodies)
      + "</body>" * len(bodies)
      + "</worldbody></mujoco>"
  )
  model = mujoco.MjModel.from_xml_string(xml)
  descriptor = load_model(model)
  data = mujoco.MjData(model)
  rng = np.random.default_rng(212)
  data.qpos[:] = rng.normal(size=model.nq)
  data.qvel[:] = rng.normal(size=model.nv)
  mujoco.mj_forward(model, data)
  expected_mass = np.empty((model.nv, model.nv))
  mujoco.mj_fullM(model, data, expected_mass)
  actual = smooth_dynamics(descriptor, data.qpos, data.qvel)
  assert actual["mass_matrix"].shape == (24, 24)
  np.testing.assert_allclose(
      actual["mass_matrix"], expected_mass, rtol=1e-10, atol=1e-10
  )
  np.testing.assert_allclose(
      actual["qfrc_bias"], data.qfrc_bias, rtol=1e-10, atol=1e-10
  )


def test_empty_world_and_massless_fixed_ancestor():
  empty = load_model("<mujoco><worldbody/></mujoco>")
  result = smooth_dynamics(empty, np.empty(0), np.empty(0))
  assert result["mass_matrix"].shape == (0, 0)
  assert result["qfrc_bias"].shape == (0,)

  xml = """<mujoco><worldbody>
    <body pos="0 0 .1"><body pos=".1 0 0"><joint type="hinge"/>
      <geom type="sphere" size=".08"/></body></body>
  </worldbody></mujoco>"""
  model = mujoco.MjModel.from_xml_string(xml)
  descriptor = load_model(model)
  data = mujoco.MjData(model)
  data.qpos[0] = 0.37
  data.qvel[0] = -0.4
  mujoco.mj_forward(model, data)
  actual = smooth_dynamics(descriptor, data.qpos, data.qvel)
  expected_mass = np.empty((model.nv, model.nv))
  mujoco.mj_fullM(model, data, expected_mass)
  np.testing.assert_allclose(actual["mass_matrix"], expected_mass, atol=1e-12)
  np.testing.assert_allclose(actual["qfrc_bias"], data.qfrc_bias, atol=1e-12)


def test_rotated_inertial_frame_two_hinges_on_one_body():
  xml = """<mujoco><worldbody><body>
    <joint type="hinge" axis="0 1 0"/>
    <joint type="hinge" pos=".1 0 0" axis="1 0 0"/>
    <inertial pos=".1 .2 .3" quat=".9238795325 .3826834324 0 0"
      mass="1.7" diaginertia=".12 .19 .23"/>
  </body></worldbody></mujoco>"""
  model = mujoco.MjModel.from_xml_string(xml)
  descriptor = load_model(model)
  data = mujoco.MjData(model)
  data.qpos[:] = [0.31, -0.28]
  data.qvel[:] = [0.8, -1.1]
  mujoco.mj_forward(model, data)
  expected_mass = np.empty((model.nv, model.nv))
  mujoco.mj_fullM(model, data, expected_mass)
  actual = smooth_dynamics(descriptor, data.qpos, data.qvel)
  np.testing.assert_allclose(actual["mass_matrix"], expected_mass, atol=1e-12)
  np.testing.assert_allclose(actual["qfrc_bias"], data.qfrc_bias, atol=1e-12)


def test_gravity_disable_and_analytic_hinge_pendulum_bias():
  model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody><body>
      <joint type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -.7" mass="2" diaginertia=".1 .1 .1"/>
      </body></worldbody></mujoco>""")
  descriptor = load_model(model)
  qpos = np.array([0.4])
  qvel = np.zeros(1)
  bias = smooth_dynamics(descriptor, qpos, qvel)["qfrc_bias"]
  np.testing.assert_allclose(bias, [2 * 9.81 * 0.7 * np.sin(0.4)], atol=1e-12)

  model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)
  disabled = load_model(model)
  data = mujoco.MjData(model)
  data.qpos[:] = qpos
  mujoco.mj_forward(model, data)
  actual = smooth_dynamics(disabled, qpos, qvel)
  np.testing.assert_allclose(actual["qfrc_bias"], data.qfrc_bias, atol=1e-12)
  np.testing.assert_allclose(actual["qfrc_bias"], [0.0], atol=1e-12)
