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

"""CPU-only acceptance and rejection tests for native stepping eligibility."""

import mujoco
import numpy as np
import pytest

import mujoco_metal.stepping as stepping_module
from mujoco_metal.registry import TARGET_MUJOCO_VERSION
from mujoco_metal.stepping import SteppingProfile
from mujoco_metal.stepping import validate_stepping_profile


def _xml(extra="", body=None, option="timestep='.002'"):
  if body is None:
    body = """<body><joint name='hinge' type='hinge' armature='.2'/>
      <geom type='sphere' size='.1'/></body>"""
  worldbody = extra if extra.startswith("<worldbody>") else (
      f"<worldbody>{body}</worldbody>{extra}"
  )
  return f"""<mujoco><option {option}>
    <flag contact='disable'/></option>{worldbody}</mujoco>"""


def _model(*args, **kwargs):
  return mujoco.MjModel.from_xml_string(_xml(*args, **kwargs))


def test_accepts_supported_joint_types_and_gravity_options():
  body = """<body><freejoint/><geom type='sphere' size='.1'/>
    <body><joint type='ball'/><geom type='sphere' size='.1'/>
    <body><joint type='slide'/><geom type='sphere' size='.1'/>
    <body><joint type='hinge' armature='.1'/>
    <geom type='sphere' size='.1'/></body></body></body></body>"""
  profile = validate_stepping_profile(
      _model(body=body, option="timestep='.001'")
  )
  assert isinstance(profile, SteppingProfile)
  assert profile.name == "contact_free_euler_v1"
  assert profile.timestep == pytest.approx(0.001)
  assert profile.nq > 0 and profile.nv > 0
  assert "joint armature" in profile.supported
  assert any("constraint-solver" in entry for entry in profile.irrelevant)
  assert "tendons, including tendon armature" in profile.rejected

  model = _model()
  model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)
  assert validate_stepping_profile(model).name == profile.name


def test_accepts_empty_static_world_and_energy_diagnostic_flag():
  model = _model(body="")
  model.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_ENERGY)
  profile = validate_stepping_profile(model)
  assert (profile.nq, profile.nv) == (0, 0)
  assert any("energy diagnostics" in entry for entry in profile.irrelevant)


def test_version_contract_and_unavailable_enable_flags(monkeypatch):
  model = _model()
  monkeypatch.setattr(stepping_module.mujoco, "__version__", "3.14.1")
  with pytest.raises(RuntimeError, match=TARGET_MUJOCO_VERSION):
    validate_stepping_profile(model)
  monkeypatch.setattr(stepping_module.mujoco, "__version__", TARGET_MUJOCO_VERSION)

  model.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_FWDINV)
  with pytest.raises(ValueError, match="unsupported enable flags"):
    validate_stepping_profile(model)


def test_profile_requires_contact_disabled_euler_and_valid_timesteps():
  no_contact_disable = mujoco.MjModel.from_xml_string(
      "<mujoco><worldbody/></mujoco>"
  )
  with pytest.raises(ValueError, match="contact explicitly disabled"):
    validate_stepping_profile(no_contact_disable)

  model = _model()
  for dt in (0, -1, float("inf"), float("nan"), 1e-100, 1e100):
    with pytest.raises(ValueError, match="timestep"):
      validate_stepping_profile(model, dt)
  with pytest.raises(ValueError, match="unsupported stepping profile"):
    validate_stepping_profile(model, profile="other")
  model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
  with pytest.raises(ValueError, match="Euler integrator"):
    validate_stepping_profile(model)


@pytest.mark.parametrize(
    "extra, reason",
    [
        ("<actuator><motor joint='hinge'/></actuator>", "actuators are unsupported"),
        (
            "<tendon><fixed name='t'><joint joint='hinge' coef='1'/>"
            "</fixed></tendon>"
            "<actuator><motor tendon='t' armature='.5'/></actuator>",
            "actuators are unsupported",
        ),
        (
            "<tendon><fixed name='t' armature='.5'>"
            "<joint joint='hinge' coef='1'/></fixed></tendon>",
            "tendons, including tendon armature",
        ),
        (
            "<equality><joint joint1='hinge'/></equality>",
            "equality constraints",
        ),
        (
            "<worldbody><body><joint name='limited' range='-1 1' "
            "limited='true'/><geom type='sphere' size='.1'/></body></worldbody>",
            "joint limits",
        ),
        (
            "<worldbody><body><joint name='friction' frictionloss='.2'/>"
            "<geom type='sphere' size='.1'/></body></worldbody>",
            "friction loss",
        ),
        ("<sensor><jointpos joint='hinge'/></sensor>", "sensors are unsupported"),
        (
            "<worldbody><body gravcomp='1'><joint name='gravcomp'/>"
            "<geom type='sphere' size='.1'/></body></worldbody>",
            "gravity compensation",
        ),
    ],
)
def test_rejects_xml_model_features(extra, reason):
  model = _model(extra=extra)
  with pytest.raises(ValueError, match=reason):
    validate_stepping_profile(model)


def test_rejects_passives_fluid_mocap_sleep_callbacks_and_plugins():
  model = _model(body="""<body><joint name='hinge' damping='.2'/>
    <geom type='sphere' size='.1'/></body>""")
  with pytest.raises(ValueError, match="joint damping"):
    validate_stepping_profile(model)

  model = _model()
  model.dof_dampingpoly[0, 0] = 0.1
  with pytest.raises(ValueError, match="polynomial damping"):
    validate_stepping_profile(model)

  model = _model()
  model.jnt_stiffnesspoly[0, 0] = 0.1
  with pytest.raises(ValueError, match="joint stiffness"):
    validate_stepping_profile(model)

  model = _model(option="density='1'")
  with pytest.raises(ValueError, match="fluid forces"):
    validate_stepping_profile(model)

  mocap = _model(
      body="<body mocap='true'><geom type='sphere' size='.1'/></body>"
  )
  with pytest.raises(ValueError, match="mocap"):
    validate_stepping_profile(mocap)

  model = _model()
  model.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_SLEEP)
  with pytest.raises(ValueError, match="sleep mode"):
    validate_stepping_profile(model)

  model = _model()
  previous_passive = mujoco.get_mjcb_passive()
  try:
    mujoco.set_mjcb_passive(lambda *_: None)
    with pytest.raises(ValueError, match="global passive"):
      validate_stepping_profile(model)
  finally:
    mujoco.set_mjcb_passive(previous_passive)

  model = _model()
  model.body_plugin[1] = 0
  with pytest.raises(ValueError, match="MuJoCo plugins"):
    validate_stepping_profile(model)


def test_rejects_flex_and_unknown_flags():
  model = mujoco.MjModel.from_xml_string(
      """<mujoco><option><flag contact='disable'/></option><worldbody>
      <flexcomp name='soft' type='grid' dim='1' count='3 1 1'
        spacing='.1 .1 .1' mass='1'/>
      </worldbody></mujoco>"""
  )
  with pytest.raises(ValueError, match="flex/deformable"):
    validate_stepping_profile(model)

  model = _model()
  model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_ISLAND)
  with pytest.raises(ValueError, match="unsupported disable flags"):
    validate_stepping_profile(model)


def test_callback_rejection_does_not_modify_compiled_model():
  model = _model()
  before = (model.opt.disableflags, model.opt.enableflags, model.qpos0.copy())
  previous_control = mujoco.get_mjcb_control()
  try:
    mujoco.set_mjcb_control(lambda *_: None)
    with pytest.raises(ValueError, match="global passive and control callbacks"):
      validate_stepping_profile(model)
  finally:
    mujoco.set_mjcb_control(previous_control)
  assert model.opt.disableflags == before[0]
  assert model.opt.enableflags == before[1]
  np.testing.assert_array_equal(model.qpos0, before[2])
