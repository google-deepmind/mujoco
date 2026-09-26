"""CPU contract tests for generic model lowering."""

import mujoco
import mujoco_metal.model as model_module
import numpy as np
import pytest

from mujoco_metal.model import load_model
from mujoco_metal.registry import (
    Execution, Feature, Implementation, Qualification, Stage, feature_status)


def test_immutable_lowering_and_empty_world():
  model = load_model('<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>')
  assert model.nbody == 1
  for value in (model.body_pos, model.jnt_type, model.geom_pos):
    with pytest.raises(ValueError):
      value.flat[0] = 1
    with pytest.raises(ValueError):
      value.setflags(write=True)
  pose = model.forward_kinematics(np.empty(0))
  assert pose["geom_pos"].shape == (1, 3)


def test_mixed_topology_matches_mujoco_oracle():
  xml = '''<mujoco><worldbody>
    <geom type="plane" size="2 2 .1"/>
    <body name="free" pos="1 2 3"><freejoint/><geom type="sphere" size=".2"/>
      <body name="hinge" pos="0 1 0"><geom type="sphere" size=".1"/><joint type="hinge" pos=".1 0 0" axis="0 1 0" ref=".2"/>
        <site pos="0 0 1"/>
        <body pos="0 0 1"><geom type="sphere" size=".1"/><joint type="slide" axis="1 1 0"/>
          <body><geom type="sphere" size=".1"/><joint type="ball" pos="0 0 .2"/><geom type="capsule" size=".1 .3"/></body>
        </body>
      </body>
    </body>
    <body name="second"><freejoint/><geom type="box" size=".2 .3 .4"/></body>
    <body name="fixed0" pos="0.0 0 0"/><body name="fixed1" pos="0.1 0 0"/><body name="fixed2" pos="0.2 0 0"/><body name="fixed3" pos="0.3 0 0"/><body name="fixed4" pos="0.4 0 0"/><body name="fixed5" pos="0.5 0 0"/><body name="fixed6" pos="0.6 0 0"/><body name="fixed7" pos="0.7 0 0"/><body name="fixed8" pos="0.8 0 0"/><body name="fixed9" pos="0.9 0 0"/><body name="fixed10" pos="1.0 0 0"/><body name="fixed11" pos="1.1 0 0"/><body name="fixed12" pos="1.2 0 0"/>
  </worldbody></mujoco>'''
  model = load_model(xml)
  compiled = mujoco.MjModel.from_xml_string(xml)
  data = mujoco.MjData(compiled)
  data.qpos[:] = compiled.qpos0
  for joint, typ in enumerate(model.jnt_type):
    qa = model.jnt_qposadr[joint]
    if typ == int(mujoco.mjtJoint.mjJNT_FREE):
      data.qpos[qa:qa+3] += np.array([.1, -.2, .3])
      data.qpos[qa+3:qa+7] = [np.cos(.17), 0, np.sin(.17), 0]
    elif typ == int(mujoco.mjtJoint.mjJNT_BALL):
      data.qpos[qa:qa+4] = [np.cos(.12), np.sin(.12), 0, 0]
    else:
      data.qpos[qa] += .27
  mujoco.mj_kinematics(compiled, data)
  actual = model.forward_kinematics(data.qpos)
  np.testing.assert_allclose(actual["body_pos"], data.xpos, atol=1e-12)
  np.testing.assert_allclose(actual["body_quat"], data.xquat, atol=1e-12)
  np.testing.assert_allclose(actual["geom_pos"], data.geom_xpos, atol=1e-12)
  np.testing.assert_allclose(actual["site_pos"], data.site_xpos, atol=1e-12)
  assert model.nbody > 17


def test_invalid_quaternion_and_feature_registry_is_host_only():
  model = load_model('<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
  qpos = np.zeros(model.nq)
  qpos[3] = 1.
  qpos[3:7] = 0
  with pytest.raises(ValueError, match="nonzero"):
    model.forward_kinematics(qpos)
  inventory = feature_status()
  assert inventory
  assert all(isinstance(row, Feature) for row in inventory)
  assert all(row.qualification != Qualification.GPU_QUALIFIED for row in inventory)
  assert any(row.name == "mjtJoint.mjJNT_FREE" for row in inventory)
  assert any(row.name.startswith("api:") for row in inventory)
  assert all(row.execution in Execution for row in inventory)
  assert all(row.stage in Stage and row.implementation in Implementation for row in inventory)
  assert "torch" not in __import__("sys").modules


def test_rejects_mocap_and_malformed_source_model():
  mocap = mujoco.MjModel.from_xml_string(
      '<mujoco><worldbody><body mocap="true"><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
  with pytest.raises(ValueError, match="mocap"):
    load_model(mocap)

  valid = mujoco.MjModel.from_xml_string(
      '<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>')
  valid.geom_bodyid[0] = 999
  with pytest.raises(ValueError, match="geom_bodyid"):
    load_model(valid)

  valid = mujoco.MjModel.from_xml_string(
      '<mujoco><worldbody><body><joint type="hinge"/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
  valid.body_pos[1, 0] = np.nan
  with pytest.raises(ValueError, match="nonfinite"):
    load_model(valid)


def test_source_mutation_does_not_change_descriptor_and_version_checked_first(monkeypatch):
  source = mujoco.MjModel.from_xml_string(
      '<mujoco><worldbody><body><joint type="hinge"/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
  descriptor = load_model(source)
  before = descriptor.forward_kinematics(np.array(source.qpos0))["body_pos"].copy()
  source.body_pos[1, 0] = 50.
  np.testing.assert_array_equal(
      descriptor.forward_kinematics(np.array(source.qpos0))["body_pos"], before)
  monkeypatch.setattr(model_module.mujoco, "__version__", "3.14.1")
  with pytest.raises(RuntimeError, match="3.10.0"):
    load_model("this is deliberately not valid XML")
