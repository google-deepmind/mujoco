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

"""CPU contract tests for generic model lowering."""

import subprocess
import sys
import textwrap

import mujoco
import numpy as np
import pytest

from mujoco_metal.metal_kinematics import _prepare_host_arrays
from mujoco_metal.model import load_model
import mujoco_metal.model as model_module
from mujoco_metal.registry import Execution
from mujoco_metal.registry import Feature
from mujoco_metal.registry import feature_status
from mujoco_metal.registry import Implementation
from mujoco_metal.registry import Qualification
from mujoco_metal.registry import Stage


def test_immutable_lowering_and_empty_world():
  model = load_model(
      '<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
  )
  assert model.nbody == 1
  for value in (model.body_pos, model.jnt_type, model.geom_pos):
    with pytest.raises(ValueError):
      value.flat[0] = 1
    with pytest.raises(ValueError):
      value.setflags(write=True)
  pose = model.forward_kinematics(np.empty(0))
  assert pose["geom_pos"].shape == (1, 3)


def test_mixed_topology_matches_mujoco_oracle():
  xml = """<mujoco><worldbody>
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
  </worldbody></mujoco>"""
  model = load_model(xml)
  compiled = mujoco.MjModel.from_xml_string(xml)
  data = mujoco.MjData(compiled)
  data.qpos[:] = compiled.qpos0
  for joint, typ in enumerate(model.jnt_type):
    qa = model.jnt_qposadr[joint]
    if typ == int(mujoco.mjtJoint.mjJNT_FREE):
      data.qpos[qa : qa + 3] += np.array([0.1, -0.2, 0.3])
      data.qpos[qa + 3 : qa + 7] = [np.cos(0.17), 0, np.sin(0.17), 0]
    elif typ == int(mujoco.mjtJoint.mjJNT_BALL):
      data.qpos[qa : qa + 4] = [np.cos(0.12), np.sin(0.12), 0, 0]
    else:
      data.qpos[qa] += 0.27
  mujoco.mj_kinematics(compiled, data)
  actual = model.forward_kinematics(data.qpos)
  np.testing.assert_allclose(actual["body_pos"], data.xpos, atol=1e-12)
  np.testing.assert_allclose(actual["body_quat"], data.xquat, atol=1e-12)
  np.testing.assert_allclose(actual["geom_pos"], data.geom_xpos, atol=1e-12)
  np.testing.assert_allclose(actual["joint_anchor"], data.xanchor, atol=1e-12)
  np.testing.assert_allclose(actual["joint_axis"], data.xaxis, atol=1e-12)
  np.testing.assert_allclose(actual["site_pos"], data.site_xpos, atol=1e-12)
  assert model.nbody > 17


def test_invalid_quaternion_and_feature_registry_is_host_only():
  model = load_model(
      '<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  )
  qpos = np.zeros(model.nq)
  qpos[3] = 1.0
  qpos[3:7] = 0
  with pytest.raises(ValueError, match="nonzero"):
    model.forward_kinematics(qpos)
  inventory = feature_status()
  assert inventory
  assert all(isinstance(row, Feature) for row in inventory)
  assert any(
      row.name == "generic Metal joint FK"
      and row.qualification == Qualification.GPU_QUALIFIED
      for row in inventory
  )
  assert any(
      row.name == "integrators/stepping"
      and row.qualification == Qualification.UNQUALIFIED
      for row in inventory
  )
  assert any(row.name == "mjtJoint.mjJNT_FREE" for row in inventory)
  assert any(row.name.startswith("api:") for row in inventory)
  assert all(row.execution in Execution for row in inventory)
  assert all(
      row.stage in Stage and row.implementation in Implementation
      for row in inventory
  )


def test_rejects_mocap_and_malformed_source_model():
  mocap = mujoco.MjModel.from_xml_string(
      '<mujoco><worldbody><body mocap="true"><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  )
  with pytest.raises(ValueError, match="mocap"):
    load_model(mocap)

  valid = mujoco.MjModel.from_xml_string(
      '<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
  )
  valid.geom_bodyid[0] = 999
  with pytest.raises(ValueError, match="geom_bodyid"):
    load_model(valid)

  valid = mujoco.MjModel.from_xml_string(
      '<mujoco><worldbody><body><joint type="hinge"/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  )
  valid.body_pos[1, 0] = np.nan
  with pytest.raises(ValueError, match="nonfinite"):
    load_model(valid)


def test_source_mutation_does_not_change_descriptor_and_version_checked_first(
    monkeypatch,
):
  source = mujoco.MjModel.from_xml_string(
      '<mujoco><worldbody><body><joint type="hinge"/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  )
  descriptor = load_model(source)
  before = descriptor.forward_kinematics(np.array(source.qpos0))[
      "body_pos"
  ].copy()
  source.body_pos[1, 0] = 50.0
  np.testing.assert_array_equal(
      descriptor.forward_kinematics(np.array(source.qpos0))["body_pos"], before
  )
  monkeypatch.setattr(model_module.mujoco, "__version__", "3.14.1")
  with pytest.raises(RuntimeError, match="3.10.0"):
    load_model("this is deliberately not valid XML")


def test_metal_host_buffer_packing_without_torch_or_mps():
  empty = load_model("<mujoco><worldbody/></mujoco>")
  packed = _prepare_host_arrays(empty)
  assert packed["jnt_type"].shape == (1,)
  assert packed["geom_bodyid"].shape == (1,)
  assert packed["site_bodyid"].shape == (1,)

  moving = load_model(
      '<mujoco><worldbody><body><joint type="hinge"/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  )
  bad = __import__("dataclasses").replace(
      moving, body_pos=np.array(moving.body_pos, copy=True)
  )
  altered = np.array(bad.body_pos, copy=True)
  altered[1, 0] = np.nan
  bad = __import__("dataclasses").replace(bad, body_pos=altered)
  with pytest.raises(ValueError, match="nonfinite"):
    _prepare_host_arrays(bad)


def test_preflight_is_cpu_only_and_includes_stage_inventory(tmp_path):
  from mujoco_metal.__main__ import preflight

  result = preflight(include_inventory=True)
  assert result["gpu_qualified"] is False
  assert "complete physics backend" in result["gpu_qualification_scope"]
  assert result["shader_sha256"]
  assert set(result["shaders"]) == {
      "kinematics",
      "smooth_mass",
      "smooth_bias",
      "smooth_solve",
      "integration",
  }
  assert all(row["sha256"] for row in result["shaders"].values())
  assert result["inventory_complete"] is False
  assert "narrowly GPU-qualified" in result["stages"]["dynamics"]
  assert result["stages"]["acceleration_solve"].startswith("native dense SPD")
  assert result["stages"]["integration"].startswith(
      "native semi-implicit Euler"
  )
  assert "narrowly GPU-qualified" in result["stages"]["contact_free_euler_v1"]
  assert result["stages"]["full_stepping"].startswith("unsupported")
  assert any(
      row["name"].startswith("python-api:mj_") for row in result["features"]
  )
  assert any(
      row["name"] == "native dense SPD factorization and multiple-RHS solve"
      and row["qualification"] == "gpu_qualified"
      for row in result["features"]
  )
  assert any(
      row["name"] == "contact_free_euler_v1 native simulation pipeline"
      and row["qualification"] == "gpu_qualified"
      for row in result["features"]
  )


def test_host_only_apis_do_not_import_torch_in_fresh_process():
  script = textwrap.dedent("""
      import sys
      from mujoco_metal.__main__ import preflight
      from mujoco_metal import MetalDenseSolve
      from mujoco_metal import MetalEulerIntegration
      from mujoco_metal import MetalSimulation
      from mujoco_metal.metal_kinematics import _prepare_host_arrays
      from mujoco_metal.model import load_model

      model = load_model('<mujoco><worldbody/></mujoco>')
      _prepare_host_arrays(model)
      preflight(include_inventory=True)
      assert MetalDenseSolve.__name__ == 'MetalDenseSolve'
      assert MetalEulerIntegration.__name__ == 'MetalEulerIntegration'
      assert MetalSimulation.__name__ == 'MetalSimulation'
      assert 'torch' not in sys.modules
      """)
  subprocess.run([sys.executable, "-c", script], check=True)


def test_zero_length_output_reshape_discards_dummy_buffer():
  from mujoco_metal.metal_kinematics import _shape_output

  np.testing.assert_array_equal(
      _shape_output(np.zeros(1, dtype=np.float32), 2, 0, 3),
      np.empty((2, 0, 3), dtype=np.float32),
  )
  np.testing.assert_array_equal(
      _shape_output(np.arange(7, dtype=np.float32), 2, 1, 3),
      np.arange(6, dtype=np.float32).reshape(2, 1, 3),
  )


def test_model_lifecycle_calls_setconst_and_commits_atomically(monkeypatch):
  from mujoco_metal.lifecycle import ModelLifecycle

  xml = '<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  source = mujoco.MjModel.from_xml_string(xml)
  lifecycle = ModelLifecycle(source)
  source_mass = float(source.body_mass[1])
  original_setconst = mujoco.mj_setConst
  calls = []

  def count_setconst(model, data):
    calls.append(True)
    original_setconst(model, data)

  monkeypatch.setattr(mujoco, "mj_setConst", count_setconst)
  assert lifecycle.recompute_body_masses([1], [2.5])
  assert calls == [True]
  assert lifecycle.generation == 1
  assert lifecycle.descriptor.body_mass[1] == 2.5
  assert source.body_mass[1] == source_mass
  assert not lifecycle.recompute_body_masses([1], [2.5])
  assert lifecycle.generation == 1

  before = lifecycle.descriptor

  def fail_setconst(model, data):
    raise RuntimeError("injected setConst failure")

  monkeypatch.setattr(mujoco, "mj_setConst", fail_setconst)
  with pytest.raises(RuntimeError, match="injected"):
    lifecycle.recompute_body_masses([1], [3.0])
  assert lifecycle.descriptor is before
  assert lifecycle.generation == 1


def test_batched_qpos_updates_explicit_rows_randomize_and_restore():
  from mujoco_metal.lifecycle import KinematicsBatchState

  xml = '<mujoco><worldbody><body><joint type="hinge"/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  model = load_model(xml)
  state = KinematicsBatchState(model, batch_size=4)
  assert np.all(state.row_generation == 0)
  state.poses(2)
  same = state.qpos[[2]].copy()
  assert state.set_qpos([2], same) == ()
  assert state.row_generation[2] == 0

  changed = same.copy()
  changed[0, model.jnt_qposadr[0]] += 0.5
  assert state.set_qpos([2], changed) == (2,)
  assert state.row_generation[2] == 1
  assert state.row_generation[1] == 0
  assert not np.allclose(
      state.poses(2)["body_quat"],
      model.forward_kinematics(same[0])["body_quat"],
  )

  snapshot = state.snapshot()
  state.randomize([0, 3], seed=41, scale=0.2)
  assert state.row_generation[0] == 1
  assert state.row_generation[3] == 1
  assert state.row_generation[1] == 0
  randomized = state.qpos.copy()
  generations = state.row_generation.copy()
  state.restore(snapshot)
  np.testing.assert_array_equal(state.qpos, snapshot.qpos)
  np.testing.assert_array_equal(state.row_generation, generations + 1)
  assert not np.array_equal(randomized, state.qpos)
  with pytest.raises(ValueError, match="unique"):
    state.set_qpos([1, 1], np.zeros((2, model.nq)))


def test_randomized_mixed_joint_cpu_oracle():
  xml = """<mujoco><worldbody>
      <geom type="plane" size="2 2 .1"/>
      <body name="fixed" pos=".2 -.1 .3" quat=".9238795 0 0 .3826834">
        <geom type="sphere" size=".1"/>
      </body>
      <body name="free"><freejoint/><geom type="sphere" size=".1"/>
        <body name="multi" pos=".1 .2 0"><geom type="sphere" size=".1"/>
          <joint type="hinge" pos=".1 0 0" axis="0 1 0"/>
          <joint type="slide" axis="1 0 0"/>
          <site pos=".1 0 .2"/>
          <body pos="0 0 .3"><joint type="ball" pos="0 0 .2"/>
            <geom type="sphere" size=".1"/>
            <body pos="0 0 .4"><joint type="slide" axis="0 1 0"/>
              <geom type="sphere" size=".1"/>
            </body>
          </body>
        </body>
      </body>
      <body name="second"><freejoint/><geom type="box" size=".2 .1 .1"/></body>
    </worldbody></mujoco>"""
  model = load_model(xml)
  compiled = mujoco.MjModel.from_xml_string(xml)
  data = mujoco.MjData(compiled)
  rng = np.random.default_rng(319)
  for _ in range(100):
    qpos = np.array(compiled.qpos0)
    for joint, typ in enumerate(model.jnt_type):
      qa = int(model.jnt_qposadr[joint])
      typ = int(typ)
      if typ == int(mujoco.mjtJoint.mjJNT_FREE):
        qpos[qa : qa + 3] = rng.normal(size=3)
        quat = rng.normal(size=4)
        qpos[qa + 3 : qa + 7] = quat / np.linalg.norm(quat)
      elif typ == int(mujoco.mjtJoint.mjJNT_BALL):
        quat = rng.normal(size=4)
        qpos[qa : qa + 4] = quat / np.linalg.norm(quat)
      else:
        qpos[qa] = compiled.qpos0[qa] + rng.normal()
    mujoco.mj_kinematics(compiled, data)
    data.qpos[:] = qpos
    mujoco.mj_kinematics(compiled, data)
    result = model.forward_kinematics(qpos)
    np.testing.assert_allclose(result["body_pos"], data.xpos, atol=2e-12)
    np.testing.assert_allclose(result["body_quat"], data.xquat, atol=2e-12)
    np.testing.assert_allclose(result["geom_pos"], data.geom_xpos, atol=2e-12)
    np.testing.assert_allclose(result["site_pos"], data.site_xpos, atol=2e-12)
    np.testing.assert_allclose(result["joint_anchor"], data.xanchor, atol=2e-12)
    np.testing.assert_allclose(result["joint_axis"], data.xaxis, atol=2e-12)


def test_batched_constants_recompute_dirty_rows_and_restore(monkeypatch):
  from copy import copy

  from mujoco_metal.lifecycle import BatchedConstants

  xml = """<mujoco><worldbody>
      <body><freejoint/><geom type="sphere" size=".1"/>
        <body><joint type="hinge"/><geom type="sphere" size=".1"/></body>
      </body>
    </worldbody></mujoco>"""
  source = mujoco.MjModel.from_xml_string(xml)
  constants = BatchedConstants(source, batch_size=4)
  original_setconst = mujoco.mj_setConst
  calls = []

  def count_setconst(model, data):
    calls.append(True)
    original_setconst(model, data)

  monkeypatch.setattr(mujoco, "mj_setConst", count_setconst)
  assert constants.set_body_masses([1, 3], [1], [[2.0], [4.0]]) == (1, 3)
  assert len(calls) == 2
  np.testing.assert_array_equal(constants.row_generation, [0, 1, 0, 1])
  assert constants.set_body_masses([1, 3], [1], [[2.0], [4.0]]) == ()
  assert len(calls) == 2
  snapshot = constants.snapshot()
  before_mass = constants.body_mass.copy()
  before_invweight = constants.body_invweight0.copy()
  before_generation = constants.row_generation.copy()

  calls.clear()
  fail_at = 2

  def fail_second_candidate(model, data):
    calls.append(True)
    original_setconst(model, data)
    if len(calls) == fail_at:
      raise RuntimeError("injected row recomputation failure")

  monkeypatch.setattr(mujoco, "mj_setConst", fail_second_candidate)
  with pytest.raises(RuntimeError, match="injected"):
    constants.set_body_masses([0, 2], [2], [[3.0], [5.0]])
  np.testing.assert_array_equal(constants.body_mass, before_mass)
  np.testing.assert_array_equal(constants.body_invweight0, before_invweight)
  np.testing.assert_array_equal(constants.row_generation, before_generation)

  monkeypatch.setattr(mujoco, "mj_setConst", original_setconst)
  assert constants.randomize([0, 2], seed=17, scale=0) == ()
  assert constants.randomize([0, 2], seed=17, scale=0.2) == (0, 2)
  randomized = constants.body_mass.copy()
  generations = constants.row_generation.copy()
  constants.restore(snapshot)
  np.testing.assert_array_equal(constants.body_mass, snapshot.body_mass)
  np.testing.assert_array_equal(constants.row_generation, generations + 1)
  assert not np.array_equal(randomized, constants.body_mass)

  expected = []
  for masses in snapshot.body_mass:
    model = copy(source)
    model.body_mass[:] = masses
    mujoco.mj_setConst(model, mujoco.MjData(model))
    expected.append(model.body_invweight0.copy())
  np.testing.assert_allclose(
      constants.body_invweight0, expected, rtol=0, atol=0
  )

  other = BatchedConstants(
      '<mujoco><worldbody><body><freejoint/><geom type="box" size=".1 .2 .3"/></body></worldbody></mujoco>',
      batch_size=4,
  )
  with pytest.raises(ValueError, match="fingerprint"):
    other.restore(snapshot)


def test_batch_state_accessors_and_multienv_failure_are_isolated():
  from mujoco_metal.lifecycle import KinematicsBatchState

  model = load_model(
      '<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  )
  state = KinematicsBatchState(model, 2)
  state.poses(0)
  public_qpos = state.qpos
  public_qpos.setflags(write=True)
  public_qpos[0, 0] += 2
  assert state.poses(0)["body_pos"][1, 0] == 0

  before = state.qpos.copy()
  invalid = before[[0, 1]].copy()
  invalid[0, 0] = 3
  invalid[1, 3:7] = 0
  with pytest.raises(ValueError, match="quaternion"):
    state.set_qpos([0, 1], invalid)
  np.testing.assert_array_equal(state.qpos, before)
  with pytest.raises(ValueError, match="integers"):
    state.set_qpos([0.7], np.zeros((1, model.nq)))


def test_massless_fixed_body_constants_preserve_zero_mass():
  from mujoco_metal.lifecycle import BatchedConstants

  xml = """<mujoco><worldbody><body name="massless-fixed">
      <body name="moving"><joint type="hinge"/><geom type="sphere" size=".1"/></body>
    </body></worldbody></mujoco>"""
  constants = BatchedConstants(xml, batch_size=2)
  assert constants.body_mass[0, 1] == 0
  assert constants.randomize([0, 1], seed=4, scale=0.1) == (0, 1)
  np.testing.assert_array_equal(constants.body_mass[:, 1], 0)
  with pytest.raises(ValueError, match="massless"):
    constants.set_body_masses([0], [1], [[1.0]])
  checkpoint = constants.snapshot()
  constants.set_body_masses([0], [2], [[1.5]])
  constants.restore(checkpoint)
  np.testing.assert_array_equal(constants.body_mass[:, 1], 0)


def test_state_snapshot_fingerprint_and_rotation_scale():
  from dataclasses import replace

  from mujoco_metal.lifecycle import KinematicsBatchState

  xml_a = '<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  xml_b = '<mujoco><worldbody><body><freejoint/><geom type="box" size=".1 .1 .1"/></body></worldbody></mujoco>'
  model_a = load_model(xml_a)
  model_b = load_model(xml_b)
  state = KinematicsBatchState(model_a, batch_size=2)
  original = state.qpos.copy()
  assert state.randomize([0], seed=9, scale=0) == ()
  np.testing.assert_array_equal(state.qpos, original)
  assert state.randomize([0], seed=9, scale=0.01) == (0,)
  free_quat = state.qpos[0, 3:7]
  angle = 2 * np.arccos(np.clip(abs(np.dot(free_quat, original[0, 3:7])), 0, 1))
  assert 0 < angle < 0.1
  snapshot = state.snapshot()
  with pytest.raises(ValueError, match="fingerprint"):
    KinematicsBatchState(model_b, batch_size=2).restore(snapshot)
  with pytest.raises(ValueError, match="fingerprint"):
    state.restore(replace(snapshot, schema_version=2))
  with pytest.raises(ValueError, match="batch_size"):
    KinematicsBatchState(model_a, batch_size=1.5)


def test_constants_snapshot_same_dimensions_rejects_armature_change():
  from dataclasses import replace

  from mujoco_metal.lifecycle import BatchedConstants

  xml_a = """<mujoco><worldbody><body>
      <joint type="hinge" armature=".1"/>
      <inertial pos="0 0 0" mass="1" diaginertia=".1 .2 .3"/>
    </body></worldbody></mujoco>"""
  xml_b = xml_a.replace('armature=".1"', 'armature=".2"').replace(
      'pos="0 0 0"', 'pos=".1 0 0"'
  )
  a = BatchedConstants(xml_a, batch_size=1)
  b = BatchedConstants(xml_b, batch_size=1)
  assert a.descriptor.nq == b.descriptor.nq
  assert a.descriptor.nbody == b.descriptor.nbody
  checkpoint = a.snapshot()
  with pytest.raises(ValueError, match="fingerprint"):
    b.restore(checkpoint)
  with pytest.raises(ValueError, match="schema"):
    a.restore(replace(checkpoint, schema_version=2))
  with pytest.raises(ValueError):
    checkpoint.body_mass.setflags(write=True)


def test_batch_state_owns_and_validates_replaced_descriptor_arrays():
  from dataclasses import replace

  from mujoco_metal.lifecycle import KinematicsBatchState

  loaded = load_model(
      '<mujoco><worldbody><body pos="1 0 0"><joint type="hinge"/>'
      '<geom type="sphere" size=".1"/></body></worldbody></mujoco>'
  )
  borrowed_pos = np.array(loaded.body_pos, copy=True)
  replaced = replace(loaded, body_pos=borrowed_pos)
  state = KinematicsBatchState(replaced, batch_size=1)
  first = state.poses(0)["body_pos"].copy()
  borrowed_pos[1, 0] = 9.0
  second = state.poses(0)["body_pos"]
  np.testing.assert_array_equal(second, first)
  with pytest.raises(ValueError):
    state.model.body_pos.setflags(write=True)

  invalid_pos = np.array(loaded.body_pos, copy=True)
  invalid_pos[1, 0] = np.nan
  with pytest.raises(ValueError, match="nonfinite"):
    KinematicsBatchState(replace(loaded, body_pos=invalid_pos), batch_size=1)
