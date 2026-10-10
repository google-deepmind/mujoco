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

"""Opt-in qualification for all-device contact-free simulation stepping."""

import os

import mujoco
import numpy as np
import pytest

from mujoco_metal.simulation import MetalSimulation

_XML = """<mujoco><option timestep=".001" gravity="0 0 -9.81">
  <flag contact="disable"/></option><worldbody>
  <body><joint type="hinge" axis="0 1 0" armature=".01"/>
    <inertial pos=".2 0 0" mass="1" diaginertia=".02 .03 .04"/>
    <geom type="sphere" size=".05"/>
  </body>
</worldbody></mujoco>"""
_MIXED_XML = """<mujoco><option timestep=".001" gravity="0 0 -9.81">
  <flag contact="disable"/></option><worldbody>
  <body><freejoint/><geom type="sphere" size=".05"/>
    <body pos="0 0 .3"><joint type="hinge" axis="0 1 0"/>
      <geom type="box" size=".1 .2 .3"/>
      <body pos=".2 0 0"><joint type="slide" axis="1 0 0"/>
        <geom type="sphere" size=".04"/>
        <body><joint type="ball"/><inertial pos=".1 0 0" mass=".4"
          diaginertia=".01 .02 .03"/></body>
      </body>
    </body>
  </body>
</worldbody></mujoco>"""

pytestmark = pytest.mark.gpu
_requires_gpu = pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
)


def _oracle_step(model, qpos, qvel, count=1):
  data = mujoco.MjData(model)
  data.qpos[:] = qpos
  data.qvel[:] = qvel
  for _ in range(count):
    mujoco.mj_step(model, data)
  return data


def test_invalid_profile_rejected_before_device_construction():
  model = mujoco.MjModel.from_xml_string(
      "<mujoco><worldbody/></mujoco>"
  )
  with pytest.raises(ValueError, match="contact explicitly disabled"):
    MetalSimulation(model)


def test_batch_overflow_rejected_before_device_construction():
  model = mujoco.MjModel.from_xml_string(_XML)
  with pytest.raises(ValueError, match="uint32 index capacity"):
    MetalSimulation(model, batch_size=1 << 30)


def test_fixed_geom_output_overflow_rejected_before_state_allocation(
    monkeypatch,
):
  geoms = "".join(
      f'<geom type="sphere" size=".1" pos="{index} 0 0"/>'
      for index in range(10)
  )
  model = mujoco.MjModel.from_xml_string(
      '<mujoco><option><flag contact="disable"/></option><worldbody>'
      + geoms
      + "</worldbody></mujoco>"
  )
  assert model.nbody == 1 and model.ngeom == 10 and model.nv == 0

  def forbidden_state(*_args, **_kwargs):
    raise AssertionError("device state allocation must follow capacity checks")

  monkeypatch.setattr("mujoco_metal.simulation.DeviceState", forbidden_state)
  with pytest.raises(ValueError, match="geom_quat.*uint32 index capacity"):
    MetalSimulation(model, batch_size=110_000_000)


@_requires_gpu
def test_one_step_batch_matches_mujoco_and_never_uses_host_physics(monkeypatch):
  import torch

  model = mujoco.MjModel.from_xml_string(_MIXED_XML)
  qpos = np.tile(model.qpos0.astype(np.float32), (2, 1))
  qvel = np.zeros((2, model.nv), dtype=np.float32)
  qpos[0, 0] = -.4
  qpos[1, 0] = .25
  qvel[0] = np.linspace(-.2, .3, model.nv)
  qvel[1] = np.linspace(.3, -.2, model.nv)
  references = [_oracle_step(model, qpos[row], qvel[row]) for row in range(2)]
  simulation = MetalSimulation(model, batch_size=2, qpos=qpos, qvel=qvel)

  def forbidden(*_args, **_kwargs):
    raise AssertionError("host physics or state readback entered device step")

  with monkeypatch.context() as block:
    block.setattr(np.linalg, "solve", forbidden)
    block.setattr(mujoco, "mj_step", forbidden)
    block.setattr(mujoco, "mj_integratePos", forbidden)
    block.setattr(torch.Tensor, "cpu", forbidden)
    status = simulation.step()
  assert status.device.type == "mps"
  assert tuple(status.shape) == (2,)

  for row, reference in enumerate(references):
    np.testing.assert_allclose(
        simulation.state.qpos[row].cpu().numpy(),
        reference.qpos,
        rtol=1e-3,
        atol=1e-4,
    )
    np.testing.assert_allclose(
        simulation.state.qvel[row].cpu().numpy(),
        reference.qvel,
        rtol=1e-3,
        atol=1e-3,
    )
    assert simulation.state.status[row].item() == 0


@_requires_gpu
def test_simple_pendulum_thousand_step_trajectory():
  model = mujoco.MjModel.from_xml_string(_XML)
  qpos = np.array([[.7]], dtype=np.float32)
  qvel = np.array([[.3]], dtype=np.float32)
  expected = _oracle_step(model, qpos[0], qvel[0], count=1000)
  simulation = MetalSimulation(model, qpos=qpos, qvel=qvel)
  simulation.step(1000)
  np.testing.assert_allclose(
      simulation.state.qpos[0].cpu().numpy(),
      expected.qpos,
      rtol=1e-3,
      atol=1e-4,
  )
  np.testing.assert_allclose(
      simulation.state.qvel[0].cpu().numpy(),
      expected.qvel,
      rtol=1e-3,
      atol=1e-3,
  )
  np.testing.assert_allclose(
      simulation.state.time.cpu().numpy(),
      [.001 * 1000],
      rtol=0,
      # Float32 time accumulation over 1000 device steps.
      atol=2e-5,
  )


@_requires_gpu
def test_reset_and_restore_replace_state_references_safely():
  model = mujoco.MjModel.from_xml_string(_XML)
  initial_qpos = np.array([[.2], [-.3]], dtype=np.float32)
  initial_qvel = np.array([[.1], [-.1]], dtype=np.float32)
  simulation = MetalSimulation(
      model, batch_size=2, qpos=initial_qpos, qvel=initial_qvel
  )
  simulation.step(3)
  saved = simulation.state.snapshot()
  simulation.step(5)
  simulation.state.restore(saved)
  simulation.step()

  expected = [
      _oracle_step(model, saved.qpos[row], saved.qvel[row]) for row in range(2)
  ]
  for row, reference in enumerate(expected):
    np.testing.assert_allclose(
        simulation.state.qpos[row].cpu().numpy(), reference.qpos,
        rtol=1e-3, atol=1e-4
    )
    np.testing.assert_allclose(
        simulation.state.qvel[row].cpu().numpy(), reference.qvel,
        rtol=1e-3, atol=1e-3
    )

  simulation.state.reset([1], qpos=np.array([[.5]], dtype=np.float32))
  simulation.step()
  assert simulation.state.time[1].item() == pytest.approx(.001, abs=2e-7)
  assert simulation.state.time[0].item() == pytest.approx(.005, abs=2e-6)


@_requires_gpu
def test_solver_failure_rolls_back_only_failed_world(monkeypatch):
  import torch

  model = mujoco.MjModel.from_xml_string(_XML)
  simulation = MetalSimulation(model, batch_size=2)
  simulation.state._qacc[1].fill_(.75)
  before_pos = simulation.state._qpos.detach().clone()
  before_vel = simulation.state._qvel.detach().clone()
  before_time = simulation.state._time.detach().clone()

  def failed_second_row(_mass, rhs):
    return torch.ones_like(rhs), torch.tensor(
        [0, 3], dtype=torch.int32, device="mps"
    )

  monkeypatch.setattr(simulation._solver, "run_device", failed_second_row)
  status = simulation.step()
  np.testing.assert_array_equal(status.cpu().numpy(), [0, 3])
  np.testing.assert_array_equal(
      simulation.state._qpos[1].cpu().numpy(), before_pos[1].cpu().numpy()
  )
  np.testing.assert_array_equal(
      simulation.state._qvel[1].cpu().numpy(), before_vel[1].cpu().numpy()
  )
  np.testing.assert_array_equal(
      simulation.state._time[1].cpu().numpy(), before_time[1].cpu().numpy()
  )
  assert simulation.state._qacc[1, 0].item() == pytest.approx(.75)
  assert simulation.state._time[0].item() == pytest.approx(.001, abs=2e-7)


@_requires_gpu
def test_bad_step_count_does_not_mutate_state():
  model = mujoco.MjModel.from_xml_string(_XML)
  simulation = MetalSimulation(model)
  before = simulation.state.snapshot()
  for steps in (0, -1, True, 1.5):
    with pytest.raises((TypeError, ValueError)):
      simulation.step(steps)
  after = simulation.state.snapshot()
  for name in ("qpos", "qvel", "qacc", "time", "status"):
    np.testing.assert_array_equal(getattr(after, name), getattr(before, name))


@_requires_gpu
def test_failure_status_is_sticky_until_failed_row_reset(monkeypatch):
  import torch

  model = mujoco.MjModel.from_xml_string(_XML)
  simulation = MetalSimulation(model, batch_size=2)
  before_failed_pos = simulation.state._qpos[1].detach().clone()
  before_failed_time = simulation.state._time[1].detach().clone()
  calls = 0

  def fail_once(_mass, rhs):
    nonlocal calls
    calls += 1
    status = [0, 3] if calls == 1 else [0, 0]
    return torch.ones_like(rhs), torch.tensor(
        status, dtype=torch.int32, device="mps"
    )

  monkeypatch.setattr(simulation._solver, "run_device", fail_once)
  np.testing.assert_array_equal(simulation.step().cpu().numpy(), [0, 3])
  np.testing.assert_array_equal(simulation.step().cpu().numpy(), [0, 3])
  np.testing.assert_array_equal(
      simulation.state._qpos[1].cpu().numpy(), before_failed_pos.cpu().numpy()
  )
  np.testing.assert_array_equal(
      simulation.state._time[1].cpu().numpy(), before_failed_time.cpu().numpy()
  )
  assert simulation.state._time[0].item() == pytest.approx(.002, abs=2e-7)

  simulation.state.reset([1], qpos=np.array([[.5]], dtype=np.float32))
  np.testing.assert_array_equal(simulation.step().cpu().numpy(), [0, 0])
  assert simulation.state._time[1].item() == pytest.approx(.001, abs=2e-7)
