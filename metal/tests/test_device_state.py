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

"""CPU lifecycle tests for owned device state and immutable checkpoints."""

import os

import mujoco
import numpy as np
import pytest

from mujoco_metal.device_state import DeviceState
from mujoco_metal.device_state import StateSnapshot
from mujoco_metal.model import load_model
from mujoco_metal.stepping import validate_stepping_profile


def _model(mass="1", gravity="0 0 -9.81", timestep=".001"):
  return mujoco.MjModel.from_xml_string(
      f"""<mujoco><option timestep='{timestep}' gravity='{gravity}'>
      <flag contact='disable'/></option><worldbody>
      <body><joint type='hinge' armature='.1'/>
      <geom type='sphere' size='.1' mass='{mass}'/></body>
      </worldbody></mujoco>"""
  )


def _state(model=None, batch_size=3, **kwargs):
  model = _model() if model is None else model
  profile = validate_stepping_profile(model)
  return DeviceState(model, profile, batch_size, device="cpu", **kwargs)


def test_initial_state_owns_inputs_and_readers_return_detached_copies():
  model = _model()
  initial_qpos = np.zeros((3, model.nq), dtype=np.float32)
  initial_qvel = np.full((3, model.nv), 0.25, dtype=np.float32)
  state = _state(model, qpos=initial_qpos, qvel=initial_qvel)
  initial_qpos[:] = 123
  initial_qvel[:] = 456
  np.testing.assert_array_equal(state.qpos.numpy(), np.zeros((3, model.nq)))
  np.testing.assert_array_equal(state.qvel.numpy(), np.full((3, model.nv), 0.25))

  returned = state.qpos
  returned[0, 0] = 17
  assert not np.array_equal(returned.numpy(), state.qpos.numpy())
  assert state.generation == 0
  assert state.device == "cpu"


def test_selected_reset_clears_only_requested_rows_and_commits_generation():
  model = _model()
  state = _state(model)
  state._qacc.fill_(2)
  state._time.fill_(3)
  state._status.fill_(7)
  old_qpos = state.qpos
  qpos = np.array([[0.4]], dtype=np.float32)
  qvel = np.array([[0.7]], dtype=np.float32)
  assert state.reset([1], qpos=qpos, qvel=qvel) == 1
  qpos[:] = 8
  qvel[:] = 9

  np.testing.assert_allclose(state.qpos[1].numpy(), [0.4])
  np.testing.assert_allclose(state.qvel[1].numpy(), [0.7])
  np.testing.assert_array_equal(state.qpos[[0, 2]].numpy(), old_qpos[[0, 2]].numpy())
  np.testing.assert_array_equal(state.qacc[:, 0].numpy(), [2, 0, 2])
  np.testing.assert_array_equal(state.time.numpy(), [3, 0, 3])
  np.testing.assert_array_equal(state.status.numpy(), [7, 0, 7])
  assert state.reset([]) == 1


@pytest.mark.parametrize(
    "env_ids, qpos, qvel",
    [
        ([1, 1], None, None),
        ([3], None, None),
        ([1.0], None, None),
        ([1], np.array([[np.nan]]), None),
        ([1], np.array([[0.0, 1.0]]), None),
        ([1], None, np.array([[np.inf]])),
    ],
)
def test_invalid_reset_is_atomic(env_ids, qpos, qvel):
  state = _state()
  state._qacc.fill_(2)
  before = state.snapshot()
  with pytest.raises(ValueError):
    state.reset(env_ids, qpos=qpos, qvel=qvel)
  after = state.snapshot()
  assert state.generation == 0
  for name in ("qpos", "qvel", "qacc", "time", "status"):
    np.testing.assert_array_equal(getattr(after, name), getattr(before, name))


def test_restore_then_seeded_host_reset_is_reproducible():
  state = _state()
  checkpoint = state.snapshot()
  rng = np.random.default_rng(1234)
  first = np.asarray(rng.uniform(-0.5, 0.5, size=(3, 1)), dtype=np.float32)
  state.reset(np.arange(3), qpos=first)
  expected = state.qpos.numpy().copy()
  state.restore(checkpoint)

  rng = np.random.default_rng(1234)
  repeated = np.asarray(rng.uniform(-0.5, 0.5, size=(3, 1)), dtype=np.float32)
  state.reset(np.arange(3), qpos=repeated)
  np.testing.assert_array_equal(state.qpos.numpy(), expected)


def test_rejects_nonunit_free_joint_quaternion_before_reset_mutation():
  model = mujoco.MjModel.from_xml_string(
      """<mujoco><option><flag contact='disable'/></option><worldbody>
      <body><freejoint/><geom type='sphere' size='.1'/></body>
      </worldbody></mujoco>"""
  )
  state = _state(model, batch_size=2)
  before = state.snapshot()
  bad_qpos = np.zeros((1, model.nq), dtype=np.float32)
  with pytest.raises(ValueError, match="quaternions must be unit"):
    state.reset([1], qpos=bad_qpos)
  after = state.snapshot()
  for name in ("qpos", "qvel", "qacc", "time", "status"):
    np.testing.assert_array_equal(getattr(after, name), getattr(before, name))


def test_snapshot_is_immutable_and_restore_round_trips_every_state_field():
  state = _state()
  state._qpos[0, 0] = 0.8
  state._qvel[:, 0].copy_(state._torch.tensor([0.1, 0.2, 0.3]))
  state._qacc[:, 0].copy_(state._torch.tensor([1, 2, 3]))
  state._time.copy_(state._torch.tensor([0.01, 0.02, 0.03]))
  state._status.copy_(state._torch.tensor([0, 2, 0], dtype=state._torch.int32))
  saved = state.snapshot()
  with pytest.raises(ValueError):
    saved.qpos[0, 0] = 3

  state.reset()
  assert state.generation == 1
  assert state.restore(saved) == 2
  restored = state.snapshot()
  for name in ("qpos", "qvel", "qacc", "time", "status"):
    np.testing.assert_array_equal(getattr(restored, name), getattr(saved, name))


def test_descriptor_input_is_snapshotted_instead_of_borrowed():
  from dataclasses import replace

  model = _model()
  descriptor = load_model(model)
  borrowed_body_pos = np.array(descriptor.body_pos, copy=True)
  mutable_descriptor = replace(descriptor, body_pos=borrowed_body_pos)
  profile = validate_stepping_profile(model)
  state = DeviceState(mutable_descriptor, profile, 2, device="cpu")
  borrowed_body_pos[1, 0] = 99
  assert state._model.body_pos[1, 0] == 0
  with pytest.raises(ValueError):
    state._model.body_pos[1, 0] = 1


def test_rejects_foreign_models_even_when_dimensions_match():
  model = _model()
  profile = validate_stepping_profile(model)
  state = DeviceState(model, profile, 2, device="cpu")
  snapshot = state.snapshot()
  assert (_model(mass="2").nq, _model(mass="2").nv) == (model.nq, model.nv)
  for foreign_model in (_model(mass="2"), _model(gravity="0 0 -4")):
    foreign_profile = validate_stepping_profile(foreign_model)
    foreign = DeviceState(foreign_model, foreign_profile, 2, device="cpu")
    before = foreign.snapshot()
    with pytest.raises(ValueError, match="profile does not match"):
      DeviceState(foreign_model, profile, 2, device="cpu")
    with pytest.raises(ValueError, match="do not match"):
      foreign.restore(snapshot)
    np.testing.assert_array_equal(foreign.snapshot().qpos, before.qpos)

  changed = load_model(model)
  changed_body_mass = np.array(changed.body_mass, copy=True)
  changed_body_mass[1] *= 2
  from dataclasses import replace

  with pytest.raises(ValueError, match="descriptor"):
    DeviceState(replace(changed, body_mass=changed_body_mass), profile, 2, device="cpu")

  other_dt = validate_stepping_profile(model, timestep=0.0005)
  other_dt_state = DeviceState(model, other_dt, 2, device="cpu")
  with pytest.raises(ValueError, match="do not match"):
    other_dt_state.restore(snapshot)


def test_rejects_bad_snapshots_and_accepts_static_empty_world():
  model = mujoco.MjModel.from_xml_string(
      "<mujoco><option><flag contact='disable'/></option><worldbody/></mujoco>"
  )
  state = _state(model, batch_size=2)
  assert state.qpos.shape == (2, 0)
  assert state.qvel.shape == (2, 0)
  saved = state.snapshot()
  assert saved.qpos.shape == (2, 0)
  assert state.restore(saved) == 1

  with pytest.raises(ValueError, match="shape"):
    StateSnapshot(
        model_fingerprint=saved.model_fingerprint,
        profile_fingerprint=saved.profile_fingerprint,
        timestep=saved.timestep,
        nq=0,
        nv=0,
        batch_size=2,
        qpos=np.zeros((3, 0)),
        qvel=np.zeros((2, 0)),
        qacc=np.zeros((2, 0)),
        time=np.zeros(2),
        status=np.zeros(2, dtype=np.int32),
    )


@pytest.mark.gpu
@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and an idle Apple GPU",
)
def test_mps_state_reset_restore_and_model_identity():
  torch = pytest.importorskip("torch")
  if not torch.backends.mps.is_available():
    pytest.skip("PyTorch MPS is unavailable")

  model = _model()
  profile = validate_stepping_profile(model)
  state = DeviceState(model, profile, 2)
  state.reset([1], qpos=np.array([[0.25]], dtype=np.float32))
  checkpoint = state.snapshot()
  state.reset()
  state.restore(checkpoint)
  np.testing.assert_allclose(state.qpos.cpu().numpy()[1], [0.25])

  foreign_model = _model(mass="2")
  with pytest.raises(ValueError, match="profile does not match"):
    DeviceState(foreign_model, profile, 2)
