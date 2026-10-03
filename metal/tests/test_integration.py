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

"""CPU metadata and opt-in GPU tests for native semi-implicit Euler."""

import os
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from mujoco_metal.integration import MetalEulerIntegration
from mujoco_metal.model import load_model

pytestmark = pytest.mark.gpu

_MIXED_XML = """<mujoco><worldbody>
  <body><freejoint/><geom type="sphere" size=".1"/>
    <body pos=".2 0 0"><joint type="hinge" axis="0 1 0"/>
      <geom type="box" size=".1 .2 .3"/>
      <body pos="0 0 .3"><joint type="ball"/>
        <geom type="capsule" size=".04 .1"/>
        <body pos=".2 0 0"><joint type="slide" axis="1 0 0"/>
          <geom type="sphere" size=".08"/></body>
      </body>
    </body>
  </body>
</worldbody></mujoco>"""


def test_timestep_validation_precedes_device_initialization():
  descriptor = load_model(_MIXED_XML)
  for timestep in (0, -1, float("inf"), float("nan"), 1e-100, 1e100):
    with pytest.raises(ValueError, match="timestep"):
      MetalEulerIntegration(descriptor, 1, timestep)
  with pytest.raises(TypeError, match="batch_size"):
    MetalEulerIntegration(descriptor, True, 0.01)


def test_metadata_rejection_does_not_touch_workspace():
  stage = object.__new__(MetalEulerIntegration)
  f32, i32 = object(), object()

  class FakeTensor:

    def __init__(self, shape, dtype):
      self.shape = shape
      self.dtype = dtype

    @property
    def device(self):
      return SimpleNamespace(type="mps")

    def is_contiguous(self):
      return True

  stage._torch = SimpleNamespace(Tensor=FakeTensor, float32=f32, int32=i32)
  stage._device = "mps"
  stage.batch_size, stage.nq, stage.nv = 2, 7, 6
  stage._candidate_qpos = np.full(14, 7.0)
  with pytest.raises(ValueError, match="qpos must have shape"):
    stage.run_device(
        FakeTensor((2, 6), f32),
        FakeTensor((2, 6), f32),
        FakeTensor((2, 6), f32),
        FakeTensor((2,), f32),
        FakeTensor((2,), i32),
    )
  np.testing.assert_array_equal(stage._candidate_qpos, np.full(14, 7.0))


@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
)
def test_native_semi_implicit_euler_matches_mujoco_310():
  import torch

  model = mujoco.MjModel.from_xml_string(_MIXED_XML)
  descriptor = load_model(model)
  assert descriptor.nq != descriptor.nv
  rng = np.random.default_rng(1401)
  batch = 3
  qpos = np.tile(np.asarray(model.qpos0, dtype=np.float32), (batch, 1))
  for row in range(batch):
    for joint, typ in enumerate(model.jnt_type):
      qa = int(model.jnt_qposadr[joint])
      if typ == int(mujoco.mjtJoint.mjJNT_FREE):
        qpos[row, qa : qa + 3] = rng.normal(0, 0.2, 3)
        quat = rng.normal(size=4).astype(np.float32)
        quat /= np.linalg.norm(quat)
        if row == 1:
          quat *= -1
        qpos[row, qa + 3 : qa + 7] = quat
      elif typ == int(mujoco.mjtJoint.mjJNT_BALL):
        quat = rng.normal(size=4).astype(np.float32)
        quat /= np.linalg.norm(quat)
        if row == 1:
          quat *= -1
        qpos[row, qa : qa + 4] = quat
      else:
        qpos[row, qa] = rng.normal()
  qvel = rng.normal(0, 0.8, size=(batch, model.nv)).astype(np.float32)
  qacc = rng.normal(0, 2.0, size=(batch, model.nv)).astype(np.float32)
  qvel[0, 3:6] = [1e4, -5e3, 2e3]
  qvel[2] = 0  # Zero angular speed takes the identity quaternion increment.
  qacc[2] = 0
  qvel[1, 3:6] = [1e-8, -2e-8, 3e-8]
  qacc[1, 3:6] = 0
  ball = int(
      np.flatnonzero(model.jnt_type == int(mujoco.mjtJoint.mjJNT_BALL))[0]
  )
  ball_dadr = int(model.jnt_dofadr[ball])
  qvel[1, ball_dadr : ball_dadr + 3] = [1e-8, -2e-8, 3e-8]
  qacc[1, ball_dadr : ball_dadr + 3] = 0
  times = np.array([0.0, 0.7, 12.0], dtype=np.float32)
  timestep = np.float32(0.001)
  originals = (qpos.copy(), qvel.copy(), qacc.copy(), times.copy())
  stage = MetalEulerIntegration(descriptor, batch, timestep)
  qpos_gpu = torch.from_numpy(qpos).to("mps")
  qvel_gpu = torch.from_numpy(qvel).to("mps")
  qacc_gpu = torch.from_numpy(qacc).to("mps")
  time_gpu = torch.from_numpy(times).to("mps")
  solve_status = torch.zeros(batch, dtype=torch.int32, device="mps")
  actual_qpos, actual_qvel, actual_time, status = stage.run_device(
      qpos_gpu, qvel_gpu, qacc_gpu, time_gpu, solve_status
  )
  np.testing.assert_array_equal(status.cpu().numpy(), np.zeros(batch, np.int32))
  qpos_next = actual_qpos.cpu().numpy()
  qvel_next = actual_qvel.cpu().numpy()
  time_next = actual_time.cpu().numpy()
  for row in range(batch):
    reference_qvel = (originals[1][row] + timestep * originals[2][row]).astype(
        np.float64
    )
    reference_qpos = originals[0][row].astype(np.float64)
    mujoco.mj_integratePos(
        model, reference_qpos, reference_qvel, float(timestep)
    )
    np.testing.assert_allclose(
        qpos_next[row], reference_qpos, atol=3e-6, rtol=2e-6
    )
    np.testing.assert_allclose(
        qvel_next[row], reference_qvel, atol=3e-7, rtol=2e-6
    )
  np.testing.assert_allclose(time_next, times + timestep, atol=1e-7, rtol=0)
  np.testing.assert_array_equal(qpos_gpu.cpu().numpy(), originals[0])
  np.testing.assert_array_equal(qvel_gpu.cpu().numpy(), originals[1])
  np.testing.assert_array_equal(qacc_gpu.cpu().numpy(), originals[2])
  np.testing.assert_array_equal(time_gpu.cpu().numpy(), originals[3])


@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
)
def test_failed_rows_roll_back_and_solver_failure_is_preserved():
  import torch

  model = mujoco.MjModel.from_xml_string(_MIXED_XML)
  descriptor = load_model(model)
  batch = 5
  qpos = np.tile(np.asarray(model.qpos0, dtype=np.float32), (batch, 1))
  qvel = np.zeros((batch, model.nv), dtype=np.float32)
  qacc = np.zeros_like(qvel)
  times = np.arange(batch, dtype=np.float32)
  # The zero quaternion is invalid, while NaN acceleration and scalar position
  # overflow exercise independent row failures in the same launch.
  free_joint = int(
      np.flatnonzero(model.jnt_type == int(mujoco.mjtJoint.mjJNT_FREE))[0]
  )
  free_qadr = int(model.jnt_qposadr[free_joint])
  qpos[1, free_qadr + 3 : free_qadr + 7] = 0
  qacc[2, 0] = np.nan
  scalar = int(
      np.flatnonzero(model.jnt_type == int(mujoco.mjtJoint.mjJNT_HINGE))[0]
  )
  qadr = int(model.jnt_qposadr[scalar])
  dadr = int(model.jnt_dofadr[scalar])
  qpos[3, qadr] = 3.2e38
  qvel[3, dadr] = 3.0e38
  solve_status = np.zeros(batch, dtype=np.int32)
  solve_status[4] = 6
  inputs = (qpos.copy(), qvel.copy(), qacc.copy(), times.copy())
  stage = MetalEulerIntegration(descriptor, batch, 1.0)
  outputs = stage.run_device(
      torch.from_numpy(qpos).to("mps"),
      torch.from_numpy(qvel).to("mps"),
      torch.from_numpy(qacc).to("mps"),
      torch.from_numpy(times).to("mps"),
      torch.from_numpy(solve_status).to("mps"),
  )
  out_qpos, out_qvel, out_time, status = [
      item.cpu().numpy() for item in outputs
  ]
  np.testing.assert_array_equal(status, [0, 11, 10, 12, 6])
  for row in range(1, batch):
    np.testing.assert_array_equal(out_qpos[row], inputs[0][row])
    np.testing.assert_array_equal(out_qvel[row], inputs[1][row])
    assert out_time[row] == inputs[3][row]


@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
)
def test_empty_static_world_and_empty_batch_dimensions():
  import torch

  descriptor = load_model("<mujoco><worldbody/></mujoco>")
  stage = MetalEulerIntegration(descriptor, 2, 0.01)
  qpos, qvel, time, status = stage.run_device(
      torch.empty((2, 0), dtype=torch.float32, device="mps"),
      torch.empty((2, 0), dtype=torch.float32, device="mps"),
      torch.empty((2, 0), dtype=torch.float32, device="mps"),
      torch.tensor([1.0, 2.0], dtype=torch.float32, device="mps"),
      torch.zeros(2, dtype=torch.int32, device="mps"),
  )
  assert tuple(qpos.shape) == (2, 0)
  assert tuple(qvel.shape) == (2, 0)
  np.testing.assert_allclose(time.cpu().numpy(), [1.01, 2.01], atol=1e-6)
  np.testing.assert_array_equal(status.cpu().numpy(), [0, 0])
