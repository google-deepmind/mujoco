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

"""Opt-in MPS tensor-path checks. The ordinary CPU suite never launches GPU."""

import os

import mujoco
import numpy as np
import pytest

from mujoco_metal.metal_kinematics import MetalKinematics
from mujoco_metal.model import load_model
from mujoco_metal.smooth import smooth_dynamics
from mujoco_metal.smooth_metal import MetalSmoothDynamics

pytestmark = [
    pytest.mark.gpu,
    pytest.mark.skipif(
        os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
        reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
    ),
]

_XML = """<mujoco><worldbody>
  <body pos="0 0 .2"><joint type="hinge" axis="0 1 0" armature=".02"/>
    <inertial pos=".2 0 0" mass="1" diaginertia=".02 .03 .04"/>
    <geom type="box" size=".1 .2 .3"/>
    <site pos=".1 0 0"/>
  </body>
</worldbody></mujoco>"""


def _mps(array):
  import torch

  return torch.tensor(array, dtype=torch.float32, device="mps")


def test_device_fk_tracks_in_place_and_replaced_tensor_values():
  descriptor = load_model(_XML)
  model = mujoco.MjModel.from_xml_string(_XML)
  stage = MetalKinematics(descriptor, batch_size=2)
  first = np.tile(model.qpos0, (2, 1))
  qpos = _mps(first)
  out1 = stage.run_device(qpos)
  saved1 = out1["site_pos"].cpu().numpy().copy()

  changed = first.copy()
  changed[0, 0] = .31
  qpos.copy_(_mps(changed))
  out2 = stage.run_device(qpos)
  out2_snapshot = out2["site_pos"].cpu().numpy().copy()
  np.testing.assert_allclose(
      out2_snapshot[0],
      descriptor.forward_kinematics(changed[0])["site_pos"],
      rtol=2e-5,
      atol=2e-6,
  )
  # Workspace views are allowed to change; a new input object is also observed.
  replacement = _mps(np.tile(model.qpos0, (2, 1)))
  replacement[1, 0] = -.23
  out3 = stage.run_device(replacement)
  np.testing.assert_allclose(
      out3["site_pos"][1].cpu().numpy(),
      descriptor.forward_kinematics(replacement[1].cpu().numpy())["site_pos"],
      rtol=2e-5,
      atol=2e-6,
  )
  assert not np.allclose(saved1[0], out2_snapshot[0])


def test_device_smooth_tracks_state_and_public_outputs_keep_ownership():
  descriptor = load_model(_XML)
  model = mujoco.MjModel.from_xml_string(_XML)
  stage = MetalSmoothDynamics(descriptor, batch_size=2)
  qpos_np = np.tile(model.qpos0, (2, 1))
  qvel_np = np.array([[.4], [-.2]])
  device_result = stage.run_device(_mps(qpos_np), _mps(qvel_np))
  for row in range(2):
    expected = smooth_dynamics(descriptor, qpos_np[row], qvel_np[row])
    np.testing.assert_allclose(
        device_result["mass_matrix"][row].cpu().numpy(),
        expected["mass_matrix"],
        rtol=3e-4,
        atol=3e-5,
    )
    np.testing.assert_allclose(
        device_result["qfrc_bias"][row].cpu().numpy(),
        expected["qfrc_bias"],
        rtol=3e-4,
        atol=3e-5,
    )

  # Host convenience methods return owned results across later calls.
  host_first = stage.run(qpos_np, qvel_np)
  preserved_mass = host_first["mass_matrix"].cpu().numpy().copy()
  preserved_bias = host_first["qfrc_bias"].cpu().numpy().copy()
  qpos_changed = qpos_np.copy()
  qpos_changed[0, 0] += .6
  second = stage.run_device(_mps(qpos_changed), _mps(qvel_np))
  assert not np.allclose(second["qfrc_bias"].cpu().numpy(), preserved_bias)
  np.testing.assert_array_equal(
      host_first["mass_matrix"].cpu().numpy(), preserved_mass
  )
  np.testing.assert_array_equal(
      host_first["qfrc_bias"].cpu().numpy(), preserved_bias
  )


def test_device_path_rejects_metadata_errors_and_requires_prepared_batch():
  import torch

  descriptor = load_model(_XML)
  stage = MetalKinematics(descriptor)
  with pytest.raises(ValueError, match="prepare_workspace"):
    stage.run_device(_mps(np.zeros((2, descriptor.nq))))
  with pytest.raises(ValueError, match="positive integer"):
    stage.prepare_workspace(True)
  with pytest.raises(ValueError, match="int32 dimension limit"):
    stage.prepare_workspace(1 << 31)
  with pytest.raises(ValueError, match="uint32 index capacity"):
    stage.prepare_workspace(1 << 30)
  with pytest.raises(ValueError, match="dtype"):
    stage.run_device(
        torch.zeros((1, descriptor.nq), dtype=torch.float16, device="mps")
    )
  with pytest.raises(ValueError, match="contiguous"):
    stage.run_device(
        torch.zeros(
            (2, descriptor.nq * 2), dtype=torch.float32, device="mps"
        )[:, ::2]
    )
  with pytest.raises(ValueError, match="shape"):
    stage.run_device(_mps(np.zeros((1, descriptor.nq + 1))))


def test_empty_world_device_smooth_shapes_for_multiple_worlds():
  descriptor = load_model("<mujoco><worldbody/></mujoco>")
  stage = MetalSmoothDynamics(descriptor, batch_size=3)
  result = stage.run_device(_mps(np.empty((3, 0))), _mps(np.empty((3, 0))))
  assert tuple(result["mass_matrix"].shape) == (3, 0, 0)
  assert tuple(result["qfrc_bias"].shape) == (3, 0)
