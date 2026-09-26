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

"""Opt-in GPU qualification tests; never run during ordinary CPU checks."""

import os

import mujoco
import numpy as np
import pytest

from mujoco_metal.metal_kinematics import MetalKinematics
from mujoco_metal.model import load_model

_MIXED_XML = """<mujoco><worldbody>
<body><freejoint/><geom type="sphere" size=".1"/>
  <body pos=".2 0 0"><geom type="sphere" size=".1"/><joint type="hinge" pos=".1 0 0" axis="0 1 0"/>
    <site pos="0 0 1"/><body><joint type="slide" axis="1 0 0"/>
      <geom type="capsule" size=".1 .2"/></body>
  </body>
</body>
<body><freejoint/><geom type="box" size=".1 .2 .3"/></body>
</worldbody></mujoco>"""

pytestmark = pytest.mark.gpu


@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
)
def test_batched_native_fk_matches_cpu_oracle():
  xml = _MIXED_XML
  descriptor = load_model(xml)
  compiled = mujoco.MjModel.from_xml_string(xml)
  batch = np.tile(compiled.qpos0, (3, 1))
  batch[1, 0] += 0.1
  batch[1, compiled.jnt_qposadr[1]] += 0.2
  batch[2, compiled.jnt_qposadr[-1]] -= 0.12
  device_output = MetalKinematics(descriptor).run(batch)
  for world in range(batch.shape[0]):
    reference = descriptor.forward_kinematics(batch[world])
    for key in reference:
      tensor = device_output[key][world].cpu().numpy()
      np.testing.assert_allclose(tensor, reference[key], rtol=2e-5, atol=2e-6)


def test_gpu_fixture_compiles_on_cpu():
  mujoco.MjModel.from_xml_string(_MIXED_XML)


@pytest.mark.skipif(
    os.environ.get("MUJOCO_METAL_RUN_GPU") != "1",
    reason="requires explicit MUJOCO_METAL_RUN_GPU=1 and idle GPU",
)
def test_empty_world_zero_geom_site_batch():
  descriptor = load_model("<mujoco><worldbody/></mujoco>")
  result = MetalKinematics(descriptor).run(np.empty((2, 0)))
  assert tuple(result["geom_pos"].shape) == (2, 0, 3)
  assert tuple(result["site_quat"].shape) == (2, 0, 4)
  np.testing.assert_allclose(
      result["body_quat"].cpu().numpy(),
      np.tile([[[1.0, 0.0, 0.0, 0.0]]], (2, 1, 1)),
  )
