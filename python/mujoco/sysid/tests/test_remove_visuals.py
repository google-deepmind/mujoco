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
"""Removing visualization data preserves the model used for identification."""

import numpy as np

import mujoco
from mujoco.sysid._src import model_modifier


def test_remove_visuals_retains_shared_collision_mesh():
  spec = mujoco.MjSpec.from_string("""
  <mujoco>
    <asset>
      <mesh name="shape" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
    </asset>
    <worldbody>
      <body>
        <joint/>
        <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/>
        <geom name="collision" type="mesh" mesh="shape"/>
        <geom name="visual" type="mesh" mesh="shape"
              contype="0" conaffinity="0"/>
      </body>
    </worldbody>
  </mujoco>
  """)

  cleaned = model_modifier.remove_visuals(spec)
  model = cleaned.compile()

  assert cleaned.geom("visual") is None
  assert cleaned.mesh("shape") is not None
  assert model.ngeom == 1
  assert model.nmesh == 1
  assert model.geom("collision").dataid == model.mesh("shape").id
  assert spec.geom("visual") is not None
  assert spec.compile().ngeom == 2


def test_remove_visuals_clears_material_references():
  spec = mujoco.MjSpec.from_string("""
  <mujoco>
    <asset>
      <texture name="texture" type="2d" builtin="checker" width="8" height="8"/>
      <material name="material" texture="texture"/>
    </asset>
    <worldbody>
      <body>
        <joint/>
        <geom name="collision" size="0.1" material="material"/>
      </body>
    </worldbody>
  </mujoco>
  """)

  cleaned = model_modifier.remove_visuals(spec)
  model = cleaned.compile()

  assert not cleaned.materials
  assert not cleaned.textures
  assert cleaned.geom("collision").material == ""
  assert model.ngeom == 1
  assert spec.geom("collision").material == "material"
  assert spec.compile().nmat == 1


def test_remove_visuals_preserves_inferred_inertia_and_acceleration():
  spec = mujoco.MjSpec.from_string("""
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <body name="body">
        <joint name="slide" type="slide"/>
        <geom name="visual" size="0.1" mass="1" pos="0.2 0 0"
              contype="0" conaffinity="0"/>
        <geom name="collision" size="0.1" mass="2"/>
      </body>
    </worldbody>
    <actuator><motor joint="slide"/></actuator>
  </mujoco>
  """)
  original = spec.compile()

  cleaned = model_modifier.remove_visuals(spec)
  model = cleaned.compile()

  assert cleaned.geom("visual") is None
  np.testing.assert_array_equal(model.body_mass, original.body_mass)
  np.testing.assert_allclose(model.body_inertia, original.body_inertia)
  np.testing.assert_allclose(model.body_ipos, original.body_ipos)
  original_data = mujoco.MjData(original)
  cleaned_data = mujoco.MjData(model)
  original_data.ctrl[:] = cleaned_data.ctrl[:] = 1.0
  mujoco.mj_forward(original, original_data)
  mujoco.mj_forward(model, cleaned_data)
  np.testing.assert_allclose(cleaned_data.qacc, original_data.qacc)


def test_remove_visuals_keeps_geoms_used_by_explicit_contact_pairs():
  spec = mujoco.MjSpec.from_string("""
  <mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="1 1 0.1"/>
      <body pos="0 0 0.1">
        <joint type="slide"/>
        <geom name="paired" size="0.1" contype="0" conaffinity="0"/>
      </body>
    </worldbody>
    <contact><pair geom1="floor" geom2="paired"/></contact>
  </mujoco>
  """)

  cleaned = model_modifier.remove_visuals(spec)
  model = cleaned.compile()

  assert cleaned.geom("paired") is not None
  assert model.ngeom == 2
  assert model.npair == 1
  assert model.pair_geom2[0] == model.geom("paired").id
