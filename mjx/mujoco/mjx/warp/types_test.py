# Copyright 2026 DeepMind Technologies Limited
#
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
# ==============================================================================
"""Tests for generated MJX Warp types."""

import dataclasses
from absl.testing import absltest
from absl.testing import parameterized
import mujoco
import mujoco.mjx.warp as mjxw
from mujoco.mjx.warp import mjwp_io
from mujoco.mjx.warp import types
from mujoco.mjx.warp import warp as wp
import numpy as np


class GeneratedTypesMetadataTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    if not mjxw.WARP_INSTALLED or mjwp_io is None:
      self.skipTest('warp is not installed')

  def test_batched_fields_metadata(self):
    m = mujoco.MjModel.from_xml_string('<mujoco/>')
    d = mujoco.MjData(m)
    mw = mjwp_io.put_model(m)
    dw = mjwp_io.put_data(m, d)

    def _check_fields(cls_name, obj, warp_cls, prefix=''):
      for f in dataclasses.fields(obj):
        val = getattr(obj, f.name)
        if dataclasses.is_dataclass(val):
          sub_cls_name = type(val).__name__
          if sub_cls_name in types._BATCH_DIM and hasattr(
              types, f'{sub_cls_name}Warp'
          ):
            # Dedicated top-level sub-class (Option, Statistic)
            _check_fields(
                sub_cls_name, val, getattr(types, f'{sub_cls_name}Warp')
            )
          else:
            # Flattened nested sub-dataclass (Data.contact, Data.efc)
            _check_fields(cls_name, val, warp_cls, prefix=f'{prefix}{f.name}__')
          continue

        field_name = f'{prefix}{f.name}'
        is_batched = getattr(val, '_is_batched', False)

        if is_batched:
          self.assertTrue(
              types._BATCH_DIM[cls_name].get(field_name, False),
              f'Expected {cls_name}.{field_name} with _is_batched=True to be'
              ' True in _BATCH_DIM',
          )
          if cls_name == 'Data':
            self.assertNotIn(
                field_name,
                types.DATA_NON_VMAP,
                f'Expected batched Data field {field_name} to not be in'
                ' DATA_NON_VMAP',
            )
          ann = warp_cls.__annotations__.get(field_name)
          if ann is not None:
            self.assertIn(
                str(ann),
                ('jax.Array', "<class 'jax.Array'>"),
                f'Expected {cls_name}.{field_name} to have jax.Array'
                ' annotation',
            )
        elif isinstance(val, wp.array):
          self.assertFalse(
              types._BATCH_DIM[cls_name].get(field_name, True),
              f'Expected {cls_name}.{field_name} with _is_batched=False to be'
              ' False in _BATCH_DIM',
          )

    _check_fields('Model', mw, types.ModelWarp)
    _check_fields('Data', dw, types.DataWarp)


class TileSetTest(parameterized.TestCase):

  @parameterized.parameters(1, 3)
  def test_structural_equality_and_hash(self, count):
    tile_a = types.TileSet(np.arange(count) * 6, 16)
    tile_b = types.TileSet(np.arange(count) * 6, 16)
    tile_c = types.TileSet(np.arange(count) * 6 + 1, 16)

    self.assertEqual(tile_a, tile_b)
    self.assertEqual(hash(tile_a), hash(tile_b))
    self.assertNotEqual(tile_a, tile_c)


if __name__ == '__main__':
  absltest.main()
