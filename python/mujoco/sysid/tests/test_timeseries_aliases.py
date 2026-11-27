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

"""Shared-column signal mappings must not introduce unobserved data columns."""

from mujoco.sysid._src.residual import construct_ts_from_defaults
from mujoco.sysid._src.timeseries import SignalType
from mujoco.sysid._src.timeseries import TimeSeries
import numpy as np
import pytest


@pytest.mark.parametrize("batched", [False, True])
@pytest.mark.parametrize("enabled", [["position"], ["x"], ["position", "x"]])
def test_slice_shared_columns_once(batched, enabled):
  data = (
      np.arange(18).reshape(3, 2, 3) if batched else np.arange(9).reshape(3, 3)
  )
  mapping = {
      "position": (SignalType.MjSensor, np.array([1, 0])),
      "x": (SignalType.MjSensor, np.array([0])),
      "unused": (SignalType.CustomObs, np.array([2])),
  }
  original = TimeSeries(np.arange(3.0), data, mapping)
  columns = [1, 0] if "position" in enabled else [0]

  sliced = TimeSeries.slice_by_name(original, enabled)

  np.testing.assert_array_equal(sliced.data, data[..., columns])
  assert list(sliced.signal_mapping) == enabled
  if "position" in enabled:
    np.testing.assert_array_equal(sliced.get_indices("position")[1], [0, 1])
  if "x" in enabled:
    np.testing.assert_array_equal(
        sliced.get_indices("x")[1], [columns.index(0)]
    )
  np.testing.assert_array_equal(
      TimeSeries.slice_by_name(sliced, enabled).data, sliced.data
  )
  np.testing.assert_array_equal(original.data, data)
  np.testing.assert_array_equal(original.get_indices("position")[1], [1, 0])
  np.testing.assert_array_equal(original.get_indices("x")[1], [0])


@pytest.mark.parametrize("enabled", [["position"], ["x"], ["position", "x"]])
def test_shared_measurements_do_not_create_spurious_residuals(enabled):
  times = np.arange(3.0)
  data = np.array([[2.0, 3.0], [4.0, 5.0], [6.0, 7.0]])
  measured = TimeSeries.create(
      times,
      data,
      {
          "position": (SignalType.MjSensor, [0, 1]),
          "x": (SignalType.MjSensor, [0]),
      },
  )
  sensors = TimeSeries.create(
      times,
      np.column_stack([data, data[:, 0]]),
      {
          "position": (SignalType.MjSensor, [0, 1]),
          "x": (SignalType.MjSensor, [2]),
      },
  )
  state = TimeSeries(times, np.empty((3, 0)), {})

  selected, predicted = construct_ts_from_defaults(
      state,
      sensors,
      measured,
      [(name, SignalType.MjSensor) for name in enabled],
  )

  np.testing.assert_array_equal(predicted.data, selected.data)
  assert np.sum((predicted.data - selected.data) ** 2) == 0


def test_nonoverlapping_slice_preserves_mapping_order():
  data = np.arange(9).reshape(3, 3)
  original = TimeSeries.create(
      np.arange(3.0),
      data,
      {
          "tail": (SignalType.MjSensor, [2]),
          "pair": (SignalType.MjSensor, [0, 1]),
      },
  )

  sliced = TimeSeries.slice_by_name(original, ["pair", "tail"])

  np.testing.assert_array_equal(sliced.data, data[:, [2, 0, 1]])
  np.testing.assert_array_equal(sliced.get_indices("tail")[1], [0])
  np.testing.assert_array_equal(sliced.get_indices("pair")[1], [1, 2])
