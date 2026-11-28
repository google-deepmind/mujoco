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
"""Timestamp ordering is independent of the numeric timestamp dtype."""

import numpy as np
import pytest

from mujoco.sysid._src.timeseries import TimeSeries


@pytest.mark.parametrize("dtype", [np.uint32, np.uint64, np.int64, np.float64])
@pytest.mark.parametrize("resample", [False, True])
@pytest.mark.parametrize(
    "values, valid",
    [([0, 1, 2], True), ([0, 0, 2], False), ([2, 1, 3], False)],
)
def test_timestamp_order(dtype, resample, values, valid):
  times = np.array(values, dtype=dtype)

  def make_series():
    if resample:
      source_times = np.arange(4, dtype=float)
      source = TimeSeries(source_times, source_times[:, None])
      return source.resample(new_times=times)
    return TimeSeries(times, times.astype(float)[:, None])

  if valid:
    result = make_series()
    np.testing.assert_array_equal(result.times, times)
    np.testing.assert_allclose(result.data[:, 0], times)
  else:
    with pytest.raises(ValueError, match="times must be strictly increasing"):
      make_series()
