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
"""Tests for the Parameter and ParameterDict classes."""

from mujoco.sysid._src import parameter
import numpy as np


def test_scalar_parameter():
  """A single-valued parameter round-trips through vector conversion, sampling, and reset."""
  param = parameter.Parameter("test", 1.0, 0.5, 2.0)

  assert param.name == "test"
  assert param.size == 1
  assert param.shape == (1,)
  assert param.nominal == 1.0
  assert param.value == 1.0
  assert param.min_value == 0.5
  assert param.max_value == 2.0

  np.testing.assert_array_equal(param.as_vector(), [1.0])
  param.update_from_vector(np.array([1.5]))
  np.testing.assert_array_equal(param.value, [1.5])
  np.testing.assert_array_equal(param.as_vector(), [1.5])

  lower, upper = param.get_bounds()
  np.testing.assert_array_equal(lower, [0.5])
  np.testing.assert_array_equal(upper, [2.0])

  param.reset()
  np.testing.assert_array_equal(param.value, [1.0])

  rng = np.random.default_rng(42)
  sample = param.sample(rng)
  assert 0.5 <= sample[0] <= 2.0


def test_vector_parameter():
  """A multi-valued parameter preserves element-wise bounds and resets correctly."""
  param = parameter.Parameter("test_vector", [1.0, 2.0], [0.5, 1.0], [2.0, 3.0])

  assert param.name == "test_vector"
  assert param.size == 2
  assert param.shape == (2,)
  np.testing.assert_array_equal(param.nominal, [1.0, 2.0])
  np.testing.assert_array_equal(param.value, [1.0, 2.0])
  np.testing.assert_array_equal(param.min_value, [0.5, 1.0])
  np.testing.assert_array_equal(param.max_value, [2.0, 3.0])

  np.testing.assert_array_equal(param.as_vector(), [1.0, 2.0])
  param.update_from_vector(np.array([1.5, 2.5]))
  np.testing.assert_array_equal(param.value, [1.5, 2.5])

  lower, upper = param.get_bounds()
  np.testing.assert_array_equal(lower, [0.5, 1.0])
  np.testing.assert_array_equal(upper, [2.0, 3.0])

  param.reset()
  np.testing.assert_array_equal(param.value, [1.0, 2.0])


def test_parameter_dict():
  """A dict of mixed scalar/vector params flattens to one vector and reconstructs."""
  param1 = parameter.Parameter("param1", 1.0, 0.5, 2.0)
  param2 = parameter.Parameter("param2", [2.0, 3.0], [1.0, 2.0], [3.0, 4.0])
  params = parameter.ParameterDict({"param1": param1, "param2": param2})

  assert params.size == 3  # 1 + 2
  assert len(params) == 2

  assert params["param1"] is param1
  assert params["param2"] is param2

  np.testing.assert_array_equal(params.as_vector(), [1.0, 2.0, 3.0])

  params.update_from_vector(np.array([1.5, 2.5, 3.5]))
  np.testing.assert_array_equal(params["param1"].value, [1.5])
  np.testing.assert_array_equal(params["param2"].value, [2.5, 3.5])

  lower, upper = params.get_bounds()
  np.testing.assert_array_equal(lower, [0.5, 1.0, 2.0])
  np.testing.assert_array_equal(upper, [2.0, 3.0, 4.0])

  params.reset()
  np.testing.assert_array_equal(params["param1"].value, [1.0])
  np.testing.assert_array_equal(params["param2"].value, [2.0, 3.0])

  rng = np.random.default_rng(42)
  sample = params.sample(rng=rng)
  assert len(sample) == 3


def test_save_and_load_round_trip(tmp_path):
  """Saving to YAML and loading back recovers modified values, nominals, and bounds."""
  param1 = parameter.Parameter("p1", 1.0, 0.0, 2.0)
  param2 = parameter.Parameter("p2", [3.0, 4.0], [1.0, 2.0], [5.0, 6.0])
  params = parameter.ParameterDict({"p1": param1, "p2": param2})
  params.update_from_vector(np.array([0.7, 3.5, 4.5]))

  path = tmp_path / "params.yaml"
  params.save_to_disk(path)

  loaded = parameter.ParameterDict.load_from_disk(path)
  np.testing.assert_array_equal(loaded.as_vector(), [0.7, 3.5, 4.5])
  np.testing.assert_array_equal(loaded["p1"].nominal, [1.0])
  np.testing.assert_array_equal(loaded["p2"].min_value, [1.0, 2.0])


def test_randomize_stays_in_bounds():
  """Randomized parameter values always stay within their declared bounds."""
  param1 = parameter.Parameter("a", 5.0, 2.0, 8.0)
  param2 = parameter.Parameter("b", [1.0, 2.0], [0.0, 0.0], [3.0, 3.0])
  params = parameter.ParameterDict({"a": param1, "b": param2})

  rng = np.random.default_rng(0)
  for _ in range(10):
    params.randomize(rng=rng)
    lower, upper = params.get_bounds()
    vec = params.as_vector()
    assert np.all(vec >= lower)
    assert np.all(vec <= upper)


def test_frozen_param_excluded():
  """Freezing a parameter hides it from the optimizer: excluded from vector ops."""
  p1 = parameter.Parameter("free", 1.0, 0.0, 2.0)
  p2 = parameter.Parameter("frozen", 5.0, 3.0, 7.0, frozen=True)
  params = parameter.ParameterDict({"free": p1, "frozen": p2})

  assert params.size == 1
  np.testing.assert_array_equal(params.as_vector(), [1.0])

  params.update_from_vector(np.array([1.5]))
  np.testing.assert_array_equal(params["free"].value, [1.5])
  # Frozen param unchanged.
  np.testing.assert_array_equal(params["frozen"].value, [5.0])
