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
"""Randomized parameters retain the shapes used by model modifiers."""

import numpy as np
import pytest

from mujoco.sysid._src import model_modifier
from mujoco.sysid._src import parameter


@pytest.mark.parametrize("shape", [(1,), (3,), (2, 3), (2, 1, 2)])
def test_randomize_preserves_shapes_and_frozen_values(shape):
  nominal = np.zeros(shape)
  lower = np.full(shape, -1.0)
  upper = np.full(shape, 1.0)
  free = parameter.Parameter("free", nominal, lower, upper)
  frozen = parameter.Parameter("frozen", nominal, lower, upper, frozen=True)
  frozen.value = np.full(shape, 0.25)
  params = parameter.ParameterDict({"free": free, "frozen": frozen})

  params.randomize(np.random.default_rng(42))

  assert free.value.shape == free.shape
  np.testing.assert_allclose(
      free.value, np.random.default_rng(42).uniform(lower, upper)
  )
  np.testing.assert_array_equal(free.nominal, nominal)
  np.testing.assert_array_equal(frozen.value, np.full(shape, 0.25))
  assert "free" in str(params)


def test_randomized_matrix_reaches_native_model_modifier(box_spec):
  nominal = np.array([[0.1, 0.1, 0.1], [0.0, 0.0, 0.0]])
  lower = np.array([[0.08, 0.08, 0.08], [-0.01, -0.01, -0.01]])
  upper = np.array([[0.12, 0.12, 0.12], [0.01, 0.01, 0.01]])

  def modify_body(spec, param):
    body = spec.body("box")
    body.inertia[:] = param.value[0]
    body.ipos[:] = param.value[1]

  param = parameter.Parameter(
      "inertial", nominal, lower, upper, modifier=modify_body
  )
  params = parameter.ParameterDict({"inertial": param})
  params.randomize(np.random.default_rng(42))
  model = model_modifier.apply_param_modifiers(params, box_spec)
  expected = np.random.default_rng(42).uniform(lower, upper)

  np.testing.assert_allclose(model.body("box").inertia, expected[0])
  np.testing.assert_allclose(model.body("box").ipos, expected[1])
