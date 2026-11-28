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
"""Tests for restoring usable parameters without serialized callbacks."""

import pickle

import numpy as np
import pytest

from mujoco.sysid._src import model_modifier
from mujoco.sysid._src import parameter


@pytest.mark.parametrize("serialization", ["yaml", "pickle"])
def test_restored_parameters_can_apply_modifiers(
    tmp_path, box_spec, serialization
):
  mass = parameter.Parameter(
      "mass",
      5.0,
      1.0,
      10.0,
      modifier=lambda spec, p: setattr(spec.body("box"), "mass", p.value[0]),
  )
  params = parameter.ParameterDict({"mass": mass})
  params.update_from_vector(np.array([7.0]))

  if serialization == "yaml":
    path = tmp_path / "params.yaml"
    params.save_to_disk(path)
    restored = parameter.ParameterDict.load_from_disk(path)
  else:
    restored = pickle.loads(pickle.dumps(params))

  model = model_modifier.apply_param_modifiers(restored, box_spec)
  assert model.body("box").mass[0] == pytest.approx(5.0)
  assert restored["mass"].modifier is None
  np.testing.assert_array_equal(restored["mass"].value, [7.0])

  restored["mass"].modifier = mass.modifier
  model = model_modifier.apply_param_modifiers(restored, box_spec)
  assert model.body("box").mass[0] == pytest.approx(7.0)
