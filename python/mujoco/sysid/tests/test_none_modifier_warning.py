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
"""Regression tests for unapplied sysid parameters."""

import logging

import mujoco
from mujoco.sysid._src import model_modifier
from mujoco.sysid._src import parameter


def test_nonfrozen_none_modifier_warns_once(caplog):
  params = parameter.ParameterDict()
  params.add(parameter.Parameter('unapplied', 1.0, 0.0, 2.0))
  spec = mujoco.MjSpec()

  with caplog.at_level(logging.WARNING, logger='absl'):
    model_modifier.apply_param_modifiers_spec(params, spec.copy())

  assert any(
      'unapplied' in record.message and 'modifier=None' in record.message
      for record in caplog.records
  )

  caplog.clear()
  with caplog.at_level(logging.WARNING, logger='absl'):
    model_modifier.apply_param_modifiers_spec(params, spec.copy())
  assert not any('modifier=None' in record.message for record in caplog.records)


def test_frozen_none_modifier_does_not_warn(caplog):
  params = parameter.ParameterDict()
  params.add(parameter.Parameter('frozen', 1.0, 0.0, 2.0, frozen=True))

  with caplog.at_level(logging.WARNING, logger='absl'):
    model_modifier.apply_param_modifiers_spec(params, mujoco.MjSpec())

  assert not any('modifier=None' in record.message for record in caplog.records)
