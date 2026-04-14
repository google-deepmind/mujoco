# Copyright 2025 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Adapter that passes the commands spec through."""

from collections.abc import Mapping
from gdm_robotics.interfaces import types as gdmr_types
from reaf.core import action_space_adapter


class PassThroughActionSpaceAdapter(action_space_adapter.ActionSpaceAdapter):
  """Adapter that passes the commands spec through.

  NB the resulting environment will expose a dictionary as the action spec.
  """

  def __init__(self, commands_spec: Mapping[str, gdmr_types.AnyArraySpec]):
    self._commands_spec = commands_spec

  def commands_from_environment_action(
      self, environment_action: gdmr_types.ActionType
  ) -> Mapping[str, gdmr_types.ArrayType]:
    """Returns commands accepted by REAF.

    commands_from_environment_action usually accepts a gdmr_types.ActionType but
    since this adapter passes the same action as the commands, it needs to be a
    dict type in order to pass it through as a dict.

    Args:
      environment_action: The environment action(s) to pass as REAF commands.
    """
    if not isinstance(environment_action, dict):
      raise ValueError(
          'environment_action must be a dict, but got: '
          f'{type(environment_action)}.'
      )
    return environment_action

  def action_spec(self) -> gdmr_types.ActionSpec:
    """Returns the action spec exposed by the environment."""
    return self._commands_spec

  def task_commands_keys(self) -> set[str]:
    """Returns the keys for the commands exposed to the task layer."""
    return set(self._commands_spec.keys())
