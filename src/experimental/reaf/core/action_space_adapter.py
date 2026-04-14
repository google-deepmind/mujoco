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
"""Adapts environment action into suitable commands format accepted by REAF."""

import abc
from collections.abc import Mapping

from gdm_robotics.interfaces import types as gdmr_types


class ActionSpaceAdapter(abc.ABC):
  """Adapts environment action into suitable commands format accepted by REAF.

  Implementations of this interface are responsible for converting the more
  generic action accepted by the environment (e.g. a flat numpy array) into the
  more constraining format accepted as commands by REAF, i.e. a dictionary of
  string to tensors.
  """

  @abc.abstractmethod
  def commands_from_environment_action(
      self, environment_action: gdmr_types.ActionType
  ) -> Mapping[str, gdmr_types.ArrayType]:
    """Converts the environment action into commands accepted by REAF."""

  @abc.abstractmethod
  def action_spec(self) -> gdmr_types.ActionSpec:
    """Returns the action spec exposed by the environment."""

  @abc.abstractmethod
  def task_commands_keys(self) -> set[str]:
    """Returns the keys for the commands exposed to the task layer."""
