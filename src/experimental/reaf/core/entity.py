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
"""Basic REAF-sim protocol to interface with the simulation."""

from collections.abc import Mapping
import typing

from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types


class Entity(typing.Protocol):
  """Basic REAF component to interface with the simulation.

  An entity defines a single component in the simulation that consumes substep
  commands and outputs substep measurements at every simulation substep. It
  should be hermetic, that is, not depending on other Entities.

  Important: an Entity should return the substep commands and substep
  measurements specs immediately after initialisation without the need for any
  explicit initialisation.
  """

  @property
  def name(self) -> str:
    """Instance name."""

  def reset(self):
    """Resets the entity."""

  def substep_commands_spec(
      self,
  ) -> Mapping[str, specs.Array]:
    """Spec for the substep commands."""

  def substep_measurements_spec(
      self,
  ) -> Mapping[str, specs.Array]:
    """Spec for the substep measurements."""

  def set_substep_commands(
      self,
      model: typing.Any,
      data: typing.Any,
      consumed_substep_commands: Mapping[str, gdmr_types.ArrayType],
  ) -> None:
    """Sets the substep commands."""

  def get_substep_measurements(
      self,
      model: typing.Any,
      data: typing.Any,
  ) -> Mapping[str, gdmr_types.ArrayType]:
    """Returns the substep measurements."""
