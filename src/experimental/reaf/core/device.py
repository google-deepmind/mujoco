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
"""REAF basic device to interface with the robotic setup."""

import abc
from collections.abc import Mapping
from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types


class Device(abc.ABC):
  """REAF basic device to interface with the robotic setup.

  A device defines a single piece in the robotic setup. It should be
  hermetic, that is, not depending on other Devices. The coordination of the
  devices is responsibility of the DeviceCoordinator.

  Important: a Device should return the commands and measurements specs
  immediately after initialisation without the need for any explicit
  initialisation, nor for resource acquisition (e.g. connecting to the
  hardware).
  """

  @property
  @abc.abstractmethod
  def name(self) -> str:
    """Returns the name of this device."""

  @abc.abstractmethod
  def commands_spec(self) -> Mapping[str, gdmr_types.AnyArraySpec]:
    """Returns the commands specs for this device."""

  @abc.abstractmethod
  def measurements_spec(self) -> Mapping[str, specs.Array]:
    """Returns the measurements specs for this device."""

  @abc.abstractmethod
  def set_commands(self, commands: Mapping[str, gdmr_types.ArrayType]) -> None:
    """Sets the commands for this device."""

  @abc.abstractmethod
  def get_measurements(self) -> Mapping[str, gdmr_types.ArrayType]:
    """Returns the measurements provided by this device."""
