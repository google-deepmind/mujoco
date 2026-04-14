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
"""Support logging inside the task logic layer."""

import abc
from collections.abc import Mapping

from gdm_robotics.interfaces import types as gdmr_types


class Logger(abc.ABC):
  """Support logging inside the task logic layer.

  Lifecycle
  For each environment step, these member functions are called in this order:
    1. `record_measurements` is called with raw measurements from the sensors.
    2. `record_features` is called with features derived from the measurements.
    3. `record_commands_processing` is called for each
       `CommandsProcessor.process_commands` invocation, tracking the
       transformation of commands.
    4. `record_final_commands` is called once with the final commands sent to
       the DACL.

    Notes:
    An environment is first reset().  This triggers the first two steps above.
      See reset_with_options in ./environment.py.

    After reset, step is called repeatedly.
    1. This first triggers steps 3 and 4 (See compute_final_commands in TLL
       called from step in ./environment.py)
    2. Features are computed (see compute_all_features in TLL called from
       step in ./environment.py), triggering steps 1 and 2.
  """

  @property
  @abc.abstractmethod
  def name(self) -> str:
    """Unique string identifier for this object."""

  def record_measurements(
      self, measurements: Mapping[str, gdmr_types.ArrayType]
  ) -> None:
    """Called once with all the measurements from the DACL."""

  def record_features(
      self, features: Mapping[str, gdmr_types.ArrayType]
  ) -> None:
    """Called once with all the features computed in the Task Layer."""

  def record_final_commands(
      self, commands: Mapping[str, gdmr_types.ArrayType]
  ) -> None:
    """Called once with the final commands sent to the DACL."""

  def record_commands_processing(
      self,
      name: str,
      consumed_commands: Mapping[str, gdmr_types.ArrayType],
      produced_commands: Mapping[str, gdmr_types.ArrayType],
  ) -> None:
    """Called once per call to `process_commands` for each CommandsProcessor.

    Args:
      name: Name of the `CommandsProcessor`.
      consumed_commands: The commands consumed by the current
        `CommandsProcessor`.
      produced_commands: The commands produced by the current
        `CommandsProcessor`.
    """
