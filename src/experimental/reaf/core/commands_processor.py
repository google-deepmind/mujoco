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
"""Abstract class for commands manipulation in the task logic layer."""

import abc
from collections.abc import Mapping

from gdm_robotics.interfaces import types as gdmr_types


class CommandsProcessor(abc.ABC):
  """Perform commands manipulation.

  The following describes the processing pipeline starting from the top (closer
  to the policy) to the bottom (interfacing with the DACL commands spec).

  Assume that we have two processing units:
  Processor 1) has a consumed_commands_spec for two keys: "p1/c1" and "p1/c2".
    Its produced_commands_keys are "p2/c1".
  Processor 2) has a consumed_commands_spec for "p2/c1". Its
    produced_commands_keys are "p3/c1" and "p3/c2".

  Specs are propagated starting from the bottom:
  1) In this example assume that the DACL exposes "p3/c1", "p3/c2" and "p3/c3".
  2) Processor 2) returns ("p3/c1", "p3/c2") from input "p2/c1". This means that
    the global commands spec exposed at this level is "p2/c1" and the
    unprocessed "p3/c3".
  3) Processor 1) returns "p2/c1" from input ("p1/c1", "p1/c2"). By applying the
    same transformation rule, we can obtain the final commands spec exposed by
    the full processing pipeline: "p1/c1", "p1/c2" and "p3/c3".

    "p1/c1"     "p1/c2"           "p3/c3"
       |          |                  |
     -----------------               |
    |      P1         |              |
     -----------------               |
            |  "p2/c1"               |
     -----------------               |
    |      P2         |              |
     -----------------               |
       | "p3/c1"   | "p3/c2"         |
       |           |                 |
     ------------------------------------
    |               DACL                 |
     ------------------------------------
  """

  @property
  @abc.abstractmethod
  def name(self) -> str:
    """Returns a unique string identifier for this object."""

  @abc.abstractmethod
  def process_commands(
      self, consumed_commands: Mapping[str, gdmr_types.ArrayType]
  ) -> Mapping[str, gdmr_types.ArrayType]:
    """Processes the commands and returns a new modified version of it.

    Args:
      consumed_commands: the commands up in the processing chain (or provided by
        the Environment) that are required by this processor, i.e. with keys
        specified by `consumed_commands_spec`.

    Returns the new commands. Note that the data in consumed_commands is removed
      from the global commands dictionary. If users want to keep some of the
      elements it is their responsibility to retain them in the output
      dictionary.
    """

  @abc.abstractmethod
  def consumed_commands_spec(self) -> Mapping[str, gdmr_types.AnyArraySpec]:
    """Spec of the commands consumed by this processor."""

  @abc.abstractmethod
  def produced_commands_keys(self) -> set[str]:
    """Keys of the commands produced by this processor."""

  def reset(self) -> None:
    """Resets the internal state of the command processor."""
    ...
