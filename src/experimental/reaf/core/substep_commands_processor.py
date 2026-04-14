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
"""Protocol for substep commands manipulation in REAF-sim."""

from collections.abc import Mapping
import typing

from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types


class SubstepCommandsProcessor(typing.Protocol):
  """Processes substep commands, propagating them through a pipeline.

  This processor manipulates substep commands, acting as a node in a pipeline.
  It consumes substep commands, performs operations, and produces updated
  substep commands for the next stage in the processing chain.

  The processing pipeline starts with commands provided to the SimulationDevice
  and progresses towards the substep commands consumed by the individual
  entities. Each processor consumes a subset of substep commands and produces
  new, potentially transformed, substep commands. The order of operations is
  crucial.

  Example Pipeline (conceptual):

  Simulation Device commands --> Processor (1) --> Processor (2) --> Entities

  Specs are propagated starting from the bottom:
  1) In this example assume that the set of entities expect "p3/c1", "p3/c2" and
    "p3/c3".
  2) Processor (2) returns ("p3/c1", "p3/c2") from "p2/c1". This means
    that the global substep commands spec exposed at this level is "p2/c1" and
    the unprocessed "p3/c3".
  3) Processor (1) returns "p2/c1" from ("p1/c1", "p1/c2"). By applying
    the same transformation rule, we can obtain the final spec exposed
    by the SimulationDevice: "p1/c1", "p1/c2" and "p3/c3".

     ------------------------------------
    |        SimulationDevice           |
     ------------------------------------

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
    |            Entities               |
     ------------------------------------
  """

  @property
  def name(self) -> str:
    """Returns a unique string identifier for this object."""

  def reset(self) -> None:
    """Resets the internal state of this processor."""

  def produced_substep_commands_keys(self) -> set[str]:
    """Keys of the substep commands produced by this processor."""

  def consumed_substep_commands_spec(
      self,
  ) -> Mapping[str, specs.Array]:
    """Spec of the substep commands consumed by this processor."""

  def process_substep_commands(
      self,
      model: typing.Any,
      data: typing.Any,
      consumed_substep_commands: Mapping[str, gdmr_types.ArrayType],
  ) -> Mapping[str, gdmr_types.ArrayType]:
    """Processes the substep commands and returns a new modified version of it.

    Args:
      model: the simulation model.
      data: the simulation data.
      consumed_substep_commands: the substep commands up in the processing chain
        that are required by this processor, i.e. with keys specified by
        `consumed_substep_commands_spec`.

    Returns the new substep commands. Note that the (key, value) pairs in
    `consumed_substep_commands` are removed from the running substep commands
    dictionary. If users want to keep some of the elements it is their
    responsibility to retain them in the output dictionary.
    """
