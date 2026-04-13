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
"""Protocol for substep measurements manipulation in REAF-sim."""

from collections.abc import Mapping
import typing

from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types


class SubstepMeasurementsProcessor(typing.Protocol):
  """Processes substep measurements, propagating them through a pipeline.

  This processor manipulates substep measurements, acting as a node in a
  pipeline. It consumes substep measurements, performs operations, and produces
  updated substep measurements for the next stage in the processing chain.

  The processing pipeline starts with substep measurements produced by Entities
  and progresses towards the measurements exposed by the SimulationDevice. Each
  processor consumes a subset of substep measurements and produces new,
  potentially transformed, substep measurements. The order of operations is
  crucial.

  Example Pipeline (conceptual):

  Entities --> Processor (1) --> Processor (2) -> Simulation Device Measurements

  Specs are propagated starting from the bottom:
  1) In this example assume that the set of entities produce "p1/c1", "p1/c2"
     and "p1/c3".
  2) Processor (1) returns "p2/c1" from ("p1/c1", "p1/c2").
  3) Processor (2) returns ("p3/c1", "p3/c2") from "p2/c1".

  This resulting spec exposed by the SimulationDevice: "p3/c1", "p3/c2"
  and "p1/c3".

     ------------------------------------
    |        SimulationDevice           |
     ------------------------------------

    "p3/c1"     "p3/c2"           "p1/c3"
       |          |                  |
     -----------------               |
    |      P2         |              |
     -----------------               |
            |  "p2/c1"               |
     -----------------               |
    |      P1         |              |
     -----------------               |
       | "p1/c1"   | "p1/c2"         |
       |           |                 |
     ------------------------------------
    |            Entities               |
     ------------------------------------
  """

  @property
  def name(self) -> str:
    """Returns a unique string identifier for this object."""

  def reset(self):
    """Resets the internal state of this processor."""

  def produced_substep_measurements_spec(
      self,
  ) -> Mapping[str, specs.Array]:
    """Spec of the substep measurements consumed by this processor."""

  def consumed_substep_measurements_keys(self) -> set[str]:
    """Keys of the substep measurements consumed by this processor."""

  def process_substep_measurements(
      self,
      model: typing.Any,
      data: typing.Any,
      consumed_substep_measurements: Mapping[str, gdmr_types.ArrayType],
  ) -> Mapping[str, gdmr_types.ArrayType]:
    """Processes the substep measurements and returns a new modified version of it.

    Args:
      model: the simulation model.
      data: the simulation data.
      consumed_substep_measurements: the substep measurements up in the
        processing chain that are required by this processor, i.e. with keys
        specified by `consumed_substep_measurements_spec`.

    Returns the new substep measurements. Note that the (key, value) pairs in
    `consumed_substep_measurements` are removed from the running substep
    measurements dictionary. If users want to keep some of the elements it is
    their responsibility to retain them in the output dictionary.
    """
