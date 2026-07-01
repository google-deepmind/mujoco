# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Messages and Channels for Studio."""

import dataclasses
from typing import Protocol
from typing import runtime_checkable
import mujoco
from mujoco.experimental.studio import sim
import numpy as np

# -----------------------------------------------------------------------------
# Message and Channel types for passing data between the sim and the viewer.
# Concrete messages are defined later in this file, there is one per Event;
# add more if needed in your script.
# Concrete Channel implementations live in the launch_* modules.
# -----------------------------------------------------------------------------


class Message:
  """Base class for data sent between the sim and the viewer.

  Derive from Snapshot or Event, depending on the desired delivery semantics.
  """


class Snapshot(Message):
  """Base class for latest-wins, droppable messages.

  Each new snapshot replaces the previous one, so the receiver always sees the
  most recent value. Intermediate values may be silently dropped, making
  snapshots suitable for frequently updated state where only the latest value
  matters.
  """


class Event(Message):
  """Base class for reliable, ordered, never dropped messages.

  Events are queued in order and every event is delivered exactly once. Use
  events for discrete actions that must not be lost.
  """


@runtime_checkable
class SnapshotChannel(Protocol):
  """Channel to transport Snapshot messages with latest-wins semantics."""

  def put(self, value: Snapshot) -> None:
    """Overwrites the latest snapshot of the same type.

    After calling put() it is the caller's responsibility to ensure it no longer
    holds a reference to the snapshot.

    Args:
      value: The snapshot to put into the channel.
    """
    ...

  def get(self) -> list[Snapshot]:
    """Returns the latest pending snapshots of each type."""
    ...

  def close(self) -> None:
    """Releases resources held by the channel."""
    ...


@runtime_checkable
class EventChannel(Protocol):
  """Channel to transport Event messages in an ordered stream, nothing dropped."""

  def put(self, value: Event) -> None:
    """Puts an event into the channel, appending to the stream."""
    ...

  def get(self) -> list[Event]:
    """Returns all pending events, in order."""
    ...

  def close(self) -> None:
    """Releases resources held by the channel."""
    ...


# -----------------------------------------------------------------------------
# Concrete Event and Snapshot types.
# -----------------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class StateSnapshot(Snapshot):
  """A snapshot message that transports a MuJoCo state."""

  state: np.ndarray
  state_sig: int


@dataclasses.dataclass(frozen=True)
class ResetEvent(Event):
  """An event requesting to reset the simulation."""


@dataclasses.dataclass(frozen=True)
class ModelEvent(Event):
  """An event that transports a MuJoCo model."""

  model: mujoco.MjModel


@dataclasses.dataclass(frozen=True)
class MjOptionSnapshot(Snapshot):
  """A snapshot sending mjOption state from viewer to sim each frame."""

  opt: mujoco.MjOption


@dataclasses.dataclass(frozen=True)
class PerturbEvent(Event):
  """Carries perturbation forces from the viewer to the simulation."""

  state: np.ndarray
  state_sig: int


@dataclasses.dataclass(frozen=True)
class StepControlSnapshot(Snapshot):
  """A snapshot sending step control state from viewer to sim each frame."""

  pause_state: sim.PauseState
  speed: float
  noise_scale: float
  noise_rate: float


@dataclasses.dataclass(frozen=True)
class ExitEvent(Event):
  """An event requesting to exit."""
