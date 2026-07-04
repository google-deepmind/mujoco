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
import enum
import inspect
from typing import Any, Callable, Protocol, get_type_hints, runtime_checkable

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


# ---------------------------------------------------------------------------
# Message handling decorator.
# ---------------------------------------------------------------------------

_HANDLER_INFO_ATTR = '_studio_handler_info'


@dataclasses.dataclass(frozen=True)
class _HandlerInfo:
  """Metadata stamped on a decorated method."""

  priority: int
  message_type: type[Message]


class Priority(enum.IntEnum):
  """Execution priority levels for message handlers.

  Handlers with higher values execute first. When multiple handlers share the
  same priority, their relative order is undefined.
  """

  INTERNAL = 1  # For built-in handlers.
  LIBRARY = 10  # For library extensions.
  USER = 100  # For user extensions. Default when no priority is specified.
  CRITICAL = 1000  # For handlers that must run before everything else.


def handler(
    fn: Callable[..., Any] | None = None, *, priority: int = Priority.USER
) -> Callable[..., Any]:
  """Decorator to mark a class method as a message handler.

  The class method must accept exactly two arguments: self and a message event.
  e.g., ``@handler`` or ``@handler(priority=...)``.

  Args:
    fn: The method to stamp with handler metadata.
    priority: The priority of the handler.

  Returns:
    The stamped method.
  """

  def stamp(target: Callable[..., Any]) -> Callable[..., Any]:
    fn_name = getattr(target, '__name__', str(target))
    params = list(inspect.signature(target).parameters.values())
    if len(params) != 2:
      raise TypeError(
          f'{fn_name} must accept exactly two arguments, got {len(params)}'
      )

    message_type = params[1].annotation

    # When `from __future__ import annotations` is enabled or string forward
    # references are used, annotations are stored as strings rather than code
    # objects. In this case, resolve the string into its actual class type.
    if isinstance(message_type, str):
      try:
        hints = get_type_hints(target)
        param_name = params[1].name
        if param_name in hints:
          message_type = hints[param_name]
      except Exception:  # pylint: disable=broad-except
        pass

    # Verify we got a type annotation.
    if message_type is inspect.Parameter.empty:
      raise TypeError(
          f'{fn_name}: second parameter must have a type annotation'
      )

    # Verify that the annotation resolved to an actual Python class and that the
    # class is a valid subclass of Message. This also catches cases where a
    # string forward reference failed to resolve via get_type_hints above.
    if not (
        isinstance(message_type, type) and issubclass(message_type, Message)
    ):
      raise TypeError(
          f'{fn_name}: second parameter type annotation {message_type} is not'
          ' a Message subclass'
      )

    info = _HandlerInfo(priority=priority, message_type=message_type)
    setattr(target, _HANDLER_INFO_ATTR, info)
    return target

  # Used without parens: e.g., @handler
  if fn is not None:
    return stamp(fn)

  # Used with parens: e.g., @handler() or @handler(priority=...)
  return stamp
