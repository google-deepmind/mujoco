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
"""Internal handler registration, metadata stamping, and runtime dispatching."""

import collections
from typing import Any, Callable
from mujoco.experimental.studio import messages


def _discover_handlers(
    handler_obj: Any,
) -> list[tuple[messages._HandlerInfo, Callable[..., Any]]]:
  """Scan an object for methods marked with handler decorators."""
  handlers = []
  for name in dir(type(handler_obj)):
    unbound = getattr(type(handler_obj), name, None)
    if unbound is None:
      continue
    info = getattr(unbound, messages._HANDLER_INFO_ATTR, None)  # pylint: disable=protected-access
    if info is None:
      continue
    bound = getattr(handler_obj, name)
    handlers.append((info, bound))
  return handlers


class HandlerRegistry:
  """Runtime registry of discovered handlers from handler instances."""

  def __init__(self, handlers: list[Any] | None = None) -> None:
    self.handlers: dict[
        type[messages.Message], list[tuple[int, Callable[..., Any]]]
    ] = collections.defaultdict(list)
    for obj in handlers or []:
      for info, bound_method in _discover_handlers(obj):
        self.handlers[info.message_type].append((info.priority, bound_method))

  def dispatch(self, message: Any) -> None:
    """Dispatches a message to registered handlers in priority order."""
    # Collect all matching handlers for the given message class and all its
    # ancestors/superclasses.
    message_handlers = []
    for message_type in type(message).__mro__:
      message_handlers.extend(self.handlers.get(message_type, []))

    # Sort by priority; higher values execute first.
    message_handlers.sort(key=lambda item: item[0], reverse=True)
    for _, handler in message_handlers:
      if handler(message):
        # Handler returned True to consume the message; stop dispatching.
        return
