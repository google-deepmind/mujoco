# Copyright 2025 The Newton Developers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import dataclasses
import functools
import inspect
import warnings

import warp as wp

_STACK = None


def is_array_spec(typ) -> bool:
  """Check if a type annotation is an array spec (wp.array instance or bracket annotation)."""
  return isinstance(typ, wp.array) or type(typ).__name__ == "_ArrayAnnotation"


def mark_batched(obj):
  """Recursively set _is_batched = True on all batched warp arrays within obj."""
  if not dataclasses.is_dataclass(obj):
    return
  for f in dataclasses.fields(obj):
    val = getattr(obj, f.name, None)
    if val is None:
      continue
    if dataclasses.is_dataclass(val):
      mark_batched(val)
      continue
    if not isinstance(val, wp.array):
      continue
    if not is_array_spec(f.type):
      continue
    spec_shape = getattr(f.type, "shape", ())
    if spec_shape and spec_shape[0] in ("*", "nworld"):
      val._is_batched = True


class EventTracer:
  """Calculates elapsed times of functions annotated with `event_scope`.

  Use as a context manager like so:

    @event_trace
    def my_warp_function(...):
      ...

    with EventTracer() as tracer:
      my_warp_function(...)
      print(tracer.trace())
  """

  def __init__(self, enabled: bool = True):
    global _STACK
    if _STACK is not None:
      raise ValueError("only one EventTracer can run at a time")
    if enabled:
      _STACK = {}

  def __enter__(self):
    return self

  def trace(self) -> dict:
    """Calculates elapsed times for every node of the trace."""
    global _STACK

    if _STACK is None:
      return {}

    ret = {}

    for k, v in _STACK.items():
      events, sub_stack = v
      # push into next level of stack
      saved_stack, _STACK = _STACK, sub_stack
      sub_trace = self.trace()
      # pop!
      _STACK = saved_stack
      events = tuple(wp.get_event_elapsed_time(beg, end) for beg, end in events)
      ret[k] = (events, sub_trace)

    return ret

  def __exit__(self, type, value, traceback):
    global _STACK
    _STACK = None


def _merge(a: dict, b: dict) -> dict:
  """Merges two event trace stacks."""
  ret = {}
  if not a or not b:
    return dict(**a, **b)
  if set(a) != set(b):
    raise ValueError("incompatible stacks")
  for key in a:
    a1_events, a1_substack = a[key]
    a2_events, a2_substack = b[key]
    ret[key] = (a1_events + a2_events, _merge(a1_substack, a2_substack))
  return ret


def event_scope(fn, name: str = ""):
  """Wraps a function and records an event before and after the function invocation."""
  name = name or getattr(fn, "__name__")

  @functools.wraps(fn)
  def wrapper(*args, **kwargs):
    global _STACK
    if _STACK is None:
      return fn(*args, **kwargs)

    for frame_info in inspect.stack():
      if frame_info.function in ("capture_while", "capture_if"):
        return fn(*args, **kwargs)

    # push into next level of stack
    saved_stack, _STACK = _STACK, {}
    beg = wp.Event(enable_timing=True)
    end = wp.Event(enable_timing=True)
    wp.record_event(beg)
    res = fn(*args, **kwargs)
    wp.record_event(end)
    # pop back up to current level
    sub_stack, _STACK = _STACK, saved_stack
    # append events and substack
    prev_events, prev_substack = _STACK.get(name, ((), {}))
    events = prev_events + ((beg, end),)
    sub_stack = _merge(prev_substack, sub_stack)
    _STACK[name] = (events, sub_stack)
    return res

  return wrapper


_KERNEL_CACHE = {}


def cache_kernel(func):
  # caching kernels to avoid crashes in graph_conditional code
  @functools.wraps(func)
  def wrapper(*args):
    def _hash_arg(a):
      if hasattr(a, "size"):
        return a.size
      if isinstance(a, list):
        return hash(tuple(a))
      return hash(a)

    key = tuple(_hash_arg(a) for a in args) + (func.__module__, func.__name__)
    if key not in _KERNEL_CACHE:
      _KERNEL_CACHE[key] = func(*args)
    return _KERNEL_CACHE[key]

  return wrapper


_SIDE_STREAMS = {}


def side_stream(device: wp.Device) -> wp.Stream | None:
  """Returns a second stream while the device's stream is capturing a graph, else None.

  Launches under wp.ScopedStream(side) fork the captured node chain until
  wp.wait_stream(side) joins it, so they run beside the launches issued in between.
  """
  if not device.is_cuda or not device.stream.is_capturing:
    return None
  # one per capturing stream, so that concurrent captures do not share a branch
  key = (device.alias, device.stream.cuda_stream)
  if key not in _SIDE_STREAMS:
    _SIDE_STREAMS[key] = wp.Stream(device)
  return _SIDE_STREAMS[key]


def efc_threads_per_world(nworld: int, njmax: int, device, kernel: wp.Kernel, num_waves: int = 6) -> int:
  """Size row launches from kernel occupancy, capped at capacity and aligned to warps."""
  if not device.is_cuda or njmax <= 32:
    return njmax
  block_size, min_grid_size = wp.get_suggested_block_size(kernel, device)
  # Iterative updates need extra waves as converged worlds stop contributing work.
  warps = max(1, num_waves * block_size * min_grid_size // (32 * max(1, nworld)))
  return 32 * min(njmax // 32, warps)


def check_toolkit_driver():
  wp.init()
  if wp.get_device().is_cuda:
    if not wp.is_conditional_graph_supported():
      warnings.warn(
        """
        CUDA version < 12.4 detected
        - graph capture may be unreliable for < 12.3
        - conditional graph nodes are not available for < 12.4
          Model.opt.graph_conditional should be set to False
        """,
        stacklevel=2,
      )
