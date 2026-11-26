# Copyright 2026 The Newton Developers
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
"""Differentiability API for MuJoCo Warp.

MuJoCo Warp kernels are compiled with `enable_backward=False`, so recording API functions on a
`wp.Tape` would produce wrong gradients (Warp skips their adjoints). `differentiable` wraps every
public function: outside a tape the wrapper calls the function directly; inside a tape, functions
without a registered `AdjointRule` raise `NotImplementedError`, and functions with a rule are
validated and recorded with either a custom backward or their native kernel adjoints.
"""

import contextlib
import dataclasses
import functools
import inspect
from typing import Callable, Sequence

import warp as wp
from warp._src import context as wp_context
from warp._src import types as wp_types

from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src import warp_util


class _ScratchTape(wp.Tape):
  """Tape a wrapped forward runs on; wrapped functions it calls pass through as if untaped."""


def _call(tape: wp.Tape | None, fn: Callable, *args, verify_access: bool = True, **kwargs):
  """Calls `fn` with `tape` as the active Warp tape (`None` pauses recording)."""
  with contextlib.ExitStack() as cleanup:
    cleanup.callback(vars(wp_context.runtime).__setitem__, "tape", wp_context.runtime.tape)
    cleanup.callback(
      vars(wp.config).__setitem__,
      "verify_autograd_array_access",
      wp.config.verify_autograd_array_access,
    )
    wp_context.runtime.tape = tape
    if not verify_access:
      wp.config.verify_autograd_array_access = False
    return fn(*args, **kwargs)


def _is_float(arr: wp.array) -> bool:
  """Returns whether `arr` has a float scalar type, including float vectors and matrices."""
  return wp_types.type_is_float(wp_types.type_scalar_type(arr.dtype))


def _accessed(launches: Sequence[list | Callable]) -> set[int]:
  """Returns `id(arr)` for arrays that recorded launches read or write."""
  arrays = []
  for l in launches:
    if isinstance(l, list) and all(
      (l[1],) if isinstance(l[1], int) else l[1]
    ):  # kernel launch: [kernel, dim, max_blocks, inputs, outputs, ...]
      arrays += [*l[3], *l[4]]
    elif inspect.isfunction(l):  # wp.copy(dest, src) records a closure over `dest` and `src`
      cells = inspect.getclosurevars(l).nonlocals
      arrays += [cells.get("src"), cells.get("dest")]
  return {id(a) for a in arrays if isinstance(a, wp.array)}


def _has_adjoint(launch: list | Callable) -> bool:
  """Returns whether a recorded launch is a callable or a kernel compiled with an adjoint."""
  # Kernel options take precedence over module options, as in `wp.Tape.backward`:
  return not isinstance(launch, list) or launch[0].options.get("enable_backward", launch[0].module.options["enable_backward"])


def _fields(cls: type, prefix: str = "") -> tuple[str, ...]:
  """Returns the dotted paths of the array fields of dataclass `cls`, including nested ones."""
  out: list[str] = []
  for f in dataclasses.fields(cls):
    path = f"{prefix}.{f.name}" if prefix else f.name
    if dataclasses.is_dataclass(f.type):
      out.extend(_fields(f.type, path))
    elif warp_util.is_array_spec(f.type):
      out.append(path)
  return tuple(out)


_FIELDS: dict[type, tuple[str, ...]] = {cls: _fields(cls) for cls in (types.Model, types.Data, types.RenderContext)}


def get(obj: object, path: str) -> object:
  """Resolves a dotted attribute path on a dataclass container."""
  for part in path.split("."):
    obj = vars(obj)[part]
  return obj


@dataclasses.dataclass(frozen=True)
class AdjointRule:
  """Reverse-mode rule for an API function.

  The rule lists the arrays the function differentiates. It must also list every field with
  gradients that the forward reads or overwrites: kernel launches and copies between arrays with
  gradients are detected and raise when the field is not listed, while `zero_()`, `fill_()`,
  `assign()`, and copies involving an array without gradients are not detected. Listed Data and
  RenderContext fields and array arguments that are non-empty float arrays get gradients enabled
  automatically; Model fields only when the user enables them.

  `backward` is called with the forward arguments and the tape paused. Like a kernel adjoint, it
  must accumulate (`+=`) into the input gradients and then consume the output gradients: each must
  afterwards equal the cotangent of the array's pre-call contents (zero where the forward overwrote
  it; clone it first when the field is in place), because the earlier call that wrote the array
  receives it next. It must not change `requires_grad`, should keep scratch cotangents in inline
  allocations rather than `Data` gradients, and should recompute intermediates from its inputs
  rather than rely on outputs that later calls may overwrite.

  Attributes:
    backward: accumulates input gradients from output gradients; if None, the forward's launches
      are recorded natively and its kernels must be compiled with `enable_backward=True`
    in_fields: differentiable fields of the first Data argument
    out_fields: differentiable fields of the last Data argument
    model_fields: Model fields that can receive gradients when the user enables `requires_grad`
    render_fields: differentiable fields of the RenderContext argument
    array_args: names of differentiable standalone array arguments
    supported: called untaped with the forward arguments, except during CUDA graph capture;
      returns reasons differentiation is not supported
  """

  backward: Callable[..., None] | None = None
  in_fields: tuple[str, ...] = ()
  out_fields: tuple[str, ...] = ()
  model_fields: tuple[str, ...] = ()
  render_fields: tuple[str, ...] = ()
  array_args: tuple[str, ...] = ()
  supported: Callable[..., Sequence[str] | str | None] | None = None


RULES: dict[str, AdjointRule | None] = {}  # wrapped function name -> rule
_PARAMS: dict[str, tuple[str, ...]] = {}  # wrapped function name -> parameter names


def register_adjoint(
  fn: Callable | str,
  backward: Callable[..., None] | None = None,
  *,
  in_fields: Sequence[str] = (),
  out_fields: Sequence[str] | None = None,
  model_fields: Sequence[str] = (),
  render_fields: Sequence[str] = (),
  array_args: Sequence[str] = (),
  supported: Callable[..., Sequence[str] | str | None] | None = None,
) -> Callable:
  """Registers the `AdjointRule` for API function `fn` (a function or its name).

  Can be called directly or used as a decorator on `backward`. `out_fields` defaults to
  `in_fields`.
  """
  name = fn if isinstance(fn, str) else fn.__name__
  if name not in RULES:
    raise ValueError(f"Function {name!r} is not registered as differentiable.")
  out_fields = in_fields if out_fields is None else out_fields
  for seq, valid, label in (
    (in_fields, _FIELDS[types.Data], "Data fields"),
    (out_fields, _FIELDS[types.Data], "Data fields"),
    (model_fields, _FIELDS[types.Model], "Model fields"),
    (render_fields, _FIELDS[types.RenderContext], "RenderContext fields"),
    (array_args, _PARAMS[name], "array_args"),
  ):
    if isinstance(seq, str) or not isinstance(seq, Sequence) or not set(seq).issubset(valid):
      raise ValueError(f"Invalid {label} for {name!r}: {seq!r}")

  def register(bwd: Callable[..., None] | None) -> Callable[..., None] | None:
    fields = (in_fields, out_fields, model_fields, render_fields, array_args)
    RULES[name] = AdjointRule(bwd, *map(tuple, fields), supported)
    return bwd

  # Register now so a direct call with `backward=None` records natively; as a decorator, the
  # returned `register` replaces the rule with one that has the decorated backward:
  register(backward)
  return register if backward is None else backward


def _wrap(fn: Callable) -> Callable:
  """Wraps API function `fn` to validate and record it on an active `wp.Tape`."""
  name = fn.__name__
  params = tuple(inspect.signature(fn).parameters)
  RULES[name] = None
  _PARAMS[name] = params

  @functools.wraps(fn)
  def wrapper(*args, **kwargs):
    tape = wp_context.runtime.tape if wp_context.runtime is not None else None
    if tape is None or isinstance(tape, _ScratchTape):
      return fn(*args, **kwargs)

    rule = RULES[name]
    if rule is None:
      raise NotImplementedError(f"Differentiation of '{name}' is not supported.")

    bound = dict(zip(params, args), **kwargs)
    # Distinct containers by type in signature order; the first Data is input, the last is output:
    objs = {cls: list({id(o): o for o in map(bound.get, params) if isinstance(o, cls)}.values()) for cls in _FIELDS}
    m, data, rc = objs[types.Model][:1], objs[types.Data], objs[types.RenderContext][:1]

    # Arrays the rule differentiates; all but the opt-in Model fields get gradients enabled:
    roles = ((data[:1], rule.in_fields), (data[-1:], rule.out_fields), (rc, rule.render_fields))
    auto = [*(get(c, p) for cs, fs in roles for c in cs for p in fs), *(bound.get(k) for k in rule.array_args)]
    promote = [a for a in auto if isinstance(a, wp.array) and a.size > 0 and _is_float(a)]
    tracked = [*(get(c, p) for c in m for p in rule.model_fields), *auto]
    allowed = {id(a) for a in tracked}

    # Standalone arrays with (or about to get) gradients must be float `array_args`:
    for k, obj in bound.items():
      if (
        isinstance(obj, wp.array)
        and (obj.requires_grad or any(obj is a for a in promote))
        and (k not in rule.array_args or not _is_float(obj))
      ):
        raise NotImplementedError(f"Differentiation of '{name}' with array argument '{k}' is not supported.")

    # Container fields with gradients must be float fields of the rule; other float fields with
    # gradients (`foreign`) are allowed unless this call reads or writes them (checked below):
    foreign = []
    for cls, cs in objs.items():
      for c in cs:
        for path in _FIELDS[cls]:
          arr = get(c, path)
          if not isinstance(arr, wp.array) or not arr.requires_grad or (id(arr) in allowed and _is_float(arr)):
            continue
          label = f"{cls.__name__} field '{path}'"
          if not _is_float(arr):
            raise NotImplementedError(f"Differentiation of '{name}' with {label} is not supported.")
          foreign.append((label, arr))

    reasons = None if rule.supported is None or wp.get_device().is_capturing else _call(None, rule.supported, *args, **kwargs)
    if reasons:
      msg = reasons if isinstance(reasons, str) else ", ".join(reasons)
      raise NotImplementedError(f"Differentiation of '{name}' is not supported: {msg}.")

    for arr in promote:
      if not arr.requires_grad:
        arr.requires_grad = True

    # The forward runs on a scratch tape, whose launches are checked before native rules append
    # them to `tape`; custom rules record their backward instead:
    fwd_tape = _ScratchTape()
    result = _call(fwd_tape, fn, *args, verify_access=rule.backward is None, **kwargs)
    launches = fwd_tape.launches

    # Reading a foreign field would drop its gradient, and writing it would send this call's output
    # gradient to the earlier call:
    accessed = _accessed(launches)
    for label, arr in foreign:
      if id(arr) in accessed:
        raise NotImplementedError(
          f"Differentiation of '{name}' is not supported: it accesses {label}, which is not declared by this rule."
        )
    if rule.backward is None and not all(map(_has_adjoint, launches)):
      raise NotImplementedError(f"Differentiation of '{name}' requires a backward rule.")

    arrays = [a for a in tracked if isinstance(a, wp.array) and a.size > 0 and a.requires_grad]
    if rule.backward is None:
      tape.launches.extend(launches)
      # Warp registers kernel gradients only in `tape.backward`; register the rule's arrays now,
      # with those of the recorded copies, so that a `tape.zero()` recorded before the first
      # backward (as in a captured graph) zeroes them:
      tape.gradients.update({**fwd_tape.gradients, **{a: a.grad for a in arrays}})
    else:

      def backward():
        # Setting `requires_grad = False` after recording frees `.grad`:
        if any(a.grad is None for a in arrays):
          raise RuntimeError(f"Gradients for '{name}' were disabled after recording.")
        _call(None, rule.backward, *args, **kwargs)

      tape.record_func(backward=backward, arrays=arrays)
    return result

  return wrapper


def differentiable(namespace: dict[str, object]) -> None:
  """Wraps the public functions in `namespace` (`mujoco_warp.__init__`)."""
  for name, obj in list(namespace.items()):
    if inspect.isfunction(obj) and not name.startswith("_"):
      namespace[name] = _wrap(obj)
