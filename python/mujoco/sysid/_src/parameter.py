# Copyright 2026 DeepMind Technologies Limited
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

"""Parameter utilities."""

from __future__ import annotations

import copy
from enum import Enum
import pathlib
from typing import Callable, TYPE_CHECKING, TypeAlias

import colorama
import mujoco
import numpy as np
import numpy.typing as npt
from tabulate import tabulate
import yaml

if TYPE_CHECKING:
  from typing_extensions import Self

Fore = colorama.Fore
Style = colorama.Style

ModifierFn: TypeAlias = Callable[[mujoco.MjSpec, "Parameter"], object]


class InertiaType(Enum):
  Mass = 0
  MassIpos = 1
  Pseudo = 2


class Parameter:
  """A single (possibly multi-dimensional) parameter for system identification.

  A Parameter holds a current ``value``, a ``nominal`` baseline, and box
  bounds (``min_value``, ``max_value``).  An optional ``modifier`` callback
  is invoked during model compilation to apply the parameter to an MjSpec.

  Args:
    name: Human-readable identifier (must be unique within a ParameterDict).
    nominal: Nominal (initial) value; scalar or array-like.
    min_value: Lower bound, same shape as *nominal*.
    max_value: Upper bound, same shape as *nominal*.
    frozen: If True the parameter is excluded from optimization.
    modifier: Optional callback ``(MjSpec, Parameter) -> None`` that writes the
      parameter into a spec during model compilation.
  """

  # Type hints for dynamically-added attributes (set by parameter builders).
  if TYPE_CHECKING:
    inertia_type: InertiaType | None
    scale_rot_inertia: bool

  def __init__(
      self,
      name: str,
      nominal: float | npt.ArrayLike,
      min_value: float | npt.ArrayLike,
      max_value: float | npt.ArrayLike,
      frozen: bool = False,
      modifier: ModifierFn | None = None,
  ):
    self.name = name
    self.nominal = np.atleast_1d(nominal)
    self.min_value = np.atleast_1d(min_value)
    self.max_value = np.atleast_1d(max_value)
    self.value = self.nominal.copy()
    self.frozen = frozen
    self.modifier = modifier

  @property
  def size(self) -> int:
    return self.nominal.size

  @property
  def shape(self) -> tuple[int, ...]:
    return self.nominal.shape

  def apply_modifier(self, spec: mujoco.MjSpec) -> None:
    """Apply this parameter's modifier callback to *spec*, if one is set."""
    if self.modifier:
      self.modifier(spec, self)

  def as_vector(self) -> np.ndarray:
    """Return the current value as a flat 1-D array."""
    return self.value.flatten()

  def as_nominal_vector(self) -> np.ndarray:
    """Return the nominal value as a flat 1-D array."""
    return self.nominal.flatten()

  def update_from_vector(self, vector: np.ndarray) -> None:
    """Update the current value from a flat vector.

    Args:
      vector: Flat array of length ``self.size``.
    """
    vector_array = np.atleast_1d(vector)
    if len(vector_array) != self.size:
      raise ValueError(
          f"Input vector length {vector_array.size} does not match "
          f"parameter size {self.size}."
      )
    self.value = vector_array.reshape(self.shape)

  def get_bounds(self) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(lower, upper)`` bound arrays, each flat 1-D."""
    return (
        self.min_value.flatten(),
        self.max_value.flatten(),
    )

  def reset(self) -> None:
    """Reset the current value to nominal."""
    self.value = self.nominal.copy()

  def sample(self, rng: np.random.Generator | None = None) -> np.ndarray:
    """Sample a random value uniformly within bounds.

    Args:
      rng: Optional numpy random generator. Uses default if None.
    """
    if rng is None:
      rng = np.random.default_rng()
    return rng.uniform(self.min_value.flatten(), self.max_value.flatten())

  def __str__(self) -> str:
    """Return a string representation of the parameter."""
    if self.size == 1:
      return (
          f"{Fore.CYAN}{self.name}{Style.RESET_ALL}: "
          f"{Fore.GREEN}{float(self.value.item()):.3g}{Style.RESET_ALL} "
          f"∈ [{Fore.YELLOW}{float(self.min_value.item()):.3g}, "
          f"{float(self.max_value.item()):.3g}{Style.RESET_ALL}]"
      )
    else:
      return (
          f"{Fore.CYAN}{self.name}{Style.RESET_ALL}: "
          f"{Fore.GREEN}array(shape={self.shape}){Style.RESET_ALL} "
          f"∈ [{Fore.YELLOW}min={np.min(self.min_value):.3g}, "
          f"max={np.max(self.max_value):.3g}{Style.RESET_ALL}]"
      )

  def __repr__(self) -> str:
    return self.__str__()

  def __getstate__(self):
    return {
        "name": self.name,
        "nominal": (
            self.nominal.tolist()
            if isinstance(self.nominal, np.ndarray)
            else self.nominal
        ),
        "min_value": (
            self.min_value.tolist()
            if isinstance(self.min_value, np.ndarray)
            else self.min_value
        ),
        "max_value": (
            self.max_value.tolist()
            if isinstance(self.max_value, np.ndarray)
            else self.max_value
        ),
        "value": (
            self.value.tolist()
            if isinstance(self.value, np.ndarray)
            else self.value
        ),
        "frozen": self.frozen,
    }

  def __setstate__(self, state):
    self.name = state["name"]
    self.nominal = np.array(state["nominal"])
    self.min_value = np.array(state["min_value"])
    self.max_value = np.array(state["max_value"])
    self.value = np.array(state["value"])
    self.frozen = state["frozen"]

  # Override default deepycopy so lambda references get copied
  def __deepcopy__(self, memo):
    cls = self.__class__
    result = cls.__new__(cls)
    for k, v in self.__dict__.items():
      setattr(result, k, copy.deepcopy(v, memo))
    return result


class ParameterDict:
  """An ordered collection of :class:`Parameter` objects.

  Behaves like a ``dict[str, Parameter]`` with convenience methods for
  vectorized access (``as_vector`` / ``update_from_vector``), serialization,
  and tabular comparison of parameter estimates.

  Frozen parameters are silently skipped by vector/bounds methods so that the
  decision-variable dimension seen by optimizers matches only the free params.
  """

  def __init__(self, parameters: dict[str, Parameter] | None = None):
    if parameters is None:
      self.parameters = {}
    else:
      self.parameters = parameters

  def __getitem__(self, key: str) -> Parameter:
    return self.parameters[key]

  def __setitem__(self, key: str, value: Parameter) -> None:
    self.parameters[key] = value

  def __contains__(self, key: str) -> bool:
    return key in self.parameters

  def __len__(self) -> int:
    return len(self.parameters)

  def copy(self) -> Self:
    """Return a deep copy of this ParameterDict."""
    return copy.deepcopy(self)

  def add(self, param: Parameter) -> None:
    """Add a Parameter, keyed by its ``name``."""
    self.parameters[param.name] = param

  def update(self, pdict: Self) -> None:
    for keys in pdict.keys():
      if keys in self.parameters:
        raise ValueError(
            f"Parameter '{keys}' already exists in the dictionary."
        )
      self.parameters[keys] = pdict[keys]

  def keys(self) -> list[str]:
    return list(self.parameters.keys())

  def values(self) -> list[Parameter]:
    return list(self.parameters.values())

  def items(self) -> list[tuple[str, Parameter]]:
    return list(self.parameters.items())

  @property
  def size(self) -> int:
    """Get the total size of all non-frozen parameters."""
    return sum(p.size for p in self.parameters.values() if not p.frozen)

  def as_vector(self, include_frozen=False) -> np.ndarray:
    """Convert all non-frozen parameters to a flat vector."""
    vectors = [
        p.as_vector()
        for p in self.parameters.values()
        if not p.frozen or include_frozen
    ]
    return np.concatenate(vectors) if vectors else np.array([])

  def as_nominal_vector(self, include_frozen=False) -> np.ndarray:
    """Get the nominal values of parameters as a flat array."""
    vectors = [
        p.as_nominal_vector()
        for p in self.parameters.values()
        if not p.frozen or include_frozen
    ]
    return np.concatenate(vectors) if vectors else np.array([])

  def update_from_vector(self, vector: np.ndarray) -> None:
    """Update all non-frozen parameters from a flat vector.

    Args:
      vector: Flat array whose length equals the total size of non-frozen
        parameters.
    """
    start = 0
    for param in self.parameters.values():
      if not param.frozen:
        size = param.size
        param.update_from_vector(vector[start : start + size])
        start += size

  def save_to_disk(self, path: str | pathlib.Path) -> None:
    """Save the parameter dictionary to disk (schema and data).

    Args:
      path: Path where the data will be saved.
    """
    parameter_dicts = {
        name: param.__getstate__() for name, param in self.parameters.items()
    }
    with open(path, "w") as handle:
      yaml.safe_dump(parameter_dicts, handle, default_flow_style=False)

  @classmethod
  def load_from_disk(cls, path: str | pathlib.Path) -> "ParameterDict":
    """Load parameter dictionary from disk (schema and data).

    Args:
      path: Path to the saved data.

    Returns:
      A new ParameterDict object.
    """
    with open(path, "r") as handle:
      parameter_dicts = yaml.safe_load(handle)

    parameters = {}
    for name, param_dict in parameter_dicts.items():
      param = Parameter.__new__(Parameter)
      param.__setstate__(param_dict)
      parameters[name] = param

    return ParameterDict(parameters)

  def get_bounds(self) -> tuple[np.ndarray, np.ndarray]:
    """Get the bounds for all non-frozen parameters."""
    lower_bounds = []
    upper_bounds = []
    for param in self.parameters.values():
      if not param.frozen:
        lb, ub = param.get_bounds()
        lower_bounds.append(lb)
        upper_bounds.append(ub)

    return (
        np.concatenate(lower_bounds) if lower_bounds else np.array([]),
        np.concatenate(upper_bounds) if upper_bounds else np.array([]),
    )

  def reset(self) -> None:
    """Reset all parameters to their nominal values."""
    for param in self.parameters.values():
      param.reset()

  def sample(self, rng: np.random.Generator | None = None) -> np.ndarray:
    """Sample parameter values within bounds for non-frozen parameters.

    Args:
      rng: Optional numpy random generator. Uses default if None.
    """
    if rng is None:
      rng = np.random.default_rng()
    lower_bounds, upper_bounds = self.get_bounds()
    return rng.uniform(lower_bounds, upper_bounds)

  def randomize(self, rng: np.random.Generator | None = None) -> None:
    """Randomize parameter values for non-frozen parameters.

    Args:
      rng: Optional numpy random generator. Uses default if None.
    """
    for param in self.parameters.values():
      if not param.frozen:
        param.value = param.sample(rng)

  def compare_parameters(
      self,
      init_params: np.ndarray,
      predicted_params: np.ndarray,
      measured_params: np.ndarray | None = None,
      sig_digits: int = 4,
  ) -> str:
    """Compare true and predicted parameter values.

    Args:
      init_params: Initial parameter values as a flat array.
      predicted_params: Predicted parameter values as a flat array.
      measured_params: True parameter values as a flat array.
      sig_digits: Number of significant digits to display.

    Returns:
      A formatted string with parameter comparison table.
    """
    # Get the vector of non-frozen parameters
    non_frozen_vector = self.as_vector()

    if non_frozen_vector.size == 0:
      return "No non-frozen parameters to compare."

    if len(init_params) != non_frozen_vector.size:
      raise ValueError(
          f"Initial parameter vector length {len(init_params)} does not match "
          f"the number of non-frozen parameters {non_frozen_vector.size}."
      )

    if len(predicted_params) != non_frozen_vector.size:
      raise ValueError(
          f"Predicted parameter vector length {len(predicted_params)} does not"
          " match the number of non-frozen parameters"
          f" {non_frozen_vector.size}."
      )

    if measured_params is not None:
      if len(measured_params) != non_frozen_vector.size:
        raise ValueError(
            f"True parameter vector length {len(measured_params)} does not"
            " match the number of non-frozen parameters"
            f" {non_frozen_vector.size}."
        )

    # Compute error metrics.
    rel_deltas = []
    for i in range(predicted_params.shape[0]):
      if (
          init_params[i] == 0
          or np.abs(predicted_params[i] - init_params[i])
          / np.abs(init_params[i])
          > 2e1
      ):
        rel_deltas.append(np.nan)
      else:
        rel_deltas.append(
            np.abs(predicted_params[i] - init_params[i])
            / np.abs(init_params[i])
        )
    rel_deltas = np.array(rel_deltas)
    overall_rms_delta = np.sqrt(np.mean((predicted_params - init_params) ** 2))
    abs_deltas = np.abs(predicted_params - init_params)

    if measured_params is not None:
      rel_errors = []
      for i in range(predicted_params.shape[0]):
        if (
            measured_params[i] == 0
            or np.abs(predicted_params[i] - measured_params[i])
            / np.abs(measured_params[i])
            > 2e1
        ):
          rel_errors.append(np.nan)
        else:
          rel_errors.append(
              np.abs(predicted_params[i] - measured_params[i])
              / np.abs(measured_params[i])
          )
      rel_errors = np.array(rel_errors)

      overall_rmse = np.sqrt(np.mean((predicted_params - measured_params) ** 2))
      abs_errors = np.abs(predicted_params - measured_params)
    else:
      overall_rmse = np.nan
      abs_errors = np.full_like(predicted_params, np.nan)
      rel_errors = np.full_like(predicted_params, np.nan)

    lower_bounds, upper_bounds = self.get_bounds()

    def format_number(x):
      """Format number with fixed width for proper table alignment."""
      if abs(x) < 0.01:
        return f"{x: .{sig_digits}e}"
      else:
        return f"{x: .{sig_digits}f}"

    def get_color_for_error(error):
      """Get color code based on relative error magnitude."""
      if error < 0.02:
        return Fore.GREEN
      elif error < 0.1:
        return Fore.YELLOW
      else:
        return Fore.RED

    def create_table_row(param_name, idx):
      """Create a formatted table row for a parameter at the given index."""

      true = measured_params[idx] if measured_params is not None else np.nan
      init = init_params[idx]
      est = predicted_params[idx]
      lower_bound = lower_bounds[idx]
      upper_bound = upper_bounds[idx]
      delta = abs_deltas[idx]
      error = abs_errors[idx] if measured_params is not None else np.nan
      rel_delta = rel_deltas[idx]
      rel_err = rel_errors[idx] if measured_params is not None else np.nan

      # If a parameter is near the boundary make it magneta
      if (abs(est - lower_bound) < 1e-8 + 1e-3 * abs(lower_bound)) or (
          abs(est - upper_bound) < 1e-8 + 1e-3 * abs(upper_bound)
      ):
        color = Fore.MAGENTA
      else:
        if measured_params is None:
          color = get_color_for_error(rel_delta)
        else:
          color = get_color_for_error(error)

      # Format all values with appropriate colors
      if np.isnan(true):
        measured_val = ""
      else:
        measured_val = f"{Fore.BLUE}{format_number(true)}{Style.RESET_ALL}"
      init_val = f"{Fore.BLUE}{format_number(init)}{Style.RESET_ALL}"
      est_val = f"{color}{format_number(est)}{Style.RESET_ALL}"

      lower_bound_val = (
          f"{Fore.BLUE}{format_number(lower_bound)}{Style.RESET_ALL}"
      )
      upper_bound_val = (
          f"{Fore.BLUE}{format_number(upper_bound)}{Style.RESET_ALL}"
      )

      if np.isnan(error):
        abs_err_val = ""
      else:
        abs_err_val = f"{color}{format_number(error)}{Style.RESET_ALL}"
      abs_delta_val = f"{color}{format_number(delta)}{Style.RESET_ALL}"

      if np.isnan(rel_err):
        rel_err_val = ""
      else:
        rel_err_val = f"{color}{rel_err * 100:.1f}%{Style.RESET_ALL}"

      if np.isnan(rel_delta):
        rel_delta_val = ""
      else:
        rel_delta_val = f"{color}{rel_delta * 100:.1f}%{Style.RESET_ALL}"

      return [
          f"{Fore.CYAN}{param_name.ljust(20)}{Style.RESET_ALL}",
          init_val,
          measured_val,
          est_val,
          lower_bound_val,
          upper_bound_val,
          abs_err_val,
          abs_delta_val,
          rel_err_val,
          rel_delta_val,
      ]

    # Build table data.
    table_data = []
    non_frozen_idx = 0  # Index for non-frozen parameters in the arrays

    for param_name, param in self.parameters.items():
      if param.frozen:
        continue  # Skip frozen parameters

      if param.size == 1:
        table_data.append(create_table_row(param_name, non_frozen_idx))
        non_frozen_idx += 1
      else:
        for i in range(param.size):
          if param.shape == (param.size,):
            element_name = f"{param_name}[{i}]"
          else:
            multi_idx = np.unravel_index(i, param.shape)
            idx_str = ",".join(str(x) for x in multi_idx)
            element_name = f"{param_name}[{idx_str}]"
          table_data.append(create_table_row(element_name, non_frozen_idx))
          non_frozen_idx += 1

    # Create and return the formatted table.
    headers = [
        "Parameter",
        "Initial",
        "Nominal",
        "Identified",
        "Lower",
        "Upper",
        "Abs Err",
        "Abs Del",
        "Rel Err",
        "Rel Del",
    ]

    table = tabulate(
        table_data, headers=headers, tablefmt="outline", disable_numparse=True
    )

    overall_rmse_val = "" if np.isnan(overall_rmse) else f"{overall_rmse:.4g}"
    overall_rms_delta_val = f"{overall_rms_delta:.4g}"

    return (
        f"{table}\nRMSE: {overall_rmse_val}\nRMS Delta: {overall_rms_delta_val}"
    )

  def __str__(self) -> str:
    """Return a string representation of all parameters in the dictionary."""
    if not self.parameters:
      return f"{Fore.CYAN}ParameterDict{Style.RESET_ALL}(empty)"

    param_strings = []
    for name, param in self.parameters.items():
      if param.size == 1:
        param_strings.append(f"  {param}")
      else:
        # For multi-dimensional parameters, show each element on its own line
        param_strings.append(f"  {Fore.CYAN}{name}{Style.RESET_ALL}:")
        if param.shape == (param.size,):  # 1D array
          for i in range(param.size):
            param_strings.append(
                f"    [{i}]: {Fore.GREEN}{param.value[i]:.3g}{Style.RESET_ALL} "
                f"∈ [{Fore.YELLOW}{param.min_value[i]:.3g}, "
                f"{param.max_value[i]:.3g}{Style.RESET_ALL}]"
            )
        else:  # Multi-dimensional array
          flat_idx = 0
          for idx in np.ndindex(param.shape):
            idx_str = ",".join(str(x) for x in idx)
            param_strings.append(
                f"    [{idx_str}]:"
                f" {Fore.GREEN}{param.value[idx]:.3g}{Style.RESET_ALL} ∈"
                f" [{Fore.YELLOW}{param.min_value.flat[flat_idx]:.3g},"
                f" {param.max_value.flat[flat_idx]:.3g}{Style.RESET_ALL}]"
            )
            flat_idx += 1

    params_str = "\n".join(param_strings)
    return f"{Fore.CYAN}ParameterDict{Style.RESET_ALL}(\n{params_str}\n)"

  def __repr__(self) -> str:
    return self.__str__()

  def get_non_frozen_parameter_names(self) -> list[str]:
    """Get the names of all non-frozen parameters, expanding multi-dimensional ones."""
    names = []
    for name, param in self.parameters.items():
      if not param.frozen:
        if param.size == 1:
          names.append(name)
        else:
          if param.shape == (param.size,):
            for i in range(param.size):
              names.append(f"{name}[{i}]")
          else:
            for idx in np.ndindex(param.shape):
              idx_str = ",".join(map(str, idx))
              names.append(f"{name}[{idx_str}]")
    return names

  def get_parameter_info(self) -> str:
    """Get information about all parameters in the dictionary.

    Returns:
      A formatted string with parameter information.
    """
    if not self.parameters:
      return "No parameters in dictionary."

    info = []
    info.append(f"{Fore.CYAN}Parameter Information:{Style.RESET_ALL}")
    info.append(
        f"{Fore.CYAN}{'Name':<20} {'Size':<10} {'Shape':<15}"
        f" {'Frozen':<10}{Style.RESET_ALL}"
    )
    info.append("-" * 60)

    for name, param in self.parameters.items():
      frozen_str = (
          f"{Fore.RED}Yes{Style.RESET_ALL}"
          if param.frozen
          else f"{Fore.GREEN}No{Style.RESET_ALL}"
      )
      info.append(
          f"{Fore.CYAN}{name:<20} {param.size:<10} {str(param.shape):<15}"
          f" {frozen_str}{Style.RESET_ALL}"
      )

    return "\n".join(info)
