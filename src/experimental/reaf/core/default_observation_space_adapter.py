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
"""ObservationSpaceAdapter supporting filtering, renaming and type conversion."""

import abc
from collections.abc import Iterable, Mapping
import dataclasses

from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types
import numpy as np
import numpy.typing as npt
from reaf.core import observation_space_adapter
import tree


@dataclasses.dataclass(frozen=True, kw_only=True)
class RenameInfo:
  original_key: str
  renamed_key: str


class ObservationTypeMapper(abc.ABC):
  """Maps from REAF features and specs into corresponding environment types."""

  @abc.abstractmethod
  def to_observation_spec(
      self, features_spec: Mapping[str, specs.Array]
  ) -> gdmr_types.ObservationSpec:
    """Convert the features spec into the environment observation spec."""

  @abc.abstractmethod
  def to_observations(
      self, features: Mapping[str, gdmr_types.ArrayType]
  ) -> tree.Structure[gdmr_types.ArrayType]:
    """Convert the features into the environment observations."""


class _DefaultObservationTypeMapper(ObservationTypeMapper):
  """An ObservationTypeMapper that returns the input features specs and dict.

  This `ObservationTypeMapper` maps observations from the more constrained
  `Mapping[str, ArrayType]` used in the task layer to the more generic
  `tree.Structure[ArrayType]` exposed by the GDM Environment.
  """

  def to_observation_spec(
      self, features_spec: Mapping[str, specs.Array]
  ) -> gdmr_types.ObservationSpec:
    """Returns the features spec, unmodified, as a `gdmr_types.ObservationSpec`."""
    return features_spec

  def to_observations(
      self, features: Mapping[str, gdmr_types.ArrayType]
  ) -> tree.Structure[gdmr_types.ArrayType]:
    """Returns the features, unmodified, as a `tree.Structure`."""
    return features


class DefaultObservationSpaceAdapter(
    observation_space_adapter.ObservationSpaceAdapter
):
  """Observation adapter supporting filtering, renaming and type conversion.

    This adapter supports filtering, renaming, and converting REAF features into
    environment observations.

    The order of operations is the following:
    1) Filtering, i.e. feature selection.
    2) Downcasting floats to max_float_dtype.
    3) Renaming.
    4) Type conversion.

  Please refer to the constructor documentation for more information.
  """

  def __init__(
      self,
      *,
      task_features_spec: Mapping[str, specs.Array],
      selected_features: Iterable[str] | None,
      renamed_features: Iterable[RenameInfo] | None,
      observation_type_mapper: ObservationTypeMapper | None,
      max_float_dtype: npt.DTypeLike = np.float64,
  ):
    """Initializes the observation space adapter.

    Args:
      task_features_spec: The spec of all the features exposed by the task
        layer.
      selected_features: The features that will be exposed as observations. If
        None, all features will be exposed, i.e. no filtering.
      renamed_features: `RenameInfo` objects specifying which features should be
        renamed and the corresponding new name. If empty or None, no renaming
        will occur.
      observation_type_mapper: An `ObservationTypeMapper` specifying how to
        convert the task layer features data type (i.e. a Mapping[str,
        ArrayType]) into the more generic type exposed by the GDM Environment
        (i.e. a tree.Structure[ArrayType]). If None, an instance of
        `_DefaultObservationTypeMapper` is used which converts the task logic
        layer features dictionary to the more generic type (i.e.
        `tree.Structure[ArrayType])` exposed by the environment.
      max_float_dtype: The maximum float dtype to use for downcasting floats.
    """
    if not np.issubdtype(max_float_dtype, np.floating):
      raise ValueError(
          'max_float_dtype must be a floating point dtype. Got'
          f' {max_float_dtype}'
      )
    self._max_float_dtype = max_float_dtype
    self._max_bits = np.finfo(self._max_float_dtype).bits
    self._task_features_spec = task_features_spec
    self._selected_filter = selected_features
    self._renamed_features = renamed_features or ()
    self._observation_type_mapper = (
        observation_type_mapper or _DefaultObservationTypeMapper()
    )
    self._check_specs_consistency()
    # Compute the observation spec only once.
    self._observation_spec = self._compute_observation_spec()

  def _check_specs_consistency(self) -> None:
    # Check that filter keys are present in the spec.
    if self._selected_filter is not None:
      all_features = self._task_features_spec.keys()
      features = set()
      for feature in self._selected_filter:
        if feature not in all_features:
          raise ValueError(f'Feature {feature} is not present in the spec.')
        features.add(feature)
    else:
      # No filter applied. Select all features.
      features = set(self._task_features_spec.keys())

    # Check renaming.
    for rename_info in self._renamed_features:
      if rename_info.original_key not in features:
        raise ValueError(
            f'Feature {rename_info.original_key} is not present in the spec.'
        )

  def observations_from_features(
      self, features: Mapping[str, gdmr_types.ArrayType]
  ) -> tree.Structure[gdmr_types.ArrayType]:
    """Converts the features into the final environment observations."""
    # 1. Filter the observations.
    if (selected_features := self._selected_filter) is None:
      # No filter. Expose all observations.
      filtered_features = dict(features)
    else:
      filtered_features = {
          k: v for k, v in features.items() if k in selected_features  # pytype: disable=unsupported-operands
      }

    # 2. Downcast floats to max_float_dtype.
    filtered_features = {
        k: self._downcast_if_necessary(v) for k, v in filtered_features.items()
    }

    # 3. Rename.
    for rename_info in self._renamed_features:
      # Rename the feature.
      value = filtered_features[rename_info.original_key]
      del filtered_features[rename_info.original_key]
      filtered_features[rename_info.renamed_key] = value

    # 4. Convert type.
    return self._observation_type_mapper.to_observations(filtered_features)

  def _compute_observation_spec(self) -> gdmr_types.ObservationSpec:
    """Computes the observation spec."""
    # 1. Filter the specs
    if (features_to_filter := self._selected_filter) is None:
      # The observation spec corresponds to the task features spec.
      filtered_specs = dict(self._task_features_spec)
    else:
      filtered_specs = {
          k: v
          for k, v in self._task_features_spec.items()
          if k in features_to_filter  # pytype: disable=unsupported-operands
      }

    # 2. Downcast floats to max_float_dtype.
    for k, v in filtered_specs.items():
      if self._dtype_needs_downcast(v.dtype):
        filtered_specs[k] = v.replace(dtype=self._max_float_dtype)

    # 3. Rename.
    for rename_info in self._renamed_features:
      # Rename the feature.
      value = filtered_specs[rename_info.original_key]
      del filtered_specs[rename_info.original_key]
      filtered_specs[rename_info.renamed_key] = value

    # 4. Convert the type.
    return self._observation_type_mapper.to_observation_spec(filtered_specs)

  def observation_spec(self) -> gdmr_types.ObservationSpec:
    """Returns the observation spec."""
    return self._observation_spec

  def task_features_keys(self) -> set[str]:
    """Returns the task features keys that will be converted by this adapter."""
    return set(self._task_features_spec.keys())

  def _downcast_if_necessary(
      self, value: gdmr_types.ArrayType
  ) -> gdmr_types.ArrayType:
    if (
        hasattr(value, 'dtype') and self._dtype_needs_downcast(value.dtype)
    ) or self._dtype_needs_downcast(type(value)):
      return np.asarray(value).astype(self._max_float_dtype)
    else:
      return value

  def _dtype_needs_downcast(self, dtype: npt.DTypeLike) -> bool:
    return (
        np.issubdtype(dtype, np.floating)
        and np.finfo(dtype).bits > self._max_bits
    )
