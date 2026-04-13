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
"""Computes the discount."""

import abc
from collections.abc import Mapping

from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types
from reaf.core import termination_checker
import tree


class DiscountProvider(abc.ABC):
  """Computes the discount."""

  @abc.abstractmethod
  def name(self) -> str:
    """Returns a unique string identifier for this object."""

  @abc.abstractmethod
  def compute_discount(
      self,
      required_features: Mapping[str, gdmr_types.ArrayType],
      termination_state: termination_checker.TerminationResult,
  ) -> tree.Structure[gdmr_types.ArrayType]:
    """Computes the discount.

    Args:
      required_features: Measurements and features computed by the task logic
        that are required by this provider, i.e. that have keys specified by
        `required_features_keys`.
      termination_state: The termination state as computed by the termination
        checkers.  Returns the discount.

    Returns:
      The discount.
    """

  @abc.abstractmethod
  def discount_spec(self) -> tree.Structure[specs.Array]:
    """Returns the spec of the discount."""

  @abc.abstractmethod
  def required_features_keys(self) -> set[str]:
    """Returns the feature keys that are required to compute the discount."""

  def reset(self) -> None:
    """Resets the internal state of the discount provider."""
    ...
