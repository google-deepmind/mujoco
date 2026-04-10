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
"""Computes a constant discount given the termination state.

This provider returns a discount of 0.0 in case of termination and 1.0
otherwise (i.e. for truncation and not termination).

It is usually safe to use this discount provider for environments that return
strictly positive rewards.
"""

from collections.abc import Mapping

from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types
import numpy as np
from reaf.core import discount_provider
from reaf.core import termination_checker
import tree


class DefaultDiscountProvider(discount_provider.DiscountProvider):
  """Computes a constant discount given the termination state.

  This provider returns a discount of 0.0 in case of termination and 1.0
  otherwise (i.e. for truncation and not termination).

  It is usually safe to use this discount provider for environments that return
  strictly positive rewards.
  """

  def __init__(self, name: str = "default_discount_provider"):
    self._name = name
    self._spec = specs.BoundedArray(
        shape=(), dtype=np.float64, minimum=0.0, maximum=1.0, name="discount"
    )

  def name(self) -> str:
    """Returns a unique string identifier for this object."""
    return self._name

  def compute_discount(
      self,
      unused_required_features: Mapping[str, gdmr_types.ArrayType],
      termination_state: termination_checker.TerminationResult,
  ) -> tree.Structure[gdmr_types.ArrayType]:
    """Computes the discount.

    Args:
      unused_required_features: Unused
      termination_state: The termination state as computed by the termination
        checkers.  Returns the discount.

    Returns:
      The discount.
    """
    if termination_state == termination_state.TERMINATE:
      return np.asarray(0).astype(self._spec.dtype)
    else:  # TRUNCATION or DO_NOT_TERMINATE
      return np.asarray(1.0).astype(self._spec.dtype)

  def discount_spec(self) -> tree.Structure[specs.Array]:
    """Returns the spec of the discount."""
    return self._spec

  def required_features_keys(self) -> set[str]:
    """Returns the feature keys that are required to compute the discount."""
    return set()
