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
"""Checks if the episode should terminate."""

import abc
from collections.abc import Mapping
import enum
from typing import Self

from gdm_robotics.interfaces import types as gdmr_types


class TerminationResult(enum.IntFlag):
  """The result of an episode termination check.

  The TerminationResult refers to the possibility for an episode to terminate.
  For more details on the concept of termination we refer the readers to
  https://github.com/google-deepmind/dm_env/blob/master/docs/index.md#environment-api-and-semantics.

  Note that this enum does not refer to the possible causes of termination but
  only how the termination impacts the learning process.

  The result can be one of the following options:
  - DO_NOT_TERMINATE: The episode should not terminate.
  - TRUNCATE: The epsisode should terminate. Truncation implies a non-failure
    final state. Usually this is associated with a non-zero discount.
  - TERMINATE: The episode should terminate as the environment is in some
    final state. Usually this is associated with a zero discount for e.g.
    finite-horizon RL.
  """

  DO_NOT_TERMINATE = 0
  TRUNCATE = 2**0
  TERMINATE = 2**1

  def is_terminated(self) -> bool:
    return self == TerminationResult.TERMINATE

  def is_truncated(self) -> bool:
    return self == TerminationResult.TRUNCATE

  def combine(self, other: Self) -> Self:
    # TERMINATE has precedence over TRUNCATE, which in turn has precedence over
    # DO_NOT_TERMINATE. Given the definitions above, this can be implemented as
    # a maximum operator. To also enable tracing with JAX, we implement this in
    # a branchless manner using bitwise operations that preserve the type.
    # Note that JAX will trace TerminationResult values as ints.
    # Approach:
    # - self ^ (self ^ other) == other
    # - (-1 * (self < other)) will be bitmask of all 1s iff self < other.
    # - AND with (self ^ other) will result in either update or no-op bitmask.
    return self ^ ((self ^ other) & (-1 * (self < other)))


class TerminationChecker(abc.ABC):
  """Checks if the episode should terminate."""

  @abc.abstractmethod
  def name(self) -> str:
    """Returns a unique string identifier for this object."""

  @abc.abstractmethod
  def check_termination(
      self, required_features: Mapping[str, gdmr_types.ArrayType]
  ) -> TerminationResult:
    """Checks if the episode should terminate.

    Args:
      required_features: Measurements and features computed by the task logic
        that are required by this checker, i.e. that have keys specified by
        `required_features_keys`.

    Returns if the episode should terminate (and if so, what kind of
      termination).
    """

  @abc.abstractmethod
  def required_features_keys(self) -> set[str]:
    """Returns the feature keys that are required to check the termination."""

  def reset(self) -> None:
    """Resets the internal state of the termination checker."""
    ...
