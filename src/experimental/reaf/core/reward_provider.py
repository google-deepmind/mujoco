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
"""Computes the reward."""

import abc
from collections.abc import Mapping
import operator
from typing import Callable, TypeAlias, TypeVar, Union

from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types
import numpy as np
import tree


RewardValue: TypeAlias = tree.Structure[gdmr_types.ArrayType]
RewardSpec: TypeAlias = tree.Structure[specs.Array]


class _RewardProvider(abc.ABC):
  """Computes the reward.

  Defines the interface for a reward provider.

  Important: Users should not inherit from this class directly. Instead, use the
  RewardProvider class later in this file.
  """

  @abc.abstractmethod
  def name(self) -> str:
    """Returns a unique string identifier for this object."""

  @abc.abstractmethod
  def compute_reward(
      self, required_features: Mapping[str, gdmr_types.ArrayType]
  ) -> RewardValue:
    """Computes the reward.

    Args:
      required_features: Measurements and features computed by the task logic
        that are required by this provider, i.e. that have keys specified by
        `required_features_keys`.

    Returns the computed reward.
    """

  @abc.abstractmethod
  def reward_spec(self) -> RewardSpec:
    """Returns the spec of the reward."""

  @abc.abstractmethod
  def required_features_keys(self) -> set[str]:
    """Returns the feature keys that are required to compute the reward."""

  def reset(self) -> None:
    """Resets the internal state of the reward provider."""
    ...


RewardProviderOrValue: TypeAlias = Union['RewardProvider', RewardValue]


S = TypeVar('S')
T = TypeVar('T')
UnaryOperator: TypeAlias = Callable[[S], S]
BinaryOperator: TypeAlias = Callable[[S | T, S | T], S | T]


class RewardProvider(_RewardProvider):
  """Computes the reward.

  Important: Users should inherit from this class and implement the abstract
  methods defined in the interface _RewardProvider.
  """

  def __add__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.add, self, other)

  def __radd__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.add, other, self)

  def __sub__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.sub, self, other)

  def __rsub__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.sub, other, self)

  def __mul__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.mul, self, other)

  def __rmul__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.mul, other, self)

  def __truediv__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.truediv, self, other)

  def __rtruediv__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.truediv, other, self)

  def __floordiv__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.floordiv, self, other)

  def __rfloordiv__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.floordiv, other, self)

  def __pow__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.pow, self, other)

  def __rpow__(self, other: RewardProviderOrValue):
    return BinaryOperationRewardProvider(operator.pow, other, self)

  def __getitem__(self, index: slice):
    return GetItemOperationRewardProvider(self, index)

  def __neg__(self):
    return UnaryOperationRewardProvider(operator.neg, self)


class ConstantRewardProvider(RewardProvider):
  """A RewardProvider that always returns the same reward."""

  def __init__(self, reward: RewardValue):
    super().__init__()
    self._reward = reward

  def name(self) -> str:
    return str(self._reward)

  def compute_reward(
      self, required_features: Mapping[str, gdmr_types.ArrayType]
  ) -> RewardValue:
    return self._reward

  def reward_spec(self) -> RewardSpec:
    return tree.map_structure(
        lambda v: specs.Array(v.shape, v.dtype), self._reward
    )

  def required_features_keys(self) -> set[str]:
    return set()


class BinaryOperationRewardProvider(RewardProvider):
  """Applies a binary operator to the result of two reward providers."""

  def __init__(
      self,
      op: BinaryOperator,
      first_reward_provider: RewardProviderOrValue,
      second_reward_provider: RewardProviderOrValue,
  ):
    super().__init__()
    if not isinstance(first_reward_provider, RewardProvider):
      first_reward_provider = ConstantRewardProvider(first_reward_provider)
    if not isinstance(second_reward_provider, RewardProvider):
      second_reward_provider = ConstantRewardProvider(second_reward_provider)
    first_spec = first_reward_provider.reward_spec()
    second_spec = second_reward_provider.reward_spec()
    tree.assert_same_structure(first_spec, second_spec)
    assert all(
        tree.flatten(
            tree.map_structure(
                lambda s1, s2: s1.shape == s2.shape and s1.dtype == s2.dtype,
                first_spec,
                second_spec,
            )
        )
    )
    self._op = op
    self._first_reward_provider = first_reward_provider
    self._second_reward_provider = second_reward_provider
    self._reward_spec = first_reward_provider.reward_spec()
    self._first_required_features_keys = (
        first_reward_provider.required_features_keys()
    )
    self._second_required_features_keys = (
        second_reward_provider.required_features_keys()
    )

  def name(self) -> str:
    op_name = getattr(self._op, '__name__', str(self._op))
    return (
        f'{op_name}({self._first_reward_provider.name()},'
        f' {self._second_reward_provider.name()})'
    )

  def compute_reward(
      self, required_features: Mapping[str, gdmr_types.ArrayType]
  ) -> RewardValue:
    first_required_features = {
        k: v
        for k, v in required_features.items()
        if k in self._first_required_features_keys
    }
    second_required_features = {
        k: v
        for k, v in required_features.items()
        if k in self._second_required_features_keys
    }
    return tree.map_structure(
        self._op,
        self._first_reward_provider.compute_reward(first_required_features),
        self._second_reward_provider.compute_reward(second_required_features),
    )

  def reward_spec(self) -> RewardSpec:
    return self._reward_spec

  def required_features_keys(self) -> set[str]:
    return (
        self._first_required_features_keys | self._second_required_features_keys
    )

  def reset(self) -> None:
    self._first_reward_provider.reset()
    self._second_reward_provider.reset()


class GetItemOperationRewardProvider(RewardProvider):
  """Extracts a slice from the result of a reward provider."""

  def __init__(self, reward_provider: RewardProviderOrValue, index: slice):
    super().__init__()
    if not isinstance(reward_provider, RewardProvider):
      reward_provider = ConstantRewardProvider(reward_provider)
    self._reward_provider = reward_provider
    self._index = index

  def name(self) -> str:
    return f'{self._reward_provider.name}[{self._index}]'

  def compute_reward(
      self, required_features: Mapping[str, gdmr_types.ArrayType]
  ) -> RewardValue:
    return tree.map_structure(
        lambda v: v[self._index],
        self._reward_provider.compute_reward(required_features),
    )

  def reward_spec(self) -> RewardSpec:
    return tree.map_structure(
        lambda s: specs.Array(np.empty(s.shape)[self._index].shape, s.dtype),
        self._reward_provider.reward_spec(),
    )

  def required_features_keys(self) -> set[str]:
    return self._reward_provider.required_features_keys()

  def reset(self) -> None:
    self._reward_provider.reset()


class UnaryOperationRewardProvider(RewardProvider):
  """Applies a unary operator to the result of a reward provider."""

  def __init__(self, op: UnaryOperator, reward_provider: RewardProviderOrValue):
    super().__init__()
    if not isinstance(reward_provider, RewardProvider):
      reward_provider = ConstantRewardProvider(reward_provider)
    self._op = op
    self._reward_provider = reward_provider

  def name(self) -> str:
    op_name = getattr(self._op, '__name__', str(self._op))
    return f'{op_name}({self._reward_provider.name()})'

  def compute_reward(
      self, required_features: Mapping[str, gdmr_types.ArrayType]
  ) -> RewardValue:
    return tree.map_structure(
        self._op, self._reward_provider.compute_reward(required_features)
    )

  def reward_spec(self) -> RewardSpec:
    return self._reward_provider.reward_spec()

  def required_features_keys(self) -> set[str]:
    return self._reward_provider.required_features_keys()

  def reset(self) -> None:
    self._reward_provider.reset()
