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
"""Reward provider which provides a zero reward."""

from collections.abc import Mapping
from dm_env import specs
from gdm_robotics.interfaces import types as gdmr_types
import numpy as np
from reaf.core import reward_provider
import tree


class ZeroRewardProvider(reward_provider.RewardProvider):
  """Reward provider which provides a zero reward."""

  def __init__(self, name: str = 'zero_reward_provider'):
    self._name = name

  def name(self) -> str:
    return self._name

  def compute_reward(
      self, required_features: Mapping[str, gdmr_types.ArrayType]
  ) -> tree.Structure[gdmr_types.ArrayType]:
    """Returns a zero reward."""
    return np.zeros(1)

  def reward_spec(self) -> tree.Structure[specs.Array]:
    """Returns the spec for a constant zero reward."""
    return specs.Array(shape=(1,), dtype=float)

  def required_features_keys(self) -> set[str]:
    """Returns empty set.

    There are no feature keys that are required to compute the reward.
    """
    return set()
