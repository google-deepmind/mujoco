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
"""Observe all the produced features and measurements."""

import abc
from collections.abc import Mapping

from gdm_robotics.interfaces import types as gdmr_types


class FeaturesObserver(abc.ABC):
  """Observe all the produced features and measurements."""

  @property
  @abc.abstractmethod
  def name(self) -> str:
    """Returns a unique string identifier for this object."""

  @abc.abstractmethod
  def observe_features(
      self, features: Mapping[str, gdmr_types.ArrayType]
  ) -> None:
    """Observes all the features and measurements."""
