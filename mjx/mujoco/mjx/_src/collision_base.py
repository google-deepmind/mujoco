# Copyright 2023 DeepMind Technologies Limited
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
"""Collision base."""

import dataclasses
from typing import Dict, List, Optional, Tuple

import jax
# pylint: disable=g-importing-member
from mujoco.mjx._src.dataclasses import PyTreeNode
from mujoco.mjx._src.types import GeomType
# pylint: enable=g-importing-member

Contact = Tuple[jax.Array, jax.Array, jax.Array]


@dataclasses.dataclass
class Candidate:
  geom1: int
  geom2: int
  ipair: int
  geomp: int  # priority geom
  dim: int


CandidateSet = Dict[
    Tuple[GeomType, GeomType, Tuple[int, ...], Tuple[int, ...]],
    List[Candidate],
]


class GeomInfo(PyTreeNode):
  """Collision info for a geom."""

  pos: jax.Array
  mat: jax.Array
  size: jax.Array
  face: Optional[jax.Array] = None
  vert: Optional[jax.Array] = None
  edge: Optional[jax.Array] = None
  facenorm: Optional[jax.Array] = None


class SolverParams(PyTreeNode):
  """Contact solver params."""

  friction: jax.Array
  solref: jax.Array
  solreffriction: jax.Array
  solimp: jax.Array
  margin: jax.Array
  gap: jax.Array
