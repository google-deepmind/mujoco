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
"""Functions to initialize, load, or save data."""

from jax import numpy as jp
from mujoco.mjx._src import collision_driver
from mujoco.mjx._src import constraint
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import Contact
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member
import numpy as np


def make_data(m: Model) -> Data:
  """Allocate and initialize Data."""

  # create first d to get num contacts and nc
  d = Data(
      solver_niter=jp.array(0, dtype=jp.int32),
      ne=0,
      nf=0,
      nl=0,
      nefc=0,
      ncon=0,
      time=jp.zeros((), dtype=jp.float32),
      qpos=m.qpos0,
      qvel=jp.zeros(m.nv, dtype=jp.float32),
      act=jp.zeros(m.na, dtype=jp.float32),
      qacc_warmstart=jp.zeros(m.nv, dtype=jp.float32),
      ctrl=jp.zeros(m.nu, dtype=jp.float32),
      qfrc_applied=jp.zeros(m.nv, dtype=jp.float32),
      xfrc_applied=jp.zeros((m.nbody, 6), dtype=jp.float32),
      eq_active=jp.zeros(m.neq, dtype=jp.int32),
      qacc=jp.zeros(m.nv, dtype=jp.float32),
      act_dot=jp.zeros(m.na, dtype=jp.float32),
      xpos=jp.zeros((m.nbody, 3), dtype=jp.float32),
      xquat=jp.zeros((m.nbody, 4), dtype=jp.float32),
      xmat=jp.zeros((m.nbody, 3, 3), dtype=jp.float32),
      xipos=jp.zeros((m.nbody, 3), dtype=jp.float32),
      ximat=jp.zeros((m.nbody, 3, 3), dtype=jp.float32),
      xanchor=jp.zeros((m.njnt, 3), dtype=jp.float32),
      xaxis=jp.zeros((m.njnt, 3), dtype=jp.float32),
      geom_xpos=jp.zeros((m.ngeom, 3), dtype=jp.float32),
      geom_xmat=jp.zeros((m.ngeom, 3, 3), dtype=jp.float32),
      site_xpos=jp.zeros((m.nsite, 3), dtype=jp.float32),
      site_xmat=jp.zeros((m.nsite, 3, 3), dtype=jp.float32),
      subtree_com=jp.zeros((m.nbody, 3), dtype=jp.float32),
      cdof=jp.zeros((m.nv, 6), dtype=jp.float32),
      cinert=jp.zeros((m.nbody, 10), dtype=jp.float32),
      actuator_length=jp.zeros(m.nu, dtype=jp.float32),
      actuator_moment=jp.zeros((m.nu, m.nv), dtype=jp.float32),
      crb=jp.zeros((m.nbody, 10), dtype=jp.float32),
      qM=jp.zeros(m.nM, dtype=jp.float32),
      qLD=jp.zeros(m.nM, dtype=jp.float32),
      qLDiagInv=jp.zeros(m.nv, dtype=jp.float32),
      qLDiagSqrtInv=jp.zeros(m.nv, dtype=jp.float32),
      contact=Contact.zero(),
      efc_J=jp.zeros((), dtype=jp.float32),
      efc_frictionloss=jp.zeros((), dtype=jp.float32),
      efc_D=jp.zeros((), dtype=jp.float32),
      actuator_velocity=jp.zeros(m.nu, dtype=jp.float32),
      cvel=jp.zeros((m.nbody, 6), dtype=jp.float32),
      cdof_dot=jp.zeros((m.nv, 6), dtype=jp.float32),
      qfrc_bias=jp.zeros(m.nv, dtype=jp.float32),
      qfrc_passive=jp.zeros(m.nv, dtype=jp.float32),
      efc_aref=jp.zeros((), dtype=jp.float32),
      actuator_force=jp.zeros(m.nu, dtype=jp.float32),
      qfrc_actuator=jp.zeros(m.nv, dtype=jp.float32),
      qfrc_smooth=jp.zeros(m.nv, dtype=jp.float32),
      qacc_smooth=jp.zeros(m.nv, dtype=jp.float32),
      qfrc_constraint=jp.zeros(m.nv, dtype=jp.float32),
      qfrc_inverse=jp.zeros(m.nv, dtype=jp.float32),
      efc_force=jp.zeros((), dtype=jp.float32),
  )

  # get contact data with correct shapes
  ncon = collision_driver.ncon(m)
  d = d.replace(contact=Contact.zero((ncon,)), ncon=ncon)
  d = d.tree_replace({'contact.dim': 3 * np.ones(ncon)})

  ne, nf, nl, nc = constraint.count_constraints(m, d)
  d = d.replace(ne=ne, nf=nf, nl=nl, nefc=ne + nf + nl + nc)
  ns = ne + nf + nl
  d = d.tree_replace({'contact.efc_address': np.arange(ns, ns + ncon * 4, 4)})
  d = d.replace(
      efc_J=jp.zeros((d.nefc, m.nv), dtype=jp.float32),
      efc_frictionloss=jp.zeros(d.nefc, dtype=jp.float32),
      efc_D=jp.zeros(d.nefc, dtype=jp.float32),
      efc_aref=jp.zeros(d.nefc, dtype=jp.float32),
      efc_force=jp.zeros(d.nefc, dtype=jp.float32),
  )

  return d
