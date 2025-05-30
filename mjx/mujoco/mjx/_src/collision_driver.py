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
"""Runs collision checking for all geoms in a Model.

To do this, collision_driver builds a collision function table, and then runs
the collision functions serially on the parameters in the table.

For example, if a Model has three geoms:

geom   |   type
---------------
1      | sphere
2      | capsule
3      | sphere

collision_driver organizes it into these functions and runs them:

function       | geom pair
--------------------------
sphere_sphere  | (1, 3)
sphere_capsule | (1, 2), (2, 3)


Besides collision function, function tables are keyed on mesh id and condim,
in order to guarantee static shapes for contacts and jacobians.
"""

import itertools
import os
from typing import Dict, Iterator, List, Tuple, Union

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import support
# pylint: disable=g-importing-member
from mujoco.mjx._src.collision_convex import box_box
from mujoco.mjx._src.collision_convex import capsule_convex
from mujoco.mjx._src.collision_convex import convex_convex
from mujoco.mjx._src.collision_convex import hfield_capsule
from mujoco.mjx._src.collision_convex import hfield_convex
from mujoco.mjx._src.collision_convex import hfield_sphere
from mujoco.mjx._src.collision_convex import plane_convex
from mujoco.mjx._src.collision_convex import sphere_convex
from mujoco.mjx._src.collision_primitive import capsule_capsule
from mujoco.mjx._src.collision_primitive import plane_capsule
from mujoco.mjx._src.collision_primitive import plane_cylinder
from mujoco.mjx._src.collision_primitive import plane_ellipsoid
from mujoco.mjx._src.collision_primitive import plane_sphere
from mujoco.mjx._src.collision_primitive import sphere_capsule
from mujoco.mjx._src.collision_primitive import sphere_sphere
from mujoco.mjx._src.collision_sdf import capsule_cylinder
from mujoco.mjx._src.collision_sdf import capsule_ellipsoid
from mujoco.mjx._src.collision_sdf import cylinder_cylinder
from mujoco.mjx._src.collision_sdf import ellipsoid_cylinder
from mujoco.mjx._src.collision_sdf import ellipsoid_ellipsoid
from mujoco.mjx._src.collision_sdf import sphere_cylinder
from mujoco.mjx._src.collision_sdf import sphere_ellipsoid
from mujoco.mjx._src.collision_types import FunctionKey
from mujoco.mjx._src.types import Contact
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DataJAX
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import GeomType
from mujoco.mjx._src.types import Model
from mujoco.mjx._src.types import ModelJAX
# pylint: enable=g-importing-member
import numpy as np

# pair-wise collision functions
_COLLISION_FUNC = {
    (GeomType.PLANE, GeomType.SPHERE): plane_sphere,
    (GeomType.PLANE, GeomType.CAPSULE): plane_capsule,
    (GeomType.PLANE, GeomType.BOX): plane_convex,
    (GeomType.PLANE, GeomType.ELLIPSOID): plane_ellipsoid,
    (GeomType.PLANE, GeomType.CYLINDER): plane_cylinder,
    (GeomType.PLANE, GeomType.MESH): plane_convex,
    (GeomType.HFIELD, GeomType.SPHERE): hfield_sphere,
    (GeomType.HFIELD, GeomType.CAPSULE): hfield_capsule,
    (GeomType.HFIELD, GeomType.BOX): hfield_convex,
    (GeomType.HFIELD, GeomType.MESH): hfield_convex,
    (GeomType.SPHERE, GeomType.SPHERE): sphere_sphere,
    (GeomType.SPHERE, GeomType.CAPSULE): sphere_capsule,
    (GeomType.SPHERE, GeomType.CYLINDER): sphere_cylinder,
    (GeomType.SPHERE, GeomType.ELLIPSOID): sphere_ellipsoid,
    (GeomType.SPHERE, GeomType.BOX): sphere_convex,
    (GeomType.SPHERE, GeomType.MESH): sphere_convex,
    (GeomType.CAPSULE, GeomType.CAPSULE): capsule_capsule,
    (GeomType.CAPSULE, GeomType.BOX): capsule_convex,
    (GeomType.CAPSULE, GeomType.ELLIPSOID): capsule_ellipsoid,
    (GeomType.CAPSULE, GeomType.CYLINDER): capsule_cylinder,
    (GeomType.CAPSULE, GeomType.MESH): capsule_convex,
    (GeomType.ELLIPSOID, GeomType.ELLIPSOID): ellipsoid_ellipsoid,
    (GeomType.ELLIPSOID, GeomType.CYLINDER): ellipsoid_cylinder,
    (GeomType.CYLINDER, GeomType.CYLINDER): cylinder_cylinder,
    (GeomType.BOX, GeomType.BOX): box_box,
    (GeomType.BOX, GeomType.MESH): convex_convex,
    (GeomType.MESH, GeomType.MESH): convex_convex,
}


# geoms for which we ignore broadphase
_GEOM_NO_BROADPHASE = {GeomType.HFIELD, GeomType.PLANE}


def has_collision_fn(t1: GeomType, t2: GeomType) -> bool:
  """Returns True if a collision function exists for a pair of geom types."""
  return (t1, t2) in _COLLISION_FUNC


def geom_pairs(
    m: Union[Model, mujoco.MjModel],
) -> Iterator[Tuple[int, int, int]]:
  """Yields geom pairs to check for collisions.

  Args:
    m: a MuJoCo or MJX model

  Yields:
    geom1, geom2, and pair index if defined in <pair> (else -1)
  """
  pairs = set()

  for i in range(m.npair):
    g1, g2 = m.pair_geom1[i], m.pair_geom2[i]
    # order pairs by geom_type for correct function mapping
    if m.geom_type[g1] > m.geom_type[g2]:
      g1, g2 = g2, g1
    pairs.add((g1, g2))
    yield g1, g2, i

  exclude_signature = set(m.exclude_signature)
  geom_con = m.geom_contype | m.geom_conaffinity
  filterparent = not (m.opt.disableflags & DisableBit.FILTERPARENT)
  b_start = m.body_geomadr
  b_end = b_start + m.body_geomnum

  for b1 in range(m.nbody):
    if not geom_con[b_start[b1] : b_end[b1]].any():
      continue
    w1 = m.body_weldid[b1]
    w1_p = m.body_weldid[m.body_parentid[w1]]

    for b2 in range(b1, m.nbody):
      if not geom_con[b_start[b2] : b_end[b2]].any():
        continue
      signature = (b1 << 16) + (b2)
      if signature in exclude_signature:
        continue
      w2 = m.body_weldid[b2]
      # ignore self-collisions
      if w1 == w2:
        continue
      w2_p = m.body_weldid[m.body_parentid[w2]]
      # ignore parent-child collisions
      if filterparent and w1 != 0 and w2 != 0 and (w1 == w2_p or w2 == w1_p):
        continue
      g1_range = [g for g in range(b_start[b1], b_end[b1]) if geom_con[g]]
      g2_range = [g for g in range(b_start[b2], b_end[b2]) if geom_con[g]]

      for g1, g2 in itertools.product(g1_range, g2_range):
        t1, t2 = m.geom_type[g1], m.geom_type[g2]
        # order pairs by geom_type for correct function mapping
        if t1 > t2:
          g1, g2, t1, t2 = g2, g1, t2, t1
        # ignore plane<>plane and plane<>hfield
        if (t1, t2) == (GeomType.PLANE, GeomType.PLANE):
          continue
        if (t1, t2) == (GeomType.PLANE, GeomType.HFIELD):
          continue
        # geoms must match contype and conaffinity on some bit
        mask = m.geom_contype[g1] & m.geom_conaffinity[g2]
        mask |= m.geom_contype[g2] & m.geom_conaffinity[g1]
        if not mask:
          continue

        if (g1, g2) not in pairs:
          pairs.add((g1, g2))
          yield g1, g2, -1


def _geom_groups(
    m: Union[Model, mujoco.MjModel],
) -> Dict[FunctionKey, List[Tuple[int, int, int]]]:
  """Returns geom pairs to check for collision grouped by collision function.

  The grouping consists of:
    - The collision function to run, which is determined by geom types
    - For mesh geoms, convex functions are run for each distinct mesh in the
      model, because the convex functions expect static mesh size. If a sphere
      collides with a cube and a tetrahedron, sphere_convex is called twice.
    - The condim of the collision. This ensures that the size of the resulting
      constraint jacobian is determined at compile time.

  Args:
    m: a MuJoCo or MJX model

  Returns:
    a dict with grouping key and values geom1, geom2, pair index
  """
  groups = {}

  for g1, g2, ip in geom_pairs(m):
    types = m.geom_type[g1], m.geom_type[g2]
    data_ids = m.geom_dataid[g1], m.geom_dataid[g2]
    if ip > -1:
      condim = m.pair_dim[ip]
    elif m.geom_priority[g1] > m.geom_priority[g2]:
      condim = m.geom_condim[g1]
    elif m.geom_priority[g1] < m.geom_priority[g2]:
      condim = m.geom_condim[g2]
    else:
      condim = max(m.geom_condim[g1], m.geom_condim[g2])

    key = FunctionKey(types, data_ids, condim)

    if types[0] == mujoco.mjtGeom.mjGEOM_HFIELD:
      # add static grid bounds to the grouping key for hfield collisions
      geom_rbound_hfield = (
          m._impl.geom_rbound_hfield if isinstance(m, Model) else m.geom_rbound  # pytype: disable=attribute-error
      )
      nrow, ncol = m.hfield_nrow[data_ids[0]], m.hfield_ncol[data_ids[0]]
      xsize, ysize = m.hfield_size[data_ids[0]][:2]
      xtick, ytick = (2 * xsize) / (ncol - 1), (2 * ysize) / (nrow - 1)
      xbound = int(np.ceil(2 * geom_rbound_hfield[g2] / xtick)) + 1
      xbound = min(xbound, ncol)
      ybound = int(np.ceil(2 * geom_rbound_hfield[g2] / ytick)) + 1
      ybound = min(ybound, nrow)
      key = FunctionKey(types, data_ids, condim, (xbound, ybound))

    groups.setdefault(key, []).append((g1, g2, ip))

  return groups


def _contact_groups(m: Model, d: Data) -> Dict[FunctionKey, Contact]:
  """Returns contact groups to check for collisions.

  Contacts are grouped the same way as _geom_groups.  Only one contact is
  emitted per geom pair, even if the collision function emits multiple contacts.

  Args:
    m: MJX model
    d: MJX data

  Returns:
    a dict where the key is the grouping and value is a Contact
  """
  groups = {}
  eps = mujoco.mjMINVAL

  for key, geom_ids in _geom_groups(m).items():
    geom = np.array(geom_ids)
    geom1, geom2, ip = geom.T
    geom1, geom2, ip = geom1[ip == -1], geom2[ip == -1], ip[ip != -1]
    params = []

    if ip.size > 0:
      # pair contacts get their params from m.pair_* fields
      params.append((
          m.pair_margin[ip] - m.pair_gap[ip],
          jp.clip(m.pair_friction[ip], a_min=eps),
          m.pair_solref[ip],
          m.pair_solreffriction[ip],
          m.pair_solimp[ip],
      ))
    if geom1.size > 0 and geom2.size > 0:
      # other contacts get their params from geom fields
      margin = jp.maximum(m.geom_margin[geom1], m.geom_margin[geom2])
      gap = jp.maximum(m.geom_gap[geom1], m.geom_gap[geom2])
      solmix1, solmix2 = m.geom_solmix[geom1], m.geom_solmix[geom2]
      mix = solmix1 / (solmix1 + solmix2)
      mix = jp.where((solmix1 < eps) & (solmix2 < eps), 0.5, mix)
      mix = jp.where((solmix1 < eps) & (solmix2 >= eps), 0.0, mix)
      mix = jp.where((solmix1 >= eps) & (solmix2 < eps), 1.0, mix)
      mix = mix[:, None]  # for correct broadcasting
      # friction: max
      friction = jp.maximum(m.geom_friction[geom1], m.geom_friction[geom2])
      solref1, solref2 = m.geom_solref[geom1], m.geom_solref[geom2]
      # reference standard: mix
      solref_standard = mix * solref1 + (1 - mix) * solref2
      # reference direct: min
      solref_direct = jp.minimum(solref1, solref2)
      is_standard = (solref1[:, [0, 0]] > 0) & (solref2[:, [0, 0]] > 0)
      solref = jp.where(is_standard, solref_standard, solref_direct)
      solreffriction = jp.zeros(geom1.shape + (mujoco.mjNREF,))
      # impedance: mix
      solimp = mix * m.geom_solimp[geom1] + (1 - mix) * m.geom_solimp[geom2]

      pri = m.geom_priority[geom1] != m.geom_priority[geom2]
      if pri.any():
        # use priority geom when specified instead of mixing
        gp1, gp2 = m.geom_priority[geom1], m.geom_priority[geom2]
        gp = np.where(gp1 > gp2, geom1, geom2)[pri]
        friction = friction.at[pri].set(m.geom_friction[gp])
        solref = solref.at[pri].set(m.geom_solref[gp])
        solimp = solimp.at[pri].set(m.geom_solimp[gp])

      # unpack 5d friction:
      friction = friction[:, [0, 0, 1, 2, 2]]
      params.append((margin - gap, friction, solref, solreffriction, solimp))

    params = map(jp.concatenate, zip(*params))
    includemargin, friction, solref, solreffriction, solimp = params

    groups[key] = Contact(
        # dist, pos, frame get filled in by collision functions:
        dist=None,
        pos=None,
        frame=None,
        includemargin=includemargin,
        friction=friction,
        solref=solref,
        solreffriction=solreffriction,
        solimp=solimp,
        dim=d._impl.contact.dim,  # pytype: disable=attribute-error
        geom1=jp.array(geom[:, 0]),
        geom2=jp.array(geom[:, 1]),
        geom=jp.array(geom[:, :2]),
        efc_address=d._impl.contact.efc_address,  # pytype: disable=attribute-error
    )

  return groups


def _numeric(m: Union[Model, mujoco.MjModel], name: str) -> int:
  id_ = support.name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, name)
  return int(m.numeric_data[id_]) if id_ >= 0 else -1


def make_condim(m: Union[Model, mujoco.MjModel]) -> np.ndarray:
  """Returns the dims of the contacts for a Model."""
  if m.opt.disableflags & DisableBit.CONTACT:
    return np.empty(0, dtype=int)

  group_counts = {k: len(v) for k, v in _geom_groups(m).items()}

  # max_geom_pairs limits the number of pairs we process in a collision function
  # by first running a primitive broad phase culling on the pairs
  max_geom_pairs = _numeric(m, 'max_geom_pairs')

  if max_geom_pairs > -1:
    for k in group_counts:
      if set(k.types) & _GEOM_NO_BROADPHASE:
        continue
      group_counts[k] = min(group_counts[k], max_geom_pairs)

  # max_contact_points limits the number of contacts emitted by selecting the
  # contacts with the most penetration after calling collision functions
  max_contact_points = _numeric(m, 'max_contact_points')

  condim_counts = {}
  for k, v in group_counts.items():
    if k.types[1] == mujoco.mjtGeom.mjGEOM_SDF:
      ncon = m.opt.sdf_initpoints
    else:
      func = _COLLISION_FUNC[k.types]
      ncon = func.ncon  # pytype: disable=attribute-error
    num_contacts = condim_counts.get(k.condim, 0) + ncon * v
    if max_contact_points > -1:
      num_contacts = min(max_contact_points, num_contacts)
    condim_counts[k.condim] = num_contacts

  dims = sum(([c] * condim_counts[c] for c in sorted(condim_counts)), [])

  return np.array(dims)


def collision(m: Model, d: Data) -> Data:
  """Collides geometries."""
  if not isinstance(m._impl, ModelJAX) or not isinstance(d._impl, DataJAX):
    raise ValueError('collision requires JAX backend implementation.')

  if d._impl.ncon == 0:  # pytype: disable=attribute-error
    return d

  max_geom_pairs = _numeric(m, 'max_geom_pairs')
  max_contact_points = _numeric(m, 'max_contact_points')

  # run collision functions on groups
  groups = _contact_groups(m, d)
  for key, contact in groups.items():
    # determine which contacts we'll use for collision testing by running a
    # broad phase cull if requested
    if (
        max_geom_pairs > -1
        and contact.geom.shape[0] > max_geom_pairs
        and not set(key.types) & _GEOM_NO_BROADPHASE
    ):
      pos1, pos2 = d.geom_xpos[contact.geom.T]
      size1, size2 = m.geom_rbound[contact.geom.T]
      dist = jax.vmap(jp.linalg.norm)(pos2 - pos1) - (size1 + size2)
      _, idx = jax.lax.top_k(-dist, k=max_geom_pairs)
      contact = jax.tree_util.tree_map(lambda x, idx=idx: x[idx], contact)

    # run the collision function specified by the grouping key
    func = _COLLISION_FUNC[key.types]
    ncon = func.ncon  # pytype: disable=attribute-error

    dist, pos, frame = func(m, d, key, contact.geom)
    if ncon > 1:
      # repeat contacts to match the number of collisions returned
      repeat_fn = lambda x, r=ncon: jp.repeat(x, r, axis=0)
      contact = jax.tree_util.tree_map(repeat_fn, contact)
    groups[key] = contact.replace(dist=dist, pos=pos, frame=frame)

  # collapse contacts together, ensuring they are grouped by condim
  condim_groups = {}
  for key, contact in groups.items():
    condim_groups.setdefault(key.condim, []).append(contact)

  # limit the number of contacts per condim group if requested
  if max_contact_points > -1:
    for key, contacts in condim_groups.items():
      contact = jax.tree_util.tree_map(lambda *x: jp.concatenate(x), *contacts)
      if contact.geom.shape[0] > max_contact_points:
        _, idx = jax.lax.top_k(-contact.dist, k=max_contact_points)
        contact = jax.tree_util.tree_map(lambda x, idx=idx: x[idx], contact)
      condim_groups[key] = [contact]

  contacts = sum([condim_groups[k] for k in sorted(condim_groups)], [])
  contact = jax.tree_util.tree_map(lambda *x: jp.concatenate(x), *contacts)

  return d.tree_replace({'_impl.contact': contact})
