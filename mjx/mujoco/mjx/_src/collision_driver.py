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
"""Collide geometries."""

from typing import Callable, Dict, Optional, Sequence, Tuple, Union

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import collision_base
from mujoco.mjx._src import mesh
from mujoco.mjx._src import support
# pylint: disable=g-importing-member
from mujoco.mjx._src.collision_base import Candidate
from mujoco.mjx._src.collision_base import CandidateSet
from mujoco.mjx._src.collision_base import GeomInfo
from mujoco.mjx._src.collision_base import SolverParams
from mujoco.mjx._src.collision_convex import capsule_convex
from mujoco.mjx._src.collision_convex import convex_convex
from mujoco.mjx._src.collision_convex import plane_convex
from mujoco.mjx._src.collision_convex import sphere_convex
from mujoco.mjx._src.collision_primitive import capsule_capsule
from mujoco.mjx._src.collision_primitive import plane_capsule
from mujoco.mjx._src.collision_primitive import plane_sphere
from mujoco.mjx._src.collision_primitive import sphere_capsule
from mujoco.mjx._src.collision_primitive import sphere_sphere
from mujoco.mjx._src.types import Contact
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import GeomType
from mujoco.mjx._src.types import Model
# pylint: enable=g-importing-member

# pair-wise collision functions
_COLLISION_FUNC = {
    (GeomType.PLANE, GeomType.SPHERE): plane_sphere,
    (GeomType.PLANE, GeomType.CAPSULE): plane_capsule,
    (GeomType.PLANE, GeomType.BOX): plane_convex,
    (GeomType.PLANE, GeomType.MESH): plane_convex,
    (GeomType.SPHERE, GeomType.SPHERE): sphere_sphere,
    (GeomType.SPHERE, GeomType.CAPSULE): sphere_capsule,
    (GeomType.SPHERE, GeomType.BOX): sphere_convex,
    (GeomType.SPHERE, GeomType.MESH): sphere_convex,
    (GeomType.CAPSULE, GeomType.CAPSULE): capsule_capsule,
    (GeomType.CAPSULE, GeomType.BOX): capsule_convex,
    (GeomType.CAPSULE, GeomType.MESH): capsule_convex,
    (GeomType.BOX, GeomType.BOX): convex_convex,
    (GeomType.BOX, GeomType.MESH): convex_convex,
    (GeomType.MESH, GeomType.MESH): convex_convex,
}


def get_collision_fn(
    key: Tuple[Union[GeomType, mujoco.mjtGeom], Union[GeomType, mujoco.mjtGeom]]
) -> Optional[Callable[[GeomInfo, GeomInfo], collision_base.Contact]]:
  """Returns a collision function given a pair of geom types."""
  return _COLLISION_FUNC.get(key, None)


def _add_candidate(
    result: CandidateSet,
    m: Union[Model, mujoco.MjModel],
    g1: int,
    g2: int,
    ipair: int = -1,
):
  """Adds a candidate to test for collision."""
  t1, t2 = m.geom_type[g1], m.geom_type[g2]
  if t1 > t2:
    t1, t2, g1, g2 = t2, t1, g2, g1

  # MuJoCo does not collide planes with other planes or hfields
  if t1 == GeomType.PLANE and t2 == GeomType.PLANE:
    return
  if t1 == GeomType.PLANE and t2 == GeomType.HFIELD:
    return

  def mesh_key(i):
    convex_data = [[None] * m.ngeom] * 3
    if isinstance(m, Model):
      convex_data = [
          m.geom_convex_face,
          m.geom_convex_vert,
          m.geom_convex_edge_dir,
      ]
    elif isinstance(m, mujoco.MjModel):
      kwargs = mesh.get(m)
      convex_data = [
          kwargs['geom_convex_face'],
          kwargs['geom_convex_vert'],
          kwargs['geom_convex_edge_dir'],
      ]
    key = tuple((-1,) if v[i] is None else v[i].shape for v in convex_data)
    return key

  k1, k2 = mesh_key(g1), mesh_key(g2)

  candidates = {(c.geom1, c.geom2) for c in result.get((t1, t2, k1, k2), [])}
  if (g1, g2) in candidates:
    return

  if ipair > -1:
    candidate = Candidate(g1, g2, ipair, -1, m.pair_dim[ipair])
  elif m.geom_priority[g1] != m.geom_priority[g2]:
    gp = g1 if m.geom_priority[g1] > m.geom_priority[g2] else g2
    candidate = Candidate(g1, g2, -1, gp, m.geom_condim[gp])
  else:
    dim = max(m.geom_condim[g1], m.geom_condim[g2])
    candidate = Candidate(g1, g2, -1, -1, dim)

  result.setdefault((t1, t2, k1, k2), []).append(candidate)


def _pair_params(
    m: Model,
    candidates: Sequence[Candidate],
) -> SolverParams:
  """Gets solver params for pair geoms."""
  ipair = jp.array([c.ipair for c in candidates])
  friction = jp.clip(m.pair_friction[ipair], a_min=mujoco.mjMINMU)
  solref = m.pair_solref[ipair]
  solreffriction = m.pair_solreffriction[ipair]
  solimp = m.pair_solimp[ipair]
  margin = m.pair_margin[ipair]
  gap = m.pair_gap[ipair]

  return SolverParams(friction, solref, solreffriction, solimp, margin, gap)


def _priority_params(
    m: Model,
    candidates: Sequence[Candidate],
) -> SolverParams:
  """Gets solver params from priority geoms."""
  geomp = jp.array([c.geomp for c in candidates])
  friction = m.geom_friction[geomp][:, jp.array([0, 0, 1, 2, 2])]
  solref = m.geom_solref[geomp]
  solreffriction = jp.zeros(geomp.shape + (mujoco.mjNREF,))
  solimp = m.geom_solimp[geomp]
  g = jp.array([(c.geom1, c.geom2) for c in candidates])
  margin = jp.amax(m.geom_margin[g.T], axis=0)
  gap = jp.amax(m.geom_gap[g.T], axis=0)

  return SolverParams(friction, solref, solreffriction, solimp, margin, gap)


def _dynamic_params(
    m: Model,
    candidates: Sequence[Candidate],
) -> SolverParams:
  """Gets solver params for dynamic geoms."""
  g1 = jp.array([c.geom1 for c in candidates])
  g2 = jp.array([c.geom2 for c in candidates])

  friction = jp.maximum(m.geom_friction[g1], m.geom_friction[g2])
  # copy friction terms for the full geom pair
  friction = friction[:, jp.array([0, 0, 1, 2, 2])]

  minval = jp.array(mujoco.mjMINVAL)
  solmix1, solmix2 = m.geom_solmix[g1], m.geom_solmix[g2]
  mix = solmix1 / (solmix1 + solmix2)
  mix = jp.where((solmix1 < minval) & (solmix2 < minval), 0.5, mix)
  mix = jp.where((solmix1 < minval) & (solmix2 >= minval), 0.0, mix)
  mix_fn = jax.vmap(lambda a, b, m: m * a + (1 - m) * b)

  solref1, solref2 = m.geom_solref[g1], m.geom_solref[g2]
  solref = jp.minimum(solref1, solref2)
  s_mix = mix_fn(solref1, solref2, mix)
  solref = jp.where((solref1[0] > 0) & (solref2[0] > 0), s_mix, solref)
  solreffriction = jp.zeros(g1.shape + (mujoco.mjNREF,))
  solimp = mix_fn(m.geom_solimp[g1], m.geom_solimp[g2], mix)
  margin = jp.maximum(m.geom_margin[g1], m.geom_margin[g2])
  gap = jp.maximum(m.geom_gap[g1], m.geom_gap[g2])

  return SolverParams(friction, solref, solreffriction, solimp, margin, gap)


def _pair_info(
    m: Model, d: Data, geom1: Sequence[int], geom2: Sequence[int]
) -> Tuple[GeomInfo, GeomInfo, Sequence[Dict[str, Optional[int]]]]:
  """Returns geom pair info for calculating collision."""
  def mesh_info(geom):
    g = jp.array(geom)
    info = GeomInfo(
        g,
        d.geom_xpos[g],
        d.geom_xmat[g],
        m.geom_size[g],
    )
    in_axes = jax.tree_map(lambda x: 0, info)
    is_mesh = m.geom_convex_face[geom[0]] is not None
    if is_mesh:
      info = info.replace(
          face=jp.stack([m.geom_convex_face[i] for i in geom]),
          vert=jp.stack([m.geom_convex_vert[i] for i in geom]),
          edge=jp.stack([m.geom_convex_edge_dir[i] for i in geom]),
          facenorm=jp.stack([m.geom_convex_facenormal[i] for i in geom]),
          face_edge_normal=jp.stack(
              [m.geom_convex_face_edge_normal[i] for i in geom]
          ),
          face_edge=jp.stack([m.geom_convex_face_edge[i] for i in geom]),
      )
      in_axes = in_axes.replace(
          face=0,
          vert=0,
          edge=0,
          facenorm=0,
          face_edge=0,
          face_edge_normal=0,
      )
    return info, in_axes

  info1, in_axes1 = mesh_info(geom1)
  info2, in_axes2 = mesh_info(geom2)
  return info1, info2, [in_axes1, in_axes2]


def _body_pair_filter(
    m: Union[Model, mujoco.MjModel], b1: int, b2: int
) -> bool:
  """Filters body pairs for collision."""
  dsbl_filterparent = m.opt.disableflags & DisableBit.FILTERPARENT
  weld1 = m.body_weldid[b1]
  weld2 = m.body_weldid[b2]
  parent_weld1 = m.body_weldid[m.body_parentid[weld1]]
  parent_weld2 = m.body_weldid[m.body_parentid[weld2]]

  if weld1 == weld2:
    # filter out self-collisions
    return True

  if (
      not dsbl_filterparent
      and weld1 != 0
      and weld2 != 0
      and (weld1 == parent_weld2 or weld2 == parent_weld1)
  ):
    # filter out parent-child collisions
    return True

  return False


def _broadphase_enabled(
    geom_types: Tuple[GeomType, GeomType],
    n_pairs: int,
    max_pairs: int,
) -> bool:
  return (
      GeomType.PLANE not in geom_types
      and max_pairs > -1
      and n_pairs > max_pairs
  )


def _collide_geoms(
    m: Model,
    d: Data,
    geom_types: Tuple[GeomType, GeomType],
    candidates: Sequence[Candidate],
) -> Contact:
  """Collides a geom pair."""
  fn = get_collision_fn(geom_types)
  if not fn:
    return Contact.zero()

  # group sol params by different candidate types
  typ_cands = {}
  for c in candidates:
    typ = (c.ipair > -1, c.geomp > -1)
    typ_cands.setdefault(typ, []).append(c)

  geom1, geom2, params = [], [], []
  for (pair, priority), candidates in typ_cands.items():
    geom1.extend([c.geom1 for c in candidates])
    geom2.extend([c.geom2 for c in candidates])
    if pair:
      params.append(_pair_params(m, candidates))
    elif priority:
      params.append(_priority_params(m, candidates))
    else:
      params.append(_dynamic_params(m, candidates))

  params = jax.tree_map(lambda *x: jp.concatenate(x), *params)
  g1, g2, in_axes = _pair_info(m, d, geom1, geom2)

  # Run a crude version of broadphase.
  max_pairs = int(support.get_custom_numeric(m, 'max_geom_pairs'))
  run_broadphase = _broadphase_enabled(geom_types, len(geom1), max_pairs)
  n_pairs = max_pairs if run_broadphase else len(geom1)
  if run_broadphase:
    # broadphase over geom pairs, using bounding spheres
    size1 = jp.max(m.geom_size[g1.geom_id], axis=-1)
    size2 = jp.max(m.geom_size[g2.geom_id], axis=-1)
    dists = jax.vmap(jp.linalg.norm)(g2.pos - g1.pos) - (size1 + size2)
    _, idx = jax.lax.top_k(-dists, k=n_pairs)
    g1, g2, params = jax.tree_map(
        lambda x, idx=idx: x[idx, ...], (g1, g2, params)
    )

  # call contact function
  res = jax.vmap(fn, in_axes=in_axes)(g1, g2)
  dist, pos, frame = jax.tree_map(jp.concatenate, res)

  # repeat params by the number of contacts per geom pair
  geom1, geom2, params = jax.tree_map(
      lambda x: jp.repeat(x, fn.ncon, axis=0),  # pytype: disable=attribute-error
      (g1.geom_id, g2.geom_id, params),
  )

  con = Contact(
      dist=dist,
      pos=pos,
      frame=frame,
      includemargin=params.margin - params.gap,
      friction=params.friction,
      solref=params.solref,
      solreffriction=params.solreffriction,
      solimp=params.solimp,
      geom1=geom1,
      geom2=geom2,
  )
  return con


def collision_candidates(m: Union[Model, mujoco.MjModel]) -> CandidateSet:
  """Returns candidates for collision checking."""
  candidate_set = {}

  for ipair in range(m.npair):
    g1, g2 = m.pair_geom1[ipair], m.pair_geom2[ipair]
    _add_candidate(candidate_set, m, g1, g2, ipair)

  body_pairs = []
  exclude_signature = set(m.exclude_signature)
  geom_con = m.geom_contype | m.geom_conaffinity
  b_start = m.body_geomadr
  b_end = b_start + m.body_geomnum

  for b1 in range(m.nbody):
    if not geom_con[b_start[b1]:b_end[b1]].any():
      continue
    for b2 in range(b1, m.nbody):
      if not geom_con[b_start[b2]:b_end[b2]].any():
        continue
      signature = (b1 << 16) + (b2)
      if signature in exclude_signature:
        continue
      if _body_pair_filter(m, b1, b2):
        continue
      body_pairs.append((b1, b2))

  for b1, b2 in body_pairs:
    for g1 in range(b_start[b1], b_end[b1]):
      if not geom_con[g1]:
        continue
      for g2 in range(b_start[b2], b_end[b2]):
        if not geom_con[g2]:
          continue
        mask = m.geom_contype[g1] & m.geom_conaffinity[g2]
        mask |= m.geom_contype[g2] & m.geom_conaffinity[g1]
        if mask != 0:
          _add_candidate(candidate_set, m, g1, g2)

  return candidate_set


def ncon(m: Union[Model, mujoco.MjModel]) -> int:
  """Returns the number of contacts computed in MJX given a model."""
  if m.opt.disableflags & DisableBit.CONTACT:
    return 0

  candidates = collision_candidates(m)
  max_count = int(support.get_custom_numeric(m, 'max_contact_points'))
  max_pairs = int(support.get_custom_numeric(m, 'max_geom_pairs'))

  count = 0
  for k, v in candidates.items():
    fn = get_collision_fn(k[0:2])
    if fn is None:
      continue
    run_broadphase = _broadphase_enabled((k[0], k[1]), len(v), max_pairs)
    n_pair = max_pairs if run_broadphase else len(v)
    count += n_pair * fn.ncon  # pytype: disable=attribute-error

  return min(max_count, count) if max_count > -1 else count


def collision(m: Model, d: Data) -> Data:
  """Collides geometries."""
  if ncon(m) == 0:
    return d.replace(contact=Contact.zero())

  candidate_set = collision_candidates(m)

  contacts = []
  for key, candidates in candidate_set.items():
    geom_types = key[0:2]
    contacts.append(_collide_geoms(m, d, geom_types, candidates))

  if not contacts:
    raise RuntimeError('No contacts found.')

  contact = jax.tree_map(lambda *x: jp.concatenate(x), *contacts)

  max_contact_points = int(support.get_custom_numeric(m, 'max_contact_points'))
  if max_contact_points > -1 and contact.dist.shape[0] > max_contact_points:
    # get top-k contacts
    _, idx = jax.lax.top_k(-contact.dist, k=max_contact_points)
    contact = jax.tree_map(lambda x, idx=idx: jp.take(x, idx, axis=0), contact)

  return d.replace(contact=contact)
