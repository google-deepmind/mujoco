# Copyright 2025 The Newton Developers
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

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk import ccd
from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk_legacy import epa_legacy
from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk_legacy import gjk_legacy
from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk_legacy import multicontact_legacy
from mujoco.mjx.third_party.mujoco_warp._src.collision_hfield import hfield_prism_vertex
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import _geom
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import contact_params
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import write_contact
from mujoco.mjx.third_party.mujoco_warp._src.math import make_frame
from mujoco.mjx.third_party.mujoco_warp._src.math import upper_tri_index
from mujoco.mjx.third_party.mujoco_warp._src.math import upper_trid_index
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXCONPAIR
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import kernel as nested_kernel

# TODO(team): improve compile time to enable backward pass
wp.config.enable_backward = False

MULTI_CONTACT_COUNT = 8
mat3c = wp.types.matrix(shape=(MULTI_CONTACT_COUNT, 3), dtype=float)

_CONVEX_COLLISION_PAIRS = [
  (GeomType.HFIELD.value, GeomType.SPHERE.value),
  (GeomType.HFIELD.value, GeomType.CAPSULE.value),
  (GeomType.HFIELD.value, GeomType.ELLIPSOID.value),
  (GeomType.HFIELD.value, GeomType.CYLINDER.value),
  (GeomType.HFIELD.value, GeomType.BOX.value),
  (GeomType.HFIELD.value, GeomType.MESH.value),
  (GeomType.SPHERE.value, GeomType.ELLIPSOID.value),
  (GeomType.SPHERE.value, GeomType.MESH.value),
  (GeomType.CAPSULE.value, GeomType.ELLIPSOID.value),
  (GeomType.CAPSULE.value, GeomType.CYLINDER.value),
  (GeomType.CAPSULE.value, GeomType.MESH.value),
  (GeomType.ELLIPSOID.value, GeomType.ELLIPSOID.value),
  (GeomType.ELLIPSOID.value, GeomType.CYLINDER.value),
  (GeomType.ELLIPSOID.value, GeomType.BOX.value),
  (GeomType.ELLIPSOID.value, GeomType.MESH.value),
  (GeomType.CYLINDER.value, GeomType.CYLINDER.value),
  (GeomType.CYLINDER.value, GeomType.BOX.value),
  (GeomType.CYLINDER.value, GeomType.MESH.value),
  (GeomType.BOX.value, GeomType.MESH.value),
  (GeomType.MESH.value, GeomType.MESH.value),
]


def _check_convex_collision_pairs():
  prev_idx = -1
  for pair in _CONVEX_COLLISION_PAIRS:
    idx = upper_trid_index(len(GeomType), pair[0], pair[1])
    if pair[1] < pair[0] or idx <= prev_idx:
      return False
    prev_idx = idx
  return True


assert _check_convex_collision_pairs(), "_CONVEX_COLLISION_PAIRS is in invalid order."


@wp.func
def _max_contacts_height_field(
  # Model:
  ngeom: int,
  geom_type: wp.array(dtype=int),
  geompair2hfgeompair: wp.array(dtype=int),
  # In:
  g1: int,
  g2: int,
  worldid: int,
  # Data out:
  ncon_hfield_out: wp.array2d(dtype=int),
):
  hfield = int(GeomType.HFIELD.value)
  if geom_type[g1] == hfield or (geom_type[g2] == hfield):
    geompairid = upper_tri_index(ngeom, g1, g2)
    hfgeompairid = geompair2hfgeompair[geompairid]
    hfncon = wp.atomic_add(ncon_hfield_out[worldid], hfgeompairid, 1)
    if hfncon >= MJ_MAXCONPAIR:
      return True

  return False


@cache_kernel
def ccd_kernel_builder(
  legacy_gjk: bool,
  geomtype1: int,
  geomtype2: int,
  gjk_iterations: int,
  epa_iterations: int,
  epa_exact_neg_distance: bool,
  depth_extension: float,
):
  # runs convex collision on a set of geom pairs to recover contact info
  @nested_kernel
  def ccd_kernel(
    # Model:
    ngeom: int,
    geom_type: wp.array(dtype=int),
    geom_condim: wp.array(dtype=int),
    geom_dataid: wp.array(dtype=int),
    geom_priority: wp.array(dtype=int),
    geom_solmix: wp.array2d(dtype=float),
    geom_solref: wp.array2d(dtype=wp.vec2),
    geom_solimp: wp.array2d(dtype=vec5),
    geom_size: wp.array2d(dtype=wp.vec3),
    geom_friction: wp.array2d(dtype=wp.vec3),
    geom_margin: wp.array2d(dtype=float),
    geom_gap: wp.array2d(dtype=float),
    hfield_adr: wp.array(dtype=int),
    hfield_nrow: wp.array(dtype=int),
    hfield_ncol: wp.array(dtype=int),
    hfield_size: wp.array(dtype=wp.vec4),
    hfield_data: wp.array(dtype=float),
    mesh_vertadr: wp.array(dtype=int),
    mesh_vertnum: wp.array(dtype=int),
    mesh_vert: wp.array(dtype=wp.vec3),
    mesh_graphadr: wp.array(dtype=int),
    mesh_graph: wp.array(dtype=int),
    mesh_polynum: wp.array(dtype=int),
    mesh_polyadr: wp.array(dtype=int),
    mesh_polynormal: wp.array(dtype=wp.vec3),
    mesh_polyvertadr: wp.array(dtype=int),
    mesh_polyvertnum: wp.array(dtype=int),
    mesh_polyvert: wp.array(dtype=int),
    mesh_polymapadr: wp.array(dtype=int),
    mesh_polymapnum: wp.array(dtype=int),
    mesh_polymap: wp.array(dtype=int),
    pair_dim: wp.array(dtype=int),
    pair_solref: wp.array2d(dtype=wp.vec2),
    pair_solreffriction: wp.array2d(dtype=wp.vec2),
    pair_solimp: wp.array2d(dtype=vec5),
    pair_margin: wp.array2d(dtype=float),
    pair_gap: wp.array2d(dtype=float),
    pair_friction: wp.array2d(dtype=vec5),
    geompair2hfgeompair: wp.array(dtype=int),
    # Data in:
    nconmax_in: int,
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    geom_xmat_in: wp.array2d(dtype=wp.mat33),
    collision_pair_in: wp.array(dtype=wp.vec2i),
    collision_hftri_index_in: wp.array(dtype=int),
    collision_pairid_in: wp.array(dtype=int),
    collision_worldid_in: wp.array(dtype=int),
    ncollision_in: wp.array(dtype=int),
    epa_vert_in: wp.array2d(dtype=wp.vec3),
    epa_vert1_in: wp.array2d(dtype=wp.vec3),
    epa_vert2_in: wp.array2d(dtype=wp.vec3),
    epa_vert_index1_in: wp.array2d(dtype=int),
    epa_vert_index2_in: wp.array2d(dtype=int),
    epa_face_in: wp.array2d(dtype=wp.vec3i),
    epa_pr_in: wp.array2d(dtype=wp.vec3),
    epa_norm2_in: wp.array2d(dtype=float),
    epa_index_in: wp.array2d(dtype=int),
    epa_map_in: wp.array2d(dtype=int),
    epa_horizon_in: wp.array2d(dtype=int),
    # Data out:
    ncon_out: wp.array(dtype=int),
    ncon_hfield_out: wp.array2d(dtype=int),
    contact_dist_out: wp.array(dtype=float),
    contact_pos_out: wp.array(dtype=wp.vec3),
    contact_frame_out: wp.array(dtype=wp.mat33),
    contact_includemargin_out: wp.array(dtype=float),
    contact_friction_out: wp.array(dtype=vec5),
    contact_solref_out: wp.array(dtype=wp.vec2),
    contact_solreffriction_out: wp.array(dtype=wp.vec2),
    contact_solimp_out: wp.array(dtype=vec5),
    contact_dim_out: wp.array(dtype=int),
    contact_geom_out: wp.array(dtype=wp.vec2i),
    contact_worldid_out: wp.array(dtype=int),
  ):
    tid = wp.tid()
    if tid >= ncollision_in[0]:
      return

    geoms = collision_pair_in[tid]
    g1 = geoms[0]
    g2 = geoms[1]

    if geom_type[g1] != geomtype1 or geom_type[g2] != geomtype2:
      return

    worldid = collision_worldid_in[tid]
    _, margin, gap, condim, friction, solref, solreffriction, solimp = contact_params(
      geom_condim,
      geom_priority,
      geom_solmix,
      geom_solref,
      geom_solimp,
      geom_friction,
      geom_margin,
      geom_gap,
      pair_dim,
      pair_solref,
      pair_solreffriction,
      pair_solimp,
      pair_margin,
      pair_gap,
      pair_friction,
      collision_pair_in,
      collision_pairid_in,
      tid,
      worldid,
    )

    hftri_index = collision_hftri_index_in[tid]

    geom1 = _geom(
      geom_type,
      geom_dataid,
      geom_size,
      hfield_adr,
      hfield_nrow,
      hfield_ncol,
      hfield_size,
      hfield_data,
      mesh_vertadr,
      mesh_vertnum,
      mesh_vert,
      mesh_graphadr,
      mesh_graph,
      mesh_polynum,
      mesh_polyadr,
      mesh_polynormal,
      mesh_polyvertadr,
      mesh_polyvertnum,
      mesh_polyvert,
      mesh_polymapadr,
      mesh_polymapnum,
      mesh_polymap,
      geom_xpos_in,
      geom_xmat_in,
      worldid,
      g1,
      hftri_index,
    )

    geom2 = _geom(
      geom_type,
      geom_dataid,
      geom_size,
      hfield_adr,
      hfield_nrow,
      hfield_ncol,
      hfield_size,
      hfield_data,
      mesh_vertadr,
      mesh_vertnum,
      mesh_vert,
      mesh_graphadr,
      mesh_graph,
      mesh_polynum,
      mesh_polyadr,
      mesh_polynormal,
      mesh_polyvertadr,
      mesh_polyvertnum,
      mesh_polyvert,
      mesh_polymapadr,
      mesh_polymapnum,
      mesh_polymap,
      geom_xpos_in,
      geom_xmat_in,
      worldid,
      g2,
      hftri_index,
    )

    # TODO(kbayes): remove legacy GJK once multicontact can be enabled
    if wp.static(legacy_gjk):
      # find prism center for height field
      if geomtype1 == int(GeomType.HFIELD.value):
        x1 = wp.vec3(0.0, 0.0, 0.0)
        for i in range(6):
          x1 += hfield_prism_vertex(geom1.hfprism, i)
        x1 = geom1.pos + geom1.rot @ (x1 / 6.0)
        geom1.pos = x1

      simplex, normal = gjk_legacy(
        gjk_iterations,
        geom1,
        geom2,
        geomtype1,
        geomtype2,
      )

      depth, normal = epa_legacy(
        epa_iterations, geom1, geom2, geomtype1, geomtype2, depth_extension, epa_exact_neg_distance, simplex, normal
      )
      dist = -depth

      if dist >= 0.0 or depth < -depth_extension:
        count = 0
        return
      sphere = int(GeomType.SPHERE.value)
      ellipsoid = int(GeomType.ELLIPSOID.value)
      if geom_type[g1] == sphere or geom_type[g1] == ellipsoid or geom_type[g2] == sphere or geom_type[g2] == ellipsoid:
        count, points = multicontact_legacy(geom1, geom2, geomtype1, geomtype2, depth_extension, depth, normal, 1, 2, 1.0e-5)
      else:
        count, points = multicontact_legacy(geom1, geom2, geomtype1, geomtype2, depth_extension, depth, normal, 4, 8, 1.0e-1)
      frame = make_frame(normal)
    else:
      points = mat3c()

      x1 = geom1.pos
      x2 = geom2.pos

      # find prism center for height field
      if geomtype1 == int(GeomType.HFIELD.value):
        x1_ = wp.vec3(0.0, 0.0, 0.0)
        for i in range(6):
          x1_ += hfield_prism_vertex(geom1.hfprism, i)
        x1 += geom1.rot @ (x1_ / 6.0)

      dist, count, witness1, witness2 = ccd(
        False,
        1e-6,
        0.0,
        gjk_iterations,
        epa_iterations,
        geom1,
        geom2,
        geomtype1,
        geomtype2,
        x1,
        x2,
        epa_vert_in[tid],
        epa_vert1_in[tid],
        epa_vert2_in[tid],
        epa_vert_index1_in[tid],
        epa_vert_index2_in[tid],
        epa_face_in[tid],
        epa_pr_in[tid],
        epa_norm2_in[tid],
        epa_index_in[tid],
        epa_map_in[tid],
        epa_horizon_in[tid],
      )
      if dist >= 0.0:
        count = 0
        return

      for i in range(count):
        points[i] = 0.5 * (witness1[i] + witness2[i])
      normal = witness1[0] - witness2[0]
      frame = make_frame(normal)

    for i in range(count):
      # limit maximum number of contacts with height field
      if _max_contacts_height_field(ngeom, geom_type, geompair2hfgeompair, g1, g2, worldid, ncon_hfield_out):
        return

      write_contact(
        nconmax_in,
        dist,
        points[i],
        frame,
        margin,
        gap,
        condim,
        friction,
        solref,
        solreffriction,
        solimp,
        geoms,
        worldid,
        ncon_out,
        contact_dist_out,
        contact_pos_out,
        contact_frame_out,
        contact_includemargin_out,
        contact_friction_out,
        contact_solref_out,
        contact_solreffriction_out,
        contact_solimp_out,
        contact_dim_out,
        contact_geom_out,
        contact_worldid_out,
      )

  return ccd_kernel


@event_scope
def convex_narrowphase(m: Model, d: Data):
  """Runs narrowphase collision detection for convex geom pairs.

  This function handles collision detection for pairs of convex geometries that were
  identified during the broadphase. It uses the Gilbert-Johnson-Keerthi (GJK) algorithm to
  determine the distance between shapes and the Expanding Polytope Algorithm (EPA) to find
  the penetration depth and contact normal for colliding pairs.

  The convex geom types handled by this function are SPHERE, CAPSULE, ELLIPSOID, CYLINDER,
  BOX, MESH, HFIELD.

  To optimize performance, this function dynamically builds and launches a specialized
  kernel for each type of convex collision pair present in the model, avoiding unnecessary
  computations for non-existent pair types.
  """
  for geom_pair in _CONVEX_COLLISION_PAIRS:
    if m.geom_pair_type_count[upper_trid_index(len(GeomType), geom_pair[0], geom_pair[1])]:
      wp.launch(
        ccd_kernel_builder(
          m.opt.legacy_gjk,
          geom_pair[0],
          geom_pair[1],
          m.opt.gjk_iterations,
          m.opt.epa_iterations,
          True,
          1e9,
        ),
        dim=d.nconmax,
        inputs=[
          m.ngeom,
          m.geom_type,
          m.geom_condim,
          m.geom_dataid,
          m.geom_priority,
          m.geom_solmix,
          m.geom_solref,
          m.geom_solimp,
          m.geom_size,
          m.geom_friction,
          m.geom_margin,
          m.geom_gap,
          m.hfield_adr,
          m.hfield_nrow,
          m.hfield_ncol,
          m.hfield_size,
          m.hfield_data,
          m.mesh_vertadr,
          m.mesh_vertnum,
          m.mesh_vert,
          m.mesh_graphadr,
          m.mesh_graph,
          m.mesh_polynum,
          m.mesh_polyadr,
          m.mesh_polynormal,
          m.mesh_polyvertadr,
          m.mesh_polyvertnum,
          m.mesh_polyvert,
          m.mesh_polymapadr,
          m.mesh_polymapnum,
          m.mesh_polymap,
          m.pair_dim,
          m.pair_solref,
          m.pair_solreffriction,
          m.pair_solimp,
          m.pair_margin,
          m.pair_gap,
          m.pair_friction,
          m.geompair2hfgeompair,
          d.nconmax,
          d.geom_xpos,
          d.geom_xmat,
          d.collision_pair,
          d.collision_hftri_index,
          d.collision_pairid,
          d.collision_worldid,
          d.ncollision,
          d.epa_vert,
          d.epa_vert1,
          d.epa_vert2,
          d.epa_vert_index1,
          d.epa_vert_index2,
          d.epa_face,
          d.epa_pr,
          d.epa_norm2,
          d.epa_index,
          d.epa_map,
          d.epa_horizon,
        ],
        outputs=[
          d.ncon,
          d.ncon_hfield,
          d.contact.dist,
          d.contact.pos,
          d.contact.frame,
          d.contact.includemargin,
          d.contact.friction,
          d.contact.solref,
          d.contact.solreffriction,
          d.contact.solimp,
          d.contact.dim,
          d.contact.geom,
          d.contact.worldid,
        ],
      )
