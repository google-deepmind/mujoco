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
from mujoco.mjx.third_party.mujoco_warp._src.collision_hfield import hfield_filter
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import Geom
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import contact_params
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import geom
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import write_contact
from mujoco.mjx.third_party.mujoco_warp._src.math import make_frame
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
wp.set_module_options({"enable_backward": False})

MULTI_CONTACT_COUNT = 8
mat3c = wp.types.matrix(shape=(MULTI_CONTACT_COUNT, 3), dtype=float)
mat63 = wp.types.matrix(shape=(6, 3), dtype=float)

_CONVEX_COLLISION_PAIRS = [
  (GeomType.HFIELD, GeomType.SPHERE),
  (GeomType.HFIELD, GeomType.CAPSULE),
  (GeomType.HFIELD, GeomType.ELLIPSOID),
  (GeomType.HFIELD, GeomType.CYLINDER),
  (GeomType.HFIELD, GeomType.BOX),
  (GeomType.HFIELD, GeomType.MESH),
  (GeomType.SPHERE, GeomType.ELLIPSOID),
  (GeomType.SPHERE, GeomType.MESH),
  (GeomType.CAPSULE, GeomType.ELLIPSOID),
  (GeomType.CAPSULE, GeomType.CYLINDER),
  (GeomType.CAPSULE, GeomType.MESH),
  (GeomType.ELLIPSOID, GeomType.ELLIPSOID),
  (GeomType.ELLIPSOID, GeomType.CYLINDER),
  (GeomType.ELLIPSOID, GeomType.BOX),
  (GeomType.ELLIPSOID, GeomType.MESH),
  (GeomType.CYLINDER, GeomType.CYLINDER),
  (GeomType.CYLINDER, GeomType.BOX),
  (GeomType.CYLINDER, GeomType.MESH),
  (GeomType.BOX, GeomType.MESH),
  (GeomType.MESH, GeomType.MESH),
]


def _check_convex_collision_pairs():
  prev_idx = -1
  for pair in _CONVEX_COLLISION_PAIRS:
    idx = upper_trid_index(len(GeomType), pair[0].value, pair[1].value)
    if pair[1] < pair[0] or idx <= prev_idx:
      return False
    prev_idx = idx
  return True


assert _check_convex_collision_pairs(), "_CONVEX_COLLISION_PAIRS is in invalid order."


@cache_kernel
def ccd_kernel_builder(
  legacy_gjk: bool,
  geomtype1: int,
  geomtype2: int,
  ccd_iterations: int,
  epa_exact_neg_distance: bool,
  depth_extension: float,
  is_hfield: bool,
):
  @wp.func
  def eval_ccd_write_contact(
    # Model:
    opt_ccd_tolerance: wp.array(dtype=float),
    geom_type: wp.array(dtype=int),
    # Data in:
    naconmax_in: int,
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
    multiccd_polygon_in: wp.array2d(dtype=wp.vec3),
    multiccd_clipped_in: wp.array2d(dtype=wp.vec3),
    multiccd_pnormal_in: wp.array2d(dtype=wp.vec3),
    multiccd_pdist_in: wp.array2d(dtype=float),
    multiccd_idx1_in: wp.array2d(dtype=int),
    multiccd_idx2_in: wp.array2d(dtype=int),
    multiccd_n1_in: wp.array2d(dtype=wp.vec3),
    multiccd_n2_in: wp.array2d(dtype=wp.vec3),
    multiccd_endvert_in: wp.array2d(dtype=wp.vec3),
    multiccd_face1_in: wp.array2d(dtype=wp.vec3),
    multiccd_face2_in: wp.array2d(dtype=wp.vec3),
    # In:
    geom1: Geom,
    geom2: Geom,
    geoms: wp.vec2i,
    worldid: int,
    tid: int,
    margin: float,
    gap: float,
    condim: int,
    friction: vec5,
    solref: wp.vec2,
    solreffriction: wp.vec2,
    solimp: vec5,
    x1: wp.vec3,
    x2: wp.vec3,
    count: int,
    # Data out:
    nacon_out: wp.array(dtype=int),
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
  ) -> int:
    # TODO(kbayes): remove legacy GJK once multicontact can be enabled
    if wp.static(legacy_gjk):
      simplex, normal = gjk_legacy(
        ccd_iterations,
        geom1,
        geom2,
        geomtype1,
        geomtype2,
      )

      depth, normal = epa_legacy(
        ccd_iterations, geom1, geom2, geomtype1, geomtype2, depth_extension, epa_exact_neg_distance, simplex, normal
      )
      dist = -depth

      if dist >= 0.0 or depth < -depth_extension:
        return 0
      sphere = GeomType.SPHERE
      ellipsoid = GeomType.ELLIPSOID
      g1 = geoms[0]
      g2 = geoms[1]
      if geom_type[g1] == sphere or geom_type[g1] == ellipsoid or geom_type[g2] == sphere or geom_type[g2] == ellipsoid:
        ncontact, points = multicontact_legacy(geom1, geom2, geomtype1, geomtype2, depth_extension, depth, normal, 1, 2, 1.0e-5)
      else:
        ncontact, points = multicontact_legacy(geom1, geom2, geomtype1, geomtype2, depth_extension, depth, normal, 4, 8, 1.0e-1)
      frame = make_frame(normal)
    else:
      points = mat3c()
      geom1.margin = margin
      geom2.margin = margin
      dist, ncontact, witness1, witness2 = ccd(
        False,  # ignored for box-box, multiccd always on
        opt_ccd_tolerance[worldid],
        0.0,
        ccd_iterations,
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
        multiccd_polygon_in[tid],
        multiccd_clipped_in[tid],
        multiccd_pnormal_in[tid],
        multiccd_pdist_in[tid],
        multiccd_idx1_in[tid],
        multiccd_idx2_in[tid],
        multiccd_n1_in[tid],
        multiccd_n2_in[tid],
        multiccd_endvert_in[tid],
        multiccd_face1_in[tid],
        multiccd_face2_in[tid],
      )
      if dist >= 0.0:
        return 0

      for i in range(ncontact):
        points[i] = 0.5 * (witness1[i] + witness2[i])
      normal = witness1[0] - witness2[0]
      frame = make_frame(normal)

    for i in range(ncontact):
      write_contact(
        naconmax_in,
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
        nacon_out,
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
      if count + (i + 1) >= MJ_MAXCONPAIR:
        return i + 1

    return ncontact

  # runs convex collision on a set of geom pairs to recover contact info
  @nested_kernel(module="unique", enable_backward=False)
  def ccd_kernel(
    # Model:
    opt_ccd_tolerance: wp.array(dtype=float),
    geom_type: wp.array(dtype=int),
    geom_condim: wp.array(dtype=int),
    geom_dataid: wp.array(dtype=int),
    geom_priority: wp.array(dtype=int),
    geom_solmix: wp.array2d(dtype=float),
    geom_solref: wp.array2d(dtype=wp.vec2),
    geom_solimp: wp.array2d(dtype=vec5),
    geom_size: wp.array2d(dtype=wp.vec3),
    geom_aabb: wp.array2d(dtype=wp.vec3),
    geom_rbound: wp.array2d(dtype=float),
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
    # Data in:
    naconmax_in: int,
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    geom_xmat_in: wp.array2d(dtype=wp.mat33),
    collision_pair_in: wp.array(dtype=wp.vec2i),
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
    multiccd_polygon_in: wp.array2d(dtype=wp.vec3),
    multiccd_clipped_in: wp.array2d(dtype=wp.vec3),
    multiccd_pnormal_in: wp.array2d(dtype=wp.vec3),
    multiccd_pdist_in: wp.array2d(dtype=float),
    multiccd_idx1_in: wp.array2d(dtype=int),
    multiccd_idx2_in: wp.array2d(dtype=int),
    multiccd_n1_in: wp.array2d(dtype=wp.vec3),
    multiccd_n2_in: wp.array2d(dtype=wp.vec3),
    multiccd_endvert_in: wp.array2d(dtype=wp.vec3),
    multiccd_face1_in: wp.array2d(dtype=wp.vec3),
    multiccd_face2_in: wp.array2d(dtype=wp.vec3),
    # Data out:
    nacon_out: wp.array(dtype=int),
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

    # height field filter
    if wp.static(is_hfield):
      no_hf_collision, xmin, xmax, ymin, ymax, zmin, zmax = hfield_filter(
        geom_dataid, geom_aabb, geom_rbound, geom_margin, hfield_size, geom_xpos_in, geom_xmat_in, worldid, g1, g2
      )
      if no_hf_collision:
        return

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

    geom1_dataid = geom_dataid[g1]
    geom1 = geom(
      geomtype1,
      geom1_dataid,
      geom_size[worldid, g1],
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
      geom_xpos_in[worldid, g1],
      geom_xmat_in[worldid, g1],
    )

    geom2_dataid = geom_dataid[g2]
    geom2 = geom(
      geomtype2,
      geom2_dataid,
      geom_size[worldid, g2],
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
      geom_xpos_in[worldid, g2],
      geom_xmat_in[worldid, g2],
    )

    # see MuJoCo mjc_ConvexHField
    if wp.static(is_hfield):
      # height field subgrid
      nrow = hfield_nrow[g1]
      ncol = hfield_ncol[g1]
      size = hfield_size[g1]

      # subgrid
      x_scale = 0.5 * float(ncol - 1) / size[0]
      y_scale = 0.5 * float(nrow - 1) / size[1]
      cmin = wp.max(0, int(wp.floor((xmin + size[0]) * x_scale)))
      cmax = wp.min(ncol - 1, int(wp.ceil((xmax + size[0]) * x_scale)))
      rmin = wp.max(0, int(wp.floor((ymin + size[1]) * y_scale)))
      rmax = wp.min(nrow - 1, int(wp.ceil((ymax + size[1]) * y_scale)))

      dx = (2.0 * size[0]) / float(ncol - 1)
      dy = (2.0 * size[1]) / float(nrow - 1)
      dr = wp.vec2i(1, 0)

      prism = mat63()

      # set zbottom value using base size
      prism[0, 2] = -size[3]
      prism[1, 2] = -size[3]
      prism[2, 2] = -size[3]

      adr = hfield_adr[geom1_dataid]

      # process all prisms in subgrid
      count = int(0)
      for r in range(rmin, rmax):
        nvert = int(0)
        for c in range(cmin, cmax + 1):
          # add both triangles from this cell
          for i in range(2):
            # add vert
            x = dx * float(c) - size[0]
            y = dy * float(r + dr[i]) - size[1]
            z = hfield_data[adr + (r + dr[i]) * ncol + c] * size[2] + margin

            prism[0] = prism[1]
            prism[1] = prism[2]
            prism[3] = prism[4]
            prism[4] = prism[5]

            prism[2, 0] = x
            prism[5, 0] = x
            prism[2, 1] = y
            prism[5, 1] = y
            prism[5, 2] = z

            nvert += 1

            if nvert <= 2:
              continue

            # prism height test
            if prism[3, 2] < zmin and prism[4, 2] < zmin and prism[5, 2] < zmin:
              continue

            geom1.hfprism = prism

            # prism center
            x1 = geom1.pos
            if wp.static(not legacy_gjk):
              x1_ = wp.vec3(0.0, 0.0, 0.0)
              for i in range(6):
                x1_ += prism[i]
              x1 += geom1.rot @ (x1_ / 6.0)

            ncontact = eval_ccd_write_contact(
              opt_ccd_tolerance,
              geom_type,
              naconmax_in,
              epa_vert_in,
              epa_vert1_in,
              epa_vert2_in,
              epa_vert_index1_in,
              epa_vert_index2_in,
              epa_face_in,
              epa_pr_in,
              epa_norm2_in,
              epa_index_in,
              epa_map_in,
              epa_horizon_in,
              multiccd_polygon_in,
              multiccd_clipped_in,
              multiccd_pnormal_in,
              multiccd_pdist_in,
              multiccd_idx1_in,
              multiccd_idx2_in,
              multiccd_n1_in,
              multiccd_n2_in,
              multiccd_endvert_in,
              multiccd_face1_in,
              multiccd_face2_in,
              geom1,
              geom2,
              geoms,
              worldid,
              tid,
              margin,
              gap,
              condim,
              friction,
              solref,
              solreffriction,
              solimp,
              x1,
              geom2.pos,
              count,
              nacon_out,
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
            count += ncontact
            if count >= MJ_MAXCONPAIR:
              return
    else:
      eval_ccd_write_contact(
        opt_ccd_tolerance,
        geom_type,
        naconmax_in,
        epa_vert_in,
        epa_vert1_in,
        epa_vert2_in,
        epa_vert_index1_in,
        epa_vert_index2_in,
        epa_face_in,
        epa_pr_in,
        epa_norm2_in,
        epa_index_in,
        epa_map_in,
        epa_horizon_in,
        multiccd_polygon_in,
        multiccd_clipped_in,
        multiccd_pnormal_in,
        multiccd_pdist_in,
        multiccd_idx1_in,
        multiccd_idx2_in,
        multiccd_n1_in,
        multiccd_n2_in,
        multiccd_endvert_in,
        multiccd_face1_in,
        multiccd_face2_in,
        geom1,
        geom2,
        geoms,
        worldid,
        tid,
        margin,
        gap,
        condim,
        friction,
        solref,
        solreffriction,
        solimp,
        geom1.pos,
        geom2.pos,
        0,
        nacon_out,
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
    g1 = geom_pair[0].value
    g2 = geom_pair[1].value
    if m.geom_pair_type_count[upper_trid_index(len(GeomType), g1, g2)]:
      wp.launch(
        ccd_kernel_builder(m.opt.legacy_gjk, g1, g2, m.opt.ccd_iterations, True, 1e9, g1 == GeomType.HFIELD),
        dim=d.naconmax,
        inputs=[
          m.opt.ccd_tolerance,
          m.geom_type,
          m.geom_condim,
          m.geom_dataid,
          m.geom_priority,
          m.geom_solmix,
          m.geom_solref,
          m.geom_solimp,
          m.geom_size,
          m.geom_aabb,
          m.geom_rbound,
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
          d.naconmax,
          d.geom_xpos,
          d.geom_xmat,
          d.collision_pair,
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
          d.multiccd_polygon,
          d.multiccd_clipped,
          d.multiccd_pnormal,
          d.multiccd_pdist,
          d.multiccd_idx1,
          d.multiccd_idx2,
          d.multiccd_n1,
          d.multiccd_n2,
          d.multiccd_endvert,
          d.multiccd_face1,
          d.multiccd_face2,
        ],
        outputs=[
          d.nacon,
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
