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

from typing import Tuple

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk import ccd
from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk import multicontact
from mujoco.mjx.third_party.mujoco_warp._src.collision_gjk import support
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import Geom
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import contact_params
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import geom_collision_pair
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import write_contact
from mujoco.mjx.third_party.mujoco_warp._src.math import make_frame
from mujoco.mjx.third_party.mujoco_warp._src.math import upper_trid_index
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAX_EPAFACES
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAX_EPAHORIZON
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXCONPAIR
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import CollisionContext
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import EnableBit
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import mat43
from mujoco.mjx.third_party.mujoco_warp._src.types import mat63
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import cache_kernel
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

# TODO(team): improve compile time to enable backward pass
wp.set_module_options({"enable_backward": False})

vec_maxconpair = wp.types.vector(length=MJ_MAXCONPAIR, dtype=float)
mat_maxconpair = wp.types.matrix(shape=(MJ_MAXCONPAIR, 3), dtype=float)


@wp.func
def _hfield_filter(
  # Model:
  geom_type: wp.array(dtype=int),
  geom_dataid: wp.array(dtype=int),
  geom_size: wp.array2d(dtype=wp.vec3),
  geom_rbound: wp.array2d(dtype=float),
  geom_margin: wp.array2d(dtype=float),
  mesh_vertadr: wp.array(dtype=int),
  mesh_vertnum: wp.array(dtype=int),
  mesh_graphadr: wp.array(dtype=int),
  mesh_vert: wp.array(dtype=wp.vec3),
  mesh_graph: wp.array(dtype=int),
  hfield_size: wp.array(dtype=wp.vec4),
  # Data in:
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  # In:
  worldid: int,
  g1: int,
  g2: int,
) -> Tuple[bool, float, float, float, float, float, float]:
  """Filter for height field collisions.

  See MuJoCo mjc_ConvexHField.
  """
  # height field info
  hfdataid = geom_dataid[g1]
  size1 = hfield_size[hfdataid]

  # geom info
  rbound_id = worldid % geom_rbound.shape[0]
  margin_id = worldid % geom_margin.shape[0]
  size_id = worldid % geom_size.shape[0]

  pos1 = geom_xpos_in[worldid, g1]
  mat1 = geom_xmat_in[worldid, g1]
  mat1T = wp.transpose(mat1)
  pos2 = geom_xpos_in[worldid, g2]
  pos = mat1T @ (pos2 - pos1)
  r2 = geom_rbound[rbound_id, g2]

  # TODO(team): margin?
  margin = geom_margin[margin_id, g1] + geom_margin[margin_id, g2]

  # box-sphere test: horizontal plane
  for i in range(2):
    if (size1[i] < pos[i] - r2 - margin) or (-size1[i] > pos[i] + r2 + margin):
      return True, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL

  # box-sphere test: vertical direction
  if size1[2] < pos[2] - r2 - margin:  # up
    return True, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL

  if -size1[3] > pos[2] + r2 + margin:  # down
    return True, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL

  mat2 = geom_xmat_in[worldid, g2]
  mat = mat1T @ mat2

  # create geom in height field frame for support function queries
  geom2 = Geom()
  geom2.pos = pos
  geom2.rot = mat
  geom2.size = geom_size[size_id, g2]
  geom2.margin = 0.0  # margin handled separately
  geom2.index = -1

  geomtype2 = geom_type[g2]

  # load mesh vertex data for support function queries
  if geomtype2 == GeomType.MESH:
    dataid = geom_dataid[g2]
    geom2.vertadr = wp.where(dataid >= 0, mesh_vertadr[dataid], -1)
    geom2.vertnum = wp.where(dataid >= 0, mesh_vertnum[dataid], -1)
    geom2.graphadr = wp.where(dataid >= 0, mesh_graphadr[dataid], -1)
    geom2.vert = mesh_vert
    geom2.graph = mesh_graph

  # use support functions for tight AABB bounds
  xmax = support(geom2, geomtype2, wp.vec3(1.0, 0.0, 0.0)).point[0]
  xmin = support(geom2, geomtype2, wp.vec3(-1.0, 0.0, 0.0)).point[0]
  ymax = support(geom2, geomtype2, wp.vec3(0.0, 1.0, 0.0)).point[1]
  ymin = support(geom2, geomtype2, wp.vec3(0.0, -1.0, 0.0)).point[1]
  zmax = support(geom2, geomtype2, wp.vec3(0.0, 0.0, 1.0)).point[2]
  zmin = support(geom2, geomtype2, wp.vec3(0.0, 0.0, -1.0)).point[2]

  # box-box test
  if (
    (xmin - margin > size1[0])
    or (xmax + margin < -size1[0])
    or (ymin - margin > size1[1])
    or (ymax + margin < -size1[1])
    or (zmin - margin > size1[2])
    or (zmax + margin < -size1[3])
  ):
    return True, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL
  else:
    return False, xmin, xmax, ymin, ymax, zmin, zmax


@cache_kernel
def ccd_hfield_kernel_builder(
  geomtype1: int,
  geomtype2: int,
  gjk_iterations: int,
  epa_iterations: int,
  geomgeomid: int,
):
  """Kernel builder for heightfield CCD collisions (no multiccd args)."""

  # runs convex collision on a set of geom pairs to recover contact info
  @wp.kernel(module="unique", enable_backward=False)
  def ccd_hfield_kernel(
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
    geom_rbound: wp.array2d(dtype=float),
    geom_friction: wp.array2d(dtype=wp.vec3),
    geom_margin: wp.array2d(dtype=float),
    geom_gap: wp.array2d(dtype=float),
    mesh_vertadr: wp.array(dtype=int),
    mesh_vertnum: wp.array(dtype=int),
    mesh_graphadr: wp.array(dtype=int),
    mesh_vert: wp.array(dtype=wp.vec3),
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
    hfield_size: wp.array(dtype=wp.vec4),
    hfield_nrow: wp.array(dtype=int),
    hfield_ncol: wp.array(dtype=int),
    hfield_adr: wp.array(dtype=int),
    hfield_data: wp.array(dtype=float),
    pair_dim: wp.array(dtype=int),
    pair_solref: wp.array2d(dtype=wp.vec2),
    pair_solreffriction: wp.array2d(dtype=wp.vec2),
    pair_solimp: wp.array2d(dtype=vec5),
    pair_margin: wp.array2d(dtype=float),
    pair_gap: wp.array2d(dtype=float),
    pair_friction: wp.array2d(dtype=vec5),
    # Data in:
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    geom_xmat_in: wp.array2d(dtype=wp.mat33),
    naconmax_in: int,
    naccdmax_in: int,
    ncollision_in: wp.array(dtype=int),
    # In:
    collision_pair_in: wp.array(dtype=wp.vec2i),
    collision_pairid_in: wp.array(dtype=wp.vec2i),
    collision_worldid_in: wp.array(dtype=int),
    epa_vert_in: wp.array2d(dtype=wp.vec3),
    epa_vert_index_in: wp.array2d(dtype=int),
    epa_face_in: wp.array2d(dtype=int),
    epa_pr_in: wp.array2d(dtype=wp.vec3),
    epa_norm2_in: wp.array2d(dtype=float),
    epa_horizon_in: wp.array2d(dtype=int),
    nccd_in: wp.array(dtype=int),
    # Data out:
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
    contact_type_out: wp.array(dtype=int),
    contact_geomcollisionid_out: wp.array(dtype=int),
    nacon_out: wp.array(dtype=int),
  ):
    collisionid = wp.tid()
    if collisionid >= ncollision_in[0]:
      return

    geoms = collision_pair_in[collisionid]
    g1 = geoms[0]
    g2 = geoms[1]

    if geom_type[g1] != geomtype1 or geom_type[g2] != geomtype2:
      return

    worldid = collision_worldid_in[collisionid]

    # height field filter
    no_hf_collision, xmin, xmax, ymin, ymax, zmin, zmax = _hfield_filter(
      geom_type,
      geom_dataid,
      geom_size,
      geom_rbound,
      geom_margin,
      mesh_vertadr,
      mesh_vertnum,
      mesh_graphadr,
      mesh_vert,
      mesh_graph,
      hfield_size,
      geom_xpos_in,
      geom_xmat_in,
      worldid,
      g1,
      g2,
    )
    if no_hf_collision:
      return

    ccdid = wp.atomic_add(nccd_in, wp.static(geomgeomid), 1)
    if ccdid >= naccdmax_in:
      wp.printf("CCD overflow - please increase naccdmax to %u\n", ccdid)
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
      collisionid,
      worldid,
    )

    geom1, geom2 = geom_collision_pair(
      geom_type,
      geom_dataid,
      geom_size,
      mesh_vertadr,
      mesh_vertnum,
      mesh_graphadr,
      mesh_vert,
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
      geoms,
      worldid,
    )

    # transform geom2 into heightfield frame
    hf_pos = geom_xpos_in[worldid, g1]
    hf_mat = geom_xmat_in[worldid, g1]
    hf_matT = wp.transpose(hf_mat)

    geom2.pos = hf_matT @ (geom2.pos - hf_pos)
    geom2.rot = hf_matT @ geom2.rot

    # geom1 has identity pose
    geom1.pos = wp.vec3(0.0, 0.0, 0.0)
    geom1.rot = wp.identity(n=3, dtype=float)

    # see MuJoCo mjc_ConvexHField
    geom1_dataid = geom_dataid[g1]

    # height field subgrid
    nrow = hfield_nrow[geom1_dataid]
    ncol = hfield_ncol[geom1_dataid]
    size = hfield_size[geom1_dataid]

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

    hfield_contact_dist = vec_maxconpair()
    hfield_contact_pos = mat_maxconpair()
    hfield_contact_normal = mat_maxconpair()
    min_dist = float(MJ_MAXVAL)
    min_normal = wp.vec3(MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL)
    min_pos = wp.vec3(MJ_MAXVAL, MJ_MAXVAL, MJ_MAXVAL)
    min_id = int(-1)

    # geom1 margin added to z-axis of hfield prism top points below
    geom2.margin = margin

    # EPA memory
    epa_vert = epa_vert_in[ccdid]
    epa_vert_index = epa_vert_index_in[ccdid]
    epa_face = epa_face_in[ccdid]
    epa_pr = epa_pr_in[ccdid]
    epa_norm2 = epa_norm2_in[ccdid]
    epa_horizon = epa_horizon_in[ccdid]

    collision_pairid = collision_pairid_in[collisionid]

    # process all prisms in subgrid
    count = int(0)
    for r in range(rmin, rmax):
      # pre-initialize first 2 vertices
      for init_i in range(2):
        x = dx * float(cmin) - size[0]
        y = dy * float(r + dr[init_i]) - size[1]
        z = hfield_data[adr + (r + dr[init_i]) * ncol + cmin] * size[2] + margin

        prism[0] = prism[1]
        prism[1] = prism[2]
        prism[3] = prism[4]
        prism[4] = prism[5]

        prism[2, 0] = x
        prism[5, 0] = x
        prism[2, 1] = y
        prism[5, 1] = y
        prism[5, 2] = z

      for c in range(cmin + 1, cmax + 1):
        # add both triangles from this cell
        for i in range(2):
          if count >= MJ_MAXCONPAIR:
            wp.printf(
              "height field collision overflow, number of collisions >= %u - please adjust resolution: \n decrease the number of hfield rows/cols or modify size of colliding geom\n",
              MJ_MAXCONPAIR,
            )
            continue

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

          # prism height test
          if prism[3, 2] < zmin and prism[4, 2] < zmin and prism[5, 2] < zmin:
            continue

          geom1.hfprism = prism

          # prism center
          x1 = geom1.pos + geom1.rot @ (prism[0] + prism[1] + prism[2] + prism[3] + prism[4] + prism[5]) * wp.static(1.0 / 6.0)

          dist, ncontact, w1, w2, idx = ccd(
            opt_ccd_tolerance[worldid % opt_ccd_tolerance.shape[0]],
            0.0,
            gjk_iterations,
            epa_iterations,
            geom1,
            geom2,
            geomtype1,
            geomtype2,
            x1,
            geom2.pos,
            epa_vert,
            epa_vert_index,
            epa_face,
            epa_pr,
            epa_norm2,
            epa_horizon,
          )

          if ncontact == 0:
            continue

          # cache contact information
          hfield_contact_dist[count] = dist

          # transform contact to global frame
          pos_local = 0.5 * (w1 + w2)
          pos = hf_mat @ pos_local + hf_pos
          hfield_contact_pos[count, 0] = pos[0]
          hfield_contact_pos[count, 1] = pos[1]
          hfield_contact_pos[count, 2] = pos[2]

          frame_local = make_frame(w1 - w2)
          normal_local = wp.vec3(frame_local[0, 0], frame_local[0, 1], frame_local[0, 2])
          normal = hf_mat @ normal_local
          hfield_contact_normal[count, 0] = normal[0]
          hfield_contact_normal[count, 1] = normal[1]
          hfield_contact_normal[count, 2] = normal[2]

          # contact with minimum distance
          if dist < min_dist:
            min_dist = dist
            min_normal = normal
            min_pos = pos
            min_id = count

          count += 1

    # contact 0: minimum distance
    write_contact(
      naconmax_in,
      0,
      min_dist,
      min_pos,
      make_frame(min_normal),
      margin,
      gap,
      condim,
      friction,
      solref,
      solreffriction,
      solimp,
      geoms,
      collision_pairid,
      worldid,
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
      contact_type_out,
      contact_geomcollisionid_out,
      nacon_out,
    )

    # TODO(team): routine for select subset of contacts
    if wp.static(True):
      MIN_DIST_TO_NEXT_CONTACT = 1.0e-3

      # contact 1: furthest from minimum distance contact
      id1 = int(-1)
      dist1 = float(-MJ_MAXVAL)
      for i in range(count):
        if i == min_id:
          continue

        hf_pos = wp.vec3(hfield_contact_pos[i, 0], hfield_contact_pos[i, 1], hfield_contact_pos[i, 2])
        dist = wp.norm_l2(hf_pos - min_pos)

        if dist > dist1:
          id1 = i
          dist1 = dist

      if id1 == -1 or (0.0 < dist1 and dist1 < MIN_DIST_TO_NEXT_CONTACT):
        return

      pos1 = wp.vec3(hfield_contact_pos[id1, 0], hfield_contact_pos[id1, 1], hfield_contact_pos[id1, 2])
      normal1 = wp.vec3(hfield_contact_normal[id1, 0], hfield_contact_normal[id1, 1], hfield_contact_normal[id1, 2])

      write_contact(
        naconmax_in,
        1,
        hfield_contact_dist[id1],
        pos1,
        make_frame(normal1),
        margin,
        gap,
        condim,
        friction,
        solref,
        solreffriction,
        solimp,
        geoms,
        collision_pairid,
        worldid,
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
        contact_type_out,
        contact_geomcollisionid_out,
        nacon_out,
      )

      # contact 2: point furthest from min_pos - pos1 line
      dist_min1 = wp.cross(min_normal, min_pos - pos1)

      id2 = int(-1)
      dist_12 = float(-MJ_MAXVAL)
      for i in range(count):
        if i == min_id or i == id1:
          continue

        hf_pos = wp.vec3(hfield_contact_pos[i, 0], hfield_contact_pos[i, 1], hfield_contact_pos[i, 2])
        dist = wp.abs(wp.dot(hf_pos - min_pos, dist_min1))

        if dist > dist_12:
          id2 = i
          dist_12 = dist

      if id2 == -1 or (0.0 < dist_12 and dist_12 < MIN_DIST_TO_NEXT_CONTACT):
        return

      pos2 = wp.vec3(hfield_contact_pos[id2, 0], hfield_contact_pos[id2, 1], hfield_contact_pos[id2, 2])
      normal2 = wp.vec3(hfield_contact_normal[id2, 0], hfield_contact_normal[id2, 1], hfield_contact_normal[id2, 2])

      write_contact(
        naconmax_in,
        2,
        hfield_contact_dist[id2],
        pos2,
        make_frame(normal2),
        margin,
        gap,
        condim,
        friction,
        solref,
        solreffriction,
        solimp,
        geoms,
        collision_pairid,
        worldid,
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
        contact_type_out,
        contact_geomcollisionid_out,
        nacon_out,
      )

      # contact 3: point furthest from other triangle edge
      vec_min2 = wp.cross(min_normal, min_pos - pos2)
      vec_12 = wp.cross(min_normal, pos1 - pos2)

      id3 = int(-1)
      dist3 = float(-MJ_MAXVAL)
      for i in range(count):
        if i == min_id or i == id1 or i == id2:
          continue

        hf_pos = wp.vec3(hfield_contact_pos[i, 0], hfield_contact_pos[i, 1], hfield_contact_pos[i, 2])
        dist = wp.abs(wp.dot(hf_pos - min_pos, vec_min2)) + wp.abs(wp.dot(pos1 - hf_pos, vec_12))

        if dist > dist3:
          id3 = i
          dist3 = dist

      if id3 == -1 or (0.0 < dist3 and dist3 < MIN_DIST_TO_NEXT_CONTACT):
        return

      pos3 = wp.vec3(hfield_contact_pos[id3, 0], hfield_contact_pos[id3, 1], hfield_contact_pos[id3, 2])
      normal3 = wp.vec3(hfield_contact_normal[id3, 0], hfield_contact_normal[id3, 1], hfield_contact_normal[id3, 2])

      write_contact(
        naconmax_in,
        3,
        hfield_contact_dist[id3],
        pos3,
        make_frame(normal3),
        margin,
        gap,
        condim,
        friction,
        solref,
        solreffriction,
        solimp,
        geoms,
        collision_pairid,
        worldid,
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
        contact_type_out,
        contact_geomcollisionid_out,
        nacon_out,
      )

  return ccd_hfield_kernel


@cache_kernel
def ccd_kernel_builder(
  geomtype1: int,
  geomtype2: int,
  gjk_iterations: int,
  epa_iterations: int,
  use_multiccd: bool,
  geomgeomid: int,
):
  """Kernel builder for non-heightfield CCD collisions (no hfield args)."""

  @wp.func
  def eval_ccd_write_contact(
    # Model:
    opt_ccd_tolerance: wp.array(dtype=float),
    # Data in:
    naconmax_in: int,
    # In:
    epa_vert_in: wp.array2d(dtype=wp.vec3),
    epa_vert_index_in: wp.array2d(dtype=int),
    epa_face_in: wp.array2d(dtype=int),
    epa_pr_in: wp.array2d(dtype=wp.vec3),
    epa_norm2_in: wp.array2d(dtype=float),
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
    geom1: Geom,
    geom2: Geom,
    geoms: wp.vec2i,
    worldid: int,
    ccdid: int,
    margin: float,
    gap: float,
    condim: int,
    friction: vec5,
    solref: wp.vec2,
    solreffriction: wp.vec2,
    solimp: vec5,
    x1: wp.vec3,
    x2: wp.vec3,
    pairid: wp.vec2i,
    # Data out:
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
    contact_type_out: wp.array(dtype=int),
    contact_geomcollisionid_out: wp.array(dtype=int),
    nacon_out: wp.array(dtype=int),
  ) -> int:
    points = mat43()
    witness1 = mat43()
    witness2 = mat43()
    geom1.margin = margin
    geom2.margin = margin
    is_collision_sensor = pairid[1] >= 0
    if is_collision_sensor:
      cutoff = 1.0e32
    else:
      cutoff = 0.0
    dist, ncollision, w1, w2, multiccd_idx = ccd(
      opt_ccd_tolerance[worldid % opt_ccd_tolerance.shape[0]],
      cutoff,
      gjk_iterations,
      epa_iterations,
      geom1,
      geom2,
      geomtype1,
      geomtype2,
      x1,
      x2,
      epa_vert_in[ccdid],
      epa_vert_index_in[ccdid],
      epa_face_in[ccdid],
      epa_pr_in[ccdid],
      epa_norm2_in[ccdid],
      epa_horizon_in[ccdid],
    )

    if dist >= 0.0 and pairid[1] == -1:
      return 0

    witness1[0] = w1
    witness2[0] = w2

    if wp.static(use_multiccd or (geomtype1 == GeomType.BOX and geomtype2 == GeomType.BOX)):
      if wp.static(geomtype1 == GeomType.MESH):
        # verify that geom1 mesh data is present for multicontact
        if geom1.mesh_polyadr < 0:
          multiccd_idx = -1

      if wp.static(geomtype2 == GeomType.MESH):
        # verify that geom2 mesh data is present for multicontact
        if geom2.mesh_polyadr < 0:
          multiccd_idx = -1

      if multiccd_idx > -1:
        ncollision, witness1, witness2 = multicontact(
          multiccd_polygon_in[ccdid],
          multiccd_clipped_in[ccdid],
          multiccd_pnormal_in[ccdid],
          multiccd_pdist_in[ccdid],
          multiccd_idx1_in[ccdid],
          multiccd_idx2_in[ccdid],
          multiccd_n1_in[ccdid],
          multiccd_n2_in[ccdid],
          multiccd_endvert_in[ccdid],
          multiccd_face1_in[ccdid],
          multiccd_face2_in[ccdid],
          epa_vert_in[ccdid],
          epa_vert_index_in[ccdid],
          epa_face_in[ccdid, multiccd_idx],
          w1,
          w2,
          geom1,
          geom2,
          geomtype1,
          geomtype2,
        )

    for i in range(ncollision):
      points[i] = 0.5 * (witness1[i] + witness2[i])
    normal = witness1[0] - witness2[0]
    frame = make_frame(normal)

    # flip if collision sensor
    if pairid[1] >= 0:
      frame *= -1.0
      geoms = wp.vec2i(geoms[1], geoms[0])

    nactive = int(0)  # number of contacts contributing to the physics
    for i in range(ncollision):
      active = write_contact(
        naconmax_in,
        i,
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
        pairid,
        worldid,
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
        contact_type_out,
        contact_geomcollisionid_out,
        nacon_out,
      )
      nactive += active

    return nactive

  # runs convex collision on a set of geom pairs to recover contact info (non-heightfield)
  @wp.kernel(module="unique", enable_backward=False)
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
    geom_friction: wp.array2d(dtype=wp.vec3),
    geom_margin: wp.array2d(dtype=float),
    geom_gap: wp.array2d(dtype=float),
    mesh_vertadr: wp.array(dtype=int),
    mesh_vertnum: wp.array(dtype=int),
    mesh_graphadr: wp.array(dtype=int),
    mesh_vert: wp.array(dtype=wp.vec3),
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
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    geom_xmat_in: wp.array2d(dtype=wp.mat33),
    naconmax_in: int,
    naccdmax_in: int,
    ncollision_in: wp.array(dtype=int),
    # In:
    collision_pair_in: wp.array(dtype=wp.vec2i),
    collision_pairid_in: wp.array(dtype=wp.vec2i),
    collision_worldid_in: wp.array(dtype=int),
    epa_vert_in: wp.array2d(dtype=wp.vec3),
    epa_vert_index_in: wp.array2d(dtype=int),
    epa_face_in: wp.array2d(dtype=int),
    epa_pr_in: wp.array2d(dtype=wp.vec3),
    epa_norm2_in: wp.array2d(dtype=float),
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
    nccd_in: wp.array(dtype=int),
    # Data out:
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
    contact_type_out: wp.array(dtype=int),
    contact_geomcollisionid_out: wp.array(dtype=int),
    nacon_out: wp.array(dtype=int),
  ):
    collisionid = wp.tid()
    if collisionid >= ncollision_in[0]:
      return

    geoms = collision_pair_in[collisionid]
    g1 = geoms[0]
    g2 = geoms[1]

    if geom_type[g1] != geomtype1 or geom_type[g2] != geomtype2:
      return

    ccdid = wp.atomic_add(nccd_in, wp.static(geomgeomid), 1)
    if ccdid >= naccdmax_in:
      wp.printf("CCD overflow - please increase naccdmax to %u\n", ccdid)
      return

    worldid = collision_worldid_in[collisionid]

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
      collisionid,
      worldid,
    )

    geom1, geom2 = geom_collision_pair(
      geom_type,
      geom_dataid,
      geom_size,
      mesh_vertadr,
      mesh_vertnum,
      mesh_graphadr,
      mesh_vert,
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
      geoms,
      worldid,
    )

    eval_ccd_write_contact(
      opt_ccd_tolerance,
      naconmax_in,
      epa_vert_in,
      epa_vert_index_in,
      epa_face_in,
      epa_pr_in,
      epa_norm2_in,
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
      ccdid,
      margin,
      gap,
      condim,
      friction,
      solref,
      solreffriction,
      solimp,
      geom1.pos,
      geom2.pos,
      collision_pairid_in[collisionid],
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
      contact_type_out,
      contact_geomcollisionid_out,
      nacon_out,
    )

  return ccd_kernel


@event_scope
def convex_narrowphase(m: Model, d: Data, ctx: CollisionContext, collision_table: list[tuple[GeomType, GeomType]]):
  """Runs narrowphase collision detection for convex geom pairs.

  This function handles collision detection for pairs of convex geometries that were
  identified during the broadphase. It uses the Gilbert-Johnson-Keerthi (GJK) algorithm to
  determine the distance between shapes and the Expanding Polytope Algorithm (EPA) to find
  the penetration depth and contact normal for colliding pairs.

  The convex geom types handled by this function are `SPHERE`, `CAPSULE`, `ELLIPSOID`, `CYLINDER`,
  `BOX`, `MESH`, `HFIELD`.

  To optimize performance, this function dynamically builds and launches a specialized
  kernel for each type of convex collision pair present in the model, avoiding unnecessary
  computations for non-existent pair types.
  """

  def _pair_count(p1: int, p2: int) -> Tuple[int, int]:
    idx = upper_trid_index(len(GeomType), p1, p2)
    return m.geom_pair_type_count[idx], idx

  ncollision = sum(_pair_count(g[0].value, g[1].value)[0] for g in collision_table)
  # no convex collisions, early return

  if ncollision == 0:
    return

  # compute nmaxpolygon and nmaxmeshdeg given the geom pairs for the model
  nboxbox, _ = _pair_count(GeomType.BOX.value, GeomType.BOX.value)
  if (GeomType.BOX, GeomType.BOX) not in collision_table:
    nboxbox = 0
  nboxmesh, _ = _pair_count(GeomType.BOX.value, GeomType.MESH.value)
  nmeshmesh, _ = _pair_count(GeomType.MESH.value, GeomType.MESH.value)

  epa_iterations = 16 if nboxbox == ncollision else m.opt.ccd_iterations

  # set to true to enable multiccd
  use_multiccd = m.opt.enableflags & EnableBit.MULTICCD

  # need at least 4 (square sides) if there's a box collision needing multiccd
  nmaxpolygon = 4 if nboxbox > 0 else 0
  nmaxmeshdeg = 3 if nboxbox > 0 else 0

  # need to allocate more memory if there's meshes
  if use_multiccd and nmeshmesh + nboxmesh > 0:
    minval = 4 if nboxmesh else nmaxpolygon
    nmaxpolygon = max(m.nmaxpolygon, minval)
    nmaxmeshdeg = max(m.nmaxmeshdeg, 3)

  # ccd collider count
  nccd = wp.zeros(len(GeomType) * (len(GeomType) + 1) // 2, dtype=int)

  # epa_vert: vertices in EPA polytope
  epa_vert = wp.empty(shape=(d.naccdmax, 10 + 2 * epa_iterations), dtype=wp.vec3)
  # epa_vert_index: vertex indices in EPA polytope
  epa_vert_index = wp.empty(shape=(d.naccdmax, 10 + 2 * epa_iterations), dtype=int)
  # epa_face: faces of polytope represented by three indices
  epa_face = wp.empty(shape=(d.naccdmax, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=int)
  # epa_pr: projection of origin on polytope faces
  epa_pr = wp.empty(shape=(d.naccdmax, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=wp.vec3)
  # epa_norm2: epa_pr * epa_pr
  epa_norm2 = wp.empty(shape=(d.naccdmax, 6 + MJ_MAX_EPAFACES * epa_iterations), dtype=float)
  # epa_horizon: index pair (i j) of edges on horizon
  epa_horizon = wp.empty(shape=(d.naccdmax, MJ_MAX_EPAHORIZON), dtype=int)

  # Contact outputs
  contact_outputs = [
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
    d.contact.type,
    d.contact.geomcollisionid,
    d.nacon,
  ]

  # Launch heightfield collision kernels (no multiccd args, 72 args total)
  for geom_pair in collision_table:
    g1 = geom_pair[0].value
    g2 = geom_pair[1].value
    count, geomgeomid = _pair_count(g1, g2)
    if (g1 == GeomType.HFIELD or g2 == GeomType.HFIELD) and count:
      wp.launch(
        ccd_hfield_kernel_builder(g1, g2, m.opt.ccd_iterations, epa_iterations, geomgeomid),
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
          m.geom_rbound,
          m.geom_friction,
          m.geom_margin,
          m.geom_gap,
          m.mesh_vertadr,
          m.mesh_vertnum,
          m.mesh_graphadr,
          m.mesh_vert,
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
          m.hfield_size,
          m.hfield_nrow,
          m.hfield_ncol,
          m.hfield_adr,
          m.hfield_data,
          m.pair_dim,
          m.pair_solref,
          m.pair_solreffriction,
          m.pair_solimp,
          m.pair_margin,
          m.pair_gap,
          m.pair_friction,
          d.geom_xpos,
          d.geom_xmat,
          d.naconmax,
          d.naccdmax,
          d.ncollision,
          ctx.collision_pair,
          ctx.collision_pairid,
          ctx.collision_worldid,
          epa_vert,
          epa_vert_index,
          epa_face,
          epa_pr,
          epa_norm2,
          epa_horizon,
          nccd,
        ],
        outputs=contact_outputs,
      )

  # Allocate multiccd arrays only for non-heightfield collisions
  # multiccd_polygon: clipped contact surface
  multiccd_polygon = wp.empty(shape=(d.naccdmax, 2 * nmaxpolygon), dtype=wp.vec3)
  # multiccd_clipped: clipped contact surface (intermediate)
  multiccd_clipped = wp.empty(shape=(d.naccdmax, 2 * nmaxpolygon), dtype=wp.vec3)
  # multiccd_pnormal: plane normal of clipping polygon
  multiccd_pnormal = wp.empty(shape=(d.naccdmax, nmaxpolygon), dtype=wp.vec3)
  # multiccd_pdist: plane distance of clipping polygon
  multiccd_pdist = wp.empty(shape=(d.naccdmax, nmaxpolygon), dtype=float)
  # multiccd_idx1: list of normal index candidates for Geom 1
  multiccd_idx1 = wp.empty(shape=(d.naccdmax, nmaxmeshdeg), dtype=int)
  # multiccd_idx2: list of normal index candidates for Geom 2
  multiccd_idx2 = wp.empty(shape=(d.naccdmax, nmaxmeshdeg), dtype=int)
  # multiccd_n1: list of normal candidates for Geom 1
  multiccd_n1 = wp.empty(shape=(d.naccdmax, nmaxmeshdeg), dtype=wp.vec3)
  # multiccd_n2: list of normal candidates for Geom 1
  multiccd_n2 = wp.empty(shape=(d.naccdmax, nmaxmeshdeg), dtype=wp.vec3)
  # multiccd_endvert: list of edge vertices candidates
  multiccd_endvert = wp.empty(shape=(d.naccdmax, nmaxmeshdeg), dtype=wp.vec3)
  # multiccd_face1: contact face
  multiccd_face1 = wp.empty(shape=(d.naccdmax, nmaxpolygon), dtype=wp.vec3)
  # multiccd_face2: contact face
  multiccd_face2 = wp.empty(shape=(d.naccdmax, nmaxpolygon), dtype=wp.vec3)

  # Launch non-heightfield collision kernels (no hfield args, 78 args total)
  for geom_pair in collision_table:
    g1 = geom_pair[0].value
    g2 = geom_pair[1].value
    count, geomgeomid = _pair_count(g1, g2)
    if g1 != GeomType.HFIELD and g2 != GeomType.HFIELD and count:
      wp.launch(
        ccd_kernel_builder(g1, g2, m.opt.ccd_iterations, epa_iterations, use_multiccd, geomgeomid),
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
          m.geom_friction,
          m.geom_margin,
          m.geom_gap,
          m.mesh_vertadr,
          m.mesh_vertnum,
          m.mesh_graphadr,
          m.mesh_vert,
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
          d.geom_xpos,
          d.geom_xmat,
          d.naconmax,
          d.naccdmax,
          d.ncollision,
          ctx.collision_pair,
          ctx.collision_pairid,
          ctx.collision_worldid,
          epa_vert,
          epa_vert_index,
          epa_face,
          epa_pr,
          epa_norm2,
          epa_horizon,
          multiccd_polygon,
          multiccd_clipped,
          multiccd_pnormal,
          multiccd_pdist,
          multiccd_idx1,
          multiccd_idx2,
          multiccd_n1,
          multiccd_n2,
          multiccd_endvert,
          multiccd_face1,
          multiccd_face2,
          nccd,
        ],
        outputs=contact_outputs,
      )
