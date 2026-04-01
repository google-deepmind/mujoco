# Copyright 2026 The Newton Developers
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
"""Flex collision detection (geom vs flex triangles)."""

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import collision_primitive_core
from mujoco.mjx.third_party.mujoco_warp._src.math import make_frame
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINMU
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


@wp.func
def _write_flex_contact(
  # Data in:
  naconmax_in: int,
  # In:
  dist: float,
  pos: wp.vec3,
  frame: wp.mat33,
  margin: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solimp: vec5,
  geom: int,
  flexid: int,
  vertid: int,
  worldid: int,
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
  contact_flex_out: wp.array(dtype=wp.vec2i),
  contact_vert_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
  contact_type_out: wp.array(dtype=int),
  contact_geomcollisionid_out: wp.array(dtype=int),
  nacon_out: wp.array(dtype=int),
):
  if dist >= margin or dist >= MJ_MAXVAL:
    return

  id_ = wp.atomic_add(nacon_out, 0, 1)
  if id_ >= naconmax_in:
    return

  contact_dist_out[id_] = dist
  contact_pos_out[id_] = pos
  contact_frame_out[id_] = frame
  contact_includemargin_out[id_] = margin
  contact_friction_out[id_] = friction
  contact_solref_out[id_] = solref
  contact_solreffriction_out[id_] = wp.vec2(0.0, 0.0)
  contact_solimp_out[id_] = solimp
  contact_dim_out[id_] = condim
  contact_geom_out[id_] = wp.vec2i(geom, -1)
  contact_flex_out[id_] = wp.vec2i(-1, flexid)
  contact_vert_out[id_] = wp.vec2i(-1, vertid)
  contact_worldid_out[id_] = worldid
  contact_type_out[id_] = 1
  contact_geomcollisionid_out[id_] = 0


@wp.func
def _collide_geom_triangle(
  # Data in:
  naconmax_in: int,
  # In:
  gtype: int,
  pos: wp.vec3,
  rot: wp.mat33,
  size_val: wp.vec3,
  t1: wp.vec3,
  t2: wp.vec3,
  t3: wp.vec3,
  tri_radius: float,
  margin: float,
  condim: int,
  friction: vec5,
  solref: wp.vec2,
  solimp: vec5,
  geomid: int,
  flexid: int,
  vertex_id: int,
  worldid: int,
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
  contact_flex_out: wp.array(dtype=wp.vec2i),
  contact_vert_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
  contact_type_out: wp.array(dtype=int),
  contact_geomcollisionid_out: wp.array(dtype=int),
  nacon_out: wp.array(dtype=int),
):
  if gtype == int(GeomType.SPHERE):
    sphere_radius = size_val[0]
    dist, contact_pos, nrm = collision_primitive_core.sphere_triangle(pos, sphere_radius, t1, t2, t3, tri_radius)
    if dist < margin:
      _write_flex_contact(
        naconmax_in,
        dist,
        contact_pos,
        make_frame(nrm),
        margin,
        condim,
        friction,
        solref,
        solimp,
        geomid,
        flexid,
        vertex_id,
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
        contact_flex_out,
        contact_vert_out,
        contact_worldid_out,
        contact_type_out,
        contact_geomcollisionid_out,
        nacon_out,
      )
    return

  # Capsule, box, cylinder all return up to 2 contacts - compute then share writing code
  dists = wp.vec2(collision_primitive_core.MJ_MAXVAL, collision_primitive_core.MJ_MAXVAL)
  poss = collision_primitive_core.mat23f(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  nrms = collision_primitive_core.mat23f(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

  if gtype == int(GeomType.CAPSULE):
    cap_radius = size_val[0]
    cap_half_len = size_val[1]
    cap_axis = wp.vec3(rot[0, 2], rot[1, 2], rot[2, 2])
    dists, poss, nrms = collision_primitive_core.capsule_triangle(
      pos, cap_axis, cap_radius, cap_half_len, t1, t2, t3, tri_radius
    )
  elif gtype == int(GeomType.BOX):
    dists, poss, nrms = collision_primitive_core.box_triangle(pos, rot, size_val, t1, t2, t3, tri_radius)
  elif gtype == int(GeomType.CYLINDER):
    cyl_radius = size_val[0]
    cyl_half_height = size_val[1]
    cyl_axis = wp.vec3(rot[0, 2], rot[1, 2], rot[2, 2])
    dists, poss, nrms = collision_primitive_core.cylinder_triangle(
      pos, cyl_axis, cyl_radius, cyl_half_height, t1, t2, t3, tri_radius
    )

  # Write up to 2 contacts (shared code for capsule/box/cylinder)
  if dists[0] < margin:
    p1 = wp.vec3(poss[0, 0], poss[0, 1], poss[0, 2])
    n1 = wp.vec3(nrms[0, 0], nrms[0, 1], nrms[0, 2])
    _write_flex_contact(
      naconmax_in,
      dists[0],
      p1,
      make_frame(n1),
      margin,
      condim,
      friction,
      solref,
      solimp,
      geomid,
      flexid,
      vertex_id,
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
      contact_flex_out,
      contact_vert_out,
      contact_worldid_out,
      contact_type_out,
      contact_geomcollisionid_out,
      nacon_out,
    )
  if dists[1] < margin:
    p2 = wp.vec3(poss[1, 0], poss[1, 1], poss[1, 2])
    n2 = wp.vec3(nrms[1, 0], nrms[1, 1], nrms[1, 2])
    _write_flex_contact(
      naconmax_in,
      dists[1],
      p2,
      make_frame(n2),
      margin,
      condim,
      friction,
      solref,
      solimp,
      geomid,
      flexid,
      vertex_id,
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
      contact_flex_out,
      contact_vert_out,
      contact_worldid_out,
      contact_type_out,
      contact_geomcollisionid_out,
      nacon_out,
    )


@wp.kernel
def _flex_plane_narrowphase(
  # Model:
  ngeom: int,
  nflexvert: int,
  geom_type: wp.array(dtype=int),
  geom_condim: wp.array(dtype=int),
  geom_solref: wp.array2d(dtype=wp.vec2),
  geom_solimp: wp.array2d(dtype=vec5),
  geom_friction: wp.array2d(dtype=wp.vec3),
  geom_margin: wp.array2d(dtype=float),
  flex_condim: wp.array(dtype=int),
  flex_friction: wp.array(dtype=wp.vec3),
  flex_margin: wp.array(dtype=float),
  flex_vertadr: wp.array(dtype=int),
  flex_radius: wp.array(dtype=float),
  flex_vertflexid: wp.array(dtype=int),
  # Data in:
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  nworld_in: int,
  naconmax_in: int,
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
  contact_flex_out: wp.array(dtype=wp.vec2i),
  contact_vert_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
  contact_type_out: wp.array(dtype=int),
  contact_geomcollisionid_out: wp.array(dtype=int),
  nacon_out: wp.array(dtype=int),
):
  worldid, vertid = wp.tid()

  flexid = flex_vertflexid[vertid]
  radius = flex_radius[flexid]
  flex_margin_val = flex_margin[flexid]
  flex_condim_val = flex_condim[flexid]
  flex_fric = flex_friction[flexid]
  # Convert global vertid to local vertex index within this flex
  local_vertid = vertid - flex_vertadr[flexid]

  vert = flexvert_xpos_in[worldid, vertid]

  # TODO: Add a broadphase
  for geomid in range(ngeom):
    gtype = geom_type[geomid]
    if gtype != int(GeomType.PLANE):
      continue

    plane_pos = geom_xpos_in[worldid, geomid]
    plane_rot = geom_xmat_in[worldid, geomid]
    plane_normal = wp.vec3(plane_rot[0, 2], plane_rot[1, 2], plane_rot[2, 2])

    margin = geom_margin[worldid % geom_margin.shape[0], geomid] + flex_margin_val

    diff = vert - plane_pos
    signed_dist = wp.dot(diff, plane_normal)
    dist = signed_dist - radius

    if dist < margin:
      geom_condim_val = geom_condim[geomid]
      condim = wp.max(geom_condim_val, flex_condim_val)
      solref = geom_solref[worldid % geom_solref.shape[0], geomid]
      solimp = geom_solimp[worldid % geom_solimp.shape[0], geomid]
      geom_fric = geom_friction[worldid % geom_friction.shape[0], geomid]
      fric0 = wp.max(geom_fric[0], flex_fric[0])
      fric1 = wp.max(geom_fric[1], flex_fric[1])
      fric2 = wp.max(geom_fric[2], flex_fric[2])
      friction = vec5(
        wp.max(MJ_MINMU, fric0),
        wp.max(MJ_MINMU, fric0),
        wp.max(MJ_MINMU, fric1),
        wp.max(MJ_MINMU, fric2),
        wp.max(MJ_MINMU, fric2),
      )

      contact_pos = vert - plane_normal * (dist * 0.5 + radius)
      _write_flex_contact(
        naconmax_in,
        dist,
        contact_pos,
        make_frame(plane_normal),
        margin,
        condim,
        friction,
        solref,
        solimp,
        geomid,
        flexid,
        local_vertid,
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
        contact_flex_out,
        contact_vert_out,
        contact_worldid_out,
        contact_type_out,
        contact_geomcollisionid_out,
        nacon_out,
      )


@wp.kernel
def _flex_narrowphase_dim2(
  # Model:
  ngeom: int,
  nflex: int,
  geom_type: wp.array(dtype=int),
  geom_contype: wp.array(dtype=int),
  geom_conaffinity: wp.array(dtype=int),
  geom_condim: wp.array(dtype=int),
  geom_solref: wp.array2d(dtype=wp.vec2),
  geom_solimp: wp.array2d(dtype=vec5),
  geom_size: wp.array2d(dtype=wp.vec3),
  geom_friction: wp.array2d(dtype=wp.vec3),
  geom_margin: wp.array2d(dtype=float),
  flex_contype: wp.array(dtype=int),
  flex_conaffinity: wp.array(dtype=int),
  flex_margin: wp.array(dtype=float),
  flex_dim: wp.array(dtype=int),
  flex_vertadr: wp.array(dtype=int),
  flex_elemadr: wp.array(dtype=int),
  flex_elemnum: wp.array(dtype=int),
  flex_elemdataadr: wp.array(dtype=int),
  flex_elem: wp.array(dtype=int),
  flex_radius: wp.array(dtype=float),
  # Data in:
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  nworld_in: int,
  naconmax_in: int,
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
  contact_flex_out: wp.array(dtype=wp.vec2i),
  contact_vert_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
  contact_type_out: wp.array(dtype=int),
  contact_geomcollisionid_out: wp.array(dtype=int),
  nacon_out: wp.array(dtype=int),
):
  worldid, elemid = wp.tid()

  flexid = int(-1)
  for i in range(nflex):
    if flex_dim[i] != 2:
      continue
    elem_adr = flex_elemadr[i]
    elem_num = flex_elemnum[i]
    if elemid >= elem_adr and elemid < elem_adr + elem_num:
      flexid = i
      break

  if flexid < 0:
    return

  vert_adr = flex_vertadr[flexid]
  tri_radius = flex_radius[flexid]
  tri_margin = flex_margin[flexid]

  elem_data_idx = flex_elemdataadr[flexid] + (elemid - flex_elemadr[flexid]) * 3
  v0_local = flex_elem[elem_data_idx]
  v1_local = flex_elem[elem_data_idx + 1]
  v2_local = flex_elem[elem_data_idx + 2]

  t1 = flexvert_xpos_in[worldid, vert_adr + v0_local]
  t2 = flexvert_xpos_in[worldid, vert_adr + v1_local]
  t3 = flexvert_xpos_in[worldid, vert_adr + v2_local]

  # TODO: Add a broadphase
  for geomid in range(ngeom):
    gtype = geom_type[geomid]
    if (
      gtype != int(GeomType.SPHERE)
      and gtype != int(GeomType.CAPSULE)
      and gtype != int(GeomType.BOX)
      and gtype != int(GeomType.CYLINDER)
    ):
      continue

    g_contype = geom_contype[geomid]
    g_conaffinity = geom_conaffinity[geomid]
    f_contype = flex_contype[flexid]
    f_conaffinity = flex_conaffinity[flexid]
    if not ((g_contype & f_conaffinity) or (f_contype & g_conaffinity)):
      continue

    geom_margin_val = geom_margin[worldid % geom_margin.shape[0], geomid]
    margin = geom_margin_val + tri_margin

    geom_pos = geom_xpos_in[worldid, geomid]
    geom_rot = geom_xmat_in[worldid, geomid]
    geom_size_val = geom_size[worldid % geom_size.shape[0], geomid]

    condim = geom_condim[geomid]
    gf = geom_friction[worldid % geom_friction.shape[0], geomid]
    friction = vec5(
      wp.max(MJ_MINMU, gf[0]),
      wp.max(MJ_MINMU, gf[0]),
      wp.max(MJ_MINMU, gf[1]),
      wp.max(MJ_MINMU, gf[2]),
      wp.max(MJ_MINMU, gf[2]),
    )
    solref = geom_solref[worldid % geom_solref.shape[0], geomid]
    solimp = geom_solimp[worldid % geom_solimp.shape[0], geomid]

    _collide_geom_triangle(
      naconmax_in,
      gtype,
      geom_pos,
      geom_rot,
      geom_size_val,
      t1,
      t2,
      t3,
      tri_radius,
      margin,
      condim,
      friction,
      solref,
      solimp,
      geomid,
      flexid,
      v0_local,
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
      contact_flex_out,
      contact_vert_out,
      contact_worldid_out,
      contact_type_out,
      contact_geomcollisionid_out,
      nacon_out,
    )


@wp.kernel
def _flex_narrowphase_dim3(
  # Model:
  ngeom: int,
  nflex: int,
  geom_type: wp.array(dtype=int),
  geom_contype: wp.array(dtype=int),
  geom_conaffinity: wp.array(dtype=int),
  geom_condim: wp.array(dtype=int),
  geom_solref: wp.array2d(dtype=wp.vec2),
  geom_solimp: wp.array2d(dtype=vec5),
  geom_size: wp.array2d(dtype=wp.vec3),
  geom_friction: wp.array2d(dtype=wp.vec3),
  geom_margin: wp.array2d(dtype=float),
  flex_contype: wp.array(dtype=int),
  flex_conaffinity: wp.array(dtype=int),
  flex_margin: wp.array(dtype=float),
  flex_dim: wp.array(dtype=int),
  flex_vertadr: wp.array(dtype=int),
  flex_shellnum: wp.array(dtype=int),
  flex_shelldataadr: wp.array(dtype=int),
  flex_shell: wp.array(dtype=int),
  flex_radius: wp.array(dtype=float),
  # Data in:
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  flexvert_xpos_in: wp.array2d(dtype=wp.vec3),
  nworld_in: int,
  naconmax_in: int,
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
  contact_flex_out: wp.array(dtype=wp.vec2i),
  contact_vert_out: wp.array(dtype=wp.vec2i),
  contact_worldid_out: wp.array(dtype=int),
  contact_type_out: wp.array(dtype=int),
  contact_geomcollisionid_out: wp.array(dtype=int),
  nacon_out: wp.array(dtype=int),
):
  worldid, shellid = wp.tid()

  flexid = int(-1)
  shell_offset = int(0)
  for i in range(nflex):
    if flex_dim[i] != 3:
      continue
    shell_num = flex_shellnum[i]
    if shellid >= shell_offset and shellid < shell_offset + shell_num:
      flexid = i
      break
    shell_offset += shell_num

  if flexid < 0:
    return

  vert_adr = flex_vertadr[flexid]
  tri_radius = flex_radius[flexid]
  tri_margin = flex_margin[flexid]

  shell_adr = flex_shelldataadr[flexid]
  local_shellid = shellid - shell_offset
  shell_data_idx = shell_adr + local_shellid * 3

  v0_local = flex_shell[shell_data_idx]
  v1_local = flex_shell[shell_data_idx + 1]
  v2_local = flex_shell[shell_data_idx + 2]

  t1 = flexvert_xpos_in[worldid, vert_adr + v0_local]
  t2 = flexvert_xpos_in[worldid, vert_adr + v1_local]
  t3 = flexvert_xpos_in[worldid, vert_adr + v2_local]

  # TODO: Add a broadphase
  for geomid in range(ngeom):
    gtype = geom_type[geomid]
    if (
      gtype != int(GeomType.SPHERE)
      and gtype != int(GeomType.CAPSULE)
      and gtype != int(GeomType.BOX)
      and gtype != int(GeomType.CYLINDER)
    ):
      continue

    g_contype = geom_contype[geomid]
    g_conaffinity = geom_conaffinity[geomid]
    f_contype = flex_contype[flexid]
    f_conaffinity = flex_conaffinity[flexid]
    if not ((g_contype & f_conaffinity) or (f_contype & g_conaffinity)):
      continue

    geom_margin_val = geom_margin[worldid % geom_margin.shape[0], geomid]
    margin = geom_margin_val + tri_margin

    geom_pos = geom_xpos_in[worldid, geomid]
    geom_rot = geom_xmat_in[worldid, geomid]
    geom_size_val = geom_size[worldid % geom_size.shape[0], geomid]

    condim = geom_condim[geomid]
    gf = geom_friction[worldid % geom_friction.shape[0], geomid]
    friction = vec5(
      wp.max(MJ_MINMU, gf[0]),
      wp.max(MJ_MINMU, gf[0]),
      wp.max(MJ_MINMU, gf[1]),
      wp.max(MJ_MINMU, gf[2]),
      wp.max(MJ_MINMU, gf[2]),
    )
    solref = geom_solref[worldid % geom_solref.shape[0], geomid]
    solimp = geom_solimp[worldid % geom_solimp.shape[0], geomid]

    _collide_geom_triangle(
      naconmax_in,
      gtype,
      geom_pos,
      geom_rot,
      geom_size_val,
      t1,
      t2,
      t3,
      tri_radius,
      margin,
      condim,
      friction,
      solref,
      solimp,
      geomid,
      flexid,
      v0_local,
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
      contact_flex_out,
      contact_vert_out,
      contact_worldid_out,
      contact_type_out,
      contact_geomcollisionid_out,
      nacon_out,
    )


@event_scope
def flex_narrowphase(m: Model, d: Data):
  """Runs collision detection between geoms and flex elements."""
  if m.nflex == 0:
    return

  wp.launch(
    _flex_narrowphase_dim2,
    dim=(d.nworld, m.nflexelem),
    inputs=[
      m.ngeom,
      m.nflex,
      m.geom_type,
      m.geom_contype,
      m.geom_conaffinity,
      m.geom_condim,
      m.geom_solref,
      m.geom_solimp,
      m.geom_size,
      m.geom_friction,
      m.geom_margin,
      m.flex_contype,
      m.flex_conaffinity,
      m.flex_margin,
      m.flex_dim,
      m.flex_vertadr,
      m.flex_elemadr,
      m.flex_elemnum,
      m.flex_elemdataadr,
      m.flex_elem,
      m.flex_radius,
      d.geom_xpos,
      d.geom_xmat,
      d.flexvert_xpos,
      d.nworld,
      d.naconmax,
    ],
    outputs=[
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
      d.contact.flex,
      d.contact.vert,
      d.contact.worldid,
      d.contact.type,
      d.contact.geomcollisionid,
      d.nacon,
    ],
  )

  wp.launch(
    _flex_narrowphase_dim3,
    dim=(d.nworld, m.nflexshelldata // 3),
    inputs=[
      m.ngeom,
      m.nflex,
      m.geom_type,
      m.geom_contype,
      m.geom_conaffinity,
      m.geom_condim,
      m.geom_solref,
      m.geom_solimp,
      m.geom_size,
      m.geom_friction,
      m.geom_margin,
      m.flex_contype,
      m.flex_conaffinity,
      m.flex_margin,
      m.flex_dim,
      m.flex_vertadr,
      m.flex_shellnum,
      m.flex_shelldataadr,
      m.flex_shell,
      m.flex_radius,
      d.geom_xpos,
      d.geom_xmat,
      d.flexvert_xpos,
      d.nworld,
      d.naconmax,
    ],
    outputs=[
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
      d.contact.flex,
      d.contact.vert,
      d.contact.worldid,
      d.contact.type,
      d.contact.geomcollisionid,
      d.nacon,
    ],
  )

  wp.launch(
    _flex_plane_narrowphase,
    dim=(d.nworld, m.nflexvert),
    inputs=[
      m.ngeom,
      m.nflexvert,
      m.geom_type,
      m.geom_condim,
      m.geom_solref,
      m.geom_solimp,
      m.geom_friction,
      m.geom_margin,
      m.flex_condim,
      m.flex_friction,
      m.flex_margin,
      m.flex_vertadr,
      m.flex_radius,
      m.flex_vertflexid,
      d.geom_xpos,
      d.geom_xmat,
      d.flexvert_xpos,
      d.nworld,
      d.naconmax,
    ],
    outputs=[
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
      d.contact.flex,
      d.contact.vert,
      d.contact.worldid,
      d.contact.type,
      d.contact.geomcollisionid,
      d.nacon,
    ],
  )
