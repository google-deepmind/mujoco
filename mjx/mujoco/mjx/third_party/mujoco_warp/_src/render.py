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
# ==============================================================================

from typing import Tuple

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_box
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_capsule
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_cylinder
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_ellipsoid
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_flex_with_bvh
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_flex_with_bvh_anyhit
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_mesh_with_bvh
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_mesh_with_bvh_anyhit
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_plane
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_sphere
from mujoco.mjx.third_party.mujoco_warp._src.render_util import compute_ray
from mujoco.mjx.third_party.mujoco_warp._src.render_util import pack_rgba_to_uint32
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import ObjType
from mujoco.mjx.third_party.mujoco_warp._src.types import RenderContext
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


@wp.func
def sample_texture(
  # Model:
  geom_type: wp.array[int],
  mesh_faceadr: wp.array[int],
  # In:
  geom_id: int,
  tex_repeat: wp.vec2,
  tex: wp.Texture2D,
  pos: wp.vec3,
  rot: wp.mat33,
  mesh_facetexcoord: wp.array[wp.vec3i],
  mesh_texcoord: wp.array[wp.vec2],
  mesh_texcoord_offsets: wp.array[int],
  hit_point: wp.vec3,
  bary_u: float,
  bary_v: float,
  f: int,
  mesh_id: int,
) -> wp.vec3:
  uv = wp.vec2(0.0, 0.0)

  if geom_type[geom_id] == GeomType.PLANE:
    local = wp.transpose(rot) @ (hit_point - pos)
    uv = wp.vec2(local[0], local[1])

  if geom_type[geom_id] == GeomType.MESH:
    if f < 0 or mesh_id < 0:
      return wp.vec3(0.0, 0.0, 0.0)

    face_adr = mesh_faceadr[mesh_id] + f
    uv0 = mesh_texcoord[mesh_texcoord_offsets[mesh_id] + mesh_facetexcoord[face_adr][0]]
    uv1 = mesh_texcoord[mesh_texcoord_offsets[mesh_id] + mesh_facetexcoord[face_adr][1]]
    uv2 = mesh_texcoord[mesh_texcoord_offsets[mesh_id] + mesh_facetexcoord[face_adr][2]]
    uv = uv0 * bary_u + uv1 * bary_v + uv2 * (1.0 - bary_u - bary_v)

  u = uv[0] * tex_repeat[0]
  v = uv[1] * tex_repeat[1]
  u = u - wp.floor(u)
  v = v - wp.floor(v)
  tex_color = wp.texture_sample(tex, wp.vec2(u, v), dtype=wp.vec4)
  return wp.vec3(tex_color[0], tex_color[1], tex_color[2])


@wp.func
def sample_skybox(
  # In:
  skybox_tex: wp.Texture2D,
  face_width_inv: float,
  ray_dir_world: wp.vec3,
) -> wp.vec3:
  # MuJoCo maps a world-space direction to cube-map space by rotating 90° about X
  # (see render_gl3.c: S=x, T=z, R=-y). Faces in tex_data are stacked vertically
  # in OpenGL cube-face order: +X, -X, +Y, -Y, +Z, -Z.
  rx = ray_dir_world[0]
  ry = ray_dir_world[2]
  rz = -ray_dir_world[1]

  arx = wp.abs(rx)
  ary = wp.abs(ry)
  arz = wp.abs(rz)

  face = int(0)
  sc = float(0.0)
  tc = float(0.0)
  ma = float(1.0)

  if arx >= ary and arx >= arz:
    ma = arx
    if rx > 0.0:
      face = 0
      sc = -rz
      tc = -ry
    else:
      face = 1
      sc = rz
      tc = -ry
  elif ary >= arz:
    ma = ary
    if ry > 0.0:
      face = 2
      sc = rx
      tc = rz
    else:
      face = 3
      sc = rx
      tc = -rz
  else:
    ma = arz
    if rz > 0.0:
      face = 4
      sc = rx
      tc = -ry
    else:
      face = 5
      sc = -rx
      tc = -ry

  s = (math.safe_div(sc, ma) + 1.0) * 0.5
  t = (math.safe_div(tc, ma) + 1.0) * 0.5

  # Keep the linear filter from bleeding between adjacent faces in the vertical strip.
  t_min = 0.5 * face_width_inv
  t = wp.clamp(t, t_min, 1.0 - t_min)

  v = (float(face) + t) * wp.static(1.0 / 6.0)
  color = wp.texture_sample(skybox_tex, wp.vec2(s, v), dtype=wp.vec4)
  return wp.vec3(color[0], color[1], color[2])


def _make_cast_ray(geom_ray_types: Tuple[int], first_hit: bool = False) -> wp.Function:
  """Build a ray-cast func specialized to the geom types present in the scene.

  geom_ray_types is the set of GeomType int values that actually occur, so the
  per-type intersection branches for absent types are eliminated at compile time
  via wp.static, avoiding the register pressure of unreachable code paths.

  first_hit selects the variant (also resolved at compile time via wp.static):
    - False: full closest-hit cast. Returns the closest hit's full surface data.
    - True: any-hit cast (shadow rays). Uses the cheaper any-hit mesh/flex
      intersections and returns on the first hit within max_dist. The result is
      still the full tuple; callers test geom_id != -1 to detect a hit.
  """

  @wp.func
  def cast_ray(
    # Model:
    geom_type: wp.array[int],
    geom_dataid: wp.array2d[int],
    geom_size: wp.array2d[wp.vec3],
    flex_vertadr: wp.array[int],
    flex_edge: wp.array[wp.vec2i],
    flex_radius: wp.array[float],
    # Data in:
    geom_xpos_in: wp.array2d[wp.vec3],
    geom_xmat_in: wp.array2d[wp.mat33],
    flexvert_xpos_in: wp.array2d[wp.vec3],
    # In:
    bvh_id: wp.uint64,
    group_root: int,
    worldid: int,
    bvh_ngeom: int,
    flex_bvh_ngeom: int,
    enabled_geom_ids: wp.array[int],
    mesh_bvh_id: wp.array[wp.uint64],
    hfield_bvh_id: wp.array[wp.uint64],
    flex_geom_flexid: wp.array[int],
    flex_geom_edgeid: wp.array[int],
    flex_bvh_id: wp.array[wp.uint64],
    flex_group_root: wp.array2d[int],
    ray_origin_world: wp.vec3,
    ray_dir_world: wp.vec3,
    max_dist: float,
    cull_backfaces: bool,
  ) -> Tuple[int, float, wp.vec3, float, float, int, int]:
    dist = max_dist
    normal = wp.vec3(0.0, 0.0, 0.0)
    geom_id = int(-1)
    bary_u = float(0.0)
    bary_v = float(0.0)
    face_idx = int(-1)
    geom_mesh_id = int(-1)

    query = wp.bvh_query_ray(bvh_id, ray_origin_world, ray_dir_world, group_root)
    bounds_nr = int(0)
    ngeom = bvh_ngeom + flex_bvh_ngeom

    while wp.bvh_query_next(query, bounds_nr, dist):
      gi_global = bounds_nr
      local_id = gi_global - (worldid * ngeom)

      d = float(-1.0)
      hit_mesh_id = int(-1)
      u = float(0.0)
      v = float(0.0)
      f = int(-1)
      n = wp.vec3(0.0, 0.0, 0.0)
      hit_geom_id = int(-1)

      if local_id < bvh_ngeom:
        gi = enabled_geom_ids[local_id]
        gtype = geom_type[gi]
      else:
        gi = local_id - bvh_ngeom
        gtype = GeomType.FLEX

      hit_geom_id = gi

      if wp.static(int(GeomType.PLANE) in geom_ray_types):
        if gtype == GeomType.PLANE:
          d, n = ray_plane(
            geom_xpos_in[worldid, gi],
            geom_xmat_in[worldid, gi],
            geom_size[worldid % geom_size.shape[0], gi],
            ray_origin_world,
            ray_dir_world,
          )
      if wp.static(int(GeomType.HFIELD) in geom_ray_types):
        if gtype == GeomType.HFIELD:
          d, n, u, v, f, geom_hfield_id = ray_mesh_with_bvh(
            hfield_bvh_id,
            geom_dataid[worldid % geom_dataid.shape[0], gi],
            geom_xpos_in[worldid, gi],
            geom_xmat_in[worldid, gi],
            ray_origin_world,
            ray_dir_world,
            dist,
            cull_backfaces,
          )
      if wp.static(int(GeomType.SPHERE) in geom_ray_types):
        if gtype == GeomType.SPHERE:
          d, n = ray_sphere(
            geom_xpos_in[worldid, gi],
            geom_size[worldid % geom_size.shape[0], gi][0] * geom_size[worldid % geom_size.shape[0], gi][0],
            ray_origin_world,
            ray_dir_world,
          )
      if wp.static(int(GeomType.ELLIPSOID) in geom_ray_types):
        if gtype == GeomType.ELLIPSOID:
          d, n = ray_ellipsoid(
            geom_xpos_in[worldid, gi],
            geom_xmat_in[worldid, gi],
            geom_size[worldid % geom_size.shape[0], gi],
            ray_origin_world,
            ray_dir_world,
          )
      if wp.static(int(GeomType.CAPSULE) in geom_ray_types):
        if gtype == GeomType.CAPSULE:
          d, n = ray_capsule(
            geom_xpos_in[worldid, gi],
            geom_xmat_in[worldid, gi],
            geom_size[worldid % geom_size.shape[0], gi],
            ray_origin_world,
            ray_dir_world,
          )
      if wp.static(int(GeomType.CYLINDER) in geom_ray_types):
        if gtype == GeomType.CYLINDER:
          d, n = ray_cylinder(
            geom_xpos_in[worldid, gi],
            geom_xmat_in[worldid, gi],
            geom_size[worldid % geom_size.shape[0], gi],
            ray_origin_world,
            ray_dir_world,
          )
      if wp.static(int(GeomType.BOX) in geom_ray_types):
        if gtype == GeomType.BOX:
          d, all, n = ray_box(
            geom_xpos_in[worldid, gi],
            geom_xmat_in[worldid, gi],
            geom_size[worldid % geom_size.shape[0], gi],
            ray_origin_world,
            ray_dir_world,
          )
      if wp.static(int(GeomType.MESH) in geom_ray_types):
        if gtype == GeomType.MESH:
          if wp.static(first_hit):
            hit = ray_mesh_with_bvh_anyhit(
              mesh_bvh_id,
              geom_dataid[worldid % geom_dataid.shape[0], gi],
              geom_xpos_in[worldid, gi],
              geom_xmat_in[worldid, gi],
              ray_origin_world,
              ray_dir_world,
              dist,
            )
            d = 0.0 if hit else -1.0
          else:
            d, n, u, v, f, hit_mesh_id = ray_mesh_with_bvh(
              mesh_bvh_id,
              geom_dataid[worldid % geom_dataid.shape[0], gi],
              geom_xpos_in[worldid, gi],
              geom_xmat_in[worldid, gi],
              ray_origin_world,
              ray_dir_world,
              dist,
              cull_backfaces,
            )
      if wp.static(int(GeomType.FLEX) in geom_ray_types):
        if gtype == GeomType.FLEX:
          hit_geom_id = -2
          flexid = flex_geom_flexid[gi]
          edge_id = flex_geom_edgeid[gi]

          if edge_id >= 0:
            edge = flex_edge[edge_id]
            vert_adr = flex_vertadr[flexid]
            v0 = flexvert_xpos_in[worldid, vert_adr + edge[0]]
            v1 = flexvert_xpos_in[worldid, vert_adr + edge[1]]
            pos = 0.5 * (v0 + v1)
            vec = v1 - v0

            length = wp.length(vec)
            edgeq = math.quat_z2vec(vec)
            mat = math.quat_to_mat(edgeq)
            size = wp.vec3(flex_radius[flexid], 0.5 * length, 0.0)

            d, n = ray_capsule(pos, mat, size, ray_origin_world, ray_dir_world)
            hit_mesh_id = flexid
          else:
            if wp.static(first_hit):
              hit = ray_flex_with_bvh_anyhit(
                flex_bvh_id,
                flexid,
                flex_group_root[worldid, flexid],
                ray_origin_world,
                ray_dir_world,
                dist,
              )
              d = 0.0 if hit else -1.0
            else:
              flex_gr = flex_group_root[worldid, flexid]
              d, n, u, v, f = ray_flex_with_bvh(flex_bvh_id, flexid, flex_gr, ray_origin_world, ray_dir_world, dist)
              if d >= 0.0:
                hit_mesh_id = flexid

      # Backface cull: drop exit-face hits when the ray origin is inside the geom,
      # matching ray_mesh_with_bvh's `dot(lvec, n) < 0` rule. Strict `> 0` keeps
      # tangent hits and skips branches with a zero-vector normal (any-hit).
      if cull_backfaces and d >= 0.0 and wp.dot(ray_dir_world, n) > 0.0:
        d = -1.0

      if wp.static(first_hit):
        # Any-hit: return as soon as anything is in range; surface data is unused.
        if d >= 0.0 and d < dist:
          return hit_geom_id, d, n, u, v, f, hit_mesh_id
      else:
        if d >= 0.0 and d < dist:
          dist = d
          normal = n
          geom_id = hit_geom_id
          bary_u = u
          bary_v = v
          face_idx = f
          geom_mesh_id = hit_mesh_id

    return geom_id, dist, normal, bary_u, bary_v, face_idx, geom_mesh_id

  return cast_ray


def _make_compute_lighting(cast_ray_first_hit: wp.Function) -> wp.Function:
  """Build specialized compute_lighting."""

  @wp.func
  def compute_lighting(
    # Model:
    geom_type: wp.array[int],
    geom_dataid: wp.array2d[int],
    geom_size: wp.array2d[wp.vec3],
    flex_vertadr: wp.array[int],
    flex_edge: wp.array[wp.vec2i],
    flex_radius: wp.array[float],
    # Data in:
    geom_xpos_in: wp.array2d[wp.vec3],
    geom_xmat_in: wp.array2d[wp.mat33],
    flexvert_xpos_in: wp.array2d[wp.vec3],
    # In:
    use_shadows: bool,
    bvh_id: wp.uint64,
    group_root: int,
    bvh_ngeom: int,
    bvh_nflexgeom: int,
    enabled_geom_ids: wp.array[int],
    worldid: int,
    mesh_bvh_id: wp.array[wp.uint64],
    hfield_bvh_id: wp.array[wp.uint64],
    flex_geom_flexid: wp.array[int],
    flex_geom_edgeid: wp.array[int],
    flex_bvh_id: wp.array[wp.uint64],
    flex_group_root: wp.array2d[int],
    lightactive: bool,
    lighttype: int,
    lightcastshadow: bool,
    lightpos: wp.vec3,
    lightdir: wp.vec3,
    normal: wp.vec3,
    hitpoint: wp.vec3,
    cull_backfaces: bool,
  ) -> float:
    light_contribution = float(0.0)

    # TODO: We should probably only be looping over active lights
    # in the first place with a static loop of enabled light idx?
    if not lightactive:
      return light_contribution

    L = wp.vec3(0.0, 0.0, 0.0)
    dist_to_light = float(MJ_MAXVAL)
    attenuation = float(1.0)

    if lighttype == 1:  # directional light
      L = wp.normalize(-lightdir)
    else:
      L, dist_to_light = math.normalize_with_norm(lightpos - hitpoint)
      attenuation = 1.0 / (1.0 + 0.02 * dist_to_light * dist_to_light)
      if lighttype == 0:  # spot light
        spot_dir = wp.normalize(lightdir)
        cos_theta = wp.dot(-L, spot_dir)
        spot_factor = wp.min(1.0, wp.max(0.0, (cos_theta - 0.85) * 10.0))
        attenuation = attenuation * spot_factor

    ndotl = wp.max(0.0, wp.dot(normal, L))
    if ndotl == 0.0:
      return light_contribution

    visible = float(1.0)

    if use_shadows and lightcastshadow:
      # Nudge the origin slightly along the surface normal to avoid
      # self-intersection when casting shadow rays
      shadow_origin = hitpoint + normal * 1.0e-4
      # Distance-limited shadows: cap by dist_to_light (for non-directional)
      max_t = dist_to_light - 1.0e-3
      if lighttype == 1:  # directional light
        max_t = 1.0e8

      shadow_geom_id, shadow_d, shadow_n, shadow_u, shadow_v, shadow_f, shadow_mesh_id = cast_ray_first_hit(
        geom_type,
        geom_dataid,
        geom_size,
        flex_vertadr,
        flex_edge,
        flex_radius,
        geom_xpos_in,
        geom_xmat_in,
        flexvert_xpos_in,
        bvh_id,
        group_root,
        worldid,
        bvh_ngeom,
        bvh_nflexgeom,
        enabled_geom_ids,
        mesh_bvh_id,
        hfield_bvh_id,
        flex_geom_flexid,
        flex_geom_edgeid,
        flex_bvh_id,
        flex_group_root,
        shadow_origin,
        L,
        max_t,
        cull_backfaces,
      )

      if shadow_geom_id != -1:
        visible = 0.3

    return ndotl * attenuation * visible

  return compute_lighting


@event_scope
def render(m: Model, d: Data, rc: RenderContext):
  """Render the current frame.

  Outputs are stored in buffers within the render context.

  Args:
    m: The model on device.
    d: The data on device.
    rc: The render context on device.
  """
  rc.rgb_data.fill_(rc.background_color)
  rc.depth_data.fill_(0.0)
  rc.seg_data.fill_(wp.vec2i(-1, -1))

  # Specialize the ray-cast helpers to the geom types present in the scene so the
  # compiler eliminates intersection branches for absent types.
  geom_ray_types = rc.geom_ray_types
  cast_ray = _make_cast_ray(geom_ray_types, first_hit=False)
  cast_ray_first_hit = _make_cast_ray(geom_ray_types, first_hit=True)
  compute_lighting = _make_compute_lighting(cast_ray_first_hit)

  @wp.kernel(module="unique", enable_backward=False)
  def _render_megakernel(
    # Model:
    geom_type: wp.array[int],
    geom_dataid: wp.array2d[int],
    geom_matid: wp.array2d[int],
    geom_size: wp.array2d[wp.vec3],
    geom_rgba: wp.array2d[wp.vec4],
    cam_projection: wp.array[int],
    cam_fovy: wp.array2d[float],
    cam_sensorsize: wp.array[wp.vec2],
    cam_intrinsic: wp.array2d[wp.vec4],
    light_type: wp.array2d[int],
    light_castshadow: wp.array2d[bool],
    light_active: wp.array2d[bool],
    flex_vertadr: wp.array[int],
    flex_edge: wp.array[wp.vec2i],
    flex_radius: wp.array[float],
    mesh_faceadr: wp.array[int],
    mat_texid: wp.array3d[int],
    mat_texrepeat: wp.array2d[wp.vec2],
    mat_rgba: wp.array2d[wp.vec4],
    # Data in:
    geom_xpos_in: wp.array2d[wp.vec3],
    geom_xmat_in: wp.array2d[wp.mat33],
    cam_xpos_in: wp.array2d[wp.vec3],
    cam_xmat_in: wp.array2d[wp.mat33],
    light_xpos_in: wp.array2d[wp.vec3],
    light_xdir_in: wp.array2d[wp.vec3],
    flexvert_xpos_in: wp.array2d[wp.vec3],
    # In:
    nrender: int,
    use_shadows: bool,
    bvh_ngeom: int,
    bvh_nflexgeom: int,
    cam_res: wp.array[wp.vec2i],
    cam_id_map: wp.array[int],
    ray: wp.array[wp.vec3],
    rgb_adr: wp.array[int],
    depth_adr: wp.array[int],
    seg_adr: wp.array[int],
    render_rgb: wp.array[bool],
    render_depth: wp.array[bool],
    render_seg: wp.array[bool],
    bvh_id: wp.uint64,
    group_root: wp.array[int],
    flex_bvh_id: wp.array[wp.uint64],
    flex_group_root: wp.array2d[int],
    enabled_geom_ids: wp.array[int],
    mesh_bvh_id: wp.array[wp.uint64],
    mesh_facetexcoord: wp.array[wp.vec3i],
    mesh_texcoord: wp.array[wp.vec2],
    mesh_texcoord_offsets: wp.array[int],
    hfield_bvh_id: wp.array[wp.uint64],
    flex_rgba: wp.array[wp.vec4],
    flex_geom_flexid: wp.array[int],
    flex_geom_edgeid: wp.array[int],
    textures: wp.array[wp.Texture2D],
    # Out:
    rgb_out: wp.array2d[wp.uint32],
    depth_out: wp.array2d[float],
    seg_out: wp.array2d[wp.vec2i],
  ):
    worldid, rayid = wp.tid()

    # Map global rayid -> (camid, rayid_local) using cumulative sizes
    camid = int(-1)
    rayid_local = int(-1)
    accum = int(0)
    for i in range(nrender):
      num_i = cam_res[i][0] * cam_res[i][1]
      if rayid < accum + num_i:
        camid = i
        rayid_local = rayid - accum
        break
      accum += num_i
    if camid == -1 or rayid_local < 0:
      return

    if not render_rgb[camid] and not render_depth[camid] and not render_seg[camid]:
      return

    # Map active camera index to MuJoCo camera ID
    mujoco_cam_id = cam_id_map[camid]

    if wp.static(rc.use_precomputed_rays):
      ray_dir_local_cam = ray[rayid]
    else:
      img_w = cam_res[camid][0]
      img_h = cam_res[camid][1]
      px = rayid_local % img_w
      py = rayid_local // img_w
      ray_dir_local_cam = compute_ray(
        cam_projection[mujoco_cam_id],
        cam_fovy[worldid % cam_fovy.shape[0], mujoco_cam_id],
        cam_sensorsize[mujoco_cam_id],
        cam_intrinsic[worldid % cam_intrinsic.shape[0], mujoco_cam_id],
        img_w,
        img_h,
        px,
        py,
        wp.static(rc.znear),
      )

    ray_dir_world = cam_xmat_in[worldid, mujoco_cam_id] @ ray_dir_local_cam
    ray_origin_world = cam_xpos_in[worldid, mujoco_cam_id]

    geom_id, dist, normal, u, v, f, mesh_id = cast_ray(
      geom_type,
      geom_dataid,
      geom_size,
      flex_vertadr,
      flex_edge,
      flex_radius,
      geom_xpos_in,
      geom_xmat_in,
      flexvert_xpos_in,
      bvh_id,
      group_root[worldid],
      worldid,
      bvh_ngeom,
      bvh_nflexgeom,
      enabled_geom_ids,
      mesh_bvh_id,
      hfield_bvh_id,
      flex_geom_flexid,
      flex_geom_edgeid,
      flex_bvh_id,
      flex_group_root,
      ray_origin_world,
      ray_dir_world,
      float(MJ_MAXVAL),
      wp.static(rc.enable_backface_culling),
    )

    if render_seg[camid] and geom_id != -1:
      if geom_id == -2:
        seg_out[worldid, seg_adr[camid] + rayid_local] = wp.vec2i(mesh_id, int(ObjType.FLEX))
      else:
        seg_out[worldid, seg_adr[camid] + rayid_local] = wp.vec2i(geom_id, int(ObjType.GEOM))

    # Early Out
    if geom_id == -1:
      if wp.static(rc.render_skybox) and render_rgb[camid]:
        skybox_color = sample_skybox(
          textures[wp.static(rc.skybox_tex_id)],
          wp.static(1.0 / float(rc.skybox_face_width)),
          ray_dir_world,
        )
        rgb_out[worldid, rgb_adr[camid] + rayid_local] = pack_rgba_to_uint32(
          skybox_color[0] * 255.0,
          skybox_color[1] * 255.0,
          skybox_color[2] * 255.0,
          255.0,
        )
      return

    if render_depth[camid]:
      # Planar depth: project Euclidean distance onto the camera's optical axis.
      # In camera-local coordinates, the optical axis is -Z. The Z-component of the
      # normalized ray direction is negative, so -ray_dir_local_cam[2] gives cos(θ)
      # between the ray and the optical axis.
      depth_out[worldid, depth_adr[camid] + rayid_local] = dist * (-ray_dir_local_cam[2])

    if not render_rgb[camid]:
      return

    # Shade the pixel
    hit_point = ray_origin_world + ray_dir_world * dist

    if geom_id == -2:
      # We encode flex_id in mesh_id for flex ray hits during cast_ray
      color = flex_rgba[mesh_id]
    elif geom_matid[worldid % geom_matid.shape[0], geom_id] == -1:
      color = geom_rgba[worldid % geom_rgba.shape[0], geom_id]
    else:
      color = mat_rgba[worldid % mat_rgba.shape[0], geom_matid[worldid % geom_matid.shape[0], geom_id]]

    base_color = wp.vec3(color[0], color[1], color[2])
    hit_color = base_color

    if wp.static(rc.use_textures):
      if geom_id != -2:
        mat_id = geom_matid[worldid % geom_matid.shape[0], geom_id]
        if mat_id >= 0:
          tex_id = mat_texid[worldid % mat_texid.shape[0], mat_id, 1]
          if tex_id >= 0:
            tex_color = sample_texture(
              geom_type,
              mesh_faceadr,
              geom_id,
              mat_texrepeat[worldid % mat_texrepeat.shape[0], mat_id],
              textures[tex_id],
              geom_xpos_in[worldid, geom_id],
              geom_xmat_in[worldid, geom_id],
              mesh_facetexcoord,
              mesh_texcoord,
              mesh_texcoord_offsets,
              hit_point,
              u,
              v,
              f,
              mesh_id,
            )
            base_color = wp.cw_mul(base_color, tex_color)

    result = wp.vec3(0.0, 0.0, 0.0)
    if wp.static(rc.use_ambient_lighting):
      len_n = wp.length(normal)
      n = normal if len_n > 0.0 else wp.vec3(0.0, 0.0, 1.0)
      n = wp.normalize(n)
      hemispheric = 0.5 * (n[2] + 1.0)
      ambient_color = wp.vec3(0.4, 0.4, 0.45) * hemispheric + wp.vec3(0.1, 0.1, 0.12) * (1.0 - hemispheric)
      result = 0.5 * wp.cw_mul(base_color, ambient_color)

    # Apply lighting and shadows
    for l in range(wp.static(m.nlight)):
      light_contribution = compute_lighting(
        geom_type,
        geom_dataid,
        geom_size,
        flex_vertadr,
        flex_edge,
        flex_radius,
        geom_xpos_in,
        geom_xmat_in,
        flexvert_xpos_in,
        use_shadows,
        bvh_id,
        group_root[worldid],
        bvh_ngeom,
        bvh_nflexgeom,
        enabled_geom_ids,
        worldid,
        mesh_bvh_id,
        hfield_bvh_id,
        flex_geom_flexid,
        flex_geom_edgeid,
        flex_bvh_id,
        flex_group_root,
        light_active[worldid % light_active.shape[0], l],
        light_type[worldid % light_type.shape[0], l],
        light_castshadow[worldid % light_castshadow.shape[0], l],
        light_xpos_in[worldid, l],
        light_xdir_in[worldid, l],
        normal,
        hit_point,
        wp.static(rc.enable_backface_culling),
      )
      result = result + base_color * light_contribution

    hit_color = wp.min(result, wp.vec3(1.0, 1.0, 1.0))
    hit_color = wp.max(hit_color, wp.vec3(0.0, 0.0, 0.0))

    rgb_out[worldid, rgb_adr[camid] + rayid_local] = pack_rgba_to_uint32(
      hit_color[0] * 255.0,
      hit_color[1] * 255.0,
      hit_color[2] * 255.0,
      255.0,
    )

  wp.launch(
    kernel=_render_megakernel,
    dim=(d.nworld, rc.total_rays),
    inputs=[
      m.geom_type,
      m.geom_dataid,
      m.geom_matid,
      m.geom_size,
      m.geom_rgba,
      m.cam_projection,
      m.cam_fovy,
      m.cam_sensorsize,
      m.cam_intrinsic,
      m.light_type,
      m.light_castshadow,
      m.light_active,
      m.flex_vertadr,
      m.flex_edge,
      m.flex_radius,
      m.mesh_faceadr,
      m.mat_texid,
      m.mat_texrepeat,
      m.mat_rgba,
      d.geom_xpos,
      d.geom_xmat,
      d.cam_xpos,
      d.cam_xmat,
      d.light_xpos,
      d.light_xdir,
      d.flexvert_xpos,
      rc.nrender,
      rc.use_shadows,
      rc.bvh_ngeom,
      rc.bvh_nflexgeom,
      rc.cam_res,
      rc.cam_id_map,
      rc.ray,
      rc.rgb_adr,
      rc.depth_adr,
      rc.seg_adr,
      rc.render_rgb,
      rc.render_depth,
      rc.render_seg,
      rc.bvh_id,
      rc.group_root,
      rc.flex_bvh_id,
      rc.flex_group_root,
      rc.enabled_geom_ids,
      rc.mesh_bvh_id,
      rc.mesh_facetexcoord,
      rc.mesh_texcoord,
      rc.mesh_texcoord_offsets,
      rc.hfield_bvh_id,
      rc.flex_rgba,
      rc.flex_geom_flexid,
      rc.flex_geom_edgeid,
      rc.textures,
    ],
    outputs=[
      rc.rgb_data,
      rc.depth_data,
      rc.seg_data,
    ],
    block_dim=m.block_dim.render,
  )
