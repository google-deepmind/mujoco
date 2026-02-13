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
from mujoco.mjx.third_party.mujoco_warp._src.types import RenderContext
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False})


# TODO(team): remove after mjwarp depends on warp-lang >= 1.12 in pyproject.toml
from mujoco.mjx.third_party.mujoco_warp._src.types import TEXTURE_DTYPE


@wp.func
def sample_texture(
  # Model:
  geom_type: wp.array(dtype=int),
  mesh_faceadr: wp.array(dtype=int),
  # In:
  geom_id: int,
  tex_repeat: wp.vec2,
  tex: TEXTURE_DTYPE,
  pos: wp.vec3,
  rot: wp.mat33,
  mesh_facetexcoord: wp.array(dtype=wp.vec3i),
  mesh_texcoord: wp.array(dtype=wp.vec2),
  mesh_texcoord_offsets: wp.array(dtype=int),
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


# TODO: Investigate combining cast_ray and cast_ray_first_hit
@wp.func
def cast_ray(
  # Model:
  geom_type: wp.array(dtype=int),
  geom_dataid: wp.array(dtype=int),
  geom_size: wp.array2d(dtype=wp.vec3),
  # Data in:
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  # In:
  bvh_id: wp.uint64,
  group_root: int,
  world_id: int,
  bvh_ngeom: int,
  enabled_geom_ids: wp.array(dtype=int),
  mesh_bvh_id: wp.array(dtype=wp.uint64),
  hfield_bvh_id: wp.array(dtype=wp.uint64),
  ray_origin_world: wp.vec3,
  ray_dir_world: wp.vec3,
) -> Tuple[int, float, wp.vec3, float, float, int, int]:
  dist = float(MJ_MAXVAL)
  normal = wp.vec3(0.0, 0.0, 0.0)
  geom_id = int(-1)
  bary_u = float(0.0)
  bary_v = float(0.0)
  face_idx = int(-1)
  geom_mesh_id = int(-1)

  query = wp.bvh_query_ray(bvh_id, ray_origin_world, ray_dir_world, group_root)
  bounds_nr = int(0)

  while wp.bvh_query_next(query, bounds_nr, dist):
    gi_global = bounds_nr
    gi_bvh_local = gi_global - (world_id * bvh_ngeom)
    gi = enabled_geom_ids[gi_bvh_local]

    hit_mesh_id = int(-1)
    u = float(0.0)
    v = float(0.0)
    f = int(-1)
    n = wp.vec3(0.0, 0.0, 0.0)

    # TODO: Investigate branch elimination with static loop unrolling
    if geom_type[gi] == GeomType.PLANE:
      d, n = ray_plane(
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.HFIELD:
      d, n, u, v, f, geom_hfield_id = ray_mesh_with_bvh(
        hfield_bvh_id,
        geom_dataid[gi],
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        ray_origin_world,
        ray_dir_world,
        dist,
      )
    if geom_type[gi] == GeomType.SPHERE:
      d, n = ray_sphere(
        geom_xpos_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi][0] * geom_size[world_id % geom_size.shape[0], gi][0],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.ELLIPSOID:
      d, n = ray_ellipsoid(
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.CAPSULE:
      d, n = ray_capsule(
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.CYLINDER:
      d, n = ray_cylinder(
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.BOX:
      d, all, n = ray_box(
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.MESH:
      d, n, u, v, f, hit_mesh_id = ray_mesh_with_bvh(
        mesh_bvh_id,
        geom_dataid[gi],
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        ray_origin_world,
        ray_dir_world,
        dist,
      )

    if d >= 0.0 and d < dist:
      dist = d
      normal = n
      geom_id = gi
      bary_u = u
      bary_v = v
      face_idx = f
      geom_mesh_id = hit_mesh_id

  return geom_id, dist, normal, bary_u, bary_v, face_idx, geom_mesh_id


@wp.func
def cast_ray_first_hit(
  # Model:
  geom_type: wp.array(dtype=int),
  geom_dataid: wp.array(dtype=int),
  geom_size: wp.array2d(dtype=wp.vec3),
  # Data in:
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  # In:
  bvh_id: wp.uint64,
  group_root: int,
  world_id: int,
  bvh_ngeom: int,
  enabled_geom_ids: wp.array(dtype=int),
  mesh_bvh_id: wp.array(dtype=wp.uint64),
  hfield_bvh_id: wp.array(dtype=wp.uint64),
  ray_origin_world: wp.vec3,
  ray_dir_world: wp.vec3,
  max_dist: float,
) -> bool:
  """A simpler version of casting rays that only checks for the first hit."""
  query = wp.bvh_query_ray(bvh_id, ray_origin_world, ray_dir_world, group_root)
  bounds_nr = int(0)

  while wp.bvh_query_next(query, bounds_nr, max_dist):
    gi_global = bounds_nr
    gi_bvh_local = gi_global - (world_id * bvh_ngeom)
    gi = enabled_geom_ids[gi_bvh_local]

    # TODO: Investigate branch elimination with static loop unrolling
    if geom_type[gi] == GeomType.PLANE:
      d, n = ray_plane(
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.HFIELD:
      d, n, u, v, f, geom_hfield_id = ray_mesh_with_bvh(
        hfield_bvh_id,
        geom_dataid[gi],
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        ray_origin_world,
        ray_dir_world,
        max_dist,
      )
    if geom_type[gi] == GeomType.SPHERE:
      d, n = ray_sphere(
        geom_xpos_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi][0] * geom_size[world_id % geom_size.shape[0], gi][0],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.ELLIPSOID:
      d, n = ray_ellipsoid(
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.CAPSULE:
      d, n = ray_capsule(
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.CYLINDER:
      d, n = ray_cylinder(
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.BOX:
      d, all, n = ray_box(
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        geom_size[world_id % geom_size.shape[0], gi],
        ray_origin_world,
        ray_dir_world,
      )
    if geom_type[gi] == GeomType.MESH:
      hit = ray_mesh_with_bvh_anyhit(
        mesh_bvh_id,
        geom_dataid[gi],
        geom_xpos_in[world_id, gi],
        geom_xmat_in[world_id, gi],
        ray_origin_world,
        ray_dir_world,
        max_dist,
      )
      d = 0.0 if hit else -1.0

    if d >= 0.0 and d < max_dist:
      return True

  return False


@wp.func
def compute_lighting(
  # Model:
  geom_type: wp.array(dtype=int),
  geom_dataid: wp.array(dtype=int),
  geom_size: wp.array2d(dtype=wp.vec3),
  # Data in:
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  # In:
  use_shadows: bool,
  bvh_id: wp.uint64,
  group_root: int,
  bvh_ngeom: int,
  enabled_geom_ids: wp.array(dtype=int),
  world_id: int,
  mesh_bvh_id: wp.array(dtype=wp.uint64),
  hfield_bvh_id: wp.array(dtype=wp.uint64),
  lightactive: bool,
  lighttype: int,
  lightcastshadow: bool,
  lightpos: wp.vec3,
  lightdir: wp.vec3,
  normal: wp.vec3,
  hitpoint: wp.vec3,
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
      spot_factor = wp.min(1.0, wp.max(0.0, (cos_theta - 0.85) / (0.95 - 0.85)))
      attenuation = attenuation * spot_factor

  ndotl = wp.max(0.0, wp.dot(normal, L))
  if ndotl == 0.0:
    return light_contribution

  visible = float(1.0)

  if use_shadows and lightcastshadow:
    # Nudge the origin slightly along the surface normal to avoid
    # self-intersection when casting shadow rays
    eps = 1.0e-4
    shadow_origin = hitpoint + normal * eps
    # Distance-limited shadows: cap by dist_to_light (for non-directional)
    max_t = float(dist_to_light - 1.0e-3)
    if lighttype == 1:  # directional light
      max_t = float(1.0e8)

    shadow_hit = cast_ray_first_hit(
      geom_type,
      geom_dataid,
      geom_size,
      geom_xpos_in,
      geom_xmat_in,
      bvh_id,
      group_root,
      world_id,
      bvh_ngeom,
      enabled_geom_ids,
      mesh_bvh_id,
      hfield_bvh_id,
      shadow_origin,
      L,
      max_t,
    )

    if shadow_hit:
      visible = 0.3

  return ndotl * attenuation * visible


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

  @wp.kernel(module="unique", enable_backward=False)
  def _render_megakernel(
    # Model:
    geom_type: wp.array(dtype=int),
    geom_dataid: wp.array(dtype=int),
    geom_matid: wp.array2d(dtype=int),
    geom_size: wp.array2d(dtype=wp.vec3),
    geom_rgba: wp.array2d(dtype=wp.vec4),
    cam_projection: wp.array(dtype=int),
    cam_fovy: wp.array2d(dtype=float),
    cam_sensorsize: wp.array(dtype=wp.vec2),
    cam_intrinsic: wp.array2d(dtype=wp.vec4),
    light_type: wp.array2d(dtype=int),
    light_castshadow: wp.array2d(dtype=bool),
    light_active: wp.array2d(dtype=bool),
    mesh_faceadr: wp.array(dtype=int),
    mat_texid: wp.array3d(dtype=int),
    mat_texrepeat: wp.array2d(dtype=wp.vec2),
    mat_rgba: wp.array2d(dtype=wp.vec4),
    # Data in:
    geom_xpos_in: wp.array2d(dtype=wp.vec3),
    geom_xmat_in: wp.array2d(dtype=wp.mat33),
    cam_xpos_in: wp.array2d(dtype=wp.vec3),
    cam_xmat_in: wp.array2d(dtype=wp.mat33),
    light_xpos_in: wp.array2d(dtype=wp.vec3),
    light_xdir_in: wp.array2d(dtype=wp.vec3),
    # In:
    nrender: int,
    use_shadows: bool,
    bvh_ngeom: int,
    cam_res: wp.array(dtype=wp.vec2i),
    cam_id_map: wp.array(dtype=int),
    ray: wp.array(dtype=wp.vec3),
    rgb_adr: wp.array(dtype=int),
    depth_adr: wp.array(dtype=int),
    render_rgb: wp.array(dtype=bool),
    render_depth: wp.array(dtype=bool),
    bvh_id: wp.uint64,
    group_root: wp.array(dtype=int),
    flex_bvh_id: wp.uint64,
    flex_group_root: wp.array(dtype=int),
    enabled_geom_ids: wp.array(dtype=int),
    mesh_bvh_id: wp.array(dtype=wp.uint64),
    mesh_facetexcoord: wp.array(dtype=wp.vec3i),
    mesh_texcoord: wp.array(dtype=wp.vec2),
    mesh_texcoord_offsets: wp.array(dtype=int),
    hfield_bvh_id: wp.array(dtype=wp.uint64),
    flex_rgba: wp.array(dtype=wp.vec4),
    # TODO: remove after mjwarp depends on warp-lang >= 1.12 in pyproject.toml
    textures: wp.array(dtype=TEXTURE_DTYPE),
    # Out:
    rgb_out: wp.array2d(dtype=wp.uint32),
    depth_out: wp.array2d(dtype=float),
  ):
    world_idx, ray_idx = wp.tid()

    # Map global ray_idx -> (cam_idx, ray_idx_local) using cumulative sizes
    cam_idx = int(-1)
    ray_idx_local = int(-1)
    accum = int(0)
    for i in range(nrender):
      num_i = cam_res[i][0] * cam_res[i][1]
      if ray_idx < accum + num_i:
        cam_idx = i
        ray_idx_local = ray_idx - accum
        break
      accum += num_i
    if cam_idx == -1 or ray_idx_local < 0:
      return

    if not render_rgb[cam_idx] and not render_depth[cam_idx]:
      return

    # Map active camera index to MuJoCo camera ID
    mujoco_cam_id = cam_id_map[cam_idx]

    if wp.static(rc.ray is None):
      img_w = cam_res[cam_idx][0]
      img_h = cam_res[cam_idx][1]
      px = ray_idx_local % img_w
      py = ray_idx_local // img_w
      ray_dir_local_cam = compute_ray(
        cam_projection[mujoco_cam_id],
        cam_fovy[world_idx % cam_fovy.shape[0], mujoco_cam_id],
        cam_sensorsize[mujoco_cam_id],
        cam_intrinsic[world_idx % cam_intrinsic.shape[0], mujoco_cam_id],
        img_w,
        img_h,
        px,
        py,
        wp.static(rc.znear),
      )
    else:
      ray_dir_local_cam = ray[ray_idx]

    ray_dir_world = cam_xmat_in[world_idx, mujoco_cam_id] @ ray_dir_local_cam
    ray_origin_world = cam_xpos_in[world_idx, mujoco_cam_id]

    geom_id, dist, normal, u, v, f, mesh_id = cast_ray(
      geom_type,
      geom_dataid,
      geom_size,
      geom_xpos_in,
      geom_xmat_in,
      bvh_id,
      group_root[world_idx],
      world_idx,
      bvh_ngeom,
      enabled_geom_ids,
      mesh_bvh_id,
      hfield_bvh_id,
      ray_origin_world,
      ray_dir_world,
    )

    if wp.static(m.nflex > 0):
      d, n, u, v, f = ray_flex_with_bvh(
        flex_bvh_id,
        flex_group_root[world_idx],
        ray_origin_world,
        ray_dir_world,
        dist,
      )
      if d >= 0.0 and d < dist:
        dist = d
        normal = n
        geom_id = -2

    # Early Out
    if geom_id == -1:
      return

    if render_depth[cam_idx]:
      depth_out[world_idx, depth_adr[cam_idx] + ray_idx_local] = dist

    if not render_rgb[cam_idx]:
      return

    # Shade the pixel
    hit_point = ray_origin_world + ray_dir_world * dist

    if geom_id == -2:
      # TODO: Currently flex textures are not supported, and only the first rgba value
      # is used until further flex support is added.
      color = flex_rgba[0]
    elif geom_matid[world_idx % geom_matid.shape[0], geom_id] == -1:
      color = geom_rgba[world_idx % geom_rgba.shape[0], geom_id]
    else:
      color = mat_rgba[world_idx % mat_rgba.shape[0], geom_matid[world_idx % geom_matid.shape[0], geom_id]]

    base_color = wp.vec3(color[0], color[1], color[2])
    hit_color = base_color

    if wp.static(rc.use_textures):
      if geom_id != -2:
        mat_id = geom_matid[world_idx % geom_matid.shape[0], geom_id]
        if mat_id >= 0:
          tex_id = mat_texid[world_idx % mat_texid.shape[0], mat_id, 1]
          if tex_id >= 0:
            tex_color = sample_texture(
              geom_type,
              mesh_faceadr,
              geom_id,
              mat_texrepeat[world_idx % mat_texrepeat.shape[0], mat_id],
              textures[tex_id],
              geom_xpos_in[world_idx, geom_id],
              geom_xmat_in[world_idx, geom_id],
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
        geom_xpos_in,
        geom_xmat_in,
        use_shadows,
        bvh_id,
        group_root[world_idx],
        bvh_ngeom,
        enabled_geom_ids,
        world_idx,
        mesh_bvh_id,
        hfield_bvh_id,
        light_active[world_idx % light_active.shape[0], l],
        light_type[world_idx % light_type.shape[0], l],
        light_castshadow[world_idx % light_castshadow.shape[0], l],
        light_xpos_in[world_idx, l],
        light_xdir_in[world_idx, l],
        normal,
        hit_point,
      )
      result = result + base_color * light_contribution

    hit_color = wp.min(result, wp.vec3(1.0, 1.0, 1.0))
    hit_color = wp.max(hit_color, wp.vec3(0.0, 0.0, 0.0))

    rgb_out[world_idx, rgb_adr[cam_idx] + ray_idx_local] = pack_rgba_to_uint32(
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
      rc.nrender,
      rc.use_shadows,
      rc.bvh_ngeom,
      rc.cam_res,
      rc.cam_id_map,
      rc.ray,
      rc.rgb_adr,
      rc.depth_adr,
      rc.render_rgb,
      rc.render_depth,
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
      rc.textures,
    ],
    outputs=[
      rc.rgb_data,
      rc.depth_data,
    ],
  )
