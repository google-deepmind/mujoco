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

import dataclasses
from typing import Tuple

import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src import render_optix
from mujoco.mjx.third_party.mujoco_warp._src.bvh import SPLAT_MIN_RESPONSE
from mujoco.mjx.third_party.mujoco_warp._src.ray import RAY_TOL_ABS
from mujoco.mjx.third_party.mujoco_warp._src.ray import RAY_TOL_REL
from mujoco.mjx.third_party.mujoco_warp._src.ray import geom_color
from mujoco.mjx.third_party.mujoco_warp._src.ray import make_ray_primitive
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_capsule
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_flex_with_bvh
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_flex_with_bvh_anyhit
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_mesh_with_bvh
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_mesh_with_bvh_anyhit
from mujoco.mjx.third_party.mujoco_warp._src.ray import ray_plane
from mujoco.mjx.third_party.mujoco_warp._src.render_optix import world_offset
from mujoco.mjx.third_party.mujoco_warp._src.render_util import TILE_ROWS
from mujoco.mjx.third_party.mujoco_warp._src.render_util import compute_ray
from mujoco.mjx.third_party.mujoco_warp._src.render_util import pack_rgba_to_uint32
from mujoco.mjx.third_party.mujoco_warp._src.render_util import tile_pixel
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MAXVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import ObjType
from mujoco.mjx.third_party.mujoco_warp._src.types import OptixScene
from mujoco.mjx.third_party.mujoco_warp._src.types import RenderContext
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope

wp.set_module_options({"enable_backward": False, "default_grid_stride": False})

# Limit each BVH traversal pass so splat compositing uses bounded local storage.
_MAX_SPLAT_HITS = 32
# Stop compositing once remaining light cannot affect an 8-bit output pixel.
_MIN_SPLAT_TRANSMITTANCE = 0.005
# Record splat surface depth once accumulated opacity exceeds 90%.
_SPLAT_DEPTH_TRANSMITTANCE = 0.1
# Cull splats whose alpha contribution rounds to zero in the 8-bit framebuffer.
_MIN_SPLAT_ALPHA = 1.0 / 255.0


@wp.func
def ray_splat(
  # In:
  position: wp.vec3,
  rotation: wp.quat,
  scale: wp.vec3,
  opacity: float,
  pnt: wp.vec3,
  vec: wp.vec3,
  min_distance: float,
  max_distance: float,
) -> Tuple[float, float]:
  """Returns the distance and alpha at which a ray intersects a splat."""
  inverse_rotation = math.quat_inv(rotation)
  lpnt = wp.cw_div(math.rot_vec_quat(pnt - position, inverse_rotation), scale)
  lvec = wp.cw_div(math.rot_vec_quat(vec, inverse_rotation), scale)

  distance = -wp.dot(lpnt, lvec) / wp.dot(lvec, lvec)
  if distance <= min_distance or distance >= max_distance:
    return -1.0, 0.0

  delta = lpnt + lvec * distance
  response = wp.exp(-0.5 * wp.dot(delta, delta))
  alpha = wp.min(response * opacity, 1.0)
  if alpha < wp.static(SPLAT_MIN_RESPONSE) or alpha < wp.static(_MIN_SPLAT_ALPHA):
    return -1.0, 0.0
  return distance, alpha


@wp.func
def shade_splats(
  # In:
  splat_position: wp.array[wp.vec3],
  splat_rotation: wp.array[wp.quat],
  splat_scale: wp.array[wp.vec3],
  splat_rgba: wp.array[wp.vec4],
  bvh_id: wp.uint64,
  group_root: int,
  ray_origin: wp.vec3,
  ray_direction: wp.vec3,
  max_distance: float,
) -> tuple[wp.vec3, float, float]:
  min_distance = float(0.0)
  transmittance = float(1.0)
  color = wp.vec3(0.0)
  depth = float(-1.0)

  hit_distances = wp.vector(MJ_MAXVAL, length=_MAX_SPLAT_HITS, dtype=float)
  hit_indices = wp.vector(-1, length=_MAX_SPLAT_HITS, dtype=int)
  hit_alphas = wp.vector(0.0, length=_MAX_SPLAT_HITS, dtype=float)

  while transmittance > wp.static(_MIN_SPLAT_TRANSMITTANCE):
    num_hits = int(0)
    for i in range(wp.static(_MAX_SPLAT_HITS)):
      hit_distances[i] = max_distance
      hit_indices[i] = -1
      hit_alphas[i] = 0.0

    index = int(0)
    query = wp.bvh_query_ray(bvh_id, ray_origin, ray_direction, group_root)
    while wp.bvh_query_next(query, index, hit_distances[_MAX_SPLAT_HITS - 1]):
      distance, alpha = ray_splat(
        splat_position[index],
        splat_rotation[index],
        splat_scale[index],
        splat_rgba[index][3],
        ray_origin,
        ray_direction,
        min_distance,
        max_distance,
      )
      if distance > 0.0:
        if num_hits < wp.static(_MAX_SPLAT_HITS):
          num_hits += 1
        for i in range(num_hits):
          if distance < hit_distances[i]:
            for j in range(num_hits - 1, i, -1):
              hit_distances[j] = hit_distances[j - 1]
              hit_indices[j] = hit_indices[j - 1]
              hit_alphas[j] = hit_alphas[j - 1]
            hit_distances[i] = distance
            hit_indices[i] = index
            hit_alphas[i] = alpha
            break

    if num_hits == 0:
      break

    for i in range(num_hits):
      index = hit_indices[i]
      alpha = hit_alphas[i]
      color += wp.vec3(splat_rgba[index][0], splat_rgba[index][1], splat_rgba[index][2]) * alpha * transmittance
      transmittance *= 1.0 - alpha
      if depth < 0.0 and transmittance < wp.static(_SPLAT_DEPTH_TRANSMITTANCE):
        depth = hit_distances[i]

    if num_hits < wp.static(_MAX_SPLAT_HITS):
      break
    min_distance = hit_distances[_MAX_SPLAT_HITS - 1] + 1.0e-6

  return color, transmittance, depth


# Default value for mat_shininess in MuJoCo is 0.5
# With an 8 bit image format, the maximum value is 255.0
# So max shininess value for the Phong lighting model is 128.0
MAX_SHININESS = 128.0
# The exponent value for mat_shininess is 0.5 times the max shininess value
DEFAULT_MAT_SHININESS_EXPONENT = 0.5 * MAX_SHININESS

# Default value for mat_specular in MuJoCo is 0.5
DEFAULT_MAT_SPECULAR = 0.5

# Default value for mat_emission in MuJoCo is 0.0
DEFAULT_MAT_EMISSION = 0.0

NO_LIGHT_AMBIENT_FALLBACK = 0.3


@wp.func
def _texture_plane(
  # In:
  local: wp.vec3,
  size: wp.vec3,
  repeat: wp.vec2,
  tex_uniform: bool,
) -> Tuple[wp.vec2, wp.vec2, wp.vec2]:
  # Evaluate X and Y independently to support mixed finite/infinite planes
  # (render_context.c lines 211 & 219)
  u = 0.5 * local[0]
  rep_u = repeat[0]
  if size[0] > 0.0:
    s0 = wp.max(size[0], 1e-6)
    u = (local[0] + size[0]) / (2.0 * s0)
    if tex_uniform:
      rep_u = repeat[0] * size[0]

  v = -0.5 * local[1]
  rep_v = repeat[1]
  if size[1] > 0.0:
    s1 = wp.max(size[1], 1e-6)
    v = (size[1] - local[1]) / (2.0 * s1)
    if tex_uniform:
      rep_v = repeat[1] * size[1]

  off = -0.5 if (size[0] <= 0.0 or size[1] <= 0.0) else 0.0
  return wp.vec2(u, v), wp.vec2(off, off), wp.vec2(rep_u, rep_v)


@wp.func
def _texture_sphere_ellipsoid(
  # In:
  local: wp.vec3,
  size: wp.vec3,
  repeat: wp.vec2,
  tex_uniform: bool,
  is_sphere: bool,
) -> Tuple[wp.vec2, wp.vec2, wp.vec2]:
  radii = size
  if is_sphere:
    radii = wp.vec3(size[0], size[0], size[0])
  rx = wp.max(radii[0], 1e-6)
  ry = wp.max(radii[1], 1e-6)
  rz = wp.max(radii[2], 1e-6)
  pt = wp.vec3(local[0] / rx, local[1] / ry, local[2] / rz)
  inv_len = 1.0 / wp.max(wp.length(pt), 1e-6)
  d = pt * inv_len

  azimuth = wp.atan2(d[1], d[0])
  if azimuth < 0.0:
    azimuth += wp.static(2.0 * wp.pi)
  elevation = wp.asin(wp.clamp(d[2], -1.0, 1.0))
  u = azimuth * wp.static(0.5 / wp.pi)
  v = 0.5 - elevation * wp.static(1.0 / wp.pi)
  rep = repeat
  if tex_uniform:
    rep = wp.vec2(repeat[0] * radii[0], repeat[1] * radii[1])
  return wp.vec2(u, v), wp.vec2(0.0, 0.0), rep


@wp.func
def _texture_cylinder(
  # In:
  local: wp.vec3,
  local_normal: wp.vec3,
  size: wp.vec3,
  repeat: wp.vec2,
  tex_uniform: bool,
) -> Tuple[wp.vec2, wp.vec2, wp.vec2]:
  r = wp.max(size[0], 1e-6)
  h = wp.max(size[1], 1e-6)
  if wp.abs(local_normal[2]) > 0.7071:
    # Top and bottom caps match MuJoCo disk(+1) and disk(-1)
    uv = wp.vec2(0.5 + 0.5 * (local[0] / r), 0.5 + 0.5 * (local[1] / r))
  else:
    # Cylindrical body
    azimuth = wp.atan2(local[1], local[0])
    if azimuth < 0.0:
      azimuth += wp.static(2.0 * wp.pi)
    u = azimuth * wp.static(0.5 / wp.pi)
    v = (h - local[2]) / (2.0 * h)
    uv = wp.vec2(u, v)
  rep = repeat
  if tex_uniform:
    rep = wp.vec2(repeat[0] * size[0], repeat[1] * size[0])
  return uv, wp.vec2(0.0, 0.0), rep


@wp.func
def _texture_capsule(
  # In:
  local: wp.vec3,
  size: wp.vec3,
  repeat: wp.vec2,
  tex_uniform: bool,
) -> Tuple[wp.vec2, wp.vec2, wp.vec2]:
  r = wp.max(size[0], 1e-6)
  h = wp.max(size[1], 1e-6)
  z = local[2]
  azimuth = wp.atan2(local[1], local[0])
  if azimuth < 0.0:
    azimuth += wp.static(2.0 * wp.pi)
  u = azimuth * wp.static(0.5 / wp.pi)

  if z > h:
    dz = wp.clamp((z - h) / r, 0.0, 1.0)
    v = 1.0 - wp.asin(dz) * wp.static(2.0 / wp.pi)
  elif z < -h:
    dz = wp.clamp((-h - z) / r, 0.0, 1.0)
    v = wp.asin(dz) * wp.static(2.0 / wp.pi)
  else:
    v = (h - z) / (2.0 * h)
  rep = repeat
  if tex_uniform:
    rep = wp.vec2(repeat[0] * size[0], repeat[1] * size[0])
  return wp.vec2(u, v), wp.vec2(0.0, 0.0), rep


@wp.func
def _texture_box(
  # In:
  local: wp.vec3,
  local_normal: wp.vec3,
  size: wp.vec3,
  repeat: wp.vec2,
  tex_uniform: bool,
) -> Tuple[wp.vec2, wp.vec2, wp.vec2]:
  sx = wp.max(size[0], 1e-6)
  sy = wp.max(size[1], 1e-6)
  sz = wp.max(size[2], 1e-6)
  ax = wp.abs(local_normal[0])
  ay = wp.abs(local_normal[1])
  az = wp.abs(local_normal[2])

  if az >= ax and az >= ay:
    if local_normal[2] > 0.0:
      uv = wp.vec2((local[0] + sx) / (2.0 * sx), (sy - local[1]) / (2.0 * sy))
    else:
      uv = wp.vec2((local[0] + sx) / (2.0 * sx), (local[1] + sy) / (2.0 * sy))
  elif ax >= ay and ax >= az:
    if local_normal[0] > 0.0:
      uv = wp.vec2((local[1] + sy) / (2.0 * sy), (sz - local[2]) / (2.0 * sz))
    else:
      uv = wp.vec2((sy - local[1]) / (2.0 * sy), (sz - local[2]) / (2.0 * sz))
  else:
    if local_normal[1] < 0.0:
      uv = wp.vec2((local[0] + sx) / (2.0 * sx), (sz - local[2]) / (2.0 * sz))
    else:
      uv = wp.vec2((sx - local[0]) / (2.0 * sx), (sz - local[2]) / (2.0 * sz))

  rep = repeat
  if tex_uniform:
    rep = wp.vec2(repeat[0] * size[0], repeat[1] * size[1])
  return uv, wp.vec2(0.0, 0.0), rep


@wp.func
def _texture_hfield(
  # In:
  local: wp.vec3,
  size: wp.vec3,
  repeat: wp.vec2,
  tex_uniform: bool,
) -> Tuple[wp.vec2, wp.vec2, wp.vec2]:
  # Heightfield horizontal extent is [-size[0], size[0]] x [-size[1], size[1]]
  # render_context.c lines 465-500
  s0 = wp.max(size[0], 1e-6)
  s1 = wp.max(size[1], 1e-6)
  uv = wp.vec2(
    (local[0] + size[0]) / (2.0 * s0),
    (size[1] - local[1]) / (2.0 * s1),
  )
  rep = repeat
  if tex_uniform:
    rep = wp.vec2(repeat[0] * size[0], repeat[1] * size[1])
  return uv, wp.vec2(0.0, 0.0), rep


@wp.func
def _texture_mesh(
  # Model:
  mesh_faceadr: wp.array[int],
  # In:
  local: wp.vec3,
  size: wp.vec3,
  repeat: wp.vec2,
  tex_uniform: bool,
  f: int,
  mesh_id: int,
  bary_u: float,
  bary_v: float,
  mesh_facetexcoord: wp.array[wp.vec3i],
  mesh_texcoord: wp.array[wp.vec2],
  mesh_texcoord_offsets: wp.array[int],
) -> Tuple[wp.vec2, wp.vec2, wp.vec2]:
  has_uv = False
  uv = wp.vec2(0.0, 0.0)
  offset = wp.vec2(0.0, 0.0)
  rep = repeat

  if f >= 0 and mesh_id >= 0:
    texcoord_offset = mesh_texcoord_offsets[mesh_id]
    if texcoord_offset >= 0:
      face_adr = mesh_faceadr[mesh_id] + f
      coords = mesh_facetexcoord[face_adr]
      # OBJ faces can omit individual UV indices even when the mesh has texcoords.
      if coords[0] >= 0 and coords[1] >= 0 and coords[2] >= 0:
        uv0 = mesh_texcoord[texcoord_offset + coords[0]]
        uv1 = mesh_texcoord[texcoord_offset + coords[1]]
        uv2 = mesh_texcoord[texcoord_offset + coords[2]]
        uv = uv0 * bary_u + uv1 * bary_v + uv2 * (1.0 - bary_u - bary_v)
        has_uv = True

  if not has_uv:
    # Fallback to OBJECT_PLANE texgen for untextured mesh (render_gl3.c:163-200)
    uv = wp.vec2(0.5 * local[0], -0.5 * local[1])
    offset = wp.vec2(-0.5, -0.5)
    if size[0] > 0.0:
      rep = wp.vec2(rep[0] / size[0], rep[1])
    if size[1] > 0.0:
      rep = wp.vec2(rep[0], rep[1] / size[1])
    if tex_uniform:
      if size[0] > 0.0:
        rep = wp.vec2(rep[0] * size[0], rep[1])
      if size[1] > 0.0:
        rep = wp.vec2(rep[0], rep[1] * size[1])

  return uv, offset, rep


def _make_sample_texture(geom_ray_types: Tuple[int]) -> wp.Function:
  """Build a texture-sampling func specialized to the geom types present in the scene.

  geom_ray_types is the set of GeomType int values that actually occur, so the
  per-type texture mapping branches for absent types are eliminated at compile time
  via wp.static, avoiding the register pressure of unreachable code paths.
  """

  @wp.func
  def sample_texture(
    # Model:
    geom_type: wp.array[int],
    mesh_faceadr: wp.array[int],
    # In:
    geom_id: int,
    size: wp.vec3,
    tex_repeat: wp.vec2,
    tex_uniform: bool,
    tex: wp.Texture2D,
    pos: wp.vec3,
    rot: wp.mat33,
    mesh_facetexcoord: wp.array[wp.vec3i],
    mesh_texcoord: wp.array[wp.vec2],
    mesh_texcoord_offsets: wp.array[int],
    hit_point: wp.vec3,
    normal: wp.vec3,
    bary_u: float,
    bary_v: float,
    f: int,
    mesh_id: int,
  ) -> wp.vec3:
    uv = wp.vec2(0.0, 0.0)
    offset = wp.vec2(0.0, 0.0)
    repeat = tex_repeat
    gtype = geom_type[geom_id]

    local = wp.transpose(rot) @ (hit_point - pos)
    local_normal = wp.transpose(rot) @ normal

    if wp.static(int(GeomType.PLANE) in geom_ray_types):
      if gtype == GeomType.PLANE:
        uv, offset, repeat = _texture_plane(local, size, repeat, tex_uniform)

    if wp.static(int(GeomType.SPHERE) in geom_ray_types or int(GeomType.ELLIPSOID) in geom_ray_types):
      if gtype == GeomType.SPHERE or gtype == GeomType.ELLIPSOID:
        uv, offset, repeat = _texture_sphere_ellipsoid(local, size, repeat, tex_uniform, gtype == GeomType.SPHERE)

    if wp.static(int(GeomType.CYLINDER) in geom_ray_types):
      if gtype == GeomType.CYLINDER:
        uv, offset, repeat = _texture_cylinder(local, local_normal, size, repeat, tex_uniform)

    if wp.static(int(GeomType.CAPSULE) in geom_ray_types):
      if gtype == GeomType.CAPSULE:
        uv, offset, repeat = _texture_capsule(local, size, repeat, tex_uniform)

    if wp.static(int(GeomType.BOX) in geom_ray_types):
      if gtype == GeomType.BOX:
        uv, offset, repeat = _texture_box(local, local_normal, size, repeat, tex_uniform)

    if wp.static(int(GeomType.HFIELD) in geom_ray_types):
      if gtype == GeomType.HFIELD:
        uv, offset, repeat = _texture_hfield(local, size, repeat, tex_uniform)

    if wp.static(int(GeomType.MESH) in geom_ray_types or int(GeomType.SDF) in geom_ray_types):
      if gtype == GeomType.MESH or gtype == GeomType.SDF:
        uv, offset, repeat = _texture_mesh(
          mesh_faceadr,
          local,
          size,
          repeat,
          tex_uniform,
          f,
          mesh_id,
          bary_u,
          bary_v,
          mesh_facetexcoord,
          mesh_texcoord,
          mesh_texcoord_offsets,
        )

    u = uv[0] * repeat[0] + offset[0]
    v = uv[1] * repeat[1] + offset[1]
    u = u - wp.floor(u)
    v = v - wp.floor(v)
    tex_color = wp.texture_sample(tex, wp.vec2(u, v), dtype=wp.vec4)
    return wp.vec3(tex_color[0], tex_color[1], tex_color[2])

  return sample_texture


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


@wp.func
def vertex_normal(n: wp.vec3, face: wp.vec3) -> wp.vec3:
  # Matches mjr_uploadMesh: a vertex normal more than ~37 degrees off the face is unusable.
  if wp.dot(n, face) < 0.8:
    return face
  return n


# TODO(team): consider precomputing transparent geom/flex filtering
def _make_cast_ray(
  geom_ray_types: Tuple[int],
  first_hit: bool = False,
  has_reframed_meshes: bool = False,
  optix: OptixScene | None = None,
) -> wp.Function:
  """Build closest-hit or shadow-ray traversal specialized to the scene and backend."""
  ray_primitive = make_ray_primitive(geom_ray_types)
  # Extract scalar fields so cast_ray's closure does not retain OptixScene.
  optix_extent, optix_grid, optix_planes = (optix.extent, optix.grid, optix.planes) if optix is not None else (0.0, 0, ())
  wo = render_optix.get_wo() if optix is not None else None

  @wp.func
  def cast_ray(
    # Model:
    geom_type: wp.array[int],
    geom_dataid: wp.array2d[int],
    geom_matid: wp.array2d[int],
    geom_size: wp.array2d[wp.vec3],
    geom_rgba: wp.array2d[wp.vec4],
    flex_vertadr: wp.array[int],
    flex_edge: wp.array[wp.vec2i],
    flex_radius: wp.array[float],
    mat_rgba: wp.array2d[wp.vec4],
    # Data in:
    geom_xpos_in: wp.array2d[wp.vec3],
    geom_xquat_in: wp.array2d[wp.quat],
    flexvert_xpos_in: wp.array2d[wp.vec3],
    # In:
    bvh_id: wp.uint64,
    group_root: int,
    worldid: int,
    bvh_ngeom: int,
    flex_bvh_ngeom: int,
    enabled_geom_ids: wp.array[int],
    mesh_bvh_id: wp.array[wp.uint64],
    mesh_bvh_rot: wp.array[wp.mat33],
    hfield_bvh_id: wp.array[wp.uint64],
    flex_rgba: wp.array[wp.vec4],
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

    if wp.static(wo is not None):
      # Intersect the world's cell before traversing, including rays from cameras outside it.
      tmin = float(0.0)
      tmax = max_dist
      intersects_cell = True
      for axis in range(3):
        if ray_dir_world[axis] != 0.0:
          a = (-wp.static(optix_extent) - ray_origin_world[axis]) / ray_dir_world[axis]
          b = (wp.static(optix_extent) - ray_origin_world[axis]) / ray_dir_world[axis]
          tmin = wp.max(tmin, wp.min(a, b))
          tmax = wp.min(tmax, wp.max(a, b))
        elif wp.abs(ray_origin_world[axis]) > wp.static(optix_extent):
          intersects_cell = False
      intersects_cell = intersects_cell and tmax >= tmin
      tmax = wp.max(0.0, tmax)

      origin = ray_origin_world + world_offset(worldid, wp.static(optix_grid), wp.static(3.0 * optix_extent))

      if intersects_cell:
        if wp.static(first_hit):
          # Shadow rays only test for an occluder, so they stop at the first hit and skip surface
          # data. Mesh shadows are two-sided, like the BVH any-hit cast.
          if wo.accel_query_ray_anyhit(bvh_id, origin, ray_dir_world, tmax, tmin, 0xFF):
            return 0, 0.0, normal, bary_u, bary_v, face_idx, geom_mesh_id
        else:
          ray_flags = wp.uint32(0)
          if cull_backfaces:
            ray_flags = wp.uint32(wp.static(int(wo.RayFlags.CULL_BACK_FACING_TRIANGLES)))
          query = wo.accel_query_ray(bvh_id, origin, ray_dir_world, tmax, tmin, 0xFF, ray_flags)
          if query.result:
            geom_id = int(query.instance_id) - worldid * geom_type.shape[0]
            dist = query.t
            # Custom normals use three attributes; the query holds the first two.
            is_triangle = False
            if wp.static(any(t in geom_ray_types for t in (GeomType.MESH, GeomType.SDF, GeomType.HFIELD))):
              gtype = geom_type[geom_id]
              is_triangle = gtype == GeomType.MESH or gtype == GeomType.SDF or gtype == GeomType.HFIELD
              if is_triangle:
                face_idx = int(query.primitive_index)
                dataid = geom_dataid[worldid % geom_dataid.shape[0], geom_id]
                mesh = wp.uint64(0)
                mat = math.quat_to_mat(geom_xquat_in[worldid, geom_id])
                if wp.static(int(GeomType.HFIELD) in geom_ray_types):
                  if gtype == GeomType.HFIELD:
                    mesh = hfield_bvh_id[dataid]
                if wp.static(int(GeomType.MESH) in geom_ray_types or int(GeomType.SDF) in geom_ray_types):
                  if gtype != GeomType.HFIELD:
                    mesh = mesh_bvh_id[dataid]
                    geom_mesh_id = dataid
                    if wp.static(has_reframed_meshes):
                      mat = mat @ mesh_bvh_rot[dataid]
                normal = wp.normalize(mat @ wp.mesh_eval_face_normal(mesh, face_idx))
                bary_u = 1.0 - query.u - query.v
                bary_v = query.u
            if not is_triangle:
              normal = wp.vec3(query.u, query.v, wp.optix_uint32_as_float(wp.optix_hit_object_get_attribute(2)))

      # Planes are unbounded, so they cannot live in a world cell; intersect them directly.
      # ray_plane only reports front-face hits, so planes need no separate backface cull.
      for i in range(wp.static(len(optix_planes))):
        gi = wp.static(optix_planes[i])
        rgba, _ = geom_color(geom_matid, geom_rgba, mat_rgba, worldid, gi)
        if rgba[3] != 0.0:
          d, n = ray_plane(
            geom_xpos_in[worldid, gi],
            math.quat_to_mat(geom_xquat_in[worldid, gi]),
            geom_size[worldid % geom_size.shape[0], gi],
            ray_origin_world,
            ray_dir_world,
          )
          if wp.static(first_hit):
            if d >= 0.0 and d < dist:
              return gi, d, n, 0.0, 0.0, -1, -1
          else:
            tol = RAY_TOL_REL * wp.max(dist, 1.0)
            closer = (d < dist - tol) or (wp.abs(d - dist) <= tol and (geom_id < 0 or gi > geom_id))
            if d >= 0.0 and closer:
              geom_id, dist, normal, bary_u, bary_v, face_idx, geom_mesh_id = gi, d, n, 0.0, 0.0, -1, -1
    else:
      query = wp.bvh_query_ray(bvh_id, ray_origin_world, ray_dir_world, group_root)
      bounds_nr = int(0)
      ngeom = bvh_ngeom + flex_bvh_ngeom

      # max_t is exclusive: widen so coincident hits reach the tie-break below.
      while wp.bvh_query_next(query, bounds_nr, dist * (1.0 + RAY_TOL_REL) + RAY_TOL_ABS):
        gi_global = bounds_nr
        local_id = gi_global - (worldid * ngeom)

        query_dist = dist * (1.0 + RAY_TOL_REL) + RAY_TOL_ABS

        d = float(-1.0)
        hit_mesh_id = int(-1)
        u = float(0.0)
        v = float(0.0)
        f = int(-1)
        n = wp.vec3(0.0, 0.0, 0.0)
        hit_geom_id = int(-1)

        if local_id < bvh_ngeom:
          gi = enabled_geom_ids[local_id]
          rgba, _ = geom_color(geom_matid, geom_rgba, mat_rgba, worldid, gi)
          if rgba[3] == 0.0:
            continue
          gtype = geom_type[gi]
          d, n = ray_primitive(
            geom_xpos_in[worldid, gi],
            math.quat_to_mat(geom_xquat_in[worldid, gi]),
            geom_size[worldid % geom_size.shape[0], gi],
            ray_origin_world,
            ray_dir_world,
            gtype,
          )
        else:
          gi = local_id - bvh_ngeom
          gtype = GeomType.FLEX

        hit_geom_id = gi

        if wp.static(any(t in geom_ray_types for t in (GeomType.HFIELD, GeomType.MESH, GeomType.SDF))):
          if gtype == GeomType.HFIELD or gtype == GeomType.MESH or gtype == GeomType.SDF:
            meshes = mesh_bvh_id
            if wp.static(int(GeomType.HFIELD) in geom_ray_types):
              if gtype == GeomType.HFIELD:
                meshes = hfield_bvh_id
            dataid = geom_dataid[worldid % geom_dataid.shape[0], gi]
            mesh_mat = math.quat_to_mat(geom_xquat_in[worldid, gi])
            if wp.static(has_reframed_meshes):
              if gtype != GeomType.HFIELD:
                mesh_mat = mesh_mat @ mesh_bvh_rot[dataid]
            if wp.static(first_hit):
              hit = ray_mesh_with_bvh_anyhit(
                meshes,
                dataid,
                geom_xpos_in[worldid, gi],
                mesh_mat,
                ray_origin_world,
                ray_dir_world,
                dist,
              )
              d = 0.0 if hit else -1.0
            else:
              d, n, u, v, f, mesh_id = ray_mesh_with_bvh(
                meshes,
                dataid,
                geom_xpos_in[worldid, gi],
                mesh_mat,
                ray_origin_world,
                ray_dir_world,
                query_dist,
                cull_backfaces,
              )
              if gtype != GeomType.HFIELD:
                hit_mesh_id = mesh_id
        if wp.static(int(GeomType.FLEX) in geom_ray_types):
          if gtype == GeomType.FLEX:
            hit_geom_id = -2
            flexid = flex_geom_flexid[gi]
            if flex_rgba[flexid][3] == 0.0:
              continue
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
                d, n, u, v, f = ray_flex_with_bvh(flex_bvh_id, flexid, flex_gr, ray_origin_world, ray_dir_world, query_dist)
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
          # Ties go to the higher geom index, like MuJoCo's GL depth test.
          tol = RAY_TOL_REL * wp.max(dist, 1.0)
          closer = (d < dist - tol) or (wp.abs(d - dist) <= tol and (geom_id < 0 or hit_geom_id > geom_id))
          if d >= 0.0 and closer:
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
    geom_matid: wp.array2d[int],
    geom_size: wp.array2d[wp.vec3],
    geom_rgba: wp.array2d[wp.vec4],
    flex_vertadr: wp.array[int],
    flex_edge: wp.array[wp.vec2i],
    flex_radius: wp.array[float],
    mat_rgba: wp.array2d[wp.vec4],
    # Data in:
    geom_xpos_in: wp.array2d[wp.vec3],
    geom_xquat_in: wp.array2d[wp.quat],
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
    mesh_bvh_rot: wp.array[wp.mat33],
    hfield_bvh_id: wp.array[wp.uint64],
    flex_rgba: wp.array[wp.vec4],
    flex_geom_flexid: wp.array[int],
    flex_geom_edgeid: wp.array[int],
    flex_bvh_id: wp.array[wp.uint64],
    flex_group_root: wp.array2d[int],
    lightactive: bool,
    lighttype: int,
    lightcastshadow: bool,
    lightpos: wp.vec3,
    lightdir: wp.vec3,
    lightattenuation: wp.vec3,
    lightcutoff_rad: float,
    lightexp: float,
    lightdiff: wp.vec3,
    lightspec: wp.vec3,
    normal: wp.vec3,
    hitpoint: wp.vec3,
    view_dir: wp.vec3,
    mat_spec: float,
    mat_shin_exp: float,
    cull_backfaces: bool,
    shadow_light_fraction: float,
    enable_specular: bool,
    default_attenuation: bool,
    has_spot: bool,
  ) -> Tuple[wp.vec3, wp.vec3]:
    diff_rgb = wp.vec3(0.0)
    spec_rgb = wp.vec3(0.0)

    # TODO: We should probably only be looping over active lights
    # in the first place with a static loop of enabled light idx?
    if not lightactive:
      return diff_rgb, spec_rgb

    L = wp.vec3(0.0)
    dist_to_light = float(MJ_MAXVAL)
    attenuation = 1.0

    if lighttype == 1:  # directional light
      # MuJoCo guarantees `lightdir` is unit length.
      L = -lightdir
    else:
      L, dist_to_light = math.normalize_with_norm(lightpos - hitpoint)
      if not default_attenuation:
        light_attenuation_factor = wp.vec3(1.0, dist_to_light, dist_to_light * dist_to_light)
        attenuation = math.safe_div(1.0, wp.dot(light_attenuation_factor, lightattenuation))
      if has_spot:
        if lighttype == 0:  # spot light
          cos_theta = wp.dot(-L, lightdir)
          cos_cutoff = wp.cos(lightcutoff_rad)
          if cos_theta < cos_cutoff:
            return diff_rgb, spec_rgb
          attenuation = attenuation * wp.pow(wp.max(cos_theta, 0.0), lightexp)

    ndotl = wp.max(0.0, wp.dot(normal, L))
    if ndotl == 0.0:
      return diff_rgb, spec_rgb

    visible = 1.0

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
        geom_matid,
        geom_size,
        geom_rgba,
        flex_vertadr,
        flex_edge,
        flex_radius,
        mat_rgba,
        geom_xpos_in,
        geom_xquat_in,
        flexvert_xpos_in,
        bvh_id,
        group_root,
        worldid,
        bvh_ngeom,
        bvh_nflexgeom,
        enabled_geom_ids,
        mesh_bvh_id,
        mesh_bvh_rot,
        hfield_bvh_id,
        flex_rgba,
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
        visible = shadow_light_fraction

    weight = attenuation * visible
    diff_rgb = lightdiff * (ndotl * weight)
    if enable_specular:
      if mat_spec > 0.0 and mat_shin_exp > 0.0:
        H = wp.normalize(L + view_dir)
        ndoth = wp.max(0.0, wp.dot(normal, H))
        spec_rgb = lightspec * (mat_spec * wp.pow(ndoth, mat_shin_exp) * weight)

    return diff_rgb, spec_rgb

  return compute_lighting


def _build_megakernel(nlight: int, rc: RenderContext, block_dim: int = 0):
  """Construct the specialised megakernel for this context."""
  has_splats = rc.splat_count > 0

  # Specialize the ray-cast helpers to the geom types present in the scene so the
  # compiler eliminates intersection branches for absent types.
  geom_ray_types = rc.geom_ray_types
  cast_ray = _make_cast_ray(geom_ray_types, False, rc.has_reframed_meshes, rc.optix)
  cast_ray_first_hit = _make_cast_ray(geom_ray_types, True, rc.has_reframed_meshes, rc.optix)
  compute_lighting = _make_compute_lighting(cast_ray_first_hit)
  sample_texture = _make_sample_texture(geom_ray_types)

  # Static parameters extracted for JAX FFI closure.
  rc_static = {f.name: getattr(rc, f.name) for f in dataclasses.fields(rc) if f.type in (int, wp.uint32, bool, float, wp.vec3)}
  rc_static["enable_specular_or_emission"] = rc.enable_specular or rc.enable_emission
  bg = int(rc.background_color)
  rc_static["background_color_vec3"] = wp.vec3(
    float((bg >> 16) & 0xFF) / 255.0,
    float((bg >> 8) & 0xFF) / 255.0,
    float(bg & 0xFF) / 255.0,
  )
  M_NLIGHT = nlight
  nsamples = rc.samples_per_pixel * rc.samples_per_pixel
  hoist_skybox = rc.any_rgb and rc.render_skybox and rc.backend == "warp"
  n_aa = rc.samples_per_pixel
  # Hoist tile_rows before defining _render_megakernel so its closure does not retain rc.
  tile_rows = TILE_ROWS[rc.backend]
  has_mesh_bvh = bool({int(GeomType.MESH), int(GeomType.SDF), int(GeomType.HFIELD), int(GeomType.FLEX)} & set(geom_ray_types))
  # OptiX caps registers in its pipeline instead.
  cap_registers = block_dim > 0 and nsamples == 1 and not has_mesh_bvh and not has_splats and not rc.use_shadows and nlight <= 2

  @wp.kernel(
    module="unique",
    enable_backward=False,
    grid_stride=False,
    module_options={"fast_math": rc.use_fast_math, "max_unroll": 6},
    launch_bounds=(block_dim, max(1, 640 // block_dim)) if cap_registers else None,
  )
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
    light_attenuation: wp.array2d[wp.vec3],
    light_cutoff: wp.array2d[float],
    light_exponent: wp.array2d[float],
    light_ambient: wp.array2d[wp.vec3],
    light_diffuse: wp.array2d[wp.vec3],
    light_specular: wp.array2d[wp.vec3],
    flex_vertadr: wp.array[int],
    flex_edge: wp.array[wp.vec2i],
    flex_radius: wp.array[float],
    mesh_faceadr: wp.array[int],
    mesh_normaladr: wp.array[int],
    mesh_normal: wp.array[wp.vec3],
    mat_texid: wp.array3d[int],
    mat_texuniform: wp.array2d[bool],
    mat_texrepeat: wp.array2d[wp.vec2],
    mat_emission: wp.array2d[float],
    mat_specular: wp.array2d[float],
    mat_shininess: wp.array2d[float],
    mat_rgba: wp.array2d[wp.vec4],
    # Data in:
    geom_xpos_in: wp.array2d[wp.vec3],
    geom_xquat_in: wp.array2d[wp.quat],
    cam_xpos_in: wp.array2d[wp.vec3],
    cam_xmat_in: wp.array2d[wp.mat33],
    light_xpos_in: wp.array2d[wp.vec3],
    light_xdir_in: wp.array2d[wp.vec3],
    flexvert_xpos_in: wp.array2d[wp.vec3],
    # In:
    nrender: int,
    total_rays: int,
    nsamples_in: int,
    use_shadows: bool,
    bvh_ngeom: int,
    bvh_nflexgeom: int,
    cam_res: wp.array[wp.vec2i],
    cam_id_map: wp.array[int],
    ray: wp.array[wp.vec3],
    ray_offset: wp.array[wp.vec3],
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
    mesh_bvh_rot: wp.array[wp.mat33],
    mesh_facetexcoord: wp.array[wp.vec3i],
    mesh_facenormal: wp.array[wp.vec3i],
    mesh_texcoord: wp.array[wp.vec2],
    mesh_texcoord_offsets: wp.array[int],
    hfield_bvh_id: wp.array[wp.uint64],
    flex_rgba: wp.array[wp.vec4],
    flex_geom_flexid: wp.array[int],
    flex_geom_edgeid: wp.array[int],
    skybox_tex_id: wp.array[int],
    skybox_face_width: wp.array[int],
    textures: wp.array[wp.Texture2D],
    splat_position: wp.array[wp.vec3],
    splat_rotation: wp.array[wp.quat],
    splat_scale: wp.array[wp.vec3],
    splat_rgba: wp.array[wp.vec4],
    splat_bvh_id: wp.uint64,
    splat_group_root: wp.array[int],
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

    do_rgb = render_rgb[camid] if wp.static(rc_static["any_rgb"]) else False
    do_depth = render_depth[camid]
    do_seg = render_seg[camid] if wp.static(rc_static["any_seg"]) else False
    if not do_rgb and not do_depth and not do_seg:
      return

    # Map active camera index to MuJoCo camera ID and tiled pixel coordinate
    mujoco_cam_id = cam_id_map[camid]
    img_w = cam_res[camid][0]
    img_h = cam_res[camid][1]
    px, py, pixelid = tile_pixel(rayid_local, img_w, img_h, wp.static(tile_rows))

    rgb_out_idx = rgb_adr[camid] + pixelid if do_rgb else 0
    depth_out_idx = depth_adr[camid] + pixelid if do_depth else 0
    seg_out_idx = seg_adr[camid] + pixelid if do_seg else 0

    # Loading the skybox before the trace speeds up the Warp BVH but slows OptiX, which keeps it
    # live across traversal.
    if wp.static(hoist_skybox):
      skybox_id = skybox_tex_id[worldid % skybox_tex_id.shape[0]]
      skybox_tex = textures[skybox_id]
      skybox_face_width_inv = 1.0 / float(skybox_face_width[worldid % skybox_face_width.shape[0]])

    cam_mat_world = cam_xmat_in[worldid, mujoco_cam_id]
    cam_pos_world = cam_xpos_in[worldid, mujoco_cam_id]
    world_bvh_root = group_root[worldid]
    color_accum = wp.vec3(0.0)

    if wp.static(not rc_static["use_precomputed_rays"]):
      cam_proj = cam_projection[mujoco_cam_id]
      cam_fovy_val = cam_fovy[worldid % cam_fovy.shape[0], mujoco_cam_id]
      cam_sensor_val = cam_sensorsize[mujoco_cam_id]
      cam_intr_val = cam_intrinsic[worldid % cam_intrinsic.shape[0], mujoco_cam_id]
      sample_w = img_w * wp.static(n_aa) if wp.static(nsamples > 1) else img_w
      sample_h = img_h * wp.static(n_aa) if wp.static(nsamples > 1) else img_h

    for sample in range(1 if wp.static(nsamples == 1) else nsamples_in):
      if wp.static(rc_static["use_precomputed_rays"]):
        if wp.static(nsamples > 1):
          ray_idx = sample * total_rays + rayid
        else:
          ray_idx = rayid
        ray_dir_local_cam = ray[ray_idx]
        ray_offset_local_cam = ray_offset[ray_idx]
      else:
        if wp.static(nsamples > 1):
          sample_px = px * wp.static(n_aa) + (sample % wp.static(n_aa))
          sample_py = py * wp.static(n_aa) + (sample // wp.static(n_aa))
        else:
          sample_px = px
          sample_py = py
        ray_dir_local_cam, ray_offset_local_cam = compute_ray(
          cam_proj,
          cam_fovy_val,
          cam_sensor_val,
          cam_intr_val,
          sample_w,
          sample_h,
          sample_px,
          sample_py,
          wp.static(rc_static["znear"]),
        )

      ray_origin_world = cam_pos_world
      if wp.static(rc_static["has_orthographic_camera"]):
        ray_origin_world = ray_origin_world + cam_mat_world @ ray_offset_local_cam
      ray_dir_world = cam_mat_world @ ray_dir_local_cam

      # Start primary rays on the near plane.
      inv_cos = 1.0 / wp.max(-ray_dir_local_cam[2], 1e-6)
      ray_origin_world = ray_origin_world + wp.static(rc_static["znear"]) * inv_cos * ray_dir_world

      if wp.static(rc_static["zfar"] > 0.0):
        max_cam_dist = wp.static(rc_static["zfar"] - rc_static["znear"]) * inv_cos
      else:
        max_cam_dist = float(MJ_MAXVAL)

      geom_id, dist, normal, u, v, f, mesh_id = cast_ray(
        geom_type,
        geom_dataid,
        geom_matid,
        geom_size,
        geom_rgba,
        flex_vertadr,
        flex_edge,
        flex_radius,
        mat_rgba,
        geom_xpos_in,
        geom_xquat_in,
        flexvert_xpos_in,
        bvh_id,
        world_bvh_root,
        worldid,
        bvh_ngeom,
        bvh_nflexgeom,
        enabled_geom_ids,
        mesh_bvh_id,
        mesh_bvh_rot,
        hfield_bvh_id,
        flex_rgba,
        flex_geom_flexid,
        flex_geom_edgeid,
        flex_bvh_id,
        flex_group_root,
        ray_origin_world,
        ray_dir_world,
        max_cam_dist,
        wp.static(rc_static["enable_backface_culling"]),
      )
      if wp.static(rc_static["zfar"] > 0.0):
        if geom_id != -1 and (dist * -ray_dir_local_cam[2]) > wp.static(rc_static["zfar"] - rc_static["znear"]):
          geom_id = -1

      splat_color = wp.vec3(0.0)
      splat_transmittance = float(1.0)
      splat_depth = float(-1.0)
      if wp.static(has_splats):
        splat_color, splat_transmittance, splat_depth = shade_splats(
          splat_position,
          splat_rotation,
          splat_scale,
          splat_rgba,
          splat_bvh_id,
          splat_group_root[worldid],
          ray_origin_world,
          ray_dir_world,
          dist,
        )

      # Depth and seg are single-sample outputs: only the first sample writes them.
      if sample == 0:
        if wp.static(rc_static["any_seg"]):
          if do_seg:
            if geom_id == -1:
              seg_out[worldid, seg_out_idx] = wp.vec2i(-1, -1)
            elif geom_id == -2:
              seg_out[worldid, seg_out_idx] = wp.vec2i(mesh_id, int(ObjType.FLEX))
            else:
              seg_out[worldid, seg_out_idx] = wp.vec2i(geom_id, int(ObjType.GEOM))

        if do_depth:
          if geom_id == -1:
            depth = 0.0
            if wp.static(has_splats):
              if splat_depth > 0.0:
                depth = splat_depth * -ray_dir_local_cam[2] + wp.static(rc_static["znear"])
            depth_out[worldid, depth_out_idx] = depth
          else:
            # Planar depth: project distance from the near plane onto the optical axis.
            # In camera-local coordinates, the optical axis is -Z. The Z-component of the
            # normalized ray direction is negative, so -ray_dir_local_cam[2] gives cos(θ)
            # between the ray and the optical axis.
            depth = dist
            if wp.static(has_splats):
              if splat_depth > 0.0:
                depth = splat_depth
            depth_out[worldid, depth_out_idx] = depth * -ray_dir_local_cam[2] + wp.static(rc_static["znear"])

      if wp.static(not rc_static["any_rgb"]):
        return
      if not do_rgb:
        return

      if geom_id == -1:
        if wp.static(rc_static["render_skybox"]):
          if wp.static(not hoist_skybox):
            skybox_tex = textures[skybox_tex_id[worldid % skybox_tex_id.shape[0]]]
            skybox_face_width_inv = 1.0 / float(skybox_face_width[worldid % skybox_face_width.shape[0]])
          skybox_color = sample_skybox(
            skybox_tex,
            skybox_face_width_inv,
            ray_dir_world,
          )
          if wp.static(has_splats):
            skybox_color = splat_color + skybox_color * splat_transmittance
          if wp.static(nsamples > 1):
            color_accum += skybox_color * 255.0
          else:
            rgb_out[worldid, rgb_out_idx] = pack_rgba_to_uint32(
              skybox_color[0] * 255.0, skybox_color[1] * 255.0, skybox_color[2] * 255.0, 255.0
            )
        elif wp.static(has_splats):
          pixel_color = splat_color + wp.static(rc_static["background_color_vec3"]) * splat_transmittance
          if wp.static(nsamples > 1):
            color_accum += pixel_color * 255.0
          else:
            rgb_out[worldid, rgb_out_idx] = pack_rgba_to_uint32(
              pixel_color[0] * 255.0, pixel_color[1] * 255.0, pixel_color[2] * 255.0, 255.0
            )
        elif wp.static(nsamples > 1):
          color_accum += wp.static(rc_static["background_color_vec3"]) * 255.0
        else:
          rgb_out[worldid, rgb_out_idx] = wp.static(rc_static["background_color"])
        continue

      if wp.static(rc_static["enable_vertex_normals"]) and geom_id >= 0 and mesh_id >= 0 and f >= 0:
        gtype = geom_type[geom_id]
        if gtype == int(GeomType.MESH.value) or gtype == int(GeomType.SDF.value):
          mat = math.quat_to_mat(geom_xquat_in[worldid, geom_id])
          face = wp.transpose(mat) @ normal
          tri = mesh_facenormal[mesh_faceadr[mesh_id] + f]
          adr = mesh_normaladr[mesh_id]
          vec = (
            vertex_normal(mesh_normal[adr + tri[0]], face) * u
            + vertex_normal(mesh_normal[adr + tri[1]], face) * v
            + vertex_normal(mesh_normal[adr + tri[2]], face) * (1.0 - u - v)
          )
          normal = wp.normalize(mat @ vec)

      if wp.static(not rc_static["enable_backface_culling"]):
        # Two-sided shading: light a back-facing hit as if it faced the viewer.
        if wp.dot(normal, ray_dir_world) > 0.0:
          normal = -normal

      # Shade the pixel
      hit_point = ray_origin_world + ray_dir_world * dist

      mat_id = -1
      if geom_id == -2:
        # We encode flex_id in mesh_id for flex ray hits during cast_ray
        color = flex_rgba[mesh_id]
      else:
        color, mat_id = geom_color(geom_matid, geom_rgba, mat_rgba, worldid, geom_id)

      base_color = wp.vec3(color[0], color[1], color[2])

      if wp.static(rc_static["use_textures"]):
        if mat_id >= 0:
          tex_id = mat_texid[worldid % mat_texid.shape[0], mat_id, 1]
          if tex_id >= 0:
            tex_color = sample_texture(
              geom_type,
              mesh_faceadr,
              geom_id,
              geom_size[worldid % geom_size.shape[0], geom_id],
              mat_texrepeat[worldid % mat_texrepeat.shape[0], mat_id],
              mat_texuniform[worldid % mat_texuniform.shape[0], mat_id],
              textures[tex_id],
              geom_xpos_in[worldid, geom_id],
              math.quat_to_mat(geom_xquat_in[worldid, geom_id]),
              mesh_facetexcoord,
              mesh_texcoord,
              mesh_texcoord_offsets,
              hit_point,
              normal,
              u,
              v,
              f,
              mesh_id,
            )
            base_color = wp.cw_mul(base_color, tex_color)

      mat_spec = DEFAULT_MAT_SPECULAR
      mat_shin_exp = DEFAULT_MAT_SHININESS_EXPONENT
      mat_emis = DEFAULT_MAT_EMISSION
      if wp.static(rc_static["enable_specular_or_emission"]):
        if mat_id >= 0:
          if wp.static(rc_static["enable_specular"]):
            mat_spec = mat_specular[worldid % mat_specular.shape[0], mat_id]
            mat_shin_exp = mat_shininess[worldid % mat_shininess.shape[0], mat_id] * MAX_SHININESS
          if wp.static(rc_static["enable_emission"]):
            mat_emis = mat_emission[worldid % mat_emission.shape[0], mat_id]

      result = wp.vec3(0.0)
      if wp.static(rc_static["enable_emission"]):
        result = base_color * mat_emis

      light_active_worldid = light_active[worldid % light_active.shape[0]]
      if wp.static(rc_static["use_ambient_lighting"]):
        if wp.static(rc_static["headlight_active"]):
          result = result + wp.cw_mul(base_color, wp.static(rc_static["headlight_ambient"]))
        elif wp.static(M_NLIGHT == 0):
          result = result + base_color * NO_LIGHT_AMBIENT_FALLBACK
        if wp.static(rc_static["enable_per_light_ambient"]):
          for light_index in range(wp.static(M_NLIGHT)):
            if light_active_worldid[light_index]:
              result = result + wp.cw_mul(base_color, light_ambient[worldid % light_ambient.shape[0], light_index])

      view_dir = -ray_dir_world

      light_cutoff_worldid = light_cutoff[worldid % light_cutoff.shape[0]]
      light_type_worldid = light_type[worldid % light_type.shape[0]]
      light_castshadow_worldid = light_castshadow[worldid % light_castshadow.shape[0]]
      light_xpos_in_worldid = light_xpos_in[worldid]
      light_xdir_in_worldid = light_xdir_in[worldid]
      light_attenuation_worldid = light_attenuation[worldid % light_attenuation.shape[0]]
      light_exponent_worldid = light_exponent[worldid % light_exponent.shape[0]]
      light_diffuse_worldid = light_diffuse[worldid % light_diffuse.shape[0]]
      light_specular_worldid = light_specular[worldid % light_specular.shape[0]]
      # Apply Lighting for each light
      for light_index in range(wp.static(M_NLIGHT)):
        diff_rgb, spec_rgb = compute_lighting(
          geom_type,
          geom_dataid,
          geom_matid,
          geom_size,
          geom_rgba,
          flex_vertadr,
          flex_edge,
          flex_radius,
          mat_rgba,
          geom_xpos_in,
          geom_xquat_in,
          flexvert_xpos_in,
          use_shadows,
          bvh_id,
          world_bvh_root,
          bvh_ngeom,
          bvh_nflexgeom,
          enabled_geom_ids,
          worldid,
          mesh_bvh_id,
          mesh_bvh_rot,
          hfield_bvh_id,
          flex_rgba,
          flex_geom_flexid,
          flex_geom_edgeid,
          flex_bvh_id,
          flex_group_root,
          light_active_worldid[light_index],
          light_type_worldid[light_index],
          light_castshadow_worldid[light_index],
          light_xpos_in_worldid[light_index],
          light_xdir_in_worldid[light_index],
          light_attenuation_worldid[light_index],
          light_cutoff_worldid[light_index] * wp.static(wp.pi / 180.0),
          light_exponent_worldid[light_index],
          light_diffuse_worldid[light_index],
          light_specular_worldid[light_index],
          normal,
          hit_point,
          view_dir,
          mat_spec,
          mat_shin_exp,
          wp.static(rc_static["enable_backface_culling"]),
          wp.static(rc_static["shadow_light_fraction"]),
          wp.static(rc_static["enable_specular"]),
          wp.static(rc_static["light_attenuation_is_default"]),
          wp.static(rc_static["has_spot_lights"]),
        )
        result = result + wp.cw_mul(base_color, diff_rgb) + spec_rgb

      # Apply Headlight
      if wp.static(rc_static["headlight_active"]):
        cam_pos = ray_origin_world
        cam_fwd = -cam_mat_world[:, 2]
        hl_diff, hl_spec = compute_lighting(
          geom_type,
          geom_dataid,
          geom_matid,
          geom_size,
          geom_rgba,
          flex_vertadr,
          flex_edge,
          flex_radius,
          mat_rgba,
          geom_xpos_in,
          geom_xquat_in,
          flexvert_xpos_in,
          use_shadows,
          bvh_id,
          world_bvh_root,
          bvh_ngeom,
          bvh_nflexgeom,
          enabled_geom_ids,
          worldid,
          mesh_bvh_id,
          mesh_bvh_rot,
          hfield_bvh_id,
          flex_rgba,
          flex_geom_flexid,
          flex_geom_edgeid,
          flex_bvh_id,
          flex_group_root,
          True,
          1,
          False,
          cam_pos,
          cam_fwd,
          wp.vec3(1.0, 0.0, 0.0),
          0.0,
          0.0,
          wp.static(rc_static["headlight_diffuse"]),
          wp.static(rc_static["headlight_specular"]),
          normal,
          hit_point,
          view_dir,
          mat_spec,
          mat_shin_exp,
          wp.static(rc_static["enable_backface_culling"]),
          wp.static(rc_static["shadow_light_fraction"]),
          wp.static(rc_static["enable_specular"]),
          True,
          False,
        )
        result = result + wp.cw_mul(base_color, hl_diff) + hl_spec

      hit_color = wp.min(result, wp.vec3(1.0, 1.0, 1.0))
      hit_color = wp.max(hit_color, wp.vec3(0.0, 0.0, 0.0))
      if wp.static(has_splats):
        hit_color = splat_color + hit_color * splat_transmittance

      if wp.static(nsamples > 1):
        color_accum += hit_color * 255.0
      else:
        rgb_out[worldid, rgb_out_idx] = pack_rgba_to_uint32(
          hit_color[0] * 255.0, hit_color[1] * 255.0, hit_color[2] * 255.0, 255.0
        )

    if wp.static(rc_static["any_rgb"] and nsamples > 1):
      c = color_accum * wp.static(1.0 / float(nsamples))
      rgb_out[worldid, rgb_out_idx] = pack_rgba_to_uint32(
        wp.clamp(c[0], 0.0, 255.0), wp.clamp(c[1], 0.0, 255.0), wp.clamp(c[2], 0.0, 255.0), 255.0
      )

  return _render_megakernel


@event_scope
def render(m: Model, d: Data, rc: RenderContext):
  """Render the current frame.

  Outputs are stored in buffers within the render context.

  Args:
    m: The model on device.
    d: The data on device.
    rc: The render context on device.
  """
  # Specialising the megakernel costs more than launching it, so keep it on the
  # context: the static configuration it closes over is fixed at creation.
  if rc._megakernel is None:
    rc._megakernel = _build_megakernel(m.nlight, rc, m.block_dim.render)
  _render_megakernel = rc._megakernel

  inputs = [
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
    m.light_attenuation,
    m.light_cutoff,
    m.light_exponent,
    m.light_ambient,
    m.light_diffuse,
    m.light_specular,
    m.flex_vertadr,
    m.flex_edge,
    m.flex_radius,
    m.mesh_faceadr,
    m.mesh_normaladr,
    m.mesh_normal,
    m.mat_texid,
    m.mat_texuniform,
    m.mat_texrepeat,
    m.mat_emission,
    m.mat_specular,
    m.mat_shininess,
    m.mat_rgba,
    d.geom_xpos,
    d.geom_xquat,
    d.cam_xpos,
    d.cam_xmat,
    d.light_xpos,
    d.light_xdir,
    d.flexvert_xpos,
    rc.nrender,
    rc.total_rays,
    rc.samples_per_pixel * rc.samples_per_pixel,
    rc.use_shadows,
    rc.bvh_ngeom,
    rc.bvh_nflexgeom,
    rc.cam_res,
    rc.cam_id_map,
    rc.ray,
    rc.ray_offset,
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
    rc.mesh_bvh_rot,
    rc.mesh_facetexcoord,
    rc.mesh_facenormal,
    rc.mesh_texcoord,
    rc.mesh_texcoord_offsets,
    rc.hfield_bvh_id,
    rc.flex_rgba,
    rc.flex_geom_flexid,
    rc.flex_geom_edgeid,
    rc.skybox_tex_id,
    rc.skybox_face_width,
    rc.textures,
    rc.splat_position,
    rc.splat_rotation,
    rc.splat_scale,
    rc.splat_rgba,
    rc.splat_bvh_id,
    rc.splat_group_root,
  ]
  outputs = [
    rc.rgb_data,
    rc.depth_data,
    rc.seg_data,
  ]
  dim = (d.nworld, rc.total_rays)
  if rc.backend == "warp":
    wp.launch(_render_megakernel, dim=dim, inputs=inputs, outputs=outputs, block_dim=m.block_dim.render)
  elif dim[0] and dim[1]:  # OptiX rejects empty launches
    render_optix.get_wo().launch(rc.optix.pipeline, dim, inputs=inputs, outputs=outputs)
