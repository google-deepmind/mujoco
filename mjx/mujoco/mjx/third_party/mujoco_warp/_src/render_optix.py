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
"""OptiX acceleration structures and pipeline for the batch renderer.

Meshes and heightfields share triangle GASes across worlds. Analytic geoms instance a padded
unit cube and reuse the ray intersections in ray.py. Worlds occupy separate grid cells;
bounded geometry must remain within each cell's extent. Planes are intersected separately.

Flexes and splats are unsupported.
"""

import functools
from typing import Tuple

import mujoco
import numpy as np
import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import math
from mujoco.mjx.third_party.mujoco_warp._src.ray import geom_color
from mujoco.mjx.third_party.mujoco_warp._src.ray import make_ray_primitive
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import OptixScene
from mujoco.mjx.third_party.mujoco_warp._src.types import RenderContext

wp.set_module_options({"enable_backward": False})


@functools.cache
def get_wo():
  """Import warp_optix on first use, since importing it initializes CUDA and OptiX."""
  try:
    import warp_optix as wo  # noqa: PLC0415

    return wo
  except ImportError:
    return None


_TRIANGLE_SBT_OFFSET = 0
_PRIMITIVE_SBT_OFFSET = 1
# Capping raygen registers raises occupancy, which outweighs the extra spills.
_MAX_REGISTERS = 72


@wp.func
def world_offset(worldid: int, grid: int, spacing: float) -> wp.vec3:
  """Returns the translation of a world's cell in the cubic grid of worlds."""
  center = float(grid - 1) * 0.5
  cell = wp.vec3(float(worldid % grid), float((worldid // grid) % grid), float(worldid // (grid * grid)))
  return (cell - wp.vec3(center)) * spacing


def build_scene_bvh(mjm: mujoco.MjModel, mjd: mujoco.MjData, rc: RenderContext, nworld: int):
  """Build OptiX acceleration structures for all geometries in all worlds."""
  wo = get_wo()
  extent = max(4.0 * mjm.stat.extent, 1.0)
  grid = max(1, int(np.ceil(nworld ** (1.0 / 3.0))))

  accels = []
  traversables = []
  for bvh_ids, registry in ((rc.mesh_bvh_id, rc.mesh_registry), (rc.hfield_bvh_id, rc.hfield_registry)):
    ids = []
    for bvh_id in bvh_ids.numpy():
      mesh = registry.get(int(bvh_id))
      if mesh is None:
        ids.append(0)
      else:
        accels.append(wo.Accel([wo.triangles(mesh.points, mesh.indices)], compact=True))
        ids.append(accels[-1].id)
    traversables.append(wp.array(ids, dtype=wp.uint64))
  mesh_traversable, hfield_traversable = traversables

  # Analytic geoms share a padded unit cube, scaled per instance to the current geom size.
  enabled_types = mjm.geom_type[rc.enabled_geom_ids.numpy()]
  primitive_traversable = 0
  if any(t not in (GeomType.PLANE, GeomType.HFIELD, GeomType.MESH, GeomType.SDF) for t in enabled_types):
    bounds = wp.array([[-1.001] * 3 + [1.001] * 3], dtype=float)
    accels.append(wo.Accel([wo.aabbs(bounds=bounds)], compact=True))
    primitive_traversable = accels[-1].id

  spacing = 3.0 * extent

  @wp.kernel(module="unique", enable_backward=False)
  def write_instances(
    # Model:
    geom_type: wp.array[int],
    geom_dataid: wp.array2d[int],
    geom_matid: wp.array2d[int],
    geom_size: wp.array2d[wp.vec3],
    geom_rgba: wp.array2d[wp.vec4],
    mat_rgba: wp.array2d[wp.vec4],
    # Data in:
    geom_xpos_in: wp.array2d[wp.vec3],
    geom_xquat_in: wp.array2d[wp.quat],
    # In:
    enabled_geom_ids: wp.array[int],
    mesh_bvh_rot: wp.array[wp.mat33],
    mesh_traversable: wp.array[wp.uint64],
    hfield_traversable: wp.array[wp.uint64],
    primitive_traversable: wp.uint64,
    # Out:
    instances_out: wp.array2d[wo.Instance],
  ):
    worldid, i = wp.tid()
    gi = enabled_geom_ids[i]
    gtype = geom_type[gi]
    linear = math.quat_to_mat(geom_xquat_in[worldid, gi])
    traversable = primitive_traversable
    sbt_offset = _PRIMITIVE_SBT_OFFSET
    if gtype == GeomType.PLANE:
      traversable = wp.uint64(0)
    elif gtype == GeomType.HFIELD:
      traversable = hfield_traversable[geom_dataid[worldid % geom_dataid.shape[0], gi]]
      sbt_offset = _TRIANGLE_SBT_OFFSET
    elif gtype == GeomType.MESH or gtype == GeomType.SDF:
      dataid = geom_dataid[worldid % geom_dataid.shape[0], gi]
      traversable = mesh_traversable[dataid]
      sbt_offset = _TRIANGLE_SBT_OFFSET
      # The triangles share the mesh BVH's points, which may be rotated to the authored frame.
      linear = linear @ mesh_bvh_rot[dataid]
    else:
      size = geom_size[worldid % geom_size.shape[0], gi]
      if gtype == GeomType.SPHERE:
        half = wp.vec3(size[0], size[0], size[0])
      elif gtype == GeomType.CAPSULE:
        half = wp.vec3(size[0], size[0], size[0] + size[1])
      elif gtype == GeomType.CYLINDER:
        half = wp.vec3(size[0], size[0], size[1])
      else:
        half = size
      linear = linear @ wp.diag(half)

    # Fully transparent geoms are invisible to rays, as in the BVH renderer. Planes are
    # intersected analytically by cast_ray and have no acceleration structure.
    rgba, _ = geom_color(geom_matid, geom_rgba, mat_rgba, worldid, gi)
    mask = wp.where(rgba[3] == 0.0 or gtype == GeomType.PLANE, wp.uint32(0), wp.uint32(0xFF))

    instances_out[worldid, i] = wo.make_instance(
      linear,
      geom_xpos_in[worldid, gi] + world_offset(worldid, wp.static(grid), wp.static(spacing)),
      traversable,
      wp.uint32(worldid * geom_type.shape[0] + gi),
      wp.uint32(sbt_offset),
      mask,
      wp.uint32(wp.static(int(wo.InstanceFlags.DISABLE_ANYHIT))),
    )

  scene = OptixScene(
    mesh_traversable=mesh_traversable,
    hfield_traversable=hfield_traversable,
    primitive_traversable=primitive_traversable,
    instances=wp.empty((nworld, rc.bvh_ngeom), dtype=wo.Instance),
    ias=None,
    extent=extent,
    planes=tuple(int(gi) for gi in rc.enabled_geom_ids.numpy() if mjm.geom_type[gi] == GeomType.PLANE),
    grid=grid,
    write_instances=write_instances,
  )
  wp.launch(
    write_instances,
    dim=scene.instances.shape,
    inputs=[
      wp.array(mjm.geom_type, dtype=int),
      wp.array(mjm.geom_dataid[None], dtype=int),
      wp.array(mjm.geom_matid[None], dtype=int),
      wp.array(mjm.geom_size[None], dtype=wp.vec3),
      wp.array(mjm.geom_rgba[None], dtype=wp.vec4),
      wp.array(mjm.mat_rgba[None], dtype=wp.vec4),
      wp.array(np.tile(mjd.geom_xpos[None], (nworld, 1, 1)), dtype=wp.vec3),
      wp.array(np.tile(mjd.geom_xquat[None], (nworld, 1, 1)), dtype=wp.quat),
      rc.enabled_geom_ids,
      rc.mesh_bvh_rot,
      mesh_traversable,
      hfield_traversable,
      wp.uint64(primitive_traversable),
    ],
    outputs=[scene.instances],
  )
  rc.bvh_id = 0
  if rc.bvh_ngeom:
    scene.ias = wo.Accel(
      [wo.instances(scene.instances, accels)], flags=wo.BuildFlags.ALLOW_UPDATE | wo.BuildFlags.PREFER_FAST_TRACE
    )
    rc.bvh_id = scene.ias.id
  rc.optix = scene


def refit_scene_bvh(m: Model, d: Data, rc: RenderContext):
  """Move every instance to its geom's current pose and refit the scene in place."""
  scene = rc.optix
  wp.launch(
    scene.write_instances,
    dim=scene.instances.shape,
    inputs=[
      m.geom_type,
      m.geom_dataid,
      m.geom_matid,
      m.geom_size,
      m.geom_rgba,
      m.mat_rgba,
      d.geom_xpos,
      d.geom_xquat,
      rc.enabled_geom_ids,
      rc.mesh_bvh_rot,
      scene.mesh_traversable,
      scene.hfield_traversable,
      wp.uint64(scene.primitive_traversable),
    ],
    outputs=[scene.instances],
  )
  if scene.ias is not None:
    scene.ias.refit()


def create_pipeline(scene: OptixScene, megakernel: wp.Kernel, geom_ray_types: Tuple[int, ...], cull_backfaces: bool):
  """Compile the render megakernel into an OptiX pipeline over `scene`."""
  wo = get_wo()
  ray_primitive = make_ray_primitive(geom_ray_types)
  grid, spacing = scene.grid, 3.0 * scene.extent

  # Binds the megakernel's arguments of the same names.
  @wp.kernel(module="unique", enable_backward=False)
  def intersect_primitive(
    # Model:
    geom_type: wp.array[int],
    geom_size: wp.array2d[wp.vec3],
    # Data in:
    geom_xpos_in: wp.array2d[wp.vec3],
    geom_xquat_in: wp.array2d[wp.quat],
  ):
    ngeom = geom_type.shape[0]
    inst_id = int(wp.optix_get_instance_id())
    worldid = inst_id // ngeom
    gi = inst_id - worldid * ngeom
    pnt = wp.optix_get_world_ray_origin() - world_offset(worldid, wp.static(grid), wp.static(spacing))
    vec = wp.optix_get_world_ray_direction()
    d, n = ray_primitive(
      geom_xpos_in[worldid, gi],
      math.quat_to_mat(geom_xquat_in[worldid, gi]),
      geom_size[worldid % geom_size.shape[0], gi],
      pnt,
      vec,
      geom_type[gi],
    )
    if d < wp.optix_get_ray_tmin():
      return
    if wp.static(cull_backfaces):
      if wp.dot(vec, n) > 0.0:
        return
    wp.optix_report_intersection(
      d, wp.uint32(0), wp.optix_float_as_uint32(n[0]), wp.optix_float_as_uint32(n[1]), wp.optix_float_as_uint32(n[2])
    )

  return wo.Pipeline(
    megakernel,
    sbt=wo.SbtLayout(hit_groups={"triangles": wo.HitGroup(), "primitives": wo.HitGroup(intersection=intersect_primitive)}),
    attribute_words=3,
    max_registers=_MAX_REGISTERS,
    module_options={name: megakernel.module.options[name] for name in ("fast_math", "max_unroll")},
    graph_flags=wo.TraversableGraphFlags.ALLOW_SINGLE_LEVEL_INSTANCING,
  )
