# Copyright 2025 The Physics-Next Project Developers
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
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import _geom
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import contact_params
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import write_contact
from mujoco.mjx.third_party.mujoco_warp._src.math import make_frame
from mujoco.mjx.third_party.mujoco_warp._src.types import Data
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import Model
from mujoco.mjx.third_party.mujoco_warp._src.types import vec5
from mujoco.mjx.third_party.mujoco_warp._src.util_misc import halton
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import event_scope


@wp.struct
class OptimizationParams:
  rel_mat: wp.mat33
  rel_pos: wp.vec3
  attr1: wp.vec3
  attr2: wp.vec3


@wp.struct
class AABB:
  min: wp.vec3
  max: wp.vec3


@wp.func
def transform_aabb(aabb_pos: wp.vec3, aabb_size: wp.vec3, pos: wp.vec3, ori: wp.mat33) -> AABB:
  aabb = AABB()
  aabb.max = wp.vec3(-1000000000.0, -1000000000.0, -1000000000.0)
  aabb.min = wp.vec3(1000000000.0, 1000000000.0, 1000000000.0)

  for i in range(8):
    vec = wp.vec3(
      aabb_size.x * (1.0 if (i & 1) else -1.0),
      aabb_size.y * (1.0 if (i & 2) else -1.0),
      aabb_size.z * (1.0 if (i & 4) else -1.0),
    )

    frame_vec = ori * (vec + aabb_pos) + pos

    aabb.min = wp.min(aabb.min, frame_vec)
    aabb.max = wp.max(aabb.max, frame_vec)

  return aabb


@wp.func
def radial_field(a: wp.vec3, x: wp.vec3, size: wp.vec3) -> wp.vec3:
  field = wp.cw_div(-size, a)
  field = wp.normalize(field)
  field[0] *= wp.sign(x[0])
  field[1] *= wp.sign(x[1])
  field[2] *= wp.sign(x[2])
  return field


@wp.func
def sphere(p: wp.vec3, size: wp.vec3) -> float:
  return wp.length(p) - size[0]


@wp.func
def box(p: wp.vec3, size: wp.vec3) -> float:
  a = wp.abs(p) - size
  if a[0] >= 0 or a[1] >= 0 or a[2] >= 0:
    z = wp.vec3(0.0, 0.0, 0.0)
    b = wp.max(a, z)
    return wp.norm_l2(b) + wp.min(wp.max(a), 0.0)
  b = radial_field(a, p, size)
  t = -wp.cw_div(a, wp.abs(b))
  return -wp.min(t) * wp.norm_l2(b)


@wp.func
def ellipsoid(p: wp.vec3, size: wp.vec3) -> float:
  scaled_p = wp.vec3(p[0] / size[0], p[1] / size[1], p[2] / size[2])
  k0 = wp.length(scaled_p)
  k1 = wp.length(wp.vec3(p[0] / (size[0] ** 2.0), p[1] / (size[1] ** 2.0), p[2] / (size[2] ** 2.0)))
  if k1 != 0.0:
    denom = k1
  else:
    denom = 1e-12
  return k0 * (k0 - 1.0) / denom


@wp.func
def grad_sphere(p: wp.vec3) -> wp.vec3:
  c = wp.length(p)
  if c > 1e-9:
    return p / c
  else:
    wp.vec3(0.0)


@wp.func
def grad_box(p: wp.vec3, size: wp.vec3) -> wp.vec3:
  a = wp.abs(p) - size
  if wp.max(a) < 0:
    return radial_field(a, p, size)
  z = wp.vec3(0.0, 0.0, 0.0)
  b = wp.max(a, z)
  c = wp.norm_l2(b)
  g = wp.cw_mul(wp.div(b, c), wp.cw_div(p, wp.abs(p)))
  if a[0] <= 0:
    g[0] = 0.0
  if a[1] <= 0:
    g[1] = 0.0
  if a[2] <= 0:
    g[2] = 0.0
  return g


@wp.func
def grad_ellipsoid(p: wp.vec3, size: wp.vec3) -> wp.vec3:
  a = wp.vec3(p[0] / size[0], p[1] / size[1], p[2] / size[2])

  b = wp.vec3(a[0] / size[0], a[1] / size[1], a[2] / size[2])
  k0 = wp.length(a)
  k1 = wp.length(b)
  invK0 = 1.0 / k0
  invK1 = 1.0 / k1

  gk0 = b * invK0
  gk1 = wp.vec3(
    b[0] * invK1 / (size[0] * size[0]),
    b[1] * invK1 / (size[1] * size[1]),
    b[2] * invK1 / (size[2] * size[2]),
  )
  df_dk0 = (2.0 * k0 - 1.0) * invK1
  df_dk1 = k0 * (k0 - 1.0) * invK1 * invK1

  raw_grad = gk0 * df_dk0 - gk1 * df_dk1
  return raw_grad / wp.length(raw_grad)


@wp.func
def user_sdf(p: wp.vec3, attr: wp.vec3, sdf_type: int) -> float:
  wp.printf("ERROR: user_sdf function must be implemented by user code\n")
  return 0.0


@wp.func
def user_sdf_grad(p: wp.vec3, attr: wp.vec3, sdf_type: int) -> wp.vec3:
  wp.printf("ERROR: user_sdf_grad function must be implemented by user code\n")
  return wp.vec3(0.0)


@wp.func
def sdf(type: int, p: wp.vec3, attr: wp.vec3, sdf_type: int) -> float:
  if type == int(GeomType.PLANE.value):
    return p[2]
  elif type == int(GeomType.SPHERE.value):
    return sphere(p, attr)
  elif type == int(GeomType.BOX.value):
    return box(p, attr)
  elif type == int(GeomType.ELLIPSOID.value):
    return ellipsoid(p, attr)
  elif type == int(GeomType.SDF.value):
    return user_sdf(p, attr, sdf_type)
  wp.printf("ERROR: SDF type not implemented\n")
  return 0.0


@wp.func
def sdf_grad(type: int, p: wp.vec3, attr: wp.vec3, sdf_type: int) -> wp.vec3:
  if type == int(GeomType.PLANE.value):
    grad = wp.vec3(0.0, 0.0, 1.0)
    return grad
  elif type == int(GeomType.SPHERE.value):
    return grad_sphere(p)
  elif type == int(GeomType.BOX.value):
    return grad_box(p, attr)
  elif type == int(GeomType.ELLIPSOID.value):
    return grad_ellipsoid(p, attr)
  elif type == int(GeomType.SDF.value):
    return user_sdf_grad(p, attr, sdf_type)
  wp.printf("ERROR: SDF grad type not implemented\n")
  return wp.vec3(0.0)


@wp.func
def clearance(
  type1: int, p1: wp.vec3, p2: wp.vec3, s1: wp.vec3, s2: wp.vec3, sdf_type1: int, sdf_type2: int, sfd_intersection: bool
) -> float:
  sdf1 = sdf(type1, p1, s1, sdf_type1)
  sdf2 = sdf(int(GeomType.SDF.value), p2, s2, sdf_type2)
  if sfd_intersection:
    return wp.max(sdf1, sdf2)
  else:
    return sdf1 + sdf2 + wp.abs(wp.max(sdf1, sdf2))


@wp.func
def compute_grad(
  type1: int, p1: wp.vec3, p2: wp.vec3, params: OptimizationParams, sdf_type1: int, sdf_type2: int, sfd_intersection: bool
) -> wp.vec3:
  A = sdf(type1, p1, params.attr1, sdf_type1)
  B = sdf(int(GeomType.SDF.value), p2, params.attr2, sdf_type2)
  grad1 = sdf_grad(type1, p1, params.attr1, sdf_type1)
  grad2 = sdf_grad(int(GeomType.SDF.value), p2, params.attr2, sdf_type2)
  grad1_transformed = params.rel_mat * grad1
  if sfd_intersection:
    if A > B:
      return grad1_transformed
    else:
      return grad2
  else:
    gradient = grad2 + grad1_transformed
    max_val = wp.max(A, B)
    if A > B:
      max_grad = grad1_transformed
    else:
      max_grad = grad2
    sign = wp.sign(max_val)
    gradient += max_grad * sign
    return gradient


@wp.func
def gradient_step(
  type1: int, x: wp.vec3, params: OptimizationParams, sdf_type1: int, sdf_type2: int, niter: int, sfd_intersection: bool
) -> Tuple[float, wp.vec3]:
  amin = 1e-4
  rho = 0.5
  c = 0.1
  dist = float(1e10)

  for _ in range(niter):
    alpha = float(2.0)
    x2 = wp.vec3(x[0], x[1], x[2])
    x1 = params.rel_mat * x2 + params.rel_pos
    grad = compute_grad(type1, x1, x2, params, sdf_type1, sdf_type2, sfd_intersection)
    dist0 = clearance(type1, x1, x, params.attr1, params.attr2, sdf_type1, sdf_type2, sfd_intersection)
    grad_dot = wp.dot(grad, grad)

    if grad_dot < 1e-12:
      return dist0, x

    wolfe = -c * alpha * grad_dot
    while True:
      alpha *= rho
      wolfe *= rho

      x = x2 - grad * alpha
      x1 = params.rel_mat * x + params.rel_pos
      dist = clearance(type1, x1, x, params.attr1, params.attr2, sdf_type1, sdf_type2, sfd_intersection)

      if alpha <= amin or (dist - dist0) <= wolfe:
        break
    if dist > dist0:
      return dist, x
  return dist, x


@wp.func
def gradient_descent(
  # In:
  type1: int,
  x0_initial: wp.vec3,
  attr1: wp.vec3,
  attr2: wp.vec3,
  pos1: wp.vec3,
  rot1: wp.mat33,
  pos2: wp.vec3,
  rot2: wp.mat33,
  sdf_type1: int,
  sdf_type2: int,
  sdf_iterations: int,
) -> Tuple[float, wp.vec3, wp.vec3]:
  params = OptimizationParams()
  params.rel_mat = wp.transpose(rot1) * rot2
  params.rel_pos = wp.transpose(rot1) * (pos2 - pos1)
  params.attr1 = attr1
  params.attr2 = attr2

  # Collision phase (10 iterations, sfd_intersection=False)
  dist, x = gradient_step(type1, x0_initial, params, sdf_type1, sdf_type2, sdf_iterations, False)

  # Intersection phase (1 iteration, sfd_intersection=True)
  dist, x = gradient_step(type1, x, params, sdf_type1, sdf_type2, 1, True)

  # Midsurface calculation
  x_1 = params.rel_mat * x + params.rel_pos

  grad1 = sdf_grad(type1, x_1, params.attr1, sdf_type1)
  grad1 = wp.transpose(params.rel_mat) * grad1
  grad1 = wp.normalize(grad1)

  grad2 = sdf_grad(int(GeomType.SDF.value), x, params.attr2, sdf_type2)
  grad2 = wp.normalize(grad2)

  n = grad1 - grad2
  n = wp.normalize(n)
  pos = rot2 * x + pos2
  n = rot2 * n
  pos3 = pos - n * dist / 2.0
  return dist, pos3, n


@wp.kernel
def _sdf_narrowphase(
  # Model:
  geom_type: wp.array(dtype=int),
  geom_condim: wp.array(dtype=int),
  geom_dataid: wp.array(dtype=int),
  geom_priority: wp.array(dtype=int),
  geom_solmix: wp.array2d(dtype=float),
  geom_solref: wp.array2d(dtype=wp.vec2),
  geom_solimp: wp.array2d(dtype=vec5),
  geom_size: wp.array2d(dtype=wp.vec3),
  geom_aabb: wp.array2d(dtype=wp.vec3),
  geom_pos: wp.array2d(dtype=wp.vec3),
  geom_quat: wp.array2d(dtype=wp.quat),
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
  # In:
  plugin: wp.array(dtype=int),
  plugin_attr: wp.array(dtype=wp.vec3f),
  geom_plugin_index: wp.array(dtype=int),
  # Data in:
  nconmax_in: int,
  geom_xpos_in: wp.array2d(dtype=wp.vec3),
  geom_xmat_in: wp.array2d(dtype=wp.mat33),
  collision_pair_in: wp.array(dtype=wp.vec2i),
  collision_hftri_index_in: wp.array(dtype=int),
  collision_pairid_in: wp.array(dtype=int),
  collision_worldid_in: wp.array(dtype=int),
  ncollision_in: wp.array(dtype=int),
  # In:
  sdf_initpoints: int,
  sdf_iterations: int,
  # Data out:
  ncon_out: wp.array(dtype=int),
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

  g2 = geoms[1]
  type2 = geom_type[g2]
  if type2 != int(GeomType.SDF.value):
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
  g1 = geoms[0]

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

  type1 = geom_type[g1]
  g1_plugin = geom_plugin_index[g1]
  g2_plugin = geom_plugin_index[g2]

  g1_to_g2_rot = wp.transpose(geom1.rot) * geom2.rot
  g1_to_g2_pos = wp.transpose(geom1.rot) * (geom2.pos - geom1.pos)

  aabb_pos = geom_aabb[g1, 0]
  aabb_size = geom_aabb[g1, 1]
  identity = wp.identity(3, dtype=float)
  aabb1 = transform_aabb(aabb_pos, aabb_size, wp.vec3(0.0), identity)
  aabb_pos = geom_aabb[g2, 0]
  aabb_size = geom_aabb[g2, 1]
  aabb2 = transform_aabb(aabb_pos, aabb_size, g1_to_g2_pos, g1_to_g2_rot)

  aabb_intersection = AABB()
  aabb_intersection.min = wp.max(aabb1.min, aabb2.min)
  aabb_intersection.max = wp.min(aabb1.max, aabb2.max)

  pos2 = geom2.pos
  rot2 = geom2.rot
  pos1 = geom1.pos
  rot1 = geom1.rot

  if type1 == int(GeomType.SDF.value):
    attr1 = plugin_attr[g1_plugin]
    g1_plugin_id = plugin[g1_plugin]
  else:
    attr1 = geom1.size
    g1_plugin_id = -1

  if g2_plugin != -1:
    attr2 = plugin_attr[g2_plugin]
    g2_plugin_id = plugin[g2_plugin]
  else:
    attr2 = geom2.size
    g2_plugin_id = -1

  for i in range(sdf_initpoints):
    x_g2 = wp.vec3(
      aabb_intersection.min[0] + (aabb_intersection.max[0] - aabb_intersection.min[0]) * halton(i, 2),
      aabb_intersection.min[1] + (aabb_intersection.max[1] - aabb_intersection.min[1]) * halton(i, 3),
      aabb_intersection.min[2] + (aabb_intersection.max[2] - aabb_intersection.min[2]) * halton(i, 5),
    )

    x = geom1.rot * x_g2 + geom1.pos
    x0_initial = wp.transpose(rot2) * (x - pos2)

    dist, pos, n = gradient_descent(
      type1, x0_initial, attr1, attr2, pos1, rot1, pos2, rot2, g1_plugin_id, g2_plugin_id, sdf_iterations
    )

    write_contact(
      nconmax_in,
      dist,
      pos,
      make_frame(n),
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


@event_scope
def sdf_narrowphase(m: Model, d: Data):
  wp.launch(
    _sdf_narrowphase,
    dim=d.nconmax,
    inputs=[
      m.geom_type,
      m.geom_condim,
      m.geom_dataid,
      m.geom_priority,
      m.geom_solmix,
      m.geom_solref,
      m.geom_solimp,
      m.geom_size,
      m.geom_aabb,
      m.geom_pos,
      m.geom_quat,
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
      m.plugin,
      m.plugin_attr,
      m.geom_plugin_index,
      d.nconmax,
      d.geom_xpos,
      d.geom_xmat,
      d.collision_pair,
      d.collision_hftri_index,
      d.collision_pairid,
      d.collision_worldid,
      d.ncollision,
      m.opt.sdf_initpoints,
      m.opt.sdf_iterations,
    ],
    outputs=[
      d.ncon,
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
