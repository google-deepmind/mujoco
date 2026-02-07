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

from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import Geom
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType
from mujoco.mjx.third_party.mujoco_warp._src.types import mat43
from mujoco.mjx.third_party.mujoco_warp._src.types import mat63

# TODO(team): improve compile time to enable backward pass
wp.set_module_options({"enable_backward": False})

FLOAT_MIN = -1e30
FLOAT_MAX = 1e30
MINVAL = 1e-15
MIN_DIST = 1e-10

# TODO(kbayes): write out formulas to derive these constants
FACE_TOL = 0.99999872
EDGE_TOL = 0.00159999931


@wp.struct
class GJKResult:
  dist: float
  x1: wp.vec3
  x2: wp.vec3
  dim: int
  simplex: mat43
  simplex1: mat43
  simplex2: mat43
  simplex_index1: wp.vec4i
  simplex_index2: wp.vec4i


@wp.struct
class Polytope:
  status: int

  # vertices in polytope
  vert: wp.array(dtype=wp.vec3)
  vert1: wp.array(dtype=wp.vec3)
  vert2: wp.array(dtype=wp.vec3)
  vert_index1: wp.array(dtype=int)
  vert_index2: wp.array(dtype=int)
  nvert: int

  # faces in polytope
  face: wp.array(dtype=wp.vec3i)
  face_pr: wp.array(dtype=wp.vec3)
  face_norm2: wp.array(dtype=float)
  face_index: wp.array(dtype=int)
  nface: int

  # TODO(kbayes): look into if a linear map actually improves performance
  face_map: wp.array(dtype=int)
  nmap: int

  # edges that make up the horizon when adding new vertices to polytope
  horizon: wp.array(dtype=int)
  nhorizon: int


@wp.struct
class SupportPoint:
  point: wp.vec3
  cached_index: int
  vertex_index: int


@wp.func
def _discrete_geoms(g1: int, g2: int) -> bool:
  return (g1 == GeomType.MESH or g1 == GeomType.BOX or g1 == GeomType.HFIELD) and (
    g2 == GeomType.MESH or g2 == GeomType.BOX or g2 == GeomType.HFIELD
  )


@wp.func
def _support(geom: Geom, geomtype: int, dir: wp.vec3) -> SupportPoint:
  sp = SupportPoint()
  sp.cached_index = -1
  sp.vertex_index = -1
  if geomtype == GeomType.SPHERE:
    sp.point = geom.pos + (0.5 * geom.margin) * geom.size[0] * dir
    return sp

  local_dir = wp.transpose(geom.rot) @ dir
  if geomtype == GeomType.BOX:
    tmp = wp.sign(local_dir)
    res = wp.cw_mul(tmp, geom.size)
    sp.point = geom.rot @ res + geom.pos
    sp.vertex_index = wp.where(tmp[0] > 0, 1, 0)
    sp.vertex_index += wp.where(tmp[1] > 0, 2, 0)
    sp.vertex_index += wp.where(tmp[2] > 0, 4, 0)
  elif geomtype == GeomType.CAPSULE:
    res = local_dir * geom.size[0]
    # add cylinder contribution
    res[2] += wp.sign(local_dir[2]) * geom.size[1]
    sp.point = geom.rot @ res + geom.pos
  elif geomtype == GeomType.ELLIPSOID:
    res = wp.cw_mul(local_dir, geom.size)
    res = wp.normalize(res)
    # transform to ellipsoid
    res = wp.cw_mul(res, geom.size)
    sp.point = geom.rot @ res + geom.pos
  elif geomtype == GeomType.CYLINDER:
    res = wp.vec3(0.0, 0.0, 0.0)
    # set result in XY plane: support on circle
    d = wp.sqrt(local_dir[0] * local_dir[0] + local_dir[1] * local_dir[1])
    if d > MINVAL:
      scl = geom.size[0] / d
      res[0] = local_dir[0] * scl
      res[1] = local_dir[1] * scl
    # set result in Z direction
    res[2] = wp.sign(local_dir[2]) * geom.size[1]
    sp.point = geom.rot @ res + geom.pos
  elif geomtype == GeomType.MESH:
    max_dist = float(FLOAT_MIN)
    if geom.graphadr == -1 or geom.vertnum < 10:
      if geom.index > -1:
        sp.cached_index = geom.index
        max_dist = wp.dot(geom.vert[geom.index], local_dir)
        sp.point = geom.vert[geom.index]
      # exhaustive search over all vertices
      for i in range(geom.vertnum):
        vert = geom.vert[geom.vertadr + i]
        dist = wp.dot(vert, local_dir)
        if dist > max_dist:
          max_dist = dist
          sp.point = vert
          sp.cached_index = geom.vertadr + i
      sp.vertex_index = sp.cached_index - geom.vertadr
    else:
      numvert = geom.graph[geom.graphadr]
      vert_edgeadr = geom.graphadr + 2
      vert_globalid = geom.graphadr + 2 + numvert
      edge_localid = geom.graphadr + 2 + 2 * numvert
      prev = int(-1)
      imax = wp.where(geom.index > -1, geom.index, 0)

      # hillclimb until no change
      while imax != prev:
        prev = imax
        i = geom.graph[vert_edgeadr + imax]
        subidx = geom.graph[edge_localid + i]
        while subidx >= 0:
          idx = geom.graph[vert_globalid + subidx]
          dist = wp.dot(local_dir, geom.vert[geom.vertadr + idx])
          imax = wp.where(dist > max_dist, subidx, imax)
          max_dist = wp.where(dist > max_dist, dist, max_dist)
          i += 1
          subidx = geom.graph[edge_localid + i]

      sp.cached_index = imax
      sp.vertex_index = geom.graph[vert_globalid + imax]
      sp.point = geom.vert[geom.vertadr + sp.vertex_index]

    sp.point = geom.rot @ sp.point + geom.pos
  elif geomtype == GeomType.HFIELD:
    max_dist = float(FLOAT_MIN)
    # TODO(kbayes): Support edge prisms
    sp.vertex_index = wp.where(local_dir[2] < 0, -2, -3)
    for i in range(6):
      vert = geom.hfprism[i]
      dist = wp.dot(vert, local_dir)
      if dist > max_dist:
        max_dist = dist
        sp.point = vert
    sp.point = geom.rot @ sp.point + geom.pos

  if geom.margin > 0.0:
    sp.point += dir * (0.5 * geom.margin)
  return sp


@wp.func
def _attach_face(pt: Polytope, idx: int, v1: int, v2: int, v3: int) -> float:
  # out of memory, returning 0 will force EPA to return early without contact
  if pt.nface == pt.face.shape[0]:
    return 0.0

  # compute witness point v
  r, ret = _project_origin_plane(pt.vert[v3], pt.vert[v2], pt.vert[v1])
  if ret:
    return 0.0

  face = wp.vec3i(v1, v2, v3)
  pt.face[idx] = face
  pt.face_pr[idx] = r

  pt.face_norm2[idx] = wp.dot(r, r)
  pt.face_index[idx] = -1
  return pt.face_norm2[idx]


@wp.func
def _epa_support(
  pt: Polytope, idx: int, geom1: Geom, geom2: Geom, geom1_type: int, geom2_type: int, dir: wp.vec3
) -> Tuple[int, int]:
  sp = _support(geom1, geom1_type, dir)
  pt.vert1[idx] = sp.point
  pt.vert_index1[idx] = sp.vertex_index
  index1 = sp.cached_index

  sp = _support(geom2, geom2_type, -dir)
  pt.vert2[idx] = sp.point
  pt.vert_index2[idx] = sp.vertex_index
  index2 = sp.cached_index

  pt.vert[idx] = pt.vert1[idx] - pt.vert2[idx]

  return index1, index2


@wp.func
def _linear_combine(n: int, coefs: wp.vec4, mat: mat43) -> wp.vec3:
  v = wp.vec3(0.0)
  if n == 1:
    v = coefs[0] * mat[0]
  elif n == 2:
    v = coefs[0] * mat[0] + coefs[1] * mat[1]
  elif n == 3:
    v = coefs[0] * mat[0] + coefs[1] * mat[1] + coefs[2] * mat[2]
  else:
    v = coefs[0] * mat[0] + coefs[1] * mat[1] + coefs[2] * mat[2] + coefs[3] * mat[3]
  return v


@wp.func
def _almost_equal(v1: wp.vec3, v2: wp.vec3) -> bool:
  return wp.abs(v1[0] - v2[0]) < MINVAL and wp.abs(v1[1] - v2[1]) < MINVAL and wp.abs(v1[2] - v2[2]) < MINVAL


@wp.func
def _subdistance(n: int, simplex: mat43) -> wp.vec4:
  if n == 4:
    return _S3D(simplex[0], simplex[1], simplex[2], simplex[3])
  if n == 3:
    coordinates3 = _S2D(simplex[0], simplex[1], simplex[2])
    return wp.vec4(coordinates3[0], coordinates3[1], coordinates3[2], 0.0)
  if n == 2:
    coordinates2 = _S1D(simplex[0], simplex[1])
    return wp.vec4(coordinates2[0], coordinates2[1], 0.0, 0.0)
  return wp.vec4(1.0, 0.0, 0.0, 0.0)


@wp.func
def _det3(v1: wp.vec3, v2: wp.vec3, v3: wp.vec3) -> float:
  return wp.dot(v1, wp.cross(v2, v3))


@wp.func
def _same_sign(a: float, b: float) -> int:
  if a > 0 and b > 0:
    return 1
  if a < 0 and b < 0:
    return -1
  return 0


@wp.func
def _project_origin_line(v1: wp.vec3, v2: wp.vec3) -> wp.vec3:
  diff = v2 - v1
  scl = -(wp.dot(v2, diff) / wp.dot(diff, diff))
  return v2 + scl * diff


@wp.func
def _project_origin_plane(v1: wp.vec3, v2: wp.vec3, v3: wp.vec3) -> Tuple[wp.vec3, int]:
  z = wp.vec3(0.0)
  diff21 = v2 - v1
  diff31 = v3 - v1
  diff32 = v3 - v2

  # n = (v1 - v2) x (v3 - v2)
  n = wp.cross(diff32, diff21)
  nv = wp.dot(n, v2)
  nn = wp.dot(n, n)
  if nn == 0:
    return z, 1
  if nv != 0 and nn > MINVAL:
    v = (nv / nn) * n
    return v, 0

  # n = (v2 - v1) x (v3 - v1)
  n = wp.cross(diff21, diff31)
  nv = wp.dot(n, v1)
  nn = wp.dot(n, n)
  if nn == 0:
    return z, 1
  if nv != 0 and nn > MINVAL:
    v = (nv / nn) * n
    return v, 0

  # n = (v1 - v3) x (v2 - v3)
  n = wp.cross(diff31, diff32)
  nv = wp.dot(n, v3)
  nn = wp.dot(n, n)
  v = (nv / nn) * n
  return v, 0


@wp.func
def _S3D(s1: wp.vec3, s2: wp.vec3, s3: wp.vec3, s4: wp.vec3) -> wp.vec4:
  #  [[ s1_x, s2_x, s3_x, s4_x ],
  #   [ s1_y, s2_y, s3_y, s4_y ],
  #   [ s1_z, s2_z, s3_z, s4_z ],
  #   [ 1,    1,    1,    1    ]]
  # we want to solve M*lambda = P, where P = [p_x, p_y, p_z, 1] with [p_x, p_y, p_z] is the
  # origin projected onto the simplex

  # compute cofactors to find det(M)
  C41 = -_det3(s2, s3, s4)
  C42 = _det3(s1, s3, s4)
  C43 = -_det3(s1, s2, s4)
  C44 = _det3(s1, s2, s3)

  # NOTE: m_det = 6*SignVol(simplex) with C4i corresponding to the volume of the 3-simplex
  # with vertices {s1, s2, s3, 0} - si
  m_det = C41 + C42 + C43 + C44

  comp1 = _same_sign(m_det, C41)
  comp2 = _same_sign(m_det, C42)
  comp3 = _same_sign(m_det, C43)
  comp4 = _same_sign(m_det, C44)

  # if all signs are the same then the origin is inside the simplex
  if comp1 and comp2 and comp3 and comp4:
    return wp.vec4(C41 / m_det, C42 / m_det, C43 / m_det, C44 / m_det)

  # find the smallest distance, and use the corresponding barycentric coordinates
  coordinates = wp.vec4(0.0, 0.0, 0.0, 0.0)
  dmin = FLOAT_MAX

  if not comp1:
    subcoord = _S2D(s2, s3, s4)
    x = subcoord[0] * s2 + subcoord[1] * s3 + subcoord[2] * s4
    d = wp.dot(x, x)
    coordinates[0] = 0.0
    coordinates[1] = subcoord[0]
    coordinates[2] = subcoord[1]
    coordinates[3] = subcoord[2]
    dmin = d

  if not comp2:
    subcoord = _S2D(s1, s3, s4)
    x = subcoord[0] * s1 + subcoord[1] * s3 + subcoord[2] * s4
    d = wp.dot(x, x)
    if d < dmin:
      coordinates[0] = subcoord[0]
      coordinates[1] = 0.0
      coordinates[2] = subcoord[1]
      coordinates[3] = subcoord[2]
      dmin = d

  if not comp3:
    subcoord = _S2D(s1, s2, s4)
    x = subcoord[0] * s1 + subcoord[1] * s2 + subcoord[2] * s4
    d = wp.dot(x, x)
    if d < dmin:
      coordinates[0] = subcoord[0]
      coordinates[1] = subcoord[1]
      coordinates[2] = 0.0
      coordinates[3] = subcoord[2]
      dmin = d

  if not comp4:
    subcoord = _S2D(s1, s2, s3)
    x = subcoord[0] * s1 + subcoord[1] * s2 + subcoord[2] * s3
    d = wp.dot(x, x)
    if d < dmin:
      coordinates[0] = subcoord[0]
      coordinates[1] = subcoord[1]
      coordinates[2] = subcoord[2]
      coordinates[3] = 0.0
  return coordinates


@wp.func
def _S2D(s1: wp.vec3, s2: wp.vec3, s3: wp.vec3) -> wp.vec3:
  # project origin onto affine hull of the simplex
  p_o, ret = _project_origin_plane(s1, s2, s3)
  if ret:
    v = _S1D(s1, s2)
    return wp.vec3(v[0], v[1], 0.0)

  # Below are the minors M_i4 of the matrix M given by
  # [[ s1_x, s2_x, s3_x, s4_x ],
  #  [ s1_y, s2_y, s3_y, s4_y ],
  #  [ s1_z, s2_z, s3_z, s4_z ],
  #  [ 1,    1,    1,    1    ]]
  M_14 = s2[1] * s3[2] - s2[2] * s3[1] - s1[1] * s3[2] + s1[2] * s3[1] + s1[1] * s2[2] - s1[2] * s2[1]
  M_24 = s2[0] * s3[2] - s2[2] * s3[0] - s1[0] * s3[2] + s1[2] * s3[0] + s1[0] * s2[2] - s1[2] * s2[0]
  M_34 = s2[0] * s3[1] - s2[1] * s3[0] - s1[0] * s3[1] + s1[1] * s3[0] + s1[0] * s2[1] - s1[1] * s2[0]

  # exclude the axis with the largest projection of the simplex using the computed minors
  M_max = 0.0
  s1_2D = wp.vec2(0.0)
  s2_2D = wp.vec2(0.0)
  s3_2D = wp.vec2(0.0)
  p_o_2D = wp.vec2(0.0)

  mu1 = wp.abs(M_14)
  mu2 = wp.abs(M_24)
  mu3 = wp.abs(M_34)

  if mu1 >= mu2 and mu1 >= mu3:
    M_max = M_14
    s1_2D[0] = s1[1]
    s1_2D[1] = s1[2]

    s2_2D[0] = s2[1]
    s2_2D[1] = s2[2]

    s3_2D[0] = s3[1]
    s3_2D[1] = s3[2]

    p_o_2D[0] = p_o[1]
    p_o_2D[1] = p_o[2]
  elif mu2 >= mu3:
    M_max = M_24
    s1_2D[0] = s1[0]
    s1_2D[1] = s1[2]

    s2_2D[0] = s2[0]
    s2_2D[1] = s2[2]

    s3_2D[0] = s3[0]
    s3_2D[1] = s3[2]

    p_o_2D[0] = p_o[0]
    p_o_2D[1] = p_o[2]
  else:
    M_max = M_34
    s1_2D[0] = s1[0]
    s1_2D[1] = s1[1]

    s2_2D[0] = s2[0]
    s2_2D[1] = s2[1]

    s3_2D[0] = s3[0]
    s3_2D[1] = s3[1]

    p_o_2D[0] = p_o[0]
    p_o_2D[1] = p_o[1]

  # compute the cofactors C3i of the following matrix:
  # [[ s1_2D[0] - p_o_2D[0], s2_2D[0] - p_o_2D[0], s3_2D[0] - p_o_2D[0] ],
  #  [ s1_2D[1] - p_o_2D[1], s2_2D[1] - p_o_2D[1], s3_2D[1] - p_o_2D[1] ],
  #  [ 1,                    1,                    1                    ]]

  # C31 corresponds to the signed area of 2-simplex: (p_o_2D, s2_2D, s3_2D)
  C31 = (
    p_o_2D[0] * s2_2D[1]
    + p_o_2D[1] * s3_2D[0]
    + s2_2D[0] * s3_2D[1]
    - p_o_2D[0] * s3_2D[1]
    - p_o_2D[1] * s2_2D[0]
    - s3_2D[0] * s2_2D[1]
  )

  # C32 corresponds to the signed area of 2-simplex: (_po_2D, s1_2D, s3_2D)
  C32 = (
    p_o_2D[0] * s3_2D[1]
    + p_o_2D[1] * s1_2D[0]
    + s3_2D[0] * s1_2D[1]
    - p_o_2D[0] * s1_2D[1]
    - p_o_2D[1] * s3_2D[0]
    - s1_2D[0] * s3_2D[1]
  )

  # C33 corresponds to the signed area of 2-simplex: (p_o_2D, s1_2D, s2_2D)
  C33 = (
    p_o_2D[0] * s1_2D[1]
    + p_o_2D[1] * s2_2D[0]
    + s1_2D[0] * s2_2D[1]
    - p_o_2D[0] * s2_2D[1]
    - p_o_2D[1] * s1_2D[0]
    - s2_2D[0] * s1_2D[1]
  )

  comp1 = _same_sign(M_max, C31)
  comp2 = _same_sign(M_max, C32)
  comp3 = _same_sign(M_max, C33)

  # all the same sign, p_o is inside the 2-simplex
  if comp1 and comp2 and comp3:
    return wp.vec3(C31 / M_max, C32 / M_max, C33 / M_max)

  # find the smallest distance, and use the corresponding barycentric coordinates
  dmin = FLOAT_MAX
  coordinates = wp.vec3(0.0, 0.0, 0.0)

  if not comp1:
    subcoord = _S1D(s2, s3)
    x = subcoord[0] * s2 + subcoord[1] * s3
    d = wp.dot(x, x)
    coordinates[0] = 0.0
    coordinates[1] = subcoord[0]
    coordinates[2] = subcoord[1]
    dmin = d

  if not comp2:
    subcoord = _S1D(s1, s3)
    x = subcoord[0] * s1 + subcoord[1] * s3
    d = wp.dot(x, x)
    if d < dmin:
      coordinates[0] = subcoord[0]
      coordinates[1] = 0.0
      coordinates[2] = subcoord[1]
      dmin = d

  if not comp3:
    subcoord = _S1D(s1, s2)
    x = subcoord[0] * s1 + subcoord[1] * s2
    d = wp.dot(x, x)
    if d < dmin:
      coordinates[0] = subcoord[0]
      coordinates[1] = subcoord[1]
      coordinates[2] = 0.0
  return coordinates


@wp.func
def _S1D(s1: wp.vec3, s2: wp.vec3) -> wp.vec2:
  # find projection of origin onto the 1-simplex:
  p_o = _project_origin_line(s1, s2)

  # find the axis with the largest projection "shadow" of the simplex
  mu_max = 0.0
  index = 0
  for i in range(3):
    mu = s1[i] - s2[i]
    if wp.abs(mu) >= wp.abs(mu_max):
      mu_max = mu
      index = i

  C1 = p_o[index] - s2[index]
  C2 = s1[index] - p_o[index]

  # inside the simplex
  if _same_sign(mu_max, C1) and _same_sign(mu_max, C2):
    return wp.vec2(C1 / mu_max, C2 / mu_max)
  return wp.vec2(0.0, 1.0)


@wp.func
def gjk(
  # In:
  tolerance: float,
  gjk_iterations: int,
  geom1: Geom,
  geom2: Geom,
  x1_0: wp.vec3,
  x2_0: wp.vec3,
  geomtype1: int,
  geomtype2: int,
  cutoff: float,
  is_discrete: bool,
) -> GJKResult:
  """Find distance within a tolerance between two geoms."""
  cutoff2 = cutoff * cutoff
  simplex = mat43()
  simplex1 = mat43()
  simplex2 = mat43()
  simplex_index1 = wp.vec4i()
  simplex_index2 = wp.vec4i()
  n = int(0)
  cnt = int(1)
  coordinates = wp.vec4()  # barycentric coordinates
  epsilon = wp.where(is_discrete, 0.0, 0.5 * tolerance * tolerance)

  # set initial guess
  x_k = x1_0 - x2_0

  for _ in range(gjk_iterations):
    xnorm = wp.dot(x_k, x_k)
    # TODO(kbayes): determine new constant here
    if xnorm < 1e-12:
      break
    dir_neg = x_k / wp.sqrt(xnorm)

    # compute kth support point in geom1
    sp = _support(geom1, geomtype1, -dir_neg)
    simplex1[n] = sp.point
    geom1.index = sp.cached_index
    simplex_index1[n] = sp.vertex_index

    # compute kth support point in geom2
    sp = _support(geom2, geomtype2, dir_neg)
    simplex2[n] = sp.point
    geom2.index = sp.cached_index
    simplex_index2[n] = sp.vertex_index

    # compute the kth support point
    simplex[n] = simplex1[n] - simplex2[n]

    if cutoff == 0.0:
      if wp.dot(x_k, simplex[n]) > 0:
        result = GJKResult()
        result.dim = 0
        result.dist = FLOAT_MAX
        return result
    elif cutoff < FLOAT_MAX:
      vs = wp.dot(x_k, simplex[n])
      vv = wp.dot(x_k, x_k)
      if wp.dot(x_k, simplex[n]) > 0 and (vs * vs / vv) >= cutoff2:
        result = GJKResult()
        result.dim = 0
        result.dist = FLOAT_MAX
        return result

    # stopping criteria using the Frank-Wolfe duality gap given by
    #  |f(x_k) - f(x_min)|^2 <= < grad f(x_k), (x_k - simplex[n]) >
    if wp.dot(x_k, x_k - simplex[n]) < epsilon:
      break

    # run the distance subalgorithm to compute the barycentric coordinates
    # of the closest point to the origin in the simplex
    coordinates = _subdistance(n + 1, simplex)

    # remove vertices from the simplex no longer needed
    n = int(0)
    for i in range(4):
      if coordinates[i] == 0:
        continue

      simplex[n] = simplex[i]
      simplex1[n] = simplex1[i]
      simplex2[n] = simplex2[i]
      simplex_index1[n] = simplex_index1[i]
      simplex_index2[n] = simplex_index2[i]
      coordinates[n] = coordinates[i]
      n += int(1)

    # SHOULD NOT OCCUR
    if n < 1:
      break

    # get the next iteration of x_k
    x_next = _linear_combine(n, coordinates, simplex)

    # x_k has converged to minimum
    if _almost_equal(x_next, x_k):
      break

    # copy next iteration into x_k
    x_k = x_next

    # we have a tetrahedron containing the origin so return early
    if n == 4:
      break

    cnt += 1

  if cnt == gjk_iterations:
    wp.printf("Warning: opt.ccd_iterations, currently set to %d, needs to be increased.\n", gjk_iterations)

  result = GJKResult()

  # compute the approximate witness points
  # if n is zero, then there was an immediate return meaning the initial points
  # are the witness points
  result.x1 = wp.where(n == 0, x1_0, _linear_combine(n, coordinates, simplex1))
  result.x2 = wp.where(n == 0, x2_0, _linear_combine(n, coordinates, simplex2))
  result.dist = wp.norm_l2(x_k)

  result.dim = n
  result.simplex1 = simplex1
  result.simplex2 = simplex2
  result.simplex_index1 = simplex_index1
  result.simplex_index2 = simplex_index2
  result.simplex = simplex
  return result


@wp.func
def _same_side(p0: wp.vec3, p1: wp.vec3, p2: wp.vec3, p3: wp.vec3) -> bool:
  n = wp.cross(p1 - p0, p2 - p0)
  dot1 = wp.dot(n, p3 - p0)
  dot2 = wp.dot(n, -p0)
  return (dot1 > 0 and dot2 > 0) or (dot1 < 0 and dot2 < 0)


@wp.func
def _test_tetra(p0: wp.vec3, p1: wp.vec3, p2: wp.vec3, p3: wp.vec3) -> bool:
  return _same_side(p0, p1, p2, p3) and _same_side(p1, p2, p3, p0) and _same_side(p2, p3, p0, p1) and _same_side(p3, p0, p1, p2)


@wp.func
def _tri_affine_coord(v1: wp.vec3, v2: wp.vec3, v3: wp.vec3, p: wp.vec3) -> wp.vec3:
  # compute minors as in S2D
  M_14 = v2[1] * v3[2] - v2[2] * v3[1] - v1[1] * v3[2] + v1[2] * v3[1] + v1[1] * v2[2] - v1[2] * v2[1]
  M_24 = v2[0] * v3[2] - v2[2] * v3[0] - v1[0] * v3[2] + v1[2] * v3[0] + v1[0] * v2[2] - v1[2] * v2[0]
  M_34 = v2[0] * v3[1] - v2[1] * v3[0] - v1[0] * v3[1] + v1[1] * v3[0] + v1[0] * v2[1] - v1[1] * v2[0]

  # exclude one of the axes with the largest projection
  # of the simplex using the computed minors
  M_max = 0.0
  x = 0
  y = 0

  mu1 = wp.abs(M_14)
  mu2 = wp.abs(M_24)
  mu3 = wp.abs(M_34)

  if mu1 >= mu2 and mu1 >= mu3:
    M_max = M_14
    x = 1
    y = 2
  elif mu2 >= mu3:
    M_max = M_24
    x = 0
    y = 2
  else:
    M_max = M_34
    x = 0
    y = 1

  # C31 corresponds to the signed area of 2-simplex: (v, s2, s3)
  C31 = p[x] * v2[y] + p[y] * v3[x] + v2[x] * v3[y] - p[x] * v3[y] - p[y] * v2[x] - v3[x] * v2[y]

  # C32 corresponds to the signed area of 2-simplex: (v, s1, s3)
  C32 = p[x] * v3[y] + p[y] * v1[x] + v3[x] * v1[y] - p[x] * v1[y] - p[y] * v3[x] - v1[x] * v3[y]

  # C33 corresponds to the signed area of 2-simplex: (v, s1, s2)
  C33 = p[x] * v1[y] + p[y] * v2[x] + v1[x] * v2[y] - p[x] * v2[y] - p[y] * v1[x] - v2[x] * v1[y]

  # compute affine coordinates
  return wp.vec3(C31 / M_max, C32 / M_max, C33 / M_max)


@wp.func
def _tri_point_intersect(v1: wp.vec3, v2: wp.vec3, v3: wp.vec3, p: wp.vec3) -> bool:
  coordinates = _tri_affine_coord(v1, v2, v3, p)
  l1 = coordinates[0]
  l2 = coordinates[1]
  l3 = coordinates[2]

  if l1 < 0 or l2 < 0 or l3 < 0:
    return False

  pr = wp.vec3()
  pr[0] = v1[0] * l1 + v2[0] * l2 + v3[0] * l3
  pr[1] = v1[1] * l1 + v2[1] * l2 + v3[1] * l3
  pr[2] = v1[2] * l1 + v2[2] * l2 + v3[2] * l3
  return wp.norm_l2(pr - p) < MINVAL


@wp.func
def _replace_simplex3(pt: Polytope, v1: int, v2: int, v3: int) -> GJKResult:
  result = GJKResult()

  # reset GJK simplex
  simplex = mat43()
  simplex[0] = pt.vert[v1]
  simplex[1] = pt.vert[v2]
  simplex[2] = pt.vert[v3]

  simplex1 = mat43()
  simplex1[0] = pt.vert1[v1]
  simplex1[1] = pt.vert1[v2]
  simplex1[2] = pt.vert1[v3]

  simplex2 = mat43()
  simplex2[0] = pt.vert2[v1]
  simplex2[1] = pt.vert2[v2]
  simplex2[2] = pt.vert2[v3]

  simplex_index1 = wp.vec4i()
  simplex_index1[0] = pt.vert_index1[v1]
  simplex_index1[1] = pt.vert_index1[v2]
  simplex_index1[2] = pt.vert_index1[v3]

  simplex_index2 = wp.vec4i()
  simplex_index2[0] = pt.vert_index2[v1]
  simplex_index2[1] = pt.vert_index2[v2]
  simplex_index2[2] = pt.vert_index2[v3]

  result.simplex = simplex
  result.simplex1 = simplex1
  result.simplex2 = simplex2
  result.simplex_index1 = simplex_index1
  result.simplex_index2 = simplex_index2

  return result


@wp.func
def _rotmat(axis: wp.vec3) -> wp.mat33:
  n = wp.norm_l2(axis)
  u1 = axis[0] / n
  u2 = axis[1] / n
  u3 = axis[2] / n

  sin = 0.86602540378  # sin(120 deg)
  cos = -0.5  # cos(120 deg)
  R = wp.mat33()
  R[0, 0] = cos + u1 * u1 * (1.0 - cos)
  R[0, 1] = u1 * u2 * (1.0 - cos) - u3 * sin
  R[0, 2] = u1 * u3 * (1.0 - cos) + u2 * sin
  R[1, 0] = u2 * u1 * (1.0 - cos) + u3 * sin
  R[1, 1] = cos + u2 * u2 * (1.0 - cos)
  R[1, 2] = u2 * u3 * (1.0 - cos) - u1 * sin
  R[2, 0] = u1 * u3 * (1.0 - cos) - u2 * sin
  R[2, 1] = u2 * u3 * (1.0 - cos) + u1 * sin
  R[2, 2] = cos + u3 * u3 * (1.0 - cos)
  return R


@wp.func
def _ray_triangle(v1: wp.vec3, v2: wp.vec3, v3: wp.vec3, v4: wp.vec3, v5: wp.vec3) -> int:
  vol1 = _det3(v3 - v1, v4 - v1, v2 - v1)
  vol2 = _det3(v4 - v1, v5 - v1, v2 - v1)
  vol3 = _det3(v5 - v1, v3 - v1, v2 - v1)

  if vol1 >= 0 and vol2 >= 0 and vol3 >= 0:
    return 1
  if vol1 <= 0 and vol2 <= 0 and vol3 <= 0:
    return -1
  return 0


@wp.func
def _add_edge(pt: Polytope, e1: int, e2: int) -> int:
  n = pt.nhorizon

  if n < 0:
    return -1

  for i in range(n):
    old_e1 = pt.horizon[2 * i + 0]
    old_e2 = pt.horizon[2 * i + 1]
    if (old_e1 == e1 and old_e2 == e2) or (old_e1 == e2 and old_e2 == e1):
      pt.horizon[2 * i + 0] = pt.horizon[2 * (n - 1) + 0]
      pt.horizon[2 * i + 1] = pt.horizon[2 * (n - 1) + 1]
      return n - 1

  # out of memory, force EPA to return early without contact
  if n > pt.horizon.shape[0] - 2:
    return -1

  pt.horizon[2 * n + 0] = e1
  pt.horizon[2 * n + 1] = e2
  return n + 1


@wp.func
def _delete_face(pt: Polytope, face_id: int) -> int:
  index = pt.face_index[face_id]
  # delete from map
  if index >= 0:
    last_face = pt.face_map[pt.nmap - 1]
    pt.face_map[index] = last_face
    pt.face_index[last_face] = index
    pt.nmap -= 1
  # mark face as deleted from polytope
  pt.face_index[face_id] = -2
  return pt.nmap


@wp.func
def _epa_witness(
  pt: Polytope, geom1: Geom, geom2: Geom, geomtype1: int, geomtype2: int, face_idx: int
) -> Tuple[wp.vec3, wp.vec3, float]:
  face = pt.face[face_idx]

  # compute affine coordinates for witness points on plane defined by face
  v1 = pt.vert[face[0]]
  v2 = pt.vert[face[1]]
  v3 = pt.vert[face[2]]

  coordinates = _tri_affine_coord(v1, v2, v3, pt.face_pr[face_idx])
  l1 = coordinates[0]
  l2 = coordinates[1]
  l3 = coordinates[2]

  # face on geom 2
  v1 = pt.vert2[face[0]]
  v2 = pt.vert2[face[1]]
  v3 = pt.vert2[face[2]]
  x2 = wp.vec3()
  x2[0] = v1[0] * l1 + v2[0] * l2 + v3[0] * l3
  x2[1] = v1[1] * l1 + v2[1] * l2 + v3[1] * l3
  x2[2] = v1[2] * l1 + v2[2] * l2 + v3[2] * l3

  # correct witness points for hfield geoms
  i1 = pt.vert_index1[face[0]]
  i2 = pt.vert_index1[face[1]]
  i3 = pt.vert_index1[face[2]]
  if geomtype1 == GeomType.HFIELD and (i1 != i2 or i1 != i3):
    # TODO(kbayes): Fix case where geom2 is near bottom of height field or "extreme" prism heights
    n = geom1.rot[:, 2]
    a = geom1.hfprism[3]
    b = geom1.hfprism[4]
    c = geom1.hfprism[5]
    x2 = wp.normalize(x2)

    # TODO(kbayes): Support cases where geom2 is larger than the height field
    sp = _support(geom2, geomtype2, x2)
    x2 = sp.point

    coordinates2 = _tri_affine_coord(a, b, c, x2)
    if coordinates2[0] > 0 and coordinates2[1] > 0 and coordinates2[2] > 0:
      x1 = coordinates[0] * a + coordinates[1] * b + coordinates[2] * c
    else:
      p = c
      p = wp.where(coordinates[1] > 0, b, p)
      p = wp.where(coordinates[0] > 0, a, p)
      x1 = x2 - wp.dot(x2 - p, n) * n
    return x1, x2, -wp.norm_l2(x1 - x2)

  # face on geom 1
  v1 = pt.vert1[face[0]]
  v2 = pt.vert1[face[1]]
  v3 = pt.vert1[face[2]]
  x1 = wp.vec3()
  x1[0] = v1[0] * l1 + v2[0] * l2 + v3[0] * l3
  x1[1] = v1[1] * l1 + v2[1] * l2 + v3[1] * l3
  x1[2] = v1[2] * l1 + v2[2] * l2 + v3[2] * l3

  return x1, x2, -wp.sqrt(pt.face_norm2[face_idx])


@wp.func
def _polytope2(
  # In:
  pt: Polytope,
  dist: float,
  simplex: mat43,
  simplex1: mat43,
  simplex2: mat43,
  simplex_index1: wp.vec4i,
  simplex_index2: wp.vec4i,
  geom1: Geom,
  geom2: Geom,
  geomtype1: int,
  geomtype2: int,
) -> Tuple[Polytope, GJKResult]:
  """Create polytope for EPA given a 1-simplex from GJK."""
  diff = simplex[1] - simplex[0]

  # find component with smallest magnitude (so cross product is largest)
  value = FLOAT_MAX
  index = 0
  for i in range(3):
    if wp.abs(diff[i]) < value:
      value = wp.abs(diff[i])
      index = i

  # cross product with best coordinate axis
  e = wp.vec3(0.0, 0.0, 0.0)
  e[index] = 1.0
  d1 = wp.cross(e, diff)

  # rotate around the line segment to get three more points spaced 120 degrees apart
  R = _rotmat(diff)
  d2 = R @ d1
  d3 = R @ d2

  # save vertices and get indices for each one
  pt.vert[0] = simplex[0]
  pt.vert[1] = simplex[1]

  pt.vert1[0] = simplex1[0]
  pt.vert1[1] = simplex1[1]

  pt.vert_index1[0] = simplex_index1[0]
  pt.vert_index1[1] = simplex_index1[1]

  pt.vert2[0] = simplex2[0]
  pt.vert2[1] = simplex2[1]

  pt.vert_index2[0] = simplex_index2[0]
  pt.vert_index2[1] = simplex_index2[1]

  _epa_support(pt, 2, geom1, geom2, geomtype1, geomtype2, d1 / wp.norm_l2(d1))
  _epa_support(pt, 3, geom1, geom2, geomtype1, geomtype2, d2 / wp.norm_l2(d2))
  _epa_support(pt, 4, geom1, geom2, geomtype1, geomtype2, d3 / wp.norm_l2(d3))

  # build hexahedron
  if _attach_face(pt, 0, 0, 2, 3) < MIN_DIST:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 2, 3)

  if _attach_face(pt, 1, 0, 4, 2) < MIN_DIST:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 4, 2)

  if _attach_face(pt, 2, 0, 3, 4) < MIN_DIST:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 3, 4)

  if _attach_face(pt, 3, 1, 3, 2) < MIN_DIST:
    pt.status = -1
    return pt, _replace_simplex3(pt, 1, 3, 2)

  if _attach_face(pt, 4, 1, 2, 4) < MIN_DIST:
    pt.status = -1
    return pt, _replace_simplex3(pt, 1, 2, 4)

  if _attach_face(pt, 5, 1, 4, 3) < MIN_DIST:
    pt.status = -1
    return pt, _replace_simplex3(pt, 1, 4, 3)

  # check hexahedron is convex
  if not _ray_triangle(simplex[0], simplex[1], pt.vert[2], pt.vert[3], pt.vert[4]):
    pt.status = 1
    return pt, GJKResult()

  # populate face map
  for i in range(6):
    pt.face_map[i] = i
    pt.face_index[i] = i

  # set polytope counts
  pt.nvert = 5
  pt.nface = 6
  pt.nmap = 6
  pt.status = 0
  return pt, GJKResult()


@wp.func
def _polytope3(
  # In:
  pt: Polytope,
  dist: float,
  simplex: mat43,
  simplex1: mat43,
  simplex2: mat43,
  simplex_index1: wp.vec4i,
  simplex_index2: wp.vec4i,
  geom1: Geom,
  geom2: Geom,
  geomtype1: int,
  geomtype2: int,
) -> Polytope:
  """Create polytope for EPA given a 2-simplex from GJK."""
  # get normals in both directions
  n = wp.cross(simplex[1] - simplex[0], simplex[2] - simplex[0])
  if wp.norm_l2(n) < MINVAL:
    pt.status = 2
    return pt

  pt.vert[0] = simplex[0]
  pt.vert[1] = simplex[1]
  pt.vert[2] = simplex[2]

  pt.vert1[0] = simplex1[0]
  pt.vert1[1] = simplex1[1]
  pt.vert1[2] = simplex1[2]

  pt.vert_index1[0] = simplex_index1[0]
  pt.vert_index1[1] = simplex_index1[1]
  pt.vert_index1[2] = simplex_index1[2]

  pt.vert2[0] = simplex2[0]
  pt.vert2[1] = simplex2[1]
  pt.vert2[2] = simplex2[2]

  pt.vert_index2[0] = simplex_index2[0]
  pt.vert_index2[1] = simplex_index2[1]
  pt.vert_index2[2] = simplex_index2[2]

  _epa_support(pt, 3, geom1, geom2, geomtype1, geomtype2, -n)
  _epa_support(pt, 4, geom1, geom2, geomtype1, geomtype2, n)

  v1 = simplex[0]
  v2 = simplex[1]
  v3 = simplex[2]
  v4 = pt.vert[3]
  v5 = pt.vert[4]

  # check that v4 is not contained in the 2-simplex
  if _tri_point_intersect(v1, v2, v3, v4):
    pt.status = 3
    return pt

  # check that v5 is not contained in the 2-simplex
  if _tri_point_intersect(v1, v2, v3, v5):
    pt.status = 4
    return pt

  # if origin does not lie on simplex then we need to check that the hexahedron contains the
  # origin
  if dist > 1e-5 and not _test_tetra(v1, v2, v3, v4) and not _test_tetra(v1, v2, v3, v5):
    pt.status = 5
    return pt

  # create hexahedron for EPA
  if _attach_face(pt, 0, 4, 0, 1) < MIN_DIST:
    pt.status = 6
    return pt
  if _attach_face(pt, 1, 4, 2, 0) < MIN_DIST:
    pt.status = 7
    return pt
  if _attach_face(pt, 2, 4, 1, 2) < MIN_DIST:
    pt.status = 8
    return pt
  if _attach_face(pt, 3, 3, 1, 0) < MIN_DIST:
    pt.status = 9
    return pt
  if _attach_face(pt, 4, 3, 0, 2) < MIN_DIST:
    pt.status = 10
    return pt
  if _attach_face(pt, 5, 3, 2, 1) < MIN_DIST:
    pt.status = 11
    return pt

  # populate face map
  for i in range(6):
    pt.face_map[i] = i
    pt.face_index[i] = i

  # set polytope counts
  pt.nvert = 5
  pt.nface = 6
  pt.nmap = 6
  pt.status = 0
  return pt


@wp.func
def _polytope4(
  # In:
  pt: Polytope,
  dist: float,
  simplex: mat43,
  simplex1: mat43,
  simplex2: mat43,
  simplex_index1: wp.vec4i,
  simplex_index2: wp.vec4i,
  geom1: Geom,
  geom2: Geom,
  geomtype1: int,
  geomtype2: int,
) -> Tuple[Polytope, GJKResult]:
  """Create polytope for EPA given a 3-simplex from GJK."""
  pt.vert[0] = simplex[0]
  pt.vert[1] = simplex[1]
  pt.vert[2] = simplex[2]
  pt.vert[3] = simplex[3]

  pt.vert1[0] = simplex1[0]
  pt.vert1[1] = simplex1[1]
  pt.vert1[2] = simplex1[2]
  pt.vert1[3] = simplex1[3]

  pt.vert_index1[0] = simplex_index1[0]
  pt.vert_index1[1] = simplex_index1[1]
  pt.vert_index1[2] = simplex_index1[2]
  pt.vert_index1[3] = simplex_index1[3]

  pt.vert2[0] = simplex2[0]
  pt.vert2[1] = simplex2[1]
  pt.vert2[2] = simplex2[2]
  pt.vert2[3] = simplex2[3]

  pt.vert_index2[0] = simplex_index2[0]
  pt.vert_index2[1] = simplex_index2[1]
  pt.vert_index2[2] = simplex_index2[2]
  pt.vert_index2[3] = simplex_index2[3]

  # if the origin is on a face, replace the 3-simplex with a 2-simplex
  if _attach_face(pt, 0, 0, 1, 2) < MIN_DIST:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 1, 2)

  if _attach_face(pt, 1, 0, 3, 1) < MIN_DIST:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 3, 1)

  if _attach_face(pt, 2, 0, 2, 3) < MIN_DIST:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 2, 3)

  if _attach_face(pt, 3, 3, 2, 1) < MIN_DIST:
    pt.status = -1
    return pt, _replace_simplex3(pt, 3, 2, 1)

  if not _test_tetra(pt.vert[0], pt.vert[1], pt.vert[2], pt.vert[3]):
    pt.status = 12
    return pt, GJKResult()

  # populate face map
  for i in range(4):
    pt.face_map[i] = i
    pt.face_index[i] = i

  # set polytope counts
  pt.nvert = 4
  pt.nface = 4
  pt.nmap = 4
  pt.status = 0
  return pt, GJKResult()


@wp.func
def _epa(
  # In:
  tolerance: float,
  gjk_iterations: int,
  epa_iterations: int,
  pt: Polytope,
  geom1: Geom,
  geom2: Geom,
  geomtype1: int,
  geomtype2: int,
  is_discrete: bool,
) -> Tuple[float, wp.vec3, wp.vec3, int]:
  """Recover penetration data from two geoms in contact given an initial polytope."""
  upper = FLOAT_MAX
  upper2 = FLOAT_MAX
  idx = int(-1)
  pidx = int(-1)
  epsilon = wp.where(is_discrete, 1e-15, tolerance)
  cnt = int(1)

  for _ in range(epa_iterations):
    pidx = idx
    idx = int(-1)

    # find the face closest to the origin (lower bound for penetration depth)
    lower2 = float(FLOAT_MAX)
    for i in range(pt.nmap):
      face_idx = pt.face_map[i]
      if pt.face_norm2[face_idx] < lower2:
        idx = int(face_idx)
        lower2 = float(pt.face_norm2[face_idx])

    # face not valid, return previous face
    if lower2 > upper2 or idx < 0:
      idx = pidx
      break

    # check if lower bound is 0
    if lower2 <= 0:
      break

    # compute support point w from the closest face's normal
    lower = wp.sqrt(lower2)
    wi = pt.nvert
    i1, i2 = _epa_support(pt, wi, geom1, geom2, geomtype1, geomtype2, pt.face_pr[idx] / lower)
    geom1.index = i1
    geom2.index = i2
    pt.nvert += 1

    # upper bound for kth iteration
    upper_k = wp.dot(pt.face_pr[idx], pt.vert[wi]) / lower
    if upper_k < upper:
      upper = upper_k
      upper2 = upper * upper

    if upper - lower < epsilon:
      break

    # check if vertex wi is a repeated support point
    if is_discrete:
      found_repeated = bool(False)
      for i in range(pt.nvert - 1):
        if pt.vert_index1[i] == pt.vert_index1[wi] and pt.vert_index2[i] == pt.vert_index2[wi]:
          found_repeated = True
          break
      if found_repeated:
        break

    pt.nmap = _delete_face(pt, idx)
    pt.nhorizon = _add_edge(pt, pt.face[idx][0], pt.face[idx][1])
    pt.nhorizon = _add_edge(pt, pt.face[idx][1], pt.face[idx][2])
    pt.nhorizon = _add_edge(pt, pt.face[idx][2], pt.face[idx][0])
    if pt.nhorizon == -1:
      idx = -1
      break

    # compute horizon for w
    for i in range(pt.nface):
      if pt.face_index[i] == -2:
        continue

      if wp.dot(pt.face_pr[i], pt.vert[wi]) - pt.face_norm2[i] > 1e-10:
        pt.nmap = _delete_face(pt, i)
        pt.nhorizon = _add_edge(pt, pt.face[i][0], pt.face[i][1])
        pt.nhorizon = _add_edge(pt, pt.face[i][1], pt.face[i][2])
        pt.nhorizon = _add_edge(pt, pt.face[i][2], pt.face[i][0])
        if pt.nhorizon == -1:
          idx = -1
          break

    # insert w as new vertex and attach faces along the horizon
    for i in range(pt.nhorizon):
      dist2 = _attach_face(pt, pt.nface, wi, pt.horizon[2 * i + 0], pt.horizon[2 * i + 1])
      if dist2 == 0:
        idx = -1
        break

      pt.nface += 1

      # store face in map
      if dist2 >= lower2 and dist2 <= upper2:
        pt.face_map[pt.nmap] = pt.nface - 1
        pt.face_index[pt.nface - 1] = pt.nmap
        pt.nmap += 1

    # no face candidates left
    if pt.nmap == 0 or idx == -1:
      break

    # clear horizon
    pt.nhorizon = 0
    cnt += 1

  if cnt == epa_iterations:
    wp.printf("Warning: opt.ccd_iterations, currently set to %d, needs to be increased.\n", gjk_iterations)

  # return from valid face
  if idx > -1:
    x1, x2, dist = _epa_witness(pt, geom1, geom2, geomtype1, geomtype2, idx)
    return dist, x1, x2, idx
  return 0.0, wp.vec3(), wp.vec3(), -1


@wp.func
def _area4(a: wp.vec3, b: wp.vec3, c: wp.vec3, d: wp.vec3) -> float:
  """Computes area of a quadrilateral embedded in 3D space."""
  return 0.5 * wp.norm_l2(wp.cross(a - d, d - b) + wp.cross(b - c, c - a))


@wp.func
def _next(n: int, i: int) -> int:
  """Returns (i + 1) mod n for 0 <= i <= n - 1."""
  return wp.where(i == n - 1, 0, i + 1)


@wp.func
def _polygon_quad(polygon: wp.array(dtype=wp.vec3), npolygon: int) -> wp.vec4i:
  """Returns the indices of a quadrilateral of maximum area in a convex polygon."""
  b = _next(npolygon, 0)
  c = _next(npolygon, b)
  d = _next(npolygon, c)
  res = wp.vec4i(0, b, c, d)
  m = _area4(polygon[0], polygon[b], polygon[c], polygon[d])
  for a in range(npolygon):
    while True:
      m_next = _area4(polygon[a], polygon[b], polygon[c], polygon[_next(npolygon, d)])
      if m_next <= m:
        break
      m = m_next
      d = _next(npolygon, d)
      res = wp.vec4i(a, b, c, d)
      while True:
        m_next = _area4(polygon[a], polygon[b], polygon[_next(npolygon, c)], polygon[d])
        if m_next <= m:
          break
        m = m_next
        c = _next(npolygon, c)
        res = wp.vec4i(a, b, c, d)
      while True:
        m_next = _area4(polygon[a], polygon[_next(npolygon, b)], polygon[c], polygon[d])
        if m_next <= m:
          break
        m = m_next
        b = _next(npolygon, b)
        res = wp.vec4i(a, b, c, d)
    if b == a:
      b = _next(npolygon, b)
      if c == b:
        c = _next(npolygon, c)
        if d == c:
          d == _next(npolygon, d)
  return res


# return number (1, 2 or 3) of dimensions of a simplex; reorder vertices if necessary
@wp.func
def _feature_dim(
  face: wp.vec3i, vert_index: wp.array(dtype=int), vert: wp.array(dtype=wp.vec3)
) -> Tuple[int, wp.vec3i, wp.mat33]:
  v1i = vert_index[face[0]]
  v2i = vert_index[face[1]]
  v3i = vert_index[face[2]]

  feature_index = wp.vec3i(v1i, v2i, v3i)
  feature_vert = wp.mat33()
  feature_vert[0] = vert[face[0]]
  feature_vert[1] = vert[face[1]]
  feature_vert[2] = vert[face[2]]

  if v1i != v2i:
    dim = wp.where(v3i == v1i or v3i == v2i, 2, 3)
    return dim, feature_index, feature_vert

  feature_index[1] = v3i
  feature_vert[1] = vert[face[2]]

  dim = wp.where(v1i != v3i, 2, 1)
  return dim, feature_index, feature_vert


# find two normals that are facing each other within a tolerance, return 1 if found
@wp.func
def _aligned_faces(
  vert1: wp.array(dtype=wp.vec3), len1: int, vert2: wp.array(dtype=wp.vec3), len2: int
) -> Tuple[int, wp.vec2i]:
  res = wp.vec2i()
  for i in range(len1):
    for j in range(len2):
      if wp.dot(vert1[i], vert2[j]) < -FACE_TOL:
        res[0] = i
        res[1] = j
        return 1, res
  return 0, res


# find two normals that are perpendicular to each other within a tolerance
# return 1 if found
@wp.func
def _aligned_face_edge(
  edge: wp.array(dtype=wp.vec3), nedge: int, face: wp.array(dtype=wp.vec3), nface: int
) -> Tuple[int, wp.vec2i]:
  res = wp.vec2i()
  for i in range(nface):
    for j in range(nedge):
      if wp.abs(wp.dot(edge[j], face[i])) < EDGE_TOL:
        res[0] = j
        res[1] = i
        return 1, res
  return 0, res


# find up to n <= 2 common integers of two arrays, return n
@wp.func
def _intersect1(
  a1: wp.array(dtype=int), a2: wp.array(dtype=int), start1: int, start2: int, len1: int, len2: int
) -> Tuple[int, wp.vec2i]:
  count = int(0)
  res = wp.vec2i()
  for i in range(start1, start1 + len1):
    for j in range(start2, start2 + len2):
      if a1[i] == a2[j]:
        res[count] = a1[i]
        count += 1
        if count == 2:
          return 2, res
  return count, res


@wp.func
def _intersect2(a1: wp.vec2i, a2: wp.array(dtype=int), start2: int, len1: int, len2: int) -> Tuple[int, wp.vec2i]:
  count = int(0)
  res = wp.vec2i()
  for i in range(len1):
    for j in range(start2, start2 + len2):
      if a1[i] == a2[j]:
        res[count] = a1[i]
        count += 1
        if count == 2:
          return 2, res
  return count, res


# compute possible polygon normals of a mesh given up to 3 vertices
@wp.func
def _mesh_normals(
  # In:
  feature_dim: int,
  feature_index: wp.vec3i,
  mat: wp.mat33,
  vertadr: int,
  polyadr: int,
  polynormal: wp.array(dtype=wp.vec3),
  polymapadr: wp.array(dtype=int),
  polymapnum: wp.array(dtype=int),
  polymap: wp.array(dtype=int),
  # Out:
  normals_out: wp.array(dtype=wp.vec3),
  indices_out: wp.array(dtype=int),
) -> int:
  v1 = feature_index[0]
  v2 = feature_index[1]
  v3 = feature_index[2]
  if feature_dim == 3:
    v1_adr = polymapadr[vertadr + v1]
    v1_num = polymapnum[vertadr + v1]

    v2_adr = polymapadr[vertadr + v2]
    v2_num = polymapnum[vertadr + v2]

    v3_adr = polymapadr[vertadr + v3]
    v3_num = polymapnum[vertadr + v3]

    faceset = wp.vec2i()
    n, edgeset = _intersect1(polymap, polymap, v1_adr, v2_adr, v1_num, v2_num)
    if n == 0:
      return 0
    n, faceset = _intersect2(edgeset, polymap, v3_adr, n, v3_num)
    if n == 0:
      return 0

    # three vertices on mesh define a unique face
    normals_out[0] = mat @ polynormal[polyadr + faceset[0]]
    indices_out[0] = faceset[0]
    return 1

  if feature_dim == 2:
    v1_adr = polymapadr[vertadr + v1]
    v1_num = polymapnum[vertadr + v1]

    v2_adr = polymapadr[vertadr + v2]
    v2_num = polymapnum[vertadr + v2]

    # up to two faces as two vertices define an edge
    n, edgeset = _intersect1(polymap, polymap, v1_adr, v2_adr, v1_num, v2_num)
    if n == 0:
      return 0
    for i in range(n):
      normals_out[i] = mat @ polynormal[polyadr + edgeset[i]]
      indices_out[i] = edgeset[i]
    return n

  if feature_dim == 1:
    v1_adr = polymapadr[vertadr + v1]
    v1_num = polymapnum[vertadr + v1]
    for i in range(v1_num):
      index = polymap[v1_adr + i]
      normals_out[i] = mat @ polynormal[polyadr + index]
      indices_out[i] = index
    return v1_num
  return 0


# compute normal directional vectors along possible edges given by up to two vertices
@wp.func
def _mesh_edge_normals(
  # In:
  dim: int,
  mat: wp.mat33,
  pos: wp.vec3,
  vertadr: int,
  polyadr: int,
  vert: wp.array(dtype=wp.vec3),
  polyvertadr: wp.array(dtype=int),
  polyvertnum: wp.array(dtype=int),
  polyvert: wp.array(dtype=int),
  polymapadr: wp.array(dtype=int),
  polymapnum: wp.array(dtype=int),
  polymap: wp.array(dtype=int),
  v1: wp.vec3,
  v2: wp.vec3,
  v1i: int,
  # Out:
  normals_out: wp.array(dtype=wp.vec3),
  endverts_out: wp.array(dtype=wp.vec3),
) -> int:
  # only one edge
  if dim == 2:
    endverts_out[0] = v2
    normals_out[0] = wp.normalize(v2 - v1)
    return 1

  if dim == 1:
    v1_adr = polymapadr[vertadr + v1i]
    v1_num = polymapnum[vertadr + v1i]

    # loop through all faces with vertex v1
    for i in range(v1_num):
      idx = polymap[v1_adr + i]
      adr = polyvertadr[polyadr + idx]
      nvert = polyvertnum[polyadr + idx]
      # find previous vertex in polygon to form edge
      for j in range(nvert):
        if polyvert[adr + j] == v1i:
          k = wp.where(j == 0, nvert - 1, j - 1)
          endverts_out[i] = mat @ vert[vertadr + polyvert[adr + k]] + pos
          normals_out[i] = wp.normalize(endverts_out[i] - v1)
    return v1_num
  return 0


# try recovering box normal from collision normal
@wp.func
def _box_normals2(
  # In:
  mat: wp.mat33,
  n: wp.vec3,
  # Out:
  normal_out: wp.array(dtype=wp.vec3),
  index_out: wp.array(dtype=int),
) -> int:
  # list of box face normals
  face_normals = mat63(1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0)

  # get local coordinates of the normal

  local_n = wp.normalize(
    wp.vec3(
      mat[0][0] * n[0] + mat[1][0] * n[1] + mat[2][0] * n[2],
      mat[0][1] * n[0] + mat[1][1] * n[1] + mat[2][1] * n[2],
      mat[0][2] * n[0] + mat[1][2] * n[1] + mat[2][2] * n[2],
    )
  )

  # determine if there is a side close to the normal
  for i in range(6):
    if wp.dot(local_n, face_normals[i]) > FACE_TOL:
      normal_out[0] = mat @ face_normals[i]
      index_out[0] = i
      return 1

  return 0


# compute possible face normals of a box given up to 3 vertices
@wp.func
def _box_normals(
  # In:
  feature_dim: int,
  feature_index: wp.vec3i,
  mat: wp.mat33,
  dir: wp.vec3,
  # Out:
  normal_out: wp.array(dtype=wp.vec3),
  index_out: wp.array(dtype=int),
) -> int:
  v1 = feature_index[0]
  v2 = feature_index[1]
  v3 = feature_index[2]

  if feature_dim == 3:
    c = 0
    x = float((v1 & 1) and (v2 & 1) and (v3 & 1)) - float(not (v1 & 1) and not (v2 & 1) and not (v3 & 1))
    y = float((v1 & 2) and (v2 & 2) and (v3 & 2)) - float(not (v1 & 2) and not (v2 & 2) and not (v3 & 2))
    z = float((v1 & 4) and (v2 & 4) and (v3 & 4)) - float(not (v1 & 4) and not (v2 & 4) and not (v3 & 4))
    normal_out[0] = mat @ wp.vec3(x, y, z)
    sgn = x + y + z
    if x != 0.0:
      index_out[c] = 0
      c += 1
    if y != 0.0:
      index_out[c] = 2
      c += 1
    if z != 0.0:
      index_out[c] = 4
      c += 1
    if sgn == -1.0:
      index_out[0] = index_out[0] + 1
    if c == 1:
      return 1
    return _box_normals2(mat, dir, normal_out, index_out)
  if feature_dim == 2:
    c = 0
    x = float((v1 & 1) and (v2 & 1)) - float(not (v1 & 1) and not (v2 & 1))
    y = float((v1 & 2) and (v2 & 2)) - float(not (v1 & 2) and not (v2 & 2))
    z = float((v1 & 4) and (v2 & 4)) - float(not (v1 & 4) and not (v2 & 4))
    if x != 0.0:
      normal_out[c] = mat @ wp.vec3(float(x), 0.0, 0.0)
      index_out[c] = wp.where(x > 0.0, 0, 1)
      c += 1
    if y != 0.0:
      normal_out[c] = mat @ wp.vec3(0.0, y, 0.0)
      index_out[c] = wp.where(y > 0.0, 2, 3)
      c += 1
    if z != 0.0:
      normal_out[c] = mat @ wp.vec3(0.0, 0.0, z)
      index_out[c] = wp.where(z > 0.0, 4, 5)
      c += 1
    if c == 2:
      return 2
    return _box_normals2(mat, dir, normal_out, index_out)

  if feature_dim == 1:
    x = wp.where(v1 & 1, 1.0, -1.0)
    y = wp.where(v1 & 2, 1.0, -1.0)
    z = wp.where(v1 & 4, 1.0, -1.0)
    normal_out[0] = mat @ wp.vec3(x, 0.0, 0.0)
    normal_out[1] = mat @ wp.vec3(0.0, y, 0.0)
    normal_out[2] = mat @ wp.vec3(0.0, 0.0, z)
    index_out[0] = wp.where(x > 0.0, 0, 1)
    index_out[1] = wp.where(y > 0.0, 2, 3)
    index_out[2] = wp.where(z > 0.0, 4, 5)
    return 3
  return 0


# compute possible edge normals for box for edge collisions
@wp.func
def _box_edge_normals(
  # In:
  dim: int,
  mat: wp.mat33,
  pos: wp.vec3,
  size: wp.vec3,
  v1: wp.vec3,
  v2: wp.vec3,
  v1i: int,
  # Out:
  normal_out: wp.array(dtype=wp.vec3),
  endvert_out: wp.array(dtype=wp.vec3),
) -> int:
  if dim == 2:
    endvert_out[0] = v2
    normal_out[0] = wp.normalize(v2 - v1)
    return 1

  # return 3 adjacent vertices
  if dim == 1:
    x = wp.where(v1i & 1, size[0], -size[0])
    y = wp.where(v1i & 2, size[1], -size[1])
    z = wp.where(v1i & 4, size[2], -size[2])

    endvert_out[0] = mat @ wp.vec3(-x, y, z) + pos
    normal_out[0] = wp.normalize(endvert_out[0] - v1)

    endvert_out[1] = mat @ wp.vec3(x, -y, z) + pos
    normal_out[1] = wp.normalize(endvert_out[1] - v1)

    endvert_out[2] = mat @ wp.vec3(x, y, -z) + pos
    normal_out[2] = wp.normalize(endvert_out[2] - v1)
    return 3
  return 0


# recover face of a box from its index
@wp.func
def _box_face(mat: wp.mat33, pos: wp.vec3, size: wp.vec3, idx: int, face_out: wp.array(dtype=wp.vec3)) -> int:
  # compute global coordinates of the box face and face normal
  if idx == 0:  # right
    face_out[0] = mat @ wp.vec3(size[0], size[1], size[2]) + pos
    face_out[1] = mat @ wp.vec3(size[0], size[1], -size[2]) + pos
    face_out[2] = mat @ wp.vec3(size[0], -size[1], -size[2]) + pos
    face_out[3] = mat @ wp.vec3(size[0], -size[1], size[2]) + pos
    return 4
  if idx == 1:  # left
    face_out[0] = mat @ wp.vec3(-size[0], size[1], -size[2]) + pos
    face_out[1] = mat @ wp.vec3(-size[0], size[1], size[2]) + pos
    face_out[2] = mat @ wp.vec3(-size[0], -size[1], size[2]) + pos
    face_out[3] = mat @ wp.vec3(-size[0], -size[1], -size[2]) + pos
    return 4
  if idx == 2:  # top
    face_out[0] = mat @ wp.vec3(-size[0], size[1], -size[2]) + pos
    face_out[1] = mat @ wp.vec3(size[0], size[1], -size[2]) + pos
    face_out[2] = mat @ wp.vec3(size[0], size[1], size[2]) + pos
    face_out[3] = mat @ wp.vec3(-size[0], size[1], size[2]) + pos
    return 4
  if idx == 3:  # bottom
    face_out[0] = mat @ wp.vec3(-size[0], -size[1], size[2]) + pos
    face_out[1] = mat @ wp.vec3(size[0], -size[1], size[2]) + pos
    face_out[2] = mat @ wp.vec3(size[0], -size[1], -size[2]) + pos
    face_out[3] = mat @ wp.vec3(-size[0], -size[1], -size[2]) + pos
    return 4
  if idx == 4:  # front
    face_out[0] = mat @ wp.vec3(-size[0], size[1], size[2]) + pos
    face_out[1] = mat @ wp.vec3(size[0], size[1], size[2]) + pos
    face_out[2] = mat @ wp.vec3(size[0], -size[1], size[2]) + pos
    face_out[3] = mat @ wp.vec3(-size[0], -size[1], size[2]) + pos
    return 4
  if idx == 5:  # back
    face_out[0] = mat @ wp.vec3(size[0], size[1], -size[2]) + pos
    face_out[1] = mat @ wp.vec3(-size[0], size[1], -size[2]) + pos
    face_out[2] = mat @ wp.vec3(-size[0], -size[1], -size[2]) + pos
    face_out[3] = mat @ wp.vec3(size[0], -size[1], -size[2]) + pos
    return 4
  return 0


# recover mesh polygon from its index, return number of edges
@wp.func
def _mesh_face(
  # In:
  mat: wp.mat33,
  pos: wp.vec3,
  vertadr: int,
  polyadr: int,
  vert: wp.array(dtype=wp.vec3),
  polyvertadr: wp.array(dtype=int),
  polyvertnum: wp.array(dtype=int),
  polyvert: wp.array(dtype=int),
  idx: int,
  # Out:
  face_out: wp.array(dtype=wp.vec3),
) -> int:
  adr = polyvertadr[polyadr + idx]
  j = int(0)
  nvert = polyvertnum[polyadr + idx]
  for i in range(nvert - 1, -1, -1):
    v = vert[vertadr + polyvert[adr + i]]
    face_out[j] = mat @ v + pos
    j += 1
  return nvert


@wp.func
def _plane_normal(v1: wp.vec3, v2: wp.vec3, n: wp.vec3) -> Tuple[float, wp.vec3]:
  v3 = v1 + n
  res = wp.cross(v2 - v1, v3 - v1)
  return wp.dot(res, v1), res


@wp.func
def _halfspace(a: wp.vec3, n: wp.vec3, p: wp.vec3) -> bool:
  return wp.dot(p - a, n) > -1e-10


@wp.func
def _plane_intersect(pn: wp.vec3, pd: float, a: wp.vec3, b: wp.vec3) -> Tuple[float, wp.vec3]:
  res = wp.vec3()
  ab = b - a
  temp = wp.dot(pn, ab)
  if temp == 0.0:
    return FLOAT_MAX, res  # parallel; no intersection
  t = (pd - wp.dot(pn, a)) / temp
  if t >= 0.0 and t <= 1.0:
    res[0] = a[0] + t * ab[0]
    res[1] = a[1] + t * ab[1]
    res[2] = a[2] + t * ab[2]
  return t, res


# clip a polygon against another polygon
@wp.func
def _polygon_clip(
  # In:
  plane_normal: wp.array(dtype=wp.vec3),
  plane_dist: wp.array(dtype=float),
  face1: wp.array(dtype=wp.vec3),
  nface1: int,
  face2: wp.array(dtype=wp.vec3),
  nface2: int,
  n: wp.vec3,
  dir: wp.vec3,
  # Out:
  polygon_out: wp.array(dtype=wp.vec3),
  clipped_out: wp.array(dtype=wp.vec3),
) -> Tuple[int, mat43, mat43]:
  witness1 = mat43()
  witness2 = mat43()

  # clipping face needs to be at least a triangle
  if nface1 < 3:
    return 0, witness1, witness2

  # compute plane normal and distance to plane for each vertex
  pn = plane_normal
  pd = plane_dist
  for i in range(nface1 - 1):
    pdi, pni = _plane_normal(face1[i], face1[i + 1], n)
    pd[i] = pdi
    pn[i] = pni
  pdi, pni = _plane_normal(face1[nface1 - 1], face1[0], n)
  pd[nface1 - 1] = pdi
  pn[nface1 - 1] = pni

  # reserve 2 * max_sides as max sides for a clipped polygon
  npolygon = nface2
  nclipped = int(0)

  for i in range(nface2):
    polygon_out[i] = face2[i]

  # clip the polygon by one edge e at a time
  for e in range(nface1):
    for i in range(npolygon):
      # get edge PQ of the polygon
      P = polygon_out[i]
      Q = wp.where(i < npolygon - 1, polygon_out[i + 1], polygon_out[0])

      # determine if P and Q are in the halfspace of the clipping edge
      inside1 = _halfspace(face1[e], pn[e], P)
      inside2 = _halfspace(face1[e], pn[e], Q)

      # PQ entirely outside the clipping edge, skip
      if not inside1 and not inside2:
        continue

      # edge PQ is inside the clipping edge, add Q
      if inside1 and inside2:
        clipped_out[nclipped] = Q
        nclipped += 1
        continue

      # add new vertex to clipped polygon where PQ intersects the clipping edge
      t, res = _plane_intersect(pn[e], pd[e], P, Q)
      if t >= 0.0 and t <= 1.0:
        clipped_out[nclipped] = res
        nclipped += 1

      # add Q as PQ is now back inside the clipping edge
      if inside2:
        clipped_out[nclipped] = Q
        nclipped += 1

    # swap clipped and polygon
    tmp = polygon_out
    polygon_out = clipped_out
    clipped_out = tmp
    npolygon = nclipped
    nclipped = 0

  if npolygon < 1:
    return 0, witness1, witness2

  if npolygon > 4:
    quad = _polygon_quad(polygon_out, npolygon)
    for i in range(4):
      witness2[i] = polygon_out[quad[i]]
      witness1[i] = witness2[i] - dir
    return 4, witness1, witness2

  # no pruning needed
  for i in range(npolygon):
    witness2[i] = polygon_out[i]
    witness1[i] = witness2[i] - dir
  return npolygon, witness1, witness2


@wp.func
def _set_edge(
  vert1: wp.array(dtype=wp.vec3), vert2: wp.array(dtype=wp.vec3), start: int, end: int, face_out: wp.array(dtype=wp.vec3)
) -> int:
  face_out[0] = vert1[start]
  face_out[1] = vert2[end]
  return 2


# recover multiple contacts from EPA polytope
@wp.func
def multicontact(
  # In:
  polygon: wp.array(dtype=wp.vec3),
  clipped: wp.array(dtype=wp.vec3),
  plane_normal: wp.array(dtype=wp.vec3),
  plane_dist: wp.array(dtype=float),
  idx1: wp.array(dtype=int),
  idx2: wp.array(dtype=int),
  n1: wp.array(dtype=wp.vec3),
  n2: wp.array(dtype=wp.vec3),
  endvert: wp.array(dtype=wp.vec3),
  face1: wp.array(dtype=wp.vec3),
  face2: wp.array(dtype=wp.vec3),
  epa_vert1: wp.array(dtype=wp.vec3),
  epa_vert2: wp.array(dtype=wp.vec3),
  epa_vert_index1: wp.array(dtype=int),
  epa_vert_index2: wp.array(dtype=int),
  face: wp.vec3i,
  x1: wp.vec3,
  x2: wp.vec3,
  geom1: Geom,
  geom2: Geom,
  geomtype1: int,
  geomtype2: int,
) -> Tuple[int, mat43, mat43]:
  witness1 = mat43()
  witness2 = mat43()
  witness1[0] = x1
  witness2[0] = x2

  if geomtype1 == GeomType.MESH:
    vert = geom1.vert
    polynormal = geom1.mesh_polynormal
    polyvertadr = geom1.mesh_polyvertadr
    polyvertnum = geom1.mesh_polyvertnum
    polyvert = geom1.mesh_polyvert
    polymapadr = geom1.mesh_polymapadr
    polymapnum = geom1.mesh_polymapnum
    polymap = geom1.mesh_polymap
  elif geomtype2 == GeomType.MESH:
    vert = geom2.vert
    polynormal = geom2.mesh_polynormal
    polyvertadr = geom2.mesh_polyvertadr
    polyvertnum = geom2.mesh_polyvertnum
    polyvert = geom2.mesh_polyvert
    polymapadr = geom2.mesh_polymapadr
    polymapnum = geom2.mesh_polymapnum
    polymap = geom2.mesh_polymap

  # get dimensions of features of geoms 1 and 2
  nface1, feature_index1, feature_vertex1 = _feature_dim(face, epa_vert_index1, epa_vert1)
  nface2, feature_index2, feature_vertex2 = _feature_dim(face, epa_vert_index2, epa_vert2)

  dir = x2 - x1
  dir_neg = -dir

  # get all possible face normals for each geom
  if geomtype1 == GeomType.BOX:
    nnorms1 = _box_normals(nface1, feature_index1, geom1.rot, dir_neg, n1, idx1)
  elif geomtype1 == GeomType.MESH:
    nnorms1 = _mesh_normals(
      nface1,
      feature_index1,
      geom1.rot,
      geom1.vertadr,
      geom1.mesh_polyadr,
      polynormal,
      polymapadr,
      polymapnum,
      polymap,
      n1,
      idx1,
    )
  if geomtype2 == GeomType.BOX:
    nnorms2 = _box_normals(nface2, feature_index2, geom2.rot, dir, n2, idx2)
  elif geomtype2 == GeomType.MESH:
    nnorms2 = _mesh_normals(
      nface2,
      feature_index2,
      geom2.rot,
      geom2.vertadr,
      geom2.mesh_polyadr,
      polynormal,
      polymapadr,
      polymapnum,
      polymap,
      n2,
      idx2,
    )

  # determine if any two face normals match
  is_edge_contact_geom1 = 0
  is_edge_contact_geom2 = 0
  nres, res = _aligned_faces(n1, nnorms1, n2, nnorms2)
  if not nres:
    # check if edge-face collision
    if nface1 < 3 and nface1 <= nface2:
      nnorms1 = 0
      if geomtype1 == GeomType.BOX:
        nnorms1 = _box_edge_normals(
          nface1, geom1.rot, geom1.pos, geom1.size, feature_vertex1[0], feature_vertex1[1], feature_index1[0], n1, endvert
        )
      elif geomtype1 == GeomType.MESH:
        nnorms1 = _mesh_edge_normals(
          nface1,
          geom1.rot,
          geom1.pos,
          geom1.vertadr,
          geom1.mesh_polyadr,
          geom1.vert,
          polyvertadr,
          polyvertnum,
          polyvert,
          polymapadr,
          polymapnum,
          polymap,
          feature_vertex1[0],
          feature_vertex1[1],
          feature_index1[0],
          n1,
          endvert,
        )
      nres, res = _aligned_face_edge(n1, nnorms1, n2, nnorms2)
      if not nres:
        return 1, witness1, witness2
      is_edge_contact_geom1 = 1

    # check if face-edge collision
    elif nface2 < 3:
      nnorms2 = 0
      if geomtype2 == GeomType.BOX:
        nnorms2 = _box_edge_normals(
          nface2, geom2.rot, geom2.pos, geom2.size, feature_vertex2[0], feature_vertex2[1], feature_index2[0], n2, endvert
        )
      elif geomtype2 == GeomType.MESH:
        nnorms2 = _mesh_edge_normals(
          nface2,
          geom2.rot,
          geom2.pos,
          geom2.vertadr,
          geom2.mesh_polyadr,
          geom2.vert,
          polyvertadr,
          polyvertnum,
          polyvert,
          polymapadr,
          polymapnum,
          polymap,
          feature_vertex2[0],
          feature_vertex2[1],
          feature_index2[0],
          n2,
          endvert,
        )
      nres, res = _aligned_face_edge(n2, nnorms2, n1, nnorms1)
      if not nres:
        return 1, witness1, witness2
      is_edge_contact_geom2 = 1
    else:
      # no multi-contact
      return 1, witness1, witness2

  i = res[0]
  j = res[1]

  # recover geom1 matching edge or face
  if is_edge_contact_geom1:
    nface1 = _set_edge(epa_vert1, endvert, face[0], i, face1)
  else:
    ind = wp.where(is_edge_contact_geom2, idx1[j], idx1[i])
    if geomtype1 == GeomType.BOX:
      nface1 = _box_face(geom1.rot, geom1.pos, geom1.size, ind, face1)
    elif geomtype1 == GeomType.MESH:
      nface1 = _mesh_face(
        geom1.rot,
        geom1.pos,
        geom1.vertadr,
        geom1.mesh_polyadr,
        vert,
        polyvertadr,
        polyvertnum,
        polyvert,
        ind,
        face1,
      )

  # recover geom2 matching edge or face
  if is_edge_contact_geom2:
    nface2 = _set_edge(epa_vert2, endvert, face[0], i, face2)
  else:
    if geomtype2 == GeomType.BOX:
      nface2 = _box_face(geom2.rot, geom2.pos, geom2.size, idx2[j], face2)
    elif geomtype2 == GeomType.MESH:
      nface2 = _mesh_face(
        geom2.rot,
        geom2.pos,
        geom2.vertadr,
        geom2.mesh_polyadr,
        vert,
        polyvertadr,
        polyvertnum,
        polyvert,
        idx2[j],
        face2,
      )

  # face1 is an edge; clip face1 against face2
  if is_edge_contact_geom1:
    approx_dir = wp.norm_l2(dir) * n2[j]
    return _polygon_clip(plane_normal, plane_dist, face2, nface2, face1, nface1, n2[j], approx_dir, polygon, clipped)

  # face2 is an edge; clip face2 against face1
  if is_edge_contact_geom2:
    approx_dir = -wp.norm_l2(dir) * n1[j]
    return _polygon_clip(plane_normal, plane_dist, face1, nface1, face2, nface2, n1[j], approx_dir, polygon, clipped)

  # face-face collision
  approx_dir = wp.norm_l2(dir) * n2[j]

  return _polygon_clip(plane_normal, plane_dist, face1, nface1, face2, nface2, n1[i], approx_dir, polygon, clipped)


@wp.func
def _inflate(dist: float, x1: wp.vec3, x2: wp.vec3, margin1: float, margin2: float) -> Tuple[float, wp.vec3, wp.vec3]:
  n = wp.normalize(x2 - x1)
  if margin1 > 0.0:
    x1 += margin1 * n

  if margin2 > 0.0:
    x2 -= margin2 * n
  dist -= margin1 + margin2
  return dist, x1, x2


@wp.func
def ccd(
  # In:
  tolerance: float,
  cutoff: float,
  gjk_iterations: int,
  epa_iterations: int,
  geom1: Geom,
  geom2: Geom,
  geomtype1: int,
  geomtype2: int,
  x_1: wp.vec3,
  x_2: wp.vec3,
  vert: wp.array(dtype=wp.vec3),
  vert1: wp.array(dtype=wp.vec3),
  vert2: wp.array(dtype=wp.vec3),
  vert_index1: wp.array(dtype=int),
  vert_index2: wp.array(dtype=int),
  face: wp.array(dtype=wp.vec3i),
  face_pr: wp.array(dtype=wp.vec3),
  face_norm2: wp.array(dtype=float),
  face_index: wp.array(dtype=int),
  face_map: wp.array(dtype=int),
  horizon: wp.array(dtype=int),
) -> Tuple[float, int, wp.vec3, wp.vec3, int]:
  """General convex collision detection via GJK/EPA."""
  full_margin1 = 0.0
  full_margin2 = 0.0
  size1 = 0.0
  size2 = 0.0

  # determine if the geoms being tested are discrete
  is_discrete = _discrete_geoms(geomtype1, geomtype2) and (geom1.margin == 0.0 and geom2.margin == 0.0)

  if geomtype1 == GeomType.SPHERE or geomtype1 == GeomType.CAPSULE:
    size1 = geom1.size[0]
    full_margin1 = size1 + 0.5 * geom1.margin
    geom1.margin = 0.0
    geom1.size = wp.vec3(0.0, geom1.size[1], geom1.size[2])

  # TODO(kbayes): support gjk margin trick with height fields
  if geomtype1 != GeomType.HFIELD and (geomtype2 == GeomType.SPHERE or geomtype2 == GeomType.CAPSULE):
    size2 = geom2.size[0]
    full_margin2 = size2 + 0.5 * geom2.margin
    geom2.margin = 0.0
    geom2.size = wp.vec3(0.0, geom2.size[1], geom2.size[2])

  # special handling for sphere and capsule (shrink to point and line respectively)
  if size1 + size2 > 0.0:
    cutoff += full_margin1 + full_margin2
    result = gjk(tolerance, gjk_iterations, geom1, geom2, x_1, x_2, geomtype1, geomtype2, cutoff, is_discrete)

    # shallow penetration, inflate contact
    if result.dist > tolerance:
      if result.dist == FLOAT_MAX:
        return result.dist, 1, result.x1, result.x2, -1
      dist, x1, x2 = _inflate(result.dist, result.x1, result.x2, full_margin1, full_margin2)
      return dist, 1, x1, x2, -1

    # deep penetration, reset initial conditions and rerun GJK + EPA
    geom1.margin = full_margin1 - size1
    geom1.size = wp.vec3(size1, geom1.size[1], geom1.size[2])
    geom2.margin = full_margin2 - size2
    geom2.size = wp.vec3(size2, geom2.size[1], geom2.size[2])
    cutoff -= full_margin1 + full_margin2

  result = gjk(tolerance, gjk_iterations, geom1, geom2, x_1, x_2, geomtype1, geomtype2, cutoff, is_discrete)

  # no penetration depth to recover
  if result.dist > tolerance or result.dim < 2:
    return result.dist, 1, result.x1, result.x2, -1

  pt = Polytope()
  pt.nface = 0
  pt.nmap = 0
  pt.nvert = 0
  pt.nhorizon = 0
  pt.vert = vert
  pt.vert1 = vert1
  pt.vert2 = vert2
  pt.vert_index1 = vert_index1
  pt.vert_index2 = vert_index2
  pt.face = face
  pt.face_pr = face_pr
  pt.face_norm2 = face_norm2
  pt.face_index = face_index
  pt.face_map = face_map
  pt.horizon = horizon

  if result.dim == 2:
    pt, new_result = _polytope2(
      pt,
      result.dist,
      result.simplex,
      result.simplex1,
      result.simplex2,
      result.simplex_index1,
      result.simplex_index2,
      geom1,
      geom2,
      geomtype1,
      geomtype2,
    )
    if pt.status == -1:
      result.simplex = new_result.simplex
      result.simplex1 = new_result.simplex1
      result.simplex2 = new_result.simplex2
      result.simplex_index1 = new_result.simplex_index1
      result.simplex_index2 = new_result.simplex_index2
      result.dim = 3
  elif result.dim == 4:
    pt, new_result = _polytope4(
      pt,
      result.dist,
      result.simplex,
      result.simplex1,
      result.simplex2,
      result.simplex_index1,
      result.simplex_index2,
      geom1,
      geom2,
      geomtype1,
      geomtype2,
    )
    if pt.status == -1:
      result.simplex = new_result.simplex
      result.simplex1 = new_result.simplex1
      result.simplex2 = new_result.simplex2
      result.simplex_index1 = new_result.simplex_index1
      result.simplex_index2 = new_result.simplex_index2
      result.dim = 3

  # polytope2 and polytope4 may need to fallback here
  if result.dim == 3:
    pt = _polytope3(
      pt,
      result.dist,
      result.simplex,
      result.simplex1,
      result.simplex2,
      result.simplex_index1,
      result.simplex_index2,
      geom1,
      geom2,
      geomtype1,
      geomtype2,
    )

  # origin on boundary (objects are not considered penetrating)
  if pt.status:
    return result.dist, 1, result.x1, result.x2, -1

  dist, x1, x2, idx = _epa(tolerance, gjk_iterations, epa_iterations, pt, geom1, geom2, geomtype1, geomtype2, is_discrete)
  if idx == -1:
    return FLOAT_MAX, 0, wp.vec3(), wp.vec3(), -1
  return dist, 1, x1, x2, idx
