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

from mujoco.mjx.third_party.mujoco_warp._src.collision_hfield import hfield_prism_vertex
from mujoco.mjx.third_party.mujoco_warp._src.collision_primitive import Geom
from mujoco.mjx.third_party.mujoco_warp._src.types import MJ_MINVAL
from mujoco.mjx.third_party.mujoco_warp._src.types import GeomType

# TODO(team): improve compile time to enable backward pass
wp.config.enable_backward = False

FLOAT_MIN = -1e30
FLOAT_MAX = 1e30
MJ_MINVAL2 = MJ_MINVAL * MJ_MINVAL

mat43 = wp.types.matrix(shape=(4, 3), dtype=float)


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


@wp.func
def _support(geom: Geom, geomtype: int, dir: wp.vec3):
  cached_index = -1
  vertex_index = -1
  local_dir = wp.transpose(geom.rot) @ dir
  if geomtype == int(GeomType.SPHERE.value):
    support_pt = geom.pos + geom.size[0] * dir
  elif geomtype == int(GeomType.BOX.value):
    tmp = wp.sign(local_dir)
    res = wp.cw_mul(tmp, geom.size)
    support_pt = geom.rot @ res + geom.pos
    vertex_index = 0
    if tmp[0] > 0:
      vertex_index += 1
    if tmp[1] > 0:
      vertex_index += 2
    if tmp[2] > 0:
      vertex_index += 4
  elif geomtype == int(GeomType.CAPSULE.value):
    res = local_dir * geom.size[0]
    # add cylinder contribution
    res[2] += wp.sign(local_dir[2]) * geom.size[1]
    support_pt = geom.rot @ res + geom.pos
  elif geomtype == int(GeomType.ELLIPSOID.value):
    res = wp.cw_mul(local_dir, geom.size)
    res = wp.normalize(res)
    # transform to ellipsoid
    res = wp.cw_mul(res, geom.size)
    support_pt = geom.rot @ res + geom.pos
  elif geomtype == int(GeomType.CYLINDER.value):
    res = wp.vec3(0.0, 0.0, 0.0)
    # set result in XY plane: support on circle
    d = wp.sqrt(local_dir[0] * local_dir[0] + local_dir[1] * local_dir[1])
    if d > MJ_MINVAL:
      scl = geom.size[0] / d
      res[0] = local_dir[0] * scl
      res[1] = local_dir[1] * scl
    # set result in Z direction
    res[2] = wp.sign(local_dir[2]) * geom.size[1]
    support_pt = geom.rot @ res + geom.pos
  elif geomtype == int(GeomType.MESH.value):
    max_dist = float(FLOAT_MIN)
    if geom.graphadr == -1 or geom.vertnum < 10:
      if geom.index > -1:
        cached_index = geom.index
        max_dist = wp.dot(geom.vert[geom.index], local_dir)
        support_pt = geom.vert[geom.index]
      # exhaustive search over all vertices
      for i in range(geom.vertnum):
        vert = geom.vert[geom.vertadr + i]
        dist = wp.dot(vert, local_dir)
        if dist > max_dist:
          max_dist = dist
          support_pt = vert
          cached_index = geom.vertadr + i
      vertex_index = cached_index - geom.vertadr
    else:
      numvert = geom.graph[geom.graphadr]
      vert_edgeadr = geom.graphadr + 2
      vert_globalid = geom.graphadr + 2 + numvert
      edge_localid = geom.graphadr + 2 + 2 * numvert
      # hillclimb until no change
      prev = int(-1)
      imax = int(0)
      if geom.index > -1:
        imax = geom.index
        cached_index = geom.index

      while True:
        prev = int(imax)
        i = int(geom.graph[vert_edgeadr + imax])
        while geom.graph[edge_localid + i] >= 0:
          subidx = geom.graph[edge_localid + i]
          idx = geom.graph[vert_globalid + subidx]
          dist = wp.dot(local_dir, geom.vert[geom.vertadr + idx])
          if dist > max_dist:
            max_dist = dist
            imax = int(subidx)
          i += int(1)
        if imax == prev:
          break
      cached_index = imax
      imax = geom.graph[vert_globalid + imax]
      vertex_index = imax
      support_pt = geom.vert[geom.vertadr + imax]

    support_pt = geom.rot @ support_pt + geom.pos
  elif geomtype == int(GeomType.HFIELD.value):
    max_dist = float(FLOAT_MIN)
    for i in range(6):
      vert = hfield_prism_vertex(geom.hfprism, i)
      dist = wp.dot(vert, local_dir)
      if dist > max_dist:
        max_dist = dist
        support_pt = vert
    support_pt = geom.rot @ support_pt + geom.pos

  return support_pt, cached_index, vertex_index


@wp.func
def _attach_face(pt: Polytope, idx: int, v1: int, v2: int, v3: int):
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
def _epa_support(pt: Polytope, idx: int, geom1: Geom, geom2: Geom, geom1_type: int, geom2_type: int, dir: wp.vec3):
  s1, index1, vertex_index1 = _support(geom1, geom1_type, dir)
  s2, index2, vertex_index2 = _support(geom2, geom2_type, -dir)

  pt.vert[idx] = s1 - s2
  pt.vert1[idx] = s1
  pt.vert2[idx] = s2
  pt.vert_index1[idx] = vertex_index1
  pt.vert_index2[idx] = vertex_index2
  return index1, index2


@wp.func
def _linear_combine(n: int, coefs: wp.vec4, mat: mat43):
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
def _almost_equal(v1: wp.vec3, v2: wp.vec3):
  return wp.abs(v1[0] - v2[0]) < MJ_MINVAL and wp.abs(v1[1] - v2[1]) < MJ_MINVAL and wp.abs(v1[2] - v2[2]) < MJ_MINVAL


@wp.func
def _subdistance(n: int, simplex: mat43):
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
def _det3(v1: wp.vec3, v2: wp.vec3, v3: wp.vec3):
  return wp.dot(v1, wp.cross(v2, v3))


@wp.func
def _same_sign(a: float, b: float):
  if a > 0 and b > 0:
    return 1
  if a < 0 and b < 0:
    return -1
  return 0


@wp.func
def _project_origin_line(v1: wp.vec3, v2: wp.vec3):
  diff = v2 - v1
  scl = -(wp.dot(v2, diff) / wp.dot(diff, diff))
  return v2 + scl * diff


@wp.func
def _project_origin_plane(v1: wp.vec3, v2: wp.vec3, v3: wp.vec3):
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
  if nv != 0 and nn > MJ_MINVAL:
    v = (nv / nn) * n
    return v, 0

  # n = (v2 - v1) x (v3 - v1)
  n = wp.cross(diff21, diff31)
  nv = wp.dot(n, v1)
  nn = wp.dot(n, n)
  if nn == 0:
    return z, 1
  if nv != 0 and nn > MJ_MINVAL:
    v = (nv / nn) * n
    return v, 0

  # n = (v1 - v3) x (v2 - v3)
  n = wp.cross(diff31, diff32)
  nv = wp.dot(n, v3)
  nn = wp.dot(n, n)
  v = (nv / nn) * n
  return v, 0


@wp.func
def _S3D(s1: wp.vec3, s2: wp.vec3, s3: wp.vec3, s4: wp.vec3):
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
def _S2D(s1: wp.vec3, s2: wp.vec3, s3: wp.vec3):
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
def _S1D(s1: wp.vec3, s2: wp.vec3):
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
def _gjk(
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
):
  """Find distance within a tolerance between two geoms."""
  cutoff2 = cutoff * cutoff
  simplex = mat43()
  simplex1 = mat43()
  simplex2 = mat43()
  simplex_index1 = wp.vec4i()
  simplex_index2 = wp.vec4i()
  n = int(0)
  coordinates = wp.vec4()  # barycentric coordinates
  epsilon = 0.5 * tolerance * tolerance

  # set initial guess
  x_k = x1_0 - x2_0

  for k in range(gjk_iterations):
    xnorm = wp.dot(x_k, x_k)
    # TODO(kbayes): determine new constant here
    if xnorm < 1e-12:
      break
    dir_neg = x_k / wp.sqrt(xnorm)

    # compute the kth support point
    s1_k, i1, vertex_index1 = _support(geom1, geomtype1, -dir_neg)
    s2_k, i2, vertex_index2 = _support(geom2, geomtype2, dir_neg)
    geom1.index = i1
    geom2.index = i2
    simplex1[n] = s1_k
    simplex2[n] = s2_k
    simplex_index1[n] = vertex_index1
    simplex_index2[n] = vertex_index2
    simplex[n] = s1_k - s2_k

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

  result = GJKResult()

  # compute the approximate witness points
  result.x1 = _linear_combine(n, coordinates, simplex1)
  result.x2 = _linear_combine(n, coordinates, simplex2)
  result.dist = wp.norm_l2(x_k)

  result.dim = n
  result.simplex1 = simplex1
  result.simplex2 = simplex2
  result.simplex_index1 = simplex_index1
  result.simplex_index2 = simplex_index2
  result.simplex = simplex
  return result


@wp.func
def _same_side(p0: wp.vec3, p1: wp.vec3, p2: wp.vec3, p3: wp.vec3):
  n = wp.cross(p1 - p0, p2 - p0)
  dot1 = wp.dot(n, p3 - p0)
  dot2 = wp.dot(n, -p0)
  if dot1 > 0 and dot2 > 0:
    return 1
  if dot1 < 0 and dot2 < 0:
    return 1
  return 0


@wp.func
def _test_tetra(p0: wp.vec3, p1: wp.vec3, p2: wp.vec3, p3: wp.vec3):
  return _same_side(p0, p1, p2, p3) and _same_side(p1, p2, p3, p0) and _same_side(p2, p3, p0, p1) and _same_side(p3, p0, p1, p2)


@wp.func
def _tri_affine_coord(v1: wp.vec3, v2: wp.vec3, v3: wp.vec3, p: wp.vec3):
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
def _tri_point_intersect(v1: wp.vec3, v2: wp.vec3, v3: wp.vec3, p: wp.vec3):
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
  return wp.norm_l2(pr - p) < MJ_MINVAL


@wp.func
def _replace_simplex3(pt: Polytope, v1: int, v2: int, v3: int):
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
def _rotmat(axis: wp.vec3):
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
def _ray_triangle(v1: wp.vec3, v2: wp.vec3, v3: wp.vec3, v4: wp.vec3, v5: wp.vec3):
  vol1 = _det3(v3 - v1, v4 - v1, v2 - v1)
  vol2 = _det3(v4 - v1, v5 - v1, v2 - v1)
  vol3 = _det3(v5 - v1, v3 - v1, v2 - v1)

  if vol1 >= 0 and vol2 >= 0 and vol3 >= 0:
    return 1
  if vol1 <= 0 and vol2 <= 0 and vol3 <= 0:
    return -1
  return 0


@wp.func
def _add_edge(pt: Polytope, e1: int, e2: int):
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
def _delete_face(pt: Polytope, face_id: int):
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
def _epa_witness(pt: Polytope, face_idx: int):
  # compute affine coordinates for witness points on plane defined by face
  v1 = pt.vert[pt.face[face_idx][0]]
  v2 = pt.vert[pt.face[face_idx][1]]
  v3 = pt.vert[pt.face[face_idx][2]]

  coordinates = _tri_affine_coord(v1, v2, v3, pt.face_pr[face_idx])
  l1 = coordinates[0]
  l2 = coordinates[1]
  l3 = coordinates[2]

  # face on geom 1
  v1 = pt.vert1[pt.face[face_idx][0]]
  v2 = pt.vert1[pt.face[face_idx][1]]
  v3 = pt.vert1[pt.face[face_idx][2]]
  x1 = wp.vec3()
  x1[0] = v1[0] * l1 + v2[0] * l2 + v3[0] * l3
  x1[1] = v1[1] * l1 + v2[1] * l2 + v3[1] * l3
  x1[2] = v1[2] * l1 + v2[2] * l2 + v3[2] * l3

  # face on geom 2
  v1 = pt.vert2[pt.face[face_idx][0]]
  v2 = pt.vert2[pt.face[face_idx][1]]
  v3 = pt.vert2[pt.face[face_idx][2]]
  x2 = wp.vec3()
  x2[0] = v1[0] * l1 + v2[0] * l2 + v3[0] * l3
  x2[1] = v1[1] * l1 + v2[1] * l2 + v3[1] * l3
  x2[2] = v1[2] * l1 + v2[2] * l2 + v3[2] * l3

  return x1, x2


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
):
  """Create polytope for EPA given a 1-simplex from GJK"""
  diff = simplex[1] - simplex[0]

  # find component with smallest magnitude (so cross product is largest)
  value = FLOAT_MAX
  index = 0
  for i in range(3):
    if wp.abs(diff[i]) < value:
      value = wp.abs(diff[i])
      index = i

  # cross product with best coordinate axis
  e = wp.vec(0.0, 0.0, 0.0)
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
  if _attach_face(pt, 0, 0, 2, 3) < MJ_MINVAL:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 2, 3)

  if _attach_face(pt, 1, 0, 4, 2) < MJ_MINVAL2:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 4, 2)

  if _attach_face(pt, 2, 0, 3, 4) < MJ_MINVAL2:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 3, 4)

  if _attach_face(pt, 3, 1, 3, 2) < MJ_MINVAL2:
    pt.status = -1
    return pt, _replace_simplex3(pt, 1, 3, 2)

  if _attach_face(pt, 4, 1, 2, 4) < MJ_MINVAL2:
    pt.status = -1
    return pt, _replace_simplex3(pt, 1, 2, 4)

  if _attach_face(pt, 5, 1, 4, 3) < MJ_MINVAL2:
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
):
  """Create polytope for EPA given a 2-simplex from GJK"""
  # get normals in both directions
  n = wp.cross(simplex[1] - simplex[0], simplex[2] - simplex[0])
  if wp.norm_l2(n) < MJ_MINVAL:
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
  if _attach_face(pt, 0, 4, 0, 1) < MJ_MINVAL2:
    pt.status = 6
    return pt
  if _attach_face(pt, 1, 4, 2, 0) < MJ_MINVAL2:
    pt.status = 7
    return pt
  if _attach_face(pt, 2, 4, 1, 2) < MJ_MINVAL2:
    pt.status = 8
    return pt
  if _attach_face(pt, 3, 3, 1, 0) < MJ_MINVAL2:
    pt.status = 9
    return pt
  if _attach_face(pt, 4, 3, 0, 2) < MJ_MINVAL2:
    pt.status = 10
    return pt
  if _attach_face(pt, 5, 3, 2, 1) < MJ_MINVAL2:
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
):
  """Create polytope for EPA given a 3-simplex from GJK"""
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
  if _attach_face(pt, 0, 0, 1, 2) < MJ_MINVAL2:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 1, 2)

  if _attach_face(pt, 1, 0, 3, 1) < MJ_MINVAL2:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 3, 1)

  if _attach_face(pt, 2, 0, 2, 3) < MJ_MINVAL2:
    pt.status = -1
    return pt, _replace_simplex3(pt, 0, 2, 3)

  if _attach_face(pt, 3, 3, 2, 1) < MJ_MINVAL2:
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
def _epa(tolerance2: float, epa_iterations: int, pt: Polytope, geom1: Geom, geom2: Geom, geomtype1: int, geomtype2: int):
  """Recover penetration data from two geoms in contact given an initial polytope."""
  upper = FLOAT_MAX
  upper2 = FLOAT_MAX
  idx = int(-1)
  pidx = int(-1)

  for k in range(epa_iterations):
    pidx = int(idx)
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

    if upper - lower < tolerance2:
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

      if wp.dot(pt.face_pr[i], pt.vert[wi]) - pt.face_norm2[i] > MJ_MINVAL:
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

  # return from valid face
  if idx > -1:
    x1, x2 = _epa_witness(pt, idx)
    return -wp.sqrt(pt.face_norm2[idx]), x1, x2
  return 0.0, wp.vec3(), wp.vec3()


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
):
  """General convex collision detection via GJK/EPA."""
  result = _gjk(tolerance, gjk_iterations, geom1, geom2, x_1, x_2, geomtype1, geomtype2, cutoff)

  # no penetration depth to recover
  if result.dist > tolerance or result.dim < 2:
    return result.dist, result.x1, result.x2

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
    return result.dist, result.x1, result.x2

  return _epa(tolerance * tolerance, epa_iterations, pt, geom1, geom2, geomtype1, geomtype2)
