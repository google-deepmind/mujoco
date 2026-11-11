# Copyright 2023 DeepMind Technologies Limited
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
"""Collision functions for shapes represented as signed distance functions (SDF).

A signed distance function at a given point in space is the shortest distance to
a surface. This enables to define a geometry implicitly and exactly.

See https://iquilezles.org/articles/distfunctions/ for a list of analytic SDFs.
"""

import functools
from typing import Callable
from typing import Tuple

import jax
from jax import numpy as jp
from mujoco.mjx._src import math
# pylint: disable=g-importing-member
from mujoco.mjx._src.collision_types import Collision
from mujoco.mjx._src.collision_types import GeomInfo
from mujoco.mjx._src.dataclasses import PyTreeNode
from mujoco.mjx._src.types import Data
from mujoco.mjx._src.types import DataJAX
from mujoco.mjx._src.types import Model
from mujoco.mjx._src.types import ModelJAX
# pylint: enable=g-importing-member

# the SDF function takes position in, and returns a distance or objective
SDFFn = Callable[[jax.Array], jax.Array]


def collider(ncon: int):
  """Wraps collision functions for use by collision_driver."""

  def wrapper(func):
    def collide(m: Model, d: Data, _, geom: jax.Array) -> Collision:
      g1, g2 = geom.T
      info1 = GeomInfo(d.geom_xpos[g1], d.geom_xmat[g1], m.geom_size[g1])
      info2 = GeomInfo(d.geom_xpos[g2], d.geom_xmat[g2], m.geom_size[g2])
      dist, pos, frame = jax.vmap(func)(info1, info2)
      if ncon > 1:
        return jax.tree_util.tree_map(jp.concatenate, (dist, pos, frame))
      return dist, pos, frame

    collide.ncon = ncon  # pyrefly: ignore[missing-attribute]
    return collide

  return wrapper


def _plane(pos: jax.Array, size: jax.Array) -> jax.Array:
  del size
  return pos[2]


def _sphere(pos: jax.Array, size: jax.Array):
  return math.norm(pos) - size[0]


def _capsule(pos: jax.Array, size: jax.Array):
  pa = -size[1] * jp.array([0, 0, 1])
  pb = size[1] * jp.array([0, 0, 1])
  ab = pb - pa
  ap = pos - pa
  denom = ab.dot(ab)
  denom = jp.where(jp.abs(denom) < 1e-12, 1e-12 * math.sign(denom), denom)
  t = ab.dot(ap) / denom
  t = jp.clip(t, 0, 1)
  c = pa + t * ab
  return math.norm(pos - c) - size[0]


def _ellipsoid(pos: jax.Array, size: jax.Array) -> jax.Array:
  k0 = math.norm(pos / size)
  k1 = math.norm(pos / (size * size))
  return k0 * (k0 - 1.0) / (k1 + (k1 == 0.0) * 1e-12)


def _ellipsoid_closest(
    pos: jax.Array, size: jax.Array, niter: int = 14
) -> Tuple[jax.Array, jax.Array]:
  """Returns the closest point on an ellipsoid surface and its outward normal.

  The closest point to pos is x = size**2 * pos / (t + size**2), where t is the
  root of |p(t)| = 1, p(t) = size * pos / (t + size**2), with
  t > -min(size)**2 (D. Eberly, "Distance from a Point to an Ellipse, an
  Ellipsoid, or a Hyperellipsoid", 2013). 1 / |p(t)| is concave and increasing
  there, so Newton's method on 1 / |p(t)| = 1 started left of the root
  increases monotonically to it, as in the trust-region secular equation.
  Points outside the ellipsoid converge in about 5 iterations, points inside
  it in about 8, and points inside it near the plane of its smallest axis in
  up to about 14.

  Args:
    pos: query point in the ellipsoid frame
    size: ellipsoid semi-axes
    niter: number of Newton iterations

  Returns:
    closest point on the surface and the outward normal there, in the
    ellipsoid frame
  """
  # a zero coordinate on the smallest axis can put the root outside that
  # interval (Eberly's degenerate case), so move every coordinate off zero
  eps = 1e-10 * jp.max(size)
  pos = jp.where(jp.abs(pos) < eps, jp.where(pos < 0, -eps, eps), pos)
  size2 = size * size
  # solve for u = t + min(size)**2 > 0, which keeps every denominator
  # u + gap = t + size**2 a sum of nonnegative terms; gap is factored so that
  # it is exactly zero on the smallest axis, where u can be tiny
  smin = jp.min(size)
  gap = (size - smin) * (size + smin)
  # the root of each axis alone is a lower bound, and so is t = 0 outside
  u = jp.max(size * jp.abs(pos) - gap)
  outside = jp.sum(jp.square(pos / size)) > 1
  u = jp.where(outside, jp.maximum(u, smin * smin), u)
  for _ in range(niter):
    r = 1 / (u + gap)
    p2 = jp.square(size * pos * r)
    q = jp.sum(p2)
    u += q * (jp.sqrt(q) - 1) / jp.sum(p2 * r)
  x = size2 * pos / (u + gap)
  return x, math.normalize(x / size2)


@jax.custom_jvp
def _cylinder(pos: jax.Array, size: jax.Array) -> jax.Array:
  a0 = jp.sqrt(pos[0] * pos[0] + pos[1] * pos[1]) - size[0]
  a1 = jp.abs(pos[2]) - size[1]
  b0 = jp.maximum(a0, 0)
  b1 = jp.maximum(a1, 0)
  return jp.minimum(jp.maximum(a0, a1), 0) + jp.sqrt(b0 * b0 + b1 * b1)


def _cylinder_grad(x: jax.Array, size: jax.Array) -> jax.Array:
  """Gradient of the cylinder SDF wrt query point and singularities removed."""
  c = jp.sqrt(x[0] * x[0] + x[1] * x[1])
  e = jp.abs(x[2])
  a = jp.array([c - size[0], e - size[1]])
  b = jp.array([jp.maximum(a[0], 0), jp.maximum(a[1], 0)])
  j = jp.argmax(a)
  bnorm = jp.sqrt(b[0] * b[0] + b[1] * b[1])
  bnorm += jp.allclose(bnorm, 0) * 1e-12
  grada = jp.array([
      x[0] / (c + jp.allclose(c, 0) * 1e-12),
      x[1] / (c + jp.allclose(c, 0) * 1e-12),
      x[2] / (e + jp.allclose(e, 0) * 1e-12),
  ])
  gradm = jp.array([[grada[0], grada[1], 0], [0, 0, grada[2]]])
  gradb = grada * b[jp.array([0, 0, 1])] / bnorm
  return jp.where(a[j] < 0, gradm[j], gradb)


@_cylinder.defjvp
def cylinder_jvp(primals, tangents):
  x, y = primals
  x_dot, _ = tangents
  primal_out = _cylinder(x, y)
  tangent_out = jp.dot(_cylinder_grad(x, y), x_dot)
  return primal_out, tangent_out


def _from_to(
    f: SDFFn,
    from_pos: jax.Array,
    from_mat: jax.Array,
    to_pos: jax.Array,
    to_mat: jax.Array,
) -> SDFFn:
  relmat = math.matmul_unroll(to_mat.T, from_mat)
  relpos = to_mat.T @ (from_pos - to_pos)
  return lambda p: f(relmat @ p + relpos)


def _intersect(d1: SDFFn, d2: SDFFn) -> SDFFn:
  return lambda p: jp.maximum(d1(p), d2(p))


def _clearance(d1: SDFFn, d2: SDFFn) -> SDFFn:
  return lambda p: (d1(p) + d2(p) + jp.abs(_intersect(d1, d2)(p))).squeeze()


class GradientState(PyTreeNode):
  dist: jax.Array
  x: jax.Array


def _gradient_step(objective: SDFFn, state: GradientState) -> GradientState:
  """Performs a step of gradient descent."""
  # TODO: find better parameters
  amin = 1e-4  # minimum value for line search factor scaling the gradient
  amax = 2.0  # maximum value for line search factor scaling the gradient
  nlinesearch = 10  # line search points
  grad = jax.grad(objective)(state.x)
  alpha = jp.geomspace(amin, amax, nlinesearch).reshape(nlinesearch, -1)
  candidates = state.x - alpha * grad.reshape(-1, 3)
  values = jax.vmap(objective)(candidates)
  idx = jp.argmin(values)
  return state.replace(x=candidates[idx], dist=values[idx])


def _gradient_descent(
    objective: SDFFn,
    x: jax.Array,
    niter: int,
) -> Tuple[jax.Array, jax.Array]:
  """Performs gradient descent with backtracking line search."""
  state = GradientState(
      dist=1e10,
      x=x,
  )

  state, _ = jax.lax.scan(
      lambda s, _: (_gradient_step(objective, s), None), state, (), length=niter
  )
  return state.dist, state.x


def _optim(
    d1,
    d2,
    info1: GeomInfo,
    info2: GeomInfo,
    x0: jax.Array,
) -> Collision:
  """Optimizes the clearance function."""
  d1 = functools.partial(d1, size=info1.size)
  # evaluate d1 in d2 frame
  d1 = _from_to(d1, info2.pos, info2.mat, info1.pos, info1.mat)
  d2 = functools.partial(d2, size=info2.size)
  x0 = info2.mat.T @ (x0 - info2.pos)
  fn = _clearance(d1, d2)
  _, pos = _gradient_descent(fn, x0, 10)
  dist = d1(pos) + d2(pos)
  n = jax.grad(d1)(pos) - jax.grad(d2)(pos)
  pos = info2.mat @ pos + info2.pos  # d2 to global frame
  n = info2.mat @ n
  return dist, pos, math.make_frame(n)


def _sphere_ellipsoid(
    center: jax.Array, radius: jax.Array, e: GeomInfo
) -> Collision:
  """Returns the contact between a sphere, given in world frame, and e."""
  pos = e.mat.T @ (center - e.pos)
  x, n = _ellipsoid_closest(pos, e.size)
  # signed distance from the sphere center to the ellipsoid surface
  dist = jp.dot(pos - x, n) - radius
  pos = e.mat @ (x + 0.5 * dist * n) + e.pos
  return dist, pos, math.make_frame(-(e.mat @ n))


@collider(ncon=1)
def sphere_ellipsoid(s: GeomInfo, e: GeomInfo) -> Collision:
  """Calculates contact between a sphere and an ellipsoid."""
  return _sphere_ellipsoid(s.pos, s.size[0], e)


@collider(ncon=1)
def sphere_cylinder(s: GeomInfo, c: GeomInfo) -> Collision:
  """Calculates contact between a sphere and a cylinder."""
  # TODO: implement analytical version.
  x0 = 0.5 * (s.pos + c.pos)
  return _optim(_sphere, _cylinder, s, c, x0)


@collider(ncon=1)
def capsule_ellipsoid(c: GeomInfo, e: GeomInfo) -> Collision:
  """Calculates contact between a capsule and an ellipsoid.

  The distance is the minimum over the capsule axis of the signed distance to
  the ellipsoid, less the capsule radius. This is exact while the axis stays
  outside the ellipsoid. With the axis inside, it is the depth of the deepest
  axis point, which is less than the penetration depth when that point lies on
  the plane of the smallest semi-axis.
  """
  # the signed distance to a convex set is convex, so along the capsule axis
  # c.pos + s * axis, s in [-1, 1], it has a single minimum: bisect on the sign
  # of its derivative, the outward normal at the closest point dotted with axis
  axis = c.mat[:, 2] * c.size[1]
  axis_local = e.mat.T @ axis
  center_local = e.mat.T @ (c.pos - e.pos)

  def bisect(bracket, _):
    lo, hi = bracket
    mid = 0.5 * (lo + hi)
    # only the sign of the normal is used here, which needs fewer iterations
    pos = center_local + mid * axis_local
    _, n = _ellipsoid_closest(pos, e.size, niter=8)
    rising = jp.dot(n, axis_local) > 0
    return (jp.where(rising, lo, mid), jp.where(rising, mid, hi)), None

  one = jp.ones_like(c.size[0])
  (lo, hi), _ = jax.lax.scan(bisect, (-one, one), None, length=30)
  # a bracket that never left an end has its minimum at that end
  s = jp.where(lo == -one, lo, jp.where(hi == one, hi, 0.5 * (lo + hi)))
  # inside the ellipsoid, the distance has a kink where the axis crosses the
  # plane of the smallest semi-axis, and a minimum there moves with the
  # crossing: take s from the crossing, so that its derivative is kept
  k = jp.argmin(e.size)
  z_lo = center_local[k] + lo * axis_local[k]
  z_hi = center_local[k] + hi * axis_local[k]
  kink = z_lo * z_hi < 0
  s = jp.where(kink, -center_local[k] / jp.where(kink, axis_local[k], 1), s)
  return _sphere_ellipsoid(c.pos + s * axis, c.size[0], e)


@collider(ncon=2)
def capsule_cylinder(ca: GeomInfo, cy: GeomInfo) -> Collision:
  """Calculates contact between a capsule and a cylinder."""
  # TODO: improve robustness
  # Near sharp corners, the SDF might give the penetration depth with respect
  # to a surface that is not in collision. Possible solutions is to find the
  # contact points analytically or to change the SDF depending on the relative
  # pose of the bodies.
  mid = 0.5 * (ca.pos + cy.pos)
  vec = ca.mat[:, 2] * ca.size[1]
  x0 = jp.array([mid - vec, mid + vec])
  optim_ = functools.partial(_optim, _capsule, _cylinder, ca, cy)
  return jax.vmap(optim_)(x0)


@collider(ncon=1)
def ellipsoid_ellipsoid(e1: GeomInfo, e2: GeomInfo) -> Collision:
  """Calculates contact between two ellipsoids."""
  x0 = 0.5 * (e1.pos + e2.pos)
  return _optim(_ellipsoid, _ellipsoid, e1, e2, x0)


@collider(ncon=1)
def ellipsoid_cylinder(e: GeomInfo, c: GeomInfo) -> Collision:
  """Calculates contact between and ellipsoid and a cylinder."""
  x0 = 0.5 * (e.pos + c.pos)
  return _optim(_ellipsoid, _cylinder, e, c, x0)


@collider(ncon=4)
def cylinder_cylinder(c1: GeomInfo, c2: GeomInfo) -> Collision:
  """Calculates contact between a cylinder and a cylinder."""
  # TODO: improve robustness
  # Near sharp corners, the SDF might give the penetration depth with respect
  # to a surface that is not in collision. Possible solutions is to find the
  # contact points analytically or to change the SDF depending on the relative
  # pose of the bodies.
  basis = math.make_frame(c2.pos - c1.pos)
  mid = 0.5 * (c1.pos + c2.pos)
  r = jp.maximum(c1.size[0], c2.size[0])
  x0 = jp.array([
      mid + r * basis[1],
      mid + r * basis[2],
      mid - r * basis[1],
      mid - r * basis[2],
  ])
  optim_ = functools.partial(_optim, _cylinder, _cylinder, c1, c2)
  return jax.vmap(optim_)(x0)
