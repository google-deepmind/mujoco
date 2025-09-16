import math

import warp as wp


@wp.func
def torus(p: wp.vec3, attr: wp.vec3) -> wp.float32:
  major_radius = attr[0]
  minor_radius = attr[1]

  q = math.sqrt(p[0] * p[0] + p[1] * p[1]) - major_radius
  sdf = math.sqrt(q * q + p[2] * p[2]) - minor_radius
  return sdf


@wp.func
def torus_sdf_grad(p: wp.vec3, attr: wp.vec3) -> wp.vec3:
  grad = wp.vec3()
  major_radius = attr[0]
  minor_val = attr[1]

  len_xy = math.sqrt(p[0] * p[0] + p[1] * p[1])
  q = len_xy - major_radius
  len_xy = wp.max(len_xy, 1e-8)
  grad_q_x = p[0] / len_xy
  grad_q_y = p[1] / len_xy
  len_qz = math.sqrt(q * q + p[2] * p[2])
  denom = wp.max(len_qz, wp.max(minor_val, 1e-8))

  grad[0] = q * grad_q_x / denom
  grad[1] = q * grad_q_y / denom
  grad[2] = p[2] / denom

  return grad
