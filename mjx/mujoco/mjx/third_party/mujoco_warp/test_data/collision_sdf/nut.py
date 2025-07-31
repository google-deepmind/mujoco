import warp as wp


@wp.func
def Fract(x: float) -> float:
  return x - wp.floor(x)


@wp.func
def Subtraction(a: float, b: float) -> float:
  return wp.max(a, -b)


@wp.func
def Union(a: float, b: float) -> float:
  return wp.min(a, b)


@wp.func
def Intersection(a: float, b: float) -> float:
  return wp.max(a, b)


@wp.func
def nut(p: wp.vec3, attr: wp.vec3) -> float:
  screw = 12.0
  radius2 = wp.sqrt(p[0] * p[0] + p[1] * p[1]) - attr[0]
  sqrt12 = wp.sqrt(2.0) / 2.0
  azimuth = wp.atan2(p[1], p[0])
  triangle = wp.abs(Fract(p[2] * screw - azimuth / (wp.pi * 2.0)) - 0.5)
  thread2 = (radius2 - triangle / screw) * sqrt12
  cone2 = (p[2] - radius2) * sqrt12
  hole = Subtraction(thread2, cone2 + 0.5 * sqrt12)
  hole = Union(hole, -cone2 - 0.05 * sqrt12)
  k = 6.0 / wp.pi / 2.0
  angle = -wp.floor((wp.atan2(p[1], p[0])) * k + 0.5) / k
  s0 = wp.sin(angle)
  s1 = wp.sin(angle + wp.pi * 0.5)
  res0 = s1 * p[0] - s0 * p[1]
  res1 = s0 * p[0] + s1 * p[1]
  point3D0 = res0
  point3D2 = p[2]
  head = point3D0 - 0.5
  head = Intersection(head, wp.abs(point3D2 + 0.25) - 0.25)
  head = Intersection(head, (point3D2 + radius2 - 0.22) * sqrt12)
  return Subtraction(head, hole)


@wp.func
def nut_sdf_grad(p: wp.vec3, attr: wp.vec3) -> wp.vec3:
  grad = wp.vec3()
  eps = 1e-6
  f_original = nut(p, attr)
  x_plus = wp.vec3(p[0] + eps, p[1], p[2])
  f_plus = nut(x_plus, attr)
  grad[0] = (f_plus - f_original) / eps

  x_plus = wp.vec3(p[0], p[1] + eps, p[2])
  f_plus = nut(x_plus, attr)
  grad[1] = (f_plus - f_original) / eps

  x_plus = wp.vec3(p[0], p[1], p[2] + eps)
  f_plus = nut(x_plus, attr)
  grad[2] = (f_plus - f_original) / eps
  return grad
