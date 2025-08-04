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
def bolt(p: wp.vec3, attr: wp.vec3) -> float:
  screw = 12.0
  radius = wp.sqrt(p[0] * p[0] + p[1] * p[1]) - attr[0]
  sqrt12 = wp.sqrt(2.0) / 2.0

  azimuth = wp.atan2(p[1], p[0])
  triangle = wp.abs(Fract(p[2] * screw - azimuth / wp.pi / 2.0) - 0.5)
  thread = (radius - triangle / screw) * sqrt12

  bolt_val = Subtraction(thread, 0.5 - wp.abs(p[2] + 0.5))
  cone = (p[2] - radius) * sqrt12

  bolt_val = Subtraction(bolt_val, cone + 1.0 * sqrt12)

  point2D = wp.vec2(p[0], p[1])
  k = 6.0 / wp.pi / 2.0
  angle = -wp.floor((wp.atan2(point2D[1], point2D[0])) * k + 0.5) / k
  s = wp.vec2(wp.sin(angle), wp.sin(angle + wp.pi * 0.5))

  res = wp.vec2(s[1] * point2D[0] - s[0] * point2D[1], s[0] * point2D[0] + s[1] * point2D[1])
  point3D = wp.vec3(res[0], res[1], p[2])
  head = point3D[0] - 0.5

  head = Intersection(head, wp.abs(point3D[2] + 0.25) - 0.25)
  head = Intersection(head, (point3D[2] + radius - 0.22) * sqrt12)

  return Union(bolt_val, head)


@wp.func
def bolt_sdf_grad(p: wp.vec3, attr: wp.vec3) -> wp.vec3:
  grad = wp.vec3()
  eps = 1e-6
  f_original = bolt(p, attr)
  x_plus = wp.vec3(p[0] + eps, p[1], p[2])
  f_plus = bolt(x_plus, attr)
  grad[0] = (f_plus - f_original) / eps

  x_plus = wp.vec3(p[0], p[1] + eps, p[2])
  f_plus = bolt(x_plus, attr)
  grad[1] = (f_plus - f_original) / eps

  x_plus = wp.vec3(p[0], p[1], p[2] + eps)
  f_plus = bolt(x_plus, attr)
  grad[2] = (f_plus - f_original) / eps
  return grad
