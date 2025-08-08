import warp as wp


@wp.func
def Subtraction(a: float, b: float) -> float:
  return wp.max(a, -b)


@wp.func
def Intersection(a: float, b: float) -> float:
  return wp.max(a, b)


@wp.func
def circle(rho: float, r: float) -> float:
  return rho - r


@wp.func
def smoothUnion(a: float, b: float, k: float) -> float:
  h = wp.min(wp.max(0.5 + 0.5 * (b - a) / k, 0.0), 1.0)
  return b * (1.0 - h) + a * h - k * h * (1.0 - h)


@wp.func
def smoothIntersection(a: float, b: float, k: float) -> float:
  return Subtraction(Intersection(a, b), smoothUnion(Subtraction(a, b), Subtraction(b, a), k))


@wp.func
def extrusion(p: wp.vec3, sdf_2d: float, h: float) -> float:
  w = wp.vec2()
  w[0] = sdf_2d
  w[1] = wp.abs(p[2]) - h
  w_abs = wp.vec2()
  w_abs[0] = wp.max(w[0], 0.0)
  w_abs[1] = wp.max(w[1], 0.0)
  return wp.min(wp.max(w[0], w[1]), 0.0) + wp.sqrt(w_abs[0] * w_abs[0] + w_abs[1] * w_abs[1])


@wp.func
def mod(x: float, y: float) -> float:
  return x - y * wp.floor(x / y)


@wp.func
def distance2D(p: wp.vec3, attributes: wp.vec3) -> float:
  # see https://www.shadertoy.com/view/3lG3WR
  D = 2.8
  N = 25.0
  psi = 3.096e-5 * N * N - 6.557e-3 * N + 0.551  # pressure angle
  alpha = 0.0
  innerdiameter = -1.0

  R = D / 2.0
  rho = wp.sqrt(p[0] * p[0] + p[1] * p[1])
  Pd = N / D  # Diametral Pitch: teeth per unit length of diameter
  P = wp.PI / Pd  # Circular Pitch
  a = 1.0 / Pd  # Addendum: radial length of a tooth from the pitch
  # circle to the tip of the tooth.

  Do = D + 2.0 * a  # Outside Diameter
  Ro = Do / 2.0

  h = 2.2 / Pd

  innerR = Ro - h - 0.14 * D
  if innerdiameter >= 0.0:
    innerR = innerdiameter / 2.0

  # Early exit
  if innerR - rho > 0.0:
    return innerR - rho

  # Early exit
  if Ro - rho < -0.2:
    return rho - Ro

  Db = D * wp.cos(psi)  # Base Diameter
  Rb = Db / 2.0

  fi = wp.atan2(p[1], p[0]) + alpha
  alphaStride = P / R

  invAlpha = wp.acos(Rb / R)
  invPhi = wp.tan(invAlpha) - invAlpha

  shift = alphaStride / 2.0 - 2.0 * invPhi

  fia = mod(fi + shift / 2.0, alphaStride) - shift / 2.0
  fib = mod(-fi - shift + shift / 2.0, alphaStride) - shift / 2.0

  dista = -1.0e6
  distb = -1.0e6

  if Rb < rho:
    acos_rbRho = wp.acos(Rb / rho)

    thetaa = fia + acos_rbRho
    thetab = fib + acos_rbRho

    ta = wp.sqrt(rho * rho - Rb * Rb)

    # https://math.stackexchange.com/questions/1266689/distance-from-a-point-to-the-involute-of-a-circle
    dista = ta - Rb * thetaa
    distb = ta - Rb * thetab

  gearOuter = circle(rho, Ro)
  gearLowBase = circle(rho, Ro - h)
  crownBase = circle(rho, innerR)
  cogs = Intersection(dista, distb)
  baseWalls = Intersection(fia - (alphaStride - shift), fib - (alphaStride - shift))

  cogs = Intersection(baseWalls, cogs)
  cogs = smoothIntersection(gearOuter, cogs, 0.0035 * D)
  cogs = smoothUnion(gearLowBase, cogs, Rb - Ro + h)
  cogs = Subtraction(cogs, crownBase)

  return cogs


@wp.func
def gear(p: wp.vec3, attr: wp.vec3) -> float:
  thickness = 0.2
  return extrusion(p, distance2D(p, attr), thickness / 2.0)


@wp.func
def gear_sdf_grad(p: wp.vec3, attr: wp.vec3) -> wp.vec3:
  grad = wp.vec3()
  eps = 1e-6
  f_original = gear(p, attr)
  x_plus = wp.vec3(p[0] + eps, p[1], p[2])
  f_plus = gear(x_plus, attr)
  grad[0] = (f_plus - f_original) / eps

  x_plus = wp.vec3(p[0], p[1] + eps, p[2])
  f_plus = gear(x_plus, attr)
  grad[1] = (f_plus - f_original) / eps

  x_plus = wp.vec3(p[0], p[1], p[2] + eps)
  f_plus = gear(x_plus, attr)
  grad[2] = (f_plus - f_original) / eps
  return grad
