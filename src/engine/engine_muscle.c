// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "engine/engine_muscle.h"

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_support.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"

//------------------------------ Millard2012 curves ------------------------------------------------

// The four Millard2012 curves are OpenSim's SmoothSegmentedFunctionFactory curves: a chain of
// quintic Bezier corners through a few knots (x, y, dy/dx), extrapolated linearly outside the
// first and last knot. A curve is stored as its knots; the Bezier control points of the one
// section a query falls in are built on the fly, and x(u) = x is inverted by Newton, as OpenSim
// does. No table is sampled, so the curves are exact and depend on the model parameters alone.

#define mjMUSCLE_MAXKNOT 6

// Bezier parameter at which the inversion of x(u) stops
#ifdef mjUSESINGLE
  #define mjMUSCLE_UTOL 1e-7f
#else
  #define mjMUSCLE_UTOL 1e-15
#endif

typedef struct mjMuscleCurve_ {
  int n;                              // number of knots
  mjtNum x[mjMUSCLE_MAXKNOT];         // knot abscissae, increasing
  mjtNum y[mjMUSCLE_MAXKNOT];         // knot values
  mjtNum dydx[mjMUSCLE_MAXKNOT];      // knot slopes; the end slopes extrapolate
  mjtNum curv[mjMUSCLE_MAXKNOT-1];    // curviness of each section, scaled to (0, 1]
} mjMuscleCurve;


// OpenSim's scaleCurviness: user curviness in [0, 1] maps to [0.1, 0.9]
static inline mjtNum scaleCurviness(mjtNum c) {
  return 0.1 + 0.8*c;
}


// append a knot
static inline void addKnot(mjMuscleCurve* c, mjtNum x, mjtNum y, mjtNum dydx) {
  c->x[c->n] = x;
  c->y[c->n] = y;
  c->dydx[c->n] = dydx;
  c->n++;
}


// active force-length curve, OpenSim createFiberActiveForceLengthCurve with x2 = 1, curviness 1
static void activeCurve(mjMuscleCurve* c, const mjtNum* gainprm, mjtNum ylow) {
  mjtNum x0 = gainprm[mjMILLARD_AFL_MIN], x1 = gainprm[mjMILLARD_AFL_TRANS], x2 = 1;
  mjtNum x3 = gainprm[mjMILLARD_AFL_MAX], dydx = gainprm[mjMILLARD_AFL_SLOPE];

  mjtNum xdelta = 0.05*x2;                // half-width of the plateau shoulder
  mjtNum xs = x2 - xdelta;
  mjtNum y1 = 1 - dydx*(xs - x1);
  mjtNum y23 = 0.5, x23 = (x2 + xdelta) + 0.5*(x3 - (x2 + xdelta));

  c->n = 0;
  addKnot(c, x0, ylow, 0);
  addKnot(c, x0 + 0.5*(x1 - x0), 0.5*y1, 1.25*y1/(x1 - x0));
  addKnot(c, x1 + 0.5*(xs - x1), y1 + 0.5*(1 - y1), dydx);
  addKnot(c, x2, 1, 0);
  addKnot(c, x23, y23, -1/((x3 - xdelta) - (x2 + xdelta)));
  addKnot(c, x3, ylow, 0);
  for (int i=0; i < 5; i++) {
    c->curv[i] = scaleCurviness(1);
  }
}


// passive force-length curve, OpenSim createFiberForceLengthCurve
static void passiveCurve(mjMuscleCurve* c, const mjtNum* biasprm) {
  mjtNum xzero = 1 + biasprm[mjMILLARD_PFL_STRAIN0];
  mjtNum xiso = 1 + biasprm[mjMILLARD_PFL_STRAIN1];
  mjtNum klow = biasprm[mjMILLARD_PFL_KLOW], kiso = biasprm[mjMILLARD_PFL_KISO];

  mjtNum xlow = xzero + mju_min(0.1/kiso, 0.1*(xiso - xzero));
  mjtNum xfoot = xzero + 0.5*(xlow - xzero);

  c->n = 0;
  addKnot(c, xzero, 0, 0);
  addKnot(c, xlow, klow*(xlow - xfoot), klow);
  addKnot(c, xiso, 1, kiso);
  c->curv[0] = c->curv[1] = scaleCurviness(biasprm[mjMILLARD_PFL_CURV]);
}


// tendon force-length curve, OpenSim createTendonForceLengthCurve; linear above the toe
static void tendonCurve(mjMuscleCurve* c, const mjtNum* biasprm) {
  mjtNum kiso = biasprm[mjMILLARD_TFL_KISO], ftoe = biasprm[mjMILLARD_TFL_FTOE];
  mjtNum xiso = 1 + biasprm[mjMILLARD_TFL_STRAIN1];

  mjtNum xtoe = (ftoe - 1)/kiso + xiso;
  mjtNum xfoot = 1 + (xtoe - 1)/10;
  mjtNum xmid = (0.5*ftoe - 1)/kiso + xiso;
  mjtNum dydxmid = 0.5*ftoe/(xmid - xfoot);
  mjtNum xctrl = xfoot + 0.5*(xmid - xfoot);

  c->n = 0;
  addKnot(c, 1, 0, 0);
  addKnot(c, xctrl, dydxmid*(xctrl - xfoot), dydxmid);
  addKnot(c, xtoe, ftoe, kiso);
  c->curv[0] = c->curv[1] = scaleCurviness(biasprm[mjMILLARD_TFL_CURV]);
}


// force-velocity curve, OpenSim createFiberForceVelocityCurve; slopes at vmax given explicitly
static void velocityCurve(mjMuscleCurve* c, const mjtNum* dynprm, mjtNum slopecon,
                          mjtNum slopeecc) {
  mjtNum fmax = dynprm[mjMILLARD_FV_FMAXECC];
  mjtNum nearcon = dynprm[mjMILLARD_FV_CONSLOPE], nearecc = dynprm[mjMILLARD_FV_ECCSLOPE];

  c->n = 0;
  addKnot(c, -1, 0, slopecon);
  addKnot(c, -0.9, 0.05*(nearcon + slopecon), nearcon);
  addKnot(c, 0, 1, dynprm[mjMILLARD_FV_ISOSLOPE]);
  addKnot(c, 0.9, fmax - 0.05*(nearecc + slopeecc), nearecc);
  addKnot(c, 1, fmax, slopeecc);
  c->curv[0] = c->curv[1] = scaleCurviness(dynprm[mjMILLARD_FV_CONCURV]);
  c->curv[2] = c->curv[3] = scaleCurviness(dynprm[mjMILLARD_FV_ECCCURV]);
}


// power basis of the quintic Bezier with control points (p0, p1, p1, p2, p2, p3)
static inline void cornerPower(mjtNum b[6], mjtNum p0, mjtNum p1, mjtNum p2, mjtNum p3) {
  b[0] = p0;
  b[1] = 5*(p1 - p0);
  b[2] = 10*(p0 - p1);
  b[3] = 10*(p2 - p0);
  b[4] = 5*(p0 + 2*p1 - 3*p2);
  b[5] = p3 + 5*p2 - 5*p1 - p0;
}


// value and derivative of a power-basis quintic
static inline mjtNum quintic(const mjtNum b[6], mjtNum u, mjtNum* deriv) {
  *deriv = (((5*b[5]*u + 4*b[4])*u + 3*b[3])*u + 2*b[2])*u + b[1];
  return ((((b[5]*u + b[4])*u + b[3])*u + b[2])*u + b[1])*u + b[0];
}


// evaluate a curve at x, OpenSim SmoothSegmentedFunction::calcValue and calcDerivative
static mjtNum curveEval(const mjMuscleCurve* c, mjtNum x, mjtNum* dydx) {
  int n = c->n;

  // linear extrapolation
  if (x <= c->x[0]) {
    *dydx = c->dydx[0];
    return c->y[0] + c->dydx[0]*(x - c->x[0]);
  }
  if (x >= c->x[n-1]) {
    *dydx = c->dydx[n-1];
    return c->y[n-1] + c->dydx[n-1]*(x - c->x[n-1]);
  }

  // section
  int i = 0;
  while (i < n-2 && x >= c->x[i+1]) {
    i++;
  }

  // corner control points, SegmentedQuinticBezierToolkit::calcQuinticBezierCornerControlPoints:
  // the end tangents meet at (xc, yc), and the curviness slides the interior points toward it
  mjtNum x0 = c->x[i], y0 = c->y[i], d0 = c->dydx[i];
  mjtNum x1 = c->x[i+1], y1 = c->y[i+1], d1 = c->dydx[i+1];
  mjtNum xc = mju_abs(d0 - d1) > 1.4901161193847656e-08 ?   // sqrt(machine epsilon)
              (y1 - y0 - x1*d1 + x0*d0) / (d0 - d1) : 0.5*(x0 + x1);
  mjtNum yc = (xc - x1)*d1 + y1;
  mjtNum k = c->curv[i];

  mjtNum bx[6], by[6];
  cornerPower(bx, x0, x0 + k*(xc - x0), x1 + k*(xc - x1), x1);
  cornerPower(by, y0, y0 + k*(yc - y0), y1 + k*(yc - y1), y1);

  // invert x(u) = x on [0, 1]: x(u) is increasing, Newton safeguarded by bisection
  mjtNum lo = 0, hi = 1, u = (x - x0) / (x1 - x0), dxdu = 1;
  for (int iter=0; iter < 60; iter++) {
    mjtNum f = quintic(bx, u, &dxdu) - x;
    if (f == 0) {
      break;
    }
    if (f > 0) {
      hi = u;
    } else {
      lo = u;
    }
    mjtNum next = u - f/dxdu;
    if (!(next > lo && next < hi)) {
      next = 0.5*(lo + hi);
    }
    mjtNum step = next - u;
    u = next;
    if (mju_abs(step) < mjMUSCLE_UTOL) {
      break;
    }
  }

  mjtNum dydu;
  mjtNum y = quintic(by, u, &dydu);
  quintic(bx, u, &dxdu);
  *dydx = dydu / dxdu;
  return y;
}


//------------------------------ Geyer2010 curves --------------------------------------------------

// Geyer and Herr (2010), Appendix II, with the curves of Geyer et al. (2003) as Song and Geyer
// implement them. The force-velocity curve is Hill's hyperbola when the fiber shortens and Song's
// eccentric hyperbola when it lengthens, continued smoothly beyond v_max. That branch reaches 1 at
// rest only for the eccentric force enhancement N = 1.5, which is therefore fixed.

#define mjGEYER_N   1.5               // eccentric force enhancement, f_V(v_max)
#define mjGEYER_ECC 7.56              // eccentric curvature factor

// active force-length: exp(ln(c) |(x - 1)/w|^3)
static mjtNum geyerActive(mjtNum x, mjtNum width, mjtNum logc, mjtNum* dydx) {
  mjtNum u = (x - 1) / width, au = mju_abs(u);
  mjtNum y = mju_exp(logc*au*au*au);
  *dydx = y * (3*logc) * (u*au) / width;
  return y;
}


// spring stretched beyond its rest length: ((x - rest)/strain)^2
static mjtNum geyerStretch(mjtNum x, mjtNum rest, mjtNum strain, mjtNum* dydx) {
  if (x <= rest) {
    *dydx = 0;
    return 0;
  }
  mjtNum t = (x - rest) / strain;
  *dydx = 2*t / strain;
  return t*t;
}


// spring compressed below its rest length: ((rest - x)/strain)^2
static mjtNum geyerCompress(mjtNum x, mjtNum rest, mjtNum strain, mjtNum* dydx) {
  if (x >= rest) {
    *dydx = 0;
    return 0;
  }
  mjtNum t = (rest - x) / strain;
  *dydx = -2*t / strain;
  return t*t;
}


// force-velocity, x = v_ce/(v_max l_opt) positive when lengthening: (1 + x)/(1 - K x) when
// shortening, N - (x - 1)/((1 - 7.56 K) x - 2) when lengthening (Song's N - t/(t - 1) with
// t = (x - 1)/(7.56 K x + 1), whose pole lies at negative x for K > 1/7.56)
static mjtNum geyerVelocity(mjtNum x, mjtNum K, mjtNum* dydx) {
  if (x <= 0) {
    mjtNum den = 1 - K*x;
    *dydx = (1 + K) / (den*den);
    return (1 + x) / den;
  }
  mjtNum c = 1 - mjGEYER_ECC*K;
  mjtNum b = c*x - 2;
  *dydx = (2 - c) / (b*b);
  return mjGEYER_N - (x - 1)/b;
}


//------------------------------ muscle parameters -------------------------------------------------

// OpenSim's MIN_NONZERO_DAMPING_COEFFICIENT: smaller fiber damping is no damping
#define mjMILLARD_MINDAMPING 0.001

// sin(acos(0.1)): OpenSim MuscleFixedWidthPennationModel maximum pennation angle
#define mjMILLARD_SINPHIMAX 0.99498743710662

// margin before a rigid tendon is called buckled: SimTK::SignificantReal, or in single precision,
// where that is below roundoff, a relative margin
#ifdef mjUSESINGLE
  #define mjMILLARD_BUCKLED(lslack) ((lslack) * (1 - 1e-6f))
#else
  #define mjMILLARD_BUCKLED(lslack) ((lslack) - 2.009718347115232e-14)
#endif

// fiber Newton: residual tolerance (in F_max) and iteration cap
#ifdef mjUSESINGLE
  #define mjMUSCLE_TOL 1e-5f
#else
  #define mjMUSCLE_TOL 1e-10
#endif
#define mjMUSCLE_MAXITER 50

typedef struct mjMuscle_ {
  mjtDyn type;                        // mjDYN_MILLARD2012 or mjDYN_GEYER2010
  mjtNum fmax;                        // maximum isometric force
  mjtNum lopt;                        // optimal fiber length
  mjtNum lslack;                      // tendon slack length
  mjtNum vmax;                        // maximum contraction velocity (m/s)
  mjtNum lmin;                        // minimum fiber length, -mjMAXVAL if none
  mjtNum beta;                        // fiber damping
  int rigid;                          // rigid tendon

  // Millard2012
  mjtNum width;                       // fixed pennation width l_opt sin(phi_opt)
  mjtNum minact;                      // minimum activation
  mjMuscleCurve curve[4];             // indexed by mjtMuscleCurve

  // Geyer2010
  mjtNum flwidth;                     // active force-length width w
  mjtNum logc;                        // log of the active force at |l/l_opt - 1| = w
  mjtNum K;                           // force-velocity shape factor
  mjtNum perest;                      // parallel element rest length (l_opt)
  mjtNum pestrain;                    // parallel element reference strain
  mjtNum sestrain;                    // tendon reference strain
} mjMuscle;


// unpack Millard2012 parameters, OpenSim Millard2012EquilibriumMuscle::extendFinalizeFromProperties
static void millardInit(mjMuscle* p, const mjtNum* dynprm, const mjtNum* gainprm,
                        const mjtNum* biasprm) {
  p->fmax = gainprm[mjMILLARD_FMAX];
  p->lopt = gainprm[mjMILLARD_LOPT];
  p->lslack = gainprm[mjMILLARD_LSLACK];
  p->vmax = gainprm[mjMILLARD_VMAX] * p->lopt;
  p->width = p->lopt * mju_sin(gainprm[mjMILLARD_PENNATION]);
  p->minact = dynprm[mjMILLARD_MINACT];
  p->rigid = biasprm[mjMILLARD_RIGID] != 0;

  // the active curve has no force below its minimum length, and pennation diverges at the width
  p->lmin = mju_max(gainprm[mjMILLARD_AFL_MIN]*p->lopt, p->width/mjMILLARD_SINPHIMAX);

  // classic Hill model (compliant tendon, no damping): OpenSim raises the active curve's floor
  // to 0.1 and the force-velocity slopes at vmax to half the slopes near vmax, which keeps the
  // force-velocity curve invertible; the damped and rigid models use 0 for all three
  int damped = dynprm[mjMILLARD_DAMPING] >= mjMILLARD_MINDAMPING;
  int classic = !damped && !p->rigid;
  p->beta = damped ? dynprm[mjMILLARD_DAMPING] : 0;

  activeCurve(p->curve + mjMUSCLECURVE_ACTIVE, gainprm, classic ? 0.1 : 0);
  passiveCurve(p->curve + mjMUSCLECURVE_PASSIVE, biasprm);
  tendonCurve(p->curve + mjMUSCLECURVE_TENDON, biasprm);
  velocityCurve(p->curve + mjMUSCLECURVE_VELOCITY, dynprm,
                classic ? 0.5*dynprm[mjMILLARD_FV_CONSLOPE] : 0,
                classic ? 0.5*dynprm[mjMILLARD_FV_ECCSLOPE] : 0);
}


// unpack Geyer2010 parameters; the fiber has no minimum length, the buffer element holds it
static void geyerInit(mjMuscle* p, const mjtNum* dynprm, const mjtNum* gainprm,
                      const mjtNum* biasprm) {
  p->fmax = gainprm[mjGEYER_FMAX];
  p->lopt = gainprm[mjGEYER_LOPT];
  p->lslack = gainprm[mjGEYER_LSLACK];
  p->vmax = gainprm[mjGEYER_VMAX] * p->lopt;
  p->lmin = -mjMAXVAL;
  p->beta = dynprm[mjGEYER_DAMPING];
  p->rigid = biasprm[mjGEYER_RIGID] != 0;
  p->flwidth = gainprm[mjGEYER_AFL_WIDTH];
  p->logc = mju_log(gainprm[mjGEYER_AFL_RESIDUAL]);
  p->K = dynprm[mjGEYER_FV_K];
  p->perest = biasprm[mjGEYER_PFL_REST];
  p->pestrain = biasprm[mjGEYER_PFL_STRAIN];
  p->sestrain = biasprm[mjGEYER_TFL_STRAIN];
}


// unpack the parameters of a muscle of the given type
static void muscleInit(mjMuscle* p, int type, const mjtNum* dynprm, const mjtNum* gainprm,
                       const mjtNum* biasprm) {
  p->type = type;
  switch ((mjtDyn) type) {
  case mjDYN_MILLARD2012:
    millardInit(p, dynprm, gainprm, biasprm);
    return;
  case mjDYN_GEYER2010:
    geyerInit(p, dynprm, gainprm, biasprm);
    return;
  default:
    mjERROR("dyntype %d is not a muscle-tendon model", type);
  }
}


// evaluate a curve of a muscle model at x
mjtNum mju_muscleCurve(int type, int curve, mjtNum x, const mjtNum dynprm[mjNDYN],
                       const mjtNum gainprm[mjNGAIN], const mjtNum biasprm[mjNBIAS],
                       mjtNum* dydx) {
  mjMuscle p;
  muscleInit(&p, type, dynprm, gainprm, biasprm);
  mjtNum y, slope;
  if (type == mjDYN_MILLARD2012) {
    if (curve < mjMUSCLECURVE_ACTIVE || curve > mjMUSCLECURVE_VELOCITY) {
      mjERROR("unknown Millard2012 curve %d", curve);
    }
    y = curveEval(p.curve + curve, x, &slope);
  } else {
    switch ((mjtMuscleCurve) curve) {
    case mjMUSCLECURVE_ACTIVE:
      y = geyerActive(x, p.flwidth, p.logc, &slope);
      break;
    case mjMUSCLECURVE_PASSIVE:
      y = geyerStretch(x, p.perest, p.pestrain, &slope);
      break;
    case mjMUSCLECURVE_TENDON:
      y = geyerStretch(x, 1, p.sestrain, &slope);
      break;
    case mjMUSCLECURVE_VELOCITY:
      y = geyerVelocity(x, p.K, &slope);
      break;
    case mjMUSCLECURVE_BUFFER:
      y = geyerCompress(x, 1 - p.flwidth, 0.5*p.flwidth, &slope);
      break;
    default:
      mjERROR("unknown Geyer2010 curve %d", curve);
      return 0;
    }
  }
  if (dydx) {
    *dydx = slope;
  }
  return y;
}


//------------------------------ fiber equilibrium -------------------------------------------------

// fixed-width pennation at fiber length l: cos(phi), sin(phi) and dphi/dl,
// OpenSim MuscleFixedWidthPennationModel; dphi is taken at the clamped angle
static void pennation(const mjMuscle* p, mjtNum l, mjtNum* cosphi, mjtNum* sinphi,
                      mjtNum* dphi) {
  if (p->width == 0) {
    *cosphi = 1;
    *sinphi = 0;
    *dphi = 0;
    return;
  }
  mjtNum s = mju_min(p->width / l, mjMILLARD_SINPHIMAX);
  *sinphi = s;
  *cosphi = mju_sqrt(1 - s*s);
  *dphi = -(s/l) / *cosphi;
}


// fiber force balance at fiber length l: residual and derivatives
typedef struct mjFiberEval_ {
  mjtNum cosphi;                      // cos of the pennation angle
  mjtNum active;                      // fiber force per unit activation
  mjtNum passive;                     // fiber force at zero activation
  mjtNum tendon;                      // tendon force f_T
  mjtNum dtendon;                     // d f_T / d lt, lt = l_T/l_slack
  mjtNum dlt;                         // d lt / d l
  mjtNum res;                         // f_T - cos(phi) (a active + passive)
  mjtNum dres;                        // d res / d l
} mjFiberEval;


// Millard2012: f_T = cos(phi) (a f_L f_V + f_P + beta v)
static void millardEval(mjFiberEval* e, const mjMuscle* p, mjtNum l, mjtNum lprev,
                        mjtNum lmtu, mjtNum act, mjtNum dt) {
  mjtNum cosphi, sinphi, dphi;
  pennation(p, l, &cosphi, &sinphi, &dphi);

  // normalized fiber length and velocity, and their derivatives with respect to l
  mjtNum ln = l / p->lopt, dln = 1 / p->lopt;
  mjtNum dvn = dt > 0 ? 1 / (dt * p->vmax) : 0;
  mjtNum vn = (l - lprev) * dvn;

  // tendon length, normalized, and its derivative
  mjtNum lt = (lmtu - l*cosphi) / p->lslack;
  mjtNum dlt = (l*sinphi*dphi - cosphi) / p->lslack;

  mjtNum dfl, dfp, dfv, dft;
  mjtNum fl = curveEval(p->curve + mjMUSCLECURVE_ACTIVE, ln, &dfl);
  mjtNum fp = curveEval(p->curve + mjMUSCLECURVE_PASSIVE, ln, &dfp);
  mjtNum fv = curveEval(p->curve + mjMUSCLECURVE_VELOCITY, vn, &dfv);
  mjtNum ft = curveEval(p->curve + mjMUSCLECURVE_TENDON, lt, &dft);

  mjtNum fiber = act*fl*fv + fp + p->beta*vn;
  mjtNum dfiber = act*(dfl*dln*fv + fl*dfv*dvn) + dfp*dln + p->beta*dvn;

  e->cosphi = cosphi;
  e->active = fl*fv;
  e->passive = fp + p->beta*vn;
  e->tendon = ft;
  e->dtendon = dft;
  e->dlt = dlt;
  e->res = ft - cosphi*fiber;
  e->dres = dft*dlt + sinphi*dphi*fiber - cosphi*dfiber;
}


// Geyer2010: f_SE + f_BE = f_V (f_PE + a f_L) + beta v
static void geyerEval(mjFiberEval* e, const mjMuscle* p, mjtNum l, mjtNum lprev, mjtNum lmtu,
                      mjtNum act, mjtNum dt) {
  mjtNum ln = l / p->lopt, dln = 1 / p->lopt;
  mjtNum dvn = dt > 0 ? 1 / (dt * p->vmax) : 0;
  mjtNum vn = (l - lprev) * dvn;
  mjtNum lt = (lmtu - l) / p->lslack, dlt = -1 / p->lslack;

  mjtNum dfl, dfp, dfb, dfv, dft;
  mjtNum fl = geyerActive(ln, p->flwidth, p->logc, &dfl);
  mjtNum fp = geyerStretch(ln, p->perest, p->pestrain, &dfp);
  mjtNum fb = geyerCompress(ln, 1 - p->flwidth, 0.5*p->flwidth, &dfb);
  mjtNum fv = geyerVelocity(vn, p->K, &dfv);
  mjtNum ft = geyerStretch(lt, 1, p->sestrain, &dft);

  e->cosphi = 1;
  e->active = fv*fl;
  e->passive = fv*fp + p->beta*vn - fb;
  e->tendon = ft;
  e->dtendon = dft;
  e->dlt = dlt;
  e->res = ft - (act*e->active + e->passive);
  e->dres = dft*dlt - (act*(dfv*dvn*fl + fv*dfl*dln) + dfv*dvn*fp + fv*dfp*dln + p->beta*dvn
                       - dfb*dln);
}


// residual of the fiber force balance, backward Euler with v = (l - lprev)/dt;
// dt <= 0: isometric (v = 0)
static void fiberEval(mjFiberEval* e, const mjMuscle* p, mjtNum l, mjtNum lprev, mjtNum lmtu,
                      mjtNum act, mjtNum dt) {
  if (p->type == mjDYN_MILLARD2012) {
    millardEval(e, p, l, lprev, lmtu, act, dt);
  } else {
    geyerEval(e, p, l, lprev, lmtu, act, dt);
  }
}


// one backward-Euler step of the fiber: Newton from the current length, steps capped at l_opt/2
static mjtNum fiberStep(const mjMuscle* p, mjtNum l, mjtNum lmtu, mjtNum act, mjtNum dt) {
  mjtNum lprev = l, maxstep = 0.5*p->lopt;
  l = mju_max(l, p->lmin);
  for (int iter=0; iter < mjMUSCLE_MAXITER; iter++) {
    mjFiberEval e;
    fiberEval(&e, p, l, lprev, lmtu, act, dt);

    // converged, or flat: slack tendon and slack fiber, every length is a root
    if (mju_abs(e.res) < mjMUSCLE_TOL || e.dres == 0) {
      break;
    }
    l = mju_max(p->lmin, l + mju_clip(-e.res/e.dres, -maxstep, maxstep));
  }
  return l;
}


// isometric equilibrium: Newton safeguarded by a bracket, independent of the current fiber
static mjtNum fiberIsometric(const mjMuscle* p, mjtNum lmtu, mjtNum act) {
  mjFiberEval e;

  // the residual is positive toward short fibers, where the tendon is stretched: at the minimum
  // length, where the fiber is clamped if it is not, or below the shortest length tried
  mjtNum lo = p->lmin;
  if (lo > -mjMAXVAL) {
    fiberEval(&e, p, lo, lo, lmtu, act, 0);
    if (e.res <= 0) {
      return lo;
    }
  } else {
    lo = 0;
    for (int i=0; i < 64; i++) {
      fiberEval(&e, p, lo, lo, lmtu, act, 0);
      if (e.res > 0) {
        break;
      }
      lo -= p->lopt;
    }
  }

  // walk up until the residual turns negative (the passive force grows without bound)
  mjtNum hi = lo + p->lopt;
  for (int i=0; i < 64; i++) {
    fiberEval(&e, p, hi, hi, lmtu, act, 0);
    if (e.res < 0) {
      break;
    }
    lo = hi;
    hi += p->lopt;
  }
  if (e.res >= 0) {
    return hi;
  }

  mjtNum l = 0.5*(lo + hi);
  for (int iter=0; iter < 100; iter++) {
    fiberEval(&e, p, l, l, lmtu, act, 0);
    if (mju_abs(e.res) < mjMUSCLE_TOL) {
      break;
    }
    if (e.res > 0) {
      lo = l;
    } else {
      hi = l;
    }
    mjtNum next = e.dres != 0 ? l - e.res/e.dres : l;
    if (!(next > lo && next < hi)) {
      next = 0.5*(lo + hi);
    }
    if (next == l) {
      break;
    }
    l = next;
  }
  return l;
}


//------------------------------ rigid tendon ------------------------------------------------------

// Millard2012 rigid tendon fiber length, OpenSim MuscleFixedWidthPennationModel::calcFiberLength
static mjtNum millardRigidLength(const mjMuscle* p, mjtNum lmtu) {
  mjtNum along = lmtu - p->lslack;
  mjtNum l;
  if (p->width == 0) {
    l = along;
  } else {
    mjtNum lmax = p->width / mjMILLARD_SINPHIMAX;
    l = along >= lmax*mju_sqrt(1 - mjMILLARD_SINPHIMAX*mjMILLARD_SINPHIMAX) ?
        mju_sqrt(along*along + p->width*p->width) : lmax;
  }
  return mju_max(l, p->lmin);
}


// Millard2012 rigid tendon gain and bias, OpenSim Millard2012EquilibriumMuscle with
// ignore_tendon_compliance: v = v_mtu cos(phi), no velocity if the tendon buckles, no force if the
// fiber is clamped, and the fiber only pulls
static void millardRigid(const mjMuscle* p, mjtNum lmtu, mjtNum vmtu, mjtNum act,
                         mjtNum* gain, mjtNum* bias, mjtNum* dforce_dvel) {
  *gain = *bias = 0;
  if (dforce_dvel) *dforce_dvel = 0;

  mjtNum l = millardRigidLength(p, lmtu);
  mjtNum cosphi, sinphi, dphi;
  pennation(p, l, &cosphi, &sinphi, &dphi);

  int buckled = lmtu - l*cosphi < mjMILLARD_BUCKLED(p->lslack);
  mjtNum v = buckled ? 0 : vmtu*cosphi;

  if (l < p->lmin || (l <= p->lmin && v <= 0)) {
    return;
  }

  mjtNum dfl, dfp, dfv = 0, fv = 1, vn = v / p->vmax;
  mjtNum fl = curveEval(p->curve + mjMUSCLECURVE_ACTIVE, l / p->lopt, &dfl);
  mjtNum fp = curveEval(p->curve + mjMUSCLECURVE_PASSIVE, l / p->lopt, &dfp);
  if (!buckled) {
    fv = curveEval(p->curve + mjMUSCLECURVE_VELOCITY, vn, &dfv);
  }
  if (act*fl*fv + fp + p->beta*vn <= 0) {
    return;
  }

  *gain = -p->fmax * cosphi * fl * fv;
  *bias = -p->fmax * cosphi * (fp + p->beta*vn);
  if (dforce_dvel && !buckled) {
    *dforce_dvel = -p->fmax * cosphi*cosphi * (act*fl*dfv + p->beta) / p->vmax;
  }
}


// Millard2012 d(force)/d(l_mtu) with a rigid tendon: the fiber moves with the path
static mjtNum millardRigidLen(const mjMuscle* p, mjtNum lmtu, mjtNum vmtu, mjtNum act) {
  // no force, or a fiber on its minimum length
  mjtNum gain, bias;
  millardRigid(p, lmtu, vmtu, act, &gain, &bias, NULL);
  mjtNum l = millardRigidLength(p, lmtu);
  if ((gain == 0 && bias == 0) || l <= p->lmin) {
    return 0;
  }

  // dl/dl_mtu: 1 unpennated, along/l on the pennated branch, 0 at the maximum pennation
  mjtNum along = lmtu - p->lslack;
  mjtNum dl = p->width == 0 ? 1 : (l > p->width / mjMILLARD_SINPHIMAX ? along / l : 0);

  mjtNum cosphi, sinphi, dphi;
  pennation(p, l, &cosphi, &sinphi, &dphi);
  mjtNum dcos = -sinphi*dphi;
  int buckled = lmtu - l*cosphi < mjMILLARD_BUCKLED(p->lslack);
  mjtNum vn = buckled ? 0 : vmtu*cosphi / p->vmax;
  mjtNum dvn = buckled ? 0 : vmtu*dcos / p->vmax;

  mjtNum dfl, dfp, dfv = 0, fv = 1;
  mjtNum fl = curveEval(p->curve + mjMUSCLECURVE_ACTIVE, l / p->lopt, &dfl);
  mjtNum fp = curveEval(p->curve + mjMUSCLECURVE_PASSIVE, l / p->lopt, &dfp);
  if (!buckled) {
    fv = curveEval(p->curve + mjMUSCLECURVE_VELOCITY, vn, &dfv);
  }
  mjtNum fiber = act*fl*fv + fp + p->beta*vn;
  mjtNum dfiber = act*(dfl/p->lopt*fv + fl*dfv*dvn) + dfp/p->lopt + p->beta*dvn;
  return -p->fmax * (dcos*fiber + cosphi*dfiber) * dl;
}


// rigid tendon fiber length
static mjtNum rigidLength(const mjMuscle* p, mjtNum lmtu) {
  return p->type == mjDYN_MILLARD2012 ? millardRigidLength(p, lmtu) : lmtu - p->lslack;
}


//------------------------------ muscle actuators --------------------------------------------------

// activation that multiplies the gain, as in mj_fwdActuation
static mjtNum muscleActivation(const mjModel* m, const mjData* d, int i) {
  int adr = m->actuator_actadr[i] + m->actuator_actnum[i] - 1;
  if (m->actuator_actearly[i]) {
    return mj_nextActivation(m, d, i, adr, d->act_dot[adr]);
  }
  return d->act[adr];
}


// unpack the parameters of muscle actuator i
static void muscleModel(mjMuscle* p, const mjModel* m, int i) {
  int type = m->actuator_dyntype[i];
  if (type != mjDYN_MILLARD2012 && type != mjDYN_GEYER2010) {
    mjERROR("actuator %d is not a muscle-tendon actuator", i);
  }
  muscleInit(p, type, m->actuator_dynprm + mjNDYN*i, m->actuator_gainprm + mjNGAIN*i,
             m->actuator_biasprm + mjNBIAS*i);
}


// muscle activation and fiber velocity: act_dot of both states of actuator i
void mj_muscleActDot(const mjModel* m, mjData* d, int i, mjtNum ctrl) {
  int fiber = m->actuator_actadr[i];
  int activation = fiber + m->actuator_actnum[i] - 1;
  const mjtNum* dynprm = m->actuator_dynprm + mjNDYN*i;
  mjMuscle p;
  muscleModel(&p, m, i);

  // activation, time constants scaled by the activation as in Millard et al. (2013); Millard2012
  // as OpenSim's MuscleFirstOrderActivationDynamicModel, excitation floored at the minimum
  // activation; Geyer2010 with a smooth switch between the time constants
  if (p.type == mjDYN_MILLARD2012) {
    mjtNum prm[3] = {dynprm[mjMILLARD_TAUACT], dynprm[mjMILLARD_TAUDEACT], 0};
    d->act_dot[activation] = mju_muscleDynamics(mju_max(ctrl, p.minact), d->act[activation], prm);
  } else {
    mjtNum prm[3] = {dynprm[mjGEYER_TAUACT], dynprm[mjGEYER_TAUDEACT], dynprm[mjGEYER_TAUSMOOTH]};
    d->act_dot[activation] = mju_muscleDynamics(ctrl, d->act[activation], prm);
  }

  // fiber: one backward-Euler step, or the rigid-tendon length
  mjtNum dt = m->opt.timestep;
  mjtNum lmtu = d->actuator_length[m->actuator_outadr[i]];
  mjtNum l = p.lopt * (1 + d->act[fiber]);
  mjtNum lnext = p.rigid ? rigidLength(&p, lmtu)
                         : fiberStep(&p, l, lmtu, muscleActivation(m, d, i), dt);
  d->act_dot[fiber] = (lnext - l) / (p.lopt * dt);
}


// muscle gain and bias of actuator i at the fiber solved by mj_muscleActDot
void mj_muscleGainBias(const mjModel* m, const mjData* d, int i, mjtNum* gain, mjtNum* bias) {
  int fiber = m->actuator_actadr[i];
  int oadr = m->actuator_outadr[i];
  mjtNum act = muscleActivation(m, d, i);
  mjMuscle p;
  muscleModel(&p, m, i);

  if (p.rigid && p.type == mjDYN_MILLARD2012) {
    millardRigid(&p, d->actuator_length[oadr], d->actuator_velocity[oadr], act, gain, bias, NULL);
    return;
  }

  // end-of-step fiber, consistent with act_dot
  mjtNum dt = m->opt.timestep;
  mjtNum lmtu = d->actuator_length[oadr];
  mjtNum l = p.rigid ? rigidLength(&p, lmtu)
                     : mju_max(p.lmin, p.lopt * (1 + d->act[fiber] + dt*d->act_dot[fiber]));
  mjtNum lprev = p.lopt * (1 + d->act[fiber]);

  mjFiberEval e;
  fiberEval(&e, &p, l, lprev, lmtu, act, dt);

  // clamped fiber: the force balance does not hold, the tendon force is transmitted
  if (l <= p.lmin && e.res < 0) {
    *gain = 0;
    *bias = -p.fmax * e.tendon;
    return;
  }

  // Geyer2010 rigid tendon: the fiber force at the path beyond the tendon, which only pulls
  if (p.rigid && act*e.active + e.passive <= 0) {
    *gain = *bias = 0;
    return;
  }
  *gain = -p.fmax * e.cosphi * e.active;
  *bias = -p.fmax * e.cosphi * e.passive;
}


// d(actuator_force)/d(actuator_velocity) of muscle actuator i: zero where the force depends on
// lengths alone, a compliant tendon or a Geyer2010 rigid tendon, whose fiber velocity is the
// change of the fiber over the step; with a Millard2012 rigid tendon the fiber moves with the path
mjtNum mj_muscleForceVel(const mjModel* m, const mjData* d, int i) {
  int oadr = m->actuator_outadr[i];
  mjMuscle p;
  muscleModel(&p, m, i);
  if (!p.rigid || p.type != mjDYN_MILLARD2012) {
    return 0;
  }
  mjtNum gain, bias, dforce_dvel;
  millardRigid(&p, d->actuator_length[oadr], d->actuator_velocity[oadr],
               muscleActivation(m, d, i), &gain, &bias, &dforce_dvel);
  return dforce_dvel;
}


// d(actuator_force)/d(actuator_length) of muscle actuator i: with a compliant tendon the fiber
// follows the path through the backward-Euler solve, so tendon and fiber act as springs in series
mjtNum mj_muscleForceLen(const mjModel* m, const mjData* d, int i) {
  int fiber = m->actuator_actadr[i];
  int oadr = m->actuator_outadr[i];
  mjtNum act = muscleActivation(m, d, i);
  mjtNum lmtu = d->actuator_length[oadr];
  mjMuscle p;
  muscleModel(&p, m, i);

  if (p.rigid && p.type == mjDYN_MILLARD2012) {
    return millardRigidLen(&p, lmtu, d->actuator_velocity[oadr], act);
  }

  // end-of-step fiber and the residual there
  mjtNum dt = m->opt.timestep;
  mjtNum l = p.rigid ? rigidLength(&p, lmtu)
                     : mju_max(p.lmin, p.lopt * (1 + d->act[fiber] + dt*d->act_dot[fiber]));
  mjtNum lprev = p.lopt * (1 + d->act[fiber]);
  mjFiberEval e;
  fiberEval(&e, &p, l, lprev, lmtu, act, dt);

  // Geyer2010 rigid tendon: the fiber moves with the path, d(force)/d(l) of the fiber force
  if (p.rigid) {
    if (act*e.active + e.passive <= 0) {
      return 0;
    }
    return -p.fmax * (e.dtendon*e.dlt - e.dres);
  }

  // tendon stiffness, and its series combination with the fiber unless the fiber is clamped
  mjtNum ktendon = p.fmax * e.dtendon / p.lslack;
  if ((l <= p.lmin && e.res < 0) || e.dres == 0) {
    return -ktendon;
  }
  return -ktendon * (1 - e.dtendon*e.dlt / e.dres);
}


// set every muscle fiber to its isometric equilibrium at the current actuator lengths
void mj_muscleEquilibrium(const mjModel* m, mjData* d) {
  for (int i=0; i < m->nu; i++) {
    int type = m->actuator_dyntype[i];
    if ((type != mjDYN_MILLARD2012 && type != mjDYN_GEYER2010) || m->actuator_actadr[i] < 0) {
      continue;
    }
    mjMuscle p;
    muscleModel(&p, m, i);
    int fiber = m->actuator_actadr[i];
    int activation = fiber + m->actuator_actnum[i] - 1;
    mjtNum lmtu = d->actuator_length[m->actuator_outadr[i]];
    mjtNum l = p.rigid ? rigidLength(&p, lmtu) : fiberIsometric(&p, lmtu, d->act[activation]);
    d->act[fiber] = l / p.lopt - 1;
    d->act_dot[fiber] = 0;
  }
}
