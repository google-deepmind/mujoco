# Copyright 2026 DeepMind Technologies Limited
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
"""Accuracy-vs-cost benchmark for MuJoCo flex formulations.

Reproduces the tables in ../references/benchmarks.md:

1. Cantilever: a 0.48 x 0.04 x 0.04 m rubber beam (E = 5 MPa, nu = 0.25)
   clamped at x = 0 under a 0.25 N tip load with gravity off, compared
   against the Euler-Bernoulli tip deflection P L^3 / (3 E I).
2. Torsion: a 0.6 m, 5 mm radius rod (G = 1.5 MPa) clamped at x = 0 with a
   0.02 N*m end torque on its <name>_twist joint, compared against T L / (G J).
3. Block: a 0.1 m soft cube (E = 50 kPa, nu = 0) squeezed between the floor
   and a 2 kg plate, compared against uniaxial compression (plate load plus
   self-weight).
4. Plate: a 0.2 m square, 4 mm plastic plate (E = 250 MPa, nu = 0.3) sagging
   under its own weight with its boundary pinned (simply supported) or its
   two outer vertex rings pinned (clamped), compared against the Kirchhoff
   center deflection alpha q a^4 / D, on flexcomp grids and on staggered
   (near-equilateral) type="direct" meshes.
5. Membrane: a 0.3 x 0.2 m, 1 mm stretch-only sheet (E = 1 MPa, nu = 0)
   pinned along x = 0 and pulled by 0.2 N along x = L, gravity off, compared
   against the uniaxial elongation F L / (E t W).
6. Hertz: a rigid, frictionless 0.1 m sphere resting under its weight (1.47 N)
   on a 0.06 x 0.06 x 0.03 m block (E = 100 kPa, nu = 0.3) pinned at its base,
   compared against half-space Hertz theory (contact radius a = 10 mm,
   indentation 1 mm) for the sphere's drop relative to the top surface at
   r = 1.5 a.
7. Incompressible: the cantilever at poisson = 0.25 / 0.45 / 0.49, whose
   Euler-Bernoulli reference ignores poisson, to quantify volumetric locking.
8. Dynamics: the cantilever released from its static tip-loaded shape and
   ringing with damping = 0.005 s, compared against the first bending
   frequency f1 = (1.8751^2 / (2 pi)) sqrt(E I / (rho A L^4)) and the
   stiffness-proportional decay rate damping * omega^2 / 2, plus the extra
   decay of the implicit step at several timesteps.
9. Stability: the largest stable timestep of the tip-loaded cantilever per
   formulation, Young's modulus, and integrator (discrete vs. Euler).
10. Attach: the cantilever beam (quadratic 2x1x1) and a box-section rod of
    the same size sagging under gravity from a static mount, from a mount
    driven along a sinusoid by a slide or hinge position servo, or with a
    free tip mass held by <connect> or <weld>; plus a trilinear gripper pad
    pressed onto a table by a slide-jointed finger. Settled deflections are
    compared against Euler-Bernoulli for the model's lumped gravity loads
    (uniaxial compression for the pad), and settled servo forces against the
    load they hold.
11. Stretch: a vertical rod (17 and 33 vertices) and a 13x13 cloth hanging
    from their top vertices, held inextensible by <edge equality="true"/>
    with solref timeconst 0.02 to 0.002 s; reports the steady edge strain.
12. Collision: cloth with self-collision dropped onto a sphere on the floor
    (11x11 to 61x61 vertices, plus a trilinear shell on a 41x41 mesh); every
    solver is timed from the same folded state, against nv and the vertex
    count.

Every formulation runs with integrator="discrete" under the Newton and CG
solvers, except the Newton-only timestep and stability sweeps (the latter also
runs Euler). Timings are wall-clock milliseconds per mj_step over the loaded
transient (the free vibration for dynamics) and depend on the host CPU.
"""

from collections.abc import Callable
import dataclasses
import functools
import os
import sys
import time

from absl import app
from absl import flags
import mujoco
import numpy as np

_PROBLEMS = flags.DEFINE_list(
    'problems',
    [
        'cantilever',
        'torsion',
        'block',
        'plate',
        'membrane',
        'hertz',
        'incompressible',
        'dynamics',
        'stability',
        'attach',
        'stretch',
        'collision',
    ],
    'Problems to run.',
)
_SOLVERS = flags.DEFINE_list('solvers', ['Newton', 'CG'], 'Solvers to time.')
_TIMING_STEPS = flags.DEFINE_integer(
    'timing_steps', 500, 'Steps timed from the moment the load is applied.'
)
_ONLY = flags.DEFINE_string(
    'only', '', 'Run only formulations whose label contains this substring.'
)
_VERBOSE = flags.DEFINE_bool('verbose', False, 'Print per-run diagnostics.')

_DT = 0.002
_TOLERANCE = 1e-6
_ITERATIONS = 400
_GRAVITY = 9.81

# Cantilever: rubber beam, static tip load, gravity off.
_BEAM_L = 0.48
_BEAM_H = 0.04
_BEAM_E = 5e6
_BEAM_NU = 0.25
_BEAM_MASS = 0.15
_TIP_LOAD = 0.25

# Torsion: thin elastic rod, static end torque, gravity off.
_ROD_L = 0.6
_ROD_R = 0.005
_ROD_E = 4e6
_ROD_G = 1.5e6
_ROD_MASS = 0.06
_TORQUE = 0.02

# Block: soft cube between a floor and a heavy plate. Frictional contacts keep
# the cube from squirting out sideways, and poisson = 0 keeps the uniaxial
# reference exact despite the friction (no lateral expansion to resist). The
# contacts use solref = 2 * timestep: contact stiffness scales with the light
# flex vertices' effective mass, so the default solref lets the 20x heavier
# plate sink through fine full/quadratic meshes.
_BLOCK_S = 0.1
_BLOCK_E = 5e4
_BLOCK_NU = 0.0
_BLOCK_MASS = 0.1
_PLATE_MASS = 2.0

# Plate: square plate sagging under its own weight (w_max / t < 0.05). flexcomp
# splits `mass` equally over all vertices, boundary ones included, so the grid
# mass is rho_A (n h)^2 to give every vertex its tributary load rho_A g h^2.
_SHELL_A = 0.2
_SHELL_T = 0.004
_SHELL_E = 2.5e8
_SHELL_NU = 0.3
_SHELL_RHO_A = 4.0  # Areal density in kg/m^2: q = rho_A g = 39.2 Pa.
# Center deflection w_max D / (q a^4): Navier series for simple supports and
# Taylor & Govindjee (2004) for clamped edges (Timoshenko: 0.00406, 0.00126).
_SHELL_SIMPLE_COEF = 0.00406235
_SHELL_CLAMPED_COEF = 0.00126532

# Membrane: stretch-only sheet pinned along x = 0 and pulled along x = L with
# consistent (trapezoidal) vertex forces, gravity off. poisson = 0 makes the
# uniaxial reference exact; the 0.1% strain keeps the StVK membrane within
# 0.15% of linear elasticity.
_SHEET_L = 0.3
_SHEET_W = 0.2
_SHEET_T = 0.001
_SHEET_E = 1e6
_SHEET_MASS = 0.06
_SHEET_FORCE = 0.2

# Hertz: a rigid, frictionless sphere pressed into a soft block by its own
# weight. The block bottom is pinned to a rigid base, and the sphere weight
# gives a Hertz contact radius a = _HERTZ_A, so the block spans 6a x 6a x 3a.
# The contacts use solref = 2 * timestep and a near-hard solimp: under the
# default solimp the sphere sinks 0.5-1.3 mm into fine full/quadratic blocks,
# as much as the 0.7 mm dent being measured, and fine quadratic blocks never
# settle.
_HERTZ_E = 1e5
_HERTZ_NU = 0.3
_HERTZ_R = 0.1
_HERTZ_A = 0.01
_HERTZ_S = 0.06
_HERTZ_H = 0.03
_HERTZ_MASS = 0.1
_HERTZ_RADIUS = 5e-4
# The dent is measured against the top surface at r = 1.5 a; see _run_hertz.
_HERTZ_RING = 0.015
_HERTZ_SOLIMP = '0.9999 0.9999 0.001'

# Dynamics: the cantilever settles under its tip load with the default beam
# damping, then the load is removed and it rings down with light damping
# (flex_damping is read at every step, so it is switched at runtime). The
# implicit step of the discrete integrator adds decay of its own, which the
# timestep sweep isolates with damping = 0.
_BEAM_DAMPING = 0.05
_RING_DAMPING = 0.005
_RING_TIME = 2.0
_RING_TIMESTEPS = (0.004, 0.002, 0.001, 0.0005)
_BETA_L = 1.8751040687  # First root of cos(x) cosh(x) = -1.

# Stability: the cantilever under its tip load for 0.5 s per Young's modulus,
# timestep, and integrator. Stable means no bad-state warning, a finite state,
# and every DOF speed below 10 (m/s on vertex slides, rad/s on twist hinges).
# Euler integrates elasticity and its damping explicitly, so it also tries
# timesteps below the grid.
_STAB_YOUNG = (5e5, 5e6, 5e7)
_STAB_TIMESTEPS = (0.0005, 0.001, 0.002, 0.004, 0.008, 0.016)
_STAB_EULER_TIMESTEPS = (0.0001, 0.0002) + _STAB_TIMESTEPS
_STAB_TIME = 0.5
_STAB_SPEED = 10.0
_STAB_INTEGRATORS = (  # (integrator, damping)
    ('discrete', _RING_DAMPING),
    ('Euler', _RING_DAMPING),
    ('Euler', 0.0),
)
_BAD_STATE = (
    mujoco.mjtWarning.mjWARN_BADQPOS,
    mujoco.mjtWarning.mjWARN_BADQVEL,
    mujoco.mjtWarning.mjWARN_BADQACC,
)

# Attach: the cantilever beam and rod under gravity. Driven mounts follow a
# sinusoid below the beam's first bending mode (about 4.5 Hz) during the
# timed steps and then hold still, so every row settles to a static sag. The
# free tip mass and the attachment equalities use solref = 2 * timestep, like
# the block contacts, because they act on light flex vertices. Jointed mounts
# keep their origin _MOUNT_X behind the clamp face to avoid a known engine bug
# in mj_flexPassiveInterp / mj_flexPassiveBendInterp: its direct-write fast
# path writes R' f only into each node body's 3 slide DOFs (omitting the free
# nodes' share on parent joint DOFs, and writing into the parent's first 3 DOFs
# when a pinned node sits at the parent's (0, 0, 0) origin).
_MOUNT_X = 0.02
_DRIVE_HZ = 2.0
_SLIDE_AMP = 0.02
_SLIDE_KP = 2000.0
_HINGE_AMP = 0.1
_HINGE_KP = 100.0
_TIP_MASS = 0.05
_ATTACH_SOLREF = '0.004 1'
# Gripper pad: 0.04 m square, 0.02 m thick along the finger's slide axis,
# 1 mm above the table; the finger target travels 5 mm down into it.
_PAD_T = 0.02
_PAD_W = 0.04
_PAD_E = 5e4
_PAD_MASS = 0.02
_PAD_GAP = 0.001
_PAD_PUSH = 0.005
_FINGER_KP = 1000.0

# Stretch: a rod with the torsion rod's section and a cloth, both 0.48 m
# long, hanging under gravity from their top vertices. Edge equalities hold
# the length with the default solimp, so their timeconst sets the stretch.
_STRETCH_L = 0.48
_STRETCH_MASS = 0.1

# Collision: a 0.6 m cloth released 5 cm above a 0.1 m radius sphere that
# rests on the floor drapes over it, lands, and folds onto itself. The drop
# runs once, untimed, under CG; every solver is then timed for
# _COLLISION_STEPS from that folded state (fewer than --timing_steps: Newton
# takes about 0.6 s per step on the 61x61 cloth).
_CLOTH_L = 0.6
_CLOTH_MASS = 0.2
_CLOTH_R = 0.002
_SPHERE_R = 0.1
_DROP_TIME = 0.8
_COLLISION_STEPS = 100


@dataclasses.dataclass(frozen=True)
class Formulation:
  """A flex formulation and problem configuration to benchmark."""

  label: str
  dof: str  # 'rod', 'full', 'trilinear', or 'quadratic'.
  count: tuple[int, int, int]
  cellcount: tuple[int, int, int] = (1, 1, 1)
  # Rods: start the rod this many segments behind the clamp. The clamp acts at
  # the middle of the pinned edge 0, so 0.5 makes the bending length exact;
  # torsion also loses half a segment at the loaded end, so 1.0 makes the
  # twisting length exact. Clamped plates: extend the grid this many cells
  # past the clamp line, which runs midway between the two pinned rings.
  offset: float = 0.0
  # Cantilever only: overrides _BEAM_NU.
  poisson: float | None = None
  elastic2d: str = 'both'  # 2D flexes only.
  # 2D flexes only: 'grid' (flexcomp type="grid", all quads split along the
  # same diagonal) or 'staggered' (type="direct", rows shifted by half a cell).
  mesh: str = 'grid'
  load: float = 1.0  # Membrane only: multiple of the nominal edge force.
  # Attach only: 'static', 'slide', 'hinge', 'connect', 'weld', or 'finger'.
  attach: str = ''
  # Stretch only: solref timeconst of the edge equality constraints.
  timeconst: float = 0.0
  # Non-interpolated 3D flexes only: 0 = StVK, 1 = simplified SNH (MjSpec-only).
  elastic3d: int = 0


_CANTILEVER = (
    Formulation('1D rod, 9 vertices', 'rod', (9, 1, 1)),
    Formulation('1D rod, 17 vertices', 'rod', (17, 1, 1)),
    Formulation('1D rod, 9 vertices, half-segment offset', 'rod', (9, 1, 1),
                offset=0.5),
    Formulation('1D rod, 17 vertices, half-segment offset', 'rod', (17, 1, 1),
                offset=0.5),
    Formulation('1D rod, 17 vertices, full-segment offset', 'rod', (17, 1, 1),
                offset=1.0),
    Formulation('trilinear 1x1x1', 'trilinear', (17, 3, 3), (1, 1, 1)),
    Formulation('trilinear 2x1x1', 'trilinear', (17, 3, 3), (2, 1, 1)),
    Formulation('trilinear 4x1x1', 'trilinear', (17, 3, 3), (4, 1, 1)),
    Formulation('trilinear 8x1x1', 'trilinear', (17, 3, 3), (8, 1, 1)),
    Formulation('trilinear 4x2x2', 'trilinear', (17, 3, 3), (4, 2, 2)),
    Formulation('quadratic 1x1x1', 'quadratic', (17, 3, 3), (1, 1, 1)),
    Formulation('quadratic 2x1x1', 'quadratic', (17, 3, 3), (2, 1, 1)),
    Formulation('quadratic 4x1x1', 'quadratic', (17, 3, 3), (4, 1, 1)),
    Formulation('full 17x3x3', 'full', (17, 3, 3)),
    Formulation('full 33x5x5', 'full', (33, 5, 5)),
)

_TORSION = (
    Formulation('1D rod, 9 vertices', 'rod', (9, 1, 1)),
    Formulation('1D rod, 31 vertices', 'rod', (31, 1, 1)),
    Formulation('1D rod, 9 vertices, half-segment offset', 'rod', (9, 1, 1),
                offset=0.5),
    Formulation('1D rod, 31 vertices, half-segment offset', 'rod', (31, 1, 1),
                offset=0.5),
    Formulation('1D rod, 9 vertices, full-segment offset', 'rod', (9, 1, 1),
                offset=1.0),
    Formulation('1D rod, 31 vertices, full-segment offset', 'rod', (31, 1, 1),
                offset=1.0),
)

_BLOCK = (
    Formulation('trilinear 1x1x1', 'trilinear', (10, 10, 10), (1, 1, 1)),
    Formulation('trilinear 2x2x2', 'trilinear', (10, 10, 10), (2, 2, 2)),
    Formulation('trilinear 3x3x3', 'trilinear', (10, 10, 10), (3, 3, 3)),
    Formulation('quadratic 1x1x1', 'quadratic', (10, 10, 10), (1, 1, 1)),
    Formulation('quadratic 2x2x2', 'quadratic', (10, 10, 10), (2, 2, 2)),
    Formulation('full 6x6x6', 'full', (6, 6, 6)),
    Formulation('full 8x8x8', 'full', (8, 8, 8)),
    Formulation('full 8x8x8, SNH', 'full', (8, 8, 8), elastic3d=1),
    Formulation('full 10x10x10', 'full', (10, 10, 10)),
    Formulation('full 12x12x12', 'full', (12, 12, 12)),
)

# Interpolated shells (trilinear/quadratic with elastic2d != "none") cannot
# model these open flat sheets: their node lattice spans the bounding box of
# the vertices, which has zero thickness here (compile error "flex grid
# rotation R0 is not orthonormal"), and shell mode puts the shell on the faces
# of that box (closed surfaces such as bunny_shell.xml).
_PLATE_SIMPLE = (
    Formulation('full 9x9', 'full', (9, 9, 1)),
    Formulation('full 17x17', 'full', (17, 17, 1)),
    Formulation('full 33x33', 'full', (33, 33, 1)),
    Formulation('full 17x17, bend only', 'full', (17, 17, 1),
                elastic2d='bend'),
    Formulation('full 33x33, bend only', 'full', (33, 33, 1),
                elastic2d='bend'),
    Formulation('staggered 10x11', 'full', (10, 11, 1), mesh='staggered'),
    Formulation('staggered 19x21', 'full', (19, 21, 1), mesh='staggered'),
    Formulation('staggered 34x39', 'full', (34, 39, 1), mesh='staggered'),
)

_PLATE_CLAMPED = (
    Formulation('full 9x9', 'full', (9, 9, 1)),
    Formulation('full 17x17', 'full', (17, 17, 1)),
    Formulation('full 33x33', 'full', (33, 33, 1)),
    Formulation('full 9x9, half-cell offset', 'full', (9, 9, 1), offset=0.5),
    Formulation('full 17x17, half-cell offset', 'full', (17, 17, 1),
                offset=0.5),
    Formulation('full 33x33, half-cell offset', 'full', (33, 33, 1),
                offset=0.5),
)

_MEMBRANE = (
    Formulation('full 5x5', 'full', (5, 5, 1), elastic2d='stretch'),
    Formulation('full 9x9', 'full', (9, 9, 1), elastic2d='stretch'),
    Formulation('full 17x17', 'full', (17, 17, 1), elastic2d='stretch'),
    Formulation('full 9x9, 10x load (1% strain)', 'full', (9, 9, 1),
                elastic2d='stretch', load=10),
    Formulation('full 9x9, 100x load (10% strain)', 'full', (9, 9, 1),
                elastic2d='stretch', load=100),
)

# Interpolated blocks share a 25x25x5 collision grid (2.5 mm = a / 4 on top).
# The DOF tiers (150 / 972 / 3042) space the nodes 1.5 a / 0.75 a / 0.5 a.
_HERTZ = (
    Formulation('trilinear 4x4x2', 'trilinear', (25, 25, 5), (4, 4, 2)),
    Formulation('trilinear 8x8x4', 'trilinear', (25, 25, 5), (8, 8, 4)),
    Formulation('trilinear 12x12x6', 'trilinear', (25, 25, 5), (12, 12, 6)),
    Formulation('quadratic 2x2x1', 'quadratic', (25, 25, 5), (2, 2, 1)),
    Formulation('quadratic 4x4x2', 'quadratic', (25, 25, 5), (4, 4, 2)),
    Formulation('quadratic 6x6x3', 'quadratic', (25, 25, 5), (6, 6, 3)),
    Formulation('full 5x5x3', 'full', (5, 5, 3)),
    Formulation('full 9x9x5', 'full', (9, 9, 5)),
    Formulation('full 13x13x7', 'full', (13, 13, 7)),
)


def _incompressible_forms() -> tuple[Formulation, ...]:
  """Returns the cantilever formulations swept over Poisson's ratio."""
  labels = (
      'trilinear 8x1x1', 'quadratic 2x1x1', 'quadratic 4x1x1',
      'full 17x3x3', 'full 33x5x5',
  )
  base = [form for form in _CANTILEVER if form.label in labels]
  base.insert(
      4, Formulation('full 17x3x3, SNH', 'full', (17, 3, 3), elastic3d=1)
  )
  forms = []
  for form in base:
    for nu in (0.25, 0.45, 0.49):
      forms.append(
          dataclasses.replace(
              form, label=f'{form.label}, nu = {nu}', poisson=nu
          )
      )
  return tuple(forms)


_INCOMPRESSIBLE = _incompressible_forms()

_DYNAMICS = (
    Formulation('1D rod, 9 vertices, half-segment offset', 'rod', (9, 1, 1),
                offset=0.5),
    Formulation('1D rod, 17 vertices, half-segment offset', 'rod', (17, 1, 1),
                offset=0.5),
    Formulation('1D rod, 33 vertices, half-segment offset', 'rod', (33, 1, 1),
                offset=0.5),
    Formulation('trilinear 8x1x1', 'trilinear', (17, 3, 3), (8, 1, 1)),
    Formulation('quadratic 2x1x1', 'quadratic', (17, 3, 3), (2, 1, 1)),
    Formulation('quadratic 4x1x1', 'quadratic', (17, 3, 3), (4, 1, 1)),
    Formulation('full 17x3x3', 'full', (17, 3, 3)),
)

# Ring-downs repeated over _RING_TIMESTEPS: the 17-vertex rod and quadratic
# 2x1x1.
_RING_TIMESTEP_FORMS = (_DYNAMICS[1], _DYNAMICS[4])

_STABILITY = (
    Formulation('1D rod, 17 vertices, half-segment offset', 'rod', (17, 1, 1),
                offset=0.5),
    Formulation('trilinear 4x1x1', 'trilinear', (17, 3, 3), (4, 1, 1)),
    Formulation('quadratic 2x1x1', 'quadratic', (17, 3, 3), (2, 1, 1)),
    Formulation('full 17x3x3', 'full', (17, 3, 3)),
)

_ATTACH = (
    Formulation('quadratic 2x1x1, static mount', 'quadratic', (17, 3, 3),
                (2, 1, 1), attach='static'),
    Formulation('quadratic 2x1x1, driven slide mount', 'quadratic',
                (17, 3, 3), (2, 1, 1), attach='slide'),
    Formulation('quadratic 2x1x1, driven hinge mount', 'quadratic',
                (17, 3, 3), (2, 1, 1), attach='hinge'),
    Formulation('quadratic 2x1x1, 4 connects to free tip', 'quadratic',
                (17, 3, 3), (2, 1, 1), attach='connect'),
    Formulation('quadratic 2x1x1, weld to free tip', 'quadratic', (17, 3, 3),
                (2, 1, 1), attach='weld'),
    Formulation('1D rod, 17 vertices, static mount', 'rod', (17, 1, 1),
                offset=0.5, attach='static'),
    Formulation('1D rod, 17 vertices, driven slide mount', 'rod', (17, 1, 1),
                offset=0.5, attach='slide'),
    Formulation('1D rod, 17 vertices, driven hinge mount', 'rod', (17, 1, 1),
                offset=0.5, attach='hinge'),
    Formulation('1D rod, 17 vertices, 2 connects to free tip', 'rod',
                (17, 1, 1), offset=0.5, attach='connect'),
    Formulation('1D rod, 17 vertices, weld to free tip', 'rod', (17, 1, 1),
                offset=0.5, attach='weld'),
    Formulation('trilinear 2x2x2 pad, slide finger', 'trilinear', (5, 5, 3),
                (2, 2, 2), attach='finger'),
)

_STRETCH = (
    Formulation('1D rod, 17 vertices, timeconst 0.02', 'rod', (17, 1, 1),
                timeconst=0.02),
    Formulation('1D rod, 17 vertices, timeconst 0.01', 'rod', (17, 1, 1),
                timeconst=0.01),
    Formulation('1D rod, 17 vertices, timeconst 0.004', 'rod', (17, 1, 1),
                timeconst=0.004),
    Formulation('1D rod, 17 vertices, timeconst 0.002', 'rod', (17, 1, 1),
                timeconst=0.002),
    # Same length, twice the vertices.
    Formulation('1D rod, 33 vertices, timeconst 0.004', 'rod', (33, 1, 1),
                timeconst=0.004),
    Formulation('cloth 13x13, timeconst 0.02', 'full', (13, 13, 1),
                timeconst=0.02),
    Formulation('cloth 13x13, timeconst 0.01', 'full', (13, 13, 1),
                timeconst=0.01),
    Formulation('cloth 13x13, timeconst 0.004', 'full', (13, 13, 1),
                timeconst=0.004),
    Formulation('cloth 13x13, timeconst 0.002', 'full', (13, 13, 1),
                timeconst=0.002),
)

_COLLISION = (
    Formulation('cloth 11x11', 'full', (11, 11, 1)),
    Formulation('cloth 21x21', 'full', (21, 21, 1)),
    Formulation('cloth 41x41', 'full', (41, 41, 1)),
    Formulation('cloth 61x61', 'full', (61, 61, 1)),
    Formulation('trilinear shell 6x6, 41x41 vertices', 'trilinear',
                (41, 41, 1), (6, 6, 1)),
)


def _vec(values) -> str:
  return ' '.join(f'{val:.10g}' for val in values)


def _option(
    solver: str,
    gravity: float,
    timestep: float = _DT,
    integrator: str = 'discrete',
) -> str:
  return (
      f'<option timestep="{timestep}" integrator="{integrator}"'
      f' solver="{solver}" tolerance="{_TOLERANCE}"'
      f' iterations="{_ITERATIONS}" gravity="0 0 {-gravity}"/>'
  )


def _dof_attrs(form: Formulation) -> str:
  if form.dof in ('trilinear', 'quadratic'):
    return f'dof="{form.dof}" cellcount="{_vec(form.cellcount)}"'
  return 'dof="full"'


def _rod_layout(form: Formulation, length: float) -> tuple[float, float]:
  """Returns (segment length, grid center x) for a rod clamped at x = 0."""
  nx = form.count[0]
  segment = length / (nx - 1 - form.offset)
  start = -form.offset * segment
  return segment, start + segment * (nx - 1) / 2


def _cantilever_xml(
    form: Formulation,
    solver: str,
    young: float = _BEAM_E,
    damping: float = _BEAM_DAMPING,
    timestep: float = _DT,
    integrator: str = 'discrete',
) -> str:
  """Returns MJCF for the clamped cantilever in the given formulation."""
  nx, ny, nz = form.count
  nu = _BEAM_NU if form.poisson is None else form.poisson
  if form.dof == 'rod':
    half = _BEAM_H / 2
    segment, center = _rod_layout(form, _BEAM_L)
    flex = f"""
      <flexcomp name="beam" type="grid" dim="1" count="{nx} 1 1"
                spacing="{_vec((segment, 1, 1))}"
                pos="{center} 0 0" mass="{_BEAM_MASS}">
        <pin id="0 1"/>
        <edge equality="true" solref="0.002 1"/>
        <elasticity young="{young}" poisson="{nu}" damping="{damping}"
                    section="box" sectionsize="{half} {half}"/>
        <contact contype="0" conaffinity="0"/>
      </flexcomp>"""
  else:
    order = 2 if form.dof == 'quadratic' else 1
    if form.dof == 'full':
      ey, ez = ny - 1, nz - 1
    else:
      ey, ez = order * form.cellcount[1], order * form.cellcount[2]
    spacing = (_BEAM_L / (nx - 1), _BEAM_H / (ny - 1), _BEAM_H / (nz - 1))
    flex = f"""
      <flexcomp name="beam" type="grid" dim="3" count="{nx} {ny} {nz}"
                spacing="{_vec(spacing)}" pos="{_BEAM_L / 2} 0 0"
                radius="0.001" mass="{_BEAM_MASS}" {_dof_attrs(form)}>
        <pin gridrange="0 0 0 0 {ey} {ez}"/>
        <elasticity young="{young}" poisson="{nu}" damping="{damping}"/>
        <contact contype="0" conaffinity="0" selfcollide="none"/>
      </flexcomp>"""
  return f"""<mujoco>
  {_option(solver, 0.0, timestep, integrator)}
  <worldbody>
    <body name="clamp">{flex}
    </body>
  </worldbody>
</mujoco>"""


def _torsion_xml(form: Formulation, solver: str) -> str:
  """Returns MJCF for the clamped rod driven by an end torque."""
  nx = form.count[0]
  segment, center = _rod_layout(form, _ROD_L)
  return f"""<mujoco>
  {_option(solver, 0.0)}
  <worldbody>
    <body name="clamp">
      <flexcomp name="rod" type="grid" dim="1" count="{nx} 1 1"
                spacing="{_vec((segment, 1, 1))}" pos="{center} 0 0"
                radius="{_ROD_R}" mass="{_ROD_MASS}">
        <pin id="0 1"/>
        <edge equality="true" solref="0.002 1"/>
        <elasticity young="{_ROD_E}" shear="{_ROD_G}" damping="0.005"
                    section="circle"/>
        <contact contype="0" conaffinity="0"/>
      </flexcomp>
    </body>
  </worldbody>
  <actuator>
    <motor name="twist" joint="rod_twist"/>
  </actuator>
</mujoco>"""


def _block_xml(form: Formulation, solver: str) -> str:
  """Returns MJCF for the block squeezed between the floor and a plate."""
  nx = form.count[0]
  radius = 0.002
  half_plate = 0.005
  side = _BLOCK_S
  spacing = side / (nx - 1)
  return f"""<mujoco>
  {_option(solver, _GRAVITY)}
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1" solref="0.004 1"/>
    <flexcomp name="block" type="grid" dim="3" count="{nx} {nx} {nx}"
              spacing="{_vec((spacing,) * 3)}" pos="0 0 {side / 2 + radius}"
              radius="{radius}" mass="{_BLOCK_MASS}" {_dof_attrs(form)}>
      <elasticity young="{_BLOCK_E}" poisson="{_BLOCK_NU}" damping="0.02"/>
      <contact selfcollide="none" solref="0.004 1"/>
    </flexcomp>
    <body name="plate" pos="0 0 {side + 2 * radius + half_plate}">
      <joint name="plate_z" type="slide" axis="0 0 1"/>
      <geom type="box" size="{side} {side} {half_plate}"
            mass="{_PLATE_MASS}" solref="0.004 1"/>
    </body>
  </worldbody>
</mujoco>"""


def _staggered_mesh(
    nx: int, ny: int, size: float
) -> tuple[np.ndarray, list[int]]:
  """Returns points and triangles of a square meshed with staggered rows.

  Even rows hold nx points; odd rows are shifted by half a spacing and closed
  by points on the boundary. Interior triangles are equilateral when
  (ny - 1) / (nx - 1) = 2 / sqrt(3).

  Args:
    nx: Number of points in even rows (odd rows hold nx + 1).
    ny: Number of rows.
    size: Side length of the square, centered on the origin.
  """
  spacing = size / (nx - 1)
  rows = []
  for j in range(ny):
    xs = spacing * np.arange(nx)
    if j % 2:
      xs = np.concatenate(([0.0], xs[:-1] + spacing / 2, [size]))
    rows.append(xs - size / 2)
  start = np.cumsum([0] + [len(xs) for xs in rows])
  elements = []
  for j in range(ny - 1):
    lower, upper = rows[j], rows[j + 1]
    i = k = 0
    # Zip adjacent rows together, advancing past the nearer next point.
    while i < len(lower) - 1 or k < len(upper) - 1:
      if k == len(upper) - 1 or (
          i < len(lower) - 1 and lower[i + 1] <= upper[k + 1]
      ):
        elements += [start[j] + i, start[j] + i + 1, start[j + 1] + k]
        i += 1
      else:
        elements += [start[j] + i, start[j + 1] + k + 1, start[j + 1] + k]
        k += 1
  ys = np.linspace(-size / 2, size / 2, ny)
  points = []
  for j, row in enumerate(rows):
    for xc in row:
      points.append((xc, ys[j], 0.0))
  return np.array(points), elements


def _plate_xml(form: Formulation, solver: str, clamped: bool) -> str:
  """Returns MJCF for the square plate with its outer vertex rings pinned."""
  nx, ny = form.count[0], form.count[1]
  if form.mesh == 'staggered':
    if clamped:
      raise ValueError('staggered plates support simple supports only')
    points, elements = _staggered_mesh(nx, ny, _SHELL_A)
    edge = np.max(np.abs(points[:, :2]), axis=1) > _SHELL_A / 2 - 1e-9
    geometry = (
        f'type="direct" point="{_vec(points.ravel())}"'
        f' element="{_vec(elements)}"'
    )
    pin = f'<pin id="{_vec(np.flatnonzero(edge))}"/>'
    # Every interior point carries the tributary area of one shifted cell.
    mass = _SHELL_RHO_A * _SHELL_A**2 / ((nx - 1) * (ny - 1)) * len(points)
  else:
    spacing = _SHELL_A / (nx - 1 - 2 * form.offset)
    geometry = (
        f'type="grid" count="{nx} {nx} 1" spacing="{_vec((spacing,) * 3)}"'
    )
    last, k = nx - 1, 1 if clamped else 0
    ranges = (0, 0, last, k, 0, last - k, last, last,
              0, 0, k, last, last - k, 0, last, last)
    pin = f'<pin gridrange="{_vec(ranges)}"/>'
    mass = _SHELL_RHO_A * (nx * spacing) ** 2
  return f"""<mujoco>
  {_option(solver, _GRAVITY)}
  <worldbody>
    <body name="clamp">
      <flexcomp name="plate" dim="2" {geometry}
                radius="0.001" mass="{mass:.10g}">
        {pin}
        <elasticity young="{_SHELL_E}" poisson="{_SHELL_NU}"
                    thickness="{_SHELL_T}" elastic2d="{form.elastic2d}"
                    damping="0.005"/>
        <contact contype="0" conaffinity="0" selfcollide="none"/>
      </flexcomp>
    </body>
  </worldbody>
</mujoco>"""


def _membrane_xml(form: Formulation, solver: str) -> str:
  """Returns MJCF for the stretch-only sheet pinned along x = 0."""
  nx, ny = form.count[0], form.count[1]
  spacing = (_SHEET_L / (nx - 1), _SHEET_W / (ny - 1), 0.01)
  return f"""<mujoco>
  {_option(solver, 0.0)}
  <worldbody>
    <body name="clamp">
      <flexcomp name="sheet" type="grid" dim="2" count="{nx} {ny} 1"
                spacing="{_vec(spacing)}" pos="{_SHEET_L / 2} 0 0"
                radius="0.001" mass="{_SHEET_MASS}">
        <pin gridrange="0 0 0 {ny - 1}"/>
        <elasticity young="{_SHEET_E}" poisson="0" thickness="{_SHEET_T}"
                    elastic2d="{form.elastic2d}" damping="0.01"/>
        <contact contype="0" conaffinity="0" selfcollide="none"/>
      </flexcomp>
    </body>
  </worldbody>
</mujoco>"""


def _hertz_force() -> float:
  """Returns the sphere weight that gives the Hertz contact radius _HERTZ_A."""
  e_star = _HERTZ_E / (1 - _HERTZ_NU**2)
  return 4 * e_star * _HERTZ_A**3 / (3 * _HERTZ_R)


def _hertz_xml(form: Formulation, solver: str) -> str:
  """Returns MJCF for the sphere resting on the block pinned at its base."""
  nx, ny, nz = form.count
  spacing = (_HERTZ_S / (nx - 1), _HERTZ_S / (ny - 1), _HERTZ_H / (nz - 1))
  if form.dof == 'full':
    ex, ey = nx - 1, ny - 1
  else:
    order = 2 if form.dof == 'quadratic' else 1
    ex, ey = order * form.cellcount[0], order * form.cellcount[1]
  contact = f'condim="1" solref="{2 * _DT} 1" solimp="{_HERTZ_SOLIMP}"'
  ball_z = _HERTZ_H + _HERTZ_RADIUS + _HERTZ_R
  return f"""<mujoco>
  {_option(solver, _GRAVITY)}
  <worldbody>
    <body name="base">
      <flexcomp name="block" type="grid" dim="3" count="{nx} {ny} {nz}"
                spacing="{_vec(spacing)}" pos="0 0 {_HERTZ_H / 2}"
                radius="{_HERTZ_RADIUS}" mass="{_HERTZ_MASS}"
                {_dof_attrs(form)}>
        <pin gridrange="0 0 0 {ex} {ey} 0"/>
        <elasticity young="{_HERTZ_E}" poisson="{_HERTZ_NU}" damping="0.01"/>
        <contact selfcollide="none" {contact}/>
      </flexcomp>
    </body>
    <body name="ball" pos="0 0 {ball_z}">
      <joint name="ball_z" type="slide" axis="0 0 1" damping="20"/>
      <geom type="sphere" size="{_HERTZ_R}"
            mass="{_hertz_force() / _GRAVITY}" {contact}/>
    </body>
  </worldbody>
</mujoco>"""


def _attach_xml(form: Formulation, solver: str) -> str:
  """Returns MJCF for the beam, rod, or pad in its attachment style."""
  if form.attach == 'finger':
    return _finger_xml(form, solver)
  nx = form.count[0]
  if form.dof == 'rod':
    half = _BEAM_H / 2
    segment, center = _rod_layout(form, _BEAM_L)
    tips = [f'rod_{nx - 2}', f'rod_{nx - 1}']
    welded = tips[-1]
    flex = f"""
      <flexcomp name="rod" type="grid" dim="1" count="{nx} 1 1"
                spacing="{_vec((segment, 1, 1))}"
                pos="{_MOUNT_X + center} 0 0"
                mass="{_BEAM_MASS}">
        <pin id="0 1"/>
        <edge equality="true" solref="0.002 1"/>
        <elasticity young="{_BEAM_E}" poisson="{_BEAM_NU}"
                    damping="{_BEAM_DAMPING}"
                    section="box" sectionsize="{half} {half}"/>
        <contact contype="0" conaffinity="0"/>
      </flexcomp>"""
  else:
    order = 2 if form.dof == 'quadratic' else 1
    ex, ey, ez = (order * cells for cells in form.cellcount)
    # Four non-collinear corners of the tip face, or its center node.
    corners = ((0, 0), (0, ez), (ey, 0), (ey, ez))
    tips = [f'beam_{ex}_{iy}_{iz}' for iy, iz in corners]
    welded = f'beam_{ex}_{ey // 2}_{ez // 2}'
    spacing = (_BEAM_L / (nx - 1), _BEAM_H / (form.count[1] - 1),
               _BEAM_H / (form.count[2] - 1))
    flex = f"""
      <flexcomp name="beam" type="grid" dim="3" count="{_vec(form.count)}"
                spacing="{_vec(spacing)}" pos="{_MOUNT_X + _BEAM_L / 2} 0 0"
                radius="0.001" mass="{_BEAM_MASS}" {_dof_attrs(form)}>
        <pin gridrange="0 0 0 0 {ey} {ez}"/>
        <elasticity young="{_BEAM_E}" poisson="{_BEAM_NU}"
                    damping="{_BEAM_DAMPING}"/>
        <contact contype="0" conaffinity="0" selfcollide="none"/>
      </flexcomp>"""
  joint = actuator = tip = equality = ''
  if form.attach in ('slide', 'hinge'):
    kind, axis, kp = (('slide', '0 0 1', _SLIDE_KP) if form.attach == 'slide'
                      else ('hinge', '0 1 0', _HINGE_KP))
    joint = f'<joint name="drive" type="{kind}" axis="{axis}"/>'
    actuator = f"""
  <actuator>
    <position name="drive" joint="drive" kp="{kp}" dampratio="1"/>
  </actuator>"""
  elif form.attach in ('connect', 'weld'):
    tip = f"""
    <body name="tip" pos="{_BEAM_L} 0 0.5">
      <freejoint/>
      <geom type="box" size="0.01 0.03 0.03" mass="{_TIP_MASS}"
            contype="0" conaffinity="0"/>
    </body>"""
    if form.attach == 'connect':
      lines = [f'<connect body1="{body}" body2="tip" anchor="0 0 0"'
               f' solref="{_ATTACH_SOLREF}"/>' for body in tips]
    else:
      lines = [f'<weld body1="{welded}" body2="tip"'
               f' solref="{_ATTACH_SOLREF}"/>']
    equality = '\n  <equality>\n    ' + '\n    '.join(lines)
    equality += '\n  </equality>'
  # The mount (and hinge axis) sits _MOUNT_X behind the clamp face at x = 0.
  return f"""<mujoco>
  {_option(solver, _GRAVITY)}
  <worldbody>
    <body name="mount" pos="{-_MOUNT_X} 0 0.5">
      {joint}
      <geom type="box" size="{_MOUNT_X} 0.04 0.04" mass="0.5"
            contype="0" conaffinity="0"/>{flex}
    </body>{tip}
  </worldbody>{equality}{actuator}
</mujoco>"""


def _finger_xml(form: Formulation, solver: str) -> str:
  """Returns MJCF for a pad on a slide-jointed finger above a table."""
  nx, ny, nz = form.count
  spacing = (_PAD_W / (nx - 1), _PAD_W / (ny - 1), _PAD_T / (nz - 1))
  ex, ey, ez = form.cellcount
  radius = 0.001
  # The pad spans z = radius + gap to that plus _PAD_T above the table; the
  # finger's origin sits _MOUNT_X / 2 above it and slides down (axis -z), so
  # gravity loads the press direction only and friction stays unloaded.
  back = _MOUNT_X / 2
  top = radius + _PAD_GAP + _PAD_T
  return f"""<mujoco>
  {_option(solver, _GRAVITY)}
  <worldbody>
    <geom name="table" type="plane" size="0.1 0.1 0.01" solref="0.004 1"/>
    <body name="mount" pos="0 0 {top + back}">
      <joint name="drive" type="slide" axis="0 0 -1"/>
      <geom type="box" size="0.03 0.03 {back / 2}" pos="0 0 {-back / 2}"
            mass="0.1" contype="0" conaffinity="0"/>
      <flexcomp name="pad" type="grid" dim="3" count="{nx} {ny} {nz}"
                spacing="{_vec(spacing)}" pos="0 0 {-back - _PAD_T / 2}"
                radius="{radius}" mass="{_PAD_MASS}" {_dof_attrs(form)}>
        <pin gridrange="0 0 {ez} {ex} {ey} {ez}"/>
        <elasticity young="{_PAD_E}" poisson="0" damping="0.01"/>
        <contact selfcollide="none" solref="0.004 1"/>
      </flexcomp>
    </body>
  </worldbody>
  <actuator>
    <position name="drive" joint="drive" kp="{_FINGER_KP}" dampratio="1"/>
  </actuator>
</mujoco>"""


def _stretch_xml(form: Formulation, solver: str) -> str:
  """Returns MJCF for a rod or cloth hanging from its top vertices."""
  nx, ny = form.count[0], form.count[1]
  solref = f'{form.timeconst} 1'
  if form.dof == 'rod':
    # Rotating the grid's x axis onto -z puts vertex 0 on top.
    flex = f"""
      <flexcomp name="rod" type="grid" dim="1" count="{nx} 1 1"
                spacing="{_vec((_STRETCH_L / (nx - 1), 1, 1))}"
                euler="0 90 0" radius="{_ROD_R}" mass="{_STRETCH_MASS}">
        <pin id="0 1"/>
        <edge equality="true" solref="{solref}"/>
        <elasticity young="{_ROD_E}" shear="{_ROD_G}" damping="0.005"
                    section="circle"/>
        <contact contype="0" conaffinity="0"/>
      </flexcomp>"""
  else:
    # Rotating the grid's y axis onto z makes row ny - 1 the top edge.
    spacing = _STRETCH_L / (nx - 1)
    flex = f"""
      <flexcomp name="cloth" type="grid" dim="2" count="{nx} {ny} 1"
                spacing="{_vec((spacing,) * 3)}" euler="90 0 0"
                radius="0.002" mass="{_STRETCH_MASS}">
        <pin gridrange="0 {ny - 1} {nx - 1} {ny - 1}"/>
        <edge equality="true" solref="{solref}"/>
        <elasticity young="5e4" poisson="0.35" thickness="1.5e-3"
                    elastic2d="bend" damping="0.01"/>
        <contact contype="0" conaffinity="0" selfcollide="none"/>
      </flexcomp>"""
  return f"""<mujoco>
  {_option(solver, _GRAVITY)}
  <worldbody>
    <body name="mount" pos="0 0 1">{flex}
    </body>
  </worldbody>
</mujoco>"""


def _collision_xml(form: Formulation, solver: str) -> str:
  """Returns MJCF for a cloth dropped onto a sphere resting on the floor."""
  nx = form.count[0]
  spacing = _CLOTH_L / (nx - 1)
  # Interpolated shells bend explicitly and warn on damping. Their lattice
  # spans the grid's axis-aligned bounding box, which a flat grid leaves with
  # zero thickness ("flex grid rotation R0 is not orthonormal"), so tilt it.
  extra = 'euler="10 0 0"'
  damping = ''
  if form.dof == 'full':
    extra = ''
    damping = 'damping="0.01"'
  return f"""<mujoco>
  {_option(solver, _GRAVITY)}
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1" solref="0.004 1"/>
    <geom name="sphere" type="sphere" size="{_SPHERE_R}"
          pos="0 0 {_SPHERE_R}" solref="0.004 1"/>
    <flexcomp name="cloth" type="grid" dim="2" count="{nx} {nx} 1"
              spacing="{_vec((spacing,) * 3)}" {extra}
              pos="0 0 {2 * _SPHERE_R + 0.05}" radius="{_CLOTH_R}"
              mass="{_CLOTH_MASS}" {_dof_attrs(form)}>
      <elasticity young="5e4" poisson="0.35" thickness="1.5e-3"
                  elastic2d="both" {damping}/>
      <contact selfcollide="auto" solref="0.004 1"/>
    </flexcomp>
  </worldbody>
</mujoco>"""


def _flex_bodies(model: mujoco.MjModel, flex_id: int) -> np.ndarray:
  """Returns the body ids that carry the DOFs of a flex."""
  if model.flex_interp[flex_id]:
    adr = model.flex_nodeadr[flex_id]
    return model.flex_nodebodyid[adr : adr + model.flex_nodenum[flex_id]]
  adr = model.flex_vertadr[flex_id]
  return model.flex_vertbodyid[adr : adr + model.flex_vertnum[flex_id]]


def _tip_bodies(model: mujoco.MjModel, data: mujoco.MjData) -> list[int]:
  """Returns the cantilever's DOF-carrying bodies at its free end."""
  clamp = model.body('clamp').id
  bodies = {int(body) for body in _flex_bodies(model, 0) if body != clamp}
  x_tip = max(data.xpos[body, 0] for body in bodies)
  return sorted(body for body in bodies if data.xpos[body, 0] > x_tip - 1e-6)


def _islands(data: mujoco.MjData) -> slice:
  return slice(0, max(1, data.nisland))


def _step_grad(
    model: mujoco.MjModel, data: mujoco.MjData
) -> tuple[float, bool]:
  """Returns (max final gradient, all islands < tolerance) for a step."""
  grad = 0.0
  converged = True
  for isl in range(max(1, int(data.nisland))):
    niter = int(data.solver_niter[isl])
    if niter > 0:
      isl_grad = float(data.solver.gradient[isl * mujoco.mjNSOLVER + niter - 1])
      grad = max(grad, isl_grad)
      if isl_grad >= model.opt.tolerance:
        converged = False
  return grad, converged


def _step_timed(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    nstep: int,
    callback: Callable[[mujoco.MjData], None] | None = None,
) -> tuple[float, float, int, float, float]:
  """Returns (ms/step, mean iters, max nnz, median grad, conv %) over nstep."""
  iterations = 0
  nnz = 0
  elapsed = 0.0
  grads = []
  converged = 0
  constrained = 0
  for _ in range(nstep):
    if callback is not None:
      callback(data)
    start = time.perf_counter()
    mujoco.mj_step(model, data)
    elapsed += time.perf_counter() - start
    nisl = _islands(data)
    niter = int(data.solver_niter[nisl].sum())
    iterations += niter
    nnz = max(nnz, int(data.solver_nnz[nisl].sum()))
    if data.nefc > 0:
      constrained += 1
      grad, ok = _step_grad(model, data)
      if niter > 0:
        grads.append(grad)
      if ok:
        converged += 1
  med_grad = (
      float(np.median(grads)) if grads
      else (0.0 if constrained else float('nan'))
  )
  conv_pct = 100.0 * converged / constrained if constrained else float('nan')
  return 1e3 * elapsed / nstep, iterations / nstep, nnz, med_grad, conv_pct


def _settle(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    min_time: float = 1.5,
    max_time: float = 10.0,
    vel_tol: float = 1e-5,
) -> float:
  """Steps until the largest DOF speed drops below vel_tol; returns it."""
  speed = float(np.max(np.abs(data.qvel)))
  while data.time < max_time:
    mujoco.mj_step(model, data)
    speed = float(np.max(np.abs(data.qvel)))
    if data.time > min_time and speed < vel_tol:
      break
  return speed


def _check(data: mujoco.MjData) -> None:
  if data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number:
    raise RuntimeError('simulation diverged (mjWARN_BADQACC)')


def _compile_form(xml: str, form: Formulation) -> mujoco.MjModel:
  """Compiles XML, applying MjSpec-only fields such as elastic3d if set."""
  if not form.elastic3d:
    return mujoco.MjModel.from_xml_string(xml)
  spec = mujoco.MjSpec.from_string(xml)
  for flex in spec.flexes:
    flex.elastic3d = form.elastic3d
  return spec.compile()


def _run_cantilever(form: Formulation, solver: str) -> dict[str, float]:
  """Applies the tip load, times the transient, and measures deflection."""
  model = _compile_form(_cantilever_xml(form, solver), form)
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  verts0 = data.flexvert_xpos.copy()
  tip = verts0[:, 0] > verts0[:, 0].max() - 1e-6
  loaded = _tip_bodies(model, data)
  data.xfrc_applied[loaded, 2] = -_TIP_LOAD / len(loaded)
  ms, iters, nnz, grad, conv = _step_timed(model, data, _TIMING_STEPS.value)
  speed = _settle(model, data)
  _check(data)
  sag = -float(np.mean(data.flexvert_xpos[tip, 2] - verts0[tip, 2]))
  inertia = _BEAM_H**4 / 12
  reference = _TIP_LOAD * _BEAM_L**3 / (3 * _BEAM_E * inertia)
  if _VERBOSE.value:
    print(
        f'  {form.label} {solver}: sag {1e3 * sag:.4f} mm (ref'
        f' {1e3 * reference:.4f}), inertia warnings'
        f' {data.warning[mujoco.mjtWarning.mjWARN_INERTIA].number},'
        f' t {data.time:.2f}, speed {speed:.1e}',
        file=sys.stderr,
        flush=True,
    )
  return dict(
      nv=model.nv,
      nnz=nnz,
      ms=ms,
      iterations=iters,
      grad=grad,
      conv=conv,
      error=100 * (sag / reference - 1),
      speed=speed,
  )


def _run_torsion(form: Formulation, solver: str) -> dict[str, float]:
  """Applies the end torque and compares the static twist with T L / (G J)."""
  model = mujoco.MjModel.from_xml_string(_torsion_xml(form, solver))
  data = mujoco.MjData(model)
  data.ctrl[0] = _TORQUE
  ms, iters, nnz, grad, conv = _step_timed(model, data, _TIMING_STEPS.value)
  speed = _settle(model, data)
  _check(data)
  twist = float(data.joint('rod_twist').qpos[0])
  reference = _TORQUE * _ROD_L / (_ROD_G * np.pi * _ROD_R**4 / 2)
  return dict(
      nv=model.nv,
      nnz=nnz,
      ms=ms,
      iterations=iters,
      grad=grad,
      conv=conv,
      error=100 * (twist / reference - 1),
      speed=speed,
  )


def _run_block(form: Formulation, solver: str) -> dict[str, float]:
  """Times the loaded transient and measures the steady compression."""
  model = _compile_form(_block_xml(form, solver), form)
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  z0 = data.flexvert_xpos[:, 2].copy()
  top = z0 > z0.max() - 1e-6
  bottom = z0 < z0.min() + 1e-6
  height0 = z0[top].mean() - z0[bottom].mean()
  ms, iters, nnz, grad, conv = _step_timed(model, data, _TIMING_STEPS.value)
  # Contact keeps the block micro-vibrating, so settle for a bounded time.
  speed = _settle(model, data, min_time=1.0, max_time=3.0, vel_tol=1e-4)
  _check(data)
  z_pos = data.flexvert_xpos[:, 2]
  squeeze = height0 - (z_pos[top].mean() - z_pos[bottom].mean())
  stress = _PLATE_MASS * _GRAVITY / _BLOCK_S**2
  stress += _BLOCK_MASS / _BLOCK_S**3 * _GRAVITY * _BLOCK_S / 2
  reference = stress * _BLOCK_S / _BLOCK_E
  if _VERBOSE.value:
    geoms = data.contact.geom[: data.ncon]
    plate = model.body('plate')
    print(
        f'  {form.label} {solver}: squeeze {1e3 * squeeze:.3f} mm (ref'
        f' {1e3 * reference:.3f}), plate drop'
        f' {1e3 * (model.body_pos[plate.id][2] - data.xpos[plate.id][2]):.3f}'
        f' mm, z in [{z_pos.min():.4f}, {z_pos.max():.4f}], contacts floor'
        f' {int(np.sum(geoms == 0))} plate {int(np.sum(geoms == 1))}',
        file=sys.stderr,
        flush=True,
    )
  return dict(
      nv=model.nv,
      ncon=data.ncon,
      nnz=nnz,
      ms=ms,
      iterations=iters,
      grad=grad,
      conv=conv,
      error=100 * (squeeze / reference - 1),
      speed=speed,
  )


def _run_plate(
    form: Formulation, solver: str, clamped: bool
) -> dict[str, float]:
  """Times the sag under gravity and compares the center deflection."""
  model = mujoco.MjModel.from_xml_string(_plate_xml(form, solver, clamped))
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  verts0 = data.flexvert_xpos.copy()
  center = int(np.argmin(np.sum(verts0[:, :2] ** 2, axis=1)))
  ms, iters, nnz, grad, conv = _step_timed(model, data, _TIMING_STEPS.value)
  speed = _settle(model, data, min_time=1.0, max_time=3.0, vel_tol=1e-6)
  _check(data)
  sag = float(verts0[center, 2] - data.flexvert_xpos[center, 2])
  rigidity = _SHELL_E * _SHELL_T**3 / (12 * (1 - _SHELL_NU**2))
  coef = _SHELL_CLAMPED_COEF if clamped else _SHELL_SIMPLE_COEF
  reference = coef * _SHELL_RHO_A * _GRAVITY * _SHELL_A**4 / rigidity
  if _VERBOSE.value:
    drift = np.abs(data.flexvert_xpos[:, :2] - verts0[:, :2]).max()
    print(
        f'  {form.label} {solver}: sag {1e3 * sag:.4f} mm (ref'
        f' {1e3 * reference:.4f}), w/t {sag / _SHELL_T:.3f}, in-plane drift'
        f' {1e3 * drift:.2e} mm, speed {speed:.1e}, t {data.time:.2f}',
        file=sys.stderr,
        flush=True,
    )
  return dict(
      nv=model.nv,
      nnz=nnz,
      ms=ms,
      iterations=iters,
      grad=grad,
      conv=conv,
      error=100 * (sag / reference - 1),
      speed=speed,
  )


def _run_membrane(form: Formulation, solver: str) -> dict[str, float]:
  """Pulls the free edge and compares the elongation with F L / (E t W)."""
  model = mujoco.MjModel.from_xml_string(_membrane_xml(form, solver))
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  verts0 = data.flexvert_xpos.copy()
  edge = np.flatnonzero(verts0[:, 0] > verts0[:, 0].max() - 1e-6)
  # Consistent vertex forces of a uniform traction: half at the two corners.
  ys = verts0[edge, 1]
  weights = np.where((ys < ys.min() + 1e-6) | (ys > ys.max() - 1e-6), 0.5, 1.0)
  weights /= weights.sum()
  force = form.load * _SHEET_FORCE
  data.xfrc_applied[model.flex_vertbodyid[edge], 0] = force * weights
  ms, iters, nnz, grad, conv = _step_timed(model, data, _TIMING_STEPS.value)
  speed = _settle(model, data, min_time=1.0, max_time=3.0, vel_tol=1e-6)
  _check(data)
  stretch = float(weights @ (data.flexvert_xpos[edge, 0] - verts0[edge, 0]))
  reference = force * _SHEET_L / (_SHEET_E * _SHEET_T * _SHEET_W)
  if _VERBOSE.value:
    spread = np.ptp(data.flexvert_xpos[edge, 0] - verts0[edge, 0])
    print(
        f'  {form.label} {solver}: elongation {1e3 * stretch:.5f} mm (ref'
        f' {1e3 * reference:.5f}), edge spread {1e3 * spread:.2e} mm,'
        f' max |z| {np.abs(data.flexvert_xpos[:, 2]).max():.1e}, speed'
        f' {speed:.1e}, t {data.time:.2f}',
        file=sys.stderr,
        flush=True,
    )
  return dict(
      nv=model.nv,
      nnz=nnz,
      ms=ms,
      iterations=iters,
      grad=grad,
      conv=conv,
      error=100 * (stretch / reference - 1),
      speed=speed,
  )


def _axis_profile(xy: np.ndarray, dz: np.ndarray, radius: float) -> float:
  """Interpolates dz at a radius along the four half-axes; returns the mean."""
  values = []
  for axis in (0, 1):
    on_axis = np.abs(xy[:, 1 - axis]) < 1e-6
    for sign in (1, -1):
      rad = sign * xy[on_axis, axis]
      keep = rad > -1e-6
      order = np.argsort(rad[keep])
      values.append(
          np.interp(radius, rad[keep][order], dz[on_axis][keep][order])
      )
  return float(np.mean(values))


def _hertz_surface(rad: float, radius_a: float, delta: float) -> float:
  """Returns the half-space surface deflection at r >= a (Johnson eq. 3.42b)."""
  ratio = rad / radius_a
  return (
      delta
      / np.pi
      * ((2 - ratio**2) * np.arcsin(1 / ratio) + np.sqrt(ratio**2 - 1))
  )


def _hertz_spacing(form: Formulation) -> float:
  """Returns the spacing of the DOF nodes (vertices for dof='full')."""
  if form.dof == 'full':
    return _HERTZ_S / (form.count[0] - 1)
  order = 2 if form.dof == 'quadratic' else 1
  return _HERTZ_S / (order * form.cellcount[0])


def _run_hertz(form: Formulation, solver: str) -> dict[str, float]:
  """Times the loaded transient and measures the Hertz indentation."""
  model = mujoco.MjModel.from_xml_string(_hertz_xml(form, solver))
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  verts0 = data.flexvert_xpos.copy()
  top = verts0[:, 2] > verts0[:, 2].max() - 1e-6
  xy = verts0[top, :2]
  rim = np.max(np.abs(xy), axis=1) > _HERTZ_S / 2 - 1e-6
  center = np.hypot(xy[:, 0], xy[:, 1]) < 1e-6
  ball = model.body('ball').id
  ball_z0 = float(data.xpos[ball][2])
  ms, iters, nnz, grad, conv = _step_timed(model, data, _TIMING_STEPS.value)
  speed = _settle(model, data, min_time=1.0, max_time=2.0, vel_tol=1e-4)
  _check(data)
  dz = data.flexvert_xpos[top, 2] - verts0[top, 2]
  drop = ball_z0 - float(data.xpos[ball][2])
  # Indentation: the sphere's drop minus the drop of the top surface at
  # r = 1.5 a, compared with the same quantity on a half-space. The ring
  # cancels the bulk compression and self-weight of the block and most of the
  # pinned-base stiffening (a 2x larger block reads <= 2% softer), unlike the
  # block rim at 3 a. The sphere avoids the vertex under its center, which
  # dips up to 0.19 mm below the sphere into a point-loaded apex. Contact
  # penetration (< 0.01 mm with the near-hard solimp) is not subtracted.
  ring = -_axis_profile(xy, dz, _HERTZ_RING)
  indentation = drop - ring
  e_star = _HERTZ_E / (1 - _HERTZ_NU**2)
  force = float(model.body('ball').mass[0]) * _GRAVITY
  delta = (9 * force**2 / (16 * e_star**2 * _HERTZ_R)) ** (1 / 3)
  radius_a = np.sqrt(_HERTZ_R * delta)
  reference = delta - _hertz_surface(_HERTZ_RING, radius_a, delta)
  if _VERBOSE.value:
    dist = data.contact.dist[: data.ncon]
    far = drop + float(np.mean(dz[rim]))
    print(
        f'  {form.label} {solver}: dent {1e3 * indentation:.4f} mm (ref'
        f' {1e3 * reference:.4f}), ball drop {1e3 * drop:.4f} mm, center'
        f' vertex drop {-1e3 * dz[center][0]:.4f} mm, ring drop'
        f' {1e3 * ring:.4f} mm, rim dent {1e3 * far:.4f} mm'
        f' ({100 * (far / delta - 1):+.1f}% vs Hertz {1e3 * delta:.4f}), max'
        f' penetration {-1e3 * min(dist.min(initial=0), 0):.4f} mm, ncon'
        f' {data.ncon}, t {data.time:.2f}, speed {speed:.1e}',
        file=sys.stderr,
        flush=True,
    )
  return {
      'nv': model.nv,
      'ncon': data.ncon,
      'h / a': round(_hertz_spacing(form) / radius_a, 3),
      'nnz': nnz,
      'ms': ms,
      'iterations': iters,
      'grad': grad,
      'conv': conv,
      'error': 100 * (indentation / reference - 1),
      'speed': speed,
  }


def _ringdown(signal: np.ndarray, timestep: float) -> tuple[float, float]:
  """Returns (damped angular frequency, decay rate) of a free oscillation.

  The frequency fits the interpolated zero-crossing times and the decay rate
  fits the log amplitudes of the peaks between crossings, skipping peaks below
  0.1% of the first one.

  Args:
    signal: Samples oscillating about zero.
    timestep: Sampling interval in seconds.
  """
  cross = np.flatnonzero(np.signbit(signal[:-1]) != np.signbit(signal[1:]))
  if len(cross) < 4:
    return np.nan, np.nan
  frac = signal[cross] / (signal[cross] - signal[cross + 1])
  times = (cross + frac) * timestep
  half_period = np.polyfit(np.arange(len(cross)), times, 1)[0]
  peaks = np.array([
      i + 1 + np.argmax(np.abs(signal[i + 1 : j + 1]))
      for i, j in zip(cross[:-1], cross[1:])
  ])
  amplitude = np.abs(signal[peaks])
  keep = amplitude > 1e-3 * amplitude[0]
  if np.count_nonzero(keep) < 2:
    return np.pi / half_period, np.nan
  slope = np.polyfit(peaks[keep] * timestep, np.log(amplitude[keep]), 1)[0]
  return np.pi / half_period, -slope


def _implicit_decay(omega: float, timestep: float, damping: float) -> float:
  """Returns the decay rate of a linear mode under the discrete integrator.

  The step solves (M + h D + h^2 K) dv = h (f - h K v), backward Euler for
  linear forces. With D = damping * K it scales a mode of angular frequency
  omega by |z| = (1 + h omega^2 (damping + h))^(-1/2) per step: the implicit
  step damps like an extra damping = h for h omega << 1.

  Args:
    omega: Undamped angular frequency of the mode in rad/s.
    timestep: Timestep h in seconds.
    damping: Stiffness-proportional damping in seconds.
  """
  return np.log1p(timestep * omega**2 * (damping + timestep)) / (2 * timestep)


def _first_bending_omega() -> float:
  """Returns the Euler-Bernoulli first bending angular frequency in rad/s."""
  stiffness = _BEAM_E * _BEAM_H**4 / 12
  return _BETA_L**2 * np.sqrt(stiffness / (_BEAM_MASS * _BEAM_L**3))


def _ring_down(
    form: Formulation, solver: str, timestep: float, damping: float
) -> dict[str, float]:
  """Releases the statically tip-loaded cantilever and records the tip."""
  xml = _cantilever_xml(form, solver, timestep=timestep)
  model = mujoco.MjModel.from_xml_string(xml)
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  verts0 = data.flexvert_xpos.copy()
  tip = verts0[:, 0] > verts0[:, 0].max() - 1e-6
  loaded = _tip_bodies(model, data)
  data.xfrc_applied[loaded, 2] = -_TIP_LOAD / len(loaded)
  speed = _settle(model, data, min_time=0.5)
  _check(data)
  # Release from rest with the ring-down damping. Lowering flex_damping at
  # runtime is safe; raising it from a compile-time 0 is not: a dof="full"
  # flex compiled without damping gets no edge-Jacobian sparsity for a damper.
  model.flex_damping[0] = damping
  data.xfrc_applied[:] = 0
  data.qvel[:] = 0
  nstep = round(_RING_TIME / timestep)
  sag = np.empty(nstep)
  stats = np.empty((nstep, 5))
  for i in range(nstep):
    # mj_step leaves flexvert_xpos at the start of the step: sag[i] is t = i*h.
    stats[i] = _step_timed(model, data, 1)
    sag[i] = np.mean(data.flexvert_xpos[tip, 2] - verts0[tip, 2])
  _check(data)
  omega_d, decay = _ringdown(sag, timestep)
  return dict(
      nv=model.nv,
      nnz=int(stats[:, 2].max()),
      ms=float(stats[:, 0].mean()),
      iterations=float(stats[:, 1].mean()),
      grad=float('nan'),
      conv=float('nan'),
      omega=float(np.hypot(omega_d, decay)),
      decay=float(decay),
      static=-float(sag[0]),
      speed=speed,
  )


def _run_dynamics(form: Formulation, solver: str) -> dict[str, float | str]:
  """Rings down at the default timestep; compares f1 and the decay rate."""
  result = _ring_down(form, solver, _DT, _RING_DAMPING)
  omega, decay = result['omega'], result['decay']
  viscous = _RING_DAMPING * omega**2 / 2
  implicit = _implicit_decay(omega, _DT, _RING_DAMPING)
  inertia = _BEAM_H**4 / 12
  reference = _TIP_LOAD * _BEAM_L**3 / (3 * _BEAM_E * inertia)
  result.update({
      'Static error (%)': f"{100 * (result['static'] / reference - 1):+.1f}",
      'f (Hz)': f'{omega / (2 * np.pi):.3f}',
      'Decay (1/s)': f'{decay:.2f}',
      'Decay vs. c w^2/2 (%)': f'{100 * (decay / viscous - 1):+.1f}',
      'Decay vs. implicit step (%)': f'{100 * (decay / implicit - 1):+.1f}',
      'error': 100 * (omega / _first_bending_omega() - 1),
  })
  return result


def _stable(
    form: Formulation,
    young: float,
    timestep: float,
    integrator: str,
    damping: float,
) -> bool:
  """Returns whether the tip-loaded cantilever stays stable for _STAB_TIME."""
  xml = _cantilever_xml(
      form, 'Newton', young=young, damping=damping, timestep=timestep,
      integrator=integrator,
  )
  model = mujoco.MjModel.from_xml_string(xml)
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  loaded = _tip_bodies(model, data)
  data.xfrc_applied[loaded, 2] = -_TIP_LOAD / len(loaded)
  for _ in range(round(_STAB_TIME / timestep)):
    mujoco.mj_step(model, data)
    # A bad value auto-resets mjData to qpos0, so check the warning counters.
    if (
        any(data.warning[warn].number for warn in _BAD_STATE)
        or not np.all(np.isfinite(data.qvel))
        or np.max(np.abs(data.qvel)) >= _STAB_SPEED
    ):
      return False
  return True


def _largest_stable(grid: tuple[float, ...], stable: list[float]) -> str:
  """Formats the largest stable timestep, flagging smaller failing ones."""
  if not stable:
    return f'< {min(grid):g}'
  best = max(stable)
  text = f'>= {best:g}' if best == max(grid) else f'{best:g}'
  fails = [f'{dt:g}' for dt in grid if dt < best and dt not in stable]
  if fails:
    text += f' (fails {", ".join(fails)})'
  return text


def _in_frame(
    data: mujoco.MjData, body: int, points: np.ndarray
) -> np.ndarray:
  """Returns world points in the frame of the given body."""
  return (points - data.xpos[body]) @ data.xmat[body].reshape(3, 3)


def _attach_error(model: mujoco.MjModel, data: mujoco.MjData) -> float:
  """Returns the largest <connect>/<weld> anchor separation (m)."""
  rows = np.flatnonzero(
      data.efc_type[: data.nefc] == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
  )
  attachments = (int(mujoco.mjtEq.mjEQ_CONNECT), int(mujoco.mjtEq.mjEQ_WELD))
  error = 0.0
  for eq in range(model.neq):
    if model.eq_type[eq] in attachments:
      # The first three rows of both are the world-frame anchor offset.
      offset = data.efc_pos[rows[data.efc_id[rows] == eq][:3]]
      error = max(error, float(np.linalg.norm(offset)))
  return error


def _drive_target(style: str, time_s: float) -> float:
  """Returns the position-servo target of a driven mount at time_s."""
  phase = 2 * np.pi * _DRIVE_HZ * time_s
  if style == 'slide':
    return _SLIDE_AMP * np.sin(phase)
  if style == 'hinge':
    return _HINGE_AMP * np.sin(phase)
  return _PAD_PUSH * (1 - np.cos(phase)) / 2


def _sag_reference(
    model: mujoco.MjModel, data: mujoco.MjData, mount: int
) -> float:
  """Returns the Euler-Bernoulli tip sag under the lumped gravity loads."""
  bodies = [int(body) for body in set(_flex_bodies(model, 0)) if body != mount]
  tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'tip')
  if tip >= 0:
    bodies.append(tip)
  # Distance from the clamp face.
  xpos = _in_frame(data, mount, data.xpos[bodies])[:, 0] - _MOUNT_X
  load = model.body_mass[bodies] * _GRAVITY
  stiffness = _BEAM_E * _BEAM_H**4 / 12
  return float(np.sum(load * xpos**2 * (3 * _BEAM_L - xpos)) / (6 * stiffness))


def _drive_step(
    model: mujoco.MjModel,
    form: Formulation,
    peak: dict[str, int | float],
    data: mujoco.MjData,
) -> None:
  """Records peak constraint counts and sets the servo target before a step."""
  peak['nefc'] = max(int(peak['nefc']), int(data.nefc))
  peak['attach'] = max(float(peak['attach']), _attach_error(model, data))
  if model.nu:
    data.ctrl[0] = _drive_target(form.attach, data.time)


def _run_attach(form: Formulation, solver: str) -> dict[str, float]:
  """Drives or loads an attached flex, then measures its settled deflection."""
  model = mujoco.MjModel.from_xml_string(_attach_xml(form, solver))
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)
  mount = model.body('mount').id
  finger = form.attach == 'finger'
  verts0 = _in_frame(data, mount, data.flexvert_xpos)
  # The beam tip sags along -z; the pad's bottom face rises into it.
  if finger:
    loaded = verts0[:, 2] < verts0[:, 2].min() + 1e-6
  else:
    loaded = verts0[:, 0] > verts0[:, 0].max() - 1e-6
  reference = 0.0 if finger else _sag_reference(model, data, mount)
  peak: dict[str, int | float] = dict(nefc=0, attach=0.0)
  drive = functools.partial(_drive_step, model, form, peak)
  ms, iters, nnz, grad, conv = _step_timed(
      model, data, _TIMING_STEPS.value, drive
  )
  if finger:
    # Hold the squeeze; contact micro-vibrates, so settle for a bounded time.
    data.ctrl[0] = _PAD_PUSH
    speed = _settle(model, data, data.time + 1, data.time + 3, vel_tol=1e-4)
  else:
    data.ctrl[:] = 0.0
    speed = _settle(model, data)
  _check(data)
  mujoco.mj_forward(model, data)
  verts = _in_frame(data, mount, data.flexvert_xpos)
  rise = float(np.mean(verts[loaded, 2] - verts0[loaded, 2]))
  deflection = rise if finger else -rise
  servo = float('nan')
  if model.nu:
    # At rest the servo must hold gravity and the contact reactions on its
    # joint; the flex's internal forces do no work under the mount's motion.
    dof = model.jnt_dofadr[model.joint('drive').id]
    load = data.qfrc_bias[dof] - data.qfrc_constraint[dof]
    servo = 100 * (data.qfrc_actuator[dof] / load - 1)
    if finger:
      # Uniaxial compression under the table reaction, less half the pad's
      # weight (the axial force falls linearly to the pinned top face).
      force = -data.qfrc_constraint[dof] - _PAD_MASS * _GRAVITY / 2
      reference = force * _PAD_T / (_PAD_E * _PAD_W**2)
  if _VERBOSE.value:
    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'tip')
    tilt = 2 * np.arccos(min(1.0, abs(data.xquat[tip, 0]))) if tip >= 0 else 0
    fastest = int(np.argmax(np.abs(data.qvel)))
    print(
        f'  {form.label} {solver}: deflection {1e3 * deflection:.3f} mm'
        f' (ref {1e3 * reference:.3f}), servo load error {servo:+.1f} %,'
        f' mount at {data.xpos[mount]}, tip tilt {np.degrees(tilt):.2f}'
        f' deg, ncon {data.ncon}, speed {speed:.2e} at dof {fastest} (body'
        f' {model.body(model.dof_bodyid[fastest]).name}), max attach error'
        f' {1e3 * float(peak["attach"]):.4f} mm, warnings'
        f' {data.warning.number}',
        file=sys.stderr,
        flush=True,
    )
  equalities = form.attach in ('connect', 'weld')
  return dict(
      nv=model.nv,
      nnz=nnz,
      ms=ms,
      iterations=iters,
      grad=grad,
      conv=conv,
      nefc=int(peak['nefc']),
      deflection=deflection,
      reference=reference,
      error=100 * (deflection / reference - 1),
      servo=servo,
      attach=float(peak['attach']) if equalities else float('nan'),
      warnings=int(np.sum(data.warning.number)),
      speed=speed,
  )


def _run_stretch(form: Formulation, solver: str) -> dict[str, float]:
  """Hangs the rod or cloth and measures the settled edge strain."""
  model = mujoco.MjModel.from_xml_string(_stretch_xml(form, solver))
  data = mujoco.MjData(model)
  ms, iters, nnz, grad, conv = _step_timed(model, data, _TIMING_STEPS.value)
  speed = _settle(model, data)
  _check(data)
  mujoco.mj_forward(model, data)
  free = model.flexedge_rigid == 0
  strain = data.flexedge_length[free] / model.flexedge_length0[free] - 1
  if _VERBOSE.value:
    print(
        f'  {form.label} {solver}: strain in [{100 * strain.min():.4f},'
        f' {100 * strain.max():.4f}] %, {int(np.sum(free))} edges, speed'
        f' {speed:.2e}, warnings {data.warning.number}',
        file=sys.stderr,
        flush=True,
    )
  return dict(
      nv=model.nv,
      nnz=nnz,
      ms=ms,
      iterations=iters,
      grad=grad,
      conv=conv,
      max=100 * float(strain.max()),
      mean=100 * float(strain.mean()),
      speed=speed,
  )


@functools.lru_cache(maxsize=1)
def _draped_state(form: Formulation) -> np.ndarray:
  """Drops the cloth under CG, untimed; returns its integration state."""
  model = mujoco.MjModel.from_xml_string(_collision_xml(form, 'CG'))
  data = mujoco.MjData(model)
  while data.time < _DROP_TIME:
    mujoco.mj_step(model, data)
  _check(data)
  spec = mujoco.mjtState.mjSTATE_INTEGRATION
  state = np.empty(mujoco.mj_stateSize(model, spec))
  mujoco.mj_getState(model, data, state, spec)
  return state


def _run_collision(form: Formulation, solver: str) -> dict[str, float]:
  """Times every solver from the same folded state and counts contacts."""
  model = mujoco.MjModel.from_xml_string(_collision_xml(form, solver))
  data = mujoco.MjData(model)
  mujoco.mj_setState(model, data, _draped_state(form),
                     mujoco.mjtState.mjSTATE_INTEGRATION)
  mujoco.mj_forward(model, data)
  ms, iters, nnz, grad, conv = _step_timed(model, data, _COLLISION_STEPS)
  _check(data)
  # The Python bindings install a default timer, so mjData.timer is filled.
  collision = data.timer[mujoco.mjtTimer.mjTIMER_POS_COLLISION].duration
  flex = data.contact.flex[: data.ncon]
  verts = data.flexvert_xpos
  center = np.array([0, 0, _SPHERE_R])
  clearance = min(
      float(verts[:, 2].min()),
      float(np.linalg.norm(verts - center, axis=1).min()) - _SPHERE_R,
  )
  if _VERBOSE.value:
    print(
        f'  {form.label} {solver}: ncon {data.ncon}, z in'
        f' [{verts[:, 2].min():.4f}, {verts[:, 2].max():.4f}], warnings'
        f' {data.warning.number}',
        file=sys.stderr,
        flush=True,
    )
  return dict(
      nv=model.nv,
      nnz=nnz,
      ms=ms,
      iterations=iters,
      grad=grad,
      conv=conv,
      vertices=model.nflexvert,
      ncon=data.ncon,
      self=int(np.sum(np.all(flex >= 0, axis=1))),
      collision=1e3 * collision / _COLLISION_STEPS,
      penetration=max(0.0, _CLOTH_R - clearance),
  )


def _cpu_name() -> str:
  try:
    with open('/proc/cpuinfo') as f:
      for line in f:
        if line.startswith('model name'):
          return line.split(':', 1)[1].strip()
  except OSError:
    pass
  return f'{os.cpu_count()} CPUs'


def _print_head(title: str, head: list[str]) -> None:
  print(f'\n### {title}\n')
  print('| ' + ' | '.join(head) + ' |')
  print('|' + ' --- |' * len(head))


def _grad_cell(result: dict[str, float]) -> str:
  grad, conv = result['grad'], result['conv']
  if np.isnan(grad):
    return '-'
  return f'{grad:.1e} ({conv:.0f}%)'


def _report(title, forms, run, extra_cols) -> None:
  """Runs every formulation under every solver and prints a markdown table."""
  solvers = _SOLVERS.value
  head = ['Formulation', 'nv', *extra_cols, 'Newton nnz']
  head += [f'{solver} ms/step' for solver in solvers]
  head += [f'{solver} iters' for solver in solvers]
  head += [f'{solver} grad (%<tol)' for solver in solvers]
  head += ['Error (%)', 'Settled']
  _print_head(title, head)
  for form in forms:
    if _ONLY.value and _ONLY.value not in form.label:
      continue
    start = time.perf_counter()
    results = {solver: run(form, solver) for solver in solvers}
    print(f'{form.label}: {time.perf_counter() - start:.1f} s',
          file=sys.stderr, flush=True)
    first = results[solvers[0]]
    row = [form.label, str(first['nv'])]
    row += [str(first[col]) for col in extra_cols]
    row.append(str(results.get('Newton', first)['nnz']))
    row += [f"{results[solver]['ms']:.3g}" for solver in solvers]
    row += [f"{results[solver]['iterations']:.1f}" for solver in solvers]
    row += [_grad_cell(results[solver]) for solver in solvers]
    row.append(
        ' / '.join(f"{results[solver]['error']:+.1f}" for solver in solvers)
    )
    settled = all(res['speed'] < 1e-3 for res in results.values())
    row.append('yes' if settled else 'no')
    print('| ' + ' | '.join(row) + ' |', flush=True)


_Cell = Callable[[dict[str, float]], str]


def _report_columns(
    title: str,
    forms: tuple[Formulation, ...],
    run: Callable[[Formulation, str], dict[str, float]],
    shared: list[tuple[str, _Cell]],
    metrics: list[tuple[str, _Cell]],
) -> None:
  """Like _report, with problem-specific columns.

  Args:
    title: Table title.
    forms: Formulations to run.
    run: Maps a formulation and a solver to a dict of results.
    shared: (header, cell) pairs formatted from the first solver's results.
    metrics: (header, cell) pairs formatted for every solver; a cell shows
      'Newton / CG' unless the solvers agree to the printed precision.
  """
  solvers = _SOLVERS.value
  head = ['Formulation', 'nv', *(hdr for hdr, _ in shared), 'Newton nnz']
  head += [f'{solver} ms/step' for solver in solvers]
  head += [f'{solver} iters' for solver in solvers]
  head += [f'{solver} grad (%<tol)' for solver in solvers]
  head += [hdr for hdr, _ in metrics]
  _print_head(title, head)
  for form in forms:
    if _ONLY.value and _ONLY.value not in form.label:
      continue
    start = time.perf_counter()
    results = {solver: run(form, solver) for solver in solvers}
    print(f'{form.label}: {time.perf_counter() - start:.1f} s',
          file=sys.stderr, flush=True)
    first = results[solvers[0]]
    row = [form.label, str(first['nv'])]
    row += [cell(first) for _, cell in shared]
    row.append(str(results.get('Newton', first)['nnz']))
    row += [f"{results[solver]['ms']:.3g}" for solver in solvers]
    row += [f"{results[solver]['iterations']:.1f}" for solver in solvers]
    row += [_grad_cell(results[solver]) for solver in solvers]
    for _, cell in metrics:
      cells = [cell(results[solver]) for solver in solvers]
      row.append(cells[0] if len(set(cells)) == 1 else ' / '.join(cells))
    print('| ' + ' | '.join(row) + ' |', flush=True)


def _settled(result: dict[str, float]) -> str:
  return 'yes' if result['speed'] < 1e-3 else 'no'


def _mm(value: float) -> str:
  return '-' if np.isnan(value) else f'{1e3 * value:.3g}'


def _pct(value: float) -> str:
  return '-' if np.isnan(value) else f'{value:+.1f}'


def _report_ring_timesteps(forms: tuple[Formulation, ...]) -> None:
  """Prints the ring-down decay per timestep, undamped and damped (Newton)."""
  damping = _RING_DAMPING
  _print_head(
      f'Dynamics: implicit-step damping vs. timestep (Newton, c = {damping} s)',
      ['Formulation', 'Timestep (s)', 'f (Hz)', 'Decay, c = 0 (1/s)',
       '2 decay / w^2, c = 0 (s)', f'Decay, c = {damping} (1/s)',
       'vs. c w^2/2 (%)', 'vs. implicit step (%)'],
  )
  for form in forms:
    if _ONLY.value and _ONLY.value not in form.label:
      continue
    start = time.perf_counter()
    for timestep in _RING_TIMESTEPS:
      free = _ring_down(form, 'Newton', timestep, 0.0)
      damped = _ring_down(form, 'Newton', timestep, damping)
      omega, decay = damped['omega'], damped['decay']
      viscous = damping * omega**2 / 2
      implicit = _implicit_decay(omega, timestep, damping)
      row = [
          form.label,
          f'{timestep:g}',
          f'{omega / (2 * np.pi):.3f}',
          f"{free['decay']:.3f}",
          f"{2 * free['decay'] / free['omega']**2:.2e}",
          f'{decay:.3f}',
          f'{100 * (decay / viscous - 1):+.1f}',
          f'{100 * (decay / implicit - 1):+.1f}',
      ]
      print('| ' + ' | '.join(row) + ' |', flush=True)
    print(f'{form.label}: {time.perf_counter() - start:.1f} s',
          file=sys.stderr, flush=True)


def _report_stability() -> None:
  """Prints the largest stable timestep per formulation, young, integrator."""
  youngs = [f'{young:.0e}'.replace('e+0', 'e') for young in _STAB_YOUNG]
  _print_head(
      f'Stability (largest stable timestep in s, {_STAB_TIME} s under the tip'
      ' load, Newton)',
      ['Formulation', 'nv', 'Integrator'] + [f'young {ym}' for ym in youngs],
  )
  for form in _STABILITY:
    if _ONLY.value and _ONLY.value not in form.label:
      continue
    nv = mujoco.MjModel.from_xml_string(_cantilever_xml(form, 'Newton')).nv
    for integrator, damping in _STAB_INTEGRATORS:
      label = f'{integrator}, c = {damping:g} s'
      start = time.perf_counter()
      grid = (_STAB_TIMESTEPS if integrator == 'discrete'
              else _STAB_EULER_TIMESTEPS)
      cells = []
      for young in _STAB_YOUNG:
        stable = [
            dt for dt in grid
            if _stable(form, young, dt, integrator, damping)
        ]
        cells.append(_largest_stable(grid, stable))
      print(f'{form.label}, {label}: {time.perf_counter() - start:.1f} s',
            file=sys.stderr, flush=True)
      row = [form.label, str(nv), label, *cells]
      print('| ' + ' | '.join(row) + ' |', flush=True)


def main(argv) -> None:
  del argv  # Unused.
  print(f'MuJoCo {mujoco.__version__}; {_cpu_name()}; dt = {_DT} s')
  if 'cantilever' in _PROBLEMS.value:
    _report('Cantilever (tip-deflection error vs. Euler-Bernoulli)',
            _CANTILEVER, _run_cantilever, [])
  if 'torsion' in _PROBLEMS.value:
    _report('Torsion (end-twist error vs. T L / (G J))',
            _TORSION, _run_torsion, [])
  if 'block' in _PROBLEMS.value:
    _report('Block (compression error vs. uniaxial theory)',
            _BLOCK, _run_block, ['ncon'])
  if 'plate' in _PROBLEMS.value:
    _report('Plate, simply supported (center-deflection error vs. Kirchhoff)',
            _PLATE_SIMPLE, functools.partial(_run_plate, clamped=False), [])
    _report('Plate, clamped (center-deflection error vs. Kirchhoff)',
            _PLATE_CLAMPED, functools.partial(_run_plate, clamped=True), [])
  if 'membrane' in _PROBLEMS.value:
    _report('Membrane (elongation error vs. F L / (E t W))',
            _MEMBRANE, _run_membrane, [])
  if 'hertz' in _PROBLEMS.value:
    _report('Hertz (indentation error vs. half-space theory)',
            _HERTZ, _run_hertz, ['ncon', 'h / a'])
  if 'incompressible' in _PROBLEMS.value:
    _report('Incompressibility (tip-deflection error vs. Euler-Bernoulli)',
            _INCOMPRESSIBLE, _run_cantilever, [])
  if 'dynamics' in _PROBLEMS.value:
    _report(
        f'Dynamics (first-mode frequency error vs. Euler-Bernoulli f1 ='
        f' {_first_bending_omega() / (2 * np.pi):.3f} Hz, c ='
        f' {_RING_DAMPING} s)',
        _DYNAMICS, _run_dynamics,
        ['Static error (%)', 'f (Hz)', 'Decay (1/s)', 'Decay vs. c w^2/2 (%)',
         'Decay vs. implicit step (%)'],
    )
    _report_ring_timesteps(_RING_TIMESTEP_FORMS)
  if 'stability' in _PROBLEMS.value:
    _report_stability()
  if 'attach' in _PROBLEMS.value:
    _report_columns(
        'Attachments (settled deflection vs. Euler-Bernoulli or uniaxial)',
        _ATTACH, _run_attach,
        [('max nefc', lambda res: str(res['nefc']))],
        [('Deflection (mm)', lambda res: _mm(res['deflection'])),
         ('Reference (mm)', lambda res: _mm(res['reference'])),
         ('Error (%)', lambda res: f"{res['error']:+.1f}"),
         ('Servo load error (%)', lambda res: _pct(res['servo'])),
         ('Attach error (mm)', lambda res: _mm(res['attach'])),
         ('Warnings', lambda res: str(res['warnings'])),
         ('Settled', _settled)])
  if 'stretch' in _PROBLEMS.value:
    _report_columns(
        'Stretch (steady edge strain vs. edge solref timeconst)',
        _STRETCH, _run_stretch, [],
        [('Max strain (%)', lambda res: f"{res['max']:.3g}"),
         ('Mean strain (%)', lambda res: f"{res['mean']:.3g}"),
         ('Settled', _settled)])
  if 'collision' in _PROBLEMS.value:
    _report_columns(
        'Collision (cloth with self-collision dropped onto a sphere)',
        _COLLISION, _run_collision,
        [('vertices', lambda res: str(res['vertices']))],
        [('ncon', lambda res: str(res['ncon'])),
         ('self ncon', lambda res: str(res['self'])),
         ('Collision ms/step', lambda res: f"{res['collision']:.3g}"),
         ('Max penetration (mm)', lambda res: _mm(res['penetration']))])


if __name__ == '__main__':
  app.run(main)
