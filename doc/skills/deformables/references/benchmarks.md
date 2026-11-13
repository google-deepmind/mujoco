# Flex Formulation Benchmarks: Accuracy vs. Cost

Measured with this skill's `scripts/benchmark_flex.py` at MuJoCo commit
`70dede937` (`integrator="discrete"`, `timestep="0.002"`, `tolerance="1e-6"`,
`iterations="400"` unless noted). Timings are wall-clock ms per `mj_step` over
the first 500 loaded steps (100 steps from a shared draped state in Sec. 12) on
one core of a shared AMD EPYC 7B13, so compare them relative to each other.
Errors are signed: negative means the model is too stiff. On constrained
problems (`nefc > 0`), `grad (%<tol)` reports the median final scaled primal
gradient `data.solver[k].gradient` (across steps with `solver_niter > 0`) and
the percentage of constrained steps reaching `gradient < opt.tolerance`.

**When to rerun:** Rerun the affected `--problems=<section>` tables with
`scripts/benchmark_flex.py` (and update any derived guardrails in `SKILL.md`)
whenever flex elasticity or formulations (`engine_passive.c`, `user_flexcomp.cc`
-- e.g. fixing the jointed-parent fast path in
`mj_flexPassiveInterp`/`mj_flexPassiveBendInterp`), flex collision/contact
(`engine_collision_*`), the `discrete` effective metric (`engine_metric.c`), or
the `Newton`/`CG` primal solvers (`engine_solver.c`) change.

## 1. Cantilever Bending (Shear Locking & Rod Clamps)

A 0.48 x 0.04 x 0.04 m rubber beam (`young="5e6"`, `poisson="0.25"`, length /
thickness 12) is clamped at `x = 0` with a 0.25 N tip load and gravity off.
Reference: Euler-Bernoulli `P L^3 / (3 E I)` = 8.64 mm (shear adds < 1%).

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=cantilever`
`--timing_steps=500` (`solver="Newton"`).*

| Formulation | nv | ms/step | Tip error |
| :--- | ---: | ---: | ---: |
| 1D rod, 9 vertices | 28 | 0.04 | -18.0% |
| 1D rod, 17 vertices | 60 | 0.07 | -9.2% |
| 1D rod, 9 vertices, half-segment offset | 28 | 0.04 | -0.5% |
| 1D rod, 17 vertices, half-segment offset | 60 | 0.07 | -0.1% |
| 1D rod, 17 vertices, full-segment offset | 60 | 0.07 | +10.2% |
| `trilinear` 1x1x1 / 2x1x1 / 4x1x1 | 12 / 24 / 48 | 0.02-0.06 | -98% / -94% / -79% |
| `trilinear` 8x1x1 | 96 | 0.14 | -50.0% |
| `trilinear` 4x2x2 | 108 | 0.21 | -78.4% |
| `quadratic` 1x1x1 | 54 | 0.11 | -24.5% |
| `quadratic` 2x1x1 | 108 | 0.34 | -6.4% |
| `quadratic` 4x1x1 | 216 | 1.09 | -1.8% |
| `full` 17x3x3 | 432 | 1.74 | -55.7% |
| `full` 33x5x5 | 2400 | 16.3 | -24.8% |

Rod nv counts 3 slides per unpinned vertex plus one twist hinge per unclamped
edge.

-   **Rods win slender problems:** the half-segment-offset 17-vertex rod is
    60x more accurate than `quadratic` 2x1x1 at 1/5 of the cost.
-   **`trilinear` converges very slowly in bending** (-98% / -79% / -50% at
    1 / 4 / 8 cells along the span), and extra cells across the thickness
    (4x2x2) do not help. **`full` linear tets lock too** while costing 5-15x a
    more accurate `quadratic` beam; the 33x5x5 grid also warns `Flex stiffness
    is too ill-conditioned for the effective-metric block preconditioner`.
-   **Rod clamp offset:** `<pin id="0 1"/>` clamps the rod at the middle of
    edge 0, so a rod that starts at the mount bends as if half a segment
    short. Start it half a segment behind the mount: `spacing = L / (n - 1.5)`
    with vertex 0 at `x = -spacing / 2` (for a `grid`, `pos` x =
    `(L - spacing / 2) / 2`). A full-segment offset overshoots bending by about
    `+3 spacing / (2 L)`; see Sec. 2 for rods loaded in both.
-   3D rows have no constraints (`nefc = 0`), so `Newton` and `CG` both reduce
    to the smooth `M + h*D + h^2*K` solve at equal cost. On 1D rods (`<edge
    equality="true"/>`), `Newton` reaches `~1e-17` gradient (`100%` `< tol`) in
    0.6-0.7 iterations at the same `0.04-0.07` ms/step where `CG` exits early
    at `1.5e-3`-`5.4e-3` (`0-2%` `< tol`).

## 2. Rod Torsion

A 0.6 m rod (5 mm radius, `young="4e6"`, `shear="1.5e6"`) is clamped with
`<pin id="0 1"/>` and twisted by a 0.02 N*m motor on `<name>_twist`, gravity
off. Reference: `T L / (G J)` = 8.15 rad.

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=torsion`
`--timing_steps=500` (`solver="Newton"`).*

| Formulation | nv | ms/step | Twist error |
| :--- | ---: | ---: | ---: |
| 9 vertices | 28 | 0.03 | -12.5% |
| 31 vertices | 116 | 0.11 | -3.3% |
| 9 vertices, half-segment offset | 28 | 0.03 | -6.7% |
| 31 vertices, half-segment offset | 116 | 0.10 | -1.7% |
| 9 vertices, full-segment offset | 28 | 0.03 | +0.0% |
| 31 vertices, full-segment offset | 116 | 0.10 | -0.0% |

-   The error is half a segment at each end: the clamp acts at the middle of
    edge 0, and the end torque acts on the last edge's material frame. A
    full-segment offset (`spacing = L / (n - 2)`, vertex 1 at the mount)
    removes it.
-   **Rods loaded in both bending and torsion:** the two offsets conflict, and
    the bending error grows 3x faster than the torsion error. Use the
    half-segment offset (exact bending) with >= 30 segments, which keeps the
    twist error `-spacing / (2 L)` under 2%.

## 3. Block Compression with Contact (`Newton` vs. `CG` Convergence)

A 0.1 m cube (`young="5e4"`, `poisson="0"`, 0.1 kg, 10x10x10 collision grid
unless noted) is squeezed between the floor and a 2 kg plate on a vertical
slide joint, with `solref="0.004 1"` on all contacts. Reference: uniaxial
`sigma S / E` = 4.02 mm (plate load plus self-weight).

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=block`
`--timing_steps=500` (`tolerance="1e-6"`).*

| Formulation | nv | ncon | `Newton` ms/step | `Newton` grad (%<tol) | `CG` ms/step | `CG` grad (%<tol) | Error |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `trilinear` 1x1x1 | 25 | 100 | 0.60 | 5.2e-15 (100%) | 0.68 | 2.5e-04 (0%) | -0.0% |
| `trilinear` 2x2x2 | 82 | 100 | 0.89 | 1.9e-15 (100%) | 0.88 | 7.6e-05 (0%) | +0.0% |
| `trilinear` 3x3x3 | 193 | 100 | 1.52 | 1.9e-15 (100%) | 1.14 | 9.1e-05 (0%) | +0.1% |
| `quadratic` 1x1x1 | 82 | 100 | 1.13 | 2.1e-15 (100%) | 1.13 | 1.2e-04 (0%) | -0.7% |
| `quadratic` 2x2x2 | 376 | 100 | 3.82 | 2.7e-15 (100%) | 2.71 | 1.3e-04 (0%) | -2.2% |
| `full` 6x6x6 | 649 | 114 | 4.51 | 2.0e-15 (100%) | 2.70 | 1.7e-04 (0%) | +9.4% |
| `full` 8x8x8 | 1537 | 202 | 20.7 | 3.1e-15 (100%) | 7.70 | 1.6e-04 (0%) | +9.4% |
| `full` 8x8x8, SNH | 1537 | 202 | 20.4 | 3.1e-15 (100%) | 8.39 | 1.6e-04 (0%) | +2.5% |
| `full` 10x10x10 | 3001 | 314 | 71.1 | 3.4e-15 (100%) | 17.5 | 1.6e-04 (0%) | +9.1% |
| `full` 12x12x12 | 5185 | 450 | 244 | 4.2e-15 (100%) | 34.1 | 1.6e-04 (0%) | +9.0% |

-   **This problem tests cost and contact handling, not discretization
    accuracy:** with `poisson="0"` the compression is uniform, so a single
    linear cell represents it exactly. Do not read the `trilinear` 1x1x1 row
    as general accuracy; non-uniform strain (bending, indentation) needs the
    resolution shown in Sec. 1 and Sec. 6.
-   **`full` reads ~9% softer with default StVK, which SNH (`elastic3d=1`)
    removes:** non-interpolated 3D flexes default to Saint Venant-Kirchhoff
    (`elastic3d=0`), which softens under the 4% compression here (`+9.4%`),
    whereas `MjSpec`-only Stable Neo-Hookean (`flex.elastic3d = 1`) cuts the
    error to `+2.5%` (remaining offset is contact compliance) and
    `trilinear`/`quadratic` corotational elasticity matches within 0-2%.
-   **`CG`'s lower `ms/step` above ~200 DOFs is not at matched accuracy:**
    `Newton` converges to `~2e-15` (`100%` of steps `< 1e-6`) in 1.0-1.7
    iterations, whereas `CG` exits early on `improvement < tolerance` with
    final scaled gradients `7.6e-5`-`2.5e-4` (`0%` of steps `< 1e-6`, 76-250x
    above `tolerance`). Default to `Newton`, and fall back to `CG` only when
    `Newton`'s Cholesky factor becomes prohibitive (e.g. `full` 12x12x12 at
    1.63 M nonzeros).
-   **Heavy bodies on light vertices:** contact stiffness scales with the
    effective mass of the touching flex vertices. With the default
    `solref="0.02 1"` the 20x heavier plate sinks straight through the
    `quadratic` 2x2x2 and `full` blocks (it holds on coarse `trilinear` cells,
    whose interpolated vertices are heavier); `solref="0.004 1"` (about
    `2*timestep`) fixes it.

## 4. Thin-Plate Bending (Grid Bias vs. Staggered `type="direct"` Mesh)

A 0.2 x 0.2 m plate (`thickness="0.004"`, `young="2.5e8"`, `poisson="0.3"`,
areal density 4.0 kg/m^2, uniform gravity load `q = 39.24` Pa, flexural
rigidity `D = E t^3 / (12 (1 - nu^2)) = 1.465` N*m) sags under self-weight.
Reference center deflection: Navier `0.004062 q a^4 / D = 0.1741` mm (simply
supported, 1 outer ring pinned) and Taylor-Govindjee `0.001265 q a^4 / D =
0.0542` mm (clamped, 2 outer rings pinned).

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=plate`
`--timing_steps=500` (`nefc = 0`).*

| Support & formulation | nv | `Newton` ms/step | `CG` ms/step | Center error |
| :--- | ---: | ---: | ---: | ---: |
| Simply supported, `full` 9x9 | 147 | 0.21 | 0.21 | -42.4% |
| Simply supported, `full` 17x17 | 675 | 1.13 | 1.14 | -43.8% |
| Simply supported, `full` 33x33 | 2883 | 7.06 | 6.91 | -44.3% |
| Simply supported, `full` 17x17, `elastic2d="bend"` | 675 | 0.87 | 0.22 | -43.8% |
| Simply supported, `full` 33x33, `elastic2d="bend"` | 2883 | 5.68 | 0.93 | -44.2% |
| Simply supported, staggered `type="direct"` 10x11 | 231 | 0.36 | 0.34 | -8.3% |
| Simply supported, staggered `type="direct"` 19x21 | 999 | 1.81 | 1.80 | -6.3% |
| Simply supported, staggered `type="direct"` 34x39 | 3609 | 9.37 | 9.51 | -2.6% |
| Clamped (2 rings), `full` 9x9 / 17x17 / 33x33 | 75 / 507 / 2523 | 0.14 / 0.86 / 5.88 | 0.14 / 0.90 / 5.77 | -62.1% / -44.5% / -35.3% |
| Clamped (2 rings, half-cell offset), `full` 9x9 / 17x17 / 33x33 | 75 / 507 / 2523 | 0.13 / 0.87 / 5.82 | 0.13 / 0.88 / 5.88 | -35.3% / -28.2% / -26.5% |

-   **`type="grid"` plates are ~1.8x too stiff in 2D bending at any
    resolution:** `type="grid"` splits every quad along the same diagonal, so
    discrete hinge bending is anisotropic (exact for cylindrical bending along
    a grid axis, while diagonal bending costs 3.1x and twist 4.7x the Kirchhoff
    energy; -42% to -44% deflection error from 9x9 to 33x33). `type="grid"`
    remains fine for cloth and drape where bending accuracy is secondary.
-   **Use a staggered near-equilateral `type="direct"` mesh when plate bending
    accuracy matters:** shifting odd rows by half a spacing with
    `(ny - 1) / (nx - 1) ~ 2 / sqrt(3)` restores isotropic bending and
    converges cleanly (-8.3% at 10x11, -6.3% at 19x21, -2.6% at 34x39).
-   **Clamped-edge offset and unconstrained `elastic2d="bend"` metric path:**
    pinning two boundary rings places the effective clamp midway between them,
    shortening each edge by `h` (-62% at 9x9); extending the grid by half a
    cell per side (`spacing = a / (n - 2)`) removes the span loss. Here `nefc =
    0` (0 constraint iterations): on `elastic2d="bend"`-only plates, setting
    `solver="CG"` skips explicit CSR assembly in `mj_effBuild` and reuses the
    constant `mj_setConst` bending factor (`0.93` vs. `5.68` ms/step at 33x33),
    whereas once constraints are active (Sec. 11) `CG` stops early with
    unconverged gradients -- so keep `solver="Newton"` whenever contacts or
    equalities are present.
-   **Documented `mass` split and flat `dim=2` interpolated shells:** per
    `.. _body-flexcomp-mass:` in `XMLreference.rst` (*"pinning some points does
    not affect the mass of the other bodies"*), `<flexcomp>` assigns
    `mass / npnt` to every point including pinned boundary points, so a pinned
    `n x n` grid needs `mass` scaled by `(n / (n - 1))^2` to match a continuous
    areal density. Flat `dim="2"` shells cannot use `dof="trilinear"` or
    `"quadratic"` (`flex grid rotation R0 is not orthonormal`, zero-thickness
    bounding box); use `dof="full"` for flat sheets.

## 5. Membrane Uniaxial Stretch (`elastic2d="stretch"`)

A 0.3 x 0.2 m membrane (`thickness="0.001"`, `young="1e6"`, `poisson="0"`,
`elastic2d="stretch"`, gravity off) is pinned at `x = 0` and pulled at `x = L`
by `F = 0.2` N distributed with trapezoidal vertex weights (0.5 at corners,
1.0 on interior boundary vertices). Reference small-strain elongation:
`F L / (E t W) = 0.3` mm (0.1% strain).

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=membrane`
`--timing_steps=500` (`solver="Newton"`, `nefc = 0`).*

| Formulation | nv | ms/step | Elongation error |
| :--- | ---: | ---: | ---: |
| `full` 5x5 / 9x9 / 17x17 (0.1% strain) | 60 / 216 / 816 | 0.04 / 0.12 / 0.49 | -0.1% / -0.1% / -0.1% |
| `full` 9x9, 10x load (1% strain) | 216 | 0.13 | -1.5% |
| `full` 9x9, 100x load (10% strain) | 216 | 0.13 | -12.0% |

-   **Constant-strain membrane triangles are exact in uniform tension** across
    all grid resolutions (-0.1% at 0.1% strain from 5x5 to 17x17), provided
    edge traction uses trapezoidal vertex weights (uniform per-vertex forces
    over-weight corners and under-predict extension by ~11% at 5x5).
-   **Nonlinear Green-strain stiffening:** 2D stretching uses Green (StVK)
    strain (`F / (E t W) = lambda (lambda^2 - 1) / 2`), so membranes stiffen
    relative to linear `F L / (E t W)` by -1.5% at 1% strain and -12.0% at 10%
    strain.

## 6. Hertz Contact Indentation

A rigid frictionless sphere (`R = 0.1` m, `condim="1"`, 0.149 kg, `F = 1.465`
N) indents a 0.06 x 0.06 x 0.03 m block (`young="1e5"`, `poisson="0.3"`,
`E* = 109.9` kPa, base pinned, contact radius `a = (3 F R / (4 E*))^{1/3} =
10.0` mm, half-space `delta = a^2 / R = 1.000` mm) with `solref="0.004 1"` and
`solimp="0.9999 0.9999 0.001"`. Indentation is measured relative to the top
surface at `r = 1.5 a` against the Johnson half-space profile
`delta - u_z(1.5 a) = 0.702` mm. Interpolated runs share a 25x25x5 collision
mesh (`2.5 mm = a / 4` on top).

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=hertz`
`--timing_steps=500` (`tolerance="1e-6"`).*

| Formulation | nv | ncon | DOF `h / a` | `Newton` ms/step | `Newton` grad (%<tol) | `CG` ms/step | `CG` grad (%<tol) | Dent error |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `trilinear` 4x4x2 | 151 | 4 | 1.50 | 1.08 | 5.5e-16 (100%) | 1.02 | 1.1e-02 (2%) | -29.8% |
| `trilinear` 8x8x4 | 973 | 8 | 0.75 | 5.27 | 5.2e-16 (100%) | 3.81 | 2.9e-03 (1%) | -18.1% |
| `trilinear` 12x12x6 | 3043 | 50 | 0.50 | 59.9 | 1.0e-15 (66%) | 14.9 | 4.7e-03 (0%) | -7.9% |
| `quadratic` 2x2x1 | 151 | 8 | 1.50 | 1.23 | 5.8e-16 (100%) | 1.25 | 1.3e-02 (2%) | -39.2% |
| `quadratic` 4x4x2 | 973 | 12 | 0.75 | 8.56 | 5.2e-16 (100%) | 5.28 | 2.7e-03 (1%) | -7.1% |
| `quadratic` 6x6x3 | 3043 | 50 | 0.50 | 80.3 | 5.2e-16 (100%) | 22.3 | 1.9e-03 (0%) | -4.0% |
| `full` 5x5x3 | 151 | 2 | 1.50 | 0.37 | 6.5e-16 (100%) | 0.33 | 4.5e-03 (0%) | -55.5% |
| `full` 9x9x5 | 973 | 8 | 0.75 | 4.18 | 5.9e-16 (100%) | 3.04 | 3.0e-03 (0%) | -31.0% |
| `full` 13x13x7 | 3043 | 16 | 0.50 | 32.8 | 5.6e-16 (100%) | 11.8 | 2.4e-03 (0%) | -13.3% |

-   **Resolve contact patches with DOF spacing `h <= 0.5 a` (`0.75 a` for
    `quadratic`):** at equal DOF count (`nv = 973`), `quadratic` 4x4x2 reaches
    -7.1% error in 8.56 ms/step (`Newton`, `100%` `< tol`), whereas `trilinear`
    8x8x4 is at -18.1% and `full` 9x9x5 is at -31.0% (and even at `nv = 3043`,
    `full` 13x13x7 is still -13.3%). `CG`'s lower `ms/step` on the fine grids
    reflects early termination (`gradient` `1.9e-3`-`1.3e-2`, 1900-13,000x
    above `tolerance="1e-6"`), not faster convergence.
-   **Heavy indenters need both fast `solref` and near-hard `solimp`:** with
    default `solimp="0.9 0.95 0.001"`, constraint compliance adds 0.5-1.3 mm of
    artificial penetration (comparable to the 0.7 mm elastic dent) and prevents
    fine `quadratic` grids from settling; pair `solref="0.004 1"` with
    `solimp="0.9999 0.9999 0.001"`.
-   **50-contact cap on interpolated flexes:** `trilinear` and `quadratic`
    flexes collide as a single flex geom capped at `mjMAXCONPAIR = 50` contacts
    per geom pair, which both `h / a = 0.5` interpolated grids saturate.

## 7. Volumetric Locking Near Incompressibility (`poisson -> 0.5`)

The Sec. 1 cantilever (`0.48 x 0.04 x 0.04` m, `young="5e6"`, 0.25 N tip load,
Euler-Bernoulli reference 8.64 mm, which is independent of `poisson`) is swept
across `poisson = 0.25, 0.45, 0.49`.

*Measured at MuJoCo `70dede937` (`benchmark_flex.py --problems=incompressible`
`--timing_steps=500`, `solver="Newton"`, `nefc = 0`).*

| Formulation | nv | ms/step (0.25 / 0.45 / 0.49) | Tip error (`nu = 0.25`) | Tip error (`nu = 0.45`) | Tip error (`nu = 0.49`) |
| :--- | ---: | ---: | ---: | ---: | ---: |
| `trilinear` 8x1x1 | 96 | 0.15 / 0.18 / 0.26 | -50.0% | -58.4% | -67.2% |
| `quadratic` 2x1x1 | 108 | 0.34 / 0.45 / 0.70 | -6.4% | -24.4% | -47.6% |
| `quadratic` 4x1x1 | 216 | 1.00 / 1.71 / 2.70 | -1.8% | -11.5% | -21.0% |
| `full` 17x3x3 | 432 | 1.71 / 2.45 / 2.51 | -55.7% | -73.0% | -90.1% |
| `full` 17x3x3, SNH | 432 | 1.86 / 2.66 / 2.69 | -55.7% | -73.0% | -90.1% |
| `full` 33x5x5 | 2400 | 16.0 / 19.3 / 21.9 | -24.8% | -43.1% | -72.2% |

-   **All 3D formulations lock volumetrically toward `poisson = 0.5`:** as the
    Lame parameter `lambda = E nu / ((1 + nu)(1 - 2 nu))` diverges, even
    `quadratic` 2x1x1 stiffens from -6.4% (`0.25`) to -24.4% (`0.45`) and
    -47.6% (`0.49`), while `full` 17x3x3 locks to -90.1% under both StVK and
    SNH (`flex.elastic3d = 1`, since volumetric locking comes from constant-
    strain `P1` tetrahedra rather than the constitutive model). Keep 3D
    `poisson <= 0.45` (and refine `quadratic` to 4x1x1 if `0.45` accuracy
    within ~12% is needed). Both `full` and interpolated 3D flexes reject
    `poisson >= 0.5` at compile time (`Poisson ratio must be in [0, 0.5)`).
-   **3D smooth solve is block-PCG capped by `<option iterations>`:** the
    unconstrained `M + h*D + h^2*K` solve uses block-preconditioned PCG. Near
    `poisson = 0.49` (or on fine `full` grids), ill-conditioning causes PCG to
    take more iterations (raising step cost 1.4-2.7x) or hit the iteration cap
    and emit `mjWARN_INERTIA`.

## 8. Cantilever Dynamics & Implicit-Step Damping

The Sec. 1 cantilever (mass 0.15 kg, `density = 195.3` kg/m^3) is settled under
the 0.25 N tip load and released to ring down for 2.0 s (`damping = c = 0.005`
s, `gravity="0 0 0"`). Reference: Euler-Bernoulli fundamental frequency
`f_1 = (1.8751^2 / (2 pi)) sqrt(E I / (rho A L^4)) = 4.487` Hz, exact Rayleigh
decay rate `sigma = c omega^2 / 2`, and backward-Euler step decay
`log(1 + h omega^2 (c + h)) / (2 h) ~ (c + h) omega^2 / 2`.

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=dynamics`
(`solver="Newton"`, 2.0 s ring-down).*

| Formulation (`dt = 0.002` s, `c = 0.005` s) | nv | ms/step | Static error | `f_1` (Hz) | Freq error | Decay (1/s) | vs. `c w^2 / 2` | vs. implicit step |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1D rod, 9v, half-segment offset | 28 | 0.05 | -0.5% | 4.345 | -3.2% | 2.61 | +40.0% | +0.5% |
| 1D rod, 17v, half-segment offset | 60 | 0.08 | -0.1% | 4.408 | -1.8% | 2.68 | +39.9% | +0.5% |
| 1D rod, 33v, half-segment offset | 124 | 0.15 | -0.0% | 4.441 | -1.0% | 2.73 | +40.0% | +0.5% |
| `trilinear` 8x1x1 | 96 | 0.14 | -50.0% | 6.046 | +34.7% | 5.05 | +39.9% | +0.9% |
| `quadratic` 2x1x1 | 108 | 0.33 | -6.4% | 4.480 | -0.2% | 2.77 | +40.0% | +0.5% |
| `quadratic` 4x1x1 | 216 | 1.01 | -1.8% | 4.426 | -1.4% | 2.71 | +40.0% | +0.5% |
| `full` 17x3x3 | 432 | 2.23 | -55.7% | 6.082 | +35.5% | 5.28 | +44.6% | +4.4% |

Timestep sweep on `quadratic` 2x1x1 (same law holds on 1D rods;
*measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=dynamics`*):

| `timestep` `h` (s) | `f_1` (Hz) | Decay, `c = 0` (1/s) | `2 * decay / w^2` (`c = 0`) | Decay, `c = 0.005` (1/s) | Error vs. `c w^2 / 2` | Error vs. implicit step |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.004 | 4.462 | 1.580 | 3.99e-03 s | 3.533 | +79.8% | +1.3% |
| 0.002 | 4.480 | 0.795 | 2.00e-03 s | 2.772 | +40.0% | +0.5% |
| 0.001 | 4.486 | 0.398 | 1.00e-03 s | 2.384 | +20.0% | +0.2% |
| 0.0005 | 4.489 | 0.198 | 4.98e-04 s | 2.188 | +10.0% | +0.1% |

-   **`integrator="discrete"` adds numerical damping equal to `damping =
    timestep`:** at `c = 0`, `2 * decay / omega^2 = h` to three digits across
    all timesteps, so with `c = 0.005` s and `h = 0.002` s every mode decays
    40% faster than `c omega^2 / 2` (`(c + h) / c = 1.40`), matching the
    backward-Euler formula within 0.5%. Lightly damped oscillations require a
    smaller `timestep` (and/or reduced `c = c_target - h` for one target mode).
-   **Modal frequency tracks static bending stiffness** (`f ~ sqrt(k)`):
    `quadratic` 2x1x1 and 4x1x1 match `f_1` within -0.2% and -1.4% (3D
    Timoshenko shear and rotary inertia shift the continuum limit ~-0.9% below
    Euler-Bernoulli), whereas shear-locked `trilinear` 8x1x1 and `full` 17x3x3
    oscillate +35% too fast.
-   **Lumped-mass tip bias on 1D rods (`.. _body-flexcomp-mass:`):** because
    `<flexcomp>` assigns `mass / n` to every vertex regardless of `<pin>`, a
    full segment's mass sits at the free tip vertex instead of half a segment,
    shifting `f_1` down by `~1 / (4 n)` (-3.2% / -1.8% / -1.0% at 9 / 17 / 33
    vertices).

## 9. Integrator Stability (`discrete` vs. `Euler`)

Largest stable `timestep` (in seconds, tested across `0.0001` to `0.016` s over
0.5 s under the 0.25 N tip load) for the Sec. 1 cantilever across `young` in
`{5e5, 5e6, 5e7}` Pa.

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=stability`
(`solver="Newton"`).*

| Formulation | `discrete`, `c = 0.005` (0.5 / 5 / 50 MPa) | `Euler`, `c = 0.005` (0.5 / 5 / 50 MPa) | `Euler`, `c = 0` (0.5 / 5 / 50 MPa) |
| :--- | ---: | ---: | ---: |
| 1D rod, 17v, half-segment offset | >=0.016 / >=0.016 / >=0.016 | <0.0001 / <0.0001 / <0.0001 | 0.0005 / 0.0002 / <0.0001 |
| `trilinear` 4x1x1 | >=0.016 / >=0.016 / >=0.016 | <0.0001 / <0.0001 / <0.0001 | 0.0005 / 0.0002 / <0.0001 |
| `quadratic` 2x1x1 | >=0.016 / >=0.016 / >=0.016 | <0.0001 / <0.0001 / <0.0001 | 0.0002 / <0.0001 / <0.0001 |
| `full` 17x3x3 | >=0.016 / >=0.016 / >=0.016 | <0.0001 / <0.0001 / <0.0001 | 0.0002 / <0.0001 / <0.0001 |

-   **Always use `integrator="discrete"` for elastic flexes:** it integrates
    `M + h*D + h^2*K` implicitly and stays unconditionally stable at the
    maximum tested `0.016` s across every formulation and stiffness up to 50
    MPa.
-   **Explicit `Euler` stability depends on the highest element mode
    `omega_max`:** `Euler` integrates both elastic stiffness and Rayleigh
    stiffness-damping forces explicitly in `qfrc_passive`, requiring `h < 2 /
    (c * omega_max^2)` when `c > 0` (`< 0.0001` s on this cantilever across
    `0.5`-`50` MPa at `c = 0.005` s; softer or coarser meshes allow larger `h`).
    Even at `damping="0"`, this cantilever needs `0.2-0.5` ms at 0.5 MPa and
    `< 0.1` ms at 50 MPa.

## 10. Rigid-Body Attachments & Jointed Parents

A `quadratic` 2x1x1 beam (`nv = 108`) and a 17-vertex box rod (`nv = 60`,
half-segment offset) sag under gravity from: (1) a static mount, (2) a mount
driven at 2 Hz by a position servo (`slide` +-20 mm or `hinge` +-0.1 rad,
mount origin offset 20 mm behind the clamp face) for 500 timed steps then held
at 0 to settle, and (3) a static mount with a 0.05 kg free tip body held by
`<connect solref="0.004 1"/>` (4 corner nodes for the beam, 2 adjacent vertices
for the rod) or `<weld solref="0.004 1"/>`; plus a `trilinear` 2x2x2 gripper
pad (`0.04 x 0.04 x 0.02` m, `young="5e4"`, `poisson="0"`) on a slide-jointed
finger pressed onto a table. Reference sag: `w_self + w_tip` = 20.8 mm (beam),
37.8 mm (beam + 50 g tip), 18.9 mm (rod, using active span mass), 35.9 mm
(rod + 50 g tip), and 0.553 mm (pad).

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=attach`
`--timing_steps=500` (`tolerance="1e-6"`).*

| Formulation & attachment | nv | `Newton` ms/step | `Newton` grad (%<tol) | `CG` ms/step | `CG` grad (%<tol) | Defl error | Extra metric |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | :--- |
| `quadratic` 2x1x1, static mount | 108 | 0.37 | - | 0.37 | - | -8.3% | -- |
| `quadratic` 2x1x1, driven slide mount | 109 | 0.85 | - | 1.21 | - | -8.3% | Servo load error +23.1% |
| `quadratic` 2x1x1, driven hinge mount | 109 | 0.83 | - | 1.21 | - | -8.4% | Servo load error +100.3% |
| `quadratic` 2x1x1, 4 connects to free tip | 114 | 0.46 | 5.7e-16 (100%) | 0.60 | 9.8e-03 (0%) | -8.4% | Attach gap 0.55 mm (`CG` unsettles) |
| `quadratic` 2x1x1, weld to free tip | 114 | 0.45 | 6.4e-16 (100%) | 0.61 | 1.0e-02 (0%) | -8.4% | Attach gap 0.21 mm, tip tilt 0 deg (`CG` unsettles) |
| 1D rod, 17v, static mount | 60 | 0.07 | 4.9e-17 (100%) | 0.07 | 5.3e-03 (3%) | -0.4% | -- |
| 1D rod, 17v, driven slide mount | 61 | 0.12 | 2.3e-16 (100%) | 0.28 | 5.3e-03 (0%) | -0.4% | Servo load error +0.0% |
| 1D rod, 17v, driven hinge mount | 61 | 0.13 | 5.2e-16 (100%) | 0.30 | 6.8e-03 (0%) | -0.4% | Servo load error +0.0% |
| 1D rod, 17v, 2 connects to free tip | 66 | 0.09 | 1.8e-16 (100%) | 0.10 | 5.8e-03 (2%) | -0.7% | Attach gap 0.16 mm |
| 1D rod, 17v, weld to free tip | 66 | 0.09 | 1.7e-16 (100%) | 0.10 | 4.9e-03 (1%) | -0.7% | Attach gap 0.16 mm, tip tilt 0 deg |
| `trilinear` 2x2x2 pad, slide finger | 55 | 0.20 | 1.6e-15 (100%) | 0.20 | 1.0e-03 (0%) | -1.5% | Servo load error +186.6% |

-   **`Newton` is both faster (1.3-2.3x) and far more accurate than `CG` on
    jointed mounts and equality attachments:** `Newton` factors the coupled
    system to `~1e-16` (`100%` `< tol`) in 1 iteration, whereas `CG` stops
    early on `improvement < tolerance` at `1.0e-3`-`1.0e-2` (`0-3%` `< tol`,
    leaving ~2e-3 m/s residual tip velocity after 10 s on `quadratic`
    `<connect>`/`<weld>`).
-   **Known engine bug (interpolated flexes on jointed parents):** in
    `mj_flexPassiveInterp` / `mj_flexPassiveBendInterp`, the direct-write fast
    path writes `R' f` only into each free node body's own three slide DOFs,
    which equals `J' f` only when the body has no moving ancestors. With a
    jointed parent, (1) the free nodes' share of the elastic and damping forces
    never reaches the parent joint DOFs, leaving a spurious net force on the
    servo (+23.1% slide, +100.3% hinge, +186.6% finger pad; 1D rods report
    0.0%), and (2) a pinned node at the jointed parent's `(0, 0, 0)` origin
    writes three components starting at the parent's first DOF. Rerun
    `--problems=attach` and remove this note once the `mj_flexPassiveInterp`
    fix lands.
-   **`<weld>` to a flex vertex/node locks world orientation:** flex vertex and
    node bodies have only 3 translational slide joints and keep constant world
    orientation, so `<weld>` locks the attached rigid body's world orientation
    (`0.0 deg` tip tilt vs `6.2 deg` with 4 corner `<connect>`s). Use 3+
    non-collinear `<connect>` constraints on 3D beams or 2 adjacent-vertex
    `<connect>` constraints on 1D rods to transmit torque while letting the tip
    rotate with the flex.

## 11. Edge-Equality Stretch (`<edge equality="true"/>` vs. `solref`)

A vertical 0.48 m, 0.1 kg rod (`young="4e6"`, `shear="1.5e6"`,
`<pin id="0 1"/>`) and a 0.3 x 0.3 m, 0.1 kg `13x13` cloth (`elastic2d="bend"`,
top row pinned) are suspended under gravity with `<edge equality="true"
solref="{timeconst} 1"/>`. Reference elongation of an inextensible cable/sheet:
0%.

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=stretch`
`--timing_steps=500` (`tolerance="1e-6"`).*

| Formulation & `solref` `timeconst` | nv | `Newton` ms/step | `Newton` grad (%<tol) | `CG` ms/step | `CG` grad (%<tol) | Max edge strain | Mean edge strain |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1D rod, 17v, `timeconst="0.02"` | 60 | 0.07 | 8.2e-16 (100%) | 0.06 | 5.5e-04 (7%) | 9.81% | 5.34% |
| 1D rod, 17v, `timeconst="0.01"` | 60 | 0.07 | 8.0e-16 (100%) | 0.06 | 5.2e-04 (6%) | 2.67% | 1.75% |
| 1D rod, 17v, `timeconst="0.004"` | 60 | 0.07 | 7.8e-16 (100%) | 0.06 | 6.8e-04 (2%) | 0.81% | 0.45% |
| 1D rod, 17v, `timeconst="0.002"` | 60 | 0.06 | 8.2e-16 (100%) | 0.06 | 8.9e-04 (2%) | 0.22% | 0.12% |
| 1D rod, 33v, `timeconst="0.004"` | 124 | 0.14 | 1.2e-15 (100%) | 0.12 | 4.6e-04 (0%) | 2.84% | 1.63% |
| Cloth 13x13, `timeconst="0.02"` | 468 | 1.64 | 3.1e-16 (100%) | 0.72 | 2.9e-04 (0%) | 6.31% | 1.15% |
| Cloth 13x13, `timeconst="0.01"` | 468 | 1.59 | 3.2e-16 (100%) | 0.66 | 2.8e-04 (0%) | 1.80% | 0.41% |
| Cloth 13x13, `timeconst="0.004"` | 468 | 1.55 | 3.3e-16 (100%) | 0.59 | 3.1e-04 (0%) | 0.50% | 0.10% |
| Cloth 13x13, `timeconst="0.002"` | 468 | 1.28 | 3.3e-16 (100%) | 0.64 | 4.3e-04 (0%) | 0.13% | 0.02% |

-   **Edge-equality stretch scales as `timeconst^2` at zero extra step cost:**
    under `integrator="discrete"`, `refsafe` never clamps equality `solref`, so
    reducing `timeconst` from the default `0.02` s to `0.002` s cuts peak root
    edge stretch ~45x (9.81% to 0.22% on the 17-vertex rod; 6.31% to 0.13% on
    the 13x13 cloth) without increasing `ms/step`.
-   **Subdividing a rod increases edge-equality stretch as `~ (n - 1)^2`:**
    MuJoCo normalizes constraint stiffness per row by inverse vertex mass
    (`1 / A_ii ~ m_vert / 2`), so doubling a suspended rod from 17 to 33
    vertices at fixed total mass and `timeconst="0.004"` raises peak strain
    from 0.81% to 2.84% (3.5x). Tighten `timeconst` (or raise `solimp` `d0`)
    when refining edge-constrained meshes.
-   **`Newton` converges (`100%` `< tol`, `~3e-16`) while `CG` stops early
    (`0%` `< tol`, `~3e-4`):** `CG`'s 2.0-2.6x shorter step time on `13x13`
    cloth reflects early exit on `improvement < tolerance` after 5.5-7.0
    iterations, which leaves ~3e-3 m/s residual flutter after 10 s (`Newton`
    settles to rest in 1 iteration per step).

## 12. Cloth Self-Collision Scaling

A 0.6 x 0.6 m cloth (`young="5e4"`, `poisson="0.35"`, `thickness="1.5e-3"`,
`elastic2d="both"`, `selfcollide="auto"`, `solref="0.004 1"`) drops onto a 0.1
m sphere on the floor for 0.8 s, and is then timed for 100 steps from that
identical folded state under both solvers (plus a `trilinear` 6x6 shell on a
41x41 vertex mesh, tilted 10 deg so its 3D bounding box is non-degenerate).

*Measured at MuJoCo `70dede937` with `benchmark_flex.py --problems=collision`
(100 steps from draped state, `tolerance="1e-6"`).*

| Formulation | nv | Vertices | `Newton` nnz | `Newton` ms/step | `Newton` grad (%<tol) | `CG` ms/step | `CG` grad (%<tol) | Contacts (self) | Collision ms/step |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Cloth 11x11 (`dof="full"`) | 363 | 121 | 23,226 | 1.57 | 1.0e-16 (100%) | 0.89 | 7.9e-05 (0%) | 82 (28) | 0.22 |
| Cloth 21x21 (`dof="full"`) | 1323 | 441 | 129,678 | 8.80 | 7.0e-17 (89%) | 4.28 | 1.1e-04 (0%) | 266 (82) | 1.16 |
| Cloth 41x41 (`dof="full"`) | 5043 | 1681 | 804,576 | 79.3 | 4.4e-17 (65%) | 25.0 | 1.0e-04 (0%) | 826 (122) | 4.45 |
| Cloth 61x61 (`dof="full"`) | 11163 | 3721 | 2,381,451 | 515 | 5.3e-17 (52%) | 92.4 | 1.0e-04 (0%) | 1442 (274) | 12.2 |
| `trilinear` shell 6x6, 41x41v | 294 | 1681 | 18,201 | 1.20 | 1.5e-16 (90%) | 1.05 | 1.1e-04 (0%) | 15 (0) | 0.20 |

-   **Step time is dominated by the solver (`nv` and `nnz`), not collision
    detection:** `mjTIMER_POS_COLLISION` accounts for only 2-14% of total step
    time up to 61x61 (`1442` contacts, `274` self-contacts). Cutting `nv` from
    5043 to 294 on the same 1681-vertex mesh (`trilinear` 6x6 shell) reduces
    `Newton` step time by 66x, though a 6x6 control grid is too smooth to form
    self-folds.
-   **`Newton` vs. `CG` at large `nv`:** `Newton` drives median scaled
    gradients to `~5e-17`-`1e-16`, whereas `CG` exits early on `improvement <
    tolerance` at `~1e-4` (`0%` of steps `< 1e-6`, 79-110x above `tolerance`)
    across every mesh size. Use `Newton` up to moderate cloth sizes (`11x11`-
    `21x21`), and switch to `CG` only at high `nv` (`41x41`+, where `Newton`
    factors 0.8M-2.4M nonzeros per step) after checking `data.solver` gradient
    residuals for acceptable visual/contact fidelity.
