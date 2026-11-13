---
name: mujoco-deformables
description: >-
  Models, tunes, and attaches deformable objects in MuJoCo using MJCF
  (flexcomp, flex, skin, elasticity, pin) and Python MjSpec
  (make_flex, add_flex). Covers 1D elastic rods (cables, ropes, wires, coils),
  2D cloth draping, garments, and thin shells, and 3D FEM/tetrahedral soft
  bodies (sponges, foam, rubber, elasto-plastic clay, compliant gripper pads),
  plus implicit position-stiffness integration and reduced-DOF interpolation.
  Use when creating, migrating (from removed composite or cable plugins),
  attaching, or debugging unstable flex or soft-body simulations. Don't use for
  rigid-only modeling (mujoco-spec-editing), general stepping loops
  (mujoco-python), GPU rollout setup (mujoco-accelerated), cross-engine parity
  (mujoco-parity), or rendering (mujoco-rendering).
---

# MuJoCo Deformables: Modeling, Tuning & Attachments

> [!TIP]
> **Official MuJoCo docs (`doc/*.rst`):** Grep literal `.. _Label:` anchors in
> [`../../modeling.rst`](../../modeling.rst) (`.. _CDeformable:`, `.. _CRod:`,
> `.. _CTuningDeformable:`, `.. _CSize:`),
> [`../../XMLreference.rst`](../../XMLreference.rst) (`.. _body-flexcomp:`,
> `.. _deformable-flex:`, `.. _flex-elasticity:`, `.. _flex-contact:`,
> `.. _deformable-skin:`),
> [`../../computation/index.rst`](../../computation/index.rst)
> (`.. _geIntegrators:`), and [`../../changelog.rst`](../../changelog.rst).
> Online: `https://mujoco.readthedocs.io/en/latest/` and GitHub `model/flex/`.
> **Related skills:** [mujoco-spec-editing](../spec_editing/SKILL.md),
> [mujoco-python](../python/SKILL.md), [mujoco-studio](../studio/SKILL.md),
> [mujoco-accelerated](../accelerated/SKILL.md),
> [mujoco-parity](../parity/SKILL.md).

> [!CAUTION]
> - **Removed APIs:** `<composite>`, `mujoco.elasticity.cable`, `<flexvert>`,
>   and `<edge equality="vert"/>` (`2`) are removed; use `<flexcomp>`/`<flex>`.
> - **Integrators:** `<elasticity>` rejects `implicit`/`implicitfast`;
>   `passive="true"` & `MjSpec` `flex.elastic3d = 1` reject non-`discrete`.
> - **No `<edge stiffness/damping>` under `discrete`:** Use `<elasticity
>   damping="..."/>` (seconds) and `<edge equality="..."/>` instead.
> - **Parent joints & joint `damping`:** Unpinned `full`/`trilinear`/`quadratic`
>   flexes forbid parent/ancestor joints (`2d`/`radial` exempt); scope
>   `<default><joint damping="..."/>` away from `<flexcomp>` (compile error).

## 1. Choosing Dimension (`dim`), `type` & DOF Parametrization (`dof`)

Cost grows with flex DOFs, while **collision always uses the full flex mesh**
(`.. _CTuningDeformable:`). `full`/`2d`/`radial` put DOFs on vertices;
`trilinear`/`quadratic` use a coarser node lattice; pinned points add 0 DOFs.
Deflection errors below are signed relative to theory (negative = too stiff).

| Use Case | `dim` | `dof` | Unpinned DOFs | Guidance |
| :--- | :--- | :--- | :--- | :--- |
| Cables, wires, ropes, coils, slender beams | `1` | `full` | `4 * nvert - 1` for elastic open rods (3 slides per vertex + 1 twist hinge per edge; `4 * (nvert - 2)` with `<pin id="0 1"/>`) | Discrete elastic rod (`section="circle"` or `"box"`). Cheapest and most accurate for slender parts: within 1.5% of beam theory from 9 vertices (half-segment clamp offset, Sec. 3.1). |
| Cloth, garments, flags, thin plates/shells | `2` | `full` | `3 * nvert` | Pair positive collision `radius` with physical `thickness` and `elastic2d="both"` (or `"stretch"`). Required for flat 2D sheets (cloth, plates). |
| Compliant solids or lattice shells | `2` or `3` | `trilinear` | `3 * (cx+1)*(cy+1)*(cz+1)` for `cellcount="cx cy cz"` (`24` for `1 1 1`) | **Default for compact solids.** Two independent choices: `dim` picks the embedded/collision mesh (`2` surface triangles, `3` tets); `elastic2d` picks mechanics (`"none"` = 3D volume/cut-cell, `bunny_multicell.xml`; else = shell on lattice boundary faces with interior nodes auto-pinned, `bunny_shell.xml`; both at `dim=3` in `hollow_vs_solid.xml`). Under-predicts bending deflection by 50-98% at 1-8 cells; supports `yield`. |
| 3D beams, solids, or lattice shells that bend | `2` or `3` | `quadratic` | `3 * (2*cx+1)*(2*cy+1)*(2*cz+1)` (`81` for `1 1 1`) | Same independent `dim` (`2` triangles as in `bunny_quadratic.xml`, `3` tets) and `elastic2d` (volume vs. shell) choices, without shear locking: -6% / -2% bending error at `cellcount="2 1 1"` / `"4 1 1"` (`mechanisms/cantilever.xml`); supports `yield`. |
| Fine local indentation (`3`) or 2D bend plasticity (`2`) | `2` or `3` | `full` | `3 * nvert` | Highest cost in 3D and still stiff in bending (-56% / -25% bending error at 17x3x3 / 33x5x5 for 5-17x the cost of `quadratic`); use only when localized 3D dents/yield (`model/plastic/clay.xml`) or 2D bending plasticity (`paper.xml`, `foil.xml`; `dim="2"` required) need per-vertex DOFs. |
| Planar compliant mechanisms (e.g. Fin-Ray) | `2` or `3` | `2d` | `2 * nvert` | X-Y slide DOFs in parent frame (`model/flex/gripper_2d.xml`); parent body may have joints even when unpinned. |
| Hollow pressurized / spherical-like blobs | `2` or `3` | `radial` | `nvert - 1` (+6 parent `freejoint`) | 1 radial slide DOF per shell vertex; center vertex `0` is pinned to the parent (`type` `box`/`cylinder`/`ellipsoid`, `model/flex/softbox.xml`). Add a parent `<freejoint/>` for free motion. |
| Exact non-convex rigid collider | `2` or `3` | `rigid="true"` | `0` extra DOFs | Pins all vertices to parent body (auto-set if all points are pinned); exact triangle/tet collisions without convex hulling or SDFs. |
| High-res visual skin over coarse flex or links | -- | `<skin>` | `0` physics DOFs | Hide coarse physics `<flexcomp group="3">` (`0..2` visible, `3..5` hidden) and bind `<deformable><skin group="0" inflate="...">` (`spec.add_skin()`) via `<bone body="<flex>_<i>"/>`. |

-   **Guardrails ([`references/benchmarks.md`](references/benchmarks.md)):** 1D
    rod if length > 5x thickness; for bending use `quadratic` with >= 2 span
    cells (`trilinear` under-predicts bending deflection by 50-98% at 1-8
    cells); DOF spacing `<= 0.5 * contact_radius` under contacts (`quadratic`
    cheapest); compact solids default to `trilinear` `1 1 1` (24 DOFs, exact
    uniform strain) or `2 2 2` (81 DOFs); `dof="full"` > ~100 3D vertices only
    for local dents/plasticity. **Interpolated lattices need a non-degenerate 3D
    bounding box** -- flat sheets (cloth, plates) must use `dof="full"`.
-   **`<flexcomp type="...">` (`.. _body-flexcomp:`):** `grid` (centered at
    `+-0.5*(count-1)*spacing` on `pos`), `circle` (`dim=1`), `square`/`disc`
    (`dim=2`), `box`/`cylinder`/`ellipsoid` (`dim="3"` solid or `dim="2"` open
    box, `basket.xml`), `direct` (`point`/`element`, `bag.xml`; `dim="1"`
    auto-chains `[0,1, 1,2, ...]`, `coil.xml`), `mesh`/`gmsh` (Sec. 5). Per
    `.. _body-flexcomp-mass:`, each point gets `mass / npnt` even if `<pin>`.

## 2. Deformation Models, Integrators & Solvers

### Track A: Continuum `<elasticity>` + `integrator="discrete"`

Implicit position-stiffness (`integrator="discrete"`) integrates `<elasticity>`,
springs, and servo `kp` in `M_hat = M + h*D + h^2*K`, stable up to `0.016` s
(`Euler` is explicit: `h < 2 / (damping * omega_max^2)` -- `< 0.1` ms at
`damping="0.005"` and `0.2`-`0.5` ms undamped on the benchmark cantilever),
adds numerical damping `damping = timestep`
([`references/benchmarks.md`](references/benchmarks.md)), stores `qacc =
(v_next - v)/h`, and skips `refsafe` on equalities. For **every** solver,
`qacc_smooth = (M_hat)^-1 f` (`mj_effSolve`) runs PCG on `M_hat`: 1D rods and
non-interpolated 2D flexes build a per-step sparse Cholesky preconditioner (1
iter on rods/2D stretch, a few on 2D bend), whereas 3D `full` and all
`trilinear`/`quadratic` flexes (`dim=2` shells included) iterate up to
`opt.iterations` (hitting the cap warns `"Flex stiffness is too
ill-conditioned..."` as `mjWARN_INERTIA`, leaving `qacc_smooth` unconverged).

-   **`solver="Newton"` (default -- use unless factorization is prohibitive):**
    Constraint solve sparse-Cholesky factors the assembled Hessian (`M_hat` +
    active constraints) to `gradient ~ 1e-12`-`1e-15`. Supports all `dof` modes,
    articulated attachments (`<pin>` on jointed bodies, `<connect>`, `<weld>`,
    `2d`, `radial`), and `ipc` (`tolerance="1e-6"`, `iterations="150"`-`"400"`;
    used across `model/flex/`).
-   **`solver="CG"` & early-stop warning:** Matrix-free; **use `CG` only when
    `Newton` factorization is prohibitive (e.g. `plastic/clay.xml`, `nv = 6349`)
    or on soft / `Euler` models**. `CG` often exits early on `improvement <
    tolerance` with `gradient` 10x-3800x above `opt.tolerance`
    ([`references/benchmarks.md`](references/benchmarks.md)): check `s =
    data.solver[i*mujoco.mjNSOLVER + data.solver_niter[i] - 1]` (`s.gradient <
    model.opt.tolerance` vs. `improvement` / `opt.iterations`). `PGS` &
    `noslip_iterations > 0` fail under `discrete` (`ipc` also forbids `sleep`/
    `fwdinv`; CPU `sleep`+`island` work otherwise).
-   **Material `<elasticity>`:** `young` in `Pa` (`1e4`-`1e5` foam, `1e6`-`2e7`
    rubber/rods); **keep 3D `poisson <= 0.45`** in `[0, 0.5)` (`0.5` locks &
    fails compile); `damping` in **seconds** (`0.002`-`0.05` s in `D = damping *
    K`, so scaling `young` by `alpha` scales `D` unless `damping /= alpha`).

### Track B: `<edge equality="...">` + `integrator="Euler"` / `"implicitfast"`

For soft parts -- or parts whose exact material response does not matter -- in
scenes that must stay on `Euler` or `implicitfast`, replace `<elasticity>` with
soft constraints: stiffness comes from `solref`/`solimp` (not `young`), and
`refsafe` clamps **every** `timeconst >= 2 * timestep` (**equalities included**;
only `discrete` skips this), capping stiffness by `timestep`. Constrain edges
(`equality="true"` on `full`/`radial`/`2d`) or strain (`equality="strain"`, the
**only** mode on `trilinear`/`quadratic`; disables `yield`); `young > 0` fails
except on 1D rods and `elastic2d="bend"` (Sec. 8). **Set `solref`/`solimp` on
`<edge>`** (ignores `<default><equality>`) with **`timeconst <= 0.005` s** (at
`timestep="0.002"`, `< 0.004` is silently raised to `0.004`; 0.8% vs. 2.8%
stretch at 17v vs. 33v, [`references/benchmarks.md`](references/benchmarks.md)):

```xml
<mujoco model="compliant_pad">
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <worldbody>
    <geom name="target" type="sphere" size="0.015" pos="0 0 0.08" contype="2" conaffinity="2"/>
    <body name="finger" pos="0 0 0.11">
      <joint name="grip" type="slide" axis="0 0 1" damping="1.0"/>
      <geom type="box" size="0.02 0.02 0.005" pos="0 0 0.008"/>
      <flexcomp name="pad" type="grid" dim="3" count="4 4 2" spacing="0.01 0.01 0.008" radius="0.001" mass="0.05">
        <pin gridrange="0 0 1 3 3 1"/>
        <edge equality="true" solref="0.005 1" solimp="0.95 0.99 0.001 0.5 2"/>
        <contact selfcollide="none" contype="2" conaffinity="2"/>
      </flexcomp>
    </body>
  </worldbody>
</mujoco>
```

## 3. Canonical MJCF Recipes by Geometry

### 3.1 -- 1D Discrete Elastic Rods (Cables, Wires, Coils & Beams)

1D `<elasticity>` models **bending and torsion only** (`.. _CRod:`); pair it
with `<edge equality="true" solref="0.003 1"/>` (`timeconst <= 0.005`):

```xml
<mujoco model="clamped_cable">
  <compiler autolimits="true"/>
  <option timestep="0.002" integrator="discrete" solver="Newton" tolerance="1e-6"/>
  <worldbody>
    <body name="mount" pos="0 0 0.5">  <!-- x in [-0.04, 0.6]: v1 at mount, v16 at slider -->
      <flexcomp name="cable" type="grid" dim="1" count="17 1 1" pos="0.28 0 0"
                spacing="0.04 1 1" radius="0.005" mass="0.06" rgba="0.8 0.25 0.15 1">
        <pin id="0 1"/>  <!-- Clamps edge 0; cable_twist is the last edge's hinge -->
        <edge equality="true" solref="0.003 1" solimp="0.99 0.999 0.001"/>
        <elasticity young="4e6" shear="1.5e6" damping="0.005" section="circle"/>
        <contact condim="3" selfcollide="auto"/>
      </flexcomp>
    </body>
    <body name="slider" pos="0.6 0 0.5">
      <joint name="slide_x" type="slide" axis="1 0 0" damping="0.5"/>
      <geom type="sphere" size="0.01" mass="0.05" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <equality><connect body1="cable_16" body2="slider" anchor="0 0 0" solref="0.002 1"/></equality>
  <actuator><motor name="twist_drive" joint="cable_twist" ctrlrange="-0.005 0.005"/></actuator>
</mujoco>
```

-   **Twist joints & clamping:** Open elastic rods add 1 twist hinge per edge
    (`<name>_twist_0` ... `<name>_twist`, under world) + 3 slides/vertex. `<pin
    id="0 1"/>` clamps edge 0 (omits `_twist_0`); `id="0"` is a ball anchor.
-   **Clamp offset (root `x = 0`, tip `x = L` with `<pin id="0 1"/>`):** Zero
    offset (`x_0 = 0`) clamps edge `0` inside `[0, L]`, making bending ~9-18%
    stiff and torsion `-1/(n - 1)` stiff at 9-17v; place `x_0` behind the root:
    -   *Pure torsion (`<name>_twist`):* **Full-segment offset** `x_0 =
        -spacing` (`spacing = L/(n - 2)`, `grid` `pos` x `= (L - spacing)/2`)
        gives **0% twist error from 9v** (`nv = 28`, as in `clamped_cable`).
    -   *Cantilever bending:* **Half-segment offset** `x_0 = -0.5*spacing`
        (`spacing = L/(n - 1.5)`, `grid` `pos` x `= (L - 0.5*spacing)/2`) gives
        **-1.5% to -0.1% sag error at 9-17v** (use `>= 30` edges if combined).
-   **Sections & rest curvature:** `section="circle"` sets `I, J` and collision
    size from `radius > 0`, so `grid` needs `spacing > 2*radius + margin` (`n <
    L/(2*radius) + 1.5`; use 9-15 `grid` vertices or `direct` if `2*radius`
    nears `L/15.5`). `section="box" sectionsize="hy hz"` sets `(half-y, half-z)`
    along `x` (omit `radius`); `shear` overrides `G = young/(2*(1 + poisson))`.
    `qpos0` bends `> 1e-4 rad` (`coil.xml`) are stress-free; author straight to
    store pre-load energy.
-   **`<composite type="cable">` migration:** `count`/`curve`/`vertex` ->
    `grid`/`circle`/`direct` + `<edge equality="true"/>`; `<geom size="r">` ->
    `radius="r"`; `bend`/`twist` -> `<elasticity young shear damping>`;
    `initial` `none`/`ball`/`free` -> `<pin id="0 1"/>` / `"0"` / unpinned.

### 3.2 -- 2D Cloth, Membranes & Thin Shells (`dim="2"`)

2D flexes decouple collision `2*radius` from `thickness` (stretch `~ thickness`,
bend `D = young * thickness^3 / (12*(1 - poisson^2))`). Use **`type="grid"`**
for cloth/drape, or a **staggered near-equilateral `type="direct"` mesh** when
plate/shell bending accuracy matters (`grid` splits all quads on one diagonal ->
anisotropic ~1.8x stiff hinge bending: -44% vs. -2.6% deflection error,
[`references/benchmarks.md`](references/benchmarks.md)):

```xml
<mujoco model="cloth_drape">
  <option timestep="0.002" integrator="discrete" solver="Newton" tolerance="1e-6" iterations="300"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <geom name="pedestal" type="sphere" size="0.2" pos="0 0 0.2"/>
    <flexcomp name="cloth" type="grid" dim="2" count="15 15 1" pos="0 0 0.5"
              spacing="0.04 0.04 0.04" radius="0.003" mass="0.2" rgba="0.85 0.35 0.25 1">
      <elasticity young="5e4" poisson="0.35" thickness="1.5e-3" elastic2d="both" damping="0.01"/>
      <contact selfcollide="auto" solref="0.008 1" solimp="0.95 0.99 0.0005"/>
    </flexcomp>
  </worldbody>
</mujoco>
```

```python
# Staggered type="direct" plate mesh: odd rows shifted by dx/2, (ny-1)/(nx-1) ~ 2/sqrt(3)
dx, ys = a / (nx - 1), np.linspace(-a / 2, a / 2, ny)  # e.g. nx, ny = 25, 29 (<5% Kirchhoff err)
rows = [(dx * np.arange(nx) if j % 2 == 0 else
         np.r_[0.0, dx * np.arange(nx - 1) + dx / 2, a]) - a / 2 for j in range(ny)]
s, elems = np.cumsum([0] + [len(r) for r in rows]), []
for j in range(ny - 1):
  i = k = 0
  while i < len(rows[j]) - 1 or k < len(rows[j + 1]) - 1:
    step_i = k == len(rows[j + 1]) - 1 or (i < len(rows[j]) - 1 and rows[j][i + 1] <= rows[j + 1][k + 1])
    elems += [s[j] + i, s[j] + i + 1, s[j + 1] + k] if step_i else [s[j] + i, s[j + 1] + k + 1, s[j + 1] + k]
    i, k = i + int(step_i), k + int(not step_i)
point_attr = ' '.join(f'{v:.8g}' for j, r in enumerate(rows) for x in r for v in (x, ys[j], 0.0))
elem_attr = ' '.join(map(str, elems))  # <flexcomp dim="2" type="direct" point=... element=...>
```

-   **`spacing`, `elastic2d` & `thickness`:** Generators reject `spacing < 2 *
    radius + margin` (keep `>= 5 * radius` against fold locking). **`elastic2d`
    defaults to `"none"`** (zero elastic force on `dof="full"` cloth!) and
    `thickness` defaults to `-1` (error): set `"both"`/`"stretch"`/`"bend"` with
    `thickness > 0`.
-   **Interpolated shells (`trilinear`/`quadratic`, `dim="2"` or `"3"`):**
    `elastic2d != "none"` selects boundary-face shell mode and auto-pins
    interior nodes (`bunny_shell.xml`). **Standalone shells need
    `elastic2d="both"`** (`hollow_vs_solid.xml` pairs `"bend"` with `<edge
    equality="strain"/>`; `"bend"` alone has zero membrane stiffness). Bending
    is **explicit** with **no bending damping** (`damping > 0` warns).

### 3.3 -- 3D Volumetric Solids & Elasto-Plasticity (`dim="3"`)

```xml
<mujoco model="soft_and_plastic">
  <option timestep="0.002" integrator="discrete" solver="Newton" tolerance="1e-6" iterations="200"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="sponge_frame" pos="-0.25 0 0.2">  <!-- 2x2x2 trilinear = 81 DOFs -->
      <flexcomp name="sponge" type="grid" dim="3" dof="trilinear" count="6 6 6"
                cellcount="2 2 2" spacing="0.03 0.03 0.03" radius="0.002" mass="0.3" rgba="0.2 0.7 0.7 1">
        <elasticity young="3e4" poisson="0.2" damping="0.01"/>
        <contact selfcollide="none" solref="0.005 1"/>
      </flexcomp>
    </body>
    <body name="clay_frame" pos="0.25 0 0.04">  <!-- pinned iz=0, smooth shading -->
      <flexcomp name="clay" type="grid" dim="3" count="9 9 4" radius="0.002"
                spacing="0.02 0.02 0.02" mass="0.5" flatskin="false" rgba="0.78 0.4 0.22 1">
        <pin gridrange="0 0 0 8 8 0"/>
        <elasticity young="4.5e4" poisson="0.0" damping="0.02" yield="0.002"/>
        <contact condim="1" selfcollide="none" solref="0.004 1" solimp="0.98 0.995 0.001"/>
      </flexcomp>
    </body>
    <body name="stamp" pos="0.25 0 0.115"><freejoint/><geom type="sphere" size="0.04" mass="0.12"/></body>
  </worldbody>
</mujoco>
```

-   **Plasticity (`yield`, `.. _flex-elasticity-yield:`):** Dimensionless von
    Mises (`J2`) yield strain (`0` = elastic) in `data.plastic` (Sec. 6) on
    `full` and `trilinear`/`quadratic` (3D cells & 2D stretch; ignored on
    interpolated `elastic2d="bend"`, disabled by `equality="strain"`). Set
    `flatskin="false"` for smooth shading (`box`/`cylinder`/3D `grid` set
    `"true"`, `<flex>` `"false"`).
-   **Experimental Neo-Hookean (`flex.elastic3d`, `MjSpec`-only):** Set
    `flex.elastic3d = 1` after `make_flex` (`0` = default StVK; `1` = simplified
    Stable Neo-Hookean, `dim=3`, `dof="full"`, `integrator="discrete"` only). No
    MJCF attribute -- `spec.to_file()` raises `FatalError` (cache compiled
    `.mjb` via `mj_saveModel` + `MjModel.from_binary_path`; avoid
    `spec.to_xml()`). Use on `dof="full"` 3D tets under compression to avoid
    StVK softening (`+2.5%` vs. `+9.4%` block error; `P1` tets still lock near
    `poisson = 0.5`, [`references/benchmarks.md`](references/benchmarks.md)).

## 4. Pinning & Rigid-Body Attachments

### Pinning to the Parent Body (`<pin>`, `.. _flexcomp-pin:`)

Pinned points bind to `<flexcomp>`'s **named** parent body (`flex_vertbodyid`/
`flex_nodebodyid`) with **0 extra bodies/DOFs** (pinning all auto-sets
`rigid="true"`). Parent may be `<worldbody>`, a 0-DOF `<body>`, or jointed (when
`>= 1` point is pinned):

-   **`id`/`range` vs. `grid`/`gridrange`:** `<pin id="0 1 2"/>` or `range="0
    8"` (inclusive). `grid`/`gridrange` (`dim="2"`/`"3"` only; fails on `"1"`)
    take `dim` ints/point or `2*dim`/range (`"0 0 14 0"`, `"0 0 0 8 8 0"`).
-   **Interpolated & pruned nodes (`trilinear`/`quadratic`):** `id`/`range`
    index nodes (`z` fastest, `y`, `x`); `grid`/`gridrange` use `0..p*cx` (`p` =
    1 or 2), even on `mesh`/`gmsh`. Cut-cell and shell modes **auto-pin
    empty/interior nodes** -- inspect `[b.name for b in spec.bodies]` first.
-   **Known engine bug on jointed parents
    ([`references/benchmarks.md`](references/benchmarks.md)):**
    `mj_flexPassiveInterp`/`mj_flexPassiveBendInterp` write `R' f` only to each
    node's 3 slides, omitting free nodes' elastic/damping share on parent DOFs
    (~1.2-2.9x servo load; 1D rods exact) and corrupting DOFs if a pinned node
    sits at the jointed parent's `(0, 0, 0)`.

### Attaching Remote Vertices / Nodes (`<connect>`)

Attach remote ends to moving bodies (`model/flex/mechanisms/cantilever.xml`):

```xml
<mujoco model="cantilever_3d">
  <option timestep="0.002" integrator="discrete" solver="Newton" tolerance="1e-6"/>
  <worldbody>
    <body name="clamp" pos="0 0 0.5">  <!-- 17x3x3 (L=0.48 m): ix=0 at clamp -->
      <flexcomp name="beam" type="grid" dim="3" dof="quadratic" count="17 3 3"
                cellcount="2 1 1" spacing="0.03 0.02 0.02" pos="0.24 0 0" radius="0.001" mass="0.15">
        <pin gridrange="0 0 0 0 2 2"/>  <!-- pins all 9 root nodes on ix=0 face -->
        <elasticity young="5e6" poisson="0.25" damping="0.005"/>
        <contact selfcollide="none" contype="0" conaffinity="0"/>
      </flexcomp>
    </body>
    <body name="tip" pos="0.48 0 0.5"><freejoint/><geom type="box" size="0.02 0.02 0.02" mass="0.1"/></body>
  </worldbody>
  <equality>  <!-- 4 end-face corner nodes (ix = 2*cx = 4) lock 6-DOF tip pose -->
    <connect body1="beam_4_0_0" body2="tip" anchor="0 0 0" solref="0.002 1"/>
    <connect body1="beam_4_0_2" body2="tip" anchor="0 0 0" solref="0.002 1"/>
    <connect body1="beam_4_2_0" body2="tip" anchor="0 0 0" solref="0.002 1"/>
    <connect body1="beam_4_2_2" body2="tip" anchor="0 0 0" solref="0.002 1"/>
  </equality>
</mujoco>
```

1.  **Body names, `anchor` & `solref`:** Unpinned vertices create `<name>_<i>`;
    nodes create `<name>_<ix>_<iy>_<iz>`. **`anchor` is in `body1`'s frame**
    (`"0 0 0"` on `<flex_node>`); set `solref="0.002 1"` (`~timestep`) so heavy
    payloads do not droop mm under `0.02 1` (measure deflection at
    `data.flexvert_xpos`, not payload).
2.  **`<connect>` count vs. `<weld>`:** **1** = ball joint; **2** adjacent 1D
    rod vertices = clamped tangent; **3+ non-collinear** (4 corners) = 6-DOF
    pose lock. `<weld>` to a flex vertex/node locks world orientation (3 slides
    only).
3.  **Collision isolation (`contype`/`conaffinity`, NOT `<exclude>`):**
    `<exclude>` **does NOT filter `flex:geom` contacts**. Use `contype="0"
    conaffinity="0"` on fittings (`cantilever.xml`), or `2`/`2` on pads and
    `contype="2" conaffinity="1"` on grasped objects while fingers stay on `1`
    (`gripper.xml`).
4.  **`mocap="true"` rule:** Non-interpolated (`full`/`radial`/`2d`) elastic
    flexes reject `mocap="true"` when **`dim > 1` or `damping > 0`**.
    `trilinear`/`quadratic`, undamped 1D rods, and `<edge equality="true"/>`
    work on mocap; else `<weld>` a free body to the mocap.

## 5. Collisions, Meshing & Memory Sizing

-   **Self-collision & contact caps (`.. _flex-contact:`):** `selfcollide`
    defaults to `"auto"` (`sap` 1D/2D, `bvh` 3D); use `"none"` if non-folding,
    never `"narrow"` (`O(N^2)`). Interpolated self-collision skips same/adjacent
    cells (needs `cellcount >= 3`); self-collisions ignore `margin`/`gap`.
    Interpolated flexes cap at 50 contacts/geom (`mjMAXCONPAIR`). Under heavy
    indenters, use `solref` ~ `2*timestep` + `"0.9999 0.9999 0.001"` `solimp`.
-   **Frictional contacts vs. `passive="true"` vs. `ipc`:**
    -   *Grasping / friction:* Keep `passive="false"` with `<option
        cone="elliptic" impratio="10"/>` and `condim="4"`/`"6" priority="1"`.
    -   *Piled cloth (`passive="true"`, `dim >= 2` non-rigid; ignored on `1`/
        `rigid`):* Frictionless implicit normal penalty in `M_hat` (`discrete` +
        `Newton`/`CG`, `sleep`+`island` ok, `drape.xml`).
    -   *2D IPC (`<flag ipc="enable"/>`):* Experimental frictionless barrier 2D
        contact (`discrete` + `Newton`/`CG`; no `sleep`/`fwdinv`, `bag.xml`).
-   **`type="mesh"` vs. `type="gmsh"`:** `type="mesh"` loads surface triangles
    (`dim="2"` XML default; **Python `make_flex` defaults `dim=3`**, pass `2`!).
    `dim="3"` connects faces to `origin` (star-convex; **omitting `origin`
    errors** unless `<mesh>` has `usertet`). Use `mesh="asset"` for `scale`/
    `refpos`. At `dim="2"`, `trilinear`/`quadratic` embeds triangles directly
    (cut-cell volume if `elastic2d="none"`, shell else); for non-convex `full`
    3D tets, export `type="gmsh"` v4.1/v2.2 (`fTetWild`/`Gmsh`, types `2`/`4`).
-   **Arena sizing (`.. _CSize:`):** Omitting `<size memory>` auto-sizes arena
    memory. Only on `mjWARN_CNSTRFULL`, set `<size memory="100M"/>`
    (`spec.memory = 100 * 1024**2`, not `spec.size.memory`); **never `"4M"`**.

## 6. Procedural Deformables in Python (`MjSpec`) & Runtime State

`body.make_flex(...)` expands `<flexcomp>` on a **named** `MjsBody`
(`spec.worldbody` is `"world"`; a jointless `add_body(name=...)` welds with `0`
extra DOFs) and returns `MjsFlex`:

```python
# MjsBody.make_flex(name, type=None, dim=3, dof=None, count=None, cellcount=None,
#   spacing=None, scale=None, radius=0.0, mass=1.0, inertiabox=0.005, equality=0,
#   rigid=0, flatskin=0, elastic2d=0, pos=None, quat=None, origin=None, file=None,
#   mesh=None, vfs=None) -> MjsFlex  (None -> count=10, cellcount=1, spacing=0.02)
import mujoco, numpy as np
spec = mujoco.MjSpec()
spec.option.timestep, spec.option.tolerance = 0.002, 1e-6
spec.option.integrator = mujoco.mjtIntegrator.mjINT_DISCRETE
spec.option.solver = mujoco.mjtSolver.mjSOL_NEWTON  # spec.memory = 64 * 1024**2
spec.worldbody.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.1])
holder = spec.worldbody.add_body(name='soft_holder', pos=[0, 0, 0.2])  # 0 DOFs
flex = holder.make_flex(
    name='cube', type='grid', dim=3, dof='trilinear', count=[6, 6, 6],
    cellcount=[2, 2, 2], spacing=[0.03, 0.03, 0.03], radius=0.002, mass=0.4,
    equality=0)  # 0: none, 1: edge, 3: strain (2: removed)
flex.young, flex.poisson, flex.damping = 2e4, 0.2, 0.01
flex.selfcollide = mujoco.mjtFlexSelf.mjFLEXSELF_NONE  # flex.solref = [0.004, 1]
model = spec.compile()
data = mujoco.MjData(model)
mujoco.mj_step(model, data, 100)
assert data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number == 0
assert np.all(np.isfinite(data.flexvert_xpos))
```

1.  **`make_flex` traps:** **`dim` defaults to `3`** (pass `dim=2` on cloth/
    surface meshes; 3-vectors `count`, `cellcount`, `spacing`, `scale`, `pos`,
    `origin` need len 3); **`radius` defaults to `0.0`** (`> 0` on circle rods);
    **parent needs `name`**; set `mesh.scale`/`uservert` **before** `make_flex`
    (never both `file=` & `mesh=`); set `flex.flatskin = False` **after** on
    `box`/`cylinder`/3D `grid`.
2.  **`MjsFlex` fields:** Mirror XML except **`stressyield`** (XML `yield`;
    `flex.yield` is a `SyntaxError`!), `elastic3d` (`0`/`1`, Sec. 3.3),
    `elastic2d` = `0`-`3` (`none`/`bend`/`stretch`/`both`), and `equality` =
    `0`/`1`/`3` (`3` adds 1 eq/cell: set `solref` on `name1`=flex).
3.  **Pinning in Python:** `make_flex` has no `pin` arg and omits rod twist
    joints; parse `<flexcomp>` via `MjSpec.from_string(xml)` + **`spec.attach(
    sub, site=..., prefix=...)`** on parent `MjSpec`, or use `spec.add_flex`.

| Expression | Shape / Type | Notes |
| :--- | :--- | :--- |
| `data.flexvert_xpos` | `(model.nflexvert, 3)` | World vertex positions (m) after `mj_forward`/`mj_step`; static-pinned points stay fixed. |
| `data.plastic` | `(model.nplastic,)` | Plastic strain (`yield > 0`) plus 1D rod reference frames and twists; saved by `mjSTATE_PLASTIC` (`mjSTATE_PHYSICS`). |
| `data.contact[i]` | `MjContact` | Flex contacts set `flex`/`elem`/`vert >= 0` (`exclude == 4`, `efc_address == -1` when `passive="true"`). |
| `data.solver[k]` | `MjSolverStat` | Last step stat on island `i` (`n = data.solver_niter[i]`): `k = i*mujoco.mjNSOLVER + n - 1`; check `gradient < opt.tolerance` (vs. `improvement < opt.tolerance` early `CG` exit). |
| `data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number` | `int` | Assert `== 0` in tests to catch transient `QACC` blowups. |

## 7. Batched GPU Simulation (`MJWarp`)

Tracks A and B run on **`MJWarp` (`mujoco_warp`, `impl='warp'`)** in `float32`
(`MJX-JAX` `impl='jax'` does **not** support flex; H100 vs. CPU timings in
[`references/mjwarp.md`](references/mjwarp.md), setup in
[`mujoco-accelerated`](../accelerated/SKILL.md)):

-   **Use for batched rollouts:** At `nworld=64` on H100, all 33 runnable flex
    scenes beat 1 CPU core (`1.3x`-`91x`); at `nworld=1` it is often slower
    (`gripper_2d.xml`: `3.5x` slower at `1`, `8x` faster at `64` -- use CPU
    `mujoco.batch.Batch` / `mujoco.rollout` for small batches).
-   **Integrator & solver regimes:** **`Newton` + `Euler`** is best-supported
    (14/16 scenes `< 4` ms/step in contact). **`Newton` + `discrete`** is within
    `1.3x` of one CPU core (up to `2.8x` faster) at `1` world on 7/16 scenes;
    1D rods (iterative `M+K`, 24-129 PCG iters, `12x`-`37x` slower), cloth/3D
    contact fill-in (`poncho.xml` `104` ms, `gripper.xml` `29` ms), and passive
    cloth hold back the rest. `CG` exits early on `improvement < tolerance`.
-   **Size buffers (`make_data` / `put_data`):** Defaults overflow on 20/33
    scenes (`nefc`, causing `NaN`s); `nconmax >= 16384`, `njmax >= 16384`,
    `njmax_nnz >= 1048576` cleared all 33 (`--measure_alloc` / `d.overflow`).
-   **Not supported:** `ipc`, `dof="quadratic"`, `rigid` flexes, interpolated
    self-collision, flex-`SDF`/`HField`, `sleep` with `discrete`/`<edge
    equality="true"/>`, contact sensor `linvel`/`angvel`, and `discrete` +
    `Newton` with articulated/general attachments (`gripper_trilinear.xml`).

## 8. Quick Troubleshooting Matrix

| Symptom / Error | Root Cause | Fix |
| :--- | :--- | :--- |
| Runtime error on `implicit`, `implicitfast`, `Euler`, or `RK4` | `<elasticity>` rejects `implicit`/`implicitfast`; `passive="true"` and `MjSpec` `flex.elastic3d = 1` reject all non-`discrete` integrators | Set `<option integrator="discrete" solver="Newton"/>`. |
| `"PGS and noslip not yet supported"`, slow `Newton`, or unconverged `CG` | `discrete` flex rejects `PGS`/`noslip_iterations > 0`; `Newton` factors can fill in at very large `nv`, while `CG` often stops early on `improvement < tolerance` with `gradient >> tolerance` | Default to `Newton`; use `CG` only when `Newton` factorization is prohibitive or on soft/`Euler` models, and check `data.solver[n - 1].gradient < opt.tolerance`; keep `noslip_iterations="0"`. |
| `mjWARN_INERTIA` (`"Flex stiffness is too ill-conditioned..."`) | `mj_effSolve` PCG on `M_hat` hit `opt.iterations` (3D `full` or any `trilinear`/`quadratic` flex, including `dim=2` shells), leaving `qacc_smooth` under-converged | Increase `<option iterations="300"/>` (or `"500"`), or reduce `young` / mesh or lattice resolution. |
| Compile error on parent/ancestor joints or joint `damping` | Unpinned `full`/`trilinear`/`quadratic` flex under jointed body, or `<default><joint damping="..."/>` active on flex | Remove parent joints (or `<pin>` >= 1 node; avoid `(0, 0, 0)` on interpolated nodes while the `mj_flexPassiveInterp` bug is open), and scope joint `damping` defaults away from `<flexcomp>`. |
| `"Spacing must be larger than geometry size"` or step-1 `QACC` | Procedural `spacing < 2*radius + margin` (compile error), initial overlap, or `Euler` with `<elasticity damping>` exceeding `2 / (damping * omega_max^2)` | Keep `spacing >= 5*radius`, use `integrator="discrete"` with `<elasticity damping="0.01"/>`, or set `selfcollide="none"`. |
| `dim="2"` zero stiffness, `"positive thickness"`, `"R0 is not orthonormal"`, or `"Pin grid..."` | `elastic2d` defaults to `"none"`; `thickness` defaults to `-1`; flat `dim="2"` interpolated lattice has degenerate 3D bounding box; `<pin grid>` int count mismatch | Set `elastic2d="both"`/`"stretch"` with `thickness > 0`, use `dof="full"` on flat sheets, and pass `dim` ints per `grid` point / `2*dim` per `gridrange`. |
| `"flex constraints and elasticity (young) cannot both be present"` | `<edge equality>` plus `young > 0` on a 2D/3D flex (only 1D rods and `elastic2d="bend"` allow both) | Drop `young` (Track B) or drop `<edge equality>` (Track A). |
| 3D beam or 2D grid plate too stiff; 1D rod stretches; or `"positive radius"` | `trilinear` under-predicts bending deflection by 50-98% and `poisson > 0.45` locks; `type="grid"` uniform-diagonal quads are ~1.8x stiff in hinge bending; `<edge equality>` stretch scales with `timeconst` (`>= 2*timestep` under `refsafe`) and vertex count | Use `dof="quadratic" cellcount="2 1 1"` and `poisson <= 0.45`; use staggered `type="direct"` meshes when plate bending accuracy matters; tighten `<edge equality="true" solref="0.002 1"/>` (`discrete`) and `radius > 0`. |
| Cloth slides off static sphere; or `<connect>` end jitters | `passive="true"`/`ipc` is frictionless; `<exclude>` ignores `flex:geom` pairs so attached geom hits flex | Set `passive="false"` for friction; isolate fittings with `contype`/`conaffinity` bitmasks (`0` or `2`). |
| Heavy rigid body sinks into a fine flex or droops on `<connect>`/`<weld>` | Contact and equality stiffnesses scale with light flex vertex masses (`0.02 1` default `solref` droops mm; default `solimp` penetrates 0.5-1.3 mm) | Set `solref="0.002 1"` on `<connect>`/`<weld>`; for contacts set `solref="0.004 1"` (`~2*timestep`) and `solimp="0.9999 0.9999 0.001"`, or use coarser DOFs. |
| `mjWARN_CNSTRFULL`, or `MJWarp` `d.overflow` / `NaN`s | Heavy contact fill-in exceeded `mjData.arena` (or small `<size memory="4M"/>`); on `MJWarp`, default `make_data` capacities (`nefc`, narrowphase, CCD, `njmax_nnz`) overflowed | Remove small `<size memory>` or raise `<size memory="100M"/>` (`spec.memory` bytes); on `MJWarp`, pass `nconmax >= 16384`, `njmax >= 16384`, `njmax_nnz >= 1048576`. |

## 9. Reference Models ([`model/flex/`](../../../model/flex/))

-   **1D rods & mechanisms:** `cable.xml`, `coil.xml`, `curved_rods.xml`,
    `belt.xml`, `pulley.xml`, `mechanisms/cantilever.xml`,
    `mechanisms/slidercrank.xml`, `mechanisms/dipper.xml`.
-   **2D cloth & shells:** `drape.xml` (`passive="true"`), `poncho*.xml`,
    `flag.xml`, `plate.xml`, `basket.xml` (`type="box" dim="2"`), `bag.xml`
    (`ipc`), `bunny_shell.xml`, `gripper_2d.xml` (`2d`).
-   **3D solids & plasticity:** `trilinear.xml`, `quadratic.xml`, `bunny_*.xml`,
    `strain.xml`, `hollow_vs_solid.xml`, `sphere_*.xml`, `gripper*.xml`,
    `../plastic/clay.xml` (`yield`, `CG` at `nv = 6349`), `jelly.xml`,
    `softbox.xml` (`radial`).
