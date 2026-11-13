# MJWarp Flex Benchmarks: CPU vs. H100 GPU (39 Models)

Measured over 1000 steps from `qpos0` (`t = 0 .. 1000 * timestep`, default XML
`<option>` settings) at MuJoCo commit `18c7e3338` and `mujoco_warp` commit
`8d649ced4`. Compares single-core C MuJoCo (`float64`) against `mujoco_warp`
(`float32`) on one NVIDIA H100 (80 GB HBM3) at `nworld=1` and `nworld=64` with
`nconmax=16384`, `njmax=16384`, `njmax_nnz=1048576`.

**When to rerun:** Rerun 1000-step timings across `model/flex/` and
`model/plastic/clay.xml` (and update the summary in `SKILL.md`) whenever
`mujoco_warp` changes 1D-rod or general `M+K` metric preconditioning, sparse
`Newton` Cholesky factoring or zero-constraint skips, default `make_data` flex
buffer heuristics, or supported flex features (`dof="quadratic"`, `ipc`,
flex-`SDF`, or articulated `discrete` + `Newton`).

## 1. Decision Rules & Operational Guardrails

-   **Use `MJWarp` for batched rollouts (`nworld >= 64`):** All 33 runnable
    scenes finish 1000 steps with `0` NaNs and beat 1 CPU core at `nworld=64`
    (`1.3x`-`90.9x` higher throughput per world-step). At `nworld=1`, kernel
    launch and single-CTA solver latency often make GPU slower than CPU (e.g.
    `gripper_2d.xml` is `3.5x` slower at `nworld=1`, `8.0x` faster at
    `nworld=64`); prefer `mujoco.batch.Batch` or `mujoco.rollout` on CPU for
    single-world or small-batch runs.
-   **Size buffers explicitly (`make_data` / `put_data`):** Default `make_data`
    capacities overflow on 20 of 33 runnable scenes (15 `nefc`, 4 narrowphase,
    4 CCD, 2 broadphase, 1 `njmax_nnz`), dropping constraints or producing
    `NaN`s. Start with `nconmax=16384`, `njmax=16384`, `njmax_nnz=1048576`
    (clears all 33 scenes) and tighten per scene via `mjwarp-testspeed
    --measure_alloc` and `d.overflow`.
-   **Integrator & solver regimes:**
    -   **`Newton` + `Euler` (Track B, 16 runnable scenes):** Best-supported GPU
        combination -- 14 of 16 scenes run under `4` ms/step at `nworld=1` even
        in sustained contact (`basket.xml` `12.3` ms and `sphere_full.xml` `9.3`
        ms pay Cholesky fill-in at ~900-1270 active contact rows, still `10.0x`
        and `15.5x` faster than 1 CPU core at `nworld=64`).
    -   **`Newton` + `discrete` (Track A, 16 runnable scenes):** Within `1.3x`
        of one CPU core (up to `2.8x` faster at `nworld=1`; `19.0x`-`90.9x` at
        `nworld=64`) on 7 of 16 scenes (`drape.xml`, `strain.xml`,
        `bunny_shell.xml`, `bunny_multicell.xml`, `floppy.xml`, `press.xml`,
        `hollow_vs_solid.xml`). At `8d649ced4`, the other 9 scenes are held back
        at `nworld=1` by: (1) 1D rods using iterative `M+K` PCG (24-129
        iterations, `11.8x`-`37.3x` slower than CPU direct banded Cholesky at
        `1` world; `1.3x`-`3.7x` faster at `64`), (2) Cholesky fill-in on
        self-colliding cloth and dense 3D contact (`poncho.xml` `104` ms/step,
        `poncho_edgeequality.xml` `123` ms/step, `gripper.xml` `28.6` ms/step),
        and (3) constraint-solver overhead when passive-contact cloth has
        `nefc == 0`.
    -   **`CG` + `discrete`:** Stops early on `improvement < tolerance` with
        scaled gradients `27x`-`3800x` above `tolerance` on flex contacts (same
        as CPU); reserve it for very large `nv` where `Newton` factorization is
        prohibitive (`plastic/clay.xml`, `nv = 6349`: `4.7x` faster at `1`
        world, `74.7x` at `64`) or general `discrete` attachments.
-   **Unsupported on `MJWarp` (`MJX-JAX` supports no flex at all):**
    `dof="quadratic"` (`mechanisms/cantilever.xml`, `quadratic.xml`,
    `bunny_quadratic.xml`), `<flag ipc="enable"/>` (`bag.xml`), flex vs. `SDF`
    (`cloth_sdf.xml`) or `HField` geoms, `rigid="true"` flexes, interpolated
    self-collision, `sleep` with `discrete` flexes or `<edge equality="true"/>`,
    flex contact sensors reading `linvel`/`angvel`, and `discrete` + `Newton`
    with general attachments or interpolated nodes on non-3-DOF-slider bodies
    (`gripper_trilinear.xml`; requires `solver="CG"`).

## 2. Per-Scene 1000-Step Timings (`model/flex/` & `model/plastic/clay.xml`)

Columns report: active-contact step share (`Con%`), 1000-step single-core C
`ms/step`, H100 `nworld=1` overall and sustained (`steps 500..999`) `ms/step`,
overall `nworld=1` ratio vs. C, H100 `nworld=64` `ms / world-step` and
throughput speedup vs. 1 C core (`> 1x` = faster than C), and which default
`make_data(model)` buffers overflowed without explicit sizing.

### 2.1 `Newton` + `discrete` (20 Scenes)

*Measured at MuJoCo `18c7e3338`, `mujoco_warp` `8d649ced4` (1000 steps, H100).*

| Scene (`model/flex/...`) | nv | Con% | C ms | MJW nw=1 (500+) ms | nw=1 vs. C | MJW nw=64 ms/w (xC) | Default `make_data` |
| :--- | ---: | ---: | ---: | ---: | :--- | :--- | :--- |
| `bag.xml` | 2049 | 100% | 74.58 | -- | Unsupported (`ipc`) | -- | -- |
| `belt.xml` | 76 | 0% | 0.12 | 2.70 (2.73) | 22.1x slower | 0.062 (2.0x) | ok |
| `bunny_multicell.xml` | 178 | 96% | 1.89 | 2.14 (2.12) | 1.1x slower | 0.060 (31.8x) | `njmax_nnz`, narrowphase |
| `bunny_quadratic.xml` | 82 | 87% | 1.33 | -- | Unsupported (`quadratic`) | -- | -- |
| `bunny_shell.xml` | 169 | 96% | 3.27 | 2.22 (2.12) | 1.5x faster | 0.061 (53.2x) | `nefc`, narrowphase |
| `cable.xml` | 157 | 100% | 0.32 | 3.72 (3.73) | 11.8x slower | 0.087 (3.7x) | ok |
| `coil.xml` | 280 | 100% | 0.70 | 13.81 (13.78) | 19.6x slower | 0.272 (2.6x) | ok |
| `curved_rods.xml` | 283 | 89% | 0.79 | 11.17 (11.16) | 14.2x slower | 0.226 (3.5x) | ok |
| `drape.xml` | 1521 | 94% | 56.77 | 20.11 (21.37) | 2.8x faster | 0.625 (90.9x) | broadphase |
| `floppy.xml` | 1153 | 70% | 5.42 | 6.21 (7.32) | 1.1x slower | 0.235 (23.1x) | ok |
| `gripper.xml` | 1047 | 100% | 1.95 | 28.57 (28.57) | 14.7x slower | 0.576 (3.4x) | `nefc` |
| `gripper_trilinear.xml` | 33 | 100% | 0.19 | -- | Unsupported (`step` attachment) | -- | -- |
| `hollow_vs_solid.xml` | 594 | 95% | 2.49 | 3.27 (3.26) | 1.3x slower | 0.087 (28.7x) | `nefc` |
| `mechanisms/cantilever.xml` | 436 | 0% | 10.45 | -- | Unsupported (`quadratic`) | -- | -- |
| `mechanisms/dipper.xml` | 106 | 0% | 0.22 | 8.23 (9.36) | 37.3x slower | 0.169 (1.3x) | ok |
| `mechanisms/slidercrank.xml` | 87 | 0% | 0.18 | 4.35 (4.55) | 23.9x slower | 0.095 (1.9x) | ok |
| `poncho.xml` | 1458 | 100% | 35.00 | 103.86 (114.87) | 3.0x slower | 1.635 (21.4x) | broadphase |
| `poncho_edgeequality.xml` | 1458 | 100% | 30.35 | 122.80 (124.97) | 4.0x slower | 2.269 (13.4x) | `nefc` |
| `press.xml` | 386 | 78% | 1.53 | 1.93 (1.89) | 1.3x slower | 0.052 (29.4x) | ok |
| `strain.xml` | 54 | 83% | 3.57 | 1.93 (1.85) | 1.9x faster | 0.188 (19.0x) | `nefc`, CCD |

### 2.2 `CG` + `discrete` (1 Scene)

*Measured at MuJoCo `18c7e3338`, `mujoco_warp` `8d649ced4` (1000 steps, H100).*

| Scene | nv | Con% | C ms | MJW nw=1 (500+) ms | nw=1 vs. C | MJW nw=64 ms/w (xC) | Default `make_data` |
| :--- | ---: | ---: | ---: | ---: | :--- | :--- | :--- |
| `model/plastic/clay.xml` | 6349 | 4% | 52.72 | 11.14 (12.80) | 4.7x faster | 0.706 (74.7x) | CCD |

### 2.3 `Newton` + `Euler` (18 Scenes)

*Measured at MuJoCo `18c7e3338`, `mujoco_warp` `8d649ced4` (1000 steps, H100).*

| Scene (`model/flex/...`) | nv | Con% | C ms | MJW nw=1 (500+) ms | nw=1 vs. C | MJW nw=64 ms/w (xC) | Default `make_data` |
| :--- | ---: | ---: | ---: | ---: | :--- | :--- | :--- |
| `basket.xml` | 1050 | 100% | 3.06 | 12.30 (17.15) | 4.0x slower | 0.306 (10.0x) | `nefc` |
| `bunny.xml` | 25 | 74% | 0.75 | 1.42 (0.97) | 1.9x slower | 0.037 (20.5x) | `nefc`, narrowphase |
| `bunny_with_uv.xml` | 25 | 74% | 0.68 | 1.43 (0.98) | 2.1x slower | 0.037 (18.5x) | `nefc`, narrowphase |
| `cloth_sdf.xml` | 198 | 91% | 2.98 | -- | Unsupported (flex-`SDF`) | -- | -- |
| `flag.xml` | 513 | 25% | 1.64 | 2.61 (3.88) | 1.6x slower | 0.081 (20.3x) | `nefc` |
| `gripper_2d.xml` | 201 | 92% | 0.34 | 1.20 (1.20) | 3.5x slower | 0.043 (8.0x) | ok |
| `jelly.xml` | 1536 | 65% | 1.61 | 1.46 (1.41) | 1.1x faster | 0.104 (15.5x) | ok |
| `pancake.xml` | 243 | 68% | 0.53 | 0.90 (0.90) | 1.7x slower | 0.031 (17.0x) | `nefc` |
| `pinch.xml` | 246 | 93% | 0.52 | 0.98 (0.98) | 1.9x slower | 0.032 (15.9x) | `nefc` |
| `plate.xml` | 768 | 80% | 2.49 | 3.02 (2.95) | 1.2x slower | 0.097 (25.8x) | `nefc` |
| `pulley.xml` | 121 | 0% | 0.07 | 0.74 (0.73) | 10.8x slower | 0.028 (2.4x) | ok |
| `quadratic.xml` | 81 | 15% | 0.30 | -- | Unsupported (`quadratic`) | -- | -- |
| `softbox.xml` | 224 | 77% | 1.02 | 1.07 (1.07) | 1.05x slower | 0.036 (28.4x) | `nefc` |
| `sphere_full.xml` | 891 | 75% | 3.51 | 9.26 (9.12) | 2.6x slower | 0.226 (15.5x) | `nefc` |
| `sphere_radial.xml` | 302 | 75% | 2.06 | 2.02 (2.13) | 1.02x faster | 0.062 (33.0x) | `nefc` |
| `sphere_trilinear.xml` | 48 | 75% | 0.53 | 0.91 (0.96) | 1.7x slower | 0.044 (12.2x) | CCD |
| `trampoline.xml` | 762 | 55% | 0.77 | 1.01 (1.06) | 1.3x slower | 0.043 (17.7x) | ok |
| `trilinear.xml` | 24 | 38% | 0.25 | 0.50 (0.50) | 2.0x slower | 0.018 (13.8x) | ok |
