# MJWarp Flex Benchmarks: CPU vs. H100 GPU (40 Models)

Measured over 1000 steps from `qpos0` (`t = 0 .. 1000 * timestep`, default XML
`<option>` settings) at MuJoCo commit `a2db3285b` and `mujoco_warp` commit
`792775aa5`. Compares single-core C MuJoCo (`float64`) against `mujoco_warp`
(`float32`) on one NVIDIA H100 (80 GB HBM3, 132 SMs) at `nworld=1` and
`nworld=64` with `nconmax=16384`, `njmax=16384`, `njmax_nnz=1048576`.

**When to rerun:** Rerun 1000-step timings across `model/flex/` and
`model/plastic/clay.xml` (and update the summary in `SKILL.md`) whenever
`mujoco_warp` or C MuJoCo changes 1D-rod or general `M+K` metric
preconditioning, sparse `Newton` Cholesky factoring or zero-constraint skips,
default `make_data` flex buffer heuristics, or supported flex features
(`dof="quadratic"`, `ipc`, flex-`SDF`, or articulated `discrete` + `Newton`).

## 1. Decision Rules & Operational Guardrails

-   **Use `MJWarp` for batched rollouts (`nworld >= 64`):** All 33 runnable
    scenes finish 1000 steps with `0` NaNs and beat 1 CPU core at `nworld=64`
    (`1.5x`-`85.6x` higher throughput per world-step). At `nworld=1`, kernel
    launch and single-CTA solver latency often make GPU slower than CPU (e.g.
    `gripper_2d.xml` is `3.0x` slower at `nworld=1`, `6.2x` faster at
    `nworld=64`); prefer `mujoco.batch.Batch` or `mujoco.rollout` on CPU for
    single-world or small-batch runs.
-   **Size buffers explicitly (`make_data` / `put_data`):** Default `make_data`
    capacities overflow on 20 of 33 runnable scenes (15 `nefc`, 4 narrowphase,
    3 CCD, 2 broadphase, 1 `njmax_nnz`), dropping constraints or producing
    `NaN`s. Start with `nconmax=16384`, `njmax=16384`, `njmax_nnz=1048576`
    (clears all 33 scenes) and tighten per scene via `mjwarp-testspeed
    --measure_alloc` and `d.overflow`.
-   **Integrator & solver regimes:**
    -   **`Newton` + `Euler` (Track B, 16 runnable scenes):** Best-supported GPU
        combination -- 14 of 16 scenes run under `4` ms/step at `nworld=1` even
        in sustained contact (`basket.xml` `12.2` ms and `sphere_full.xml` `9.3`
        ms pay Cholesky fill-in at ~900-1270 active contact rows, still `9.9x`
        and `9.5x` faster than 1 CPU core at `nworld=64`).
    -   **`Newton` + `discrete` (Track A, 16 runnable scenes):** Faster than 1
        CPU core at `nworld=1` on 5 scenes (`drape.xml` `3.1x`,
        `bunny_shell.xml` `2.0x`, `strain.xml` `2.0x`, `bunny_multicell.xml`
        `1.6x`, `floppy.xml` `1.2x`) and within `1.6x` on 7 of 16 (adding
        `hollow_vs_solid.xml` and `press.xml`; `17.9x`-`85.6x` faster at
        `nworld=64`). All 6 1D rod scenes use the exact sparse 3x3-blocked
        Cholesky preconditioner (`effChol`, 0 PCG iterations) and run in
        `1.2`-`2.8` ms/step at `nworld=1` (`3.2x`-`13.0x` slower than
        sub-millisecond CPU `mj_step` due to GPU kernel-launch overhead at small
        `nv = 76`-`283`; `1.9x`-`8.0x` faster at `nworld=64`). The remaining 3
        scenes are held back at `nworld=1` by: (1) single-thread `efm0_L`
        triangular solve (`poncho_edgeequality.xml` `123` ms/step), (2) Cholesky
        fill-in on self-colliding cloth and dense 3D contact (`poncho.xml` `100`
        ms/step, `poncho_edgeequality.xml` `123` ms/step, `gripper.xml` `28.6`
        ms/step), and (3) constraint-solver overhead when passive-contact cloth
        has `nefc == 0`.
    -   **`CG` + `discrete`:** Stops early on `improvement < tolerance` with
        scaled gradients `27x`-`3800x` above `tolerance` on flex contacts (same
        as CPU); reserve it for very large `nv` where `Newton` factorization is
        prohibitive (`plastic/clay.xml`, `nv = 6349`: `8.2x` faster at `1`
        world, `41.6x` at `64`) or general `discrete` attachments.
-   **Unsupported on `MJWarp` (`MJX-JAX` supports no flex at all):**
    `dof="quadratic"` (`mechanisms/cantilever.xml`, `quadratic.xml`,
    `bunny_quadratic.xml`), `<flag ipc="enable"/>` (`bag.xml`, `bag_hoop.xml`),
    flex vs. `SDF` (`cloth_sdf.xml`) or `HField` geoms, `rigid="true"` flexes,
    interpolated self-collision, `sleep` with `discrete` flexes or `<edge
    equality="true"/>`, flex contact sensors reading `linvel`/`angvel`, and
    `discrete` + `Newton` with general attachments or interpolated nodes on
    non-3-DOF-slider bodies (`gripper_trilinear.xml`; requires `solver="CG"`).

## 2. Per-Scene 1000-Step Timings (`model/flex/` & `model/plastic/clay.xml`)

Columns report: active-contact step share (`Con%`), 1000-step single-core C
`ms/step`, H100 `nworld=1` overall and sustained (`steps 500..999`) `ms/step`,
overall `nworld=1` ratio vs. C, H100 `nworld=64` `ms / world-step` and
throughput speedup vs. 1 C core (`> 1x` = faster than C), and which default
`make_data(model)` buffers overflowed without explicit sizing.

### 2.1 `Newton` + `discrete` (21 Scenes)

*Measured at MuJoCo `a2db3285b`, `mujoco_warp` `792775aa5` (1000 steps, H100).*

| Scene (`model/flex/...`) | nv | Con% | C ms | MJW nw=1 (500+) ms | nw=1 vs. C | MJW nw=64 ms/w (xC) | Default `make_data` |
| :--- | ---: | ---: | ---: | ---: | :--- | :--- | :--- |
| `bag.xml` | 2049 | 100% | 45.67 | -- | Unsupported (`ipc`) | -- | -- |
| `bag_hoop.xml` | 2049 | 100% | 48.62 | -- | Unsupported (`ipc`) | -- | -- |
| `belt.xml` | 76 | 0% | 0.10 | 1.36 (1.36) | 13.0x slower | 0.056 (1.9x) | ok |
| `bunny_multicell.xml` | 178 | 96% | 3.34 | 2.14 (2.11) | 1.6x faster | 0.074 (45.2x) | `njmax_nnz`, narrowphase |
| `bunny_quadratic.xml` | 82 | 87% | 2.13 | -- | Unsupported (`quadratic`) | -- | -- |
| `bunny_shell.xml` | 169 | 96% | 4.44 | 2.18 (2.08) | 2.0x faster | 0.073 (60.6x) | `nefc`, narrowphase |
| `cable.xml` | 157 | 100% | 0.32 | 1.86 (1.86) | 5.9x slower | 0.070 (4.5x) | ok |
| `coil.xml` | 280 | 100% | 0.64 | 2.70 (2.66) | 4.2x slower | 0.091 (7.0x) | ok |
| `curved_rods.xml` | 283 | 89% | 0.78 | 2.77 (2.76) | 3.6x slower | 0.097 (8.0x) | ok |
| `drape.xml` | 1521 | 94% | 51.84 | 16.96 (14.72) | 3.1x faster | 0.606 (85.6x) | broadphase |
| `floppy.xml` | 1153 | 70% | 7.52 | 6.17 (7.40) | 1.2x faster | 0.251 (30.0x) | ok |
| `gripper.xml` | 1047 | 100% | 2.66 | 28.58 (28.58) | 10.8x slower | 0.598 (4.4x) | `nefc` |
| `gripper_trilinear.xml` | 33 | 100% | 0.36 | -- | Unsupported (`step` attachment) | -- | -- |
| `hollow_vs_solid.xml` | 594 | 95% | 2.35 | 3.23 (3.21) | 1.4x slower | 0.099 (23.7x) | `nefc` |
| `mechanisms/cantilever.xml` | 436 | 0% | 15.93 | -- | Unsupported (`quadratic`) | -- | -- |
| `mechanisms/dipper.xml` | 106 | 0% | 0.42 | 1.35 (1.35) | 3.2x slower | 0.055 (7.6x) | ok |
| `mechanisms/slidercrank.xml` | 87 | 0% | 0.20 | 1.20 (1.20) | 6.1x slower | 0.052 (3.8x) | ok |
| `poncho.xml` | 1458 | 100% | 24.33 | 99.63 (108.82) | 4.1x slower | 1.682 (14.5x) | broadphase |
| `poncho_edgeequality.xml` | 1458 | 100% | 25.81 | 122.78 (124.30) | 4.8x slower | 2.286 (11.3x) | `nefc` |
| `press.xml` | 386 | 78% | 1.21 | 1.91 (1.87) | 1.6x slower | 0.068 (17.9x) | ok |
| `strain.xml` | 54 | 83% | 3.92 | 1.93 (1.84) | 2.0x faster | 0.210 (18.6x) | `nefc`, CCD |

### 2.2 `CG` + `discrete` (1 Scene)

*Measured at MuJoCo `a2db3285b`, `mujoco_warp` `792775aa5` (1000 steps, H100).*

| Scene | nv | Con% | C ms | MJW nw=1 (500+) ms | nw=1 vs. C | MJW nw=64 ms/w (xC) | Default `make_data` |
| :--- | ---: | ---: | ---: | ---: | :--- | :--- | :--- |
| `model/plastic/clay.xml` | 6349 | 4% | 30.13 | 3.68 (2.89) | 8.2x faster | 0.725 (41.6x) | CCD |

### 2.3 `Newton` + `Euler` (18 Scenes)

*Measured at MuJoCo `a2db3285b`, `mujoco_warp` `792775aa5` (1000 steps, H100).*

| Scene (`model/flex/...`) | nv | Con% | C ms | MJW nw=1 (500+) ms | nw=1 vs. C | MJW nw=64 ms/w (xC) | Default `make_data` |
| :--- | ---: | ---: | ---: | ---: | :--- | :--- | :--- |
| `basket.xml` | 1050 | 100% | 3.26 | 12.20 (16.94) | 3.7x slower | 0.330 (9.9x) | `nefc` |
| `bunny.xml` | 25 | 74% | 0.60 | 1.43 (0.98) | 2.4x slower | 0.046 (13.0x) | `nefc`, narrowphase |
| `bunny_with_uv.xml` | 25 | 74% | 1.04 | 1.42 (0.97) | 1.4x slower | 0.046 (22.7x) | `nefc`, narrowphase |
| `cloth_sdf.xml` | 198 | 91% | 2.55 | -- | Unsupported (flex-`SDF`) | -- | -- |
| `flag.xml` | 513 | 25% | 1.28 | 2.64 (3.95) | 2.1x slower | 0.096 (13.3x) | `nefc` |
| `gripper_2d.xml` | 201 | 92% | 0.40 | 1.18 (1.18) | 3.0x slower | 0.065 (6.2x) | ok |
| `jelly.xml` | 1536 | 65% | 1.25 | 1.44 (1.39) | 1.2x slower | 0.118 (10.6x) | ok |
| `pancake.xml` | 243 | 68% | 0.75 | 0.88 (0.89) | 1.2x slower | 0.046 (16.4x) | `nefc` |
| `pinch.xml` | 246 | 93% | 0.37 | 0.98 (0.98) | 2.6x slower | 0.048 (7.7x) | `nefc` |
| `plate.xml` | 768 | 80% | 2.13 | 3.04 (2.97) | 1.4x slower | 0.114 (18.7x) | `nefc` |
| `pulley.xml` | 121 | 0% | 0.06 | 0.72 (0.72) | 11.4x slower | 0.042 (1.5x) | ok |
| `quadratic.xml` | 81 | 15% | 0.46 | -- | Unsupported (`quadratic`) | -- | -- |
| `softbox.xml` | 224 | 77% | 0.90 | 1.06 (1.07) | 1.2x slower | 0.050 (17.9x) | `nefc` |
| `sphere_full.xml` | 891 | 75% | 2.27 | 9.27 (9.18) | 4.1x slower | 0.238 (9.5x) | `nefc` |
| `sphere_radial.xml` | 302 | 75% | 1.53 | 2.04 (2.15) | 1.3x slower | 0.077 (19.7x) | `nefc` |
| `sphere_trilinear.xml` | 48 | 75% | 0.94 | 0.90 (0.95) | 1.05x faster | 0.059 (16.0x) | CCD |
| `trampoline.xml` | 762 | 55% | 1.12 | 1.01 (1.05) | 1.1x faster | 0.060 (18.7x) | ok |
| `trilinear.xml` | 24 | 38% | 0.29 | 0.50 (0.51) | 1.7x slower | 0.028 (10.4x) | ok |
