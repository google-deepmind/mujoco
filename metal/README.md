# Experimental native Metal physics

An installable, community experimental physics package for Apple Silicon. It
ports the existing Microduck Metal pipeline into a standalone package with an
owned-state API, CPU preflight, reset/restore, explicit failure checks, bundled
model resources and numerical qualification tests. It does not require mjlab,
RSL-RL, BAM, JAX or a sibling checkout. MuJoCo's normal CPU build and Python
package do not import or depend on it.

**The first supported profile is `microduck-flat-v1`, not arbitrary MJCF.** It has
14 direct-torque actuators, 21 positions, 20 velocities and 17 bodies. The kernels
still specialize this topology. Model/option changes are rejected before GPU
allocation instead of silently running a partially supported model.

This is an implementation under review, not official MuJoCo Metal support. New
GPU qualification of this port is pending. The downstream implementation was
previously exercised on an M1 Max (32 GB unified memory, 24-core GPU); this is not
a guarantee of accuracy, speed or policy quality for the standalone port.

## Install and inspect

From this repository's root, create a **new** Python 3.12 environment. The package
pins MuJoCo 3.10.0 and Torch 2.9.1, matching its numerical reference. It does not
build against the newer MuJoCo source in this checkout. Using a newer MuJoCo
version is rejected until its changed physics semantics have been qualified.

```bash
uv venv .venv-metal --python 3.12
uv pip install --python .venv-metal/bin/python './metal[metal,test]'
.venv-metal/bin/python -m mujoco_metal preflight
```

Preflight loads the CPU model and reports dimensions, installed module/shader
paths, a shader SHA256 and model fingerprint. It does not import Torch, compile
a shader or allocate GPU physics. Assets and shaders are included in the wheel;
no runtime downloads occur. CPU-only users can install `./metal` without extras
for model loading and preflight.

On an **idle Apple Silicon GPU**, explicitly run:

```bash
.venv-metal/bin/python -m mujoco_metal smoke --batch-size 4 --steps 10
```

This advances zero motor torque, checks statuses and finite state, then exercises
reset/restore. It does not run a walking controller, optimizer or benchmark.
macOS with working Torch MPS and `torch.mps.compile_shader` is required.

## Batched physics API

```python
import torch
from mujoco_metal import Simulation, load_model

model = load_model()                 # independent CPU reference; fixed profile
sim = Simulation(model, batch_size=4)
ctrl = torch.zeros((4, 14), device="mps", dtype=torch.float32)
result = sim.step(ctrl)              # one fixed 0.005 s physics step
positions = sim.qpos                 # device copy, shape (4, 21)
velocities = sim.qvel                # device copy, shape (4, 20)
saved = sim.state_dict()             # independent NumPy snapshot, GPU fence
sim.reset(env_ids=[1, 3])            # selected worlds, standing keyframe
sim.load_state_dict(saved)           # validates full snapshot before changing state
```

Controls are motor torques, not joint-position targets. `step` checks contact and
constraint overflow, factorization, solver and integration status, and nonfinite
state before committing positions/velocities. Invalid results leave owned state
unchanged. PGS status `1` means a finite, bounded iteration-limited solve; it is
returned in diagnostics. Pass `require_convergence=True` to reject it as well.
The smoke command counts these unconverged world-steps explicitly.

`contact_capacity` is in `[1, 35]`; `constraint_capacity` is a multiple of four in
`[4, 128]`, default 128. Insufficient capacity raises an error; contacts are not
silently truncated. Simulation calls must be serialized per instance. Diagnostics
alias persistent scratch buffers and are overwritten by later steps; call
`result.clone()` to retain them. Position/velocity properties return copies to
prevent accidental mutation of owned state. Snapshots include a format version,
profile, model fingerprint, batch size and simulation time. They are physics
snapshots, not training checkpoints, and are not compatible with downstream
full-environment checkpoints.

Reset inputs are finite CPU-compatible arrays, shaped `(selected_rows, 21)` and
`(selected_rows, 20)` with unit root quaternions. Empty integer selections are
allowed; duplicate, noninteger or out-of-range IDs are rejected. Missing reset
positions use the standing keyframe; missing velocities are zero.

The safe stepping API synchronizes for status checks. Shared physical memory does
not remove synchronization costs. This API is a correctness-first standalone
boundary; it does not claim the downstream trainer's throughput.

## Supported physics and limits

- Native Metal: kinematics, articulated dynamics, Cholesky factorization,
  convex-foot/plane contact generation, pyramidal contact constraints, joint
  limits/friction loss, PGS solve and implicit-fast integration.
- The bundled public profile **explicitly disables self-collision and all
  non-foot ground contacts**, in both GPU configuration and CPU reference.
  Its contact set is the two original CAD soles against the horizontal plane.
  A fallen robot can pass through the ground except at its feet. This is not
  the complete downstream training collision model.
- Float32 device arithmetic; MuJoCo 3.10.0 CPU references use its standard double
  precision wheel. No double-precision Metal path is provided. The guide's full
  single/double precision qualification is not satisfied by these Python tests.
- Fixed topology, joint placement, gravity, timestep, solver configuration,
  actuator mapping/gains and ground geometry. Exact compiled-model checks reject
  edits, even parameter edits that the private core can represent. General model
  loading and runtime domain randomization are not public features yet.
- No sensors, renderer, learning algorithm, trained policy, ONNX export,
  real-robot deployment, differentiation API, terrain generalization, fluid,
  flex, tendon or equality-constraint support in this standalone API.
- Linux and Windows can perform CPU preflight/tests; physics requires Apple
  Silicon MPS. Other Macs and OS configurations remain unqualified.
- The low-level modules prefixed `_` are research internals retained to run the
  reference corpus. Their optional per-world buffers and oracle inputs do not
  constitute a general model-support promise.

## Validate

The [port validation record](VALIDATION.md) reports 97 CPU checks passing
against both source and wheel installs, with 165 GPU cases explicitly skipped.


The CPU suite covers installed resource loading, actual CPU stepping, support
rejection, reset/restore ownership and failure atomicity, status gates, reference
contact/constraint formulas and deliberate qualification-harness failures.
It blocks shader compilation and MPS synchronization in CPU-marked tests.

```bash
.venv-metal/bin/python -m pytest -q metal/tests
```

GPU tests are skipped by default. Only on an idle Apple Silicon machine:

```bash
.venv-metal/bin/python -m pytest -q metal/tests --run-metal
.venv-metal/bin/python -m mujoco_metal smoke --batch-size 4 --steps 10
```

The GPU suite contains the ported 25-scenario contact/constraint corpus,
integration/trajectory tests, and new public-API reset/continuation/overflow
checks. CPU helper success, collection, shader source equality and successful
installation **do not establish GPU physics correctness**. Run these commands
before relying on the port. The new API/profile tests have not yet run on GPU.
No new performance benchmark or speedup is claimed by this contribution.

For a wheel rather than a source install:

```bash
uv build ./metal --out-dir /tmp/mujoco-metal-dist
uv pip install --python .venv-metal/bin/python --reinstall \
  '/tmp/mujoco-metal-dist/mujoco_metal_experimental-0.1.0-py3-none-any.whl[metal,test]'
```

## Provenance and contribution

Ported from [Microduck RL Mac at 60f0b26](https://github.com/keeeeenw/microduck_rl_mac/tree/60f0b26a55e3236d2112126386b25ac922228083).
The shader differs only in trailing whitespace cleanup. The numerical Python core has
import relocation and formatting changes; the new API/profile is separately
validated on CPU and awaits GPU qualification. The original corpus uses the
original canonical model, whereas public-API tests use the explicitly restricted
foot-ground profile described above.

MuJoCo supplies the original physics methods and CPU references. Pollen Robotics
supplies the Microduck robot and assets. See [NOTICE](NOTICE) and [LICENSE](LICENSE).
Community reports and pull requests are welcome. Include package commit, chip,
memory, OS, dependency versions, exact command and all numerical failure/status
results. Keep correctness qualification separate from synchronized, controlled
performance comparisons. This experiment can be installed from this branch
regardless of the outcome of upstream review.
