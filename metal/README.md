# Experimental MuJoCo Metal package

**Current status: physics computations, not a complete simulation backend.**
This generalized branch computes kinematics, `M(q)`, and inertial/gravity bias
on Metal. It does not yet solve for acceleration, integrate state, or implement
`mj_step`. The earlier robot-specific implementation is preserved on
[`archive/metal-microduck-v1`](https://github.com/keeeeenw/mujoco/tree/archive/metal-microduck-v1);
its restricted stepping pipeline has not been generalized into this package.

This optional package targets Python 3.12 and MuJoCo **3.10.0**. The surrounding MuJoCo source checkout is 3.14.1; that version is not a target. Use an isolated environment so the pinned package does not alter the surrounding checkout:

```sh
cd metal
python3.12 -m venv .venv-metal
source .venv-metal/bin/activate
python -m pip install -e '.[metal,test]'
```

For CPU-only utilities, replace the install extra with `.[test]`. Importing `mujoco_metal` and running preflight do not import Torch or initialize MPS.

Run `python -m mujoco_metal preflight --model path/to/model.xml --json --inventory` to inspect runtime version, model dimensions, package and shader paths/hash, capability boundaries, and the versioned feature/API inventory. Inventory completeness is explicitly false because the captured enum and Python binding inventory is not exhaustive.

`load_model(xml_or_path)` returns an immutable, dimension-derived descriptor. `descriptor.forward_kinematics(qpos)` is a CPU reference for body, inertial, geom, site, and joint-anchor/axis world poses across hinge, slide, ball, and free joints. `MetalKinematics(descriptor).run(qpos_batch)` computes the same pose fields through the batched native Metal kinematics kernel. GPU checks have passed on an Apple M1 for empty and fixed worlds, mixed hinge/slide/ball/free models with off-center joints and multiple free roots, world-attached sites/geoms, and a 32-DOF chain. This is a narrow correctness qualification, not a general model-support or performance claim. Constructing `MetalKinematics` initializes MPS and compiles the bundled shader.

`smooth_dynamics(descriptor, qpos, qvel)` is the CPU reference returning a dense joint-space mass matrix and inertial/gravity bias forces. `MetalSmoothDynamics(descriptor).run(qpos_batch, qvel_batch)` returns those two outputs as MPS tensors. Native GPU checks have passed on an Apple M1 across empty/fixed, mixed-joint, rotated-inertia, massless-ancestor, disabled-gravity, and 32-DOF-chain fixtures. This is a narrow correctness qualification for the smooth M and bias stages, not a general model-support or performance claim. Each batch shares one immutable model descriptor; per-environment native mass randomization is not connected to the CPU lifecycle utilities. Inputs are host NumPy arrays and outputs remain MPS tensors. Models with actuators or nonzero tendon armature are rejected. Actuator forces/armature, tendon armature, passive forces, contacts, constraints, sensors, integration, and stepping are outside this stage.

`ModelLifecycle` performs transactional CPU body-mass updates through MuJoCo `mj_setConst`. `BatchedConstants` maintains per-environment body masses and derived `body_invweight0` rows with atomic recomputation/restore and seeded mass randomization. `KinematicsBatchState` tracks explicit environment rows with generation-based FK cache invalidation, snapshots, restore, and tangent-space joint randomization. These are CPU lifecycle utilities and do not advance physics.

Run the opt-in GPU correctness tests only on an available Apple GPU with the pinned Torch extra installed: `MUJOCO_METAL_RUN_GPU=1 python -m pytest -m gpu`. Ordinary `python -m pytest` runs CPU tests and skips the GPU cases. The standalone source tree carries the Apache 2.0 license and notices.

## Gaps before full simulation

| Stage | Status in this generalized package |
| --- | --- |
| Kinematics, dense mass matrix, inertial/gravity bias | Native Metal; qualified on the documented small fixtures. |
| Mass factorization, linear solve, generalized acceleration | Missing. Producing `M` and bias does not solve the dynamics equation. |
| Persistent device state, time advancement, quaternion-aware integration | Missing. No generalized `step`, `mj_step1` or `mj_step2` equivalent. |
| Applied forces, passive forces, actuators and tendon dynamics | Not integrated into a complete force/acceleration pipeline. Actuator models and nonzero tendon armature are rejected by the smooth stage. |
| Collision/contact generation, joint limits, equality constraints, friction and constraint solvers | Missing. A contact-free pendulum demonstration would not qualify these features. |
| Device reset/checkpoint lifecycle and per-environment model randomization | CPU utilities exist; they are not connected to a persistent native simulation loop. |
| Sensors, remaining integrators, flexes/plugins, broad API and precision compatibility | Unimplemented or unqualified; full MuJoCo coverage is not established. |
| Native rendering and end-to-end training integration | Outside the implemented scope. |

## Runnable Mac demo

The [side-by-side chaotic pendulum demo](examples/README.md) now advances a
four-hinge, contact-free model with **Metal mass/bias plus CPU solve and
integration**, alongside an independent CPU MuJoCo reference. Its 200-step
rollout and reset checks pass on the local Apple M1; maximum position error
on the initial rollout was approximately 2.2e-8 radians. The interactive
viewer uses OpenGL and must be launched with `mjpython` on macOS.

This is a working **hybrid demonstration**, not a native Metal solve/integrator,
full simulation port, contact qualification or performance result. See the
example instructions for launch, numerical checks and limitations.

## FAQ: MuJoCo, Metal, and Apple Silicon

Checked on **2026-09-26**. These answers distinguish upstream documentation,
version-specific issue reports, community projects, and this package's local
validation. External projects were not installed or benchmarked for this FAQ.
Our numerical contract remains **MuJoCo 3.10.0**; newer upstream source and
documentation do not automatically extend this package's support.

### Does installing MuJoCo on a Mac enable Metal physics?

The standard C engine's `mj_step` runs CPU physics. GPU simulation uses a
separate implementation; upstream documents MJX and MuJoCo Warp alongside the
C engine. Installing this experimental package does not replace `mj_step` or
redirect arbitrary MuJoCo applications to Metal. See the
[upstream overview](https://mujoco.readthedocs.io/en/stable/overview.html) and
[our explicit native entry points](mujoco_metal/smooth_metal.py).

Physics, rendering, and neural-network training are separate workloads. A GPU
viewer does not demonstrate GPU physics, and GPU PPO can accompany CPU physics.
Likewise, a Metal physics kernel does not make a viewer use Metal.

### Does the classic visualizer require OpenGL 3.3 or newer?

The documented requirement is **OpenGL 1.5 compatibility functionality**, with
`ARB_framebuffer_object` and `ARB_vertex_buffer_object`. The classic renderer
uses fixed-function OpenGL. Forcing a forward-compatible 3.3 **core** context
can remove functionality it needs; that is not a general macOS fix. See
[MuJoCo's OpenGL requirements](https://mujoco.readthedocs.io/en/stable/programming/visualization.html#using-opengl),
also present in the
[3.10.0 documentation source](https://github.com/google-deepmind/mujoco/blob/3.10.0/doc/programming/visualization.rst).

Apple's OpenGL deprecation does not itself make existing OpenGL applications
Metal-native or prevent them from running. Context/profile behavior also
depends on macOS and GLFW versions; consult
[GLFW's macOS notes](https://www.glfw.org/docs/latest/compat_guide.html#compat_osx).

### How should I investigate `OpenGL ARB_framebuffer_object required`?

Start with MuJoCo's own viewer or example for the installed version. In custom
windowing code, verify window/context creation succeeded and make that context
current on the calling thread **before** `mjr_makeContext`. Check the actual GL
version and extension support, and remove incompatible core-profile hints.
`mjrContext` manages MuJoCo rendering resources; it does not create the operating
system's OpenGL context. Offscreen rendering still needs a suitable context.
See [context setup](https://mujoco.readthedocs.io/en/stable/programming/visualization.html#context-and-gpu-resources).

For Python's `mujoco.viewer.launch_passive` on macOS, use
`mjpython your_viewer_script.py` from the intended environment to satisfy its
main-thread requirement. This applies to that viewer API, not every Python
simulation script. See the
[passive-viewer documentation](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer).

The author of [discussion #2970](https://github.com/google-deepmind/mujoco/discussions/2970)
ultimately identified a `glutin` problem. The thread's suggested 3.3-core-context
recipe should not override MuJoCo's renderer requirements.

### What about upstream Filament or other Metal renderers?

Renderer support must be checked separately for each release and build.
The surrounding upstream-derived source includes a Filament integration, and
[its documentation](../doc/programming/visualization.rst) notes that Filament
itself supports Metal. However, the
[backend selector in this source snapshot](../src/render/filament/core/filament_platform_factory.cc)
selects OpenGL or Vulkan; it does not expose a direct Metal selection.
Filament's capabilities alone therefore do not establish a working native
Metal MuJoCo viewer. This package supplies no renderer, and we have not
qualified that Filament integration on macOS.

### Can MJX use the Apple GPU through JAX-Metal?

MJX-JAX needs a backend that can compile and execute the operations used by the
chosen model and solver. Seeing an Apple GPU in JAX, or successfully loading a
model, does not establish that `mjx.step` works. Current
[MJX documentation](https://mujoco.readthedocs.io/en/stable/mjx.html) distinguishes
JAX and Warp implementations; the older 3.1.5 documentation is not a current
compatibility matrix. Apple's
[JAX-Metal page](https://developer.apple.com/metal/jax/) explains its compiler
path and version requirements.

There are concrete failure reports, with different scopes:

- [stac-mjx #126](https://github.com/talmolab/stac-mjx/issues/126) reports
  `mhlo.cholesky` legalization failure with `jax-metal 0.1.1` and JAX 0.5.0,
  and a separate compatibility failure with JAX 0.7.2. It describes
  `mhlo.triangular_solve` as a possible additional blocker, not a demonstrated
  failure in that report.
- [stretch_mujoco #40](https://github.com/hello-robot/stretch_mujoco/issues/40)
  reports `mhlo.reduce` failure and separate model/rendering limitations.

These are useful reproductions to investigate, not proof that every version,
model, or future backend fails. Record the exact macOS, JAX, jaxlib, plugin,
MuJoCo and model versions when reproducing them. We have not requalified those
JAX-Metal combinations here.

### Are Cholesky factorization and triangular solves impossible on Metal/MPS?

No. Apple exposes
[MPSMatrixDecompositionCholesky](https://developer.apple.com/documentation/metalperformanceshaders/mpsmatrixdecompositioncholesky)
and [MPSMatrixSolveTriangular](https://developer.apple.com/documentation/metalperformanceshaders/mpsmatrixsolvetriangular).
A missing compiler lowering in a JAX plugin does not mean the underlying
hardware or MPS library lacks the operation. Metal, MPS, MPSGraph, JAX-Metal,
PyTorch's MPS backend, and MLX have distinct APIs and operation coverage.
Using an available native operation still requires integration, supported data
types/layouts, numerical checks, and performance measurement.

### How do the community alternatives compare?

These projects are worth evaluating against a specific model and workload.
The descriptions below summarize their own documentation, not a compatibility
or speed certification from this project.

| Project | Documented approach | Qualification to keep in mind |
| --- | --- | --- |
| [RobotFlow-Labs/Mujoco-mlx](https://github.com/RobotFlow-Labs/Mujoco-mlx) | Ports MJX operations from JAX to Apple MLX; reports working stepping and simple physics comparisons. | Its README also mentions moving linear algebra to a CPU stream. MLX usage alone does not establish that every operation runs on GPU, or that all MuJoCo features match. |
| [genesisinteractive/MuJoCo-MLX-Cpp](https://github.com/genesisinteractive/MuJoCo-MLX-Cpp) | C++/MLX physics with a batched Metal pipeline and a dual CPU/GPU API. The supplied `arghyasur1991` URL redirects here. | Review its [conformance report](https://github.com/genesisinteractive/MuJoCo-MLX-Cpp/blob/main/CONFORMANCE.md), scalar versus batched paths, and model coverage. Its hardware/workload-specific benchmark numbers are not measurements of this package. |
| [Genesis World](https://github.com/Genesis-Embodied-AI/genesis-world) | A separate simulator whose [installation documentation](https://genesis-world.readthedocs.io/en/latest/user_guide/overview/installation.html) lists Apple Silicon simulation through `gs.metal`. | Current source describes the Quadrants compiler, forked from Taichi. Treat older Taichi-only descriptions as historical. Metal availability does not establish MuJoCo API, contact-dynamics, or trained-policy equivalence; rendering and optional components have their own requirements. |

### Does this package use JAX or MLX, and what is actually validated?

It uses custom Metal Shading Language kernels launched through
`torch.mps.compile_shader`, with MPS tensors for outputs. JAX and MLX are not
dependencies of this package. The
[launcher](mujoco_metal/smooth_metal.py), [dependency pins](pyproject.toml), and
[GPU tests](tests/test_gpu.py) document that path.

The current validation covers batched kinematics, dense mass matrices, and
inertial/gravity bias on small M1-family fixtures compared with MuJoCo. It does
not establish full stepping, contacts, constraint solving, integration,
rendering, or training support. Actuator models and nonzero tendon armature are
explicitly rejected by the smooth-dynamics stage. Validation of a separate
robot-specific backend does not extend this generalized package's coverage.

### Will it work on every M1–M5 Mac, and is it faster than CPU physics?

Neither is established by the current tests. Apple Silicon is the intended
hardware family, but chip generation, macOS, framework versions and memory
requirements still need qualification. Successful shader compilation and small
correctness tests establish no training-throughput advantage. This package has
no demonstrated CPU-to-Metal speedup.

Use `python -m mujoco_metal preflight --json --inventory` to record this
installation's module/shader paths, hashes, version and declared support.
Preflight is CPU-only; it is not a runtime GPU probe. After the isolated install
above, run `MUJOCO_METAL_RUN_GPU=1 python -m pytest -q tests/test_gpu.py` from
`metal/` for the scoped GPU checks. Benchmark separately on idle hardware with
matching physics, precision, environment counts and synchronization, including
state transfers and the full workload being claimed.
