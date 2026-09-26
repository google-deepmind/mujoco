# RFC: Native Metal physics on Apple Silicon

**Draft for scope discussion, not a supported backend or a merge-ready design.**
This proposal requests maintainer guidance before porting an existing downstream
prototype into MuJoCo. It makes no change to the engine, public API, dependencies,
or supported platforms. The desired capability is batched physics on the Apple
GPU with device-resident state that a learner can consume without a host round
trip on every simulation step.

## Existing implementation available for review

The public [Microduck RL Mac project](https://github.com/keeeeenw/microduck_rl_mac)
contains an experimental Metal physics implementation. The reference snapshot is
[60f0b26](https://github.com/keeeeenw/microduck_rl_mac/tree/60f0b26a55e3236d2112126386b25ac922228083).
Its [runtime and shaders](https://github.com/keeeeenw/microduck_rl_mac/tree/60f0b26a55e3236d2112126386b25ac922228083/src/mjlab_microduck/native_gpu/metal),
[qualification tests](https://github.com/keeeeenw/microduck_rl_mac/tree/60f0b26a55e3236d2112126386b25ac922228083/tests/metal),
and [evidence and limitations](https://github.com/keeeeenw/microduck_rl_mac/blob/60f0b26a55e3236d2112126386b25ac922228083/docs/experimental-metal.md)
are available independently of this proposal.

The prototype uses custom Metal kernels launched through PyTorch MPS. It keeps
rigid-body dynamics, ground contacts, constraint solving and policy/optimizer
state on the GPU. CPU MuJoCo still supplies self-contact narrowphase and
reset-time constant recomputation, with explicit staging. Unified memory does
not remove synchronization, layout conversion, or these transfers.

The tested scope is a particular flat-ground biped task: 14 actuated joints,
21 generalized positions, 20 velocities and 17 bodies. Qualification used
MuJoCo 3.10.0 and Torch 2.9.1 on an M1 Max with 32 GB unified memory and a 24-core
GPU. This is evidence that the approach can run a downstream learning workload;
it does not establish compatibility with arbitrary models or current MuJoCo.
The linked report distinguishes qualification of the original implementation
from GPU validation still pending after packaging.

## What could be contributed

Potentially reusable work includes batched kinematics and articulated dynamics,
Metal constraint and integration kernels, persistent device-buffer management,
and stage-by-stage comparison against CPU MuJoCo. These need to be separated
from fixed robot dimensions, CAD assets, task rewards, actuator models, and
training-framework integration before proposing an upstream implementation.

The useful boundary would be an explicitly selected, optional physics backend
that consumes supported compiled models, owns batched device state, and exposes
advance/reset/readback operations. The backend must enumerate its supported
features and reject unsupported configurations before allocation or simulation.
Its existence must not require Torch or Metal for ordinary CPU builds/imports.
This describes a capability boundary, not a proposed public API signature.

Questions for maintainers:

- Is a community-maintained native Metal compute implementation in scope, and
  should it live in this repository, an accelerator repository, or an external
  project? See the earlier [Metal discussion #95](https://github.com/google-deepmind/mujoco/issues/95).
- Which existing backend abstraction should a contribution target? The current
  Torch launcher is a prototype integration choice, not a request to add Torch
  to MuJoCo's core dependencies.
- What minimal model/feature subset and Apple hardware CI would be acceptable
  for the first implementation PR?
- Is float32-only device arithmetic acceptable with comparisons against both
  single- and double-precision CPU builds? The prototype does not implement a
  double-precision Metal physics path.

## Acceptance boundary for an implementation

An implementation would need model-independent tests for supported topology,
kinematics, inertia, forces, contacts, constraints and integration; reset and
model-parameter updates; and repeated stepping/continuation. Unsupported joints,
geometries, solver/integrator choices and capacity overflow need explicit error
behavior. Device-buffer lifetime, aliasing and synchronization must be specified
and tested rather than inferred from shared physical memory.

Tests should compare both intermediate quantities and trajectories against a
pinned CPU reference, with precision-aware tolerances and documented limitations.
Existing CPU behavior must pass its single- and double-precision checks. A Metal
implementation requires actual GPU execution and CI; successful Python imports
or CPU-only helper tests are insufficient.

Performance claims require completed, synchronized wall times at matched model,
solver, timestep, precision and batch settings, with warmup and repetitions.
Report physics throughput separately from end-to-end learner throughput,
including CPU work, transfers, resets and memory. Historical downstream training
rates are not a controlled MuJoCo backend benchmark and are not offered as one.

This RFC deliberately carries no engine implementation or claim of official
Metal support. It seeks agreement on contribution scope before generalizing and
submitting the downstream code in reviewable increments.
