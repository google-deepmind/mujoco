# Batched simulation: `mjBatch` and `mujoco.batch`

Status: experimental, implemented in this change as `mujoco/experimental/batch.h` (C, inside
libmujoco) and `mujoco.batch` (Python). The design follows Kevin Zakka's
[mjbatch](https://github.com/kevinzakka/mjbatch), and quotes its measurements where noted.

## Summary

A batch holds `nsim` simulations of one model and runs them on a thread pool of its own. Each
simulation is its `mjSTATE_INTEGRATION` row plus its warning counters; a call loads each row into an
`mjData`, runs, copies registered output fields out, and saves the row back. Results are
bit-identical to a loop over one `mjData` per simulation at any thread count. The C API is
`mjb_*`; the Python module exposes every field as a live `(nsim, ...)` NumPy array.

## Why

MuJoCo had two batched CPU paths, and neither fits closed-loop use. `mujoco.rollout` is a function
on trajectories: initial states and open-loop controls in, states and sensor values out. A policy in
the loop needs the simulations to persist between calls and needs derived quantities out; rollout
re-injects the full physics state every call, drops the solver warmstart, and returns only the state
and `sensordata`. Reinforcement learning on CPU, which mjlab and every vectorized-environment
library want, is closed-loop.

mjbatch fills that gap as a standalone package, but outside MuJoCo it pays for it: `mjModel` and
`mjData` layouts are its ABI, so it pins the exact MuJoCo version, needs a release per MuJoCo
release, cannot run against a development build, and duplicates machinery MuJoCo has. Inside
libmujoco those costs vanish, and every language binding can use the C API.

The accelerated backends (MJX, MuJoCo Warp) batch on the GPU. An exact, deterministic CPU batch that
runs wherever MuJoCo runs is the missing middle.

## A simulation is its state row

`mjSTATE_INTEGRATION` exists to define exact continuation: it is everything `mj_step` reads that it
did not compute in the same call. A batch stores one such row per simulation, in `mj_getState`
order, plus the `mjWarningStat` counters. Consequences:

- **Memory scales with threads.** By default there is one `mjData` per thread. Measured footprints
  (buffer plus touched arena; the arena reservation is virtual): cart-pole 6 KB, humanoid 81 KB,
  Unitree Go1 467 KB, Unitree G1 910 KB, against rows of 0.2, 1.8, 1.3 and 2.8 KB. mjbatch measured
  4096 Go1 simulations at 2.5 GB resident with one `mjData` per simulation and 53 MB with one per
  thread.
- **Rows are the only input storage.** `ctrl`, `qpos`, `mocap_pos`, applied forces and the other
  inputs are components of the row, so a write is a write to the row and becomes the simulation's
  state at its next call. Nothing is tracked or rounded, and the round trip through `mj_setState`
  and `mj_getState` is exact in `mjtNum`.
- **Reset discards earlier writes.** `reset` overwrites the row; write after it, then `forward`.
- **Exactness.** Results depend on neither the thread count nor the order of simulations. The tests
  run lockstep against one `mjData` per simulation with writes to every input kind, keyframe resets
  and subsets, at 1, 3 and 7 threads, and require equality, not tolerance.
- **Derived fields need the call that computes them.** Output fields (`xpos`, `sensordata`, ...)
  are copied out after each call. With one `mjData` per thread, a field the call does not compute
  (`qfrc_inverse` after a step, quantities computed only for the sensors that need them) holds
  whichever simulation last computed it. This is documented rather than hidden, and a persistent
  batch removes it.
- **Statelessness is a usage pattern.** The batch owns scratch `mjData` and thread models; the rows
  are data the caller can read, write or replace wholesale. Writing them every call is rollout;
  never writing them is a vectorized environment.

### Persistent batches and sleep

`persistent` keeps one `mjData` per simulation instead of per thread. The row protocol is unchanged:
the row holds exactly the bytes that simulation's `mjData` last had, so writing it back changes
nothing unless the caller wrote to it. That is what sleeping needs. Sleep bookkeeping (the asleep
trees, their countdowns, the values the wake check compares against) is a history of one `mjData`,
not part of the state, and waking is detected bytewise, so an `mjData` shared across simulations
would wake everything on every call. Sleep-enabled models therefore require a persistent batch and
are rejected otherwise, including when sleep is enabled for one simulation through an expanded
`opt.enableflags`. Moving the sleep bookkeeping into `mjtState` was considered and rejected. A
persistent batch also makes every output field that simulation's own value, and gives plugins one
instance per simulation; it costs `nsim` `mjData` of memory.

## Threading

The engine has its own pool, installed on one `mjData` by `mju_threadpool`, which parallelizes the
work within one step of one simulation. The batch parallelizes across simulations. These are
different use cases with separate pools, and they are not combined: a simulation is threaded from
the inside or from the outside, never both, so there is no nested parallelism or oversubscription to
manage. The batch creates its `mjData` itself and never installs an engine pool on them; rollout,
which takes the caller's `MjData`, rejects one that has a pool (the first commit of this change).

The pool runs `nthread` threads including the caller's, which takes a share of the work instead of
waiting. Simulation `i` starts in the slice of thread `i·T/n`, and a thread that finishes its slice
takes from the others. Idle workers spin for 50 µs before parking, so back-to-back calls skip the
wake-up. `nthread <= 0` means one thread per logical CPU: MuJoCo's small dense linear algebra stalls
on memory latency, and mjbatch measured a second thread per core adding 1.3 to 1.5× throughput on a
Threadripper 7960X for every model and batch size it tried. Calls on one batch are serialized.

## API

### C

```c
mjBatch* mjb_makeBatch(const mjModel* m, int nsim, int nthread, int persistent,
                       char* error, int error_sz);
void mjb_deleteBatch(mjBatch* b);

mjtNum* mjb_state(mjBatch* b);                 // (nsim, nstate) rows
mjWarningStat* mjb_warning(mjBatch* b);        // (nsim, mjNWARNING)
void* mjb_output(mjBatch* b, const char* name, int* size, int* elemsize);
void* mjb_expand(mjBatch* b, const char* name, int* size, int* elemsize);

int mjb_step(mjBatch* b, const int* ids, int nid, int nstep);
int mjb_forward(mjBatch* b, const int* ids, int nid);
int mjb_reset(mjBatch* b, const int* ids, int nid, int key);
int mjb_setConst(mjBatch* b, const int* ids, int nid);
int mjb_rollout(mjBatch* b, const int* ids, int nid, int nstep, const mjtNum* control,
                int control_spec, const mjBatchRecord* record, int nrecord);
int mjb_apply(mjBatch* b, const int* ids, int nid, mjfBatchFunc func, void* arg, int save);

const int* mjb_status(const mjBatch* b);       // (nsim), per simulation's last call
const char* mjb_error(const mjBatch* b, int sim);
```

Fields are named as in the X-macro tables (`"xpos"`, `"body_mass"`), and `mjOption` members as
`"opt.<name>"`. Calls take sorted unique `ids`, or `NULL` for all simulations, and return the number
of simulations in which MuJoCo raised an error.

- `mjb_rollout` is the trajectory call, with mujoco.rollout's semantics: before each substep it sets
  the `control_spec` components from `control`, `(n, nstep, ncontrol)` in call order, and after it
  copies each record, an `mjData` field or an `mjtState` spec, into `(n, nstep, size)`. A simulation
  whose warning counters rise stops stepping and repeats its last records, as rollout's divergence
  rule does. It replaces loops of single-step calls, where the per-call cost dominates short
  horizons on small models.
- `mjb_apply` runs a C function on each simulation's model and data, optionally saving the state.
  It refreshes no output field, since the function may compute any subset of them; results go
  through its argument. Queries such as Jacobians and rays are built on it.

### Python

```python
from mujoco import batch

b = batch.Batch(model, nsim=1024, nthread=0, persistent=False)
qpos, ctrl, xpos = b.bind('qpos'), b.bind('ctrl'), b.bind('xpos')
mass = b.expand('body_mass')            # (nsim, nbody), per simulation
gravity = b.expand('opt.gravity')       # (nsim, 3)
b.set_const()                           # derived constants follow, per simulation
for _ in range(1000):
  ctrl[:] = policy(qpos, xpos)
  b.step()                              # GIL released

out = b.rollout(50, control, record=(mujoco.mjtState.mjSTATE_FULLPHYSICS, 'sensordata'))
jacp, jacr = b.jac(body, points)
dist, geomid = b.ray(pnt, vec)
```

`bind` returns views of the rows for state fields and output buffers for the rest, in `MjData`'s
layout, and `b.joint('hinge').qpos`, `b.sensor('touch').data` and the other named views of
`MjData` slice them per object. `ids` may be integers or a boolean mask, and are validated before narrowing to int32. An
error in some simulations raises `mujoco.batch.SimulationError` listing all of them; they keep their
pre-call state, the others run to completion, and `b.status` and `b.error(i)` hold the outcome.
Names follow MuJoCo (`nsim`, `nthread`, `ids`) rather than mjbatch.

## Model fields and derived constants

`expand` gives a field per-simulation storage, seeded from the model and copied into the thread's
model before each simulation's call. `set_const` runs `mj_setConst` per simulation and compares
every non-expanded field with the template: whatever it changed is expanded and every simulation is
recomputed, until nothing changes. So derived constants are complete without a maintained list, and
the scalars it writes (`ngravcomp`, the gravcomp, surface-velocity and adhesion flags, `stat`) are
kept per simulation too. A simulation that raises keeps its constants, and the others complete.

Thread models are shallow: a copy of the struct and of every non-asset array, with meshes,
heightfields, textures, skins and BVH data shared with the batch's model, which `mj_setConst` and
the pipeline only read. A Menagerie G1's model buffer is 76 MB, all but 81 KB of it assets, so full
copies at 48 threads would cost 3.8 GB.

## Errors

Batch threads install a thread-local log handler that traps `mju_error` by unwinding to a `setjmp`
around each simulation's call (`_setjmp` on macOS, where `setjmp` saves the signal mask with a
system call). Other messages go to the thread's previous handler. The failing simulation's row is
not saved, so it keeps its state, and its `mjData` is reset for reuse. Arguments are validated
before a call takes its lock, so a handler that does not return cannot leave the batch locked.

## Performance

The batch adds no measurable cost over mjbatch, which was tuned for throughput first. Both built
with LTO, on an Apple M5 (4 performance and 6 efficiency cores), best of three interleaved runs:

| | threads | `mujoco.batch` | mjbatch 0.1.4 |
|---|---|---|---|
| Humanoid, 256 simulations, mjlab's access pattern (env-steps/s) | 1 / 4 / 8 | 9.3k / 28.8k / 44.6k | 9.3k / 29.3k / 45.0k |
| 64 two-joint simulations, one step per call (µs per call) | 1 / 4 / 8 | 63 / 23 / 17 | 67 / 22 / 18 |

In the second benchmark `mj_step` itself is 52 of the 63 µs, so the batch's per-simulation cost
(state load and save, error trap, output copies) is under 0.2 µs, and the Python layer adds nothing
measurable over the bound call. On a 24-core Threadripper 7960X, mjbatch measured 256 Unitree G1
simulations at 18k substeps/s on one thread, 257k on 24 and 383k on 48: the second thread per core
is the reason for the default of one thread per logical CPU.

## Decisions

- `mjb_` is a new module prefix, as `mjs_` was for specs.
- Calls return a failure count; per-simulation status and messages replace a first-failure index.
- Experimental header, with no stability promise until a release of feedback.
- One object; rollout's stateless form is a usage pattern of it.
- `mjtNum` only. float32 views are a framework concern; single-precision builds are float32
  throughout.
- `mujoco.rollout` is unchanged.
- In Python, physics callbacks must be C functions (ctypes). A Python callback would need `MjModel`
  and `MjData` wrappers for the batch's own models and data, and the GIL on every batch thread.
- A function run by `mjb_apply` may call other batches that do not call back into its own. A
  direct call on its own batch raises in that simulation rather than deadlocking.

## Not in this change

- Reimplementing `mujoco.rollout` on `mjb_rollout`, with `rollout_test.py` as its acceptance test.
- Template lists: structurally different models of one size signature in one batch, which mjlab's
  mesh variants need.
- Tagging the fields `mj_setConst` writes in `mjxmacro.h`, which would replace the
  change-detection loop.
- Recording at a reduced rate for long horizons.

## Alternatives

- **Upgrade rollout in place.** Giving rollout persistent rows, arbitrary outputs and
  per-simulation model fields is this design with rollout's signature; the stateful object is the
  primitive because the stateless call is expressible on it and not the other way round.
- **Keep it external.** That keeps the version pin, the release lockstep and the duplicate
  machinery, and keeps it from other languages.
- **Batch inside the Python bindings only.** The batching code has nothing Python-specific in it; in
  libmujoco every binding gets it.
- **Share the engine's thread pool.** The two kinds of threading serve different use cases and do
  not combine (above).
