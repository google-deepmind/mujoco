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
`opt.enableflags`. Moving the sleep bookkeeping into `mjtState` was considered and rejected. IPC
contact (`flexvert_lambda`, `flexvert_conage`) is cross-step bookkeeping too, under the same rule. A
persistent batch also makes every output field that simulation's own value, and gives plugins one
instance per simulation; it costs `nsim` `mjData` of memory. Plugins are not rejected otherwise:
the batch cannot tell which keep state outside `plugin_state`, and those that do share it among the
simulations a thread runs, which is documented beside the sleep rule.

### Observations of the new state

After `mj_step`, derived fields describe the state before the last substep. Reinforcement learning
wants observations of the new state, and mjbatch's answer, `mj_forward` after the steps, has two
costs: the acceleration stage it computes (actuator forces, `qacc`, constraint forces, touch and
accelerometer readings) uses controls that the next step will replace, so those values describe no
step that happens; and it is nearly a second step, 14.6 µs against 15.5 µs for the humanoid.

`mjb_step` takes an `mjtStage` instead. `mjSTAGE_VEL` runs the position and velocity stages at the
new state, which is `mj_step1`: positions, velocities, contacts and the sensors of those stages
describe the new state, while acceleration-dependent values remain those of the last substep, under
the controls it applied. That costs 7.0 µs on the humanoid in a shared batch, amortized over the
substeps of a call. `mjSTAGE_ACC` is still available, documented as what it is. `mjb_reset` and
`mjb_forward` take a stage too: a reset followed by randomization and a forward, the usual
reinforcement-learning pattern, skips the reset's own forward pass with `mjSTAGE_NONE`.

Since `mj_step1` followed by `mj_step2` is `mj_step`, a persistent batch continues its next step
from those stages with `mj_step2`, and fresh observations cost nothing. The batch decides this per
simulation and call; nothing in the API changes and the results are bit-identical. It continues only
if the stages are current and nothing they depend on changed except `ctrl`, `qfrc_applied` and
`xfrc_applied`, as MuJoCo's own step1/step2 contract requires:

- The row is compared with the `mjData`, outside those inputs, so writes such as randomization
  after a reset are seen.
- Expanded model fields are compared with the values the stages were computed with. Nothing else in
  the model can change: the batch's model is private.
- `set_const`, `mjb_apply`, rollout and errors leave the stages stale; `forward`, `reset` and steps
  with `mjSTAGE_VEL` or `mjSTAGE_ACC` leave them current.
- With a tree asleep, only `ctrl` may differ: applied forces wake trees in `mj_wake`, which runs in
  the position stage.
- Physics callbacks that run in those stages, and `mjcb_control`, which `mj_step` lets overwrite the
  caller's controls after they are written, turn continuation off, as does RK4, since `mj_step2`
  integrates with Euler.

dm_control used the same step2-then-step1 pattern for years, and its users read garbage contact
forces (google-deepmind/mujoco#13): contacts and `efc_address` come from the position stage and
`efc_force` from the acceleration stage, so between step1 and step2 they do not belong together.
The batch is safe from this because it exposes no contact or constraint arrays, only fixed-size
fields, and every sensor that reads constraint forces is an acceleration-stage sensor, evaluated in
step2 together with its contacts. The one way to reach the raw arrays is a function run by
`mjb_apply` in a persistent batch, which is documented: such a function must run the acceleration
stage before reading contact forces. Exposing contacts later would need the same care.

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

A call on fewer simulations than threads, such as a reset of a few environments, uses only as many
threads; the others neither run it nor are waited for, and a call on one simulation runs inline. A
pool that waits for every worker pays for whichever is descheduled: with one of the M5's ten cores
held by another process, a step of one of 1024 humanoids at 10 threads took 68 µs, and takes 19 µs
with this rule; on an idle machine both take about 16 µs. This is the property that
`mujoco.rollout`'s job queue has; its chunking rule (about ten chunks per thread) is not needed
here, since a thread claims items with one atomic increment on a cache line of its own. The `mjData`
are made on the pool, each by the thread that will usually run it, so that their pages are near it
on multi-socket machines.

## API

### C

```c
mjBatch* mjb_makeBatch(const mjModel* m, int nsim, int nthread, int persistent,
                       char* error, int error_sz);
void mjb_deleteBatch(mjBatch* b);

mjtNum* mjb_state(mjBatch* b);                 // (nsim, nstate) rows
mjtNum* mjb_stateComponent(mjBatch* b, int component, int* size);  // a component in row 0
mjWarningStat* mjb_warning(mjBatch* b);        // (nsim, mjNWARNING)
void* mjb_output(mjBatch* b, const char* name, int* size, int* elemsize);
void* mjb_expand(mjBatch* b, const char* name, int* size, int* elemsize);

int mjb_step(mjBatch* b, const int* ids, int nid, int nstep, int stage);
int mjb_forward(mjBatch* b, const int* ids, int nid, int stage);
int mjb_reset(mjBatch* b, const int* ids, int nid, const int* key, int stage);
int mjb_setConst(mjBatch* b, const int* ids, int nid);
int mjb_rollout(mjBatch* b, const int* ids, int nid, int nstep, const mjtNum* control,
                int control_spec, const mjBatchRecord* record, int nrecord);
int mjb_apply(mjBatch* b, const int* ids, int nid, mjfBatchFunc func, void* arg, int save);

void mjb_getModel(mjBatch* b, int sim, mjModel* m);   // one simulation, to render or
void mjb_getData(mjBatch* b, int sim, mjData* d);     // pass to any function

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
b.body('pole').mass = masses           # named, as on MjModel: expands body_mass
b.set_const()                           # derived constants follow, per simulation
for _ in range(1000):
  ctrl[:] = policy(qpos, xpos)
  b.step()                              # GIL released

out = b.rollout(50, control, record=(mujoco.mjtState.mjSTATE_FULLPHYSICS, 'sensordata'))
jacp, jacr = b.jac(body, points)       # a body, or one per simulation
dist, geomid, normal = b.ray(pnt, vec, normal=True)
b.reset(ids, keyframe=keys)             # a keyframe per simulation
model, data = b.get_model(i), b.get_data(i)   # one simulation, to render or pass anywhere
```

`bind` returns views of the rows for state fields and output buffers for the rest, in `MjData`'s
layout, and `b.joint('hinge').qpos`, `b.sensor('touch').data` and the other named views of `MjData`
slice them per object. Fields are also attributes, as on `MjData` (`b.xpos`, `b.ctrl = u`), and
`DATA_FIELDS` and `MODEL_FIELDS` name what `bind` and `expand` accept, as the batch's own tables
decide. `ids` may be integers or a boolean mask, and are validated before narrowing to int32. An
error in some simulations raises `mujoco.batch.SimulationError` listing all of them; they keep their
pre-call state, the others run to completion, and `b.status` and `b.error(i)` hold the outcome.
Names follow MuJoCo (`nsim`, `nthread`, `ids`) rather than mjbatch.

## Model fields and derived constants

`expand` gives a field per-simulation storage, seeded from the model and copied into the thread's
model before each simulation's call. `set_const` runs `mj_setConst` per simulation and compares
every non-expanded field with the template: whatever it changed is expanded and the call's
simulations are recomputed, until nothing changes. The others need not be: a newly expanded field
starts from the template's value, which is theirs unless their own `set_const` changed it, in which
case it would already be expanded. So derived constants are complete without a maintained list, and
the scalars it writes (`ngravcomp`, the gravcomp, surface-velocity and adhesion flags, `stat`) are
kept per simulation too. A simulation that raises keeps its constants, and the others complete.

Thread models are shallow: a copy of the struct and of every non-asset array, with meshes,
heightfields, textures, skins and BVH data shared with the batch's model, which the pipeline only
reads. A Menagerie G1's model buffer is 76 MB, all but 81 KB of it assets, so full copies at 48
threads would cost 3.8 GB. `mj_setConst` writes one part of the BVH data, the bodies' bounding
volumes at the start of `bvh_aabb`, which is all that `expand` and `set_const` keep per simulation.
During `set_const` a thread model writes them into its own copy; it copies the whole array, whose
mesh hierarchies can be half of a model (38 MB of mjlab's G1), only once they are per-simulation,
as when geometry is randomized.

## Errors

Batch threads install a thread-local log handler that traps `mju_error` by unwinding to a `setjmp`
around each simulation's call (`_setjmp` on macOS, where `setjmp` saves the signal mask with a
system call). Other messages go to the thread's previous handler. The failing simulation's row is
not saved, so it keeps its state, and its `mjData` is reset for reuse. Arguments are validated
before a call takes its lock, so a handler that does not return cannot leave the batch locked.

## Performance

Both built with LTO, on an idle Apple M5 (4 performance and 6 efficiency cores), best of three
interleaved runs:

| | threads | `mujoco.batch` | mjbatch 0.1.4 |
|---|---|---|---|
| Humanoid, 256 simulations, mjlab's access pattern (env-steps/s) | 1 / 4 / 8 / 10 | 9.9k / 32.3k / 51.1k / 55.0k | 10.1k / 31.8k / 52.3k / 59.7k |
| 64 two-joint simulations, one step per call (µs per call) | 1 / 4 / 8 / 10 | 62 / 18 / 16 / 14 | 63 / 19 / 16 / 15 |
| Step of 1 of 1024 humanoids (µs per call) | 8 / 10 | 16 / 16 | 15 / 17 |
| Step of 8 of 1024 humanoids (µs per call) | 8 / 10 | 38 / 39 | 34 / 35 |

Throughput is at parity up to 8 threads; at 10, runs spread by 15%, and the best of ours is 8% below
the best of mjbatch's. A step of 8 of 1024 simulations is 4 µs slower than mjbatch's, which we have
not traced. In the second benchmark `mj_step` itself is 52 of the 62 µs, so the batch's
per-simulation cost (state load and save, error trap, output copies) is under 0.2 µs, and the Python
layer adds nothing measurable over the bound call. On a 24-core Threadripper 7960X, mjbatch measured
256 Unitree G1 simulations at 18k substeps/s on one thread, 257k on 24 and 383k on 48: the second
thread per core is the reason for the default of one thread per logical CPU.

### Single precision

In a build with `mjtNum` as `float`, the C++ and Python tests pass unchanged; the Python layer takes
its element type from the rows. A C++ benchmark of both builds, without LTO and so not comparable
with the table above:

| | threads | float64 | float32 |
|---|---|---|---|
| Humanoid, 256 simulations, 4 substeps per call (env-steps/s) | 1 / 4 / 8 / 10 | 15.4k / 57.7k / 84.9k / 99.8k | 16.8k / 60.1k / 90.6k / 102.7k |
| 64 two-joint simulations, one step per call (µs per call) | 1 / 4 / 8 / 10 | 40.2 / 12.0 / 10.3 / 10.8 | 30.6 / 9.7 / 8.8 / 10.0 |

float32 is 3 to 9% faster on the humanoid and 7 to 24% on the small model. The humanoid runs
without warnings in both, with the same contacts and 3% more solver iterations in float32, so the
gain is in the arithmetic, and it is small.

## Decisions

- `mjb_` is a new module prefix, as `mjs_` was for specs.
- Calls return a failure count; per-simulation status and messages replace a first-failure index.
- Experimental header, with no stability promise until a release of feedback.
- One object; rollout's stateless form is a usage pattern of it.
- `mjtNum` only. float32 views are a framework concern; single-precision builds are float32
  throughout, and tested.
- `mujoco.rollout` is unchanged.
- A function run by `mjb_apply` may call other batches that do not call back into its own. A
  direct call on its own batch raises in that simulation rather than deadlocking.

## Requested and declined

Requests from review that this change does not take, and why:

- **`mj_forward` after the steps** (mjbatch's `forward=True`): its acceleration-stage values use
  controls the next step replaces, at nearly the cost of another step. `mjtStage` takes its place
  (see Observations of the new state).
- **Rejecting models with stateful plugins in shared batches**: the batch cannot tell which plugins
  keep state outside `plugin_state`, and rejecting every plugin would refuse the stateless ones. The
  rule is documented beside the sleep rule, and a persistent batch gives each simulation its own
  instances.
- **Python physics callbacks**: a Python callback would need `MjModel` and `MjData` wrappers for the
  batch's own models and data, and the GIL on every batch thread. Calls raise `ValueError` instead;
  C callbacks installed through ctypes work.
- **The simulation's index in global callbacks**: the `mjcb_*` signatures belong to MuJoCo, not the
  batch. Per-simulation work belongs in `mjb_apply`, whose function receives it.
- **`eq_active` bound as bytes, as in `MjData`**: the rows hold `mjtNum`, and a byte view cannot
  alias them, so writes through it would be lost. It binds as `mjtNum`.
- **Filling a newly bound field from the current state in a shared batch**: there is no
  per-simulation `mjData` to copy from. A persistent batch fills it; a shared one, at the next call.
- **`mujoco.rollout`'s chunking rule**: claiming items is already cheap, and what made rollout's
  small calls robust, not waiting for idle threads, is in the pool (see Threading).

## Not in this change

- Reimplementing `mujoco.rollout` on `mjb_rollout`, with `rollout_test.py` as its acceptance test.
- Template lists: structurally different models of one size signature in one batch, which mjlab's
  mesh variants need.
- Tagging the fields `mj_setConst` writes in `mjxmacro.h`, which would replace the
  change-detection loop.
- Recording at a reduced rate for long horizons.
- A Python entry to `mjb_apply` for C functions through ctypes. `jac` and `ray` cover mjbatch's
  queries; others can follow their uses.

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
