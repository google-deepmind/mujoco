# Contact-free Metal stepping: achievements and measured CPU comparison

The experimental package now advances a supported model entirely through native
Metal physics: forward kinematics, mass/bias, acceleration solve and semi-implicit
Euler integration. Persistent device state, quaternion updates, transactional
reset/checkpoint ownership, and per-world failure handling complete this bounded
stepping path. It is an implementation of existing rigid-body dynamics methods,
not a new physics or learning algorithm. **Contacts, general actuation and full
MuJoCo compatibility remain unsupported.**

## Complete measured CPU8 / Metal comparison

Apple **M1 Max, 32 GB unified memory**, 10 CPU cores (8 performance and 2 efficiency).
MuJoCo 3.10.0, Torch 2.9.1, NumPy 2.5.3. Four-DOF tutorial pendulum with contacts
disabled and a 1 ms timestep. CPU8 means eight worker threads in MuJoCo's compiled
`rollout` API, not a Python loop over worlds. Each row has an actual CPU measurement;
none of the CPU results below are extrapolated.

Median wall time for **200 batched physics steps**, excluding setup, compilation,
reset, warmup, final readback, and numerical checks:

| Environments | CPU8 ms | Metal ms | CPU8 / Metal | Metal million world-steps/s |
|---:|---:|---:|---:|---:|
| 1 | 0.25 | 41.66 | 0.01x | 0.0048 |
| 64 | 2.43 | 42.93 | 0.06x | 0.30 |
| 512 | 17.75 | 42.59 | 0.42x | 2.40 |
| 2,048 | 72.18 | 47.32 | 1.53x | 8.66 |
| 4,096 | 138.85 | 68.94 | 2.01x | 11.88 |
| 8,192 | 281.66 | 116.22 | 2.42x | 14.10 |
| 16,384 | 590.14 | 222.69 | 2.65x | 14.71 |
| 32,768 | 1,210.65 | 416.68 | 2.91x | 15.73 |
| 65,536 | 2,487.94 | 798.04 | 3.12x | 16.42 |
| 131,072 | 4,793.74 | 1,532.53 | 3.13x | 17.11 |
| 262,144 | 9,477.18 | 2,983.79 | 3.18x | 17.57 |
| 524,288 | 18,905.02 | 5,906.96 | 3.20x | 17.75 |

A ratio above 1 favors Metal. At **524,288 worlds, Metal is 3.20x faster than
this eight-thread CPU rollout API**: 5.907 s versus 18.905 s. CPU trials ranged
18.820–18.931 s; Metal trials ranged 5.905–5.909 s. This is a workload-specific
API-throughput result with different precision/output costs, not an isolated
hardware acceleration factor or a robot-training speedup.

Metal reaches **17.75 million world-steps/s**. Doubling from 262,144 to 524,288
adds only about 1% throughput; 65,536 already delivers about 92.5% of that rate.
The largest tested batch uses 1.58 GiB of live MPS tensors and 2.06 GiB of MPS
driver allocation. No solver failures, nonfinite states, or OOM occurred.
This is a tested capacity lower bound and throughput plateau, **not a measured
maximum memory capacity**. CPU wins at small batch sizes.

## Method and limitations

- Five trials at 1–2,048 worlds, three trials at 4,096–524,288. GPU warmup:
  50 steps. CPU warmup: one complete rollout, which touches preallocated outputs.
  Metal timing synchronizes MPS before and after each block. CPU calls block
  until workers finish. Training was stopped; no competing test or training job
  was launched during measurement.
- Large batches ran in separate processes, releasing allocations between sizes.
  CPU and Metal were measured separately; missing large CPU points were completed
  in a follow-up session using the same model, step count and starting state.
  Fixed ordering, one Mac, short trials: thermal state and scheduling are not
  controlled as in a sustained hardware benchmark. Trial ranges are in the raw data.
- **Output/precision mismatch:** CPU rollout writes every float64 trajectory state;
  Metal retains float32 state on device and does one untimed final readback.
  The largest CPU trajectory buffer is 7.03 GiB. Output recording, validation and
  scheduling costs of each API are included. A matched final-state-only CPU
  implementation and matched precision would be useful additional baselines.
- All worlds use the same model and initial state. Every world's final position
  and velocity was checked against CPU; maximum errors in the larger-batch sweep
  were 1.80e-6 and 1.18e-5 respectively, with zero native failure statuses.
  This replicated-state scaling test does not substitute for diverse-world tests.
- No rendering, policy inference, observations, contacts, PPO, or checkpoint I/O
  is timed. Do not use the batch sizes or speed ratios as robot-training guidance.
  CPU memory figures and Metal memory figures are not interchangeable.
- Source revision: `36451fba7c33fd252c90de3b29742940d9027642`; later changes
  document the experiment and add its portable runner without changing physics.
  [Raw trial data and model hash](m1-max-pendulum-20260926.json) are included.

## Correctness qualification accompanying the milestone

The source suite passed **90 tests with GPU execution enabled** and MPS CPU
fallback disabled. Independent parent review additionally compared 12 trajectories
(three states each for slide, hinge/armature, asymmetric free body, and mixed
free/ball/hinge/slide models) for 1,000 steps at 1 ms. Maximum absolute position
and velocity errors were approximately 1.25e-5 and 7.31e-5 across those fixtures;
combined tolerances were qpos `atol=1e-4, rtol=1e-3` and qvel
`atol=1e-3, rtol=1e-3`. All four fixtures resumed bit-exactly after checkpoint
restoration. Independent solve checks covered 18 dimension/conditioning/scale
cases; maximum normalized residual was 5.25e-8.

The installed package passed 51 CPU-only tests with Torch absent (39 skipped),
and its 200-step native pendulum smoke test passed. Source CPU testing with Torch
installed passed 65 tests (25 GPU tests skipped). Native offscreen rendering
passed. Native interactive playback is pending an active macOS display; the
previous hybrid viewer was checked separately. The existing GIF depicts that
hybrid path, not this native benchmark.

These checks are fixture-specific, not general physics correctness certification.
Full upstream core tests, single-precision core builds, other Apple hardware/OS
combinations, and longer or contact-rich trajectories remain unqualified.

## Reproduce

From the repository root, after the isolated installation in [the package
README](../README.md), run each backend as a separate process on idle hardware:

```sh
python metal/benchmarks/pendulum_scaling.py \
  --backend cpu8 --batch 524288 --output /tmp/pendulum-cpu8.json
PYTORCH_ENABLE_MPS_FALLBACK=0 python metal/benchmarks/pendulum_scaling.py \
  --backend metal --batch 524288 --output /tmp/pendulum-metal.json
```

Defaults are 200 steps and three trials. Use `--trials 5` for the small-batch
protocol, and start with `--batch 2048` on a machine with less free memory. The
CPU runner refuses trajectory buffers exceeding 8 GiB; this guard does not
reserve memory or guarantee sufficient system headroom. The CPU path does not
import Torch. Results include raw wall times, correctness errors and model hash.
The runner validates results after timing and raises on failure.

For a complete sweep, repeat those two commands at 1, 64, 512, 2,048, 4,096,
8,192, 16,384, 32,768, 65,536, 131,072, 262,144 and 524,288 worlds. Reproduction
results will vary with hardware, framework/compiler caches and system load.

## Next useful measurements

A more complex supported model, distinct initial states across worlds, a matched
output CPU baseline, and sustained trials are more informative than further
increasing this tiny pendulum's batch size. Earlier isolated stage timing points
to smooth dynamics and dispatch overhead as useful profiling targets; those
separate stage timings are not additive shares of end-to-end wall time.
