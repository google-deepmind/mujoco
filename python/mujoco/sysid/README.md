# Practical System Identification

A toolbox for system identification built on top of MuJoCo.

## API Overview

The library solves a **box-constrained nonlinear least-squares** problem. Given
a parameter vector `θ`, simulated sensor readings `ȳ(θ)`, and recorded sensor
data `y`, the objective is:

```
min   ½ ‖W (ȳ(θ) − y)‖²
 θ

subject to   θ_min ≤ θ ≤ θ_max
```

where `W` is a diagonal weighting matrix and the box constraints enforce
physical plausibility (e.g., positive masses).

The optimizer uses the **Gauss-Newton** method. The residual Jacobian
`J = ∂r/∂θ` is computed by **finite differences**: each column of `J` requires
one perturbed simulation rollout, and these evaluations are independent across
parameters and parallelize naturally across threads. The Gauss-Newton
approximate Hessian is `H ≈ JᵀJ` and the gradient is `g = Jᵀr`, yielding the
update `Δθ = −H⁻¹g`. Box constraints are handled by projected steps.

The pipeline has five stages:

```
Define Parameters ──> Package Data ──> Build Residual ──> Optimize ──> Save / Report
   ParameterDict       ModelSequences   build_residual_fn   optimize    save_results
```

---

### What Can You Identify?

**Anything settable on `MjSpec` can be identified** via modifier callbacks. The
convenience functions handle common cases with correct bounds; for everything
else, write a `modifier` lambda that sets the quantity on the spec.

**Physics parameters** — these change the model before simulation:

| Target | Approach |
|---|---|
| Body mass | `body_inertia_param(..., InertiaType.Mass)` |
| Body mass + center of mass | `body_inertia_param(..., InertiaType.MassIpos)` |
| Full body inertia (10-D) | `body_inertia_param(..., InertiaType.Pseudo)` |
| Actuator P/D gains | `Parameter(..., modifier=lambda s, p: apply_pgain(s, "act1", p.value[0]))` |
| Contact friction / solref | `Parameter(..., modifier=lambda s, p: s.pair("cp").friction.__setitem__(0, p.value[0]))` |
| Joint damping / stiffness | `Parameter(..., modifier=lambda s, p: setattr(s.joint("j1"), "damping", p.value[0]))` |

**Measurement parameters** — real sensors aren't perfect. They may lag behind
the simulation clock, have an unknown scale factor, or sit at a nonzero offset.
These can't be set on `MjSpec` because they aren't physics — they're artifacts
of the measurement system. `SignalTransform` (Section 4) adjusts the simulated
or recorded signals *after* rollout to account for these:

| Target | Approach |
|---|---|
| Sensor delay | `transform.delay("*_pos", params["delay"])` |
| Sensor gain/scale | `transform.gain("*_torque", params["scale"])` |
| Sensor bias/offset | `transform.bias("*_vel", params["bias"])` |

#### Common recipes

**Identify link masses of a robot arm:**

```python
from mujoco.sysid import body_inertia_param, InertiaType, ParameterDict

params = ParameterDict()
for link in ["link1", "link2", "link3"]:
    params.add(body_inertia_param(spec, model, link, inertia_type=InertiaType.Mass))
```

**Identify contact friction:**

```python
from mujoco.sysid import Parameter

params.add(Parameter(
    "floor_friction",
    nominal=1.0, min_value=0.1, max_value=3.0,
    modifier=lambda s, p: s.pair("foot_floor").friction.__setitem__(0, p.value[0]),
))
```

---

### 1. Define Parameters

A **`Parameter`** is a named value (scalar or array) with bounds and an optional
**modifier callback** that knows how to apply itself to a MuJoCo spec. A
**`ParameterDict`** collects parameters into the single vector that the
optimizer sees — it handles flattening them into one array, writing optimizer
updates back, and enforcing bounds:

```python
from mujoco.sysid import Parameter, ParameterDict

params = ParameterDict()

params.add(Parameter(
  "box_mass",
  nominal=5.0,        # starting value
  min_value=1.0,
  max_value=10.0,
  modifier=lambda spec, p: setattr(spec.body("box"), "mass", p.value[0]),
))

params.add(Parameter(
  "friction",
  nominal=[1.6, 0.005],
  min_value=[0.0, 0.0],
  max_value=[3.0, 0.01],
  frozen=True,           # excluded from optimization
  modifier=lambda spec, p: spec.pair("contact").friction.__setitem__(slice(0, 2), p.value),
))
```

**`frozen`**: A frozen parameter is completely invisible to the optimizer — it
is excluded from `as_vector()`, `update_from_vector()`, `get_bounds()`, and
`randomize()`. Its modifier callback is also **not called** during
`apply_param_modifiers`, so the model uses whatever value is already in the XML
spec for that quantity. The intended workflow: define all parameters you might
ever want to identify up front, then toggle `frozen` on and off as you
iteratively narrow which parameters matter.

Key `ParameterDict` methods:

| Method | Description |
|---|---|
| `as_vector()` | Flatten all non-frozen parameters into a 1-D array |
| `update_from_vector(x)` | Write a flat array back into the parameters |
| `get_bounds()` | Returns `(lower, upper)` bound arrays |
| `randomize(rng)` | Sample each non-frozen parameter uniformly within bounds |
| `reset()` | Restore every parameter to its nominal value |
| `copy()` | Deep copy (preserves modifier lambdas) |
| `save_to_disk(path)` / `load_from_disk(path)` | YAML serialization (schema + values) |

#### Body inertia parameterization

Rigid-body inertia is tricky to identify: mass, center-of-mass, and the rotational
inertia tensor are coupled, and naively optimizing the 6 independent entries of
the inertia tensor can produce physically impossible results (e.g. negative
eigenvalues). The library implements three parameterizations of increasing
fidelity:

| `InertiaType` | Params | What it identifies |
|---|---|---|
| `Mass` | 1 | Mass only. Optionally scales the existing rotational inertia proportionally (`scale_rot_inertia=True`). |
| `MassIpos` | 4 | Mass + center-of-mass position (3-D). Optionally scales rotational inertia. |
| `Pseudo` | 10 | Full inertia via the pseudo-inertia Cholesky factor from [Rucker & Wensing 2022](https://ieeexplore.ieee.org/document/9690029). The 10 parameters `θ = [α, d₁, d₂, d₃, s₁₂, s₂₃, s₁₃, t₁, t₂, t₃]` are the entries of a lower-triangular matrix whose product `LLᵀ` is the 4×4 pseudo-inertia matrix. Physical consistency (positive mass, positive-definite inertia tensor) is guaranteed by construction for any `θ`. |

Use `body_inertia_param` to create a `Parameter` with the right nominal values,
bounds, and modifier already wired up:

```python
from mujoco.sysid import body_inertia_param, InertiaType

param = body_inertia_param(
  spec, model, "link1",
  inertia_type=InertiaType.Pseudo,
)
params.add(param)
```

---

### 2. Package Data

Measured data is stored as **`TimeSeries`** objects (frozen dataclass: `times`,
`data`, optional `signal_mapping`):

```python
from mujoco.sysid import TimeSeries, SignalType

# For sensor/state observations:
sensordata = TimeSeries.from_names(times, sensor_data, model)     # all sensors
sensordata = TimeSeries.from_names(times, data, model, names=["joint1_pos", "joint2_pos"])

# Explicit type disambiguation (useful when sensor/state names overlap):
sensordata = TimeSeries.from_names(times, data, model, names=[
    ("joint1_pos", SignalType.MjSensor),       # sensor named "joint1_pos"
    ("joint1_qpos", SignalType.MjStateQPos),   # joint state
])

# For control signals:
control = TimeSeries.from_control_names(times, control_data, model)  # all actuators
control = TimeSeries.from_control_names(times, data, model, names=["motor1_ctrl"])

# For custom/raw data:
ts = TimeSeries.from_custom_map(times, data, ["signal1", "signal2"])
```

`signal_mapping` is a dict `{name: (SignalType, indices)}` that labels which
columns of `data` correspond to which sensor/actuator.

**`TimeSeries` factory methods:**

| Constructor | Use case |
|---|---|
| `TimeSeries.from_names(times, data, model, names=None)` | Sensor/state data. If `names=None`, maps all model sensors. |
| `TimeSeries.from_control_names(times, data, model, names=None)` | Control signals. If `names=None`, maps all actuators. |
| `TimeSeries.from_custom_map(times, data, signals)` | Custom data with explicit signal definitions. |
| `TimeSeries(times, data)` | Raw arrays, no signal mapping. |
| `TimeSeries(times, data, signal_mapping)` | Named signals with explicit mapping. |

**`TimeSeries` methods:** `resample(new_times=, target_dt=)`, `interpolate(t)`,
`get(t)`, `save_to_disk(path)`, `load_from_disk(path)`, `dt_statistics()`,
`remove_from_beginning(t)`, `slice_by_name(ts, names)`.

Bundle a spec with one or more data sequences into a **`ModelSequences`**:

```python
from mujoco.sysid import ModelSequences, create_initial_state

initial_state = create_initial_state(model, qpos, qvel, act)

ms = ModelSequences(
  name="robot",
  spec=spec,
  sequence_name=["traj_1", "traj_2"],            # or a single string
  initial_state=[initial_state_1, initial_state_2],
  control=[control_1, control_2],
  sensordata=[sensordata_1, sensordata_2],
)
```

You can pass a single sequence (not wrapped in a list) and it will be
auto-promoted.

**Multiple `ModelSequences`:** Each `ModelSequences` carries its own `spec`, but
the optimizer applies the **same parameter vector `θ`** to all of them. This
enables joint optimization across different physical configurations. For example,
you might have the same robot arm recorded with and without a known payload
attached — two different specs (one has the payload body), two sets of recorded
data, but the inertial parameters of the arm links are shared. The residuals
from all `ModelSequences` are stacked and minimized jointly, giving a better-
conditioned problem than fitting each dataset independently.

---

### 3. Build the Residual Function

The **residual** is the vector of differences between simulated sensor readings
and recorded sensor data: `r(θ) = W(ȳ(θ) − y)`. Each element measures how
much the simulation with parameters `θ` disagrees with reality for one sensor
at one timestep. The optimizer's job is to find the `θ` that makes this vector
as small as possible (in the least-squares sense).

**`build_residual_fn`** captures data and configuration, returning a closure
that the optimizer will call repeatedly:

```python
from mujoco.sysid import build_residual_fn

residual_fn = build_residual_fn(
  models_sequences=[ms],
  # Optional overrides:
  modify_residual=...,      # custom residual logic
  custom_rollout=...,       # custom simulation
  sensor_weights=...,       # per-sensor weighting
  enabled_observations=..., # subset of sensors to use
)
```

#### How `residual_fn` works internally

The returned `residual_fn(x, params)` accepts `x` as either a **1-D vector**
(plain function evaluation) or a **2-D matrix** of shape `(n_params, n_fd)`
(batched finite-difference Jacobian evaluation, where each column is a
perturbed parameter vector). This is the key to parallelism.

For each column `i` of `x`:

1. `params.update_from_vector(x[:, i])` — writes the optimizer's current
   candidate values back into the `Parameter` objects so that each
   parameter's `.value` attribute reflects column `i` of `x`
2. `model_i = apply_param_modifiers(params, spec)` — iterates over every
   non-frozen parameter and calls its `modifier(spec, param)` callback,
   then compiles the mutated spec into an `MjModel`
3. Replicate `model_i` once per trajectory chunk (if you have `C` data
   sequences, you get `C` copies)

This produces a flat list of `n_fd * C` models. All of them are rolled out in
a **single call** to `mujoco.rollout.rollout`:

```python
datas = [mujoco.MjData(models[0]) for _ in range(n_threads)]  # one per thread

state, sensordata = mujoco.rollout.rollout(
    models,          # n_fd * C models
    datas,           # K thread-local scratch MjData objects
    initial_states,  # n_fd * C initial states
    control,         # n_fd * C control sequences
)
```

MuJoCo's rollout engine distributes the `n_fd * C` independent rollouts across
`K` threads using the `MjData` objects as thread-local scratch space (each
thread gets its own `MjData` to avoid data races). **This is why the Jacobian
computation is fast**: all `n_params + 1` perturbed rollouts (times `C`
trajectory chunks) execute in one batched, multithreaded call.

After rollout, residuals are computed per-trajectory (predicted vs. measured
sensor data), then stacked and returned.

#### Concrete example

Suppose you have `p = 10` parameters and `C = 3` trajectory chunks:

- **Function eval** (`x` is 1-D): `1 * 3 = 3` rollouts, distributed across
  threads.
- **Jacobian eval** (`x` is `(10, 11)` — nominal + 10 perturbations): `11 * 3
  = 33` rollouts in one batched call. On a 16-core machine this is ~2x wall
  time of a single rollout.

#### Three tiers of customization

| Tier | What you provide | When to use |
|---|---|---|
| **Default** | Nothing extra (or `SignalTransform`) | Standard MuJoCo sensors, optional delays/gains |
| **Custom rollout** | `custom_rollout=fn` | Non-standard simulation (e.g. task-space control) |
| **Custom residual** | `modify_residual=fn` | State-based observations, exotic loss functions |

---

### 4. SignalTransform (Declarative Residual Configuration)

After simulation, the residual pipeline compares predicted sensor readings to
recorded data. But real sensors aren't ideal — position encoders may lag by a
few milliseconds, torque sensors may have an unknown scale factor, and velocity
estimates may sit at a nonzero offset. These aren't physics parameters (you
can't set "delay" on an `MjSpec`), so they need to be corrected *after* the
rollout, before the residual is computed.

**`SignalTransform`** lets you declare these corrections and which sensors to
use, without writing a custom residual callback. Internally it:

1. **Time-shifts** the predicted (or measured) signals by per-sensor delay
   parameters, resampling onto a common time grid.
2. **Scales** sensor columns by gain parameters (`target="predicted"` scales
   the simulation output, `target="measured"` scales the recording — useful
   when the sensor's scale factor is unknown on either side).
3. **Offsets** sensor columns by bias parameters.
4. Computes the weighted difference and normalizes by RMS.

Patterns use `fnmatch` syntax, so `"*_pos"` matches all sensors whose name
ends in `_pos`:

```python
from mujoco.sysid import SignalTransform

transform = SignalTransform()
transform.delay("*_pos", params["delay_pos"])       # fnmatch pattern
transform.delay("*_torque", params["delay_torque"])
transform.gain("*_torque", params["torque_scale"], target="predicted")
transform.bias("*_vel", params["vel_bias"])
transform.enable_sensors(["joint1_pos", "joint2_pos", "joint1_torque"])
transform.set_sensor_weights({"joint1_torque": 0.5})

residual_fn = build_residual_fn(
  models_sequences=[ms],
  modify_residual=transform.apply,   # drop-in replacement
)
```

`SignalTransform.apply` has the same signature as `ModifyResidualFn`, so it
plugs directly into `build_residual_fn`.

#### What this replaces

Without `SignalTransform`, you'd write the same logic by hand as a
`modify_residual` callback using the low-level `signal_modifier` functions
(Section 8):

```python
from mujoco.sysid._src import signal_modifier

def modify_residual(params, predicted, measured, model, return_pred_all, **kw):
    # 1. Apply delays and resample onto a common time grid.
    min_d, max_d = -0.02, 0.05  # must track delay bounds yourself
    measured = signal_modifier.apply_delayed_ts_window(measured, predicted, min_d, max_d)
    sensor_delays = {"joint1_pos": params["delay_pos"].value[0], ...}
    predicted = signal_modifier.apply_resample_and_delay(
        predicted, measured.times, default_delay=0.0, sensor_delays=sensor_delays,
    )
    # 2. Apply gains and biases.
    predicted = signal_modifier.apply_gain(predicted, "joint1_torque", params["torque_scale"])
    predicted = signal_modifier.apply_bias(predicted, "joint1_vel", params["vel_bias"])
    # 3. Compute residual.
    diff = signal_modifier.weighted_diff(predicted.data, measured.data, model, weights)
    diff = signal_modifier.normalize_residual(diff, measured.data)
    return diff, predicted, measured
```

`SignalTransform` does all of this — including tracking delay bounds, expanding
fnmatch patterns to sensor names, and handling the windowing/resampling
bookkeeping — from a few declarative lines.

---

### 5. Optimize

**`optimize`** runs box-constrained Gauss-Newton least-squares on the residual
function:

```python
from mujoco.sysid import optimize

opt_params, opt_result = optimize(
  initial_params=params,
  residual_fn=residual_fn,
  optimizer="mujoco",             # "mujoco", "scipy", or "scipy_parallel_fd"
  max_iters=200,
)
```

#### Optimizer backends

| Backend | Jacobian | Description |
|---|---|---|
| `"mujoco"` (recommended) | Parallel FD, batched | `mujoco.minimize.least_squares`. Calls `residual_fn(x)` with `x` as a 2-D matrix `(n_params, n_params+1)` — the nominal point plus one perturbation per parameter — so the entire Jacobian is computed in a single batched, multithreaded rollout call. |
| `"scipy"` | Sequential 2-point FD | `scipy.optimize.least_squares`. Computes the Jacobian column-by-column (sequential). Slower for problems with many parameters. |
| `"scipy_parallel_fd"` | Parallel FD via MuJoCo, scipy outer loop | Scipy's trust-region solver but with `mujoco.minimize.jacobian_fd` for the Jacobian. Gives scipy's convergence control with MuJoCo's batched FD speed. |

All three backends solve box-constrained nonlinear least-squares using the
Gauss-Newton Hessian approximation `H ≈ JᵀJ`. They differ in how the
finite-difference Jacobian is computed and how the trust-region step is
handled: `"mujoco"` uses projected Gauss-Newton steps, while `"scipy"` and
`"scipy_parallel_fd"` use scipy's trust-region reflective algorithm (`trf`).

#### Return value

Returns `(opt_params, OptimizeResult)` where `opt_params` is a deep copy of
the input `ParameterDict` with `.value` set to the solution, and
`OptimizeResult` contains:
- `.x` — the solution vector
- `.jac` — the Jacobian at the solution (used for confidence intervals)
- `.grad` — the gradient at the solution
- `.extras` — (**mujoco backend only**) dict with `"objective"` (cost per
  iteration) and `"candidate"` (parameter vector per iteration), when verbose

---

### 6. Save Results and Report

**`save_results`** writes everything to disk:

```python
from mujoco.sysid import save_results

save_results(
  experiment_results_folder="results/exp01",
  models_sequences=[ms],
  initial_params=params,
  opt_params=opt_params,
  opt_result=opt_result,
  residual_fn=residual_fn,
)
```

This creates:
- `params_x_0.yaml` &mdash; initial parameter values
- `params_x_hat.yaml` &mdash; optimized parameter values
- `results.pkl` &mdash; full `OptimizeResult`
- `confidence.pkl` &mdash; parameter covariance matrix `Σ_θ = σ²_r H⁻¹` and
  per-parameter confidence intervals, computed from the eigendecomposition of
  `H = JᵀJ` at the solution. Parameters in near-null-space directions of `H`
  receive infinite confidence intervals, making identifiability issues
  immediately visible.
- `{model_name}.xml` &mdash; identified MuJoCo XML for each model

**`default_report`** generates an HTML report with sensor comparisons, parameter
tables, and videos:

```python
from mujoco.sysid import default_report

default_report(
  models_sequences=[ms],
  initial_params=params,
  opt_params=opt_params,
  opt_result=opt_result,
  residual_fn=residual_fn,
  save_dir="results/exp01",
)
```

---

### 7. Model Modification

`apply_param_modifiers` is the default `build_model` implementation — it
iterates over all non-frozen parameters, calls each one's `modifier` callback
on the spec, and compiles. Most users never need to call it directly; it runs
automatically inside the residual pipeline.

The remaining functions are useful when writing a **custom `build_model`**
(e.g. the box case study manually mutates the spec instead of using modifier
callbacks):

| Function | Description |
|---|---|
| `apply_param_modifiers(params, spec)` | Run all modifier callbacks, return compiled `MjModel` |
| `apply_param_modifiers_spec(params, spec)` | Run all modifier callbacks, return the `MjSpec` |
| `apply_pgain(spec, name, value)` | Set proportional gain on a position actuator |
| `apply_dgain(spec, name, value)` | Set derivative gain on a position actuator |
| `apply_pdgain(spec, name, value)` | Set both P and D gains |
| `apply_body_inertia(spec, name, param)` | Apply Mass / MassIpos / Pseudo inertia |
| `body_inertia_param(spec, model, name, ...)` | Create a `Parameter` for body inertia |
| `remove_visuals(spec)` | Strip textures, materials, and visual-only geoms |

---

### 8. Signal Modification (Power-User API)

Low-level functions used internally by `SignalTransform` and the default
residual pipeline. Useful when writing a custom `modify_residual`:

| Function | Description |
|---|---|
| `get_sensor_indices(model, name)` | Column indices for a named sensor |
| `apply_gain(ts, name, param)` | Multiply sensor columns by `param.value` |
| `apply_bias(ts, name, param)` | Add `param.value` to sensor columns |
| `apply_delay(ts, name, param)` | Time-shift sensor columns |
| `apply_delayed_ts_window(ts, ts_ref, min_d, max_d)` | Crop `ts` to the valid time window |
| `apply_resample_and_delay(ts, times, default_delay, ...)` | Resample with per-sensor delays |
| `weighted_diff(pred, meas, model, weights)` | `measured - predicted`, optionally weighted |
| `normalize_residual(residual, measured)` | Divide by column-wise RMS of measured data |

---

### 9. Additional Utilities

| Function / Class | Module | Description |
|---|---|---|
| `create_initial_state(model, qpos, qvel, act)` | trajectory | Pack qpos/qvel/act into a flat state vector |
| `SystemTrajectory` | trajectory | Frozen dataclass holding a single rollout (model, control, sensordata, state) |
| `sysid_rollout(models, datas, control, initial_states)` | trajectory | Parallel MuJoCo rollout returning `SystemTrajectory` list |
| `render_rollout(model, data, state, framerate)` | plotting | Render state trajectories to pixel frames |
| `calculate_intervals(residuals, J, alpha)` | optimize | Confidence intervals from Jacobian at the solution |
| `sweep_parameter(params, name, values, residual_fn)` | optimize | 1-D parameter sweep returning cost curve |
| `plot_sensor_comparison(model, ...)` | plotting | Matplotlib overlay of predicted vs. measured sensors |
| `SignalType` | timeseries | Enum: `MjSensor`, `CustomObs`, `MjStateQPos`, `MjStateQVel`, `MjStateAct`, `MjCtrl` |

---

## Type Aliases

```python
ModifyResidualFn = Callable[
  ..., tuple[np.ndarray, TimeSeries, TimeSeries]
]
# (params, sensordata_predicted, sensordata_measured, model, return_pred_all, state=..., sensor_weights=...)
# Returns (residual_array, pred_timeseries, measured_timeseries)

CustomRolloutFn = Callable[..., Sequence[SystemTrajectory]]
# (models, datas, control_signal, initial_states, param_dicts, ...)
# Returns list of SystemTrajectory

BuildModelFn = Callable[[ParameterDict, MjSpec], MjModel]
# Default: apply_param_modifiers
```

---

## Skeleton Case Study

Pseudocode showing the five-stage pipeline. Replace the data-loading step with
your own hardware logs or simulation data. For a complete runnable example, see
`case_studies/box/`.

```python
import mujoco
import numpy as np

from mujoco.sysid import (
  Parameter,
  ParameterDict,
  TimeSeries,
  ModelSequences,
  build_residual_fn,
  create_initial_state,
  optimize,
  save_results,
)

# 1. Load model.
spec = mujoco.MjSpec.from_file("robot.xml")
model = spec.compile()

# 2. Define parameters with modifier callbacks.
params = ParameterDict()
params.add(Parameter(
  "link1_mass",
  nominal=2.0,
  min_value=0.5,
  max_value=5.0,
  modifier=lambda spec, p: setattr(spec.body("link1"), "mass", p.value[0]),
))

# 3. Package recorded data.
#    times:        (N,) timestamps
#    ctrl_array:   (N, model.nu) control inputs
#    sensor_array: (N, model.nsensordata) recorded sensor readings
#    qpos_0, qvel_0: initial joint positions and velocities
control = TimeSeries.from_control_names(times, ctrl_array, model)
sensordata = TimeSeries.from_names(times, sensor_array, model)
initial_state = create_initial_state(model, qpos_0, qvel_0)

ms = ModelSequences(
  name="robot",
  spec=spec,
  sequence_name="traj_1",
  initial_state=initial_state,
  control=control,
  sensordata=sensordata,
)

# 4. Build the residual function and optimize.
#    models_sequences is a list because you can jointly optimize across
#    multiple ModelSequences with different specs (see Section 2).
residual_fn = build_residual_fn(models_sequences=[ms])
opt_params, opt_result = optimize(
  initial_params=params,
  residual_fn=residual_fn,
  optimizer="mujoco",
)

# 5. Inspect results.
print(opt_params)
save_results("results/", [ms], params, opt_params, opt_result, residual_fn)
```
