# System Identification Toolbox

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.png)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mujoco/sysid/sysid.ipynb)

Given a MuJoCo model and recorded sensor data, find parameters
that make simulation match reality. By default, the library uses
nonlinear least-squares with box constraints to minimize the difference
between measured and simulated (predicted) outputs. Residuals
can be modified by static or optimized parameters, such as
weights and time-delays.

The optimizer uses Gauss-Newton with finite-difference Jacobians. Each
parameter perturbation requires an independent simulation rollout. All
of them execute in a single batched call to `mujoco.rollout`, parallelized
across threads.

## Pipeline

**You provide:**
- One or more `ModelSequences` each bundling a single `MjSpec` with one or
  more sequences of measured data. All will be optimized jointly.
- A `ParameterDict` defining differentiable `Parameter`s with bounds.
- Callbacks to apply `Parameter`s to an `MjSpec` (individually or jointly)
- (optional) Functions (`build_model`, `custom_rollout`, `modify_residual`)
  that override the default residual function behavior.

**The framework:**
- Composes user code into a residual function (`build_residual_fn`).
- Optimizes parameters via batched parallel rollouts (`optimize`).
- Saves results and generates an HTML report (`save_results`, `default_report`).

## What Can You Identify?

You can optimize any parameter that differentiably modifies the final
residuals. Common use cases include:

**Physics parameters** settable on `MjSpec`. Most parameters in MjSpec
can be easily set directly by user-provided callbacks:

| Target | Approach |
|---|---|
| Contact sliding friction | `spec.pair("cp").friction[0] = p.value[0]` |
| Joint damping | `spec.joint("j1").damping = p.value[0]` |

Convenience functions are provided for common system identification
parameterizations that cannot be trivially applied to an MjSpec:

| Target | Approach |
|---|---|
| Body mass | `body_inertia_param(..., InertiaType.Mass)` |
| Body mass + center of mass | `body_inertia_param(..., InertiaType.MassIpos)` |
| Full inertia (10-D) | `body_inertia_param(..., InertiaType.Pseudo)` |
| Actuator P/D gains | `apply_pdgain(spec, "act1", p.value)` |

Full inertia uses the pseudo-inertia Cholesky parameterization
([Rucker & Wensing 2022](https://ieeexplore.ieee.org/document/9690029)),
guaranteeing physical consistency without singularities.

**Measurement parameters** such as sensor delays, gains, and biases are
properties of the measurement system, not the physics model. The library
provides utilities for applying these corrections to the residual after
rollout. They are functionally complete but their API is not yet final.

## Example

```python
import mujoco
from mujoco import sysid

# 1. Load model and define parameters.
spec = mujoco.MjSpec.from_file("robot.xml")
model = spec.compile()

def set_link1_mass(spec, p):
    spec.body("link1").mass = p.value[0]

params = sysid.ParameterDict()
params.add(sysid.Parameter(
    "link1_mass", nominal=2.0, min_value=0.5, max_value=5.0,
    modifier=set_link1_mass))

# 2. Load and package measured data.
# arrays assumed to be in MuJoCo order, otherwise pass names argument
control = sysid.TimeSeries.from_control_names(times, ctrl_array, model)
measureddata = sysid.TimeSeries.from_names(times, measurement_array, model)
initial_state = sysid.create_initial_state(model, qpos_0, qvel_0)
ms = sysid.ModelSequences("robot", spec, "traj_1", initial_state, control, measureddata)

# 3. Build residual, optimize, save.
residual_fn = sysid.build_residual_fn(models_sequences=[ms])
opt_params, opt_result = sysid.optimize(initial_params=params, residual_fn=residual_fn)
sysid.save_results("results/", [ms], params, opt_params, opt_result, residual_fn)
```

`default_report` generates an interactive HTML report with videos, measurement comparisons,
parameter tables, and confidence intervals.
