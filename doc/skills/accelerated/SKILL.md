---
name: mujoco-accelerated
description: >-
  GPU and TPU batch simulation, MJX (JAX), MJWarp (CUDA), parallel rollouts
  (jax.vmap), trajectory scan (jax.lax.scan), differentiability, device memory
  (put_model, put_data), njmax contact pre-allocation, float32 precision, XLA
  compilation caching.
  Use for RL environment batches, GPU rollouts, differentiable physics.
  Do NOT use for CPU simulation, noslip/PGS solvers (mujoco-python), rendering (mujoco-rendering), or GUI (mujoco-gui).
---

# MuJoCo Accelerated: MJX & MJWarp GPU Backends

> [!TIP]
>
> **Related skills:**
>
> -   [mujoco-python](../python/SKILL.md) — CPU physics simulation, kinematics,
>     and full solver suite.
> -   [mujoco-studio](../studio/SKILL.md) — Interactive simulation and Studio
>     viewer plugins.

> [!IMPORTANT]
>
> MJX and MJWarp are **Python-only** GPU acceleration backends. Standard CPU
> simulation runs in full `float64` via `import mujoco`.

## 1. Backend Comparison: C++ vs MJX vs MJWarp

| Feature | C++ Engine (Default) | MJX (JAX) | MJWarp (CUDA) |
| :--- | :--- | :--- | :--- |
| **Import** | `import mujoco` | `from mujoco import mjx` | `import mujoco_warp as mjw` |
| **Hardware** | CPU | GPU / TPU | NVIDIA GPU |
| **Precision** | float64 | float32 | float32 |
| **Differentiable** | ❌ No | ✅ Yes (JAX gradients) | ❌ No |
| **Batch Stepping** | ❌ Serial / multi-threaded | ✅ `jax.vmap` | ✅ Native CUDA batches |
| **Solvers** | Newton, CG, PGS, **noslip** | Newton, CG | Newton, CG |
| **Constraint Islands**| ✅ Yes | ❌ No | ❌ No |
| **Plugins** | All engine plugins | ❌ No | SDF plugins only |

## 2. MJX Workflow (JAX)

MJX translates MuJoCo data structures into immutable JAX pytrees that execute
natively on GPUs and TPUs.

```python
import mujoco
from mujoco import mjx

# 1. Load CPU model and data
model = mujoco.MjModel.from_xml_path('scene.xml')
data = mujoco.MjData(model)

# 2. Transfer to device (GPU/TPU)
# ALWAYS specify njmax when contacts are present
mjx_model = mjx.put_model(model)
mjx_data = mjx.put_data(model, data, njmax=500)

# 3. Step on device (functional paradigm returns new data)
mjx_data = mjx.step(mjx_model, mjx_data)

# 4. Copy data back to host CPU
mjx.get_data_into(data, model, mjx_data)
print(data.body('torso').xpos)
```

## 3. High-Throughput Rollouts: jax.vmap and jax.lax.scan

### Parallel Batches (`jax.vmap`)

```python
import jax
from mujoco import mjx

# Vectorize step over batch dimension (model unbatched, data batched)
batched_step = jax.vmap(mjx.step, in_axes=(None, 0))

# JIT-compile for maximum device throughput
batched_step_jit = jax.jit(batched_step)
batch_data = batched_step_jit(mjx_model, batch_data)
```

### Multi-Step Trajectory Rollouts (`jax.lax.scan`)

For RL environment steps, avoid Python loops by scanning across steps:

```python
def rollout(model: mjx.Model, init_data: mjx.Data, ctrl_sequence: jax.Array):
  def step_fn(d, ctrl):
    d = d.replace(ctrl=ctrl)
    d = mjx.step(model, d)
    return d, (d.qpos, d.sensordata)

  final_data, trajectory = jax.lax.scan(step_fn, init_data, ctrl_sequence)
  return final_data, trajectory
```

## 4. Contact Pre-allocation (The `njmax` Gotcha)

> [!CAUTION]
>
> `mjx.put_data()` pre-allocates GPU memory for contacts based on `data.ncon`.
> On a newly initialized `MjData`, **`data.ncon` is 0**. If `njmax` is omitted,
> the device contact buffer size is 0 and **contacts will be silently ignored**.

Always provide `njmax`:
```python
# ✅ RIGHT — allocates capacity for up to 500 contacts
mjx_data = mjx.put_data(model, data, njmax=500)
```

A standard rule of thumb is 2× to 5× the expected peak number of contacts.

## 5. MJX Named Access & `bind()`

MJX provides declarative named access over JAX arrays via `bind()`:

```python
spec = mujoco.MjSpec.from_file('model.xml')
model = spec.compile()
data = mujoco.MjData(model)

mx = mjx.put_model(model)
dx = mjx.put_data(model, data, njmax=500)

# Read body positions from JAX device arrays
torso_pos = dx.bind(mx, spec.body('torso')).xpos

# Functional update of control inputs
dx = dx.bind(mx, spec.actuators).set('ctrl', [1.0, 0.5])
```

## 6. MJWarp Guide (NVIDIA CUDA)

MJWarp (`mujoco_warp`) provides a CUDA-native simulator optimized for maximum
batch throughput on NVIDIA hardware without JAX:

```python
import mujoco
import mujoco_warp as mjw

model = mujoco.MjModel.from_xml_path('scene.xml')
batch_size = 4096

# Transfer batch to CUDA
mjw_model = mjw.put_model(model)
mjw_data = mjw.put_data(model, batch_size)

# Advance all 4096 environments in parallel
mjw.step(mjw_model, mjw_data)

# Extract states directly into GPU tensors or NumPy arrays
qpos_tensor = mjw_data.qpos
```

## 7. Supported Geometries & Feature Gaps

### Collision Geom Support Matrix

| Geometry Pair | MJX (JAX) | MJWarp (CUDA) |
| :--- | :--- | :--- |
| Sphere - Sphere | ✅ Supported | ✅ Supported |
| Sphere - Plane | ✅ Supported | ✅ Supported |
| Capsule - Capsule | ✅ Supported | ✅ Supported |
| Capsule - Plane | ✅ Supported | ✅ Supported |
| Box - Plane | ✅ Supported | ✅ Supported |
| Mesh - Primitive | ✅ Convex meshes | ✅ Convex meshes |
| Mesh - Mesh | ⚠️ Performance sensitive | ✅ Warp accelerated |

### Solvers Not Available on GPU

-   **noslip solver**: Available only in C++ engine. If exact frictional stick
    conditions without sliding drift are required, use CPU.
-   **PGS solver**: Not available on GPU backends.
-   **Constraint Islands**: Handled monolithically on GPU.

## 8. Common Gotchas & Numerical Stability

1.  **Float32 precision**: GPU backends run in single precision (`float32`).
    Stiff systems may experience numerical drift or NaN errors. Counteract this
    by increasing `model.opt.iterations` or decreasing `model.opt.timestep`.
2.  **Immutability in MJX**: `mjx.Data` is a JAX pytree. In-place assignments
    fail; use `.replace()`:
    ```python
    mjx_data = mjx_data.replace(qpos=new_qpos)
    ```
3.  **Compilation caching**: Avoid long recompilations on startup by enabling
    persistent XLA caching:
    ```python
    import jax
    jax.config.update('jax_compilation_cache_dir', '/tmp/jax_cache')
    ```

## 9. Key References

### Documentation

-   [MJX Guide](../../mjx.rst)
-   [Python Bindings Reference](../../python.rst)

### Source Code Examples

-   [MJX Support Tests](../../../python/mujoco/mjx/_src/support_test.py)
