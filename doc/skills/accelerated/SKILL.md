---
name: mujoco-accelerated
description: >-
  GPU and TPU batch simulation, MJX (JAX), MJWarp (CUDA), parallel rollouts
  (jax.vmap), trajectory scan (jax.lax.scan), differentiability, device memory
  (put_model, put_data), static contact sizing (max_contact_points,
  max_geom_pairs, nconmax, njmax), float32 precision, XLA compilation caching.
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
> MJX and MJWarp are **Python-only** accelerated backends. Standard CPU
> simulation runs in full `float64` via `import mujoco`.

## 1. Backend Comparison: C++ vs MJX vs MJWarp

| Feature | C++ Engine (Default) | MJX (JAX) | MJWarp (CUDA) |
| :--- | :--- | :--- | :--- |
| **Import** | `import mujoco` | `from mujoco import mjx` | `import mujoco_warp as mjw` |
| **Hardware** | CPU | CPU / GPU / TPU | NVIDIA GPU |
| **Precision** | float64 | float32 | float32 |
| **Differentiable** | ❌ No | ✅ Yes (JAX gradients) | ❌ No |
| **Batch Stepping** | ❌ Serial / multi-threaded | ✅ `jax.vmap` | ✅ Native CUDA batches |
| **Solvers** | Newton, CG, PGS, **noslip** | Newton, CG | Newton, CG |
| **Constraint Islands**| ✅ Yes | ❌ No | ✅ Yes |
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
mjx_model = mjx.put_model(model)
mjx_data = mjx.put_data(model, data)

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

## 4. Static Contact and Constraint Sizing

JAX requires static array shapes, so contact and constraint buffers are sized
when the device data is created, not per step.

### MJX (default JAX implementation)

The contact buffer is sized **from the model**, not from `data.ncon`: every geom
pair that can collide (after `contype`/`conaffinity`, `exclude` and `pair`
filtering) gets a fixed number of contact slots, which is why large scenes
produce large `efc_*` arrays and slow compilation. `put_data` raises a
`ValueError` if the CPU `MjData` already holds more contacts than this capacity.
The `njmax` argument of `mjx.put_data` is ignored here.

To shrink the buffer, cap it with custom numerics in the MJCF:

```xml
<custom>
  <!-- keep only the N closest geom pairs (bounding spheres) per geom-type pair -->
  <numeric name="max_geom_pairs" data="16"/>
  <!-- keep only the N deepest contacts per condim group -->
  <numeric name="max_contact_points" data="32"/>
</custom>
```

Also reduce the number of candidate pairs with `contype`/`conaffinity` and
`<exclude>`, and prefer primitive geoms over meshes.

### MJWarp (and `mjx.put_data(..., impl='warp')`)

Buffers are sized by explicit arguments; omitted values fall back to defaults
derived from the model:

-   `nconmax`: contacts per world (MJWarp `put_data`/`make_data`).
-   `naconmax`: contacts across all worlds.
-   `njmax`: constraint rows (`efc`) **per world**, a hard cap per world.

`put_data` raises if the input `MjData` exceeds a capacity. During stepping,
excess contacts/constraints are dropped, a warning is printed, and the per-world
`d.overflow` bitmask (`mjw.OverflowType`) is set, so size for the peak.

## 5. MJX Named Access & `bind()`

MJX provides declarative named access over JAX arrays via `bind()`:

```python
spec = mujoco.MjSpec.from_file('model.xml')
model = spec.compile()
data = mujoco.MjData(model)

mx = mjx.put_model(model)
dx = mjx.put_data(model, data)

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
data = mujoco.MjData(model)

# Transfer to CUDA: one model, nworld copies of the data
mjw_model = mjw.put_model(model)
mjw_data = mjw.put_data(model, data, nworld=4096, nconmax=64, njmax=256)
# or allocate fresh: mjw.make_data(model, nworld=4096, nconmax=64, njmax=256)

# Advance all 4096 environments in parallel (in place)
mjw.step(mjw_model, mjw_data)

# Fields are Warp arrays with a leading world dimension
qpos = mjw_data.qpos.numpy()  # (4096, nq)

# Copy one world back to CPU MjData
mjw.get_data_into(data, model, mjw_data, world_id=0)
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
-   **Constraint Islands**: Not used by MJX (solved monolithically); supported
    by MJWarp.

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

-   [MJX Support Tests](../../../mjx/mujoco/mjx/_src/support_test.py)
