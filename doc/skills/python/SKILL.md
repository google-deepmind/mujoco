---
name: mujoco-python
description: >-
  Python simulation lifecycle (MjSpec, MjModel, MjData), simulation stepping
  (mj_step, mj_step1, mj_step2, mj_forward), named access (data.body, model.geom),
  bind(), contact and touch sensors, sensordata, keyframe resets, integrators,
  mju_ spatial math (quaternions, poses), energy diagnostics.
  Use for Python physics simulation, state access, stepping loops, sensors.
  Do NOT use for model building/editing (mujoco-spec-editing), headless rendering (mujoco-rendering), GPU/TPU (mujoco-accelerated), or GUI/viewers (mujoco-gui, mujoco-studio).
---

# MuJoCo Python: Simulation, Sensors & Math

> [!TIP]
>
> **Related skills:**
>
> -   [mujoco-spec-editing](../spec_editing/SKILL.md) — Procedural model
>     editing and composing scenes with `MjSpec`.
> -   [mujoco-rendering](../rendering/SKILL.md) — Offscreen image rendering,
>     cameras, depth, and segmentation.
> -   [mujoco-accelerated](../accelerated/SKILL.md) — GPU/TPU physics simulation
>     with MJX (JAX) and MJWarp (CUDA).
> -   [mujoco-studio](../studio/SKILL.md) — Interactive viewers, Studio viewer
>     apps, and plugin architecture.

> [!CAUTION]
>
> -   **NEVER use `mj_name2id`, `model.name2id`, or raw integer indexing**
>     (e.g., `data.qpos[0]`, `data.xpos[body_id]`) to look up elements by name.
>     Always use **named access** (`data.body('name').xpos`) or **`bind()`**
>     instead.
> -   **Do not use dm_control / PyMJCF lookup patterns.** `model.find('body',
>     'name')`, `.find_body()`, and `physics.named` do **not exist** in
>     MuJoCo 3.x. Use direct accessors: `.body('name')`, `.geom('name')` etc. on
>     `MjSpec`, `MjModel`, and `MjData`.

## 1. Compilation Lifecycle: MjSpec → MjModel → MjData

MuJoCo models follow a three-stage lifecycle:

```text
MjSpec  ──spec.compile()──▶  MjModel  ──MjData(model)──▶  MjData
  │                             │                            │
  │  (mutable blueprint)        │  (compiled, mostly frozen) │ (simulation state)
  │                             │                            │
  └── spec.recompile(m, d) ─────┴────────────────────────────┘
```

1.  **`MjSpec`**: Mutable data structure defining the simulation elements and
    hierarchy.
2.  **`MjModel`**: Compiled, optimized physics model containing constant and
    precomputed parameters. Structural changes require recompilation.
3.  **`MjData`**: Dynamic simulation state (positions, velocities, forces,
    contacts, diagnostic buffers).

### Python Lifecycle

```python
import mujoco

# 1. Build or load spec
spec = mujoco.MjSpec()
body = spec.worldbody.add_body(name='pendulum', pos=[0, 0, 1])
body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.1])
body.add_joint(name='hinge', type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])

# 2. Compile to model and instantiate data
model = spec.compile()
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# 3. Dynamic recompilation (preserves existing state)
body2 = spec.worldbody.add_body(name='ball2', pos=[1, 0, 1])
body2.add_geom(size=[0.1])
body2.add_freejoint()
model, data = spec.recompile(model, data)  # Always reassign return values
```

### C++ Lifecycle

```cpp
#include <mujoco/mujoco.h>

// 1. Create or load spec
mjSpec* spec = mj_makeSpec();
mjbBody* body = mj_addBody(spec, nullptr);
mj_addGeom(body, mjGEOM_SPHERE, 0.1, 0, 0);

// 2. Compile to model and create data
char error[1000] = {0};
mjModel* model = mj_compile(spec, error, sizeof(error));
mjData* data = mj_makeData(model);

// 3. Step or evaluate
mj_forward(model, data);

// 4. Clean up allocated resources
mj_deleteData(data);
mj_deleteModel(model);
mj_deleteSpec(spec);
```

### Loading and Serializing

```python
# Direct model loading from file or string
model = mujoco.MjModel.from_xml_path('model.xml')
model = mujoco.MjModel.from_xml_string(xml_str)
model = mujoco.MjModel.from_binary_path('model.mjb')  # Precompiled binary format

# Loading mutable spec from file or string
spec = mujoco.MjSpec.from_file('model.xml')
spec = mujoco.MjSpec.from_string(xml_str)

# Serialization
xml_string = spec.to_xml()
spec.encode('model.xml', model)
```

> [!CAUTION]
>
> `mujoco.load_model_from_path` does **not exist** in MuJoCo 3.x. Use
> `mujoco.MjModel.from_xml_path(path)` or `mujoco.MjSpec.from_file(path)`.

## 2. Accessing State: Named Access & bind()

### 1. Named Access (on `MjModel` and `MjData`)

In Python, access elements by calling the element-type accessor with the element
name:

```python
# Reading state
torso_pos = data.body('torso').xpos      # NumPy view (3,)
knee_angle = data.joint('knee').qpos     # Shape depends on joint type
ball_size = model.geom('ball').size      # NumPy view (3,)

# Writing controls and state
data.actuator('motor').ctrl = 1.0        # Writable view
data.joint('knee').qpos = 0.5
```

Supported element accessors: `body`, `geom`, `joint` (alias `jnt`), `site`,
`camera` (alias `cam`), `light`, `mesh`, `skin`, `texture` (alias `tex`),
`material` (alias `mat`), `pair`, `equality` (alias `eq`), `tendon` (alias
`ten`), `actuator`, `sensor`, `numeric`, `text`, `tuple`, `keyframe` (alias
`key`).

> [!WARNING]
>
> **Named access returns views, not copies.** In a simulation loop, modifying
> the simulation changes the array in-place. If logging or storing history,
> **always call `.copy()`**:
>
> ```python
> # ❌ WRONG — all history items will point to the same final state
> history.append(data.body('torso').xpos)
>
> # ✅ RIGHT — records distinct coordinates
> history.append(data.body('torso').xpos.copy())
> ```

### 2. `bind()` (Bridges `MjSpec` elements → `MjModel` / `MjData`)

When building models programmatically with `MjSpec`, use `bind()` to map spec
element handles directly to compiled data:

```python
geom = spec.worldbody.add_geom(name='ball', size=[0.1])
joint = spec.worldbody.add_joint(name='hinge', type=mujoco.mjtJoint.mjJNT_HINGE)
model = spec.compile()
data = mujoco.MjData(model)

# Read through bind
radius = model.bind(geom).size[0]
ball_pos = data.bind(geom).xpos

# Write through bind — always use .set()
data.bind(joint).set('qpos', 0.5)

# Bind lists of spec elements in batch
joints = [spec.joint('j1'), spec.joint('j2')]
joint_angles = data.bind(joints).qpos
```

## 3. Simulation Stepping & Execution Pipeline

### Stepping Pipeline (`mj_step` vs `mj_step1` / `mj_step2`)

-   **`mj_step(model, data)`**: Advances the simulation by one timestep
    (`model.opt.timestep`). It computes kinematics, forces, collision constraints,
    accelerations, and numerical integration in one full pass.
-   **Split Stepping (`mj_step1` + `mj_step2`)**: Used when custom controller
    forces or external computations depend on the current kinematic state:

```python
# 1. Advance position kinematics and sensor computations
mujoco.mj_step1(model, data)

# 2. Apply control torques or external forces based on current kinematics
data.ctrl[:] = compute_feedback_control(data.qpos, data.qvel)
data.qfrc_applied[:] = compute_external_forces(data)

# 3. Complete dynamic acceleration and integration
mujoco.mj_step2(model, data)
```

### Sensor and Kinematic Initialization (`mj_forward`)

Before reading sensors or positions at time zero, call `mj_forward()` instead of
`mj_step()`. `mj_forward` evaluates kinematics and sensor outputs from the
current state **without advancing simulation time**:

```python
# ✅ RIGHT — initialize sensors and kinematics at t=0
mujoco.mj_forward(model, data)
initial_obs = data.sensordata.copy()

# Advance physics
for _ in range(num_steps):
  mujoco.mj_step(model, data)
```

### Keyframe Resets & Restoring State

Use keyframes to reset configurations reliably:

```python
# Python: reset to named keyframe
key_id = model.key('home').id
mujoco.mj_resetDataKeyframe(model, data, key_id)
mujoco.mj_forward(model, data)  # Recompute kinematics and sensors
```

In C++:
```cpp
int key_id = mj_name2id(m, mjOBJ_KEY, "home");
if (key_id >= 0) {
  mj_resetDataKeyframe(m, d, key_id);
  mj_forward(m, d);
}
```

### Integrators & Numerical Stability

Configured via `model.opt.integrator`:
-   `mjINT_EULER` (default in earlier models): Standard semi-implicit Euler. Fast
    but can suffer from instability with stiff springs or damping.
-   `mjINT_RK4`: 4th-order Runge-Kutta. Highly accurate for smooth, passive
    systems without contacts; not suitable for contact-rich scenes.
-   `mjINT_IMPLICIT` / `mjINT_IMPLICITFAST`: Backward Euler with analytical or
    finite-difference derivatives. Provides superior numerical stability for
    stiff tendons, actuators, and complex contact manifolds.

## 4. Contact & Force Sensing: Declarative Sensors

### Why `data.contact` Should NOT Be Used for Observations

`data.contact` is a variable-length buffer that changes size and ordering every
timestep based on active collision pairs:

```python
# ❌ WRONG — variable size and non-deterministic order break ML policies
for c in data.contact:
  if c.geom1 == target_id:
    force = ...
```

### Declarative Contact Sensors

`<contact>` sensors declaratively filter contacts, reduce them to a fixed
number of slots, and write deterministic vectors directly into `data.sensordata`:

```xml
<sensor>
  <!-- Detect contacts between gripper and object: fixed 3 slots -->
  <contact name="grip_contact"
           body1="gripper" body2="object"
           num="3" data="found force normal"
           reduce="maxforce"/>

  <!-- Net wrench on an entire body -->
  <contact name="object_net"
           body1="object"
           data="force torque"
           reduce="netforce"/>
</sensor>
```

Reading contact sensors in Python:
```python
contact_data = data.sensor('grip_contact').data  # Always fixed length
```

### Touch Sensors for Scalar Normal Force

For scalar surface contact forces (e.g., fingertips), use `touch` sensors
attached to a site. Make sure the site is large enough to encompass the geom:

```python
site = body.add_site(name='fingertip', pos=[0, 0, 0.05], size=[0.02])
spec.add_sensor(
    name='touch_sensor',
    type=mujoco.mjtSensor.mjSENS_TOUCH,
    objname=site.name,
    objtype=mujoco.mjtObj.mjOBJ_SITE,
)
```

## 5. Spatial Mathematics (mju_ Library)

MuJoCo provides an optimized spatial mathematics library under the `mju_`
namespace in both C++ and Python.

> [!IMPORTANT]
>
> MuJoCo quaternions are **scalar-first**: `[w, x, y, z]`. SciPy and ROS use
> **scalar-last**: `[x, y, z, w]`. Swizzle components when converting:
>
> ```python
> # MuJoCo -> SciPy/ROS
> scipy_quat = mj_quat[[1, 2, 3, 0]]
>
> # SciPy/ROS -> MuJoCo
> mj_quat = scipy_quat[[3, 0, 1, 2]]
> ```

### Quaternion Operations

```python
res = np.zeros(3)
mujoco.mju_rotVecQuat(res, vec, quat)         # Rotate vector by quaternion

quat = np.zeros(4)
mujoco.mju_mat2Quat(quat, mat3x3)             # 3x3 rotation matrix → quaternion
mujoco.mju_quat2Mat(mat, quat)                # Quaternion → 3x3 matrix
mujoco.mju_axisAngle2Quat(quat, axis, angle)  # Axis-angle → quaternion
mujoco.mju_euler2Quat(quat, euler, 'xyz')     # Euler angles → quaternion
mujoco.mju_mulQuat(res_q, q1, q2)             # Compose quaternions (q1 * q2)
mujoco.mju_negQuat(res_q, quat)               # Quaternion conjugate (inverse)
mujoco.mju_quatZ2Vec(quat, vec)               # Quaternion rotating Z-axis to vector
```

### Pose Operations

```python
# Compose poses (pos1/quat1 then pos2/quat2)
mujoco.mju_mulPose(pos_res, quat_res, pos1, quat1, pos2, quat2)

# Invert pose
mujoco.mju_negPose(pos_res, quat_res, pos, quat)

# Transform 3D point by pose
mujoco.mju_trnVecPose(res, pos, quat, vec)
```

## 6. Energy & Diagnostics

MuJoCo can continuously compute total mechanical energy to diagnose simulation
stability:

```python
# Enable energy monitoring
model.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY

mujoco.mj_step(model, data)
potential_energy = data.energy[0]
kinetic_energy = data.energy[1]
total_energy = potential_energy + kinetic_energy
```

In passive, unforced, undamped systems, `total_energy` must remain constant.
Growing energy indicates numerical integration instability (requiring smaller
timesteps or implicit integrators).

## 7. Compute Backend Selection

| Feature | C++ Engine (Default) | MJX (JAX) | MJWarp (CUDA) |
| :--- | :--- | :--- | :--- |
| **Import** | `import mujoco` | `from mujoco import mjx` | `import mujoco_warp as mjw` |
| **Primary Use** | Real-time control, interactive tools, full solver support | Differentiable physics, large batch RL | High-throughput batch rollouts |
| **Hardware** | CPU | GPU / TPU | NVIDIA GPU |
| **Precision** | float64 | float32 | float32 |
| **Solvers** | Newton, CG, PGS, noslip | Newton, CG | Newton, CG |
| **Plugins** | All plugins | Not supported | SDF only |

*(For detailed GPU and TPU execution patterns, refer to [`mujoco-accelerated`](../accelerated/SKILL.md).)*

## 8. Common Gotchas & Best Practices

1.  **Computed fields are read-only**: Fields like `data.xpos`, `data.xmat`, and
    `data.xquat` are output buffers. Never assign to them directly; change
    inputs (`data.qpos`, `data.ctrl`) and call `mj_forward()` or `mj_step()`.
2.  **Duplicate names are forbidden**: Spec element names must be unique within
    each element type.
3.  **Orientation keywords are mutually exclusive**: When adding geoms, bodies,
    or sites, provide only one of `quat`, `euler`, `axisangle`, `xyaxes`, or
    `zaxis`.
4.  **Euler sequences**: Lowercase characters denote intrinsic rotations
    (`'xyz'`), uppercase denote extrinsic rotations (`'XYZ'`).
5.  **Coordinate convention**: +Z is up, gravity is `[0, 0, -9.81]`.
6.  **Repeating controls with `nstep`**: Calling `mj_step(model, data, nstep=N)`
    repeats the current control across `N` steps. For dynamic feedback control,
    step one timestep at a time.

## 9. Key References

### Documentation

-   [Python Bindings Reference](../../python.rst)
-   [Simulation & Computation Guide](../../programming/simulation.rst)
-   [C API Functions](../../APIreference/APIfunctions.rst)
-   [C API Types & Structs](../../APIreference/APItypes.rst)

### Source Code Examples

-   [Python Bindings Tests](../../../python/mujoco/bindings_test.py)
-   [C API Header](../../../include/mujoco/mujoco.h)
