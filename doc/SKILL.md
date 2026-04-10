---
name: mujoco-python
description: >
    Build, manipulate, and simulate MuJoCo physics models using the Python
    bindings (MjSpec, MjModel, MjData). Covers choosing between compute
    backends (C++ for full features and noslip, MJWarp for GPU batch RL). Use when constructing scenes
    programmatically via the spec API, compiling and stepping simulations,
    reading sensor/body/geom data, attaching sub-models, composing specs with
    prefixed names, using contact sensors for fixed-size observation spaces,
    configuring collision filtering, offscreen rendering (context management,
    cameras, depth/segmentation), or performing spatial math (quaternion, pose,
    rotation conversions via mju_). Covers gotchas around compilation
    lifecycle, named indexing vs bind, geom size semantics, camera conventions,
    and orientation representations.
---

# MuJoCo Python Bindings

## Compilation Lifecycle

```
MjSpec  ──spec.compile()──▶  MjModel  ──MjData(model)──▶  MjData
  │                             │                            │
  │  (mutable blueprint)        │  (compiled, mostly frozen) │ (simulation state)
  │                             │                            │
  └── spec.recompile(m, d) ─────┴────────────────────────────┘
```

1. **MjSpec** — mutable data structure you edit to define the simulation.
2. **`spec.compile()`** — produces `MjModel` + you create `MjData(model)`.
   After this, changing the spec has **no effect** until you recompile.
3. **Most `MjModel` fields are unsafe to mutate.** Changing them requires
   `spec.recompile(model, data)`, which returns **new** model and data objects
   (preserving physics state for existing elements).

```python
import mujoco

spec = mujoco.MjSpec()
body = spec.worldbody.add_body(pos=[0, 0, 1])
geom = body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.1])
body.add_freejoint()

model = spec.compile()
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# Later: add another body, recompile keeping state
body2 = spec.worldbody.add_body(pos=[1, 0, 1])
body2.add_geom(size=[0.1])
body2.add_freejoint()
model, data = spec.recompile(model, data)  # state preserved
```

> [!CAUTION]
> `recompile` returns **new** objects. Always reassign: `model, data = spec.recompile(model, data)`.

### Loading and Serializing

```python
spec = mujoco.MjSpec()                          # empty
spec = mujoco.MjSpec.from_string(xml_string)     # from XML string
spec = mujoco.MjSpec.from_file('/path/to.xml')   # from file
model = mujoco.MjModel.from_xml_string(xml)      # direct to model (no spec)

xml_out = spec.to_xml()                          # serialize back
```

### Compile Error Debugging

Use the `.info` field on spec elements for traceability:

```python
geom = spec.worldbody.add_geom()
geom.info = 'created at my_file.py:42'
spec.compile()  # Error: "size 0 must be positive in geom\nElement name '', id 0, created at my_file.py:42"
```

---

## Compute Backends

MuJoCo has two compute backends. **Choose early** — the backend determines
which features, solvers, and APIs are available.

| | C++ (default) | MJWarp (NVIDIA GPU) |
|---|---|---|
| **Import** | `import mujoco` | `import mujoco_warp as mjw` |
| **Optimized for** | Latency (single scene) | Throughput (big batches) |
| **Hardware** | CPU | NVIDIA GPU |
| **Solvers** | All (Newton, CG, PGS, **noslip**) | All except PGS, **noslip**, islands |
| **Plugins** | ✅ All | SDF only |
| **Precision** | float64 | float32 |
| **Named access / bind** | ✅ | Via wrapper libraries or MJX `bind()` |
| **Contact sensors** | ✅ | ✅ |
| **Sparse Jacobians** | ✅ | ❌ (dense only) |
| **Batch rendering** | ❌ | ✅ (BVH ray tracing) |

### When to use which

- **C++ (default)**: Real-time control, model predictive control, interactive
  visualization, any workflow needing full feature support (noslip solver,
  PGS, islands, plugins, ellipsoidal fluid model, sparse Jacobians). Also the
  only backend with native `bind()`. Use this unless you need massive
  parallelism. (For MJWarp, named access is available via wrapper
  libraries or MJX `bind()`.)

- **MJWarp**: Reinforcement learning with large batch sizes on NVIDIA GPUs.
  Scales better for contact-rich scenes and large meshes than the legacy
  MJX-JAX backend. Not differentiable. May degrade for scenes beyond ~60 DoFs.

> [!IMPORTANT]
> The `noslip` solver (post-constraint velocity correction for exact zero
> slip at contacts) is **only available in the C++ backend**. If your task
> requires accurate friction modeling without any tangential sliding at
> contacts, you must use C++.

> [!WARNING]
> MJWarp uses **float32**, which can cause numerical differences vs C++
> (float64). Solver convergence, small friction values, and long rollouts
> may be sensitive to this. If you see NaNs or instability on GPU, try
> increasing solver iterations or simplifying the model.

---

## Building Models with MjSpec

### Adding elements

Most `add_*` methods accept keyword arguments matching MJCF XML attributes:

```python
spec = mujoco.MjSpec()

body = spec.worldbody.add_body(name='arm', pos=[0, 0, 1], quat=[1, 0, 0, 0])
geom = body.add_geom(
    name='arm_geom',
    type=mujoco.mjtGeom.mjGEOM_CAPSULE,
    size=[0.05, 0.3],
    rgba=[1, 0, 0, 1],
)
joint = body.add_joint(
    name='hinge1',
    type=mujoco.mjtJoint.mjJNT_HINGE,
    axis=[0, 1, 0],
    range=[-1.57, 1.57],
)
site = body.add_site(name='sensor_site', pos=[0, 0, 0.3])
cam = body.add_camera(name='arm_cam', pos=[0, -2, 0], xyaxes=[1,0,0, 0,0,1])
```

### Orientation alternatives

In addition to `quat`, you can specify orientation with `euler`, `axisangle`,
`xyaxes`, or `zaxis`. Only one can be set at a time:

```python
body.add_geom(euler=[0, 90, 0])          # Euler angles (degrees by default)
body.add_geom(axisangle=[0, 1, 0, 1.57]) # axis + angle
body.add_geom(zaxis=[0, 1, 0])           # minimal rotation to align Z
body.add_geom(xyaxes=[1,0,0, 0,0,1])     # explicit X and Y axes
```

### Top-level elements

Sensors, actuators, tendons, materials, textures, and meshes are added directly
to the spec (not to bodies):

```python
spec.add_material(name='red', rgba=[1, 0, 0, 1])
spec.add_actuator(name='motor', joint=joint.name, gear=[1, 0, 0, 0, 0, 0])
spec.add_sensor(
    name='joint_pos',
    type=mujoco.mjtSensor.mjSENS_JOINTPOS,
    objtype=mujoco.mjtObj.mjOBJ_JOINT,
    objname=joint.name,
)
```

> [!IMPORTANT]
> Always use `element.name` (e.g., `joint.name`, `geom.name`, `site.name`)
> instead of hardcoded strings when referencing spec elements. This keeps
> references correct if the element is renamed or attached with a prefix.

### Geom size semantics

| Type      | Size params                                     |
| --------- | ----------------------------------------------- |
| sphere    | `[radius]`                                      |
| capsule   | `[radius, half_length]` or `[radius]` + fromto  |
| cylinder  | `[radius, half_length]` or `[radius]` + fromto  |
| box       | `[half_x, half_y, half_z]`                      |
| ellipsoid | `[radius_x, radius_y, radius_z]`                |
| plane     | `[half_x, half_y, grid_spacing]`                |

> [!WARNING]
> Capsule/cylinder `size` changes meaning with `fromto`. Without `fromto`,
> `size=[radius, half_length]`. With `fromto`, `size=[radius]` only — the
> length is computed from the two endpoints.

---

## Accessing Compiled Data: Named Access vs Bind

There are **two** recommended ways to read/write compiled model and data fields.
**Prefer `bind`** when working with spec elements; use **named access** otherwise.

### 1. Named Access (on MjModel / MjData)

```python
model.geom('my_geom').size          # → numpy view of geom_size for 'my_geom'
data.body('torso').xpos             # → numpy view of body_xpos
data.joint('knee').qpos             # → shape depends on joint type
data.actuator('motor').ctrl = 1.0   # writable view
```

Aliases: `joint` / `jnt`, `camera` / `cam`, `tendon` / `ten`, `material` / `mat`,
`texture` / `tex`, `equality` / `eq`, `keyframe` / `key`.

> [!WARNING]
> Named access returns **views, not copies.** After `mj_step`, old references
> reflect new values. Use `.copy()` when logging:
> `positions.append(data.body('torso').xpos.copy())`

### 2. Bind (bridges MjSpec elements → MjModel / MjData)

`bind()` connects spec elements (or lists of them) to their compiled
counterparts. **Use `.set()` to write through bind:**

```python
geom = spec.worldbody.add_geom(name='ball', size=[0.1], type=mujoco.mjtGeom.mjGEOM_SPHERE)
joint = body.add_joint(name='j1', type=mujoco.mjtJoint.mjJNT_HINGE)
model = spec.compile()
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# Reading via bind
model.bind(geom).size                 # → array([0.1, 0., 0.])
data.bind(geom).xpos                  # → array([0., 0., 0.])

# Writing via bind — always use .set()
data.bind(joint).set('qpos', 1.5)     # sets the joint's qpos

# Bind a list of spec elements
joints = [spec.joint('j1'), spec.joint('j2')]
data.bind(joints).qpos                # → concatenated array
data.bind(joints).set('qpos', np.array([0.5, 1.0]))  # write to both
```

> [!CAUTION]
> The spec must match the compiled model. If you modify the spec after
> `compile()`, you must recompile before calling `bind()`, or you get:
> `ValueError: 'The mjSpec does not match mjModel. Please recompile the mjSpec.'`

---

## Attachments: Composing Specs

Attach child specs/bodies to parent specs via frames or sites:

```python
parent = mujoco.MjSpec()
child = mujoco.MjSpec()
child_body = child.worldbody.add_body(name='arm')
child_body.add_geom(name='arm_geom', size=[0.05, 0.3], type=mujoco.mjtGeom.mjGEOM_CAPSULE)
child_body.add_joint(name='arm_joint', type=mujoco.mjtJoint.mjJNT_HINGE)

frame = parent.worldbody.add_frame(pos=[0, 0, 1])
frame.attach_body(child_body, prefix='left_')
# 'arm' → 'left_arm', 'arm_geom' → 'left_arm_geom', 'arm_joint' → 'left_arm_joint'

# Or attach entire spec to a site
site = parent.worldbody.add_site(name='attach_point', pos=[0, 0, 2])
parent.attach(child, site=site, prefix='right_', suffix='_v2')
```

> [!IMPORTANT]
> **Cross-spec references require a shared parent.**
> If you need to create an element (e.g., an equality constraint) that
> references elements from *two different child specs*, you must first
> attach both children to the same parent, then add the cross-referencing
> element to the **parent** spec using the final prefixed/suffixed names:

```python
# Two robot arms, each defined as a separate spec
arm_spec = mujoco.MjSpec()
arm_body = arm_spec.worldbody.add_body(name='hand')
arm_body.add_geom(name='hand_geom', size=[0.05])
wrist_joint = arm_body.add_joint(name='wrist', type=mujoco.mjtJoint.mjJNT_HINGE)

# Attach both to the parent with different prefixes
parent = mujoco.MjSpec()
left_prefix, right_prefix = 'left_', 'right_'

frame_l = parent.worldbody.add_frame(pos=[-0.5, 0, 1])
frame_l.attach_body(arm_body, prefix=left_prefix)     # left_wrist, left_hand, ...

frame_r = parent.worldbody.add_frame(pos=[0.5, 0, 1])
frame_r.attach_body(arm_body, prefix=right_prefix)    # right_wrist, right_hand, ...

# NOW add a constraint linking both arms — look up the prefixed joints
# from the parent spec, don't hardcode the names
left_wrist = parent.joint(f'{left_prefix}{wrist_joint.name}')
right_wrist = parent.joint(f'{right_prefix}{wrist_joint.name}')
parent.add_equality(type=mujoco.mjtEq.mjEQ_JOINT,
                    name1=left_wrist.name, name2=right_wrist.name)
model = parent.compile()
```

### Attachment Transforms

When attaching to a site or frame, the child body's position is transformed
relative to the parent's attachment point. Attachment also handles unit
conversion (degrees vs radians) between parent and child specs automatically.

### Assets Get Renamed Too

Prefix/suffix changes apply to asset filenames:
```python
child.assets = {'mesh.obj': data}
parent.attach(child, prefix='robot_')
# Asset key becomes 'robot_mesh.obj' in parent
```

---

## Cameras

### Orientation

MuJoCo cameras look down the **negative Z axis**. The camera frame is:
- **-Z** → forward (viewing direction)
- **+X** → right
- **+Y** → up

To point a camera downward (looking at the ground), set its Z axis to `[0, 0, 1]`:

```python
body.add_camera(
    name='overhead',
    xyaxes=[1, 0, 0, 0, 1, 0],  # x=[1,0,0], y=[0,1,0] → z=[0,0,1] → looks DOWN (-z)
    pos=[0, 0, 5],
)
```

### Geom Group Visibility

Each camera/viewer has 6 geom groups (0–5). Default visibility:

| Group | Default Visible | Typical Use |
|-------|----------------|-------------|
| 0     | ✅ Yes          | Standard geoms (default group for new geoms) |
| 1     | ✅ Yes          | Secondary visual geoms |
| 2     | ✅ Yes          | Tertiary visual geoms |
| 3     | ❌ No           | Collision-only or debug geoms |
| 4     | ❌ No           | Hidden geoms |
| 5     | ❌ No           | Hidden geoms |

A newly created geom is in **group 0** by default. Toggle visibility at runtime
via `mjvOption.geomgroup[i]`. The same 3-on/3-off default applies to sites,
joints, tendons, actuators, flexes, and skins.

---

## Contacts: Use Sensors, Not the Contact Array

### The problem with `data.contact`

`data.contact` is a **variable-length** array that changes size every timestep
depending on what's colliding. Iterating over it directly is fragile and
**incompatible with learning-based agents** and fixed-size observation spaces.

```python
# ❌ WRONG — don't iterate data.contact for reward/observation logic
for c in data.contact:
    if c.geom1 == target_geom_id:
        force = ...  # fragile, variable-length, non-deterministic order
```

> [!CAUTION]
> Never iterate `data.contact` to build observations or compute rewards.
> The array's length and ordering can change between timesteps and even
> between MuJoCo versions. Use **contact sensors** instead.

### Contact sensors: fixed-size, declarative contact queries

A `<contact>` sensor selects contacts via declarative matching criteria, reduces
them to a fixed number of slots, and extracts requested data fields into
`data.sensordata` — always the same size, every timestep.

The pipeline has three stages:
1. **Matching** — filter contacts by geom, body, subtree, or site volume
2. **Reduction** — keep the top `num` contacts (by order, min distance, max force, or net force)
3. **Extraction** — copy requested fields (`found`, `force`, `torque`, `dist`, `pos`, `normal`, `tangent`)

### Example: detect contact force between a gripper and an object

```python
import mujoco
import numpy as np

spec = mujoco.MjSpec()

# Build a simple scene: floor + falling object
floor = spec.worldbody.add_geom(
    name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.01]
)
obj_body = spec.worldbody.add_body(name='obj', pos=[0, 0, 0.5])
obj_body.add_freejoint()
obj_geom = obj_body.add_geom(
    name='obj_geom', type=mujoco.mjtGeom.mjGEOM_SPHERE,
    size=[0.05], mass=0.1,
)

# Add a contact sensor: report force for contacts involving obj_geom
contact_sensor = spec.add_sensor(
    name='obj_contact',
    type=mujoco.mjtSensor.mjSENS_CONTACT,
    # Match any contact involving this geom — use .name, not a literal string:
    objname=obj_geom.name, objtype=mujoco.mjtObj.mjOBJ_GEOM,
)

model = spec.compile()
data = mujoco.MjData(model)

# Step the simulation until the object lands
mujoco.mj_step(model, data, nstep=500)
mujoco.mj_forward(model, data)

# Read the contact sensor via bind — always fixed-size in data.sensordata
contact_data = data.bind(contact_sensor).sensordata
print(f'Contact sensor output: {contact_data}')
```

### XML-based contact sensor (common pattern)

When loading from XML, contact sensors are even cleaner:

```xml
<sensor>
  <!-- Is the gripper touching the object? Report force and normal for up to 3 contacts -->
  <contact name="grip_contact"
           body1="gripper" body2="object"
           num="3" data="found force normal"
           reduce="maxforce"/>

  <!-- Total wrench from all contacts on a body -->
  <contact name="object_net"
           body1="object"
           data="force torque"
           reduce="netforce"/>
</sensor>
```

The output size is deterministic: `num × size(data fields)`. For `"found force
normal"` with `num=3`, you get 3 × (1+3+3) = 21 numbers every timestep, padded
with zeros if fewer contacts match.

### Touch sensor: simpler alternative for scalar normal force

If you only need a scalar "how hard is something pressing on this site", use a
`touch` sensor instead:

```python
site = body.add_site(name='fingertip', pos=[0, 0, 0.05], size=[0.02])
spec.add_sensor(
    name='fingertip_touch',
    type=mujoco.mjtSensor.mjSENS_TOUCH,
    objname=site.name, objtype=mujoco.mjtObj.mjOBJ_SITE,
)
```

The touch sensor sums normal contact forces within the site volume — one scalar
output, always present in `sensordata`.

---

## Spatial Math Utilities (mju_)

MuJoCo ships a library of spatial computation functions under the `mju_`
namespace — quaternion algebra, rotation conversions, pose composition, and
coordinate transforms. **Always check for an existing `mju_` function before
implementing spatial math from scratch.** For basic vector arithmetic (add,
subtract, dot product, norm), just use NumPy/JAX/Torch directly.

### Quaternion Operations

```python
res = np.zeros(3)
mujoco.mju_rotVecQuat(res, vec, quat)        # rotate vector by quaternion

quat = np.zeros(4)
mujoco.mju_mat2Quat(quat, mat3x3)            # 3x3 rotation matrix → quaternion
mujoco.mju_quat2Mat(mat, quat)               # quaternion → 3x3 matrix
mujoco.mju_axisAngle2Quat(quat, axis, angle) # axis-angle → quaternion
mujoco.mju_euler2Quat(quat, euler, 'xyz')    # Euler angles → quaternion
mujoco.mju_mulQuat(res, q1, q2)              # multiply quaternions
mujoco.mju_negQuat(res, quat)                # conjugate
mujoco.mju_quatZ2Vec(quat, vec)              # quat that rotates z-axis to vec
mujoco.mju_quatIntegrate(quat, vel, scale)   # integrate quat with angular velocity
```

> [!TIP]
> `mju_quatZ2Vec` is particularly useful: given a target direction vector, it
> returns the quaternion that rotates the Z-axis to point in that direction.

### Pose Operations

```python
mujoco.mju_mulPose(pos_res, quat_res, pos1, quat1, pos2, quat2)  # compose poses
mujoco.mju_negPose(pos_res, quat_res, pos, quat)                  # invert pose
mujoco.mju_trnVecPose(res, pos, quat, vec)                        # transform vector by pose
```

---

## Common Gotchas

### 1. Computed fields are read-only

`data.xpos`, `data.xmat`, `data.xquat`, `data.geom_xpos` are **output** fields
computed by `mj_forward()`. You cannot assign to them directly. Instead, modify
input fields (`data.qpos`, `data.qvel`, `data.ctrl`) and call `mj_forward()` or
`mj_step()`.

### 2. Duplicate names are forbidden

```python
spec.add_material(name='yellow')
spec.add_material(name='yellow')  # ValueError: "repeated name 'yellow' in material"
```

Names must be unique within each element type.

### 3. Orientation keywords are mutually exclusive

```python
body.add_geom(axisangle=[1, 0, 0, 1.57], euler=[0, 0, 0])
# ValueError: 'Only one of: axisangle, xyaxes, zaxis, or euler can be set.'
```

Pick one orientation representation. Quaternion (`quat`) is the native format.

### 4. `size` must be positive for geoms

A geom with `size[0] == 0` will fail compilation. Always set at least
`size=[radius]` for spheres/capsules, or `size=[hx, hy, hz]` for boxes.

### 5. `mj_step` with `nstep` repeats the same control

```python
mujoco.mj_step(model, data, nstep=100)  # 100 steps, same ctrl each step
```

This is much faster than a Python loop and is fine for passive simulation or
constant-control scenarios. But if you need to update `data.ctrl` between steps,
you must step one at a time.

### 6. Euler sequence matters

`mju_euler2Quat` takes a 3-character sequence string. Lowercase = intrinsic
rotations, uppercase = extrinsic:

```python
mujoco.mju_euler2Quat(quat, [roll, pitch, yaw], 'xyz')  # intrinsic x-y-z
mujoco.mju_euler2Quat(quat, [roll, pitch, yaw], 'XYZ')  # extrinsic X-Y-Z
```

The sequence must be exactly 3 characters from `xyzXYZ`.

### 7. `copy()` vs view semantics

NumPy arrays from MjModel/MjData are **views** into C memory. `mj_step` changes
them in-place. Always `.copy()` when storing values for later comparison.

### 8. Default class handling

```python
main = spec.default               # global default class (always named 'main')
child_class = spec.add_default('high_friction', main)
child_class.geom.friction = [1.5, 0.005, 0.0001]

geom = body.add_geom(child_class) # use specific default class
geom = body.add_geom()            # uses 'main' class implicitly
```

### 9. Gravity is -Z by default

MuJoCo convention: **+Z is up**, gravity is `[0, 0, -9.81]`. The viewer and
all built-in models assume this. Don't fight it — orient your scene accordingly.

### 10. Capsule/cylinder size with and without fromto

```python
# With explicit pos/quat: size = [radius, half_length]
body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.05, 0.3])

# With fromto: size = [radius] only — length is inferred from endpoints
body.add_geom(
    type=mujoco.mjtGeom.mjGEOM_CAPSULE,
    size=[0.05],
    fromto=[0, 0, 0, 0, 0, 0.6],
)
```

### 11. Collision filtering with contype/conaffinity

Two geoms collide only if `(g1.contype & g2.conaffinity) || (g2.contype & g1.conaffinity)`.
By default both are `1`, so everything collides with everything.

```python
# Visual-only geom: set contype=0, conaffinity=0 to disable collisions
body.add_geom(size=[0.1], contype=0, conaffinity=0, group=1)

# Separate collision groups using bitmasks:
robot_geom = body.add_geom(size=[0.05], contype=1, conaffinity=2)
tool_geom = body.add_geom(size=[0.03], contype=2, conaffinity=1)
# Robot and tool collide (1&1=0, but 2&2=0… wait):
# contype=1 & conaffinity=1 → collide; contype=2 & conaffinity=2 → collide
```

> [!TIP]
> **`condim` and `friction` interact.** Each geom has `friction=[tangential, torsional, rolling]`
> (default `[1, 0.005, 0.0001]`). The `condim` value controls which friction coefficients are
> *active* in a contact:
>
> | condim | Active friction | Geom `friction` indices used |
> |--------|----------------|------------------------------|
> | 1 | None (frictionless, normal force only) | — |
> | 3 | Tangential (opposes sliding) | `friction[0]` |
> | 4 | Tangential + torsional (opposes sliding and twisting around contact normal) | `friction[0:2]` |
> | 6 | Tangential + torsional + rolling (also opposes rolling around tangent axes) | `friction[0:3]` |
>
> Torsional friction models a surface contact patch resisting twist — useful for soft fingers.
> Rolling friction dissipates energy from local deformations — useful for stopping balls from rolling
> forever. Both torsional and rolling coefficients have **units of length** (roughly the contact
> patch diameter or deformation depth).
>
> ```python
> # A soft finger pad: enable torsional friction for stable grasping
> finger_geom = body.add_geom(
>     type=mujoco.mjtGeom.mjGEOM_CAPSULE,
>     size=[0.01, 0.02],
>     condim=4,
>     friction=[1.0, 0.01, 0.0001],  # tangential=1.0, torsional=0.01
> )
>
> # A ball that should stop rolling on a surface
> ball_geom = body.add_geom(
>     type=mujoco.mjtGeom.mjGEOM_SPHERE,
>     size=[0.05],
>     condim=6,
>     friction=[0.8, 0.005, 0.002],  # tangential=0.8, torsional=0.005, rolling=0.002
> )
> ```

---

## Offscreen Rendering

Offscreen rendering produces images (RGB, depth, segmentation) without a
display. It requires an OpenGL context — MuJoCo auto-detects the best
available backend (EGL on headless Linux, GLFW on desktop, OSMesa as
fallback).

### The `Renderer` class

`mujoco.Renderer` wraps GL context creation, scene management, and buffer
readback. **Always use it as a context manager** to ensure GPU resources are
freed:

```python
import mujoco
import numpy as np

# Define a camera in the spec and keep a reference
overhead_cam = spec.worldbody.add_camera(
    name='overhead',
    pos=[0, 0, 3],
    quat=[0.707, 0.707, 0, 0],  # looking down
    fovy=60,
)

model = spec.compile()
data = mujoco.MjData(model)

# Create renderer — width/height must not exceed offscreen buffer (see below)
with mujoco.Renderer(model, height=480, width=640) as renderer:
    mujoco.mj_forward(model, data)

    # Use the spec element's .name — never a literal string
    renderer.update_scene(data, camera=overhead_cam.name)
    rgb = renderer.render()  # → np.ndarray (H, W, 3), dtype=uint8

    # Depth rendering
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=overhead_cam.name)
    depth = renderer.render()  # → np.ndarray (H, W), dtype=float32 (meters)
    renderer.disable_depth_rendering()

    # Segmentation rendering
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=overhead_cam.name)
    seg = renderer.render()  # → np.ndarray (H, W, 2), dtype=int32
    # seg[:,:,0] = object ID, seg[:,:,1] = object type; background = (-1, -1)
    renderer.disable_segmentation_rendering()
```

> [!WARNING]
> Forgetting to close the renderer (or not using `with`) leaks GPU memory and
> GL contexts. In loops, create the renderer **once** outside the loop.

### What the `Renderer` holds internally

When you create `mujoco.Renderer(model, height, width)`, it allocates three
internal objects that must be freed together:

1. **`GLContext`** — an offscreen OpenGL context (EGL, GLFW, or OSMesa,
   auto-detected). Created with the requested `width × height`.
2. **`MjrContext`** — MuJoCo's GPU rendering resources (shaders, textures,
   framebuffers), bound to the GLContext. Set to the offscreen framebuffer.
3. **`MjvScene`** — geometry buffer holding the scene snapshot passed to the
   GPU each frame.

The context manager (`with Renderer(...) as r:`) calls `r.close()` on exit,
which frees the MjrContext first and then the GLContext — **order matters**.
If you use the renderer without `with`, call `renderer.close()` manually.

> [!CAUTION]
> Internally, `MjrContext.free()` must be called **before** `GLContext.free()`.
> Reversing the order leaks GPU resources or segfaults. The `Renderer` class
> handles this automatically — prefer it over manual context management.

### Offscreen framebuffer size

The renderer cannot exceed the offscreen buffer dimensions. The defaults are
640×480. Set larger buffers **before compilation** via `spec.visual`:

```python
spec.visual.global_.offwidth = 1920
spec.visual.global_.offheight = 1080
model = spec.compile()

# Now you can render up to 1920×1080
with mujoco.Renderer(model, height=1080, width=1920) as renderer:
    ...
```

> [!IMPORTANT]
> Increasing offscreen buffer size consumes GPU memory. For batch rendering
> of many cameras, keep the per-frame resolution modest.

### Cameras

MuJoCo has two camera systems: **fixed cameras** defined in the model, and
the **free camera** for interactive viewing.

#### Defining cameras in MjSpec

Always store the return value of `add_camera` and use its `.name` or `.id`
to reference the camera later — never hardcode literal strings:

```python
# Fixed camera on worldbody — good for evaluation/recording
overhead_cam = spec.worldbody.add_camera(
    name='overhead',
    pos=[0, 0, 3],
    quat=[0.707, 0.707, 0, 0],
    fovy=60,
)

# Camera attached to a body — moves with the body
wrist_cam = wrist_body.add_camera(
    name='wrist_cam',
    pos=[0.05, 0, 0],
    xyaxes=[0, -1, 0, 0, 0, -1],
    fovy=90,
)
```

#### Selecting a camera for rendering

`update_scene` accepts a camera **name** (str), **id** (int), or an
`MjvCamera` object. Always derive from the spec element:

```python
# By name via spec element (recommended — survives recompilation)
renderer.update_scene(data, camera=overhead_cam.name)

# By id via spec element (after compile; matches model.cam_* arrays)
renderer.update_scene(data, camera=overhead_cam.id)

# Free camera (default) — no camera argument needed
renderer.update_scene(data)

# Custom free camera with explicit lookat/distance/angles
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [0, 0, 0.5]
cam.distance = 3.0
cam.azimuth = 135
cam.elevation = -25
renderer.update_scene(data, camera=cam)
```

#### Camera properties reference

| Property | Type | Description |
|----------|------|-------------|
| `pos` | `real(3)` | Position in parent body frame |
| `quat` | `real(4)` | Orientation quaternion (w, x, y, z) |
| `xyaxes` | `real(6)` | Alternative orientation: `[x_axis(3), y_axis(3)]` |
| `fovy` | `real` | Vertical field of view (degrees, default 45) |
| `resolution` | `int(2)` | Sensor resolution — only for camera-based sensors |
| `targetbody` | `str` | Track this body (camera always looks at it) |
| `mode` | `str` | `"fixed"`, `"track"`, `"trackcom"`, `"targetbody"`, `"targetbodycom"` |

### Scene options

Control what is visualized via `MjvOption`:

```python
scene_option = mujoco.MjvOption()
# geomgroup is a bool array indexed by group number (0–5).
# Each geom's `group` attribute (default 0) assigns it to a group.
# Toggle visibility of each group:
scene_option.geomgroup[:] = False   # hide all groups
scene_option.geomgroup[0] = True    # show group 0 (e.g. ground plane)
scene_option.geomgroup[3] = True    # show group 3 (e.g. visualization geoms)

# Toggle rendering flags
scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

renderer.update_scene(data, camera=overhead_cam.name, scene_option=scene_option)
```

### Filament backend (experimental)

MuJoCo's default renderer uses OpenGL. An alternative **Filament** backend
(Vulkan-based) is available experimentally and provides higher-quality
rendering. Filament does **not** require vertical flip (`np.flipud` is a
no-op). It is selected via build flags — see the MuJoCo Filament
[source](../src/experimental/filament) for details.

---

## Key References

### Documentation

| Document | Description |
|----------|-------------|
| [XMLreference.rst](XMLreference.rst) | Complete MJCF XML element and attribute reference |
| [python.rst](python.rst) | Python bindings API: named access, bind, enums, callbacks |
| [modeling.rst](modeling.rst) | MJCF modeling guide: coordinate frames, defaults, attachments |
| [simulation.rst](programming/simulation.rst) | Simulation loop, state, forward/inverse dynamics |
| [modeledit.rst](programming/modeledit.rst) | Procedural model editing with MjSpec |
| [visualization.rst](programming/visualization.rst) | Rendering, cameras, scene management |
| [APIfunctions.rst](APIreference/APIfunctions.rst) | C API function reference (mj_, mju_, mjv_, mjr_) |
| [APItypes.rst](APIreference/APItypes.rst) | All MuJoCo structs and enums |

### Test Files (Executable Examples)

| Test file | Key patterns demonstrated |
|-----------|--------------------------|
| [specs_test.py](../../py/mujoco/specs_test.py) | MjSpec API: compile, recompile, attach, bind, defaults, delete, actuator shortcuts |
| [bindings_test.py](../../py/mujoco/bindings_test.py) | Named indexing, mju_ functions, copy/pickle, contacts, mj_step |
| [support_test.py](../../py/mujoco/mjx/_src/support_test.py) | MJX bind `.set()` pattern, JAX functional updates |

### Source Code

| File | Description |
|------|-------------|
| [mujoco.h](../include/mujoco.h) | Main C API header with all mju_ function signatures |
| [XMLschema.rst](XMLschema.rst) | Schema-level XML structure documentation |
