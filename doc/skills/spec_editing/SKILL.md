---
name: mujoco-spec-editing
description: >-
  Procedural model authoring and editing with MjSpec (Python and C++), bodies,
  geoms, joints, sites, cameras, sub-model attachment with prefix namespaces,
  actuators (position servos, motors, velocity drives, trntype), collision bitmasks
  (contype, conaffinity), friction (condim), default classes (MjsDefault),
  procedural meshes, deletion, spec recompilation (spec.recompile).
  Use for procedural scenes, adding bodies/geoms/actuators, attaching sub-models.
  Do NOT use for stepping simulations (mujoco-python) or rendering (mujoco-rendering).
---

# MuJoCo Spec Editing: Building & Modifying Models

> [!TIP]
>
> **Related skills:**
>
> -   [mujoco-python](../python/SKILL.md) — Simulation lifecycle, stepping, and
>     diagnostics.
> -   [mujoco-rendering](../rendering/SKILL.md) — Offscreen image rendering and
>     camera setup.
> -   [mujoco-studio](../studio/SKILL.md) — Interactive viewers and Studio
>     plugins.

## 1. Procedural Modeling with MjSpec

`MjSpec` is a mutable C++ and Python data structure that mirrors the MJCF XML
schema, allowing models to be constructed and modified programmatically.

```python
import mujoco

spec = mujoco.MjSpec()

# Add a body with a freejoint and sphere geom
body = spec.worldbody.add_body(name='ball', pos=[0, 0, 1.0])
body.add_freejoint()
body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.1], rgba=[1, 0, 0, 1])

# Compile to MjModel
model = spec.compile()
```

In C++:
```cpp
#include <mujoco/mujoco.h>

mjSpec* spec = mj_makeSpec();
mjbBody* body = mj_addBody(spec, nullptr);
body->pos[2] = 1.0;
mj_addGeom(body, mjGEOM_SPHERE, 0.1, 0, 0);

mjModel* model = mj_compile(spec, nullptr, 0);
mj_deleteSpec(spec);
```

## 2. Building Hierarchies: Bodies, Geoms, Joints & Sites

### Adding Elements

```python
body = spec.worldbody.add_body(name='arm', pos=[0, 0, 0.5])

# Geom with capsule shape
geom = body.add_geom(
    name='arm_geom',
    type=mujoco.mjtGeom.mjGEOM_CAPSULE,
    size=[0.05, 0.25],
    rgba=[0.2, 0.6, 1.0, 1.0],
)

# Hinge joint
joint = body.add_joint(
    name='shoulder',
    type=mujoco.mjtJoint.mjJNT_HINGE,
    axis=[0, 1, 0],
    range=[-1.57, 1.57],
)

# Sensor site and camera
site = body.add_site(name='grip_site', pos=[0, 0, 0.25])
cam = body.add_camera(name='tool_cam', pos=[0, -0.5, 0.25], xyaxes=[1, 0, 0, 0, 0, 1])
```

### Geom Dimensions Reference

| Geom Type | `size` Parameter Semantics |
| :--- | :--- |
| `mjGEOM_SPHERE` | `[radius]` |
| `mjGEOM_CAPSULE` | `[radius, half_length]` (or `[radius]` if using `fromto`) |
| `mjGEOM_CYLINDER` | `[radius, half_length]` (or `[radius]` if using `fromto`) |
| `mjGEOM_BOX` | `[half_x, half_y, half_z]` |
| `mjGEOM_ELLIPSOID`| `[radius_x, radius_y, radius_z]` |
| `mjGEOM_PLANE` | `[half_x, half_y, grid_spacing]` |

> [!WARNING]
>
> When `fromto=[x1, y1, z1, x2, y2, z2]` is specified for capsules or cylinders,
> `size` takes only **one value** (`[radius]`); length is calculated from endpoints.

### Orientation Keywords

Only **one** orientation specification can be used at a time:
-   `quat=[w, x, y, z]` (native representation)
-   `euler=[roll, pitch, yaw]` (degrees by default)
-   `axisangle=[x, y, z, angle_rad]`
-   `xyaxes=[x_x, x_y, x_z, y_x, y_y, y_z]`
-   `zaxis=[x, y, z]`

## 3. Sub-model Composition & Attachments

Compose models modularly by attaching sub-specs with prefix namespaces:

```python
parent = mujoco.MjSpec()
child = mujoco.MjSpec()

child_body = child.worldbody.add_body(name='gripper')
child_body.add_geom(size=[0.05], type=mujoco.mjtGeom.mjGEOM_SPHERE)

# Attach child spec to a mount site on the parent
mount_site = parent.worldbody.add_site(name='mount', pos=[0, 0, 1.0])
parent.attach(child, site=mount_site, prefix='gripper_')
# Child body is renamed to 'gripper_gripper' in parent
```

### Cross-Spec Equality Constraints

When linking bodies from two different child specs (e.g. dual-arm setups),
attach both children to the common parent first, then declare constraints on the
parent spec:

```python
parent.attach(arm1_spec, prefix='left_')
parent.attach(arm2_spec, prefix='right_')

parent.add_equality(
    type=mujoco.mjtEq.mjEQ_CONNECT,
    name1='left_hand',
    name2='right_hand',
    anchor=[0, 0, 0],
)
```

## 4. Deleting & Dynamically Modifying Elements

### Deleting Elements

```python
spec.delete_body('unwanted_body')
spec.delete_geom('old_geom')
spec.delete_joint('redundant_joint')
spec.delete_actuator('motor')
```

### State-Preserving Recompilation

When changing model architecture at runtime, use `spec.recompile` to keep
existing physics state:

```python
# Add new tool to robot
spec.worldbody.add_geom(name='tool_tip', size=[0.02], type=mujoco.mjtGeom.mjGEOM_SPHERE)

# Recompile keeping positions/velocities intact
model, data = spec.recompile(model, data)
```

## 5. Actuators & Servos

Always explicitly set the transmission type (`trntype`):

```python
# 1. Torque Motor
motor = spec.add_actuator(
    name='torque_actuator',
    target='shoulder',
    trntype=mujoco.mjtTrn.mjTRN_JOINT,
)
motor.set_to_motor()

# 2. Stiff Position Servo (Recommended for position control)
servo = spec.add_actuator(
    name='pos_servo',
    target='shoulder',
    trntype=mujoco.mjtTrn.mjTRN_JOINT,
)
servo.set_to_position(kp=200.0)
servo.forcelimited = True
servo.forcerange = [-50.0, 50.0]     # Peak torque in Nm
servo.ctrllimited = True
servo.ctrlrange = [-1.57, 1.57]      # Target angle in radians

# 3. Velocity Actuator
vel = spec.add_actuator(
    name='vel_driver',
    target='wheel_joint',
    trntype=mujoco.mjtTrn.mjTRN_JOINT,
)
vel.set_to_velocity(kv=10.0)
```

> [!CAUTION]
>
> The helper functions `set_to_position()` and `set_to_motor()` configure gains
> but do **not** set the transmission. You must always specify
> `trntype=mujoco.mjtTrn.mjTRN_JOINT` (or another valid transmission type).

## 6. Default Classes (Inheritance & Styling)

Default classes (`MjsDefault`) provide cascaded styling and properties:

```python
# Create child class under root 'main' class
main_class = spec.default
rubber = spec.add_default('rubber', main_class)
rubber.geom.friction = [1.5, 0.01, 0.001]
rubber.geom.density = 1200.0

# Assign class at creation
body.add_geom(rubber, name='tire', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.1, 0.05])

# Assign class post-creation (must pass MjsDefault object, NOT a string)
other_geom.classname = rubber
```

## 7. Collisions & Friction Tuning

### Collision Filtering Bitmasks

Two geoms collide if and only if:
`(g1.contype & g2.conaffinity) || (g2.contype & g1.conaffinity)`

```python
# Visual-only geom: never collides
body.add_geom(size=[0.1], contype=0, conaffinity=0, group=1)

# Team A collides only with Team B
geom_a = body.add_geom(size=[0.1], contype=1, conaffinity=2)
geom_b = body.add_geom(size=[0.1], contype=2, conaffinity=1)
```

### Contact Dimension (`condim`) & Friction

| `condim` | Active Friction Components | Friction Index Used |
| :--- | :--- | :--- |
| 1 | Normal force only (frictionless) | — |
| 3 | Tangential sliding friction | `friction[0]` |
| 4 | Tangential + torsional (twisting) | `friction[0:2]` |
| 6 | Tangential + torsional + rolling | `friction[0:3]` |

```python
# High-friction contact for grasping
finger.add_geom(
    type=mujoco.mjtGeom.mjGEOM_CAPSULE,
    size=[0.01, 0.03],
    condim=4,
    friction=[1.2, 0.02, 0.0001],
)
```

## 8. Procedural Meshes, Materials & Sensors

### Procedural Meshes

```python
vertices = np.array([[-0.5, -0.5, 0], [0.5, -0.5, 0], [0, 0.5, 0]], dtype=np.float32)
faces = np.array([[0, 1, 2]], dtype=np.int32)
mesh = spec.add_mesh(name='triangle', vertex=vertices, face=faces)
```

### Procedural Sensors

```python
# IMU Accelerometer and Gyroscope
spec.add_sensor(name='torso_acc', type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                objtype=mujoco.mjtObj.mjOBJ_SITE, objname='imu_site')
spec.add_sensor(name='torso_gyro', type=mujoco.mjtSensor.mjSENS_GYRO,
                objtype=mujoco.mjtObj.mjOBJ_SITE, objname='imu_site')
```

## 9. Common Gotchas & Best Practices

1.  **Non-zero geom sizes**: Geoms with radius or half-extents of zero cause
    immediate compilation errors.
2.  **`classname` takes an object**: `geom.classname = 'rubber'` raises a
    `TypeError`. Pass the `MjsDefault` instance: `geom.classname = rubber`.
3.  **Always assign `recompile()`**: `spec.recompile(m, d)` produces newly
    allocated model and data instances. Reassign them: `m, d = spec.recompile(m, d)`.

## 10. Key References

### Documentation

-   [MjSpec & Procedural Model Editing](../../programming/modeledit.rst)
-   [MJCF Modeling Guide](../../modeling.rst)
-   [XML Reference](../../XMLreference.rst)

### Source Code Examples

-   [MjSpec Tests](../../../python/mujoco/specs_test.py)
