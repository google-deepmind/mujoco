---
name: mujoco-rendering
description: >-
  Offscreen image rendering, Python Renderer class, C++ pipeline (mjrContext,
  mjvScene, mjr_render, mjr_readPixels), RGB, metric depth maps, segmentation
  masks (mjtObj), offwidth/offheight framebuffer sizing, MUJOCO_GL (osmesa, egl),
  cameras (fixed, tracking, free, MjvCamera), scene options (geomgroup, flags).
  Use for headless camera rendering, dataset recording, visual observations.
  Do NOT use for interactive viewers/Studio (mujoco-studio) or ImGui controls (mujoco-gui).
---

# MuJoCo Rendering: Cameras, Contexts & Offscreen Buffers

> [!TIP]
>
> **Related skills:**
>
> -   [mujoco-python](../python/SKILL.md) — Core physics simulation, stepping,
>     and state management.
> -   [mujoco-studio](../studio/SKILL.md) — Interactive simulation, Studio
>     viewer apps, and plugin architecture.
> -   [mujoco-gui](../gui/SKILL.md) — Dear ImGui UI design and custom controls.

## 1. Overview: Headless vs Interactive Rendering

MuJoCo provides two distinct graphics workflows:

-   **Headless Offscreen Rendering (`mujoco.Renderer` / `mjrContext`)**:
    Focused on capturing image buffers (RGB, depth, object segmentation) without
    opening a desktop window. Ideal for batch dataset collection, reinforcement
    learning observations, and automated visual regression testing.
-   **Interactive Viewers (`mujoco.experimental.studio` / `mujoco.viewer`)**:
    Interactive desktop or web-streamed applications designed for real-time
    inspection, camera manipulation, and Dear ImGui control panels.

## 2. Headless Contexts on Linux: `MUJOCO_GL`

On headless Linux systems (servers, containers), configure the OpenGL context
provider via the `MUJOCO_GL` environment variable **before importing `mujoco`**:

```python
import os
os.environ['MUJOCO_GL'] = 'egl'      # Hardware-accelerated offscreen on NVIDIA GPUs
# os.environ['MUJOCO_GL'] = 'osmesa' # Portable CPU software rasterization

import mujoco  # Must import AFTER setting MUJOCO_GL
```

Or from the command line:
```bash
MUJOCO_GL=egl python3 my_script.py
```

## 3. The Python Renderer Class (RGB, Depth & Segmentation)

`mujoco.Renderer` manages GL context creation, scene geometry synchronization,
and buffer readback. **Always use it as a context manager**:

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path('scene.xml')
data = mujoco.MjData(model)

with mujoco.Renderer(model, height=480, width=640) as renderer:
  mujoco.mj_forward(model, data)

  # 1. RGB Rendering
  renderer.update_scene(data, camera='overhead_cam')
  rgb = renderer.render()  # Shape: (480, 640, 3), dtype: uint8

  # 2. Metric Depth Rendering (distance in meters)
  renderer.enable_depth_rendering()
  renderer.update_scene(data, camera='overhead_cam')
  depth = renderer.render()  # Shape: (480, 640), dtype: float32
  renderer.disable_depth_rendering()

  # 3. Semantic Segmentation Rendering
  renderer.enable_segmentation_rendering()
  renderer.update_scene(data, camera='overhead_cam')
  seg = renderer.render()  # Shape: (480, 640, 2), dtype: int32
  # seg[:, :, 0] = object ID (-1 for background)
  # seg[:, :, 1] = object type (mjtObj enum, e.g. mjOBJ_GEOM)
  renderer.disable_segmentation_rendering()
```

### Rendering Inside Simulation Loops

Creating a `Renderer` allocates GPU framebuffers. In RL or evaluation loops,
**create the renderer once outside the loop**:

```python
with mujoco.Renderer(model, height=84, width=84) as renderer:
  for step in range(1000):
    mujoco.mj_step(model, data)
    if step % render_interval == 0:
      renderer.update_scene(data, camera='agent_cam')
      # Call .copy() because the renderer's internal buffer is reused
      frame = renderer.render().copy()
```

## 4. C++ Offscreen Rendering Pipeline (mjr & mjv)

In C++, offscreen rendering is performed using the `mjvScene` and `mjrContext`
APIs:

```cpp
#include <mujoco/mujoco.h>

// 1. Initialize visualization structures
mjvCamera cam;
mjv_defaultCamera(&cam);
cam.type = mjCAMERA_FIXED;
cam.fixedcamid = mj_name2id(m, mjOBJ_CAMERA, "overhead_cam");

mjvOption opt;
mjv_defaultOption(&opt);

mjvScene scn;
mjv_makeScene(m, &scn, 2000);  // Buffer up to 2000 geoms

// 2. Initialize offscreen rendering context
mjrContext con;
mjr_defaultContext(&con);
mjr_makeContext(m, &con, mjFONTSCALE_100);

// 3. Render frame
mjrRect viewport = {0, 0, 640, 480};
mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
mjr_render(viewport, &scn, &con);

// 4. Read pixels into user buffers
unsigned char rgb[640 * 480 * 3];
float depth[640 * 480];
mjr_readPixels(rgb, depth, viewport, &con);

// 5. Clean up
mjr_freeContext(&con);
mjv_freeScene(&scn);
```

## 5. Camera Definition & Placement

MuJoCo cameras look down their **negative Z-axis**:
-   **-Z**: Viewing forward direction
-   **+X**: Right
-   **+Y**: Up

### Defining Cameras in MjSpec

```python
# Fixed camera pointing down at ground plane
cam = spec.worldbody.add_camera(
    name='overhead',
    pos=[0, 0, 3.0],
    quat=[0.707, 0.707, 0, 0],  # 90 deg rotation around X
    fovy=60,
)

# Camera attached to robot wrist
wrist_cam = wrist_body.add_camera(
    name='wrist_cam',
    pos=[0.05, 0, 0],
    xyaxes=[0, -1, 0, 0, 0, -1],
    fovy=75,
)
```

### Free Camera (`MjvCamera`)

For custom interactive or orbital viewpoints:

```python
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [0, 0, 0.5]
cam.distance = 2.5
cam.azimuth = 90.0
cam.elevation = -20.0

renderer.update_scene(data, camera=cam)
```

## 6. Framebuffers & Scene Options

### Offscreen Framebuffer Resolution

The offscreen rendering resolution cannot exceed the dimensions pre-allocated in
the model. Adjust the buffer limit in the spec before compilation:

```python
spec.visual.global_.offwidth = 1920
spec.visual.global_.offheight = 1080
model = spec.compile()
```

### The Single `<visual>` Block Rule

MuJoCo uses only the **last** `<visual>` block defined in an XML file. Multiple
visual blocks will silently overwrite earlier settings:

```xml
<!-- ❌ WRONG — second visual block silently resets offwidth/offheight to default -->
<visual><global offwidth="1920" offheight="1080"/></visual>
<visual><headlight ambient="0.3 0.3 0.3"/></visual>

<!-- ✅ RIGHT — consolidate into one visual block -->
<visual>
  <global offwidth="1920" offheight="1080"/>
  <headlight ambient="0.3 0.3 0.3"/>
</visual>
```

### Geom Group Visibility & Visualization Flags

Control rendering layers via `MjvOption`:

```python
opt = mujoco.MjvOption()
opt.geomgroup[:] = False   # Hide all groups
opt.geomgroup[0] = True    # Show primary geometry (default group)
opt.geomgroup[1] = True    # Show secondary visuals

# Toggle visual debugging flags
opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True

renderer.update_scene(data, camera='overhead', scene_option=opt)
```

## 7. Filament & Modern Graphics Backends

MuJoCo supports the Filament physically-based rendering (PBR) engine
experimentally for advanced lighting, soft shadows, and materials. In Python,
Filament renderer bindings are available under `mujoco.rendering.filament`.

## 8. Common Gotchas & Best Practices

1.  **GL context leaks**: Always close the renderer or use a `with` block. Failing
    to free renderers in loops exhausts GL contexts.
2.  **Destruction order**: In C++, `mjr_freeContext` must be called before the
    underlying OpenGL context is destroyed.
3.  **Buffer re-use**: `renderer.render()` returns an internal buffer view. Call
    `.copy()` if storing frames in a list.
4.  **Camera lookup by name**: Pass `camera='camera_name'` or the integer ID.
    Avoid passing raw string IDs that are not present in the model.

## 9. Key References

### Documentation

-   [Visualization & Rendering Reference](../../programming/visualization.rst)
-   [Python Bindings Reference](../../python.rst)

### Source Code Examples

-   [C API Header](../../../include/mujoco/mujoco.h)
