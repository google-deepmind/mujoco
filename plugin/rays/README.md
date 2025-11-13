# Rays Plugin

The Rays plugin provides raycast-based sensors for MuJoCo simulations.

## Sensors

### depth_capture

A depth capture sensor that performs raycasting from a site's frame to capture depth information in a grid pattern.

**Plugin name**: `mujoco.rays.depth_capture`

**Features**:
- Raycast-based depth sensing using MuJoCo's `mj_ray` function
- Configurable grid resolution (ncol × nrow)
- Adjustable field of view (horizontal and vertical)
- Maximum distance cutoff
- Can be attached to any site in the model

**Configuration Parameters**:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ncol` | int | 10 | Number of rays horizontally |
| `nrow` | int | 10 | Number of rays vertically |
| `fov_x` | float | 45.0 | Horizontal field of view in degrees (0, 180] |
| `fov_y` | float | 45.0 | Vertical field of view in degrees (0, 180] |
| `max_distance` | float | 10.0 | Maximum raycast distance |

**Coordinate System**:

The sensor uses the site's frame following MuJoCo's camera convention:
- **+X**: Right
- **+Y**: Up
- **-Z**: Forward (sensor pointing direction)

**Output Format**:

The sensor outputs `ncol × nrow` depth values in row-major order:
- Index: `i = row * ncol + col`
- `row`: vertical index (0 = bottom, nrow-1 = top)
- `col`: horizontal index (0 = left, ncol-1 = right)

Each depth value represents the distance to the nearest surface for that ray. If no object is detected within `max_distance`, the value is set to `max_distance`.

## Usage

### XML Configuration

```xml
<mujoco>
  <extension>
    <plugin plugin="mujoco.rays.depth_capture"/>
  </extension>

  <worldbody>
    <body name="robot">
      <!-- Create a site for the depth sensor -->
      <site name="depth_sensor" pos="0 0 0.1"/>
    </body>
  </worldbody>

  <sensor>
    <plugin name="depth" plugin="mujoco.rays.depth_capture"
            objtype="site" objname="depth_sensor">
      <config key="ncol" value="10"/>
      <config key="nrow" value="10"/>
      <config key="fov_x" value="60"/>
      <config key="fov_y" value="60"/>
      <config key="max_distance" value="10"/>
    </plugin>
  </sensor>
</mujoco>
```

### Reading Sensor Data in C/C++

```c
// Find the sensor by name
int sensor_id = mj_name2id(m, mjOBJ_SENSOR, "depth");

// Get sensor data address and dimension
int sensor_adr = m->sensor_adr[sensor_id];
int sensor_dim = m->sensor_dim[sensor_id];  // ncol * nrow

// Access the depth values
for (int i = 0; i < sensor_dim; i++) {
    mjtNum depth = d->sensordata[sensor_adr + i];
    // depth is the distance to the nearest surface for ray i
}
```

### Reading Sensor Data in Python

```python
import mujoco

# Load model
model = mujoco.MjModel.from_xml_path('your_model.xml')
data = mujoco.MjData(model)

# Find sensor
sensor_id = model.sensor('depth').id

# Get sensor address and dimension
sensor_adr = model.sensor_adr[sensor_id]
sensor_dim = model.sensor_dim[sensor_id]

# Simulate
mujoco.mj_step(model, data)

# Read depth data
depth_data = data.sensordata[sensor_adr:sensor_adr + sensor_dim]

# Reshape to 2D grid (nrow x ncol)
ncol = 10  # Your configured value
nrow = 10  # Your configured value
depth_grid = depth_data.reshape(nrow, ncol)

print(f"Depth at center: {depth_grid[nrow//2, ncol//2]}")
```

## Examples

See `/model/plugin/rays/depth-capture.xml` for a complete example.

## Building

The plugin is built as part of the MuJoCo plugin system:

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

The plugin will be compiled as `librays.so` (or `.dylib` on macOS, `.dll` on Windows).

## Implementation Details

- Ray directions are pre-computed during initialization for efficiency
- Rays are distributed uniformly in angle space across the field of view
- Each ray is normalized and transformed to world coordinates during computation
- The sensor uses `mjSTAGE_POS`, so depth is computed after position updates
