# MuJoCo WASM bindings

Official WebAssembly (WASM) bindings for the MuJoCo physics engine, compiled from the MuJoCo C/C++ sources into WASM with JS glue (Emscripten + Embind). The package ships prebuilt ESM-ready JS/WASM artifacts and TypeScript types for immediate use.

**Important**:_These bindings are still a WIP_.

## Install
```sh
npm install mujoco
```

## Quick start (ESM / TypeScript)

```ts
import loadMujoco from 'mujoco';

const mujoco = await loadMujoco();

const model = mujoco.MjModel.fromXMLString(`
<mujoco>
  <worldbody>
    <geom type="sphere" size="0.1"/>
  </worldbody>
</mujoco>
`);

const data = new mujoco.MjData(model);
mujoco.mj_step(model, data);
```

## Usage Guide
When interacting with MuJoCo objects through the WASM bindings, it's important to understand how data is accessed. Properties on objects like `MjModel` and `MjData` can expose data in two ways: by copy or by reference.

### Copy vs. Reference

#### 1. By Copy (Value-based access)

Some properties return a copy of the data at the time of access. This is common for complex data structures that need to be marshalled from C++ to JavaScript.

A key example is `MjData.contact`. When you access `data.contact`, you get a new array containing the contacts at that specific moment in the simulation. If you step the simulation forward, this array will not be updated. You must access `data.contact` again to get the new contact information.

Example:
```typescript
// Gets contacts at the current time.
const contacts = data.contact;

// Step the simulation
mujoco.mj_step(model, data);

// `contacts` is now stale. To get the new contacts, you must access the property again:
const newContacts = data.contact;
```

#### 2. By Reference (View-based access)
Many properties, especially large numerical arrays, return a live view directly into the WebAssembly memory. This is highly efficient as it avoids copying large amounts of data.

A key example is `MjData.qpos` (joint positions). When you get a reference to this array, it points directly to the simulation's state data. Any changes in the simulation (e.g., after a call to `mj_step`) will be immediately reflected in this array.

```typescript
// `qpos` is a live view into the simulation state.
const qpos = data.qpos;

console.log(qpos[0]); // Print initial position

// Step the simulation
mujoco.mj_step(model, data);

// `qpos` is automatically updated.
console.log(qpos[0]); // Print new position
```

### Data Layout: Row-Major Matrices
When a function from the MuJoCo C API returns a matrix (or needs a matrix as input), these are represented in the JavaScript bindings as flat, one-dimensional `TypedArray`'s. The elements are stored in row-major order.

For example, a 3x10 matrix will be returned as a flat array with 30 elements. The first 10 elements represent the first row, the next 10 represent the second row, and so on.

Example: Accessing an element at `(row, col)`
```typescript
// A 3x10 matrix stored as a flat array.
const matrix: Float64Array = ...;
const nRows = 3;
const nCols = 10;

// To access the element at row `i` and column `j`:
const element = matrix[i * nCols + j];
```

### Working with Out Parameters
Many functions in the MuJoCo C API use "out parameters" to return data. This means instead of returning a value, they write the result into one of the arguments passed to them by reference (using pointers). In our JavaScript bindings, you'll need to handle these cases specifically.

There are two main scenarios you'll encounter:

#### 1. Array-like Out Parameters
When a function expects a pointer to a primitive type (like `mjtNum*` or `int*`) to write an array of values, you need to pre-allocate memory for the result on the JavaScript side. We provide helper classes for this: `mujoco.Uint8Buffer`, `mujoco.DoubleBuffer`, `mujoco.FloatBuffer`, and `mujoco.IntBuffer`.

Here's how to use them:

1.  Create a buffer: Instantiate the appropriate buffer class with an initial array of the correct size (e.g., an array of zeros).
2.  Call the function: Pass the buffer instance to the function as the out parameter.
3.  Access the result: Use the `.getView()` method on the buffer to get a `TypedArray` view of the data written by the C++ function.
4.  Free the memory: When you are done with the buffer, you must call the `.delete()` method to free the underlying memory and prevent memory leaks.

Example: Rotating a vector

The function `mju_rotVecQuat` rotates a vector `vec` by a quaternion `quat` and stores the result in the `res` out parameter.

```typescript
// Create a buffer to hold the 3D vector result.
const res = new mujoco.DoubleBuffer([0, 0, 0]);

const vec = [1, 0, 0];
const quat = [0.707, 0, 0, 0.707]; // 90-degree rotation around z-axis

try {
  // Call the function with the buffer as the out parameter.
  mujoco.mju_rotVecQuat(res, vec, quat);

  // Get the result as a Float64Array.
  const resultView = res.getView();
  console.log(resultView); // Expected: approximately [0, 1, 0]

} finally {
  // IMPORTANT: Free the memory allocated for the buffer.
  res.delete();
}
```

#### 2. Struct Out Parameters (e.g., mjvCamera*, mjvScene*)
When a function modifies a struct passed by pointer, you should pass an instance of the corresponding JavaScript wrapper class. The underlying C++ struct will be modified in place.

Example: Updating a scene

The function `mjv_updateScene` populates an `mjvScene` object with information from `mjModel` and `mjData`.
```typescript
// Create instances of the necessary structs.
const model = mujoco.MjModel.loadFromXML(xmlContent);
const data = new mujoco.MjData(model);
const scene = new mujoco.MjvScene(model, 1000);
const option = new mujoco.MjvOption();
const perturb = new mujoco.MjvPerturb();
const camera = new mujoco.MjvCamera();

// ... (step simulation, etc.)

// Update the scene. The 'scene' object is modified by the function.
mujoco.mjv_updateScene(
    model,
    data,
    option,
    perturb,
    camera,
    mujoco.mjtCatBit.mjCAT_ALL.value,
    scene
);

console.log('Number of geoms in scene:', scene.ngeom);

// Remember to delete all created objects when they are no longer needed.
scene.delete();
camera.delete();
perturb.delete();
option.delete();
data.delete();
model.delete();
```

As with buffers, you are responsible for managing the memory of these struct instances and must call `.delete()` on them when you are finished.

### Enums
Access via `.value`:
```javascript
mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL.value
```

### Constants
Scalar constants will be accessed the same way they are on python, simply:

```javascript
mujoco.mjNEQDATA
```

Due to Embind limitations, more complex constants that are not scalar, but are represented in more dimensions are exposed as functions. E.g to use `mujoco.mjFRAMESTRING` you will need to call a function:

```javascript
mujoco.get_mjFRAMESTRING()
```

This will return a javascript array representation of the values in MuJoCo `mjFRAMESTRING`.

## Notes
The package is ESM (`type: module`) and ships TypeScript types.

Ensure your bundler or dev server serves the `.wasm` asset at runtime.

### Development
For detailed build instructions see the repository [README](https://github.com/google-deepmind/mujoco/tree/main/wasm#mujoco-javascript-bindings) (Emscripten toolchain, Embind bindings, producing the .wasm artifact, and targets for Node and browser).

## Versioning
Package versions follow the official MuJoCo release versions.
For example:

| npm version | MuJoCo version |
|-------------|----------------|
| 3.5.0       | 3.5.0          |

---

For full engine documentation, see: https://mujoco.readthedocs.io
