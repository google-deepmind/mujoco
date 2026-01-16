# MuJoCo WASM Package

This package contains the pre-compiled WebAssembly module and JavaScript bindings for MuJoCo.

## Installation

```sh
npm install mujoco-js
```

## Usage

```javascript
import type { MainModule, MjData, MjModel } from "mujoco-js";
import loadMujoco from 'mujoco-js';

const mujoco: MainModule = await loadMujoco();

// The Emscripten file system (FS) needs to be mounted to be used.
(mujoco as any).FS.mkdir('/working');
(mujoco as any).FS.mount((mujoco as any).MEMFS, { root: '.' }, '/working');
console.log('Mounted virtual file system.');

const xmlContent = `
  <mujoco model="Box falling">
    <option viscosity="1"/>
    <worldbody>
      <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
      <geom name="MyFloor" type="plane" size="1 1 0.1" rgba=".9 0 0 1" user="5 4 3 2 1"/>
      <geom name="MyWall" type="plane" size="0.1 1 0.1" rgba="0 1 0 1" user="5 4 3"/>
      <body pos="0 0 1" name="MyBox">
        <joint type="free"/>
        <geom name="MyBoxGeom" type="box" size=".1 .2 .3" rgba="0 .9 0 1"/>
      </body>
    </worldbody>
  </mujoco>`;

mujoco.FS.writeFile('/working/hello.xml', xmlContent);
console.log('Wrote hello.xml to virtual file system.');

let model: MjModel | undefined;
let data: MjData | undefined;

try {
  model = mujoco.MjModel.loadFromXML('/working/hello.xml');
  if (!model) {
    throw new Error('Failed to load model from XML');
  }
  console.log('Model loaded:', model);

  data = new mujoco.MjData(model);
  if (!data) {
    throw new Error('Failed to create MjData');
  }
  console.log('Data created:', data);

  console.log('SUCCESS: Model and Data exist!');

} catch (error) {
  console.error("An error occurred during MuJoCo initialization:", error);
} finally {
  model?.delete();
  data?.delete();
  mujoco.FS.unmount('/working');
  console.log('Cleaned up resources.');
}
```
