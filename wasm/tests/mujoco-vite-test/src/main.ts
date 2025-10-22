import type { MainModule, MjData, MjModel } from "mujoco-js";

// Vite will resolve 'mujoco-js' to the linked package.
// We are assuming the main export ('mujoco_wasm.js') has a default export
// which is the loader function.
import loadMujoco from 'mujoco-js';

async function main() {
  console.log('Attempting to load MuJoCo...');
  const mujoco: MainModule = await loadMujoco();
  console.log('MuJoCo module loaded successfully.');

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

  // Write the model XML to the virtual file system.
  (mujoco as any).FS.writeFile('/working/hello.xml', xmlContent);
  console.log('Wrote hello.xml to virtual file system.');

  let model: MjModel | undefined;
  let data: MjData | undefined;

  try {
    console.log('Loading model from virtual XML...');
    model = mujoco.MjModel.loadFromXML('/working/hello.xml');
    if (!model) {
      throw new Error('Failed to load model from XML');
    }
    console.log('Model loaded:', model);

    data = new mujoco.MjData(model);
    if (!data) {
      throw new Error('Failed to create MjData');
    }
    console.log('Data created:', data.act);

    console.log('SUCCESS: Model and Data exist!');

  } catch (error) {
    console.error("An error occurred during MuJoCo initialization:", error);
  } finally {
    // Clean up MuJoCo resources to prevent memory leaks.
    model?.delete();
    data?.delete();
    (mujoco as any).FS.unmount('/working');
    console.log('Cleaned up resources.');
  }
}

main();
