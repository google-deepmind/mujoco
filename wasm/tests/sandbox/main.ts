import {MainModule, MjData, MjModel} from 'google3/third_party/mujoco/wasm/codegen/generated/bindings/mujoco';

declare function loadMujoco(): Promise<MainModule>;

async function main() {
  const mujoco: MainModule = await loadMujoco();

  (mujoco as any).FS.mkdir('/working');
  (mujoco as any).FS.mount((mujoco as any).MEMFS, {root: '.'}, '/working');

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

  (mujoco as any).FS.writeFile('/working/hello.xml', xmlContent);
  let model: MjModel|undefined;
  let data: MjData|undefined;

  try {
    console.log('Hello world!: Loading model');
    model = mujoco.MjModel.loadFromXML('/working/hello.xml');
    if (!model) {
      throw new Error('Failed to load model');
    }
    data = new mujoco.MjData(model);
    if (!data) {
      throw new Error('Failed to load data');
    }

    // Add your test code here...

  } finally {
    model?.delete();
    data?.delete();
    (mujoco as any).FS.unmount('/working');
  }
}

main()
