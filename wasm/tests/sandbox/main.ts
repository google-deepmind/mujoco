// Copyright 2025 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import { MainModule, MjData, MjModel } from "../../dist/mujoco_wasm"
import loadMujoco from "../../dist/mujoco_wasm.js"

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
