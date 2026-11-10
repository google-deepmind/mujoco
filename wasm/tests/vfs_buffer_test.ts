// Copyright 2026 DeepMind Technologies Limited
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

import 'jasmine';
import loadMujoco, {MainModule, MjModel} from '../dist/mujoco.js';

describe('VFS buffer copies', () => {
  let mujoco: MainModule;
  beforeAll(async () => {
    mujoco = await loadMujoco();
  }, 60_000);

  for (const kind of ['array', 'typed array', 'subarray']) {
    it(`should copy binary data from ${kind} input independently of the source`, () => {
      const model = mujoco.from_xml_string(`
        <mujoco>
          <custom><numeric name="values" data="0 127 128 255 .125 -3.5"/></custom>
          <worldbody><body pos="1 2 3"><freejoint/>
            <geom type="sphere" size=".3" mass="3.25"/>
          </body></worldbody>
        </mujoco>`);
      const saved = new mujoco.Uint8Buffer(Number(mujoco.mj_sizeModel(model)));
      const vfs = new mujoco.MjVFS();
      let loaded: MjModel|null = null;
      try {
        mujoco.mj_saveModel(model, null, saved);
        const expected = Array.from(saved.GetView()) as number[];
        let input: number[]|Uint8Array;
        if (kind === 'array') {
          input = [...expected];
        } else if (kind === 'typed array') {
          input = new Uint8Array(expected);
        } else {
          const padded = new Uint8Array(expected.length + 32).fill(255);
          padded.set(expected, 16);
          input = padded.subarray(16, 16 + expected.length);
        }
        vfs.addBuffer('model.mjb', input);
        input.fill(0);
        loaded = mujoco.MjModel.mj_loadModel('model.mjb', vfs);
        expect(loaded.nq).toBe(7);
        expect(Array.from(loaded.body_mass)).toEqual([0, 3.25]);
        expect(Array.from(loaded.numeric_data)).toEqual([0, 127, 128, 255, .125, -3.5]);
        mujoco.mj_saveModel(loaded, null, saved);
        expect(Array.from(saved.GetView())).toEqual(expected);
      } finally {
        loaded?.delete();
        vfs.delete();
        saved.delete();
        model.delete();
      }
    });
  }
});
