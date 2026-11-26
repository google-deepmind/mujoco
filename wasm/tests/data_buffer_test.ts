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

import loadMujoco, {MainModule} from '../dist/mujoco.js';

describe('MjData byte buffers', () => {
  let mujoco: MainModule;

  beforeAll(async () => {
    mujoco = await loadMujoco();
  });

  for (const userdata of [0, 4096]) {
    it(`uses the data allocation size with ${userdata} userdata values`, () => {
      const model = mujoco.MjModel.from_xml_string(`
        <mujoco>
          <size nuserdata="${userdata}"/>
          <worldbody>
            <body><freejoint/><geom size="0.1"/></body>
          </worldbody>
        </mujoco>`);
      const data = new mujoco.MjData(model);
      try {
        // Exercise both a model buffer larger than the data buffer and the
        // reverse, so neither overlong nor truncated views can pass.
        if (userdata === 0) {
          expect(model.nbuffer).toBeGreaterThan(data.nbuffer);
        } else {
          expect(model.nbuffer).toBeLessThan(data.nbuffer);
        }
        expect(model.buffer.length).toBe(model.nbuffer);
        expect(data.buffer.length).toBe(data.nbuffer);
        expect(data.arena.length).toBe(data.narena);
        const qpos = data.qpos;
        const bytes = new Uint8Array(qpos.buffer, qpos.byteOffset,
                                    qpos.byteLength);
        const offset = qpos.byteOffset - data.buffer.byteOffset;
        expect(Array.from(data.buffer.subarray(offset, offset + bytes.length)))
            .toEqual(Array.from(bytes));
      } finally {
        data.delete();
        model.delete();
      }
    });
  }
});
