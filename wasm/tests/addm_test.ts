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

describe('Dense inertia matrix addition', () => {
  let mujoco: MainModule;
  const xml = `<mujoco><worldbody><body>
    <joint type="slide" axis="1 0 0"/><geom size=".1" mass="2"/>
    <body><joint type="slide" axis="1 0 0"/><geom size=".1" mass="3"/></body>
  </body></worldbody></mujoco>`;
  beforeAll(async () => { mujoco = await loadMujoco(); }, 60_000);

  for (const partial of [false, true]) {
    it(`should add to the dense lower triangle with ${partial ? 'partial' : 'null'} sparse metadata`, () => {
      const model = mujoco.from_xml_string(xml);
      const data = new mujoco.MjData(model);
      const Buffer = model.qpos0.BYTES_PER_ELEMENT === 4 ?
          mujoco.FloatBuffer : mujoco.DoubleBuffer;
      const dst = Buffer.FromArray([10, 11, 12, 13]);
      const full = new Buffer(4);
      try {
        expect(model.nv).toBe(2);
        expect(model.nM).toBe(3);
        mujoco.mj_forward(model, data);
        mujoco.mj_fullM(model, data, full);
        expect(Array.from(full.GetView())).toEqual([5, 3, 3, 3]);
        const rownnz = partial ? model.M_rownnz : null;
        mujoco.mj_addM(model, data, dst, rownnz, null, null);
        expect(Array.from(dst.GetView())).toEqual([15, 11, 15, 16]);
        mujoco.mj_addM(model, data, dst, rownnz, null, null);
        expect(Array.from(dst.GetView())).toEqual([20, 11, 18, 19]);
      } finally {
        dst.delete(); full.delete(); data.delete(); model.delete();
      }
    });
  }

  it('should reject dense destinations with the wrong size', () => {
    const model = mujoco.from_xml_string(xml);
    const data = new mujoco.MjData(model);
    const Buffer = model.qpos0.BYTES_PER_ELEMENT === 4 ?
        mujoco.FloatBuffer : mujoco.DoubleBuffer;
    try {
      mujoco.mj_forward(model, data);
      for (const size of [3, 5]) {
        const dst = new Buffer(size);
        try {
          expect(() => mujoco.mj_addM(model, data, dst, null, null, null))
              .toThrowError(/dst must have size 4/);
        } finally { dst.delete(); }
      }
    } finally { data.delete(); model.delete(); }
  });
});
