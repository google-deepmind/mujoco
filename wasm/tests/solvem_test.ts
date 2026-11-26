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

describe('Mass matrix solves', () => {
  let mujoco: MainModule;
  let single: boolean;
  beforeAll(async () => {
    mujoco = await loadMujoco();
    const model = mujoco.from_xml_string('<mujoco/>');
    single = model.qpos0.BYTES_PER_ELEMENT === 4;
    model.delete();
  }, 60_000);

  for (const half of [false, true]) {
    const name = half ? 'mj_solveM2' : 'mj_solveM';
    for (const worldbody of ['', '<geom type="sphere" size="1"/>']) {
      it(`should let ${name} solve a model with no degrees of freedom (${worldbody})`, () => {
        const model = mujoco.from_xml_string(
            `<mujoco><worldbody>${worldbody}</worldbody></mujoco>`);
        const data = new mujoco.MjData(model);
        const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
        const result = new Buffer(0);
        try {
          mujoco.mj_forward(model, data);
          if (half) {
            mujoco.mj_solveM2(model, data, result, [], []);
          } else {
            mujoco.mj_solveM(model, data, result, []);
          }
          expect(result.GetView().length).toBe(0);
        } finally {
          result.delete();
          data.delete();
          model.delete();
        }
      });
    }

    it(`should reject nonempty ${name} buffers when nv is zero`, () => {
      const model = mujoco.from_xml_string('<mujoco/>');
      const data = new mujoco.MjData(model);
      const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
      const result = Buffer.FromArray([7]);
      try {
        expect(() => {
          if (half) {
            mujoco.mj_solveM2(model, data, result, [1], []);
          } else {
            mujoco.mj_solveM(model, data, result, [1]);
          }
        }).toThrowError(/x must have size 0/);
        expect(Array.from(result.GetView())).toEqual([7]);
      } finally {
        result.delete();
        data.delete();
        model.delete();
      }
    });

    it(`should solve multiple right-hand sides with ${name}`, () => {
      const model = mujoco.from_xml_string(`
        <mujoco><worldbody>
          <body><joint type="slide"/><geom type="sphere" size=".1" mass="1"/></body>
          <body pos="1 0 0"><joint type="slide"/>
            <geom type="sphere" size=".1" mass="4"/>
          </body>
        </worldbody></mujoco>`);
      const data = new mujoco.MjData(model);
      const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
      const result = new Buffer(4);
      try {
        mujoco.mj_forward(model, data);
        if (half) {
          mujoco.mj_solveM2(model, data, result, [2, 8, 3, 12], [1, 0.5]);
        } else {
          mujoco.mj_solveM(model, data, result, [2, 8, 3, 12]);
        }
        expect(Array.from(result.GetView())).toEqual(half ? [2, 4, 3, 6] : [2, 2, 3, 3]);
      } finally {
        result.delete();
        data.delete();
        model.delete();
      }
    });
  }
});
