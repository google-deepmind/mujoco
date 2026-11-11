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

describe('Box QP Hessian dimensions', () => {
  let mujoco: MainModule;
  let single: boolean;
  beforeAll(async () => {
    mujoco = await loadMujoco();
    const model = mujoco.from_xml_string('<mujoco/>');
    single = model.qpos0.BYTES_PER_ELEMENT === 4;
    model.delete();
  }, 60_000);

  for (const size of [1, 9]) {
    it(`should reject a square Hessian with ${size} entries for two variables`, () => {
      const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
      const res = Buffer.FromArray([7, 9]);
      const workspace = new Buffer(18);
      try {
        const width = Math.sqrt(size);
        const hessian = Array.from({length: size}, (_, i) =>
            i % (width + 1) === 0 ? 1 : 0);
        expect(() => mujoco.mju_boxQP(
            res, workspace, null, hessian, [-4, 8],
            [-Infinity, -Infinity], [Infinity, Infinity]))
            .toThrowError(/H must have size 4/);
        expect(Array.from(res.GetView())).toEqual([7, 9]);
      } finally {
        res.delete();
        workspace.delete();
      }
    });
  }

  for (const bounded of [false, true]) {
    it(`should solve a correctly sized ${bounded ? 'bounded' : 'unbounded'} problem`, () => {
      const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
      const res = new Buffer(2);
      const workspace = new Buffer(18);
      try {
        const rank = mujoco.mju_boxQP(res, workspace, null, [4, 0, 0, 4], [-4, 8],
                                     bounded ? [-0.5, -1] : [-Infinity, -Infinity],
                                     bounded ? [0.5, 1] : [Infinity, Infinity]);
        expect(rank).toBe(bounded ? 0 : 2);
        expect(Array.from(res.GetView())).toEqual(bounded ? [0.5, -1] : [1, -2]);
      } finally {
        res.delete();
        workspace.delete();
      }
    });
  }
});
