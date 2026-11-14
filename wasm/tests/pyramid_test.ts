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

describe('Friction pyramid conversion', () => {
  let mujoco: MainModule;
  let single: boolean;
  beforeAll(async () => {
    mujoco = await loadMujoco();
    const model = mujoco.from_xml_string('<mujoco/>');
    single = model.qpos0.BYTES_PER_ELEMENT === 4;
    model.delete();
  }, 60_000);

  for (const dim of [1, 3, 4, 6]) {
    it(`should encode and decode a ${dim}-dimensional contact`, () => {
      const mu = Array.from({length: dim - 1}, (_, i) => 2 ** i);
      const force = dim === 1 ? [8] : [8 * (dim - 1), ...mu.map(x => 2 * x)];
      const expected = dim === 1 ? [8] : mu.flatMap(() => [5, 3]);
      const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
      const pyramid = new Buffer(expected.length);
      const decoded = new Buffer(dim);
      try {
        pyramid.GetView().fill(-1);
        decoded.GetView().fill(-1);
        mujoco.mju_encodePyramid(pyramid, force, mu);
        expect(Array.from(pyramid.GetView())).toEqual(expected);
        mujoco.mju_decodePyramid(decoded, expected, mu);
        expect(Array.from(decoded.GetView())).toEqual(force);
      } finally {
        pyramid.delete(); decoded.delete();
      }
    });
  }

  it('should reject incompatible force and pyramid lengths', () => {
    const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
    const pyramid = new Buffer(3);
    const force = new Buffer(2);
    try {
      expect(() => mujoco.mju_encodePyramid(pyramid, [16, 2, 2], [1, 1]))
          .toThrowError(/pyramid must have size/);
      expect(() => mujoco.mju_decodePyramid(force, [5, 3, 5, 3], [1, 1]))
          .toThrowError(/force must have size/);
    } finally { pyramid.delete(); force.delete(); }
  });
});
