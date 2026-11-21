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

describe('Fixed-size function arguments', () => {
  let mujoco: MainModule;
  let NumBuffer: MainModule['DoubleBuffer'] | MainModule['FloatBuffer'];

  beforeAll(async () => {
    mujoco = await loadMujoco();
    const option = new mujoco.MjOption();
    try {
      NumBuffer = option.gravity instanceof Float32Array ?
          mujoco.FloatBuffer : mujoco.DoubleBuffer;
    } finally {
      option.delete();
    }
  }, 60_000);

  for (const length of [0, 2, 4]) {
    it(`rejects a vector of length ${length}`, () => {
      const result = NumBuffer.FromArray([7, 8, 9]);
      try {
        expect(() => mujoco.mju_rotVecQuat(
            result, Array(length).fill(1), [1, 0, 0, 0]))
            .toThrowError(/vec must have size 3/);
        expect(Array.from(result.GetView())).toEqual([7, 8, 9]);
      } finally {
        result.delete();
      }
    });
  }

  for (const length of [0, 3, 5]) {
    it(`rejects a quaternion of length ${length}`, () => {
      const result = NumBuffer.FromArray([7, 8, 9]);
      try {
        expect(() => mujoco.mju_rotVecQuat(
            result, [1, 2, 3], Array(length).fill(1)))
            .toThrowError(/quat must have size 4/);
        expect(Array.from(result.GetView())).toEqual([7, 8, 9]);
      } finally {
        result.delete();
      }
    });
  }

  it('rejects a short output view without writing past its end', () => {
    // The backing buffer has room for the native write even before the fix.
    const storage = NumBuffer.FromArray([7, 8, 9, 99]);
    try {
      const result = storage.GetView().subarray(0, 2);
      expect(() => mujoco.mju_rotVecQuat(result, [1, 2, 3], [1, 0, 0, 0]))
          .toThrowError(/res must have size 3/);
      expect(Array.from(storage.GetView())).toEqual([7, 8, 9, 99]);
    } finally {
      storage.delete();
    }
  });

  it('checks fixed-size matrices as well as vectors', () => {
    const result = NumBuffer.FromArray([7, 8, 9]);
    try {
      expect(() => mujoco.mju_mulMatVec3(result, Array(8).fill(0), [1, 2, 3]))
          .toThrowError(/mat must have size 9/);
      expect(Array.from(result.GetView())).toEqual([7, 8, 9]);
    } finally {
      result.delete();
    }
  });

  it('preserves null optional outputs and checks supplied outputs', () => {
    const result = NumBuffer.FromArray([7, 8, 9]);
    try {
      expect(() => mujoco.mjd_quatIntegrate([0, 0, 0], 1, null, null, null))
          .not.toThrow();
      expect(() => mujoco.mjd_quatIntegrate(
          [0, 0, 0], 1, result, null, null))
          .toThrowError(/Dquat must have size 9/);
      expect(Array.from(result.GetView())).toEqual([7, 8, 9]);
    } finally {
      result.delete();
    }
  });

  it('accepts correctly sized buffers and arrays', () => {
    const result = new NumBuffer(3);
    try {
      mujoco.mju_rotVecQuat(result, [1, 2, 3], [1, 0, 0, 0]);
      expect(Array.from(result.GetView())).toEqual([1, 2, 3]);
      mujoco.mju_mulMatVec3(
          result, [1, 0, 0, 0, 1, 0, 0, 0, 1], [3, 2, 1]);
      expect(Array.from(result.GetView())).toEqual([3, 2, 1]);
    } finally {
      result.delete();
    }
  });
});
