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

import 'jasmine';

import loadMujoco, {MainModule} from '../dist/mujoco.js';

describe('MuJoCo WASM skin ray buffers', () => {
  let mujoco: MainModule;
  const face = [0, 1, 2];
  const vert = [0, 0, 0, 1, 0, 0, 0, 1, 0];
  const pnt = [0.25, 0.25, 1];
  const vec = [0, 0, -1];

  beforeAll(async () => {
    mujoco = await loadMujoco();
  }, 60_000);

  it('returns a hit and nearest vertex, or a miss', () => {
    const vertid = new mujoco.IntBuffer(1);
    try {
      expect(mujoco.mju_raySkin(1, 3, face, vert, pnt, vec, vertid)).toBe(1);
      expect(vertid.GetView()[0]).toBe(0);
      expect(mujoco.mju_raySkin(1, 3, face, vert, [2, 2, 1], vec, vertid)).toBe(-1);
      expect(vertid.GetView()[0]).toBe(-1);
    } finally {
      vertid.delete();
    }
  });

  it('hits a later triangle with an unnormalized ray direction', () => {
    const vertid = new mujoco.IntBuffer(1);
    try {
      const vertices = [...vert, 2, 0, 0, 3, 0, 0, 2, 1, 0];
      expect(mujoco.mju_raySkin(
          2, 6, [0, 1, 2, 3, 4, 5], vertices, [2.25, 0.25, 1],
          [0, 0, -2], vertid)).toBe(0.5);
      expect(vertid.GetView()[0]).toBe(3);
    } finally {
      vertid.delete();
    }
  });

  it('accepts an omitted vertex output and an empty skin', () => {
    expect(mujoco.mju_raySkin(1, 3, face, vert, pnt, vec, null)).toBe(1);
    const vertid = new mujoco.IntBuffer(1);
    try {
      expect(mujoco.mju_raySkin(0, 0, [], [], pnt, vec, vertid)).toBe(-1);
      expect(vertid.GetView()[0]).toBe(-1);
    } finally {
      vertid.delete();
    }
  });

  for (const [nface, nvert] of [[-1, 3], [1, -1]]) {
    it(`rejects negative counts (${nface}, ${nvert})`, () => {
      expect(() => mujoco.mju_raySkin(nface, nvert, face, vert, pnt, vec, null))
          .toThrowError(/nface and nvert must be non-negative/);
    });
  }

  for (const nface of [0, 2, 2147483647]) {
    it(`rejects a face count of ${nface} for one triangle`, () => {
      expect(() => mujoco.mju_raySkin(nface, 3, face, vert, pnt, vec, null))
          .toThrowError(/face must have size/);
    });
  }

  for (const nvert of [2, 4, 2147483647]) {
    it(`rejects a vertex count of ${nvert} for three vertices`, () => {
      expect(() => mujoco.mju_raySkin(1, nvert, face, vert, pnt, vec, null))
          .toThrowError(/vert must have size/);
    });
  }

  it('rejects a short ray origin', () => {
    expect(() => mujoco.mju_raySkin(1, 3, face, vert, [0, 0], vec, null))
        .toThrowError(/pnt must have size 3/);
  });

  it('rejects a short ray direction', () => {
    expect(() => mujoco.mju_raySkin(1, 3, face, vert, pnt, [0, 0], null))
        .toThrowError(/vec must have size 3/);
  });

  it('rejects an empty vertex output buffer', () => {
    const vertid = new mujoco.IntBuffer(0);
    try {
      expect(() => mujoco.mju_raySkin(1, 3, face, vert, pnt, vec, vertid))
          .toThrowError(/vertid must have size 1/);
    } finally {
      vertid.delete();
    }
  });

  for (const index of [-1, 3, 100000000]) {
    it(`rejects an out-of-range face vertex index of ${index}`, () => {
      expect(() => mujoco.mju_raySkin(1, 3, [0, 1, index], vert, pnt, vec, null))
          .toThrowError(/face vertex index out of bounds/);
    });
  }
});
