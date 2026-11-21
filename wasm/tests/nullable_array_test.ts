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

describe('Nullable array arguments', () => {
  let mujoco: MainModule;
  let single: boolean;
  beforeAll(async () => {
    mujoco = await loadMujoco();
    const model = mujoco.from_xml_string('<mujoco/>');
    single = model.qpos0.BYTES_PER_ELEMENT === 4;
    model.delete();
  }, 60_000);

  it('should solve a box QP with null bounds', () => {
    const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
    const result = new Buffer(2);
    const workspace = new Buffer(18);
    try {
      const rank = mujoco.mju_boxQP(
          result, workspace, null, [4, 0, 0, 4], [-4, 8], null, null);
      expect(rank).toBe(2);
      expect(Array.from(result.GetView())).toEqual([1, -2]);
    } finally {
      workspace.delete();
      result.delete();
    }
  });

  it('should initialize a visual geom with null arrays', () => {
    const geom = new mujoco.MjvGeom();
    try {
      mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE.value, null, null, null, null);
      expect(Array.from(geom.pos)).toEqual([0, 0, 0]);
      expect(Array.from(geom.mat)).toEqual([1, 0, 0, 0, 1, 0, 0, 0, 1]);
      expect(Array.from(geom.rgba)).toEqual([0.5, 0.5, 0.5, 1]);
    } finally {
      geom.delete();
    }
  });

  it('should allow independently omitted force and torque arrays', () => {
    const model = mujoco.from_xml_string(`
      <mujoco><worldbody><body><freejoint/>
        <geom type="sphere" size="1"/>
      </body></worldbody></mujoco>`);
    const data = new mujoco.MjData(model);
    const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
    const result = new Buffer(6);
    try {
      mujoco.mj_forward(model, data);
      mujoco.mj_applyFT(model, data, [1, 2, 3], null, [0, 0, 0], 1, result);
      expect(Array.from(result.GetView())).toEqual([1, 2, 3, 0, 0, 0]);
      mujoco.mj_applyFT(model, data, null, [4, 5, 6], [0, 0, 0], 1, result);
      expect(Array.from(result.GetView())).toEqual([1, 2, 3, 4, 5, 6]);
    } finally {
      result.delete();
      data.delete();
      model.delete();
    }
  });

  for (const diag of [null, [2, 3]]) {
    it(`should compute a matrix product with ${diag ? 'supplied' : 'omitted'} diagonal`, () => {
      const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
      const result = new Buffer(9);
      try {
        mujoco.mju_sqrMatTD(result, [1, 2, 3, 4, 5, 6], diag, 2, 3);
        expect(Array.from(result.GetView())).toEqual(
            diag ? [50, 64, 78, 64, 83, 102, 78, 102, 126] :
                   [17, 22, 27, 22, 29, 36, 27, 36, 45]);
      } finally {
        result.delete();
      }
    });
  }

  it('should still reject a diagonal with the wrong size', () => {
    const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
    const result = new Buffer(9);
    try {
      expect(() => mujoco.mju_sqrMatTD(result, [1, 2, 3, 4, 5, 6], [2], 2, 3))
          .toThrowError(/diag must have size 2/);
    } finally {
      result.delete();
    }
  });

  it('should generate a normal sample without the optional second output', () => {
    expect(Number.isFinite(mujoco.mju_standardNormal(null))).toBeTrue();
  });

  it('should write the optional second normal sample when supplied', () => {
    const Buffer = single ? mujoco.FloatBuffer : mujoco.DoubleBuffer;
    const second = Buffer.FromArray([NaN]);
    try {
      expect(Number.isFinite(mujoco.mju_standardNormal(second))).toBeTrue();
      expect(Number.isFinite(second.GetView()[0])).toBeTrue();
    } finally {
      second.delete();
    }
  });

});
