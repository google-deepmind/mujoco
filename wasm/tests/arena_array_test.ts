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

const XML = `
<mujoco>
  <option><flag island="disable"/></option>
  <worldbody>
    <site name="a"/>
    <body>
      <joint name="hinge" axis="0 1 0" range="-1 1"/>
      <geom size=".1" pos="0 0 -.5"/>
      <site name="b" pos="0 0 -1"/>
    </body>
  </worldbody>
  <tendon><spatial range="0 2"><site site="a"/><site site="b"/></spatial></tendon>
  <actuator><motor joint="hinge"/></actuator>
</mujoco>`;

describe('Unallocated mjData arrays', () => {
  let mujoco: MainModule;
  beforeAll(async () => { mujoco = await loadMujoco(); });

  it('returns initialized arrays without exposing the WASM null address', () => {
    const model = mujoco.MjModel.from_xml_string(XML);
    const data = new mujoco.MjData(model);
    try {
      const fields = [
        ['tendon_efcadr', model.ntendon, -1],
        ['tree_island', model.ntree, -1],
        ['dof_island', model.nv, -1],
        ['efm_c', model.nv, 0],
        ['efm_K_rownnz', model.nv, 0],
        ['efm_tid', model.ntendon, 0],
        ['efm_as', model.nactuator, 0],
      ] as const;
      for (const [name, count, fill] of fields) {
        const array = data[name];
        expect(count).withContext(name).toBeGreaterThan(0);
        expect(array.length).withContext(name).toBe(count);
        expect(Array.from(array)).withContext(name).toEqual(Array(count).fill(fill));
        expect(array.buffer).withContext(name).not.toBe(data.qpos.buffer);
        // A fallback is independent storage, not writable engine state.
        if (array.buffer !== data.qpos.buffer) {
          array[0] = 17;
          expect(data[name][0]).withContext(name).toBe(fill);
        }
      }
      expect(data.efm_c.BYTES_PER_ELEMENT).toBe(data.qpos.BYTES_PER_ELEMENT);
      expect(data.dof_island.BYTES_PER_ELEMENT).toBe(4);
    } finally {
      data.delete();
      model.delete();
    }
  });

  it('handles nonempty constraint arrays when islands are disabled', () => {
    const model = mujoco.MjModel.from_xml_string(XML);
    const data = new mujoco.MjData(model);
    try {
      data.qpos[0] = 2;
      mujoco.mj_forward(model, data);
      expect(data.nefc).toBeGreaterThan(0);
      for (const name of ['efc_island', 'iefc_D', 'iefc_force'] as const) {
        const array = data[name];
        expect(array.length).withContext(name).toBe(data.nefc);
        expect(Array.from(array)).withContext(name).toEqual(Array(data.nefc).fill(0));
        expect(array.buffer).withContext(name).not.toBe(data.qpos.buffer);
      }
      // The allocated constraint arrays still refer to engine memory.
      const address = data.tendon_efcadr;
      expect(address.buffer).toBe(data.qpos.buffer);
      address[0] = -7;
      expect(data.tendon_efcadr[0]).toBe(-7);
    } finally {
      data.delete();
      model.delete();
    }
  });

  it('preserves allocated numeric views and restores fallbacks after reset', () => {
    const model = mujoco.MjModel.from_xml_string(`
      <mujoco>
        <option integrator="discrete"/>
        <worldbody><body><joint damping="1"/><geom size=".1"/></body></worldbody>
      </mujoco>`);
    const data = new mujoco.MjData(model);
    try {
      mujoco.mj_forward(model, data);
      expect(data.efm_active).toBeGreaterThan(0);
      expect(data.efm_c.buffer).toBe(data.qpos.buffer);
      data.efm_c[0] = 7;
      expect(data.efm_c[0]).toBe(7);
      mujoco.mj_resetData(model, data);
      expect(data.efm_c.length).toBe(model.nv);
      expect(Array.from(data.efm_c)).toEqual(Array(model.nv).fill(0));
      expect(data.efm_c.buffer).not.toBe(data.qpos.buffer);
    } finally {
      data.delete();
      model.delete();
    }
  });
});
