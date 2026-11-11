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
  <worldbody><body>
    <joint name="slide" type="slide"/><geom size=".1"/>
    <body><joint name="ball" type="ball"/><geom size=".1" pos=".2 0 0"/></body>
  </body></worldbody>
  <actuator>
    <motor name="before" joint="slide" ctrlrange="-1 1"/>
    <pid name="pid" joint="slide" input="pos vel ff" kp="1" kv="1" velrange="-2 2"/>
    <orientation name="orient" joint="ball" kp="1" input="quat"/>
    <dcmotor name="passive" joint="slide" input="none" motorconst=".05" resistance="2"/>
    <motor name="after" joint="slide" ctrllimited="false"/>
  </actuator>
</mujoco>`;

describe('Named actuator boolean arrays', () => {
  let mujoco: MainModule;
  beforeAll(async () => { mujoco = await loadMujoco(); });

  for (const [name, expected] of [
    ['pid', [0, 1, 0]], ['orient', [0, 0, 0, 0]],
  ] as const) {
    it(`returns a writable Uint8Array for ${name} controls`, () => {
      const model = mujoco.MjModel.from_xml_string(XML);
      const actuator = model.actuator(name);
      try {
        const start = model.actuator_ctrladr[actuator.id];
        const flags = actuator.ctrllimited;
        expect(flags instanceof Uint8Array).toBeTrue();
        expect(Array.from(flags)).toEqual(Array.from(expected));
        expect(flags.buffer).toBe(model.actuator_ctrllimited.buffer);
        expect(flags.byteOffset).toBe(model.actuator_ctrllimited.byteOffset + start);
        flags[0] = 1;
        expect(model.actuator_ctrllimited[start]).toBe(1);
        model.actuator_ctrllimited[start] = 0;
        expect(flags[0]).toBe(0);
      } finally {
        actuator.delete();
        model.delete();
      }
    });

    it(`assigns ${name} flags without changing neighboring controls`, () => {
      const model = mujoco.MjModel.from_xml_string(XML);
      const actuator = model.actuator(name);
      try {
        const start = model.actuator_ctrladr[actuator.id];
        const flags = expected.map((_, i) => i % 2 === 0);
        const all = Array.from(model.actuator_ctrllimited) as number[];
        all.splice(start, flags.length, ...flags.map(Number));
        actuator.ctrllimited = flags;
        expect(Array.from(model.actuator_ctrllimited)).toEqual(all);
      } finally {
        actuator.delete();
        model.delete();
      }
    });
  }

  it('supports an empty control array and preserves scalar boolean properties', () => {
    const model = mujoco.MjModel.from_xml_string(XML);
    const passive = model.actuator('passive');
    const before = model.actuator('before');
    const after = model.actuator('after');
    try {
      expect(model.actuator_ctrlnum[passive.id]).toBe(0);
      expect(passive.ctrllimited instanceof Uint8Array).toBeTrue();
      expect(passive.ctrllimited.length).toBe(0);
      passive.ctrllimited = [];
      expect(before.ctrllimited).toBe(true);
      expect(after.ctrllimited).toBe(false);
      before.ctrllimited = false;
      after.ctrllimited = true;
      expect(model.actuator_ctrllimited[model.actuator_ctrladr[before.id]]).toBe(0);
      expect(model.actuator_ctrllimited[model.actuator_ctrladr[after.id]]).toBe(1);
    } finally {
      passive.delete();
      before.delete();
      after.delete();
      model.delete();
    }
  });
});
