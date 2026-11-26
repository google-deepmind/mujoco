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

describe('Named plastic state access', () => {
  let mujoco: MainModule;
  const xml = `<mujoco><worldbody>
    <body><joint name="elastic"/><geom size=".1"/></body>
    <body><joint name="ball" type="ball" stiffness="1" springyield="1"/>
      <geom size=".1"/></body>
    <body><joint name="hinge" stiffness="1" springyield="1"/>
      <geom size=".1"/></body>
  </worldbody><tendon>
    <fixed name="elastic"><joint joint="elastic" coef="1"/></fixed>
    <fixed name="plastic" stiffness="1" springyield="1">
      <joint joint="hinge" coef="1"/></fixed>
  </tendon></mujoco>`;

  beforeAll(async () => {
    mujoco = await loadMujoco();
  }, 60_000);

  const cases = [
    {kind: 'jnt' as const, name: 'elastic', start: 0, size: 0},
    {kind: 'jnt' as const, name: 'ball', start: 0, size: 4},
    {kind: 'jnt' as const, name: 'hinge', start: 4, size: 1},
    {kind: 'tendon' as const, name: 'elastic', start: 0, size: 0},
    {kind: 'tendon' as const, name: 'plastic', start: 5, size: 2},
  ];
  for (const {kind, name, start, size} of cases) {
    it(`should read and write only the plastic state for ${kind} ${name}`, () => {
      const model = mujoco.from_xml_string(xml);
      const data = new mujoco.MjData(model);
      const accessor = data[kind](name);
      try {
        const original = [1, 2, 3, 4, 5, 6, 7];
        data.plastic.set(original);
        expect(Array.from(accessor.plastic)).toEqual(
            original.slice(start, start + size));
        const replacement = Array.from({length: size}, (_, i) => 10 + i);
        accessor.plastic = replacement;
        const expected = original.slice();
        expected.splice(start, size, ...replacement);
        expect(Array.from(data.plastic)).toEqual(expected);
        // The getter must return a writable view, including for scalar joints.
        accessor.plastic.fill(20);
        expected.splice(start, size, ...new Array(size).fill(20));
        expect(Array.from(data.plastic)).toEqual(expected);
      } finally {
        accessor.delete();
        data.delete();
        model.delete();
      }
    });
  }

  it('should expose empty views when the model has no plastic state', () => {
    const model = mujoco.from_xml_string(`<mujoco><worldbody><body>
      <joint name="joint"/><geom size=".1"/></body></worldbody>
      <tendon><fixed name="tendon"><joint joint="joint" coef="1"/>
      </fixed></tendon></mujoco>`);
    const data = new mujoco.MjData(model);
    const joint = data.jnt('joint');
    const tendon = data.tendon('tendon');
    try {
      expect(data.plastic.length).toBe(0);
      expect(Array.from(joint.plastic)).toEqual([]);
      expect(Array.from(tendon.plastic)).toEqual([]);
      joint.plastic = [];
      tendon.plastic = [];
    } finally {
      joint.delete();
      tendon.delete();
      data.delete();
      model.delete();
    }
  });
});
