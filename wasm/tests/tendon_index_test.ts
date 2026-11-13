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

describe('Named tendon Jacobian columns', () => {
  let mujoco: MainModule;
  beforeAll(async () => { mujoco = await loadMujoco(); }, 60_000);

  it('should read and write each tendon row independently', () => {
    const model = mujoco.from_xml_string(`<mujoco><worldbody>
      <site name="s0" pos="0 0 0"/><site name="s1" pos="1 0 0"/>
      <body><joint name="a"/><geom size=".1"/></body>
      <body><joint name="b"/><geom size=".1"/></body>
      <body><joint name="c"/><geom size=".1"/></body>
    </worldbody><tendon>
      <fixed name="two"><joint joint="a" coef="1"/><joint joint="c" coef="1"/></fixed>
      <fixed name="one"><joint joint="b" coef="1"/></fixed>
      <spatial name="empty"><site site="s0"/><site site="s1"/></spatial>
    </tendon></mujoco>`);
    const two = model.tendon('two');
    const one = model.tendon('one');
    const empty = model.tendon('empty');
    try {
      expect(Array.from(model.ten_J_colind)).toEqual([0, 2, 1]);
      expect(Array.from(two.J_colind)).toEqual([0, 2]);
      expect(Array.from(one.J_colind)).toEqual([1]);
      expect(Array.from(empty.J_colind)).toEqual([]);
      two.J_colind = [1, 2];
      one.J_colind = [0];
      empty.J_colind = [];
      expect(Array.from(model.ten_J_colind)).toEqual([1, 2, 0]);
      two.J_colind[0] = 2;
      expect(Array.from(model.ten_J_colind)).toEqual([2, 2, 0]);
    } finally {
      two.delete(); one.delete(); empty.delete(); model.delete();
    }
  });
});
