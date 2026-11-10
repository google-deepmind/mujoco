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

describe('Nullable C string results', () => {
  let mujoco: MainModule;
  beforeAll(async () => { mujoco = await loadMujoco(); }, 60_000);

  it('should return empty strings for unnamed or missing objects', () => {
    const model = mujoco.from_xml_string(`<mujoco><worldbody>
      <body name="named"><geom size=".1"/></body>
      <body><geom size=".1"/></body></worldbody></mujoco>`);
    try {
      const type = mujoco.mjtObj.mjOBJ_BODY.value;
      expect(mujoco.mj_id2name(model, type, 1)).toBe('named');
      for (const id of [-1, 2, model.nbody, 100]) {
        expect(mujoco.mj_id2name(model, type, id)).toBe('');
      }
    } finally { model.delete(); }
  });

  it('should return empty strings for absent actuator input names', () => {
    const model = mujoco.from_xml_string(`<mujoco><worldbody><body>
      <joint name="j"/><geom size=".1"/></body></worldbody>
      <actuator><motor joint="j"/><position joint="j"/></actuator></mujoco>`);
    try {
      expect(mujoco.mj_actuatorInputName(model, 1, 0)).toBe('pos');
      expect(mujoco.mj_actuatorInputName(model, 0, 0)).toBe('');
      expect(mujoco.mj_actuatorInputName(model, 1, 10)).toBe('');
      expect(mujoco.mj_actuatorInputName(model, -1, 0)).toBe('');
    } finally { model.delete(); }
  });

  it('should return empty strings for absent compiler warnings', () => {
    const spec = mujoco.parseXMLString('<mujoco/>');
    try {
      expect(mujoco.mjs_numWarnings(spec)).toBe(0);
      expect(mujoco.mjs_getWarning(spec, -1)).toBe('');
      expect(mujoco.mjs_getWarning(spec, 0)).toBe('');
    } finally { spec.delete(); }
  });
});
