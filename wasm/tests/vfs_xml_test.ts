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

describe('XML strings with caller-owned VFS files', () => {
  let mujoco: MainModule;
  const asset = '<mujoco><worldbody><geom name="included" size=".1"/></worldbody></mujoco>';
  const xml = `<mujoco><include file="model.xml"/>
    <worldbody><geom name="local" size=".2"/></worldbody></mujoco>`;
  beforeAll(async () => { mujoco = await loadMujoco(); }, 60_000);

  for (const method of ['module', 'class']) {
    it(`should preserve colliding assets through the ${method} loader`, () => {
      const vfs = new mujoco.MjVFS();
      try {
        vfs.addBuffer('model.xml', new TextEncoder().encode(asset));
        vfs.addBuffer('model_.xml', new TextEncoder().encode('<mujoco/>'));
        for (let i = 0; i < 2; i++) {
          const model = method === 'module' ? mujoco.from_xml_string(xml, vfs) :
              mujoco.MjModel.from_xml_string(xml, vfs);
          try {
            expect(model.ngeom).toBe(2);
            expect(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, 'included'))
                .toBeGreaterThanOrEqual(0);
          } finally { model.delete(); }
        }
        const original = mujoco.MjModel.from_xml_path('model.xml', vfs);
        try { expect(original.ngeom).toBe(1); } finally { original.delete(); }
        const other = mujoco.MjModel.from_xml_path('model_.xml', vfs);
        try { expect(other.ngeom).toBe(0); } finally { other.delete(); }
      } finally { vfs.delete(); }
    });
  }

  it('should remove its temporary file after a loading error', () => {
    const vfs = new mujoco.MjVFS();
    try {
      expect(() => mujoco.from_xml_string('<mujoco><invalid/></mujoco>', vfs))
          .toThrowError(/Loading error/);
      // A failed load must not leave the temporary model.xml in the VFS.
      vfs.addBuffer('model.xml', new TextEncoder().encode(asset));
      const model = mujoco.from_xml_string(xml, vfs);
      try { expect(model.ngeom).toBe(2); } finally { model.delete(); }
    } finally { vfs.delete(); }
  });
});
