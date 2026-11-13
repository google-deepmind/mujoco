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

import loadMujoco, {MainModule, MjvScene} from '../dist/mujoco.js';

describe('MjvScene flex face buffers', () => {
  let mujoco: MainModule;

  beforeAll(async () => {
    mujoco = await loadMujoco();
  }, 60_000);

  function expectFaceBuffers(scene: MjvScene, faces: number) {
    expect(scene.flexface.length).toBe(9 * faces);
    expect(scene.flexnormal.length).toBe(9 * faces);
    expect(scene.flextexcoord.length).toBe(6 * faces);
  }

  const circular = `
    <flexcomp name="circular" dim="1" count="3 1 1" spacing=".1 1 1">
      <elasticity young="1e6"/>
    </flexcomp>`;
  const box = `
    <flexcomp name="box" dim="1" count="3 1 1" spacing=".1 1 1">
      <elasticity young="1e6" section="box" sectionsize=".02 .01"/>
    </flexcomp>`;
  const surface = `
    <flexcomp name="surface" dim="2" count="3 3 1" spacing=".1 .1 1"/>`;
  const volume = `
    <flexcomp name="volume" dim="3" count="3 3 3" spacing=".1 .1 .1"/>`;

  const cases = [
    {name: 'no flexes', xml: '', faces: []},
    {name: 'circular rod', xml: circular, faces: [0]},
    {name: 'box rod', xml: box, faces: [24]},
    {name: 'surface', xml: surface, faces: [32]},
    {name: 'volume', xml: volume, faces: [144]},
    {name: 'mixed flexes', xml: circular + surface + volume + box,
      faces: [0, 32, 144, 24]},
  ];

  for (const {name, xml, faces} of cases) {
    it(`should expose the allocated face buffers for ${name}`, () => {
      const path = '/scene_test.xml';
      mujoco.FS.writeFile(path, `<mujoco><worldbody>${xml}</worldbody></mujoco>`);
      const model = mujoco.MjModel.mj_loadXML(path);
      mujoco.FS.unlink(path);
      const scene = new mujoco.MjvScene(model, 0);
      try {
        expect(Array.from(scene.flexfacenum)).toEqual(faces);
        expectFaceBuffers(scene, faces.reduce((sum, count) => sum + count, 0));
      } finally {
        scene.delete();
        model.delete();
      }
    });
  }

  it('should expose empty face buffers without a model', () => {
    const scene = new mujoco.MjvScene();
    try {
      expectFaceBuffers(scene, 0);
    } finally {
      scene.delete();
    }
  });
});
