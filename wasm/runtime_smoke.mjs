// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';

globalThis.navigator ??= {hardwareConcurrency: 2};
const {default: loadMujoco} = await import(pathToFileURL(process.argv[2]));
const mujoco = await loadMujoco();
mujoco.FS.writeFile('/model.xml', `<mujoco>
  <worldbody><body pos="0 0 1"><freejoint/><geom size="0.1"/></body></worldbody>
</mujoco>`);
const model = mujoco.MjModel.mj_loadXML('/model.xml');
assert.ok(model);
const data = new mujoco.MjData(model);
mujoco.mj_step(model, data);
assert.ok(data.time > 0);
assert.ok(data.qpos[2] < 1);
data.delete();
model.delete();

mujoco.FS.writeFile('/tetra.obj', `v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 1 3 2
f 1 2 4
f 1 4 3
f 2 3 4
`);
mujoco.FS.writeFile('/mesh.xml', `<mujoco>
  <asset><mesh name="tetra" file="/tetra.obj"/></asset>
  <worldbody><geom type="mesh" mesh="tetra"/></worldbody>
</mujoco>`);
const meshModel = mujoco.MjModel.mj_loadXML('/mesh.xml');
assert.ok(meshModel);
assert.equal(meshModel.nmesh, 1);
meshModel.delete();
process.exit(0);
