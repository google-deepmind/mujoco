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
import loadMujoco, {MainModule, MjsBody, MjSpec, MjVFS} from '../dist/mujoco.js';

describe('Nullable string arguments', () => {
  let mj: MainModule;
  let spec: MjSpec;
  let body: MjsBody;
  let vfs: MjVFS;

  beforeAll(async () => {
    mj = await loadMujoco();
  }, 60_000);

  beforeEach(() => {
    spec = mj.parseXMLString('<mujoco/>');
    body = mj.mjs_findBody(spec, 'world')!;
    vfs = new mj.MjVFS();
  });

  afterEach(() => {
    vfs.delete();
    body.delete();
    spec.delete();
  });

  function makeFlex(type: string|null, dof: string|null, file: string|null,
                    name: string = 'flex') {
    // Supply arrays and a VFS explicitly to isolate nullable string handling.
    return mj.mjs_makeFlex(
        body, name, type, 1, dof, [2, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1],
        .01, 1, 0, 1, 0, 0, 0, [0, 0, 0], [1, 0, 0, 0], [0, 0, 0], file, vfs);
  }

  for (const [label, type, dof, file] of [
    ['explicit strings', 'grid', 'full', ''],
    ['null type', null, 'full', ''],
    ['null dof', 'grid', null, ''],
    ['null file', 'grid', 'full', null],
    ['all null strings', null, null, null],
  ]) {
    it(`should create a grid flex with ${label}`, () => {
      const flex = makeFlex(type, dof, file)!;
      try {
        expect(flex).toBeDefined();
        const model = mj.mj_compile(spec);
        try {
          expect(model.nflex).toBe(1);
          expect(model.nflexvert).toBe(2);
          expect(model.flex_dim[0]).toBe(1);
          expect(model.nv).toBe(6);
        } finally {
          model.delete();
        }
      } finally {
        flex?.delete();
      }
    });
  }

  it('should reject a non-string optional argument', () => {
    expect(() => makeFlex(42 as unknown as string, 'full', ''))
        .toThrowError();
  });

  it('should still reject null for the required name', () => {
    expect(() => makeFlex('grid', 'full', '', null as unknown as string))
        .toThrowError(/name is null/);
  });
});
