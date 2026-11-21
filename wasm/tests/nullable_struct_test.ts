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
import loadMujoco, {MainModule, MjsBody, MjSpec} from '../dist/mujoco.js';

describe('Nullable struct arguments', () => {
  let mj: MainModule;
  let spec: MjSpec;
  let body: MjsBody;

  beforeAll(async () => {
    mj = await loadMujoco();
  }, 60_000);

  beforeEach(() => {
    spec = mj.parseXMLString(`
      <mujoco>
        <default>
          <geom size="1" mass="2"/>
          <default class="wide"><geom size="2" mass="3"/></default>
        </default>
        <worldbody><body name="body" childclass="wide"><geom/></body></worldbody>
      </mujoco>`);
    body = mj.mjs_findBody(spec, 'body')!;
  });

  afterEach(() => {
    body.delete();
    spec.delete();
  });

  it('should inherit the body defaults when adding a geom with null defaults', () => {
    const geom = mj.mjs_addGeom(body, null)!;
    try {
      expect(Array.from(geom.size)).toEqual([2, 0, 0]);
      const model = mj.mj_compile(spec);
      try {
        expect(model.ngeom).toBe(2);
        expect(model.body_mass[1]).toBe(6);
      } finally {
        model.delete();
      }
    } finally {
      geom.delete();
    }
  });

  it('should add a child body with null defaults', () => {
    const child = mj.mjs_addBody(body, null)!;
    try {
      const geom = mj.mjs_addGeom(child, null)!;
      try {
        expect(Array.from(geom.size)).toEqual([2, 0, 0]);
        const model = mj.mj_compile(spec);
        try {
          expect(model.nbody).toBe(3);
          expect(model.ngeom).toBe(2);
        } finally {
          model.delete();
        }
      } finally {
        geom.delete();
      }
    } finally {
      child.delete();
    }
  });

  it('should still accept explicit defaults', () => {
    const defaults = mj.mjs_getSpecDefault(spec)!;
    try {
      const geom = mj.mjs_addGeom(body, defaults)!;
      try {
        expect(Array.from(geom.size)).toEqual([1, 0, 0]);
      } finally {
        geom.delete();
      }
    } finally {
      defaults.delete();
    }
  });

  for (const explicitParent of [false, true]) {
    it(`should add defaults with an explicit parent=${explicitParent}`, () => {
      const parent = explicitParent ? mj.mjs_findDefault(spec, 'wide')! : null;
      try {
        const defaults = mj.mjs_addDefault(spec, 'new', parent)!;
        try {
          const geom = mj.mjs_addGeom(body, defaults)!;
          try {
            expect(Array.from(geom.size)).toEqual([explicitParent ? 2 : 1, 0, 0]);
          } finally {
            geom.delete();
          }
        } finally {
          defaults.delete();
        }
      } finally {
        parent?.delete();
      }
    });
  }

  for (const withVfs of [false, true]) {
    it(`should adopt inferred inertia with a VFS=${withVfs}`, () => {
      const vfs = withVfs ? new mj.MjVFS() : null;
      try {
        expect(mj.mjs_adoptInertial(body, vfs)).toBe(0);
        expect(body.mass).toBe(3);
      } finally {
        vfs?.delete();
      }
    });
  }

  it('should still reject null for a required body', () => {
    expect(() => mj.mjs_addGeom(null as unknown as MjsBody, null))
        .toThrowError(/null is not a valid MjsBody/);
  });
});
