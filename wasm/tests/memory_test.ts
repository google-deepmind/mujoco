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

import loadMujoco, {MainModule} from '../dist/mujoco_wasm_memory_test.js';

describe('WASM wrapper ownership', () => {
  let mujoco: MainModule;
  const xml = '<mujoco><worldbody><geom size=".1"/></worldbody></mujoco>';

  beforeAll(async () => {
    mujoco = await loadMujoco();
  }, 60_000);

  function expectNoLeak(operation: () => void) {
    // Warm up parser caches and allocator bookkeeping before measuring.
    for (let i = 0; i < 10; i++) operation();
    const before = mujoco.getAllocatedBytes();
    for (let i = 0; i < 100; i++) operation();
    expect(mujoco.getAllocatedBytes()).toBeLessThanOrEqual(before);
  }

  it('should release parsed specs', () => {
    expectNoLeak(() => mujoco.parseXMLString(xml).delete());
  });

  it('should release specs parsed with a VFS', () => {
    const vfs = new mujoco.MjVFS();
    try {
      expectNoLeak(() => mujoco.parseXMLString(xml, vfs).delete());
    } finally {
      vfs.delete();
    }
  });

  for (const populated of [false, true]) {
    it(`should release ${populated ? 'a populated' : 'an empty'} VFS`, () => {
      expectNoLeak(() => {
        const vfs = new mujoco.MjVFS();
        try {
          if (populated) vfs.addBuffer('test.bin', new Uint8Array([1, 2, 3]));
        } finally {
          vfs.delete();
        }
      });
    });
  }

  it('should preserve a spec when a borrowed wrapper is deleted', () => {
    const spec = mujoco.parseXMLString(xml);
    try {
      const body = mujoco.mjs_findBody(spec, 'world')!;
      const element = body.element;
      const borrowed = mujoco.mjs_getSpec(element);
      element.delete();
      body.delete();
      expect(borrowed).toBeDefined();
      borrowed!.delete();
      const model = mujoco.mj_compile(spec);
      try {
        expect(model.ngeom).toBe(1);
      } finally {
        model.delete();
      }
    } finally {
      spec.delete();
    }
  });

  it('should release copied specs independently of their source', () => {
    const spec = mujoco.parseXMLString(xml);
    try {
      expectNoLeak(() => new mujoco.MjSpec(spec).delete());
      const copy = new mujoco.MjSpec(spec);
      try {
        spec.delete();
        const model = mujoco.mj_compile(copy);
        try {
          expect(model.ngeom).toBe(1);
        } finally {
          model.delete();
        }
      } finally {
        copy.delete();
      }
    } finally {
      if (!spec.isDeleted()) spec.delete();
    }
  });
});
