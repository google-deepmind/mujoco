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
import loadMujoco from '../dist/mujoco.js';

// Use a fresh module for each case so previous tests cannot leave enough free
// memory to avoid growth during the copy.
describe('Buffer copies across memory growth', () => {
  for (const convert of [false, true]) {
    it(`should copy a heap subarray while growing memory (convert=${convert})`, async () => {
      const mujoco = await loadMujoco();
      const probe = new mujoco.Uint8Buffer(1);
      const initialBytes = probe.GetView().buffer.byteLength;
      probe.delete();
      const Source = convert ? mujoco.FloatBuffer : mujoco.Uint8Buffer;
      const Target = convert ? mujoco.DoubleBuffer : mujoco.Uint8Buffer;
      const count = initialBytes / (convert ? 4 : 1);
      const source = new Source(count + 8);
      let copy: ReturnType<typeof Target.FromArray>|null = null;
      try {
        const input = source.GetView().subarray(4, count + 4);
        input.fill(convert ? 1.5 : 123);
        input[0] = convert ? -2.5 : 231;
        input[count - 1] = convert ? 3.25 : 17;
        const heapBefore = input.buffer.byteLength;
        copy = Target.FromArray(input);
        const output = copy.GetView();
        expect(output.buffer.byteLength).toBeGreaterThan(heapBefore);
        expect(output.length).toBe(count);
        expect(output[0]).toBe(convert ? -2.5 : 231);
        expect(output[Math.floor(count / 2)]).toBe(convert ? 1.5 : 123);
        expect(output[count - 1]).toBe(convert ? 3.25 : 17);
        output[0] = 0;
        expect(source.GetView()[4]).toBe(convert ? -2.5 : 231);
      } finally {
        copy?.delete();
        source.delete();
      }
    }, 60_000);
  }
});
