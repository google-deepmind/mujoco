// Copyright 2025 DeepMind Technologies Limited
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

import { MainModule, DoubleBuffer } from "../dist/mujoco_wasm_benchmark"
import loadMujoco from "../dist/mujoco_wasm_benchmark.js"

describe('MuJoCo WASM Benchmark Tests', () => {
  let mujoco: MainModule;

  function isNumberArraySorted(arr: number[]): boolean {
    for (let i = 0; i < arr.length; i++) {
      if (!Number.isFinite(arr[i]) || (i > 0 && arr[i - 1] > arr[i])) {
        return false;
      }
    }
    return true;
  }

  function verifySorted(state: any, kind: string): void {
    let array: number[] = [];
    if (kind === 'NumberArray') {
      array = state as number[];
    } else {
      const bufferView = (state as DoubleBuffer).GetView();
      array = Array.from(bufferView) as number[];
    }

    const sorted = isNumberArraySorted(array);
    if (!sorted) {
      throw new Error(
          `Verification failed for ${kind}: expected finite, sorted values`);
    }
  }

  function runBenchmark(iterations: number, items: number, kind: string) {
    // Use the same deterministic input for both APIs. These integers are
    // exactly representable in either single or double precision.
    let seed = 12345;
    const input = Array.from({length: items}, () => {
      seed = (1664525 * seed + 1013904223) >>> 0;
      return seed >>> 16;
    });
    let state: any;
    let buffer: DoubleBuffer|undefined;
    let func: (state: any) => any;
    let reset: () => void;
    if (kind === 'NumberArray') {
      reset = () => { state = input.slice(); };
      func = (state) => mujoco.SortNumberArray(state);
    } else if (kind === 'DoubleBuffer') {
      buffer = mujoco.DoubleBuffer.FromArray(input);
      state = buffer;
      reset = () => { buffer!.GetView().set(input); };
      func = (state) => mujoco.SortDoubleBuffer(state);
    } else {
      throw new Error(`Unsupported benchmark type: ${kind}`);
    }

    try {
      let result: any;
      for (let i = 0; i < 100; i++) {
        reset();
        result = func(state);
      }
      verifySorted(result, kind);

      // Restore the unsorted input outside each timed call. DoubleBuffer sorts
      // in place, whereas NumberArray sorts a copy.
      let totalTime = 0;
      for (let i = 0; i < iterations; i++) {
        reset();
        const start = performance.now();
        result = func(state);
        totalTime += performance.now() - start;
      }
      verifySorted(result, kind);

      const avgTimeMilliseconds = totalTime / iterations;
      console.log(`Benchmark: "Sort ${kind} ${iterations} iterations with ${
          items} items" - AVG time per call: ${
          avgTimeMilliseconds.toFixed(2)} ms`);
      return {totalTime, avgTimeMilliseconds};
    } finally {
      buffer?.delete();
    }
  }

  beforeAll(async () => {
    mujoco = await loadMujoco();
  }, 60_000);

  it('should reject non-finite and sparse sorting results', () => {
    for (const values of [[NaN], [1, Infinity], new Array(3), [2, 1]]) {
      expect(isNumberArraySorted(values)).toBeFalse();
      expect(() => verifySorted(values, 'NumberArray')).toThrowError();
    }
    expect(isNumberArraySorted([])).toBeTrue();
    expect(isNumberArraySorted([-1, 0, 0, 2])).toBeTrue();
  });

  for (const kind of ['NumberArray', 'DoubleBuffer']) {
    it(`should reset finite, identical inputs before every ${kind} sort`,
       () => {
         const isArray = kind === 'NumberArray';
         const method = isArray ? 'SortNumberArray' : 'SortDoubleBuffer';
         const original = mujoco[method] as (state: any) => any;
         const inputs: number[][] = [];
         spyOn(mujoco, method).and.callFake((state: any) => {
           inputs.push(Array.from(isArray ? state : state.GetView()));
           return original(state);
         });
         runBenchmark(3, 16, kind);
         // 100 warm-up calls plus 3 measured calls.
         expect(inputs.length).toBe(103);
         expect(inputs[0].every(Number.isFinite)).toBeTrue();
         expect(new Set(inputs[0]).size).toBeGreaterThan(1);
         for (const input of inputs) {
           expect(input).toEqual(inputs[0]);
         }
       });
  }

  it('should benchmark NumberArray and DoubleBuffer and compare results',
     () => {
       const na1 = runBenchmark(100, 1_000, 'NumberArray');
       const na2 = runBenchmark(100, 400_000, 'NumberArray');

       const db1 = runBenchmark(100, 1_000, 'DoubleBuffer');
       const db2 = runBenchmark(100, 400_000, 'DoubleBuffer');

       // Report performance separately from correctness. Timing order and
       // absolute limits depend on the machine running the benchmark.
       console.log(`NumberArray / DoubleBuffer time: ${
           (na1.totalTime / db1.totalTime).toFixed(2)}x (1,000 items), ${
           (na2.totalTime / db2.totalTime).toFixed(2)}x (400,000 items)`);
     });
});
