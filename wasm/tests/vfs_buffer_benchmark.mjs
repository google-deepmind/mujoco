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

// Run with: node wasm/tests/vfs_buffer_benchmark.mjs
// Reports upload time only; allocation, initialization, and deletion are untimed.
import {performance} from 'node:perf_hooks';
import loadMujoco from '../dist/mujoco.js';

const mujoco = await loadMujoco();
const vfs = new mujoco.MjVFS();
try {
  for (const size of [65_536, 1_048_576, 8_388_608]) {
    const backing = new Uint8Array(size + 32);
    for (let i = 0; i < backing.length; i++) {
      backing[i] = (i * 37 + 11) & 255;
    }
    const bytes = backing.subarray(16, 16 + size);
    const times = [];
    for (let trial = 0; trial < 10; trial++) {
      const start = performance.now();
      vfs.addBuffer('asset.bin', bytes);
      const elapsed = performance.now() - start;
      vfs.deleteFile('asset.bin');
      if (trial >= 3) times.push(elapsed);
    }
    times.sort((a, b) => a - b);
    console.log(`${size} bytes: ${times[3].toFixed(3)} ms median (7 measured uploads)`);
  }
} finally {
  vfs.delete();
}
