// Copyright 2026 Google LLC
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

// Run against a built Studio WASM module:
// MUJOCO_STUDIO_WASM=build_wasm/bin/mujoco_studio.js node --test studio/emscripten_test.mjs
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createRequire} from 'node:module';
import path from 'node:path';
import {before, test} from 'node:test';
import vm from 'node:vm';

let module;
before(async () => {
  assert.ok(process.env.MUJOCO_STUDIO_WASM,
      'Set MUJOCO_STUDIO_WASM to the built mujoco_studio.js file');
  const filename = path.resolve(process.env.MUJOCO_STUDIO_WASM);
  let initialized;
  const ready = new Promise(resolve => { initialized = resolve; });
  module = {
    wasmBinary: readFileSync(filename.replace(/\.js$/, '.wasm')),
    onRuntimeInitialized: () => initialized(),
  };
  // Load the classic Emscripten script with its Module configuration. These
  // bindings do not require app initialization, a renderer, or worker threads.
  const context = vm.createContext({
    Module: module,
    require: createRequire(filename),
    process, console, setTimeout, clearTimeout, TextDecoder, TextEncoder, performance,
    __filename: filename,
    __dirname: path.dirname(filename),
    navigator: {hardwareConcurrency: 0},
  });
  context.global = context;
  vm.runInContext(readFileSync(filename, 'utf8'), context, {filename});
  await ready;
}, {timeout: 60_000});

test('primeFetchCache accepts response filenames', () => {
  assert.doesNotThrow(() => module.primeFetchCache(
      'https://example.test/asset', new Uint8Array([0, 255, 127]), 'mesh.obj'));
});

test('primeFetchCache retains the two-argument overload', () => {
  assert.doesNotThrow(() => module.primeFetchCache(
      'https://example.test/other.obj', new Uint8Array([0, 255, 127])));
});
