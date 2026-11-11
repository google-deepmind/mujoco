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

// Run with: node --test studio/live_test.mjs
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import test from 'node:test';
import vm from 'node:vm';

const source = readFileSync(new URL('./live.js', import.meta.url), 'utf8');
const flush = () => new Promise(resolve => setImmediate(resolve));

function deferred() {
  let resolve;
  let reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return {promise, resolve, reject};
}

// Exercise the production script with controlled animation frames and WASM
// promises. No renderer or network is needed to test the scheduling contract.
function page(search = '?model=') {
  const frames = [];
  const errors = [];
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) {
      elements.set(id, {style: {}, addEventListener() {}});
    }
    return elements.get(id);
  };
  const context = vm.createContext({
    URLSearchParams,
    Uint8Array,
    navigator: {userAgent: '', platform: '', maxTouchPoints: 0},
    window: {
      location: {search, origin: 'https://example.test', pathname: '/index.html'},
      addEventListener() {},
    },
    document: {
      getElementById: element,
      createElement: () => ({}),
      addEventListener() {},
      body: {appendChild() {}},
    },
    requestAnimationFrame: callback => frames.push(callback),
    console: {log() {}, warn() {}, error: (...args) => errors.push(args)},
    fetch: async () => new Response(new Uint8Array(0)),
    FileReader: class {
      readAsArrayBuffer(file) { this.onload({target: {result: file.buffer}}); }
    },
  });
  vm.runInContext(source, context);
  const module = context.Module;
  Object.assign(module, {
    registerAsset() {}, init() {}, primeFetchCache() {},
    getXMLDependencies: async () => null,
    loadUrl() {}, loadKeyframe() {}, loadFile() {}, renderFrame() {},
  });
  return {
    module, errors,
    overlay: element('loadingOverlay'),
    upload: name => context.handleFile({name, buffer: new ArrayBuffer(1)}),
    async frame() {
      for (const callback of frames.splice(0)) callback();
      await flush();
    },
    async paint() {
      await flush();
      await this.frame();
      await this.frame();
    },
  };
}

test('rendering waits for an uploaded model and its overlay stays visible', async () => {
  const p = page();
  const load = deferred();
  let renders = 0;
  p.module.loadFile = () => load.promise;
  p.module.renderFrame = () => { renders++; };
  p.upload('model.xml');
  await p.paint();
  const frame = p.module.animate();
  try {
    await flush();
    assert.equal(renders, 0);
    assert.equal(p.overlay.style.display, 'flex');
  } finally {
    load.resolve();
    await frame;
  }
  assert.equal(renders, 1);
  assert.equal(p.overlay.style.display, 'none');
});

test('an upload waits for an already suspended render frame', async () => {
  const p = page();
  const render = deferred();
  let loads = 0;
  p.module.renderFrame = () => render.promise;
  p.module.loadFile = () => { loads++; };
  const frame = p.module.animate();
  await flush();
  p.upload('model.xml');
  try {
    await p.paint();
    assert.equal(loads, 0);
  } finally {
    render.resolve();
    await frame;
  }
  await p.paint();
  assert.equal(loads, 1);
});

test('multiple uploads run sequentially', async () => {
  const p = page();
  const first = deferred();
  const loads = [];
  p.module.loadFile = name => {
    loads.push(name);
    return name === 'first.xml' ? first.promise : undefined;
  };
  p.upload('first.xml');
  p.upload('second.xml');
  try {
    await p.paint();
    assert.deepEqual(loads, ['first.xml']);
    assert.equal(p.overlay.style.display, 'flex');
  } finally {
    first.resolve();
  }
  await p.paint();
  assert.deepEqual(loads, ['first.xml', 'second.xml']);
  assert.equal(p.overlay.style.display, 'none');
});

test('a rejected upload is reported and does not stop later operations', async () => {
  const p = page();
  let loads = 0;
  let renders = 0;
  p.module.loadFile = () => {
    loads++;
    if (loads === 1) return Promise.reject(new Error('load failed'));
  };
  p.module.renderFrame = () => { renders++; };
  p.upload('bad.xml');
  p.upload('good.xml');
  await p.paint();
  await p.paint();
  await p.module.animate();
  assert.equal(loads, 2);
  assert.equal(renders, 1);
  assert.equal(p.errors.length, 1);
  assert.equal(p.errors[0][0], 'Failed to load model from file:');
  assert.match(String(p.errors[0][1]), /load failed/);
  assert.equal(p.overlay.style.display, 'none');
});

test('initial URL loading and keyframe selection precede queued uploads', async () => {
  const p = page('?model=https://example.test/root.xml&keyframe=start');
  const initial = deferred();
  const calls = [];
  p.module.loadUrl = () => { calls.push('url'); return initial.promise; };
  p.module.loadKeyframe = name => { calls.push(name); };
  p.module.loadFile = () => { calls.push('file'); };
  p.module.onRuntimeInitialized();
  await p.paint();
  p.upload('model.xml');
  try {
    await p.paint();
    assert.deepEqual(calls, ['url']);
  } finally {
    initial.resolve();
  }
  await p.paint();
  assert.deepEqual(calls, ['url', 'start', 'file']);
  assert.equal(p.overlay.style.display, 'none');
});

test('a rejected render frame does not stop the next frame', async () => {
  const p = page();
  let renders = 0;
  p.module.renderFrame = () => {
    renders++;
    if (renders === 1) return Promise.reject(new Error('render failed'));
  };
  await p.module.animate();
  await p.frame();
  assert.equal(renders, 2);
  assert.equal(p.errors.length, 1);
  assert.equal(p.errors[0][0], 'Update error:');
});

test('a synchronous load exception restores the overlay and permits rendering', async () => {
  const p = page();
  p.module.loadFile = () => { throw new Error('load failed'); };
  p.upload('bad.xml');
  await p.paint();
  await p.module.animate();
  assert.equal(p.errors.length, 1);
  assert.equal(p.overlay.style.display, 'none');
});
