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

// Run with: node --test studio/live_prefetch_test.mjs
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import test from 'node:test';
import vm from 'node:vm';

const source = readFileSync(new URL('./live.js', import.meta.url), 'utf8');
const rootUrl = 'https://example.test/root.xml';
const assetUrl = 'https://example.test/asset';

function prefetchPage(responses, dependencies = [rootUrl, assetUrl]) {
  const primed = [];
  const fetched = [];
  const discovery = [];
  const context = vm.createContext({
    URLSearchParams, Uint8Array,
    navigator: {userAgent: '', platform: '', maxTouchPoints: 0},
    window: {
      location: {search: '?model=', origin: 'https://example.test', pathname: '/'},
      addEventListener() {},
    },
    document: {
      getElementById: () => ({style: {}, addEventListener() {}}),
      createElement: () => ({}),
      addEventListener() {},
      body: {appendChild() {}},
    },
    console: {log() {}, warn() {}, error() {}},
    fetch: async url => {
      fetched.push(url);
      assert.ok(responses.has(url), `Unexpected URL: ${url}`);
      return responses.get(url);
    },
  });
  vm.runInContext(source, context);
  context.Module.primeFetchCache = (url, bytes, filename) => {
    primed.push({url, bytes: Array.from(bytes), filename});
  };
  context.Module.getXMLDependencies = () => {
    discovery.push([...primed]);
    return {
      size: () => dependencies.length,
      get: index => dependencies[index],
      delete() {},
    };
  };
  return {primed, fetched, discovery, run: url => context.prefetchModelAssets(url)};
}

for (const [disposition, filename] of [
  ['attachment; filename="tetra.obj"', 'tetra.obj'],
  ['inline; filename=tetra.obj', 'tetra.obj'],
  ['attachment; FILENAME="mesh with spaces.obj"', 'mesh with spaces.obj'],
  [null, ''],
]) {
  test(`prefetch preserves asset filename ${JSON.stringify(disposition)}`, async () => {
    const headers = disposition === null ? {} : {'content-disposition': disposition};
    const p = prefetchPage(new Map([
      [rootUrl, new Response('<mujoco/>')],
      [assetUrl, new Response(new Uint8Array([0, 255, 127]), {headers})],
    ]));
    const stats = await p.run(rootUrl);
    assert.deepEqual(p.primed.find(entry => entry.url === assetUrl), {
      url: assetUrl, bytes: [0, 255, 127], filename,
    });
    assert.equal(stats.files, 2);
    assert.equal(stats.errors, 0);
    assert.deepEqual(p.fetched, [rootUrl, assetUrl]);
  });
}

test('root response filenames reach the cache before dependency discovery', async () => {
  const p = prefetchPage(new Map([
    [rootUrl, new Response('<mujoco/>', {
      headers: {'Content-Disposition': 'attachment; filename="scene.xml"'},
    })],
  ]), [rootUrl]);
  await p.run(rootUrl);
  assert.equal(p.primed.length, 1);
  assert.equal(p.primed[0].filename, 'scene.xml');
  assert.equal(p.discovery.length, 1);
  assert.equal(p.discovery[0][0].filename, 'scene.xml');
});

test('GitHub scheme rewriting keeps response filenames', async () => {
  const url = 'github:org/repo/main/model.xml';
  const httpUrl = 'https://raw.githubusercontent.com/org/repo/main/model.xml';
  const p = prefetchPage(new Map([
    [httpUrl, new Response('<mujoco/>', {
      headers: {'content-disposition': 'attachment; filename="model.xml"'},
    })],
  ]), [url]);
  await p.run(url);
  assert.equal(p.primed.length, 1);
  assert.equal(p.primed[0].url, httpUrl);
  assert.equal(p.primed[0].filename, 'model.xml');
  assert.deepEqual(p.fetched, [httpUrl]);
});

test('failed responses are not primed', async () => {
  const p = prefetchPage(new Map([
    [rootUrl, new Response('<mujoco/>')],
    [assetUrl, new Response('missing', {status: 404})],
  ]));
  const stats = await p.run(rootUrl);
  assert.equal(stats.files, 1);
  assert.equal(stats.errors, 1);
  assert.equal(p.primed.length, 1);
  assert.equal(p.primed[0].url, rootUrl);
});

test('unsupported schemes skip prefetching', async () => {
  const p = prefetchPage(new Map());
  const stats = await p.run('custom:model.xml');
  assert.equal(stats.files, 0);
  assert.deepEqual(p.fetched, []);
  assert.deepEqual(p.primed, []);
});
