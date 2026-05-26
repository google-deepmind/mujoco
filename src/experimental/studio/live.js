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

// --- Loading overlay (shown while model is compiling) ---
const loadingOverlay = document.getElementById('loadingOverlay');
function showLoading() {
  loadingOverlay.style.display = 'flex';
}
function hideLoading() {
  loadingOverlay.style.display = 'none';
}

// ---------------------------------------------------------------------------
// Parallel asset prefetcher.
//
// MuJoCo's XML compiler walks the model and synchronously opens every
// referenced mesh / texture / include via mjpResourceProvider. In WASM that
// becomes a chain of ASYNCIFY-suspended fetch() calls — strictly serial,
// one RTT per asset. For Menagerie models that's ~80 round trips and ~10 s
// of avoidable wall time on a cold cache.
//
// This module mirrors MuJoCo's resource-discovery rules in JavaScript
// (xml_native_reader.cc, xml.cc): it fetches the root XML, recursively
// follows <include>s, walks all file-bearing tags, resolves each path
// against <compiler assetdir|meshdir|texturedir>, and feeds the bytes
// into the WASM-side FetchCache via Module.primeFetchCache(). When
// LoadUrl subsequently asks for any of those URLs, the resource provider
// returns immediately from memory.
//
// Keep the scheme list and the file-bearing tags in sync with
// src/experimental/studio/emscripten.cc and src/xml/xml_native_reader.cc.
// ---------------------------------------------------------------------------

// MuJoCo Live custom URL schemes that the C++ resource providers rewrite
// before fetching. JS must do the same so primed cache keys match what
// the provider eventually asks for.
function resolveScheme(url) {
  if (url.startsWith('github:')) {
    return 'https://raw.githubusercontent.com/' + url.slice('github:'.length);
  }
  return url;
}

// Resolve `file` against a directory URL (one that ends with '/'). Handles
// '..' segments and absolute URLs via the WHATWG URL parser.
function resolveAgainst(file, baseDir) {
  try {
    return new URL(file, baseDir).href;
  } catch {
    return baseDir + file;  // fallback for unrecognised base schemes
  }
}

const ASSET_TAGS = [
  // tag       => attribute(s) + which compiler-dir its files resolve under.
  { tag: 'mesh',     dir: 'mesh',    attrs: ['file'] },
  { tag: 'hfield',   dir: 'asset',   attrs: ['file'] },
  { tag: 'skin',     dir: 'asset',   attrs: ['file'] },
  { tag: 'texture',  dir: 'texture', attrs: ['file', 'fileright', 'fileleft',
                                              'fileup', 'filedown',
                                              'filefront', 'fileback'] },
  // <flexcomp file=...> and inline <model file=...> resolve against the
  // *model* file directory, not assetdir.
  { tag: 'flexcomp', dir: null,      attrs: ['file'] },
  { tag: 'model',    dir: null,      attrs: ['file'] },
];

async function prefetchModelAssets(rootUrl, onProgress) {
  const visited = new Set();   // dedupe by resolved http(s) URL
  const stats = { files: 0, bytes: 0, errors: 0 };

  // Fetch a single URL and prime it into the WASM-side cache.
  // Returns the decoded text iff `decodeText` is true (used for XMLs);
  // otherwise returns null. Idempotent on the visited set.
  async function fetchAndPrime(httpUrl, decodeText) {
    if (visited.has(httpUrl)) return null;
    visited.add(httpUrl);
    try {
      const response = await fetch(httpUrl);
      if (!response.ok) {
        console.warn('[prefetch] HTTP', response.status, httpUrl);
        stats.errors++;
        return null;
      }
      const bytes = new Uint8Array(await response.arrayBuffer());
      Module.primeFetchCache(httpUrl, bytes);
      stats.files++;
      stats.bytes += bytes.length;
      onProgress?.(stats);
      return decodeText ? new TextDecoder().decode(bytes) : null;
    } catch (err) {
      console.warn('[prefetch] error', httpUrl, err);
      stats.errors++;
      return null;
    }
  }

  // Resolve the effective {asset,mesh,texture} directory URLs for one
  // XML document, given the dirs inherited from its parent (if any).
  // Empty <compiler> attributes inherit from the parent; missing ones
  // fall back to the corresponding assetdir (matching xml_native_reader.cc).
  function compilerDirs(doc, baseDir, inherited) {
    const compiler = doc.getElementsByTagName('compiler')[0];
    const attr = (name) => {
      const v = compiler?.getAttribute(name);
      return v == null ? null : (v.endsWith('/') ? v : v + '/');
    };
    const asset = attr('assetdir');
    const mesh = attr('meshdir');
    const tex = attr('texturedir');
    const assetDir = asset !== null ? resolveAgainst(asset, baseDir)
                                    : inherited.asset ?? baseDir;
    return {
      asset: assetDir,
      mesh:    mesh !== null ? resolveAgainst(mesh, baseDir)
                             : inherited.mesh    ?? assetDir,
      texture: tex  !== null ? resolveAgainst(tex,  baseDir)
                             : inherited.texture ?? assetDir,
    };
  }

  // Walk an already-fetched XML: schedule every referenced asset fetch in
  // parallel, then recurse into <include>s. The included XML inherits the
  // current dirs.
  async function walk(httpUrl, xmlText, inheritedDirs) {
    const doc = new DOMParser().parseFromString(xmlText, 'application/xml');
    if (doc.documentElement.tagName === 'parsererror') {
      console.warn('[prefetch] XML parse failed:', httpUrl);
      return;
    }
    const baseDir = httpUrl.slice(0, httpUrl.lastIndexOf('/') + 1);
    const dirs = compilerDirs(doc, baseDir, inheritedDirs);

    const pending = [];
    for (const { tag, dir, attrs } of ASSET_TAGS) {
      const base = dir ? dirs[dir] : baseDir;
      for (const el of doc.getElementsByTagName(tag)) {
        for (const a of attrs) {
          const file = el.getAttribute(a);
          if (file) pending.push(fetchAndPrime(resolveAgainst(file, base), false));
        }
      }
    }
    for (const el of doc.getElementsByTagName('include')) {
      const file = el.getAttribute('file');
      if (!file) continue;
      const child = resolveAgainst(file, baseDir);
      // Fetch + walk in parallel with the binary fetches above.
      pending.push((async () => {
        const text = await fetchAndPrime(child, true);
        if (text) await walk(child, text, dirs);
      })());
    }
    await Promise.all(pending);
  }

  const rootHttp = resolveScheme(rootUrl);
  const rootXml = await fetchAndPrime(rootHttp, true);
  if (rootXml) {
    await walk(rootHttp, rootXml,
               { asset: null, mesh: null, texture: null });
  }
  return stats;
}

var Module = {
  preRun: [],
  postRun: [],
  locateFile: function (path) {
    const baseURL = window.location.origin + window.location.pathname.substring(0, window.location.pathname.lastIndexOf("/"));
    return baseURL + "/bin/" + path;
  },
  print: console.log,
  printErr: text => {
    console.error(text + "\n" + new Error().stack);
  },
  canvas: (() => {
    const canvas = document.getElementById("canvas");
    // As a default initial behavior, pop up an alert when webgl context is lost. To make your
    // application robust, you may want to override this behavior before shipping!
    // See http://www.khronos.org/registry/webgl/specs/latest/1.0/#5.15.2
    canvas.addEventListener(
      "webglcontextlost",
      e => {
        alert("WebGL context lost. You will need to reload the page.");
        e.preventDefault();
      },
      false,
    );
    return canvas;
  })(),
  setStatus(text) { },
  totalDependencies: 0,
  monitorRunDependencies(left) { },
  onRuntimeInitialized: () => {
    // Define assets to prefetch, relative to the WASM directory.
    const assetsToPrefetch = [
      "assets/fontawesome-webfont.ttf",
      "assets/ibl.ktx",
      "assets/OpenSans-Regular.ttf",
      "assets/pbr.filamat",
      "assets/pbr_transparent.filamat",
      "assets/pbr_packed.filamat",
      "assets/pbr_packed_transparent.filamat",
      "assets/phong_2d_fade.filamat",
      "assets/phong_2d.filamat",
      "assets/phong_2d_reflect.filamat",
      "assets/phong_2d_uv_fade.filamat",
      "assets/phong_2d_uv.filamat",
      "assets/phong_2d_uv_reflect.filamat",
      "assets/phong_color_fade.filamat",
      "assets/phong_color.filamat",
      "assets/phong_color_reflect.filamat",
      "assets/phong_cube_fade.filamat",
      "assets/phong_cube.filamat",
      "assets/phong_cube_reflect.filamat",
      "assets/unlit_decor.filamat",
      "assets/unlit_depth.filamat",
      "assets/unlit_segmentation.filamat",
      "assets/unlit_ui.filamat"
    ];

    const assetPromises = assetsToPrefetch.map(async (relativePath) => {
      const assetUrl = Module.locateFile(relativePath);
      try {
        const response = await fetch(assetUrl);
        if (!response.ok) {
          throw new Error(`Failed to fetch ${assetUrl}: ${response.statusText}`);
        }
        const buffer = await response.arrayBuffer();
        const filename = relativePath.substring(relativePath.lastIndexOf('/') + 1);
        Module.registerAsset(filename, new Uint8Array(buffer));
        console.log(`Registered asset: ${filename}`);
      } catch (error) {
        console.error(`Error prefetching asset ${assetUrl}:`, error);
        throw error;  // Re-throw to be caught by Promise.all
      }
    });

    Promise.all(assetPromises)
      .then(() => {
        try {
          const prefersDark = !(window.matchMedia &&
              window.matchMedia('(prefers-color-scheme: light)').matches);
          Module.init("MuJoCo Live", prefersDark);

          // Check for a ?model= URL parameter and load from URL.
          const params = new URLSearchParams(window.location.search);
          const modelUrl = params.get('model');
          if (modelUrl) {
            // loadUrl uses ASYNCIFY (via EM_ASYNC_JS fetch), which
            // suspends the WASM module. We must not start the animation
            // loop until the load completes, otherwise renderFrame will
            // hit "Cannot have multiple async operations in flight".
            showLoading();
            // Update the loading screen's "Downloading asset file" line with
            // live progress from the JS prefetcher.
            const dlEl = document.getElementById('loadingDownload');
            const dlFileEl = document.getElementById('loadingDownloadFile');
            if (dlEl) dlEl.style.display = 'block';
            const onProgress = (s) => {
              if (dlFileEl) {
                dlFileEl.textContent =
                  `${s.files} files · ${(s.bytes/1024/1024).toFixed(1)} MB`;
              }
            };
            requestAnimationFrame(() => {
              requestAnimationFrame(async () => {
                try {
                  await prefetchModelAssets(modelUrl, onProgress);
                  await Module.loadUrl(modelUrl);
                } catch (error) {
                  console.error('Failed to load model from URL:', error);
                } finally {
                  if (dlEl) dlEl.style.display = 'none';
                  hideLoading();
                  requestAnimationFrame(Module.animate);
                }
              });
            });
          } else {
            requestAnimationFrame(Module.animate);
          }
        } catch (error) {
          console.error('Failed to initialize app.', error);
        }
      })
      .catch((error) => {
        console.error('Failed to prefetch one or more assets.', error);
      });
  },
  animate: async () => {
    try {
      await Module.renderFrame();
    } catch (error) {
      console.error('Update error:', error);
    }
    requestAnimationFrame(Module.animate);
  },
};
// Ensure the canvas is resized when the window is resized.
window.addEventListener("resize", function () {
  Module.canvas.style.width = window.innerWidth + "px";
  Module.canvas.style.height = window.innerHeight + "px";
});

function handleFile(file) {
  const reader = new FileReader();
  reader.onload = (e) => {
    const buffer = e.target.result;
    // Show the loading overlay, then defer the blocking WASM call by two
    // animation frames so the browser has a chance to paint the overlay.
    showLoading();
    requestAnimationFrame(() => {
      requestAnimationFrame(() => {
        try {
          Module.loadFile(file.name, buffer);
        } catch (error) {
          console.error('Failed to load model from file:', error);
        } finally {
          hideLoading();
        }
      });
    });
  };
  reader.onerror = (e) => {
    console.error('Error reading file:', e);
  };
  reader.readAsArrayBuffer(file);
}

document.addEventListener('DOMContentLoaded', () => {
  // Remove commit hash if not substituted
  var el = document.getElementById('commitHash');
  if (el && el.textContent === '__COMMIT_HASH_PLACEHOLDER__') el.remove();

  // --- Drag-and-drop support ---
  const dropOverlay = document.getElementById('dropOverlay');

  let dragCounter = 0;

  document.addEventListener('dragenter', (e) => {
    e.preventDefault();
    dragCounter++;
    if (dragCounter === 1) {
      dropOverlay.style.display = 'flex';
    }
  });

  document.addEventListener('dragleave', (e) => {
    e.preventDefault();
    dragCounter--;
    if (dragCounter === 0) {
      dropOverlay.style.display = 'none';
    }
  });

  document.addEventListener('dragover', (e) => {
    e.preventDefault();
  });

  document.addEventListener('drop', (e) => {
    e.preventDefault();
    dragCounter = 0;
    dropOverlay.style.display = 'none';
    const files = e.dataTransfer.files;
    for (let i = 0; i < files.length; i++) {
      handleFile(files[i]);
    }
  });
});
