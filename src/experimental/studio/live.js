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
      "assets/pbr_packed.filamat",
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
            requestAnimationFrame(() => {
              requestAnimationFrame(async () => {
                try {
                  await Module.loadUrl(modelUrl);
                } catch (error) {
                  console.error('Failed to load model from URL:', error);
                } finally {
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
