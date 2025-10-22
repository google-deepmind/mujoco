import { defineConfig } from 'vite';
import { viteStaticCopy } from 'vite-plugin-static-copy';
import path from 'path';
import { createRequire } from 'module';

// Vite runs in an ES Module context, so `require` is not available by default.
// We can create a `require` function that works within an ESM file.
const require = createRequire(import.meta.url);

/**
 * A helper function to resolve the absolute path to a linked package's directory.
 * This makes the setup robust, regardless of where your project is located.
 * @param {string} packageName - The name of the package to resolve.
 * @returns {string | null} The absolute path to the package directory or null if not found.
 */
const resolvePackagePath = (packageName) => {
  try {
    // Use our custom `require` to find the entry point of the package.
    const packageMainFile = require.resolve(packageName);
    // We then get the directory containing that file.
    return path.dirname(packageMainFile);
  } catch (err) {
    console.error(`[vite.config.js] Error: Could not resolve path for package: "${packageName}". Make sure it is linked correctly with 'npm link ${packageName}'.`);
    return null;
  }
};

const mujocoJsPath = resolvePackagePath('mujoco-js');

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [
    // This plugin handles copying files that aren't directly imported in your JS.
    // It's essential for assets like .wasm files that are loaded at runtime.
    // We also add a check to ensure the plugin only runs if the package path was found.
    mujocoJsPath ? viteStaticCopy({
      targets: [
        {
          // Source: The .wasm file from your linked mujoco-js package.
          src: `${mujocoJsPath}/mujoco_wasm.wasm`,
          // Destination: The root of the build output folder (e.g., 'dist/').
          dest: '.'
        },
        {
          // It's also good practice to copy the source map for debugging purposes.
          src: `${mujocoJsPath}/mujoco_wasm.wasm.map`,
          dest: '.'
        }
      ]
    }) : null
  ].filter(Boolean), // This filters out any `null` plugins from the array.

  // --- CORRECTED THIS SECTION ---
  // This is crucial for `npm link` to work during development.
  server: {
    fs: {
      // When linking, we must explicitly allow both the project root AND the linked package's directory.
      allow: [
        // The Vite project root (current directory)
        '.',
        // The directory of the linked package
        mujocoJsPath
      ].filter(Boolean) // Filter out null path if package not found
    }
  }
});

