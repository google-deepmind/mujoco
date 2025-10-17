import { defineConfig } from "vite"

export default defineConfig({
  root: "demo_app",
  base: "./",
  build: {
    outDir: "../demo-dist",
    emptyOutDir: true,
  },
  server: {
    open: true,
  },
})
