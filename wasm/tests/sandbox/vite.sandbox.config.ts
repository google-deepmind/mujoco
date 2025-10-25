import { defineConfig } from "vite"

export default defineConfig({
  root: "tests/sandbox", 
  base: "./", 
  build: {
    outDir: "../../sandbox-dist",
    emptyOutDir: true,
  },
  server: {
    open: true,
  },
})
