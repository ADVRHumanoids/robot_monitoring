import { fileURLToPath, URL } from 'node:url'
import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

export default defineConfig({
  plugins: [vue()],
  // Production lives below /monitor/. Using the same base in development
  // catches accidental root-relative asset URLs before deployment.
  base: '/monitor/',
  publicDir: fileURLToPath(new URL('../proto', import.meta.url)),
  build: {
    outDir: fileURLToPath(
      new URL('../server/src/xbot2_gui_server/monitor', import.meta.url),
    ),
    emptyOutDir: true,
  },
  server: {
    proxy: {
      '/version': 'http://localhost:8080',
      '/process': 'http://localhost:8080',
      '/plugin': 'http://localhost:8080',
      '/joint_states': 'http://localhost:8080',
      '/ws': { target: 'ws://localhost:8080', ws: true },
    },
  },
})
