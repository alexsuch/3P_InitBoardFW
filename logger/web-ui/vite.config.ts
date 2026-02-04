// @ts-nocheck
import { defineConfig } from 'vite';
import preact from '@preact/preset-vite';
import tailwindcss from '@tailwindcss/vite';
import { compression } from 'vite-plugin-compression2';
import { viteSingleFile } from 'vite-plugin-singlefile';
import path from 'node:path';

export default defineConfig(({ mode }) => {
  const isSingleBundle = mode === 'single';

  const plugins = [
    preact(),
    tailwindcss(),
  ];

  // Single bundle mode: inline everything into one HTML file
  if (isSingleBundle) {
    plugins.push(viteSingleFile());
  }

  // Gzip compression
  plugins.push(
    compression({
      algorithms: ['gzip'],
      deleteOriginalAssets: true,
      include: isSingleBundle ? /\.html$/ : /\.(js|css|html|svg)$/,
    })
  );

  return {
    plugins,
    resolve: {
      alias: {
        '@': path.resolve(__dirname, './src'),
      },
    },
    build: {
      target: 'es2022',
      outDir: '../main/assets',
      emptyOutDir: true,
      ...(isSingleBundle && {
        cssCodeSplit: false,
        rollupOptions: {
          output: {
            manualChunks: undefined,
            inlineDynamicImports: true,
          },
        },
      }),
    },
    esbuild: {
      target: 'es2022',
    },
  };
});
