# Build outputs

- `production/` is the current clean Teensy production build.
- `legacy-pre-reorg/` preserves generated build variants from the old absolute
  workspace path. They are reference artifacts, not reusable CMake caches.

Create or refresh named variants with `west build -p always`; do not run Ninja
directly inside a legacy cache.
