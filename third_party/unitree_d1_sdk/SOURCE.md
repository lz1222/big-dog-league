# Unitree D1 SDK Source

Imported from GitHub:

- Repository: https://github.com/chen37058/Grasp-with-the-Unitree-D1
- Commit: `2374c3256741b2b98b5b47f47e76f4682e677448`
- Imported on: 2026-07-08

This snapshot contains the Ubuntu/CMake source files and generated DDS message
bindings used by the Unitree D1 examples. The upstream `build/` directory,
compiled binaries, `.git/` metadata, and large demo GIF files are intentionally
not imported.

Local integration note:

- `CMakeLists.txt` was adjusted to prefer this repository's
  `third_party/unitree_sdk2/install` path before falling back to system
  Unitree SDK2 installs such as `/usr/local`.
- The upstream repository did not include a license file in the cloned
  snapshot. Confirm redistribution/licensing with the team before publishing
  this third-party source outside the competition workspace.

