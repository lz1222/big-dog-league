#!/usr/bin/env bash
set -euo pipefail

PROFILE="${1:-safe}"

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -- -j"$(nproc)"
./build/go2_inspection_runner --config config/competition.conf --profile "${PROFILE}"

