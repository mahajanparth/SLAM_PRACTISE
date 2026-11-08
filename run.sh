#!/usr/bin/env bash
# Usage: ./run.sh scratch/foo.cpp [-O]     (-O = optimized build, no sanitizers, for timing)
set -euo pipefail
src="${1:?usage: ./run.sh path/to/file.cpp [-O]}"
mode="${2:-}"
root="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$root/build/run"
out="$root/build/run/$(basename "${src%.cpp}")"
if [[ "$mode" == "-O" ]]; then
  flags=(-O2 -DNDEBUG)
else
  flags=(-O0 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
fi
g++ -std=c++17 -Wall -Wextra -Wpedantic "${flags[@]}" \
    -I"$root" -I/usr/include/eigen3 -pthread "$src" -o "$out"
echo "---- running $(basename "$out") ----"
if [[ "$mode" == "-O" ]]; then
  "$out"
else
  # GCC 11 ASan crashes randomly on kernel 6.x (high mmap_rnd_bits); disabling ASLR fixes it.
  setarch "$(uname -m)" -R "$out"
fi
