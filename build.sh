#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
jobs=${CMAKE_BUILD_PARALLEL_LEVEL:-4}
case "$jobs" in ''|*[!0-9]*|0) echo "CMAKE_BUILD_PARALLEL_LEVEL must be a positive integer" >&2; exit 2;; esac
# DBoW2/g2o are normal CMake targets; Sophus is header-only.
cmake -S "$root" -B "$root/build" -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build "$root/build" --parallel "$jobs"
if [ ! -f "$root/Vocabulary/ORBvoc.txt" ]; then
    tar -xf "$root/Vocabulary/ORBvoc.txt.tar.gz" -C "$root/Vocabulary"
fi
