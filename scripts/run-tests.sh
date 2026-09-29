#!/usr/bin/env bash
# Usage: run-tests.sh
# Tests are disabled by default; configure with -DBUILD_TESTS=ON to enable them.

cd "$(dirname "$0")/.." || exit 1

cmake --preset debug -DBUILD_TESTS=ON
cmake --build --preset debug
ctest --test-dir build/debug --output-on-failure
