#!/usr/bin/env bash
# Usage: run-tidy.sh [path]
#   path  (optional) subdirectory to analyze; defaults to all drone_* modules and src

cd "$(dirname "$0")/.." || exit 1

if [ -z "$1" ]; then
    output=$(run-clang-tidy -p build/debug drone_* src 2>&1)
else
    output=$(run-clang-tidy -p build/debug "$1" 2>&1)
fi

errors=$(echo "$output" | grep "error:")
if [ -n "$errors" ]; then
    echo "$errors"
    exit 1
fi
exit 0
