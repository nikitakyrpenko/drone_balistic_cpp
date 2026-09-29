#!/usr/bin/env bash
# Usage: run-formatter.sh [path]
#   path  (optional) subdirectory to format; defaults to the whole repo

cd "$(dirname "$0")/.." || exit 1

target="${1:-.}"

find "$target" -type f -regex '.*\.\(cpp\|hpp\)' -not -path './build/*' -not -path './external/*' \
    -exec clang-format --style=file:.clang-format -i {} +
find "$target" -type f -name 'CMakeLists.txt' -not -path './build/*' -not -path './external/*' \
    -exec cmake-format -c .cmake-format.json -i {} +
