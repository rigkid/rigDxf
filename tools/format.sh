#!/usr/bin/env bash
# Format first-party src/ and examples/ (never third_party/).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if ! command -v clang-format >/dev/null 2>&1; then
  echo "FAIL: clang-format not on PATH. See README Style." >&2
  exit 1
fi
mapfile -t files < <(find src examples -type f \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) 2>/dev/null | grep -v '/third_party/' | grep -v '/build/' || true)
if [[ ${#files[@]} -eq 0 ]]; then
  echo "No C/C++ under src/ or examples/."
  exit 0
fi
echo "clang-format ($(clang-format --version | head -1)) on ${#files[@]} files..."
clang-format -i "${files[@]}"
echo "Done."
