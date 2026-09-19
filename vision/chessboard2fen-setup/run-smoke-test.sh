#!/usr/bin/env bash
set -euo pipefail

setup_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="${1:-$HOME/Projects/chessboard2fen}"
if [[ ! -f "$repo_dir/detectionScript.py" ]]; then
  repo_dir="$(cd "$setup_dir/../../.." && pwd)/chessboard2fen"
fi

test -f "$repo_dir/detectionScript.py"
docker --context default run --rm \
  -v "$repo_dir:/repo:ro" \
  -v "$setup_dir/smoke_test.py:/opt/chessboard2fen-setup/smoke_test.py:ro" \
  chessboard2fen:tf2.2 \
  python /opt/chessboard2fen-setup/smoke_test.py
