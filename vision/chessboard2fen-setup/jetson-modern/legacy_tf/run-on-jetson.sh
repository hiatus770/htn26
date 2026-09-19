#!/usr/bin/env bash
set -euo pipefail

# Usage: ./run-on-jetson.sh /path/to/chessboard2fen /path/to/image-or-directory
assets_dir="${1:?legacy chessboard2fen checkout required}"
input_path="${2:?image or directory required}"
docker run --rm --runtime nvidia \
  -v "$assets_dir:/assets:ro" \
  -v "$input_path:/input:ro" \
  chessboard2fen-jetson:legacy-tf \
  --assets /assets --input /input
