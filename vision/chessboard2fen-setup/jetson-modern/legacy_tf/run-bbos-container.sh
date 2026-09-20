#!/usr/bin/env bash
set -euo pipefail

app_dir="${1:-/home/bracketbot/bbapps/chessvision}"
camera="${2:-head}"
frames="${3:-1}"

# BBOS camera frames live in host shared memory. BBOS also creates a small
# time-log handle there while opening a Reader, so /dev/shm must be shared
# read/write for IPC bookkeeping. The application and model files remain
# read-only; this container has no device-control interfaces or robot writer.
exec sudo docker run --rm --runtime nvidia --ipc=host --pid=host --network none \
  -v /dev/shm:/dev/shm \
  -v /home/bracketbot/bbos:/opt/bbos:ro \
  -v "$app_dir:/app:ro" \
  -e PYTHONPATH=/opt/bbos \
  chessvision:tf217 \
  /app/app/run_bbos.py --assets /app --camera "$camera" --frames "$frames"
