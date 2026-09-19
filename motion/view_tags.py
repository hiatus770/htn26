# /// script
# dependencies = [
#   "bbos",
#   "numpy",
#   "opencv-python-headless",
#   "pupil-apriltags",
#   "fastapi",
#   "uvicorn",
# ]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/bbos", editable = true }
# ///
"""Live annotated camera feed for calibrating the top camera / AprilTags.

Draws each detected tag's outline, ID, base-frame distance and detector
margin directly on the video and serves it as an MJPEG stream in a browser --
much easier to calibrate against than squinting at scrolling terminal numbers
while also holding a tape measure.

    uv run motion/view_tags.py                 # every tag, default port 8022
    uv run motion/view_tags.py --board 1       # also overlay board 1's fit
    uv run motion/view_tags.py --port 8030

Then open http://<robot-hostname>.local:8022/ in a browser (or
http://localhost:8022/ on the robot itself).

Read-only: this never touches drive.ctrl, so it's safe to leave running
alongside anything else, including a live alignment run.

If the page shows a red "NO DEPTH CALIBRATION" banner, `dist=` is a guess --
hold a tag at a measured distance, compare against the number on screen, and
set HEAD_INTRINSICS_OVERRIDE in motion/config.py once it tracks. See
motion/README.md's Troubleshooting section for the full story.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # repo root, for `motion.*`

import argparse
import asyncio
import math
import socket
import threading
import time

import cv2
import numpy as np
from fastapi import FastAPI, Response
from fastapi.responses import HTMLResponse, StreamingResponse
import uvicorn

from motion.config import BoardTable
from motion.tags import TagError, TopCameraTags, board_observation

GREEN = (0, 255, 0)
RED = (0, 0, 255)
YELLOW = (0, 255, 255)
CYAN = (255, 255, 0)

_lock = threading.Lock()
_latest_jpeg = [None]
_stop = threading.Event()


def draw_overlay(frame_bgr, dets, cam, board=None):
    img = frame_bgr.copy()

    for d in dets:
        pts = d.corners.astype(int).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], True, GREEN, 2)
        c = (int(d.center[0]), int(d.center[1]))
        cv2.circle(img, c, 4, RED, -1)
        dist = math.hypot(float(d.p_base[0]), float(d.p_base[1]))
        cv2.putText(img, f"id{d.tag_id} {dist:.3f}m m={d.margin:.0f}",
                    (c[0] + 8, c[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 1,
                    cv2.LINE_AA)

    y = 24
    intr = cam.intrinsics
    banner = f"fx={intr.fx:.0f} fy={intr.fy:.0f} cx={intr.cx:.0f} cy={intr.cy:.0f}"
    if not cam.calibrated:
        cv2.putText(img, "NO DEPTH CALIBRATION -- dist is an UNCALIBRATED GUESS",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2, cv2.LINE_AA)
        y += 26
        banner += "  [GUESS]"
    cv2.putText(img, banner, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, CYAN, 1,
                cv2.LINE_AA)
    y += 22
    cv2.putText(img, f"{len(dets)} tag(s)", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                CYAN, 1, cv2.LINE_AA)
    y += 24

    if board is not None:
        points = {d.tag_id: d.p_base for d in dets}
        obs, err = None, None
        try:
            # max_residual=999: never raise here -- a bad fit is exactly what
            # a calibration tool should show, in red, not hide behind a
            # swallowed exception.
            obs = board_observation(board, points, stamp=0.0, max_residual=999.0)
        except TagError as e:
            err = str(e)
        if err is not None:
            cv2.putText(img, f"board {board.index}: {err}"[:100], (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, RED, 1, cv2.LINE_AA)
        elif obs is None:
            cv2.putText(img, f"board {board.index}: fewer than 2 of its tags visible",
                        (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, RED, 1, cv2.LINE_AA)
        else:
            bad = obs.residual > 0.03
            cv2.putText(
                img,
                f"board {board.index}: lat={obs.lat_err * 100:+.1f}cm "
                f"range={obs.range_err * 100:+.1f}cm "
                f"yaw={math.degrees(obs.yaw_err):+.1f}deg fit={obs.residual * 1000:.0f}mm",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, RED if bad else YELLOW, 1,
                cv2.LINE_AA)
    return img


def capture_loop(cam, board, fps):
    period = 1.0 / fps
    while not _stop.is_set():
        t0 = time.time()
        try:
            dets, frame, _stamp = cam.detect_bgr(block=True, timeout=1.0)
        except TagError as e:
            print(f"[view_tags] {e}", flush=True)
            time.sleep(0.2)
            continue
        if frame is None:
            time.sleep(0.02)
            continue
        img = draw_overlay(frame, dets, cam, board=board)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with _lock:
                _latest_jpeg[0] = buf.tobytes()
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)


app = FastAPI()


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTMLResponse("""
<!doctype html><meta charset=utf-8>
<title>AprilTag calibration view</title>
<style>
  body { margin:0; padding:16px; background:#111; color:#eee; font-family:monospace; }
  img  { max-width:100%; display:block; margin-top:12px; border:1px solid #333; }
  p    { max-width:640px; }
</style>
<h2>Top camera / AprilTag calibration</h2>
<p>Hold a tag at a measured distance and compare the <code>id.. dist=..</code>
label against your tape measure. If it tracks within a centimetre or two,
you're calibrated; if not, tune <code>HEAD_INTRINSICS_OVERRIDE</code> in
<code>motion/config.py</code> and restart this script.</p>
<img src="/stream">
""")


@app.get("/stream")
async def stream():
    async def generate():
        last = None
        while True:
            with _lock:
                frame = _latest_jpeg[0]
            if frame is not None and frame is not last:
                last = frame
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            await asyncio.sleep(1 / 30)
    return StreamingResponse(generate(),
                             media_type="multipart/x-mixed-replace; boundary=frame",
                             headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.get("/frame")
async def frame():
    with _lock:
        f = _latest_jpeg[0]
    if f is None:
        return Response(status_code=503)
    return Response(content=f, media_type="image/jpeg")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", type=int, default=None, help="also overlay this board's fit")
    ap.add_argument("--tag-size", type=float, default=None,
                    help="override the tag size from boards.json, m")
    ap.add_argument("--fps", type=float, default=12.0,
                    help="detect+draw rate (default 12 -- this is heavier than a raw stream)")
    ap.add_argument("--port", type=int, default=8022)
    ap.add_argument("--host", default="0.0.0.0")
    args = ap.parse_args()

    board, table = None, None
    try:
        table = BoardTable.load()
        if args.board is not None:
            board = table.get(args.board)
    except FileNotFoundError as e:
        if args.board is not None:
            raise
        print(f"[view_tags] {e}", flush=True)

    tag_size = args.tag_size or (table.tag_size_m if table else 0.05)
    cam = TopCameraTags(tag_size_m=tag_size)
    cam.__enter__()

    t = threading.Thread(target=capture_loop, args=(cam, board, args.fps), daemon=True)
    t.start()

    host = socket.gethostname()
    print(f"[view_tags] tag size {tag_size * 100:.1f}cm"
          f"{f', board {board.index}' if board else ''}", flush=True)
    print(f"[view_tags] open http://{host}.local:{args.port}/ (or localhost on the robot)",
          flush=True)
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning",
                    access_log=False, timeout_graceful_shutdown=1)
    finally:
        _stop.set()
        t.join(timeout=2.0)
        cam.__exit__(None, None, None)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
