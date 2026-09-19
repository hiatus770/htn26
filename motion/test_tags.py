# /// script
# dependencies = [
#   "bbos",
#   "numpy",
#   "opencv-python-headless",
#   "pupil-apriltags",
# ]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/bbos", editable = true }
# ///
"""Top camera / AprilTag check. Reads only -- the wheels never move.

    uv run motion/test_tags.py                 # every tag the top camera sees
    uv run motion/test_tags.py --board 1       # board 1's parking error, live

This is also the calibration check for the camera mount in motion/config.py:
hold a tag at a measured distance and compare `dist` in the output. If it is
consistently off, fix CameraMount (height/pitch) before trusting any alignment
run; every centimetre of extrinsic error becomes a centimetre of parking error.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # repo root, for `motion.*`

import argparse
import math
import time

from motion.config import BoardTable, TOP_CAMERA
from motion.tags import TagError, TopCameraTags, board_observation


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", type=int, default=None, help="also fit this board's layout")
    ap.add_argument("--hz", type=float, default=5.0, help="print rate (default 5)")
    ap.add_argument("--tag-size", type=float, default=None,
                    help="override the tag size from boards.json, m")
    args = ap.parse_args()

    board, table = None, None
    try:
        table = BoardTable.load()
        if args.board is not None:
            board = table.get(args.board)
    except FileNotFoundError as e:
        if args.board is not None:
            raise
        print(f"[note] {e}")

    tag_size = args.tag_size or (table.tag_size_m if table else 0.05)
    print(f"[test_tags] tag size {tag_size * 100:.1f}cm, mount {TOP_CAMERA}")
    if board:
        print(f"[test_tags] board {board.index} layout: {board.tag_layout()}")

    period = 1.0 / max(args.hz, 0.1)
    last = 0.0
    with TopCameraTags(tag_size_m=tag_size) as cam:
        while True:
            dets, stamp = cam.detect(block=True, timeout=2.0)
            if dets is None:
                print("[test_tags] no frame on camera.head.jpeg -- is the camera "
                      "daemon running?")
                continue
            now = time.time()
            if now - last < period:
                continue
            last = now
            if not dets:
                print("[test_tags] no tags in view")
                continue
            for d in sorted(dets, key=lambda d: d.tag_id):
                x, y, z = d.p_base
                print(f"  tag {d.tag_id:>3}  base x={x:+.3f} y={y:+.3f} z={z:+.3f}  "
                      f"dist={math.hypot(x, y):.3f}m  margin={d.margin:.0f}")
            if board is not None:
                # Fit the frame we already have; calling observe() here would
                # go looking for a second, newer frame and usually find none.
                try:
                    obs = board_observation(board, {d.tag_id: d.p_base for d in dets},
                                            stamp)
                except TagError as e:
                    print(f"  !! {e}")
                    continue
                if obs is None:
                    print(f"  >> board {board.index}: none of its tags in view")
                    continue
                print(f"  >> {obs}")
                print("     (lat + = base is right of target, range + = too far back, "
                      "yaw + = base must turn LEFT)")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
