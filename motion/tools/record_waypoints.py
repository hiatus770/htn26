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
"""Teach the five board waypoints instead of typing them.

Drive the robot with bbapps/teleop.py in another terminal (this script only
READS slam.pose, so there is no writer conflict), park it where you want the
base to sit for each board, and press Enter. If the top camera can see the
board's four tags it captures those too, corner by corner.

    uv run motion/tools/record_waypoints.py              # boards 1..5
    uv run motion/tools/record_waypoints.py --boards 3   # re-teach board 3 only
    uv run motion/tools/record_waypoints.py --no-tags    # waypoints only

Hand-guessed SLAM coordinates next to a table full of chess pieces are how you
dent a table.
"""
import sys
from pathlib import Path

# repo root, for `motion.*` -- this file is two levels below it (motion/tools/)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import argparse
import math
import os

from motion.config import BOARDS_FILE, BoardSpec, BoardTable
from motion.slam_nav import SlamApproach


def capture_tags(camera, tries=30):
    """Detected tag ids assigned to corners, plus the measured span.

    Assignment assumes the robot is roughly square to the board (which it is,
    since you just parked it there): near corners are the two closest to the
    base, and +y is the robot's left.
    """
    best = None
    for _ in range(tries):
        dets, _ = camera.detect(block=True, timeout=1.0)
        if dets and len(dets) >= 4:
            best = dets
            break
        if dets and (best is None or len(dets) > len(best)):
            best = dets
    if not best or len(best) < 4:
        return None, None, best
    by_x = sorted(best, key=lambda d: d.p_base[0])
    near, far = by_x[:2], by_x[2:4]
    near = sorted(near, key=lambda d: d.p_base[1], reverse=True)   # +y = left
    far = sorted(far, key=lambda d: d.p_base[1], reverse=True)
    tags = {"near_left": near[0].tag_id, "near_right": near[1].tag_id,
            "far_left": far[0].tag_id, "far_right": far[1].tag_id}
    span_near = math.dist(near[0].p_base[:2], near[1].p_base[:2])
    span_side = math.dist(near[0].p_base[:2], far[0].p_base[:2])
    return tags, (span_near + span_side) / 2.0, best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boards", type=int, nargs="*", default=[1, 2, 3, 4, 5])
    ap.add_argument("--no-tags", action="store_true", help="skip tag capture")
    ap.add_argument("--tag-size", type=float, default=0.05,
                    help="printed side of the black tag square, m (default 0.05)")
    ap.add_argument("--standoff", type=float, default=0.35,
                    help="parked distance from board center to base, m")
    ap.add_argument("--out", default=BOARDS_FILE)
    args = ap.parse_args()

    table = BoardTable.load(args.out) if os.path.exists(args.out) else BoardTable()
    table.tag_size_m = args.tag_size
    existing = {b.index: b for b in table.boards}

    camera = None
    if not args.no_tags:
        from motion.tags import TopCameraTags
        camera = TopCameraTags(tag_size_m=args.tag_size)

    from contextlib import ExitStack
    with ExitStack() as stack:
        nav = stack.enter_context(SlamApproach(drive=None))
        if camera is not None:
            stack.enter_context(camera)
        print("Waiting for slam.pose ...")
        nav.wait_for_pose(timeout=15.0)
        print("SLAM is live. Drive with bbapps/teleop.py; this script never "
              "writes drive.ctrl.\n")

        for index in args.boards:
            input(f"--- Park the robot at BOARD {index}, then press Enter "
                  f"(Ctrl-C to abort) ---")
            nav.wait_for_pose()
            pose = nav.pose
            print(f"    pose: x={pose.x:+.3f} y={pose.y:+.3f} "
                  f"yaw={math.degrees(pose.yaw):+.1f}deg")

            spec = existing.get(index)
            tags = spec.tags if spec else None
            span = spec.tag_span_m if spec else 0.3390
            if camera is not None:
                got, measured, raw = capture_tags(camera)
                if got is None:
                    n = len(raw) if raw else 0
                    print(f"    !! saw {n} tag(s), need 4 -- keeping "
                          f"{'previous' if tags else 'no'} tag ids for this board")
                else:
                    print(f"    tags: {got}  measured span {measured * 100:.1f}cm")
                    if input("    accept these tags? [Y/n] ").strip().lower() in ("", "y"):
                        tags, span = got, round(measured, 4)
            if not tags:
                raw = input("    enter tag ids near_left,near_right,far_right,far_left "
                            "(blank to skip board): ").strip()
                if not raw:
                    print("    skipped")
                    continue
                parts = [int(v) for v in raw.replace(" ", "").split(",")]
                tags = dict(zip(("near_left", "near_right", "far_right", "far_left"), parts))

            existing[index] = BoardSpec(
                index=index, name=f"board-{index}",
                waypoint=(pose.x, pose.y, pose.yaw), tags=tags, tag_span_m=span,
                standoff_m=(spec.standoff_m if spec else args.standoff),
                lateral_offset_m=(spec.lateral_offset_m if spec else 0.0))
            print(f"    recorded board {index}")

    table.boards = [existing[i] for i in sorted(existing)]
    table.validate()
    table.save(args.out)
    print(f"\nWrote {len(table.boards)} boards -> {args.out}")
    print("Check it, then: uv run motion/test_tags.py --board 1")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\naborted -- nothing written")
        sys.exit(1)
