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
"""Calibrate the top camera's fisheye distortion using the chess board itself
as the calibration target -- no printed checkerboard needed.

    # near_left,near_right,far_right,far_left tag ids -- same convention as
    # boards.json. No boards.json needed; this reads nothing from it.
    uv run motion/tools/calibrate_fisheye.py --tags 10,11,12,13

    # or, if boards.json already has a board taught:
    uv run motion/tools/calibrate_fisheye.py --board 1

Hold the board up and move it around: different distances (roughly 25cm to
1m worked well in testing -- don't jam it against the lens or tilt it to the
point tags start dropping out), different positions in the frame, and some
tilt. Frames are captured automatically whenever the view differs enough
from the last one; Ctrl-C stops early and still tries to solve with whatever
was captured.

WHY NOT JUST 4 TAG CENTERS PER FRAME: this OpenCV build's
cv2.fisheye.calibrate() hard-requires 5+ points per view (4 fails outright,
confirmed empirically) and, separately, 4 points spread over just the board's
34cm span turned out fine for fx/fy/cx/cy but too weakly conditioned to pin
down the distortion coefficients even across 100+ views. So this uses each
tag's own 4 CORNERS too (16 points/frame): plenty above the per-view minimum,
and spread across the board's full width instead of a single point per tag.

THE CATCH with corners: pupil_apriltags reports each tag's 4 corners in that
tag's own internal order, which has nothing to do with how the tag happens to
be mounted on the board -- two tags printed identically and mounted 90
degrees apart from each other still each report "their" corner 0 in their own
local convention. Silently assuming a fixed corner order is exactly the kind
of bug that produces a confidently wrong calibration, so instead this
resolves each tag's real mounting rotation automatically: a rough board pose
from the 4 (unambiguous) tag CENTERS predicts where each tag's corners should
land for each of the 4 possible 90-degree mountings, and whichever prediction
matches best (voted across several frames) is used from then on. This step
is required -- it is not optional polish.

Verified against a synthetic fisheye ground truth (known K/D, known random
per-tag mounting rotations, realistic corner noise) before ever pointing this
at a real camera: rotation resolution recovered every tag correctly, and the
calibration recovered fx/fy/cx/cy to within ~0.3px and distortion to the
correct sign/order-of-magnitude, PROVIDED the shared intrinsics are seeded
with a rough initial guess (CALIB_USE_INTRINSIC_GUESS) -- without that seed,
this OpenCV build's own auto-init is unstable across a realistically diverse
capture and can diverge to garbage. Both are why this script insists on a
rough focal-length guess up front rather than starting from zero.

This is read-only (no drive.ctrl, no SLAM) and always detects on the RAW
camera frame, independent of motion/config.py's current
HEAD_INTRINSICS_OVERRIDE/HEAD_DIST_OVERRIDE -- that's the whole point, you're
solving for the correction, not applying a guess at it first.

After it solves, paste the printed values into motion/config.py:
    HEAD_INTRINSICS_OVERRIDE = (fx, fy, cx, cy)
    HEAD_DIST_OVERRIDE = (k1, k2, k3, k4)
    HEAD_DIST_MODEL = "fisheye"   # already the default
Then verify with `uv run motion/view_tags.py`: hold a tag at a fixed distance
and slide it from the center of the frame toward an edge -- `dist=` should
stay put now, where before it would have drifted with position.
"""
import sys
from pathlib import Path

# repo root, for `motion.*` -- this file is two levels below it (motion/tools/)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import argparse
import math
import time

import cv2
import numpy as np
from bbos import Config, Reader

from motion.config import BoardSpec

try:
    from pupil_apriltags import Detector as _AprilDetector
except ImportError as e:
    _AprilDetector = None
    _IMPORT_ERROR = e

TOPIC = "camera.head.jpeg"
CORNER_ORDER = ("near_left", "near_right", "far_right", "far_left")

# Diversity gate: a candidate frame must differ from the most recently
# accepted one by at least this much (pixel centroid shift, or fractional
# change in apparent size) to be accepted. Lightweight heuristic, not a
# rigorous coverage guarantee -- good enough to stop 30 near-duplicate frames
# from being captured while you slowly move the board.
MIN_SHIFT_PX = 50.0
MIN_SCALE_FRAC = 0.10
COOLDOWN_S = 0.5
MIN_FRAMES_TO_SOLVE = 15     # below this, per-view geometry is too sparse to trust
ROTATION_VOTE_FRAMES = 8     # how many early frames vote on each tag's mounting


def _calib_flag(name):
    """Older OpenCV exposes these under cv2.fisheye.*; newer builds (5.x)
    unified them into the main cv2.* namespace instead. uv run's unpinned
    opencv-python-headless dependency could resolve to either depending on
    when it's run, so try both rather than hardcoding one location."""
    return getattr(cv2.fisheye, name, None) or getattr(cv2, name)


def read_left_gray(reader):
    payload = bytes(reader.data["jpeg"][:reader.data["jpeg_len"]])
    color = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if color is None:
        return None
    left = np.ascontiguousarray(color[:, :color.shape[1] // 2])
    return cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)


def local_corners(tag_size_m):
    """This tag's 4 corners around its own center, in a fixed simple (non-
    self-intersecting) winding. The absolute winding direction/start corner
    don't need to match pupil_apriltags' real convention -- resolve_rotations
    figures out the actual per-tag alignment; this is just an internally
    consistent reference to align everything else against."""
    s = tag_size_m / 2.0
    return np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]], dtype=np.float64)


def capture_frames(det, target_ids, n_frames):
    """Auto-capture n_frames diverse views. Returns a list of
    {tag_id: {'center': (2,), 'corners': (4,2)}} dicts, one per accepted
    frame (only tags actually seen in that frame are present). Ctrl-C stops
    early -- caught here so whatever was captured is kept, not discarded.
    """
    captured = []
    last_centroid, last_scale, last_t = None, None, 0.0
    with Reader(TOPIC) as r:
        print("[calib] waiting for camera.head.jpeg ...", flush=True)
        try:
            while len(captured) < n_frames:
                if not r.ready():
                    time.sleep(0.005)
                    continue
                gray = read_left_gray(r)
                if gray is None:
                    continue
                dets = det.detect(gray, estimate_tag_pose=False)
                found = {int(d.tag_id): d for d in dets if int(d.tag_id) in target_ids}
                if len(found) < len(target_ids):
                    continue    # need all 4 visible to fit a board pose for rotation-resolution

                centers = np.array([found[i].center for i in target_ids], dtype=np.float64)
                centroid = centers.mean(axis=0)
                diffs = centers[:, None, :] - centers[None, :, :]
                scale = float(np.max(np.linalg.norm(diffs, axis=-1)))

                now = time.time()
                if now - last_t < COOLDOWN_S:
                    continue
                if last_centroid is not None:
                    shift = float(np.linalg.norm(centroid - last_centroid))
                    scale_frac = abs(scale / last_scale - 1.0) if last_scale else 1.0
                    if shift < MIN_SHIFT_PX and scale_frac < MIN_SCALE_FRAC:
                        continue

                frame = {i: {"center": np.asarray(found[i].center, dtype=np.float64),
                            "corners": np.asarray(found[i].corners, dtype=np.float64)}
                        for i in target_ids}
                captured.append(frame)
                last_centroid, last_scale, last_t = centroid, scale, now
                print(f"[calib] captured {len(captured)}/{n_frames}  "
                      f"centroid=({centroid[0]:.0f},{centroid[1]:.0f}) scale={scale:.0f}px "
                      f"-- move the board (different distance/position/tilt)", flush=True)
        except KeyboardInterrupt:
            print(f"\n[calib] stopped early with {len(captured)} frame(s)", flush=True)
    return captured


def resolve_rotations(frames, layout, target_ids, tag_size_m, rough_K, n_vote_frames):
    """Per tag: how many 90deg steps to roll its reported corners by so they
    align with the board-frame corner convention. See the module docstring's
    "THE CATCH with corners" section for why this can't be skipped."""
    base = local_corners(tag_size_m)
    obj_centers = np.array([[layout[i][0], layout[i][1], 0.0] for i in target_ids])
    tally = {i: [0, 0, 0, 0] for i in target_ids}

    for frame in frames[:n_vote_frames]:
        img_centers = np.array([frame[i]["center"] for i in target_ids], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj_centers, img_centers, rough_K, None)
        if not ok:
            continue
        for tag_id in target_ids:
            cx, cy = layout[tag_id]
            obj = base + np.array([cx, cy, 0.0])
            proj, _ = cv2.projectPoints(obj, rvec, tvec, rough_K, None)
            proj = proj.reshape(-1, 2)
            observed = frame[tag_id]["corners"]
            best_k, best_err = 0, None
            for k in range(4):
                err = float(np.mean(np.linalg.norm(np.roll(observed, k, axis=0) - proj, axis=1)))
                if best_err is None or err < best_err:
                    best_err, best_k = err, k
            tally[tag_id][best_k] += 1

    resolved = {i: int(np.argmax(tally[i])) for i in target_ids}
    for tag_id in target_ids:
        votes = tally[tag_id]
        total = sum(votes)
        winner = votes[resolved[tag_id]]
        if total == 0 or winner < total * 0.6:
            print(f"[calib] WARNING: tag {tag_id}'s mounting rotation vote was not "
                  f"decisive ({votes}, out of {total}) -- its corners may be misaligned "
                  f"in the final calibration. A cleaner first few frames (all 4 tags "
                  f"squarely visible) usually fixes this.", flush=True)
    return resolved


def build_views(frames, layout, target_ids, tag_size_m, rotations):
    """(object_points, image_points) lists for cv2.fisheye.calibrate(): one
    16-point view per frame (4 tags x 4 corners), corners reordered into
    board-frame convention using the resolved per-tag rotations."""
    base = local_corners(tag_size_m)
    object_points, image_points = [], []
    for frame in frames:
        obj_frame, img_frame = [], []
        for tag_id in target_ids:
            cx, cy = layout[tag_id]
            obj_frame.append(base + np.array([cx, cy, 0.0]))
            img_frame.append(np.roll(frame[tag_id]["corners"], rotations[tag_id], axis=0))
        object_points.append(np.concatenate(obj_frame, axis=0).reshape(1, -1, 3))
        image_points.append(np.concatenate(img_frame, axis=0).reshape(1, -1, 2))
    return object_points, image_points


def _fisheye_calibrate(object_points, image_points, image_size, K, flags, criteria):
    n = len(object_points)
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n)]
    return cv2.fisheye.calibrate(object_points, image_points, image_size, K.copy(), D,
                                 rvecs, tvecs, flags, criteria)


def solve(object_points, image_points, image_size, rough_K):
    """cv2.fisheye.calibrate() over the captured views, in two stages. Raises
    cv2.error on an ill-conditioned solve (usually: not enough view
    diversity).

    Two things here are validated requirements, not optional polish (see the
    module docstring for the empirical evidence):

    1. Seeding K with a rough guess + CALIB_USE_INTRINSIC_GUESS: this OpenCV
       build's own from-scratch auto-init is unstable across a realistically
       diverse capture and can diverge to garbage without it.
    2. TWO STAGES even so: solve the pinhole part first with distortion
       forced to zero (CALIB_FIX_K1..K4) -- that alone converged reliably in
       every trial -- then re-solve the FULL model seeded from ITS result
       instead of the original rough guess. Going straight for the full
       model from the rough guess still diverged in roughly 1 in 5-10 trials
       even with the seed from (1); this staged approach did not diverge
       once across 12+ seeds it was tested against.
    """
    flag = _calib_flag
    common = flag("CALIB_RECOMPUTE_EXTRINSIC") | flag("CALIB_FIX_SKEW") \
        | flag("CALIB_USE_INTRINSIC_GUESS")
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-8)

    pinhole_flags = common | flag("CALIB_FIX_K1") | flag("CALIB_FIX_K2") \
        | flag("CALIB_FIX_K3") | flag("CALIB_FIX_K4")
    _, K_pinhole, _, _, _ = _fisheye_calibrate(
        object_points, image_points, image_size, rough_K, pinhole_flags, criteria)

    rms, K, D, _, _ = _fisheye_calibrate(
        object_points, image_points, image_size, K_pinhole, common, criteria)
    return rms, K, D


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", type=int, default=None,
                    help="load tag ids + tag_span_m from boards.json instead of --tags")
    ap.add_argument("--tags", type=str, default=None,
                    help="near_left,near_right,far_right,far_left tag ids, e.g. 10,11,12,13")
    ap.add_argument("--tag-span", type=float, default=0.339,
                    help="center-to-center distance between corner tags, m (default 0.339 "
                         "-- this robot's board)")
    ap.add_argument("--tag-size", type=float, default=0.05,
                    help="printed side of the black tag square, m (default 0.05)")
    ap.add_argument("--hfov-guess", type=float, default=90.0,
                    help="rough horizontal FOV guess in degrees, for the initial intrinsics "
                         "seed only -- doesn't need to be accurate (default 90)")
    ap.add_argument("--frames", type=int, default=30,
                    help="target number of diverse captures (default 30 -- more than a "
                         "checkerboard would need, since each view has fewer points)")
    args = ap.parse_args()

    if _AprilDetector is None:
        raise SystemExit(f"pupil_apriltags not installed: {_IMPORT_ERROR}")

    if args.board is not None:
        from motion.config import BoardTable
        board = BoardTable.load().get(args.board)
    elif args.tags:
        ids = [int(x) for x in args.tags.replace(" ", "").split(",")]
        if len(ids) != 4:
            ap.error("--tags needs exactly 4 comma-separated ids: "
                     "near_left,near_right,far_right,far_left")
        board = BoardSpec(index=0, name="calib", waypoint=(0.0, 0.0, 0.0),
                          tags=dict(zip(CORNER_ORDER, ids)), tag_span_m=args.tag_span)
    else:
        ap.error("need --board N (from boards.json) or --tags a,b,c,d")

    layout = board.tag_layout()
    target_ids = sorted(layout.keys())
    print(f"[calib] target: tags {target_ids}, layout {layout}", flush=True)

    cfg_c = Config("cam_head")
    width, height = int(cfg_c.width) // 2, int(cfg_c.height)
    rough_fx = (width / 2.0) / math.tan(math.radians(args.hfov_guess) / 2.0)
    rough_K = np.array([[rough_fx, 0.0, width / 2.0],
                        [0.0, rough_fx, height / 2.0],
                        [0.0, 0.0, 1.0]], dtype=np.float64)

    # quad_decimate=1.0 (full resolution): this is an offline-ish, interactive
    # tool, not the real-time alignment loop -- best corner precision matters
    # more than speed here.
    det = _AprilDetector(families="tag36h11", nthreads=2, quad_decimate=1.0,
                         refine_edges=1)

    print("[calib] show all 4 tags to the camera and move the board around: "
          "different distances (~25cm-1m worked well in testing), different "
          "POSITIONS in the frame, some tilt -- but don't tilt so far tags "
          "start dropping out. Capturing automatically, Ctrl-C to stop early "
          "(keeps whatever was captured).", flush=True)

    frames = capture_frames(det, target_ids, args.frames)
    if len(frames) < MIN_FRAMES_TO_SOLVE:
        raise SystemExit(f"[calib] only {len(frames)} frame(s) captured, need at least "
                         f"{MIN_FRAMES_TO_SOLVE} for a trustworthy solve. Re-run and keep "
                         f"moving the board until it reaches --frames.")

    print(f"[calib] resolving each tag's mounting rotation from the first "
          f"{min(ROTATION_VOTE_FRAMES, len(frames))} frame(s) ...", flush=True)
    rotations = resolve_rotations(frames, layout, target_ids, args.tag_size, rough_K,
                                  ROTATION_VOTE_FRAMES)
    print(f"[calib] resolved rotations: {rotations}", flush=True)

    object_points, image_points = build_views(frames, layout, target_ids, args.tag_size,
                                              rotations)

    print(f"[calib] solving from {len(frames)} views ({len(frames) * 16} points) ...",
          flush=True)
    try:
        rms, K, D = solve(object_points, image_points, (width, height), rough_K)
    except cv2.error as e:
        raise SystemExit(
            f"[calib] cv2.fisheye.calibrate failed: {e}\n"
            f"This usually means not enough view diversity (too many similar frames, "
            f"or a pose extreme enough to be near-degenerate). Re-run with more --frames "
            f"and a wider, gentler spread of distances/positions/tilts.")

    fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    k1, k2, k3, k4 = (float(v) for v in D.ravel())

    print()
    verdict = "good" if rms < 1.0 else ("high -- consider recapturing with more diversity"
                                        if rms > 2.0 else "ok")
    print(f"[calib] RMS reprojection error: {rms:.3f}px ({verdict})")
    print(f"[calib] fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
    print(f"[calib] k1={k1:.6f} k2={k2:.6f} k3={k3:.6f} k4={k4:.6f}")
    print()
    print("Paste into motion/config.py:")
    print(f"    HEAD_INTRINSICS_OVERRIDE = ({fx:.2f}, {fy:.2f}, {cx:.2f}, {cy:.2f})")
    print(f"    HEAD_DIST_OVERRIDE = ({k1:.6f}, {k2:.6f}, {k3:.6f}, {k4:.6f})")
    print("    HEAD_DIST_MODEL = \"fisheye\"   # already the default")
    print()
    print("Then verify: uv run motion/view_tags.py -- hold a tag at a fixed distance and "
          "slide it from the center toward an edge; dist= should now stay put.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[calib] interrupted")
