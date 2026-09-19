# /// script
# dependencies = [
#   "numpy",
# ]
# ///
"""Hardware-free check of the board geometry and the parking controller.

    uv run motion/test_geometry.py
    (or plain `python -m motion.test_geometry` from the repo root -- no bbos
    needed, so this one also runs fine off-robot on a dev machine)

Runs with nothing but numpy -- no robot, no camera, no bbos. It synthesizes
what the top camera would report from a known base pose, feeds it through the
real board_observation()/compute_twist() code, and flies the result in a
unicycle simulator from a grid of starting offsets.

Run this after touching any sign, gain or frame convention. It catches the one
class of bug that is genuinely dangerous here -- a correction that pushes the
base toward the table instead of away from it -- for free, before anything
moves. It does NOT model the median filter, tag dropouts or the base's sway;
those need test_alignment.py on the real robot.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # repo root, for `motion.*`

import numpy as np

from motion.board_pose import board_observation, parking_point
from motion.config import AlignParams, BoardSpec
from motion.geom import apply_floor
from motion.tag_align import AIM, YAW, TagAligner

BOARD = BoardSpec(index=0, name="sim", waypoint=(0.0, 0.0, 0.0),
                  tags={"near_left": 0, "near_right": 1, "far_right": 2, "far_left": 3},
                  tag_span_m=0.3390, standoff_m=0.35, lateral_offset_m=0.0)

# The simulation world IS the board frame: board center at the origin, +x to
# the robot's right, +y away from the robot. The square parking pose is then
# (lateral_offset, -standoff) with the base heading +y, i.e. yaw = pi/2.
SQUARE = (BOARD.lateral_offset_m, -BOARD.standoff_m, math.pi / 2)

DT = 1.0 / 30.0          # camera rate: the alignment loop runs per frame

_failures = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  ' + detail) if detail else ''}")
    if not ok:
        _failures.append(name)


def observe(rx, ry, rth, board=BOARD, only=None):
    """What the top camera would report with the base at (rx, ry, rth)."""
    h = (math.cos(rth), math.sin(rth))          # base +x (forward), world coords
    l = (-math.sin(rth), math.cos(rth))         # base +y (left)
    pts = {}
    for tag_id, (bx, by) in board.tag_layout().items():
        if only is not None and tag_id not in only:
            continue
        dx, dy = bx - rx, by - ry
        pts[tag_id] = np.array([dx * h[0] + dy * h[1], dx * l[0] + dy * l[1], 0.0])
    return board_observation(board, pts, stamp=0.0)


def test_signs():
    print("signs and magnitudes")
    o = observe(*SQUARE)
    check("square pose reads zero error", o.worst(AlignParams()) < 1e-6, str(o))

    o = observe(SQUARE[0], SQUARE[1] - 0.05, SQUARE[2])
    check("5cm too far back -> range_err +5cm", abs(o.range_err - 0.05) < 1e-6,
          f"range_err={o.range_err * 100:+.2f}cm")

    o = observe(SQUARE[0] + 0.04, SQUARE[1], SQUARE[2])
    check("4cm to the robot's right -> lat_err +4cm", abs(o.lat_err - 0.04) < 1e-6,
          f"lat_err={o.lat_err * 100:+.2f}cm")

    turned = math.radians(5.0)
    o = observe(SQUARE[0], SQUARE[1], SQUARE[2] + turned)
    check("turned 5deg left -> yaw_err -5deg (must turn right)",
          abs(o.yaw_err + turned) < 1e-6, f"yaw_err={math.degrees(o.yaw_err):+.2f}deg")


def test_parking_point():
    print("parking point reconstruction")
    worst = 0.0
    for rx, ry, dth in ((0.03, -0.30, 0.10), (-0.05, -0.42, -0.15), (0.0, -0.35, 0.3)):
        rth = math.pi / 2 + dth
        o = observe(SQUARE[0] + rx, ry, rth)
        gx, gy = parking_point(o.range_err, o.lat_err, o.yaw_err)
        # Truth: the square pose expressed in the base frame.
        dx, dy = SQUARE[0] - (SQUARE[0] + rx), SQUARE[1] - ry
        tx = dx * math.cos(rth) + dy * math.sin(rth)
        ty = -dx * math.sin(rth) + dy * math.cos(rth)
        worst = max(worst, math.hypot(gx - tx, gy - ty))
    check("p* from the three errors matches the true goal", worst < 1e-9,
          f"max error {worst * 1000:.3g}mm")


def test_partial_tags():
    print("three-tag fit")
    pose = (SQUARE[0] + 0.03, SQUARE[1] - 0.04, SQUARE[2] + 0.08)
    full = observe(*pose)
    part = observe(*pose, only={0, 1, 2})
    d = max(abs(full.lat_err - part.lat_err), abs(full.range_err - part.range_err),
            abs(full.yaw_err - part.yaw_err))
    check("dropping one tag barely moves the solution", d < 1e-9,
          f"max delta {d * 1000:.3g}mm/mrad")


def simulate(start, params, aligner, noise=None, rng=None):
    """Fly the controller, mirroring TagAligner.align's structure.

    `noise` is (sigma_m, sigma_rad) added to each measurement, standing in for
    the base's sway; it is filtered by the same rolling median the real loop
    uses. Returns (converged, steps, final observation, closest approach).
    """
    from collections import deque
    x, y, th = start
    mode = AIM
    settled = 0
    out_frames = 0
    window = deque(maxlen=params.median_window)
    closest = math.inf
    max_steps = int(params.timeout_s / DT)      # the real loop's own budget
    for step in range(max_steps):
        o = observe(x, y, th)
        closest = min(closest, math.hypot(x, y))     # base to board center
        m = (o.lat_err, o.range_err, o.yaw_err)
        if noise is not None:
            m = (m[0] + rng.gauss(0, noise[0]), m[1] + rng.gauss(0, noise[0]),
                 m[2] + rng.gauss(0, noise[1]))
        window.append(m)
        if len(window) < min(3, window.maxlen):
            continue
        lat, rng_e, yaw = (float(v) for v in np.median(np.asarray(window), axis=0))
        if max(abs(lat) / params.lat_tol_m, abs(rng_e) / params.range_tol_m,
               abs(yaw) / params.yaw_tol_rad) <= 1.0:
            settled += 1
            if settled >= params.settle_frames:
                return True, step, o, closest
            continue           # the real loop holds still while it confirms
        out_frames = out_frames + 1 if mode == YAW else 0
        settled = 0
        v, w, mode = aligner.compute_twist(
            lat, rng_e, yaw, mode, allow_exit_yaw=out_frames >= params.exit_yaw_frames)
        v = apply_floor(v, params.v_min)       # what DriveBus does to the command
        w = apply_floor(w, params.omega_min)
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        th += w * DT
    return False, max_steps, observe(x, y, th), closest


CASES = [(dlat, drange, dyaw)
         for dlat in (-0.10, -0.03, 0.0, 0.03, 0.10)
         for drange in (-0.08, 0.0, 0.08)
         for dyaw in (-8.0, 0.0, 8.0)]

# The SLAM leg hands over within goal_tol of the waypoint, so these offsets
# cover what phase 2 actually has to absorb, with margin.
MIN_CLEARANCE = BOARD.standoff_m - 0.15


def run_cases(params, aligner, noise=None, seed=7):
    import random
    rng = random.Random(seed)
    worst_steps, closest_all, failed = 0, math.inf, []
    for dlat, drange, dyaw_deg in CASES:
        start = (SQUARE[0] + dlat, SQUARE[1] - drange,
                 SQUARE[2] + math.radians(dyaw_deg))
        ok, steps, o, closest = simulate(start, params, aligner, noise, rng)
        worst_steps = max(worst_steps, steps)
        closest_all = min(closest_all, closest)
        if not ok:
            failed.append((dlat, drange, dyaw_deg, str(o)))
    return worst_steps, closest_all, failed


def report(label, params, worst_steps, closest, failed):
    check(f"{label}: all {len(CASES)} starting offsets park within tolerance",
          not failed, f"slowest {worst_steps * DT:.1f}s (timeout "
                      f"{params.timeout_s:.0f}s), closest approach "
                      f"{closest * 100:.0f}cm")
    for f in failed[:5]:
        print(f"        lat{f[0] * 100:+.0f}cm range{f[1] * 100:+.0f}cm "
              f"yaw{f[2]:+.0f}deg -> {f[3]}")


def test_convergence():
    print("closed-loop convergence (unicycle sim)")
    params = AlignParams()
    aligner = TagAligner(drive=None, camera=None, params=params)

    worst_steps, closest, failed = run_cases(params, aligner)
    report("clean", params, worst_steps, closest, failed)
    check("clean: converges inside the configured timeout",
          worst_steps * DT < params.timeout_s)
    # Safety: the correction must never carry the base at the table.
    check("clean: never closes to within 15cm of the standoff",
          closest > MIN_CLEARANCE, f"closest {closest * 100:.0f}cm, "
                                   f"limit {MIN_CLEARANCE * 100:.0f}cm")

    # Sway: 5mm / 0.5deg of measurement noise per frame, median-filtered the
    # same way the real loop filters it.
    worst_steps, closest, failed = run_cases(params, aligner,
                                             noise=(0.005, math.radians(0.5)))
    report("with sway", params, worst_steps, closest, failed)
    check("with sway: never closes to within 15cm of the standoff",
          closest > MIN_CLEARANCE, f"closest {closest * 100:.0f}cm")


def test_gain_conditions():
    print("controller sanity conditions")
    p = AlignParams()
    check("positive gains", p.k_rho > 0 and p.k_alpha > 0 and p.k_yaw_polish > 0)
    check("aim hysteresis: exit threshold above entry",
          p.aim_exit_rad > p.aim_tol_rad,
          f"{math.degrees(p.aim_tol_rad):.0f}deg -> {math.degrees(p.aim_exit_rad):.0f}deg")
    check("a pivot at omega_pivot out-steers the floor",
          p.omega_pivot > p.omega_min, f"pivot={p.omega_pivot} floor={p.omega_min}")
    # The stiction floor must not be able to carry the base across a tolerance
    # band in one frame, or the loop hunts instead of settling.
    step = p.v_min / 30.0
    check("one floored step stays inside the lateral tolerance", step < p.lat_tol_m,
          f"{step * 1000:.1f}mm/frame vs {p.lat_tol_m * 1000:.0f}mm")


def main():
    for t in (test_signs, test_parking_point, test_partial_tags,
              test_gain_conditions, test_convergence):
        t()
    print()
    if _failures:
        print(f"{len(_failures)} FAILED: {', '.join(_failures)}")
        return 1
    print("all geometry checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
