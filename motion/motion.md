# motion/ — scope, status, TODO

This file tracks *where things stand* and *what's left*. For how the code
actually works (architecture, bring-up ladder, tolerances, frame
conventions), see `motion/README.md` — that one is the technical reference,
this one is the living status doc. Keep both current; this one changes
every session, that one changes when the design changes.

## Scope

`motion/` owns exactly the base-parking pipeline:

1. Drive from wherever the robot is to roughly in front of the next board (SLAM).
2. Micro-correct with the top camera's AprilTags until the board is centered.
3. Hold position and hand off.

**Not in scope here** (owned elsewhere, or not yet built anywhere):
- Obstacle avoidance / path planning around things in the lane — the five
  boards sit in a line with short, straight hops; if something blocks the
  path, `SlamApproach` aborts with a stuck-timeout rather than routing around
  it.
- Reading the board / choosing a chess move — `vision/` and `engine/`.
- Arm motion / piece placement — receives `ParkResult` (the residual parking
  error) from the `on_board(board, park)` handoff and is expected to
  compensate for it, not assume a perfectly centered base.

## Status (as of 2026-09-19)

Code for all three phases plus the orchestrator (`BoardTraversal`) is
written and committed on branch **`motion-uv-run-fixes`** — **not yet merged
to `master`**. Two commits on top of `55357ed` (the version currently on
robot `bracketbot-0185`... check `git log` before assuming any robot is
up to date).

**Verified:**
- `uv run motion/test_geometry.py` (hardware-free: signs, gains, closed-loop
  convergence under simulated sway) — all checks passing.
- Live camera + AprilTag detection on `bracketbot-0185` (`uv run
  motion/test_tags.py`) — working, after fixing the two hardware gaps below.
  Tags 10/11/12 detected at healthy margins (44-58); tag 13 not seen yet in
  the same session (likely just out of frame). Frame-to-frame distance jitter
  of a few cm on a static tag is expected right now — some mix of the
  segway's continuous balancing sway and the still-uncalibrated intrinsics
  guess, not a bug.

**New tool:** `uv run motion/view_tags.py` — live MJPEG feed (browser,
`:8022`) with detected tags outlined and labeled with id/distance/margin, plus
an on-screen red banner when running on the uncalibrated intrinsics fallback.
Built specifically to make the `HEAD_INTRINSICS_OVERRIDE` tuning loop (hold a
tag at a measured distance, compare, adjust) visual instead of
terminal-scrollback-based. `--board N` overlays that board's live lat/range/yaw
fit too.

**Not yet verified live** (either blocked or just not run yet):
- `test_alignment.py` — phase 2 micro-correction on a real board.
- `record_waypoints.py` / phase 1 (`SlamApproach`) — blocked, no SLAM (below).
- `test_traversal.py` — the full loop, single board or the whole table.
- The real `on_board(board, park)` handoff to `vision`/`engine` — every test
  script so far uses a placeholder that just prints.

## Known hardware gaps on `bracketbot-0185`

These are real robot state, not bugs in this code — recorded here so nobody
re-diagnoses them from scratch.

1. **No SLAM daemon running.** `ls /dev/shm/` shows a full, live daemon stack
   (`camera.*`, `drive.state`, `imu.*`, `arm_left/right.state`, `mic.*`,
   `quest.*`, `led.state`, `speaker.log`, `usb.*`) but zero `slam*` topics.
   `pgrep -fa "daemon.py slam"` confirms no such process (a naive `grep -l
   ... /proc/*/cmdline` will falsely "find" it via the `/proc/self` and
   `/proc/thread-self` self-referencing entries — `pgrep` doesn't have that
   problem, it excludes itself by design).
   - **Blocks:** `record_waypoints.py` (needs `slam.pose` to capture each
     board's waypoint), phase 1 of `test_traversal.py` (`SlamApproach`).
   - **Does not block:** `test_tags.py`, `test_alignment.py` — neither reads
     SLAM.
   - **Open question, not this repo's to answer:** was SLAM/`p_slam` ever
     configured for this rig? The daemon lives in `bbos` (external to this
     checkout), so bringing it up — if it's even supposed to exist here — is
     outside `motion/`'s scope.

2. **No depth/stereo calibration.** `Config("depth").camera_cal()` had no
   `stereo_calibration_fisheye.yaml` to load and raised `FileNotFoundError`
   straight out of `bbos`. Fixed in `motion/tags.py::head_eye_intrinsics()`:
   it now catches that and falls back to an uncalibrated pinhole guess
   (`HEAD_HFOV_DEG_FALLBACK` in `config.py`), logging a loud warning instead
   of crashing.
   - **Still needs empirical tuning**: hold a tag at a measured distance,
     compare against `dist=` in `test_tags.py`'s output, and pin
     `HEAD_INTRINSICS_OVERRIDE = (fx, fy, cx, cy)` in `config.py` once it
     tracks. Every centimetre of error here is a centimetre of parking error.

## TODO, in order

- [ ] Tune `HEAD_INTRINSICS_OVERRIDE` against a measured tag distance —
      use `uv run motion/view_tags.py` for live visual feedback while doing
      this.
- [ ] Confirm/measure `CameraMount` in `config.py` (height/pitch/forward) —
      `test_tags.py`'s `dist=` check is also the validation for this.
- [ ] Decide how to handle the missing SLAM daemon (see options below) —
      this is the one blocking real teaching + autonomous traversal.
- [ ] Hand-write a `motion/boards.json` for at least one board (copy the
      shape in `boards.example.json`, fill in real tag IDs) so
      `test_tags.py --board N` / `test_alignment.py --board N` can be tested
      without waiting on SLAM — neither needs the `waypoint` field to be
      real.
- [ ] Run `test_alignment.py --board N --dry-run`, check signs, then live —
      first real-hardware test of the parking controller (only validated in
      simulation so far).
- [ ] Once SLAM is resolved: `record_waypoints.py` for all 5 boards, then
      `test_traversal.py --board N` per board before the full
      `--cycles 1`.
- [ ] Merge `motion-uv-run-fixes` into `master` once the above is confirmed
      working on hardware.
- [ ] Replace the placeholder `on_board(board, park)` (currently just prints,
      see `test_traversal.py`) with the real handoff into `vision`/`engine`.

## If SLAM stays unavailable

Not yet decided — options on the table:
- **Manual reposition, autonomous alignment.** An operator drives/pushes the
  robot roughly into place between boards; only phase 2 (AprilTag
  micro-correction) runs autonomously. Demonstrates the precision half of the
  pipeline without needing SLAM at all — zero code changes needed, just skip
  `SlamApproach`/`BoardTraversal` and call `TagAligner.align()` directly per
  board (see what `test_alignment.py` already does).
- **Wheel-odometry dead reckoning** as a `SlamApproach` substitute. Rougher,
  drifts over a full traversal, but needs no `p_slam`. Not implemented —
  would mean swapping `slam_nav.py`'s pose source for one integrated from
  `drive.state`, and is a real (if small) chunk of new work.
