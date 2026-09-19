# motion/ — board traversal

The 1-to-5 loop: SLAM drives the base to the next board, the top camera's
AprilTags micro-correct it into the workspace, the base holds while the arms
play, then on to the next board.

This is the technical reference — architecture, bring-up, tolerances,
conventions. For current status, what's verified vs. not, and the TODO list,
see **`motion.md`** in this same folder.

```
BoardTraversal            the loop; owns everything below
├── SlamApproach          phase 1: slam.pose -> turn / cruise / turn
├── TopCameraTags         camera.head.jpeg -> tag centers in the base frame
│   └── board_pose        tag centers -> board pose -> parking error
├── TagAligner            phase 2: turn / drive / turn at centimetre scale
└── DriveBus              the only writer of drive.ctrl in this repo
```

## Before anything moves

1. **Stop `bbapps/nav/main.py`.** Its control loop publishes a zero twist every
   tick when idle (`nav/main.py:1265`), so two writers on `drive.ctrl` cancel
   each other and roughly half of every correction is lost.
2. **Check the camera mount** in `config.py` (`CameraMount`): measured height,
   45° pitch, forward offset. No bbapps config carries a `T_base_cam` for the
   head camera, so these are ours to get right, and every centimetre of
   extrinsic error becomes a centimetre of parking error.
3. **If depth/stereo was never calibrated on this robot**, `test_tags.py`
   will print a loud `WARNING: no depth calibration...` and fall back to an
   uncalibrated pinhole guess instead of crashing. That guess needs the same
   tuning as the camera mount — see the Troubleshooting entry below.
4. **Nothing to install by hand.** Every script below is run with `uv run` and
   carries its own PEP 723 dependency block at the top (same convention as
   every script in `bbapps/`), so `uv` resolves `pupil-apriltags`, opencv and
   numpy into an ephemeral env on first run. No `pip install` needed.

## Bring-up order

Each command below is run with `uv run <path>`, from the repo root (each
script pins its own dependencies + a local `bbos` path, matching `bbapps/`'s
convention -- see e.g. `bbapps/nav/main.py`'s header block).

```bash
# 0. hardware-free: signs, geometry, controller convergence (needs only numpy)
uv run motion/test_geometry.py

# 1. camera + tags, no motion. Hold a tag at a measured distance and check
#    `dist` in the output; that validates CameraMount and the intrinsics.
uv run motion/test_tags.py

# 2. teach the waypoints. Drive with `uv run bbapps/teleop.py` in another
#    terminal -- this script only reads slam.pose, so there is no writer conflict.
uv run motion/tools/record_waypoints.py

# 3. live tag errors for one board
uv run motion/test_tags.py --board 1

# 4. phase 2 alone, logging only: park roughly in front of board 1 first
uv run motion/test_alignment.py --board 1 --dry-run

# 5. phase 2 live, hand on the e-stop
uv run motion/test_alignment.py --board 1

# 6. one board, both phases
uv run motion/test_traversal.py --board 1

# 7. the whole table
uv run motion/test_traversal.py --cycles 1
```

Run everything from the repo root (each script inserts the repo root onto
`sys.path` itself, so `from motion.x import y` resolves regardless of how it's
invoked). `motion/test_geometry.py` needs no `bbos`, so `python -m
motion.test_geometry` works too if you want to run it off-robot.

## Using it

```python
from motion.board_traversal import BoardTraversal

def play(board, park):
    # The base is holding position here. park.lat_err / park.range_err /
    # park.yaw_err are the measured residuals -- command the arms in board
    # coordinates using them rather than assuming a perfectly centered base.
    ...

with BoardTraversal() as bt:
    bt.run(play, cycles=None)      # serpentine 1->5, 5->1, ... until Ctrl-C
```

## Tolerances, and why lateral is the loose one

Defaults: **±1.5cm lateral, ±2cm range, ±1.5° yaw**, confirmed over 5
consecutive frames with the wheels stopped.

Range and yaw are cheap — the base drives and pivots along those axes directly.
Lateral is not: a differential drive cannot strafe, so every lateral correction
is a pivot-drive-pivot shuffle. 1.5cm is where `test_geometry.py`'s noise sweep
stops hunting; at 1.0cm the loop starts missing its settle window once
measurement noise reaches ~5mm, without actually parking any more accurately.
The measured residual is returned in `ParkResult`, so the arms can absorb it in
board coordinates — more accurate and much faster than shuffling the base.

## Conventions worth knowing before editing

- **SLAM yaw is not standard.** bbapps' forward vector is `(-sin, cos)` — the
  robot faces +Y at yaw 0 (`nav/planner.py:77`, `nav/main.py:1189`). That
  convention is contained in `geom.world_to_body()` and used only by
  `slam_nav.py`.
- **The base frame is standard**: x forward, y left, z up. All tag geometry is
  in it.
- **`camera.head.jpeg` is a side-by-side stereo frame.** We detect on the left
  half, because `camera_cal()`'s left camera matrix is what matches it.
- Error signs, everywhere: `range_err > 0` = too far back, `lat_err > 0` = base
  is to the board's +x side (its own right), `yaw_err > 0` = base must turn
  left.

Re-run `uv run motion/test_geometry.py` after touching any sign, gain or frame
convention. It catches a correction that pushes the base toward the table
before any wheel turns.

## Troubleshooting

Real issues hit bringing this up on `bracketbot-0185` — see `motion.md` for
the full status/TODO, this is just the quick reference.

| Symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: No module named 'motion'` under `uv run` | Running a `.py` file directly (rather than `python -m`) doesn't put the repo root on `sys.path` the way `-m` does | Already fixed — every entry script inserts the repo root itself. If you still see this, your checkout predates that fix; `git pull` / check you're on `motion-uv-run-fixes` or later. |
| `ps aux \| grep -E "nav/main.py\|teleop.py"` shows a hit, with a different PID every time | Classic self-match: `grep`'s own command line contains your search pattern, so it matches itself in the process list | Use `pgrep -fa "nav/main.py\|teleop.py"` instead — it excludes its own PID by design. If that prints nothing, nothing is actually running. |
| `grep -l "daemon.py slam" /proc/*/cmdline` only matches `/proc/self/cmdline` / `/proc/thread-self/cmdline` | Same self-match, one level deeper: those two are magic symlinks that always resolve to whoever's currently reading them (i.e. `grep` itself) | `pgrep -fa "daemon.py slam"` — no self-match. |
| `SlamUnavailable: no slam.pose within...` | No SLAM daemon running on this robot (confirmed via the `pgrep` check above + a full `ls /dev/shm/` showing every other daemon but no `slam*` topic) | Outside this repo's scope — `p_slam` lives in `bbos`. See `motion.md`'s "Known hardware gaps" for what this blocks and the fallback options. |
| `FileNotFoundError: .../stereo_calibration_fisheye.yaml` from `Config("depth").camera_cal()` | Depth/stereo was never calibrated on this robot | Fixed — `head_eye_intrinsics()` now catches this and falls back to an uncalibrated pinhole guess, with a loud warning. Tune it: hold a tag at a measured distance, compare `dist=`, pin `HEAD_INTRINSICS_OVERRIDE` in `config.py`. |
| `tag layout fit is NNmm RMS (limit 30mm)` | Wrong `tag_span_m`, or a corner id typo in `boards.json` | Re-measure / re-teach that board. |
| `solved normal points away from the robot` | near/far corners swapped in `boards.json` | Swap `near_left`↔`far_left` and `near_right`↔`far_right` for that board. |
| Correction pushes the *wrong* way | Camera extrinsic or a sign is wrong | Stop, re-run `test_geometry.py`, redo the camera-mount distance check. |
| Alignment hunts, never settles | Tag detection noisier than the tolerance | Loosen `AlignParams.lat_tol_m`/`range_tol_m` in `config.py`. |
| Twists seem to get randomly zeroed | Another writer on `drive.ctrl` | Confirm `nav/main.py`/`teleop.py` aren't running (via `pgrep`, not `ps \| grep`). |
