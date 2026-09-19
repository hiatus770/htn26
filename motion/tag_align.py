"""Phase 2: micro-correct the wheels until the board is centered on the workspace.

The tags give us the board's full pose in the base frame, so the parking pose
is known in robot coordinates and the correction is the same turn / drive /
turn that nav uses for a waypoint (bbapps/nav/main.py:1128-1135), just at
centimetre scale:

    AIM   pivot in place until the parking point is dead ahead (or dead astern)
    GO    drive to it, steering only on the bearing
    YAW   position is inside tolerance: pivot to square up with the board

The textbook polar parking controller (omega = k_alpha*alpha + k_beta*beta) was
tried first and rejected. In motion/test_geometry.py it failed to park from a
10cm lateral offset: the base ended up ~90deg off square, creeping. Its
pivot equilibrium sits at alpha = -(k_beta/k_alpha)*beta, which for a sideways
error is tens of degrees away from the goal, so with v clamped and the stiction
floor pushing every small command back up to 2cm/s it circles instead of
parking. Turn/drive/turn has no such equilibrium, and it never translates while
pointed obliquely at the table.

Two robot-specific details matter more than the gains:

  * The base is a segway and sways continuously, so every error is a rolling
    MEDIAN over the last few detections. An instantaneous tag pose is not the
    parked pose.
  * "In tolerance" is only believed after several consecutive frames with the
    wheels stopped. A single in-tolerance frame mid-sway is a false lock, and a
    false lock is an arm reaching for a square that is not there.

Losing sight of the tags stops the base immediately -- blind creeping toward a
table is exactly the failure this loop exists to prevent.
"""
import math
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

from motion.board_pose import TagError, parking_point
from motion.config import AlignParams
from motion.geom import clamp, wrap


# Correction phases. See the module docstring.
AIM, GO, YAW = "aim", "go", "yaw"


class AlignmentError(RuntimeError):
    """Base: alignment did not converge. The base is stopped on raise."""


class AlignmentFailed(AlignmentError):
    def __init__(self, msg, observation=None):
        super().__init__(msg)
        self.observation = observation


@dataclass
class ParkResult:
    board_index: int
    lat_err: float
    range_err: float
    yaw_err: float
    n_tags: int
    iterations: int
    elapsed_s: float

    def __str__(self):
        return (f"board {self.board_index} parked: lat={self.lat_err * 100:+.2f}cm "
                f"range={self.range_err * 100:+.2f}cm "
                f"yaw={math.degrees(self.yaw_err):+.2f}deg "
                f"({self.n_tags} tags, {self.elapsed_s:.1f}s)")


class TagAligner:
    def __init__(self, drive, camera, params=None, log=print):
        self.drive = drive
        self.camera = camera
        self.p = params or AlignParams()
        self.log = log
        self._window = deque(maxlen=max(1, self.p.median_window))
        self.last_observation = None

    # --- error filtering ---

    def _push(self, obs):
        self._window.append((obs.lat_err, obs.range_err, obs.yaw_err))

    def _filtered(self):
        """Component-wise median. Yaw stays linear on purpose: these are
        sub-degree errors around zero, never anywhere near the wrap point."""
        arr = np.asarray(self._window, dtype=np.float64)
        return tuple(float(v) for v in np.median(arr, axis=0))

    # --- control ---

    def compute_twist(self, lat_err, range_err, yaw_err, mode=AIM, allow_exit_yaw=True):
        """Errors -> (v, omega, next_mode). See module docstring.

        `allow_exit_yaw=False` pins the loop in the final yaw phase: the caller
        uses it to debounce the exit, so that a single noisy frame cannot
        restart a whole pivot-drive-pivot shuffle.

        Public and side-effect free so motion/test_geometry.py can fly it in a
        simulator before it ever moves a wheel.
        """
        p = self.p
        scale = p.hysteresis if mode == YAW else 1.0
        if (mode == YAW and not allow_exit_yaw) or (
                abs(lat_err) <= p.lat_tol_m * scale
                and abs(range_err) <= p.range_tol_m * scale):
            # Position is good. Turning in place is the only motion that cannot
            # make it worse, so square up with the board and nothing else.
            return 0.0, clamp(p.k_yaw_polish * yaw_err,
                              -p.omega_pivot, p.omega_pivot), YAW
        if mode == YAW:
            mode = AIM              # position drifted back out -- re-aim

        gx, gy = parking_point(range_err, lat_err, yaw_err)
        rho = math.hypot(gx, gy)
        if rho <= p.rho_min_m:
            # The bearing to a target a few millimetres away is mostly noise
            # (2mm of jitter on a 10mm target is +-11deg), and chasing it makes
            # the base pirouette. Below this radius, only the heading is worth
            # correcting. Keep rho_min_m under the position tolerances or the
            # loop can stop translating while still outside them.
            return 0.0, clamp(p.k_yaw_polish * yaw_err,
                              -p.omega_pivot, p.omega_pivot), YAW
        alpha = math.atan2(gy, gx)
        # A parking point behind us is reached in reverse. Looping around to
        # approach it forwards would mean swinging the base through the table.
        reverse = abs(alpha) > math.pi / 2.0
        alpha_c = wrap(alpha - math.pi) if reverse else alpha

        if mode == GO and abs(alpha_c) > p.aim_exit_rad:
            mode = AIM
        if mode == AIM:
            if abs(alpha_c) > p.aim_tol_rad:
                return 0.0, clamp(p.k_alpha * alpha_c,
                                  -p.omega_pivot, p.omega_pivot), AIM
            mode = GO

        v = clamp((-1.0 if reverse else 1.0) * p.k_rho * rho, -p.v_max, p.v_max)
        omega = clamp(p.k_alpha * alpha_c, -p.omega_max, p.omega_max)
        return v, omega, GO

    # --- the loop ---

    def align(self, board, timeout=None):
        """Micro-correct until the board is centered. Returns a ParkResult."""
        p = self.p
        timeout = timeout or p.timeout_s
        t0 = time.time()
        last_obs_t = 0.0
        settle = 0
        out_frames = 0
        mode = AIM
        iterations = 0
        warned_few_tags = False
        self._window.clear()
        self.last_observation = None
        self.drive.stop()          # never inherit a twist from the approach leg
        self.log(f"[align] board {board.index}: starting micro-correction "
                 f"(tol lat {p.lat_tol_m * 100:.1f}cm / range {p.range_tol_m * 100:.1f}cm "
                 f"/ yaw {math.degrees(p.yaw_tol_rad):.1f}deg)")

        try:
            while True:
                now = time.time()
                if now - t0 > timeout:
                    self.drive.stop()
                    raise AlignmentFailed(
                        f"board {board.index}: alignment did not converge in "
                        f"{timeout:.0f}s (last: {self.last_observation})",
                        self.last_observation)

                try:
                    obs = self.camera.observe(board, max_residual=p.max_fit_residual_m)
                except TagError as e:
                    self.drive.stop()
                    raise AlignmentFailed(f"board {board.index}: {e}",
                                          self.last_observation) from e

                if obs is None or len(obs.tag_ids) < p.min_tags:
                    if obs is not None and not warned_few_tags:
                        warned_few_tags = True
                        self.log(f"[align] board {board.index}: only "
                                 f"{len(obs.tag_ids)} tag(s) visible, need {p.min_tags}")
                    if last_obs_t and (now - last_obs_t) > p.det_stale_s:
                        # Lost the board mid-correction: stop moving, keep looking.
                        self.drive.stop()
                        self._window.clear()
                        settle = 0
                    time.sleep(0.002)
                    continue

                last_obs_t = obs.stamp
                self.last_observation = obs
                self._push(obs)
                iterations += 1
                if len(self._window) < min(3, self._window.maxlen):
                    self.drive.stop()      # not enough samples to trust yet
                    continue

                lat_err, range_err, yaw_err = self._filtered()
                worst = max(abs(lat_err) / p.lat_tol_m,
                            abs(range_err) / p.range_tol_m,
                            abs(yaw_err) / p.yaw_tol_rad)

                if worst <= 1.0:
                    # Hold still and re-measure: confirm it is parked, not
                    # just passing through tolerance on a sway.
                    self.drive.stop()
                    settle += 1
                    if settle >= p.settle_frames:
                        elapsed = time.time() - t0
                        result = ParkResult(board.index, lat_err, range_err, yaw_err,
                                            len(obs.tag_ids), iterations, elapsed)
                        self.log(f"[align] {result}")
                        return result
                    continue

                out_frames = out_frames + 1 if mode == YAW else 0
                settle = 0
                v, omega, mode = self.compute_twist(
                    lat_err, range_err, yaw_err, mode,
                    allow_exit_yaw=out_frames >= p.exit_yaw_frames)
                self.drive.send(v, omega)
                if iterations % 15 == 0:
                    self.log(f"[align] board {board.index}: lat={lat_err * 100:+.1f}cm "
                             f"range={range_err * 100:+.1f}cm "
                             f"yaw={math.degrees(yaw_err):+.2f}deg "
                             f"-> {mode} v={v:+.3f} w={omega:+.3f}")
        except Exception:
            self.drive.stop()
            raise
