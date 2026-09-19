"""Phase 1: drive to the general vicinity of a board using slam.pose.

Deliberately no Dijkstra and no occupancy grid. The five boards sit on one
table along a single lane, the hops between them are ~0.4m, and pulling in
bbapps' planner would mean numba + scipy + a live mapping daemon to plan a
straight line. Obstacle avoidance is not this loop's job -- if something is in
the lane, the stuck detector aborts and a human looks at it.

The controller is a three-state turn / cruise / turn, using the same constants
and the same constant-omega-with-a-tolerance-stop heading phase nav uses
(bbapps/nav/main.py:1128-1135).
"""
import math
import time
from dataclasses import dataclass
from enum import Enum

from bbos import Reader

from motion.config import ApproachParams
from motion.geom import clamp, quat_yaw, world_to_body, wrap


class ApproachError(RuntimeError):
    """Base: the robot did not reach the waypoint. Base is stopped on raise."""


class SlamUnavailable(ApproachError):
    pass


class ApproachFailed(ApproachError):
    pass


class Phase(Enum):
    TURN_TO_BEARING = "turn_to_bearing"
    CRUISE = "cruise"
    TURN_TO_PARK = "turn_to_park"
    DONE = "done"


@dataclass
class Pose:
    x: float
    y: float
    yaw: float
    stamp: float


class SlamApproach:
    """Drives the base to a taught (x, y, yaw) SLAM waypoint."""

    def __init__(self, drive=None, params=None, log=print):
        # drive=None is legal for pose-only use (see tools/record_waypoints.py);
        # any method that moves the base will fail loudly without one.
        self.drive = drive
        self.p = params or ApproachParams()
        self.log = log
        self._r = None
        self.pose = None

    def __enter__(self):
        self._r = Reader("slam.pose")
        self._r.__enter__()
        return self

    def __exit__(self, *exc):
        if self._r is not None:
            try:
                self._r.__exit__(None, None, None)
            except Exception:
                pass
            self._r = None
        return False

    # --- pose ---

    def poll(self):
        """Refresh self.pose if a new sample is up. Returns True if it changed."""
        if self._r is None:
            raise SlamUnavailable("SlamApproach used outside its context manager")
        if not self._r.ready():
            return False
        d = self._r.data
        # Raw pose, no smoothing -- same call nav makes, and for the same
        # reason: a loop closure or relock should snap immediately rather than
        # be dragged in by a filter (bbapps/nav/main.py:925).
        self.pose = Pose(float(d["pos"][0]), float(d["pos"][1]),
                         quat_yaw(d["quat"]), time.time())
        return True

    def wait_for_pose(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.poll():
                return self.pose
            time.sleep(0.002)
        raise SlamUnavailable(
            "no slam.pose within %.1fs -- is the p_slam daemon running and "
            "relocalized?" % timeout)

    def _fresh(self):
        return self.pose is not None and (time.time() - self.pose.stamp) < self.p.pose_stale_s

    # --- the leg ---

    def go_to(self, waypoint, label=""):
        """Turn, cruise, turn. Returns the final Pose; raises on any failure."""
        gx, gy, gyaw = float(waypoint[0]), float(waypoint[1]), float(waypoint[2])
        self.wait_for_pose()

        leg = math.hypot(gx - self.pose.x, gy - self.pose.y)
        if leg > self.p.max_leg_m:
            self.drive.stop()
            raise ApproachFailed(
                f"waypoint {label} is {leg:.2f}m away, over max_leg_m="
                f"{self.p.max_leg_m:.2f}m -- refusing. A SLAM relock probably "
                f"jumped the pose; re-check localization before retrying.")

        phase = Phase.TURN_TO_BEARING if leg > self.p.goal_tol_m else Phase.TURN_TO_PARK
        t0 = time.time()
        last_move = (self.pose.x, self.pose.y, self.pose.yaw)
        last_progress_t = t0
        self.log(f"[approach] {label} start: {leg:.2f}m away, phase={phase.value}")

        try:
            while True:
                now = time.time()
                if now - t0 > self.p.timeout_s:
                    raise ApproachFailed(f"{label}: timed out after {self.p.timeout_s:.0f}s "
                                         f"in phase {phase.value}")
                self.poll()
                if not self._fresh():
                    self.drive.stop()
                    raise SlamUnavailable(
                        f"{label}: slam.pose stale (>{self.p.pose_stale_s:.1f}s) -- "
                        f"stopped mid-leg")

                dx, dy = gx - self.pose.x, gy - self.pose.y
                dist = math.hypot(dx, dy)
                fwd, lat = world_to_body(dx, dy, self.pose.yaw)
                bearing = math.atan2(lat, fwd)

                # Progress means the base actually MOVED -- translated or
                # turned. Counting only translation would exempt the two
                # turn-in-place phases from the watchdog entirely, and a base
                # that cannot turn would then spin its wheels until the global
                # timeout instead of reporting a blockage.
                moved = math.hypot(self.pose.x - last_move[0],
                                   self.pose.y - last_move[1])
                turned = abs(wrap(self.pose.yaw - last_move[2]))
                if moved > self.p.progress_eps_m or turned > self.p.yaw_progress_eps_rad:
                    last_move = (self.pose.x, self.pose.y, self.pose.yaw)
                    last_progress_t = now
                elif phase is not Phase.DONE and now - last_progress_t > self.p.stuck_time_s:
                    raise ApproachFailed(
                        f"{label}: no movement for {self.p.stuck_time_s:.0f}s in phase "
                        f"{phase.value} ({dist:.2f}m short) -- something is blocking it")

                if phase is Phase.TURN_TO_BEARING:
                    if abs(bearing) < self.p.bearing_gate_rad:
                        phase = Phase.CRUISE
                        self.log(f"[approach] {label}: bearing aligned, cruising")
                        continue
                    self.drive.send(0.0, math.copysign(self.p.heading_omega, bearing))

                elif phase is Phase.CRUISE:
                    if dist < self.p.goal_tol_m:
                        phase = Phase.TURN_TO_PARK
                        self.drive.stop()
                        self.log(f"[approach] {label}: within {dist * 100:.0f}cm, "
                                 f"turning to park heading")
                        continue
                    if abs(bearing) > self.p.bearing_gate_rad * 2.5:
                        phase = Phase.TURN_TO_BEARING   # drifted badly off; re-aim
                        continue
                    v = self.p.speed * max(0.0, math.cos(bearing))
                    omega = clamp(self.p.k_yaw * bearing, -self.p.max_omega, self.p.max_omega)
                    self.drive.send(v, omega)

                elif phase is Phase.TURN_TO_PARK:
                    herr = wrap(gyaw - self.pose.yaw)
                    if abs(herr) < self.p.heading_tol_rad:
                        phase = Phase.DONE
                        continue
                    self.drive.send(0.0, math.copysign(self.p.heading_omega, herr))

                else:
                    self.drive.stop()
                    dist_final = math.hypot(gx - self.pose.x, gy - self.pose.y)
                    self.log(f"[approach] {label}: parked {dist_final * 100:.0f}cm from "
                             f"waypoint, heading off by "
                             f"{math.degrees(wrap(gyaw - self.pose.yaw)):+.1f}deg")
                    return self.pose

                time.sleep(0.002)
        except Exception:
            self.drive.stop()
            raise

    def drive_straight(self, distance_m, speed=None, timeout=None):
        """Open-loop-ish reverse/forward nudge, closed on SLAM displacement.

        Used to back off before an alignment retry. Falls back to a timed move
        only if the pose goes stale mid-nudge, because backing up blind next to
        a table is the one thing worth being conservative about: the fallback
        stops early rather than late.
        """
        speed = abs(speed if speed is not None else self.p.speed * 0.6)
        target = abs(distance_m)
        direction = 1.0 if distance_m >= 0 else -1.0
        timeout = timeout or (target / max(speed, 1e-3)) * 3.0 + 2.0
        self.wait_for_pose()
        start = (self.pose.x, self.pose.y)
        t0 = time.time()
        try:
            while time.time() - t0 < timeout:
                self.poll()
                if not self._fresh():
                    self.drive.stop()
                    self.log("[approach] pose went stale during nudge -- stopping short")
                    return False
                moved = math.hypot(self.pose.x - start[0], self.pose.y - start[1])
                if moved >= target:
                    self.drive.stop()
                    return True
                self.drive.send(direction * speed, 0.0)
                time.sleep(0.002)
            self.drive.stop()
            return False
        except Exception:
            self.drive.stop()
            raise
