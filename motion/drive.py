"""Wheel command bus -- the only thing in this repo that writes drive.ctrl.

Clamping, the stiction floor, and the stop-on-exit guarantee live here so no
control loop can forget one of them.

IMPORTANT: do not run bbapps/nav/main.py while anything in motion/ is running.
Its control loop publishes a zero twist every tick when it is idle
(bbapps/nav/main.py:1265), so the two writers fight and the micro-corrections
get cancelled roughly half the time.

Command units match the drive daemon's contract: twist = [v (m/s), omega
(rad/s)], positive omega turning left (bbapps/teleop.py:63, nav/main.py:1301).
"""
import os
import threading
import time
from contextlib import contextmanager

import numpy as np
from bbos import Config, Type, Writer

from motion.geom import apply_floor, clamp

TOPIC = "drive.ctrl"
SHM_PATH = f"/dev/shm/{TOPIC}"

# Hard ceilings, independent of any per-phase params. Nothing in a chess match
# needs to move faster than a slow walk next to a table full of pieces.
V_CAP = 0.20
OMEGA_CAP = 0.40

HOLD_HZ = 20.0


def wheel_vels_to_twist(v_left, v_right):
    """Differential wheel speeds -> twist, as bbapps does it (teleop.py:63)."""
    R = Config("drive").robot_width * 0.5
    return (v_left + v_right) / 2.0, (v_right - v_left) / (2.0 * R)


class DriveBus:
    def __init__(self, v_min=0.0, omega_min=0.0, dry_run=False, log=print):
        self.v_min = v_min
        self.omega_min = omega_min
        self.dry_run = dry_run
        self.log = log
        self._w = None
        self._lock = threading.RLock()
        self._hold_stop = None
        self._hold_thread = None
        self.last = (0.0, 0.0)

    # --- lifecycle ---

    def __enter__(self):
        if not self.dry_run:
            self._open()
        else:
            self.log("[drive] DRY RUN -- twists are logged, never published")
        return self

    def __exit__(self, *exc):
        try:
            self.stop()
        finally:
            self._close()
        return False

    def _open(self):
        self._w = Writer(TOPIC, Type("drive_ctrl"), keeptime=False)
        self._w.__enter__()

    def _close(self):
        self.end_hold()
        with self._lock:
            if self._w is not None:
                try:
                    self._w.__exit__(None, None, None)
                except Exception:
                    pass
                self._w = None

    def _ensure_writer(self):
        """A full `restart` on the robot rm's /dev/shm/*.ctrl, which leaves our
        segment an orphan inode that nothing reads. nav handles this the same
        way (bbapps/nav/main.py:1298)."""
        if self._w is None:
            self._open()
        elif not os.path.exists(SHM_PATH):
            self.log("[drive] drive.ctrl segment vanished -- reopening writer")
            try:
                self._w.__exit__(None, None, None)
            except Exception:
                pass
            self._open()

    # --- commands ---

    def send(self, v, omega):
        """Clamp, floor, publish. Returns the twist actually commanded."""
        v = clamp(float(v), -V_CAP, V_CAP)
        omega = clamp(float(omega), -OMEGA_CAP, OMEGA_CAP)
        v = apply_floor(v, self.v_min)
        omega = apply_floor(omega, self.omega_min)
        with self._lock:
            self.last = (v, omega)
            if self.dry_run:
                self.log(f"[drive:dry] v={v:+.3f} w={omega:+.3f}")
                return v, omega
            self._ensure_writer()
            self._w["twist"] = np.array([v, omega], dtype=np.float32)
        return v, omega

    def stop(self):
        """Explicit zero twist. Bypasses the stiction floor by construction."""
        with self._lock:
            self.last = (0.0, 0.0)
            if self.dry_run:
                self.log("[drive:dry] STOP")
                return
            try:
                self._ensure_writer()
                self._w["twist"] = np.array([0.0, 0.0], dtype=np.float32)
            except Exception as e:
                self.log(f"[drive] stop failed: {e}")

    # --- position hold during the arm handoff ---

    def begin_hold(self):
        """Keep publishing zeros at 20Hz so the base stays parked and the drive
        daemon's watchdog keeps seeing a live setpoint while the arms play."""
        if self._hold_thread is not None:
            return
        self._hold_stop = threading.Event()

        def _pump():
            period = 1.0 / HOLD_HZ
            while not self._hold_stop.is_set():
                self.stop()
                time.sleep(period)

        self._hold_thread = threading.Thread(target=_pump, name="drive-hold", daemon=True)
        self._hold_thread.start()

    def end_hold(self):
        if self._hold_thread is None:
            return
        self._hold_stop.set()
        self._hold_thread.join(timeout=1.0)
        self._hold_thread = None
        self._hold_stop = None

    @contextmanager
    def hold(self):
        self.begin_hold()
        try:
            yield self
        finally:
            self.end_hold()
            self.stop()
