"""Every tunable for the board traversal loop, in one file.

Nothing here talks to the robot -- import it anywhere (including off-robot) to
inspect or edit the numbers. Control gains start from the values bbapps' nav
app has already proven on this hardware (bbapps/nav/main.py:348-374) rather
than from fresh guesses.
"""
import json
import math
import os
from dataclasses import dataclass, field, asdict

import numpy as np

from motion.geom import rot_x, rot_y, rot_z

BOARDS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "boards.json")


# --- Top camera --------------------------------------------------------------

@dataclass
class CameraMount:
    """Head ("top") camera pose in the base frame: x forward, y left, z up.

    MEASURE THESE ON THE ROBOT. Unlike the depth camera, no config in bbapps
    carries a T_base_cam for the head camera -- nav only ever looks it up for
    the depth daemons (bbapps/nav/main.py:213-219). motion/test_tags.py is the
    check: a tag held at a known distance must read that distance back.
    """
    height_m: float = 1.20      # optical center above the floor
    forward_m: float = 0.05     # ahead of the wheel axis
    left_m: float = 0.0         # lateral offset from the robot centerline
    pitch_deg: float = 45.0     # positive = tilted down
    yaw_deg: float = 0.0        # positive = rotated left
    roll_deg: float = 0.0

    def T_base_cam(self):
        """4x4 transform taking OpenCV camera coords (x right, y down, z fwd)
        into the base frame (x fwd, y left, z up)."""
        # Level camera: its z (forward) is base x, its x (right) is base -y,
        # its y (down) is base -z.
        R0 = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]], dtype=np.float64)
        R = (rot_z(math.radians(self.yaw_deg))
             @ rot_y(math.radians(self.pitch_deg))
             @ rot_x(math.radians(self.roll_deg)) @ R0)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = (self.forward_m, self.left_m, self.height_m)
        return T


TOP_CAMERA = CameraMount()

# Left-eye intrinsics override. camera.head.jpeg is a side-by-side stereo
# frame, and nav warns that the config-derived intrinsics drift when a reflash
# changes how the daemon rectifies (bbapps/nav/main.py:127-131). Leave None to
# derive them from Config("depth").camera_cal(); set (fx, fy, cx, cy) to pin
# them after calibrating against a tag at a known distance.
HEAD_INTRINSICS_OVERRIDE = None


# --- Boards ------------------------------------------------------------------

# Board frame, used for the tag layout below:
#   origin = board center
#   +x     = to the ROBOT's right along the near edge
#   +y     = away from the robot, across the board
#   (+z up, so it is right-handed like the base frame -- which is what lets
#    geom.rigid_2d solve for a pure rotation. Flipping +y to point at the robot
#    makes the pair left-handed and every fit comes back mirrored.)
CORNERS = ("near_left", "near_right", "far_right", "far_left")


@dataclass
class BoardSpec:
    index: int
    name: str
    waypoint: tuple                       # SLAM (x, y, yaw_park) -- taught, never typed
    tags: dict                            # corner name -> AprilTag id
    tag_span_m: float = 0.3390             # center-to-center distance between corner tags;
                                          # 0.339 = this robot's 33.9x33.9cm boards, tags at
                                          # the corners. record_waypoints.py measures the
                                          # real value per board anyway -- this is just the
                                          # fallback when tag capture is skipped.
    standoff_m: float = 0.35              # parked distance from board center to base
    lateral_offset_m: float = 0.0         # + parks toward the board's +x (robot's right)

    def tag_layout(self):
        """{tag_id: (x, y)} in the board frame."""
        h = self.tag_span_m * 0.5
        local = {
            "near_left": (-h, -h), "near_right": (h, -h),
            "far_right": (h, h), "far_left": (-h, h),
        }
        out = {}
        for corner, tag_id in self.tags.items():
            if corner not in local:
                raise ValueError(f"board {self.index}: unknown corner {corner!r}, "
                                 f"expected one of {CORNERS}")
            out[int(tag_id)] = local[corner]
        return out


@dataclass
class BoardTable:
    tag_size_m: float = 0.05              # printed side length of the black tag square
    boards: list = field(default_factory=list)

    @classmethod
    def load(cls, path=BOARDS_FILE):
        try:
            with open(path) as f:
                raw = json.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(
                f"{path} does not exist. Waypoints are taught, not typed -- run "
                f"`uv run motion/tools/record_waypoints.py` and drive the robot to "
                f"each board.") from None
        boards = []
        for b in raw.get("boards", []):
            boards.append(BoardSpec(
                index=int(b["index"]), name=b.get("name", f"board-{b['index']}"),
                waypoint=tuple(float(v) for v in b["waypoint"]),
                tags={k: int(v) for k, v in b["tags"].items()},
                tag_span_m=float(b.get("tag_span_m", raw.get("tag_span_m", 0.339))),
                standoff_m=float(b.get("standoff_m", raw.get("standoff_m", 0.35))),
                lateral_offset_m=float(b.get("lateral_offset_m", 0.0)),
            ))
        boards.sort(key=lambda b: b.index)
        table = cls(tag_size_m=float(raw.get("tag_size_m", 0.05)), boards=boards)
        table.validate()
        return table

    def save(self, path=BOARDS_FILE):
        raw = {"tag_size_m": self.tag_size_m,
               "boards": [asdict(b) | {"waypoint": list(b.waypoint)} for b in self.boards]}
        with open(path, "w") as f:
            json.dump(raw, f, indent=2)
            f.write("\n")

    def validate(self):
        """Duplicate tag ids across boards would let board 3's alignment lock
        onto board 4's corner, so this is a hard error, not a warning."""
        seen = {}
        for b in self.boards:
            for corner, tag_id in b.tags.items():
                if tag_id in seen:
                    raise ValueError(f"tag id {tag_id} used by both {seen[tag_id]} "
                                     f"and board {b.index}/{corner}")
                seen[tag_id] = f"board {b.index}/{corner}"
        return self

    def get(self, index):
        for b in self.boards:
            if b.index == index:
                return b
        raise KeyError(f"no board with index {index} in {BOARDS_FILE}")


# --- Control parameters ------------------------------------------------------

@dataclass
class ApproachParams:
    """Phase 1: SLAM drive to the vicinity of a board."""
    speed: float = 0.08                   # m/s -- nav's proven cruise (main.py:349)
    max_omega: float = 0.15               # rad/s -- nav's MAX_OMEGA
    heading_omega: float = 0.15           # rad/s turn-in-place speed (nav HEADING_OMEGA)
    k_yaw: float = 1.0                    # bearing P-gain while cruising
    bearing_gate_rad: float = math.radians(10.0)   # turn in place until bearing is this tight
    # Tighter than nav's 0.25: boards sit ~0.4m apart, and every centimetre
    # left here becomes a pivot-drive-pivot shuffle in phase 2.
    goal_tol_m: float = 0.10
    heading_tol_rad: float = 0.07         # nav HEADING_TOL, ~4deg
    max_leg_m: float = 3.0                # refuse a waypoint further than this
    pose_stale_s: float = 0.5             # nav's _FRESH_THRESH for slam (main.py:487)
    progress_eps_m: float = 0.02
    yaw_progress_eps_rad: float = 0.05    # so the turn phases are watchdogged too
    stuck_time_s: float = 8.0
    timeout_s: float = 60.0


@dataclass
class AlignParams:
    """Phase 2: AprilTag micro-correction. Slower and tighter than phase 1."""
    v_max: float = 0.04                   # creeping toward a table: slow
    omega_max: float = 0.15               # steering correction while driving
    omega_pivot: float = 0.30             # turning in place moves nothing toward the table
    v_min: float = 0.02                   # stiction floor -- see geom.apply_floor
    omega_min: float = 0.05
    k_rho: float = 0.6                    # distance -> forward speed
    k_alpha: float = 1.6                  # bearing -> steering
    k_yaw_polish: float = 0.8             # final heading -> pivot speed
    aim_tol_rad: float = math.radians(3.0)    # pivot until the parking point is this close
    aim_exit_rad: float = math.radians(15.0)  # drifted this far off -> re-aim (hysteresis)
    rho_min_m: float = 0.015              # below this, correct heading only (see tag_align)
    # Lateral tolerance is the loose one on purpose. A differential drive
    # cannot strafe, so every lateral correction is a pivot-drive-pivot
    # shuffle. 1.5cm is where test_geometry.py's noise sweep stops hunting:
    # at 1.0cm it starts missing its settle window once measurement noise
    # reaches ~5mm, and it is no more accurate for it. The exact residual is
    # handed to the arms in ParkResult, so they can work in board coordinates
    # instead of assuming a perfectly centered base.
    lat_tol_m: float = 0.015
    range_tol_m: float = 0.020
    yaw_tol_rad: float = math.radians(1.5)
    hysteresis: float = 1.5               # widened tolerance to STAY in the yaw phase
    median_window: int = 5                # rolling median -- the base sways continuously
    settle_frames: int = 5                # consecutive in-tolerance frames before "locked"
    exit_yaw_frames: int = 5              # frames out of tolerance before restarting a
                                          # shuffle; one noisy frame must not do it
    det_stale_s: float = 0.3              # no tags this long -> stop the base
    min_tags: int = 3                     # of 4; 2 is geometrically enough but too noisy
    max_fit_residual_m: float = 0.03      # rigid_2d RMS above this = wrong tags/layout
    timeout_s: float = 45.0               # a sideways shuffle is several pivots long


@dataclass
class TraversalParams:
    approach: ApproachParams = field(default_factory=ApproachParams)
    align: AlignParams = field(default_factory=AlignParams)
    retries: int = 1                      # re-approaches per board before giving up
    back_off_m: float = 0.20              # reverse this far before a retry
    settle_pause_s: float = 0.5           # let the base stop swaying before measuring
