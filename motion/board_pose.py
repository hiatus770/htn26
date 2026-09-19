"""Tag centers -> board pose -> parking error. Pure math: numpy only.

Split out of tags.py so the geometry that decides which way the wheels turn can
be exercised without a camera, a robot, or cv2 -- see motion/test_geometry.py.
A sign error in here is the most expensive bug in the package.
"""
import math
from dataclasses import dataclass

import numpy as np

from motion.geom import rigid_2d, rigid_2d_residual, wrap


class TagError(RuntimeError):
    pass


@dataclass
class BoardObservation:
    """Where the board is, and how wrong the base's parking is.

    All three errors are zero exactly when the base sits at the board's
    parking pose, and each is signed the way a human would say it:
      range_err > 0  the base is too far BACK
      lat_err   > 0  the base is toward the board's +x side (its own RIGHT)
      yaw_err   > 0  the base must turn LEFT to face the board squarely
    """
    stamp: float
    tag_ids: tuple
    center: np.ndarray      # board center in the base frame (x, y)
    u: np.ndarray           # board +x axis (robot's right when square), base frame
    n: np.ndarray           # board -> robot normal, base frame
    range_err: float
    lat_err: float
    yaw_err: float
    residual: float         # RMS fit error of the tag layout (m)

    def worst(self, p):
        """Normalized error, 1.0 = exactly at tolerance."""
        return max(abs(self.lat_err) / p.lat_tol_m,
                   abs(self.range_err) / p.range_tol_m,
                   abs(self.yaw_err) / p.yaw_tol_rad)

    def __str__(self):
        return (f"lat={self.lat_err * 100:+.1f}cm range={self.range_err * 100:+.1f}cm "
                f"yaw={math.degrees(self.yaw_err):+.2f}deg "
                f"tags={len(self.tag_ids)} fit={self.residual * 1000:.0f}mm")


def board_observation(board, points_base, stamp, max_residual=0.03):
    """Fit the known tag layout to detected tag centers.

    `points_base` is {tag_id: (x, y, z) or (x, y) in the base frame}. Returns
    None if fewer than two of this board's tags are present; raises TagError
    when the fit says the layout or the extrinsic is wrong, which is a
    configuration bug and must not be averaged away.
    """
    layout = board.tag_layout()
    ids = sorted(set(layout) & set(points_base))
    if len(ids) < 2:
        return None
    src = np.array([layout[i] for i in ids], dtype=np.float64)
    dst = np.array([np.asarray(points_base[i], dtype=np.float64)[:2] for i in ids])
    yaw_board, center = rigid_2d(src, dst)
    residual = rigid_2d_residual(src, dst, yaw_board, center)
    if residual > max_residual:
        raise TagError(
            f"board {board.index}: tag layout fit is {residual * 1000:.0f}mm RMS "
            f"(limit {max_residual * 1000:.0f}mm) from tags {ids}. Check tag_span_m, "
            f"the corner->id mapping, and the camera extrinsic in config.py.")

    c, s = math.cos(yaw_board), math.sin(yaw_board)
    u = np.array([c, s])                  # board +x in the base frame
    n = np.array([s, -c])                 # board -> robot normal (= -board +y)
    to_robot = -center                    # the base sits at the base frame's origin
    if float(n @ to_robot) <= 0.0:
        raise TagError(
            f"board {board.index}: solved normal points away from the robot "
            f"(center={center}, n={n}). The near/far corner ids are probably swapped.")

    goal_yaw = wrap(math.atan2(-n[1], -n[0]))
    return BoardObservation(
        stamp=stamp, tag_ids=tuple(ids), center=center, u=u, n=n,
        range_err=float(to_robot @ n) - board.standoff_m,
        lat_err=float(to_robot @ u) - board.lateral_offset_m,
        yaw_err=goal_yaw, residual=residual)


def parking_point(range_err, lat_err, yaw_err):
    """The parking pose in the base frame, from the three errors alone.

    p* = -n*range_err - u*lat_err, with n and u re-derived from yaw_err. This
    is what lets the controller run on median-filtered scalars instead of on
    raw per-frame poses, which matters because the base never stops swaying.
    """
    c, s = math.cos(yaw_err), math.sin(yaw_err)
    return c * range_err - s * lat_err, s * range_err + c * lat_err
