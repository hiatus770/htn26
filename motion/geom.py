"""Frame and pose math shared by the motion loops. Pure python, no bbos.

Two frames show up everywhere in this package, and mixing them up is the
single easiest way to drive the robot into a table:

  SLAM frame   world x/y with a yaw whose forward vector is (-sin, cos) --
               i.e. the robot faces +Y at yaw 0, NOT +X. That is bbapps'
               convention, not ours (bbapps/nav/planner.py:77 for the quat
               and bbapps/nav/main.py:1189 for the heading vector), and it is
               confined to slam_nav.py via world_to_body().

  base frame   conventional robot body frame: x forward, y left, z up. All
               camera/tag geometry lives here so the parking controller reads
               like a textbook one.
"""
import math

import numpy as np


def wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_yaw(q):
    """Yaw from a slam.pose quaternion, same derivation nav uses.

    See quat_yaw in bbapps/nav/planner.py:77 -- reproduced rather than
    imported because that module pulls in numba, scipy and PIL for the
    Dijkstra planner we deliberately do not use.
    """
    return 2.0 * math.atan2(q[2], q[3])


def heading_vectors(yaw):
    """(forward, left) unit vectors for a SLAM yaw.

    `left` is d(forward)/d(yaw), which is also the direction a positive omega
    turns toward -- that is what makes the sign of every steering term below
    agree with the drive daemon.
    """
    forward = (-math.sin(yaw), math.cos(yaw))
    left = (-math.cos(yaw), -math.sin(yaw))
    return forward, left


def world_to_body(dx, dy, yaw):
    """World-frame delta -> (forward, left) distances in the robot's frame."""
    forward, left = heading_vectors(yaw)
    return dx * forward[0] + dy * forward[1], dx * left[0] + dy * left[1]


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def rigid_2d(src, dst):
    """Least-squares 2D rigid transform (rotation + translation) src -> dst.

    Returns (yaw, t) such that dst ~= R(yaw) @ src + t. Closed form, so three
    tag centers cost the same as four and a missing tag costs nothing but
    precision. Deliberately a pure rotation: a reflection here would mean the
    tag layout was entered mirrored, and we want that to show up as a large
    residual rather than be silently absorbed.
    """
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    if src.shape != dst.shape or src.shape[0] < 2:
        raise ValueError(f"rigid_2d needs >=2 matched points, got {src.shape}")
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    s, d = src - cs, dst - cd
    num = float(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]))
    den = float(np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    yaw = math.atan2(num, den)
    c, sn = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -sn], [sn, c]], dtype=np.float64)
    t = cd - R @ cs
    return yaw, t


def rigid_2d_residual(src, dst, yaw, t):
    """RMS fit error (m) of a rigid_2d solution -- our tag-layout sanity check."""
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]], dtype=np.float64)
    err = (src @ R.T + t) - dst
    return float(np.sqrt(np.mean(np.sum(err * err, axis=1))))


def clamp(value, lo, hi):
    return lo if value < lo else (hi if value > hi else value)


def apply_floor(value, floor, dead=1e-4):
    """Push a small nonzero command up to `floor`, leaving exact zeros alone.

    The base will not break static friction below roughly 0.02 m/s, so an
    un-floored micro-correction converges to "permanently 8mm off" -- it keeps
    commanding a creep the wheels never execute.
    """
    if abs(value) < dead:
        return 0.0
    if abs(value) < floor:
        return math.copysign(floor, value)
    return value
