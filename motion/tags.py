"""Top camera -> board pose in the base frame.

camera.head.jpeg is a SIDE-BY-SIDE STEREO frame: the right half is the right
eye (bbapps/inference/vlm.py:56) and each eye is Config("cam_head").width // 2
wide (bbapps/nav/main.py:90). We detect on the left half, because the left
camera matrix from camera_cal() is the one that matches it. Feeding the whole
stereo frame to a tag detector produces plausible-looking detections with
completely wrong poses, which is the worst failure mode available here.

Board pose comes from a least-squares fit of the detected tag centers against
the known corner layout, not from a single tag's orientation: a single
AprilTag's rotation is famously ambiguous when viewed near head-on, while
three or four centers pin the board down cleanly.
"""
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
from bbos import Config, Reader

from motion.board_pose import BoardObservation, TagError, board_observation
from motion.config import HEAD_HFOV_DEG_FALLBACK, HEAD_INTRINSICS_OVERRIDE, TOP_CAMERA

try:
    from pupil_apriltags import Detector as _AprilDetector
except ImportError as e:      # pragma: no cover - robot-only dependency
    _AprilDetector = None
    _IMPORT_ERROR = e

TOPIC = "camera.head.jpeg"

# Re-exported: callers import the whole tag stack from here.
__all__ = ["TopCameraTags", "TagDetection", "BoardObservation", "TagError",
           "board_observation", "head_eye_intrinsics", "Intrinsics"]


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist: np.ndarray

    @property
    def mtx(self):
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]], dtype=np.float64)

    def __str__(self):
        return (f"fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} "
                f"cy={self.cy:.1f} {self.width}x{self.height}")


def head_eye_intrinsics():
    """Left-eye intrinsics for camera.head.jpeg.

    nav's own comment is worth heeding here: the config-derived numbers assume
    the daemon rectifies the way the stock pipeline does, and a reflash has
    broken that before (bbapps/nav/main.py:127-131). Pin
    HEAD_INTRINSICS_OVERRIDE once you have verified them with test_tags.py.

    On a robot where depth/stereo was never calibrated, Config("depth")
    .camera_cal() has no file to load and raises FileNotFoundError. Rather
    than crash the whole tag pipeline over a calibration file this robot may
    simply never need for anything else, fall back to an uncalibrated pinhole
    guess and say so loudly -- test_tags.py's distance check is exactly the
    tool for turning that guess into a real HEAD_INTRINSICS_OVERRIDE.
    """
    cfg_c = Config("cam_head")
    width, height = int(cfg_c.width) // 2, int(cfg_c.height)
    if HEAD_INTRINSICS_OVERRIDE is not None:
        fx, fy, cx, cy = (float(v) for v in HEAD_INTRINSICS_OVERRIDE)
        return Intrinsics(fx, fy, cx, cy, width, height, np.zeros(5))
    try:
        cal = Config("depth").camera_cal()
    except FileNotFoundError as e:
        fx = fy = (width / 2.0) / math.tan(math.radians(HEAD_HFOV_DEG_FALLBACK) / 2.0)
        cx, cy = width / 2.0, height / 2.0
        print(f"[tags] WARNING: no depth calibration on this robot ({e}). Using "
              f"an UNCALIBRATED pinhole guess: fx=fy={fx:.0f}px (assumed "
              f"{HEAD_HFOV_DEG_FALLBACK:.0f}deg HFOV), cx={cx:.0f} cy={cy:.0f}. "
              f"Hold a tag at a measured distance, compare against `dist=` "
              f"below, and set HEAD_INTRINSICS_OVERRIDE in motion/config.py "
              f"once it tracks -- every centimetre of error here becomes a "
              f"centimetre of parking error.")
        return Intrinsics(fx, fy, cx, cy, width, height, np.zeros(5))
    mtx_l = np.asarray(cal[0], dtype=np.float64)
    dist_l = np.asarray(cal[1], dtype=np.float64).ravel()
    return Intrinsics(float(mtx_l[0, 0]), float(mtx_l[1, 1]),
                      float(mtx_l[0, 2]), float(mtx_l[1, 2]),
                      width, height, dist_l)


@dataclass
class TagDetection:
    tag_id: int
    p_base: np.ndarray      # tag center in the base frame (x fwd, y left, z up)
    p_cam: np.ndarray       # tag center in camera coords, for debugging
    margin: float


class TopCameraTags:
    """Reads camera.head.jpeg and reports tag centers in the base frame."""

    def __init__(self, tag_size_m=0.05, intrinsics=None, mount=None,
                 quad_decimate=2.0, families="tag36h11", nthreads=2, log=print):
        if _AprilDetector is None:
            raise TagError(
                "pupil_apriltags is not installed. On the robot: "
                "`uv pip install pupil-apriltags` (or add it to the script's uv "
                f"dependency block). Original error: {_IMPORT_ERROR}")
        self.tag_size_m = float(tag_size_m)
        self.mount = mount or TOP_CAMERA
        self.T_base_cam = self.mount.T_base_cam()
        self.log = log
        self._intr = intrinsics
        self._det = _AprilDetector(families=families, nthreads=nthreads,
                                   quad_decimate=quad_decimate, refine_edges=1)
        self._r = None
        self._maps = None
        self.last_frame_stamp = 0.0
        self.last_detections = []

    def __enter__(self):
        if self._intr is None:
            self._intr = head_eye_intrinsics()
            self.log(f"[tags] head left-eye intrinsics: {self._intr}")
        self._r = Reader(TOPIC)
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

    @property
    def intrinsics(self):
        if self._intr is None:
            self._intr = head_eye_intrinsics()
        return self._intr

    # --- frame plumbing ---

    def _undistort(self, gray):
        """pupil_apriltags' pose solver assumes a pinhole camera, so the
        distortion has to come out of the image rather than be carried in the
        model."""
        intr = self.intrinsics
        if intr.dist is None or not np.any(intr.dist):
            return gray
        if self._maps is None:
            h, w = gray.shape[:2]
            self._maps = cv2.initUndistortRectifyMap(
                intr.mtx, intr.dist, None, intr.mtx, (w, h), cv2.CV_16SC2)
        return cv2.remap(gray, self._maps[0], self._maps[1], cv2.INTER_LINEAR)

    def read_frame(self, block=False, timeout=1.0):
        """Latest left-eye grayscale frame, or None if nothing new is up."""
        if self._r is None:
            raise TagError("TopCameraTags used outside its context manager")
        t0 = time.time()
        while True:
            if self._r.ready():
                payload = bytes(self._r.data["jpeg"][:self._r.data["jpeg_len"]])
                stamp = time.time()
                img = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8),
                                   cv2.IMREAD_GRAYSCALE)
                if img is None:
                    raise TagError("camera.head.jpeg published an undecodable frame")
                left = np.ascontiguousarray(img[:, :img.shape[1] // 2])
                self.last_frame_stamp = stamp
                return self._undistort(left), stamp
            if not block or time.time() - t0 > timeout:
                return None, 0.0
            time.sleep(0.002)

    # --- detection ---

    def detect(self, block=False, timeout=1.0):
        """Detections in the base frame, or None if no new frame was ready."""
        gray, stamp = self.read_frame(block=block, timeout=timeout)
        if gray is None:
            return None, 0.0
        intr = self.intrinsics
        raw = self._det.detect(
            gray, estimate_tag_pose=True,
            camera_params=(intr.fx, intr.fy, intr.cx, intr.cy),
            tag_size=self.tag_size_m)
        out = []
        for r in raw:
            p_cam = np.asarray(r.pose_t, dtype=np.float64).reshape(3)
            p_base = (self.T_base_cam[:3, :3] @ p_cam) + self.T_base_cam[:3, 3]
            out.append(TagDetection(int(r.tag_id), p_base, p_cam,
                                    float(getattr(r, "decision_margin", 0.0))))
        self.last_detections = out
        return out, stamp

    def observe(self, board, max_residual=0.03, block=False, timeout=1.0):
        """One BoardObservation, or None if there was no new frame / too few tags."""
        dets, stamp = self.detect(block=block, timeout=timeout)
        if dets is None:
            return None
        points = {d.tag_id: d.p_base for d in dets}
        return board_observation(board, points, stamp, max_residual=max_residual)
