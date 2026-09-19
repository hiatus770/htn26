"""Movement, navigation and alignment control loops for the simul chess robot.

Imports are lazy so that `motion.config` and `motion.geom` stay usable off the
robot (bbos, cv2 and pupil_apriltags only exist on the bot).
"""
__all__ = ["BoardTraversal", "DriveBus", "SlamApproach", "TagAligner", "TopCameraTags",
           "BoardTable", "TraversalParams"]

_LAZY = {
    "BoardTraversal": "motion.board_traversal",
    "DriveBus": "motion.drive",
    "SlamApproach": "motion.slam_nav",
    "TagAligner": "motion.tag_align",
    "TopCameraTags": "motion.tags",
    "BoardTable": "motion.config",
    "TraversalParams": "motion.config",
}


def __getattr__(name):
    if name in _LAZY:
        import importlib
        return getattr(importlib.import_module(_LAZY[name]), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
