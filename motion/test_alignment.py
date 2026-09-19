"""Phase 2 only: micro-correct onto one board the robot is already near.

Park the base roughly in front of the board first (teleop is fine) so the top
camera can see its four tags, then:

    python -m motion.test_alignment --board 1 --dry-run   # logs twists, no motion
    python -m motion.test_alignment --board 1             # live, hand on the e-stop

Run the dry pass first and check the signs: pushed to the LEFT of the board it
should command a negative omega (turn right) or a leftward arc, never the
opposite. A sign error here drives the base into the table.

bbapps/nav/main.py must NOT be running -- it publishes zero twists when idle
and will fight every correction.
"""
import argparse

from motion.config import AlignParams, BoardTable
from motion.drive import DriveBus
from motion.tag_align import AlignmentError, TagAligner
from motion.tags import TopCameraTags


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", type=int, required=True)
    ap.add_argument("--dry-run", action="store_true", help="log twists, never publish")
    ap.add_argument("--timeout", type=float, default=None)
    ap.add_argument("--lat-tol-cm", type=float, default=None)
    ap.add_argument("--range-tol-cm", type=float, default=None)
    args = ap.parse_args()

    table = BoardTable.load()
    board = table.get(args.board)
    params = AlignParams()
    if args.lat_tol_cm is not None:
        params.lat_tol_m = args.lat_tol_cm / 100.0
    if args.range_tol_cm is not None:
        params.range_tol_m = args.range_tol_cm / 100.0

    with DriveBus(v_min=params.v_min, omega_min=params.omega_min,
                  dry_run=args.dry_run) as drive, \
         TopCameraTags(tag_size_m=table.tag_size_m) as cam:
        aligner = TagAligner(drive, cam, params)
        try:
            result = aligner.align(board, timeout=args.timeout)
        except AlignmentError as e:
            print(f"FAILED: {e}")
            raise SystemExit(1)
        print(f"OK: {result}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\ninterrupted -- base stopped")
