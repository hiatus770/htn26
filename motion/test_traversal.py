"""The full 1-to-5 traversal, with a placeholder for the arms.

    python -m motion.test_traversal --board 3      # one board, approach + align
    python -m motion.test_traversal --cycles 1     # 1->5 once
    python -m motion.test_traversal --cycles 0     # serpentine forever (Ctrl-C to stop)

Before running: stop bbapps/nav/main.py (two writers on drive.ctrl cancel each
other), confirm SLAM is relocalized, and keep a hand on the e-stop.

--dry-run publishes nothing, so the base never moves and the SLAM leg trips its
own no-progress detector after ~8s. That is the expected result and doubles as
a test of the stuck detector; use it to check the waypoints load and the
camera/tag plumbing is alive, not to test the drive.
"""
import argparse
import time

from motion.board_traversal import BoardTraversal, BoardUnreachable


def placeholder_play(board, park):
    """Stand-in for the vision/engine handoff. The base is holding here."""
    print(f"    [arms] board {board.index}: would read the position and play a move")
    print(f"    [arms] residual error the arms must absorb: "
          f"lat {park.lat_err * 100:+.2f}cm, range {park.range_err * 100:+.2f}cm")
    time.sleep(2.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", type=int, default=None, help="visit one board and stop")
    ap.add_argument("--cycles", type=int, default=1,
                    help="passes over the table; 0 = forever (default 1)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--stop-on-error", action="store_true",
                    help="abort instead of skipping a board that cannot be reached")
    args = ap.parse_args()

    with BoardTraversal(dry_run=args.dry_run) as bt:
        if args.board is not None:
            try:
                bt.visit(bt.table.get(args.board), placeholder_play)
            except BoardUnreachable as e:
                print(f"FAILED: {e}")
                raise SystemExit(1)
            return
        reports = bt.run(placeholder_play,
                         cycles=None if args.cycles == 0 else args.cycles,
                         stop_on_error=args.stop_on_error)
        for r in reports:
            print(f"  board {r.board_index}: "
                  f"{r.result if r.parked else 'FAILED ' + r.error}")
        if any(not r.parked for r in reports):
            raise SystemExit(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\ninterrupted -- base stopped")
