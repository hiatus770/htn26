"""The 1-to-5 board traversal loop.

    SLAM waypoint  ->  AprilTag micro-correction  ->  base holds  ->  arms play

BoardTraversal owns the whole cycle: it is the only object that touches the
drive bus, so every exit path -- exception, Ctrl-C, a board it cannot reach --
goes through a stop.

Usage on the robot (nav app NOT running):

    from motion.board_traversal import BoardTraversal

    def play(board, park):
        # hand off to vision/ and engine/; the base is holding position here
        ...

    with BoardTraversal() as bt:
        bt.run(play, cycles=None)     # serpentine 1->5, 5->1, ... forever
"""
import time
from dataclasses import dataclass

from motion.config import BoardTable, TraversalParams
from motion.drive import DriveBus
from motion.slam_nav import ApproachError, SlamApproach
from motion.tag_align import AlignmentError, ParkResult, TagAligner
from motion.tags import TopCameraTags


class BoardUnreachable(RuntimeError):
    """Every attempt at one board failed. The base is stopped."""


@dataclass
class VisitReport:
    board_index: int
    parked: bool
    result: ParkResult = None
    error: str = ""


class BoardTraversal:
    def __init__(self, table=None, params=None, dry_run=False, log=print):
        self.table = table or BoardTable.load()
        self.p = params or TraversalParams()
        self.dry_run = dry_run
        self.log = log
        self.drive = DriveBus(v_min=self.p.align.v_min, omega_min=self.p.align.omega_min,
                              dry_run=dry_run, log=log)
        self.approach = SlamApproach(self.drive, self.p.approach, log=log)
        self.camera = TopCameraTags(tag_size_m=self.table.tag_size_m, log=log)
        self.aligner = TagAligner(self.drive, self.camera, self.p.align, log=log)
        self._entered = []

    # --- lifecycle ---

    def __enter__(self):
        for ctx in (self.drive, self.approach, self.camera):
            ctx.__enter__()
            self._entered.append(ctx)
        self.log(f"[traversal] ready, {len(self.table.boards)} boards: "
                 f"{[b.index for b in self.table.boards]}")
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def close(self):
        try:
            self.drive.stop()
        except Exception:
            pass
        while self._entered:
            ctx = self._entered.pop()
            try:
                ctx.__exit__(None, None, None)
            except Exception as e:
                self.log(f"[traversal] cleanup: {e}")

    # --- one board ---

    def _resolve(self, board):
        return board if hasattr(board, "waypoint") else self.table.get(int(board))

    def park_at(self, board):
        """SLAM approach then tag alignment, with retries. Returns ParkResult."""
        board = self._resolve(board)
        attempts = self.p.retries + 1
        last_error = None
        for attempt in range(attempts):
            tag = f"board {board.index}" + (f" (attempt {attempt + 1}/{attempts})"
                                            if attempt else "")
            try:
                self.approach.go_to(board.waypoint, label=tag)
                # Let the base finish swaying before the first measurement --
                # otherwise the median window starts full of transient error.
                self.drive.stop()
                time.sleep(self.p.settle_pause_s)
                return self.aligner.align(board)
            except (ApproachError, AlignmentError) as e:
                last_error = e
                self.drive.stop()
                self.log(f"[traversal] {tag} failed: {e}")
                if attempt + 1 >= attempts:
                    break
                self.log(f"[traversal] backing off {self.p.back_off_m * 100:.0f}cm "
                         f"and re-approaching")
                try:
                    self.approach.drive_straight(-self.p.back_off_m)
                except Exception as e2:
                    self.log(f"[traversal] back-off failed: {e2}")
                    break
        self.drive.stop()
        raise BoardUnreachable(f"board {board.index}: {last_error}") from last_error

    def visit(self, board, on_board):
        """Park at a board and hand the base off to the caller while it holds."""
        board = self._resolve(board)
        result = self.park_at(board)
        with self.drive.hold():
            self.log(f"[traversal] board {board.index}: base holding, handing off")
            on_board(board, result)
        self.log(f"[traversal] board {board.index}: handoff done")
        return result

    # --- the loop ---

    def order(self, cycles=1):
        """Serpentine: 1->5, then 5->1, and so on.

        A simul player walks back down the line; driving the whole length of
        the table to restart at board 1 would waste the move clock and add a
        long blind leg for no reason.
        """
        boards = list(self.table.boards)
        if not boards:
            raise RuntimeError("no boards configured -- run "
                               "`uv run motion/tools/record_waypoints.py` first")
        cycle = 0
        while cycles is None or cycle < cycles:
            yield from (boards if cycle % 2 == 0 else list(reversed(boards)))
            cycle += 1

    def run(self, on_board, cycles=1, stop_on_error=False):
        """Traverse the boards, calling on_board(board, park_result) at each.

        A board that cannot be reached is skipped (and reported) rather than
        retried forever -- by then it is a physical problem, and grinding the
        base into a table leg will not solve it.
        """
        reports = []
        try:
            for board in self.order(cycles):
                try:
                    result = self.visit(board, on_board)
                    reports.append(VisitReport(board.index, True, result))
                except BoardUnreachable as e:
                    reports.append(VisitReport(board.index, False, error=str(e)))
                    self.log(f"[traversal] SKIPPING board {board.index}: {e}")
                    if stop_on_error:
                        raise
        except KeyboardInterrupt:
            self.log("[traversal] interrupted -- stopping base")
            raise
        finally:
            self.drive.stop()
        ok = sum(1 for r in reports if r.parked)
        self.log(f"[traversal] done: {ok}/{len(reports)} visits parked cleanly")
        return reports
