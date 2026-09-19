# Project Context: Bracket Bot (5-Board Simul Chess Robot)
This repository contains the software stack for a physical robotics project: a tall, self-balancing (segway-style with training wheels) robot that plays simultaneous chess against 5 physical boards lined up on a table. 

## Hardware & Perception Assumptions
- **Base:** Segway-style two-wheel drive with training wheels (for stability).
- **Arms:** Two 4-DOF arms with shoulder joints that can move vertically. Claws have cameras for piece placing.
- **Top Camera:** Mounted at the top, angled down at 45 degrees. Used for board layout CV and reading AprilTags.
- **Localization (Global):** SLAM model maps the environment and knows rough coordinates of the table. ⚠️ **Not currently running on `bracketbot-0185`** (no `slam.pose` in `/dev/shm` as of 2026-09-19) — see `motion/motion.md` for what this blocks.
- **Localization (Local):** 4 AprilTags on the corners of each physical chessboard. Boards are 33.9×33.9cm with tags mounted right at the corners.

## Directory Structure
- `bbapps/`: Contains fully working example code for the hardware. **Treat this as the source of truth** for how to interface with SLAM and basic motor commands.
- `engine/`: The chess engine logic (already completed).
- `motion/`: Holds the movement, navigation, and alignment control loops. (CURRENT FOCUS)
- `vision/`: Top camera and hand camera computer vision logic.

## AI Agent Directives
1. **Explore > Plan > Code.** Never write code blindly. Read the relevant files in `bbapps/` first to understand the existing hardware APIs, explain your plan using the `<thinking>` tag, and only write code once the plan is solid.
2. **Safety and Precision Over Speed.** Physical robots break things if alignment is off. The micro-correction alignment loop (using Top Camera + AprilTags) must be precise. The 4-DOF arms will compensate for minor errors, but the base parking needs to be highly accurate. 
3. **Reference over Reinvention.** Do not invent new PID loops or wheel command syntax if a working version already exists in `bbapps/`.
4. **Concrete Verification.** When you write a script, tell the human exactly how to test it (e.g., "Run `python -m motion.test_alignment` while holding an AprilTag in front of the top camera").

## Current Milestone
The 1-to-5 board traversal loop in `motion/` is written and committed on
branch **`motion-uv-run-fixes`** (not yet merged to `master` — check `git log`
before assuming any given robot checkout is current).
Pipeline: 
1. SLAM drives the bot roughly to the next board.
2. Top camera detects the board's 4 AprilTags.
3. Wheel motors execute micro-corrections to center the board.
4. Base locks position and hands off to the `vision` and `engine` modules for the arms to play.

**Status as of 2026-09-19:** phase 1+2 code and the traversal orchestrator are
complete; the hardware-free geometry/controller self-test
(`uv run motion/test_geometry.py`) passes; live camera + AprilTag detection is
confirmed working on `bracketbot-0185`. Two real hardware gaps were found and
handled — no SLAM daemon running (blocks phase 1 + waypoint teaching only;
alignment testing is unaffected) and no depth/stereo calibration (now falls
back to an uncalibrated guess that needs empirical tuning). Full detail,
current status, and the TODO list live in **`motion/motion.md`** — check that
file before starting new work here, it's kept current every session.