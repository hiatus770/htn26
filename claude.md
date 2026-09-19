# Project Context: Bracket Bot (5-Board Simul Chess Robot)
This repository contains the software stack for a physical robotics project: a tall, self-balancing (segway-style with training wheels) robot that plays simultaneous chess against 5 physical boards lined up on a table. 

## Hardware & Perception Assumptions
- **Base:** Segway-style two-wheel drive with training wheels (for stability).
- **Arms:** Two 4-DOF arms with shoulder joints that can move vertically. Claws have cameras for piece placing.
- **Top Camera:** Mounted at the top, angled down at 45 degrees. Used for board layout CV and reading AprilTags.
- **Localization (Global):** SLAM model maps the environment and knows rough coordinates of the table.
- **Localization (Local):** 4 AprilTags on the corners of each physical chessboard.

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
We are writing the 1-to-5 board traversal loop in the `motion/` folder. 
Pipeline: 
1. SLAM drives the bot roughly to the next board.
2. Top camera detects the board's 4 AprilTags.
3. Wheel motors execute micro-corrections to center the board.
4. Base locks position and hands off to the `vision` and `engine` modules for the arms to play.