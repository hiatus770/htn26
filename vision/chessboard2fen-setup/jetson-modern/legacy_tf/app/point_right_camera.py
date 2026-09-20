#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["bbos", "numpy<2"]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/bbos", editable = true }
# ///
"""Move the right arm to a saved camera pose, capture camera.right, and infer.

The script is deliberately opt-in for physical motion: without ``--confirm``
it only prints the target. Run it on the Jetson host with ``uv run`` like the
BBOS arm examples; the chess inference itself is delegated to the TensorFlow
container after the camera capture.
"""

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from bbos import Reader, Writer, Config, Type


DEFAULT_TARGET = np.array(
    [-0.222, 0.004, 0.087, -0.203, -0.038, -0.155, 0.011, 0.279],
    dtype=np.float32,
)


def wait_for_new(reader, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if reader.ready():
            return reader.data
        time.sleep(0.01)
    raise TimeoutError(f"timed out waiting for {label}")


def load_limits(path):
    if not path.exists():
        raise FileNotFoundError(f"calibration file not found: {path}")
    with path.open() as handle:
        calibration = json.load(handle)
    return np.minimum(calibration["cal_min"], calibration["cal_max"]), np.maximum(
        calibration["cal_min"], calibration["cal_max"]
    )


def validate_target(target, lower, upper):
    outside = np.where((target < lower) | (target > upper))[0]
    if len(outside):
        details = ", ".join(
            f"J{i}={target[i]:.4f} (range {lower[i]:.4f}..{upper[i]:.4f})"
            for i in outside
        )
        raise ValueError(f"target outside calibrated limits: {details}")


def move_and_capture(args):
    app_dir = Path(args.assets).resolve()
    calibration_path = Path(args.calibration)
    target = np.asarray(args.target, dtype=np.float32)
    lower, upper = load_limits(calibration_path)
    validate_target(target, lower, upper)

    if not args.confirm:
        print("Dry run: target is within calibrated limits; pass --confirm to move the arm.")
        print(json.dumps({"target": target.tolist(), "limits": [lower.tolist(), upper.tolist()]}))
        return 0

    stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    output_dir = app_dir / "test_data" / "live_captures"
    output_dir.mkdir(parents=True, exist_ok=True)
    image_path = output_dir / f"right-camera-pose-{stamp}.jpg"
    pose_path = output_dir / f"right-camera-pose-{stamp}.json"

    config = Config("arm_right")
    if config.dof != len(target):
        raise ValueError(f"expected {config.dof} joints, received {len(target)}")

    state = Reader("arm_right.state", keeptime=False)
    camera = Reader("camera.right.jpeg", keeptime=False)
    ctrl = Writer("arm_right.ctrl", Type("arm_ctrl"), keeptime=False)
    torque = Writer("arm_right.torque", Type("arm_torque"), keeptime=False)
    torque_enabled = False
    actual = None

    def publish_position(position):
        # Match bbapps/examples/view_arms.py: publish a timestamped frame via
        # buf() so the daemon receives the same arm_ctrl updates as the editor.
        if ctrl.ready():
            with ctrl.buf() as buffer:
                buffer["pos"][:] = np.asarray(position, dtype=np.float32)

    def publish_torque(enabled):
        with torque.buf() as buffer:
            buffer["enable"][:] = np.full(config.dof, enabled, dtype=np.bool_)

    try:
        state_data = wait_for_new(state, args.timeout, "arm_right.state")
        actual = np.asarray(state_data["pos"], dtype=np.float32).copy()
        # Seed the command at the measured pose, then enable torque. This avoids
        # a sudden jump if the current arm position differs from the target.
        publish_position(actual)
        publish_torque(True)
        torque_enabled = True
        time.sleep(0.1)

        steps = max(1, int(round(args.move_seconds * args.rate)))
        for step in range(1, steps + 1):
            alpha = step / steps
            publish_torque(True)
            publish_position((1.0 - alpha) * actual + alpha * target)
            time.sleep(1.0 / args.rate)

        stable_since = None
        final_error = None
        deadline = time.monotonic() + args.settle_timeout
        while time.monotonic() < deadline:
            if state.ready():
                current = np.asarray(state.data["pos"], dtype=np.float32).copy()
                final_error = float(np.max(np.abs(current - target)))
                actual = current
                if final_error <= args.tolerance:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= args.stable_seconds:
                        break
                else:
                    stable_since = None
            time.sleep(0.01)
        else:
            errors = np.abs(actual - target) if actual is not None else np.full(config.dof, np.nan)
            details = ", ".join(f"J{i}={errors[i]:.4f}" for i in range(config.dof))
            raise TimeoutError(f"right arm did not settle; joint errors: {details}")

        camera_data = wait_for_new(camera, args.timeout, "camera.right.jpeg")
        payload = bytes(camera_data["jpeg"][: camera_data["jpeg_len"]])
        image_path.write_bytes(payload)
        pose = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "target": target.tolist(),
            "measured_after_settle": actual.tolist(),
            "max_joint_error": float(np.max(np.abs(actual - target))),
            "camera": "right",
            "image": str(image_path),
        }
        pose_path.write_text(json.dumps(pose, indent=2) + "\n")

        # Keep the arm actively holding the camera pose after capture. Publish
        # both channels throughout the hold, then the finally block drops
        # torque cleanly when the requested hold has elapsed.
        hold_deadline = time.monotonic() + args.hold_seconds
        while time.monotonic() < hold_deadline:
            publish_torque(True)
            publish_position(target)
            time.sleep(1.0 / args.rate)
    finally:
        if torque_enabled:
            with torque.buf() as buffer:
                buffer["enable"][:] = np.zeros(config.dof, dtype=np.bool_)
        for resource in (torque, ctrl, camera, state):
            resource.__exit__(None, None, None)

    command = [
        "sudo", "docker", "run", "--rm", "--runtime", "nvidia",
        "--ipc=host", "--pid=host", "--network", "none",
        "-v", "/dev/shm:/dev/shm",
        "-v", "/home/bracketbot/bbos:/opt/bbos:ro",
        "-v", f"{app_dir}:/app:ro",
        "-e", "PYTHONPATH=/opt/bbos",
        "chessvision:tf217", "/app/app/run_images.py", "--assets", "/app",
        "--input", f"/app/{image_path.relative_to(app_dir)}",
    ]
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    if result.stderr:
        print(result.stderr, file=sys.stderr, end="")
    if result.stdout:
        print(result.stdout, end="")
    print(json.dumps({"image": str(image_path), "pose": str(pose_path),
                      "hold_seconds": args.hold_seconds,
                      "pipeline_exit_code": result.returncode}), flush=True)
    return result.returncode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", type=Path, default=Path.cwd())
    parser.add_argument("--calibration", type=Path,
                        default=Path("/home/bracketbot/bbos/bbos/daemons/arm_right/ranges.calibration.json"))
    parser.add_argument("--target", type=float, nargs=8, default=DEFAULT_TARGET.tolist(),
                        metavar=("J0", "J1", "J2", "J3", "J4", "J5", "J6", "GRIPPER"))
    parser.add_argument("--confirm", action="store_true",
                        help="enable right-arm torque and move (required for physical motion)")
    parser.add_argument("--move-seconds", type=float, default=4.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--tolerance", type=float, default=0.02)
    parser.add_argument("--stable-seconds", type=float, default=0.25)
    parser.add_argument("--settle-timeout", type=float, default=15.0)
    parser.add_argument("--hold-seconds", type=float, default=5.0,
                        help="keep right-arm torque enabled after capturing the image")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    if args.move_seconds <= 0 or args.rate <= 0 or args.hold_seconds < 0:
        parser.error("--move-seconds and --rate must be positive; --hold-seconds cannot be negative")
    raise SystemExit(move_and_capture(args))


if __name__ == "__main__":
    main()
