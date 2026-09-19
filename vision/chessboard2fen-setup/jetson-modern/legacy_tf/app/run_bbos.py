"""Read live BracketBot camera frames and run the unchanged vision pipeline."""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from bbos import Reader

from board_detection import ChessboardDetector
from run_images import placement_fen


def decode_frame(reader):
    """Decode the JPEG payload published by BBOS camera.<name>.jpeg."""
    payload = bytes(reader.data["jpeg"][:reader.data["jpeg_len"]])
    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("BBOS published an invalid JPEG frame")
    return image


def infer(detector, image, camera):
    corners = detector.predict_board_corners(image)
    result = {"camera": camera, "detected": len(corners) == 4}
    if len(corners) == 4:
        predictions = detector.predict_board(image, corners)
        result["corners"] = np.asarray(corners).tolist()
        result["placement_fen"] = placement_fen(predictions)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", type=Path, required=True,
                        help="directory containing models/detection and models/classification.h5")
    parser.add_argument("--camera", choices=("head", "left", "right"), default="head")
    parser.add_argument("--frames", type=int, default=0,
                        help="number of frames to process; 0 runs continuously")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="minimum seconds between inferences")
    args = parser.parse_args()
    if args.frames < 0 or args.interval < 0:
        parser.error("--frames and --interval must be non-negative")

    detector = ChessboardDetector(args.assets / "models/detection",
                                  args.assets / "models/classification.h5")
    source = "camera.{}.jpeg".format(args.camera)
    completed, last_inference = 0, 0.0
    with Reader(source) as reader:
        while args.frames == 0 or completed < args.frames:
            if not reader.ready():
                time.sleep(0.002)
                continue
            now = time.monotonic()
            if now - last_inference < args.interval:
                continue
            last_inference = now
            try:
                print(json.dumps(infer(detector, decode_frame(reader), args.camera)), flush=True)
            except Exception as error:
                print(json.dumps({"camera": args.camera, "error": str(error)}), flush=True)
            completed += 1


if __name__ == "__main__":
    main()
