"""Save deterministic intermediate data for cross-runtime compatibility checks."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from board_detection import ChessboardDetector
from run_images import placement_fen


def digest(array):
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    image = cv2.imread(str(args.input))
    if image is None:
        raise ValueError("could not read {}".format(args.input))
    detector = ChessboardDetector(args.assets / "models/detection",
                                  args.assets / "models/classification.h5")
    corners = np.asarray(detector.predict_board_corners(image), dtype=np.int32)
    if len(corners) != 4:
        raise RuntimeError("board detector returned {} corners".format(len(corners)))
    cells = detector.extract_cells(image, corners)
    confidence = detector.classification_model.predict(cells, batch_size=8, verbose=0)
    filtered = detector._filter_predictions(confidence)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, image=image, corners=corners, cells=cells,
                        scaled_corners=detector.last_scaled_corners,
                        rotation=detector.last_rotation,
                        translation=detector.last_translation,
                        confidence=confidence, filtered=filtered,
                        cell_coordinates=np.asarray([item["cell"] for item in detector.last_cell_debug]),
                        image_points=np.asarray([item["image_points"] for item in detector.last_cell_debug]),
                        rectangles=np.asarray([item["rectangle"] for item in detector.last_cell_debug]),
                        boxes=np.asarray([item["box"] for item in detector.last_cell_debug]),
                        sources=np.asarray([item["source"] for item in detector.last_cell_debug]),
                        destinations=np.asarray([item["destination"] for item in detector.last_cell_debug]))
    print(json.dumps({
        "corners": corners.tolist(),
        "image_sha256": digest(image),
        "cells_sha256": digest(cells),
        "confidence_sha256": digest(confidence),
        "raw_labels": np.argmax(confidence, axis=1).tolist(),
        "placement_fen": placement_fen(filtered),
        "output": str(args.output),
    }), flush=True)


if __name__ == "__main__":
    main()
