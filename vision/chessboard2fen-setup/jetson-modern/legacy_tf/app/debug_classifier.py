"""Classify saved crop tensors to separate classifier and geometry regressions."""
import argparse
import json
from pathlib import Path

import numpy as np
import tensorflow as tf

from board_detection import ChessboardDetector
from run_images import placement_fen


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", type=Path, required=True)
    parser.add_argument("--cells", type=Path, required=True)
    args = parser.parse_args()
    cells = np.load(args.cells)["cells"]
    model = tf.keras.models.load_model(args.assets / "models/classification.h5")
    confidence = model.predict(cells, batch_size=8, verbose=0)
    layout = ChessboardDetector._filter_predictions(
        ChessboardDetector.__new__(ChessboardDetector), confidence
    )
    print(json.dumps({
        "raw_labels": np.argmax(confidence, axis=1).tolist(),
        "placement_fen": placement_fen(layout),
    }), flush=True)


if __name__ == "__main__":
    main()
