"""Headless Jetson runner for the existing chessboard2fen models."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from board_detection import ChessboardDetector

FEN_CHARS = ["p", "P", "q", "Q", "k", "K", "e", "b", "B", "n", "N", "r", "R"]
STARTING_LAYOUT = np.array([
    [11, 9, 7, 2, 4, 7, 9, 11], [0, 0, 0, 0, 0, 0, 0, 0],
    [6, 6, 6, 6, 6, 6, 6, 6], [6, 6, 6, 6, 6, 6, 6, 6],
    [6, 6, 6, 6, 6, 6, 6, 6], [6, 6, 6, 6, 6, 6, 6, 6],
    [1, 1, 1, 1, 1, 1, 1, 1], [12, 10, 8, 3, 5, 8, 10, 12],
])


def placement_fen(predictions):
    board = predictions.reshape(8, 8)
    board = np.rot90(board, np.argmin([np.count_nonzero(STARTING_LAYOUT - np.rot90(board, i))
                                       for i in range(4)]))
    ranks = []
    for rank in board:
        text, empty = "", 0
        for value in rank:
            char = FEN_CHARS[int(value)]
            if char == "e":
                empty += 1
            else:
                text += (str(empty) if empty else "") + char
                empty = 0
        ranks.append(text + (str(empty) if empty else ""))
    return "/".join(ranks)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", type=Path, required=True,
                        help="directory containing models/detection and models/classification.h5")
    parser.add_argument("--input", type=Path, required=True, help="image file or directory")
    args = parser.parse_args()
    detector = ChessboardDetector(args.assets / "models/detection",
                                  args.assets / "models/classification.h5")
    paths = [args.input] if args.input.is_file() else sorted(args.input.glob("*"))
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        corners = detector.predict_board_corners(image)
        if len(corners) != 4:
            print(json.dumps({"file": path.name, "detected": False}))
            continue
        predictions = detector.predict_board(image, corners)
        print(json.dumps({"file": path.name, "detected": True,
                          "corners": np.asarray(corners).tolist(),
                          "placement_fen": placement_fen(predictions)}))


if __name__ == "__main__":
    main()
