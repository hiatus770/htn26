"""Emit original x86 TensorFlow/OpenCV 4.4 results for regression comparison."""
import json
from pathlib import Path

import cv2
import numpy as np

from boardDetection import ChessboardDetector

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
    directory = Path("input_imgs")
    detector = ChessboardDetector("models/detection", "models/classification.h5")
    for path in sorted(directory.iterdir()):
        image = cv2.imread(str(path))
        if image is None:
            continue
        corners = detector.predict_board_corners(image)
        result = {"file": path.name, "detected": len(corners) == 4}
        if result["detected"]:
            result["corners"] = np.asarray(corners).tolist()
            result["placement_fen"] = placement_fen(detector.predictBoard(image, corners))
        print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
