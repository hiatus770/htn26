"""Run one complete, non-interactive chessboard2fen inference pass."""
from pathlib import Path
import sys

import cv2

def main():
    repo = Path.cwd()
    sys.path.insert(0, str(repo))
    from boardDetection import ChessboardDetector
    from chessboard import Chessboard

    image_path = sorted((repo / "input_imgs").glob("*.jpg"))[0]
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError("Could not read sample image: {}".format(image_path))

    detector = ChessboardDetector("models/detection", "models/classification.h5")
    corners = detector.predict_board_corners(image)
    if len(corners) != 4:
        raise RuntimeError("Board detection returned {} corners".format(len(corners)))

    predictions = detector.predictBoard(image, corners)
    board = Chessboard()
    try:
        layout = board.rotate_predictions(predictions)
        fen = board.predictions_to_fen(layout)
    finally:
        board.engine.quit()

    print("sample={}".format(image_path.name))
    print("corners={}".format(corners))
    print("placement_fen={}".format(fen))


if __name__ == "__main__":
    main()
