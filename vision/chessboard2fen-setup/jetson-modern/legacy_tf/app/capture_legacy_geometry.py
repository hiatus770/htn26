"""Capture the original OpenCV 4.4 square-crop geometry for comparison."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from boardDetection import ChessboardDetector


def digest(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def main():
    source = Path("input_imgs/inputImg01.jpg")
    image = cv2.imread(str(source))
    detector = ChessboardDetector("models/detection", "models/classification.h5")
    corners = np.asarray(detector.predict_board_corners(image), dtype=np.int32)
    detector.img_nn = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) / 255.0
    scale_factor = image.shape[1] / 512
    scaled_corners = corners * scale_factor
    _, rotation, translation = cv2.solvePnP(detector.dest_coords, scaled_corners,
                                             detector.cam_m, detector.dist_m)
    cells, points_list, rectangles, boxes, sources, destinations = [], [], [], [], [], []
    for row in range(8):
        for col in range(8):
            cell_x, cell_y = 7 - row, col
            black = np.zeros((100, 100, 1), dtype=np.uint8)
            cv2.circle(black, (5 + 10 * cell_x, 5 + 10 * cell_y), 5, 255, 0)
            low_pos = np.argwhere(black)
            up_pos = low_pos.copy()
            up_pos[:, 2] = 13
            image_points, _ = cv2.projectPoints(np.concatenate((low_pos, up_pos)).astype(np.float32),
                                                 rotation, translation, detector.cam_m, detector.dist_m)
            rectangle = cv2.minAreaRect(image_points.reshape(-1, 2))
            box = np.int0(cv2.boxPoints(rectangle))
            width, height = map(int, rectangle[1])
            source_points = box.astype(np.float32)
            destination = np.array([[0, height - 1], [0, 0], [width - 1, 0],
                                    [width - 1, height - 1]], dtype=np.float32)
            warped = cv2.warpPerspective(detector.img_nn,
                                         cv2.getPerspectiveTransform(source_points, destination),
                                         (width, height))
            if warped.shape[0] < warped.shape[1]:
                warped = cv2.rotate(warped, cv2.ROTATE_90_COUNTERCLOCKWISE)
            height, width = warped.shape[:2]
            target_width = height / 2
            if width > target_width:
                trim = round((width - target_width) / 2)
                warped = warped[:, trim:width - trim, :]
            elif width < target_width:
                target_height = width * 2
                trim = round((height - target_height) / 2)
                warped = warped[trim:height - trim, :, :]
            import imutils
            cells.append(cv2.resize(imutils.resize(warped, width=100), (100, 200)).reshape(200, 100, 3))
            points_list.append(image_points.reshape(-1, 2))
            rectangles.append([*rectangle[0], *rectangle[1], rectangle[2]])
            boxes.append(box); sources.append(source_points); destinations.append(destination)
    cells = np.asarray(cells)
    output = Path("/output/legacy-geometry-instrumented.npz")
    np.savez_compressed(output, image=image, corners=corners, cells=cells,
                        scaled_corners=scaled_corners, rotation=rotation, translation=translation,
                        image_points=np.asarray(points_list), rectangles=np.asarray(rectangles),
                        boxes=np.asarray(boxes), sources=np.asarray(sources),
                        destinations=np.asarray(destinations))
    print(json.dumps({"cells_sha256": digest(cells), "output": str(output)}))


if __name__ == "__main__":
    main()
