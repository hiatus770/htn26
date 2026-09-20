import cv2
import imutils
import numpy as np
import tensorflow as tf
from scipy.spatial.distance import cdist


class ChessboardDetector:
    """Compatibility wrapper around the original two trained models."""

    cam_m = np.array([[3.13479737e3, 0.0, 2.04366415e3],
                      [0.0, 3.13292625e3, 1.50698424e3],
                      [0.0, 0.0, 1.0]])
    dist_m = np.array([[2.08959569e-1, -9.49127601e-1, -2.70203242e-3,
                        -1.20066339e-4, 1.33323676]])
    max_fig = [8, 8, 1, 1, 1, 1, np.inf, 2, 2, 2, 2, 2, 2]
    dest_coords = np.array([[0, 80, 0], [80, 80, 0], [80, 0, 0], [0, 0, 0]],
                           dtype=np.float32)

    def __init__(self, detector_model, classifier_model):
        gpus = tf.config.list_physical_devices("GPU")
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        self.detection_model = tf.keras.models.load_model(detector_model)
        self.classification_model = tf.keras.models.load_model(classifier_model)

    def predict_board_corners(self, image):
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("image must be a BGR color image")
        self.img_rgb = cv2.cvtColor(cv2.resize(image, (512, 384)), cv2.COLOR_BGR2RGB)
        predictions = self.detection_model.predict(
            np.expand_dims(self.img_rgb, axis=0), verbose=0
        )
        if self._overlapping(predictions):
            predictions = self._rotate_and_predict(30)
        return [] if predictions is None else self._refine(predictions)

    def predict_board(self, image, corners):
        cells = self.extract_cells(image, corners)
        confidence = self.classification_model.predict(cells, batch_size=8, verbose=0)
        self.last_confidence = confidence
        return self._filter_predictions(confidence)

    def extract_cells(self, image, corners):
        """Return the 64 classifier inputs, preserving the legacy crop geometry."""
        self.img_nn = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) / 255.0
        scale_factor = image.shape[1] / 512
        scaled_corners = np.asarray(corners) * scale_factor
        _, rotation, translation = cv2.solvePnP(
            self.dest_coords, scaled_corners, self.cam_m, self.dist_m
        )
        self.last_scaled_corners = scaled_corners
        self.last_rotation = rotation
        self.last_translation = translation
        self.last_cell_debug = []
        cells = [self._cell_image(7 - row, col, rotation, translation)
                 for row in range(8) for col in range(8)]
        return np.asarray(cells)

    def _filter_predictions(self, confidence):
        layout = -np.ones(64, dtype=np.int32)
        for index in np.argsort(-np.amax(confidence, axis=-1)):
            used = dict(zip(*np.unique(layout, return_counts=True)))
            for candidate in np.argsort(-confidence[index]):
                if used.get(candidate, 0) < self.max_fig[candidate]:
                    layout[index] = candidate
                    break
        return layout

    def _cell_image(self, cell_x, cell_y, rotation, translation):
        black = np.zeros((100, 100, 1), dtype=np.uint8)
        cv2.circle(black, (5 + 10 * cell_x, 5 + 10 * cell_y), 5, 255, 0)
        low_pos = np.argwhere(black)
        up_pos = low_pos.copy()
        up_pos[:, 2] = 13
        points = np.concatenate((low_pos, up_pos)).astype(np.float32)
        image_points, _ = cv2.projectPoints(points, rotation, translation, self.cam_m, self.dist_m)
        rectangle = cv2.minAreaRect(image_points.reshape(-1, 2))
        # OpenCV 4.4 returned this box one vertex earlier and with its dimensions
        # exchanged.  The original model pipeline implicitly relied on that order
        # when assigning the four destination points.  Normalize OpenCV >= 4.5 to
        # the legacy convention before constructing the perspective transform.
        box = np.roll(cv2.boxPoints(rectangle), 1, axis=0).astype(np.int32)
        height, width = map(int, rectangle[1])
        source = box.astype(np.float32)
        destination = np.array([[0, height - 1], [0, 0], [width - 1, 0],
                                [width - 1, height - 1]], dtype=np.float32)
        warped = cv2.warpPerspective(self.img_nn, cv2.getPerspectiveTransform(source, destination),
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
        self.last_cell_debug.append({
            "cell": np.array([cell_x, cell_y], dtype=np.int32),
            "image_points": image_points.reshape(-1, 2),
            "rectangle": np.array([*rectangle[0], *rectangle[1], rectangle[2]], dtype=np.float64),
            "box": box,
            "source": source,
            "destination": destination,
        })
        return cv2.resize(imutils.resize(warped, width=100), (100, 200)).reshape(200, 100, 3)

    def _refine(self, predictions):
        points = predictions[0, :, :2].astype(np.int32)
        hull = cv2.convexHull(points, clockwise=False)[:, 0]
        return list(np.roll(hull, -np.argmin(hull.sum(axis=1)[:4]), axis=0))

    def _rotate_and_predict(self, angle):
        predictions = self.detection_model.predict(
            np.expand_dims(imutils.rotate(self.img_rgb, angle=-angle), axis=0), verbose=0
        )
        if self._overlapping(predictions):
            return None
        matrix = cv2.getRotationMatrix2D((256, 192), angle, 1)
        predictions[0, :, 2] = 1
        predictions[0, :, :2] = (matrix @ predictions[0].T).T
        return predictions

    @staticmethod
    def _overlapping(predictions):
        points = predictions[0, :4, :2]
        distances = np.triu(cdist(points, points))
        distances[distances == 0] = np.inf
        return bool(np.argwhere(distances < 30).size)
