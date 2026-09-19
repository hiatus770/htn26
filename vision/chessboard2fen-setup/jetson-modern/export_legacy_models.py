"""Export chessboard2fen's legacy Keras models to ONNX.

Run this only inside the legacy TensorFlow 2.2 compatibility image.  The
resulting ONNX files are architecture-neutral; TensorRT engines must still be
built on the target Jetson.
"""
from pathlib import Path
import sys

import tensorflow as tf
import tf2onnx


def export(model_path, output_path):
    model = tf.keras.models.load_model(str(model_path))
    print("{} input={} output={}".format(
        model_path.name, model.inputs, model.outputs
    ))
    model_proto, _ = tf2onnx.convert.from_keras(model, opset=13)
    output_path.write_bytes(model_proto.SerializeToString())
    print("wrote {}".format(output_path))


def main():
    repo = Path(sys.argv[1] if len(sys.argv) > 1 else "/repo")
    output_dir = Path(sys.argv[2] if len(sys.argv) > 2 else "/export")
    output_dir.mkdir(parents=True, exist_ok=True)
    export(repo / "models" / "detection", output_dir / "board-corners.onnx")
    export(repo / "models" / "classification.h5", output_dir / "piece-classifier.onnx")


if __name__ == "__main__":
    main()
