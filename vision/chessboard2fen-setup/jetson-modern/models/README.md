# Export status

`piece-classifier.onnx` is a structurally valid ONNX export of the original
`classification.h5` model. It is retained for a later TensorRT optimization.

`board-corners.onnx` is kept only as diagnostic output and must **not** be used:
the original detector contains TensorFlow `FFT2D`, `IFFT2D`, and complex-number
operations that are not standard ONNX/TensorRT operators. The least-resistance
Jetson port therefore loads `models/detection` through TensorFlow directly.
