# Jetson compatibility port (least resistance)

This runs the original trained TensorFlow models directly on the Jetson GPU.
It deliberately does **not** require converting the corner-detector model to
ONNX/TensorRT. The application is headless and prints one JSON result per
image, including its FEN placement.

## Build on the Jetson

On the Jetson, first identify the L4T/JetPack release:

```bash
cat /etc/nv_tegra_release
```

Choose the NVIDIA TensorFlow container that NVIDIA publishes for that exact
JetPack release, then build (the image must be built on the ARM64 Jetson):

```bash
docker build \
  --build-arg BASE_IMAGE=<jetpack-matched-nvidia-tensorflow-image> \
  -t chessboard2fen-jetson:legacy-tf .
```

Copy or clone the legacy repository onto the Jetson. Its `models/` directory
must remain intact. Run a sample image or image directory:

```bash
./run-on-jetson.sh ~/Projects/chessboard2fen \
  ~/Projects/chessboard2fen/input_imgs
```

The port avoids `cv2.imshow()` and does not require Stockfish for static FEN
recognition. An ARM64 Stockfish package can be added later for move analysis.

## BracketBot cameras

This Jetson publishes frames through BBOS shared-memory topics, not through
`cv2.VideoCapture`. Run the live reader on the **host**, where BBOS and its
camera topics are available (rather than inside the Docker container):

```bash
python3 app/run_bbos.py --assets ~/Projects/chessboard2fen --camera head --frames 1
```

The reader uses the same interface as `bbapps/examples/view_camera.py`:
`Reader("camera.head.jpeg")`, `jpeg_len`, and `jpeg`. It processes the newest
head/left/right JPEG frame, prints JSON, and never writes to or commands the
robot. Use `--frames 0 --interval 0.5` only after validating the single-frame
result.

## Validation gate

Compare the resulting `placement_fen` values for the included images with the
legacy x86 run before attaching a live camera. The model files are unchanged,
so any mismatch indicates a TensorFlow-version or preprocessing regression.
