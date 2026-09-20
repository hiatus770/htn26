# Jetson compatibility port (least resistance)

This runs the original trained TensorFlow models directly on the Jetson GPU.
It deliberately does **not** require converting the corner-detector model to
ONNX/TensorRT. The application is headless and prints one JSON result per
image, including its FEN placement.

## Build on the Jetson

The current Jetson is reachable at `192.168.109.190`. Run the commands below
on that host (for example, `ssh bracketbot@192.168.109.190`) unless noted
otherwise.

On the Jetson, first identify the L4T/JetPack release:

```bash
cat /etc/nv_tegra_release
```

This deployed Jetson is JetPack 6.2 / L4T R36.4.3. NVIDIA supports the ARM64
iGPU TensorFlow 2.17 container `nvcr.io/nvidia/tensorflow:25.02-tf2-py3-igpu`
for this JetPack release. Build on the ARM64 Jetson:

```bash
cd ~/htn26/vision/chessvision
sudo docker build -t chessvision:tf217 .
```

Copy or clone the legacy repository onto the Jetson. Its `models/` directory
must remain intact. Run a sample image or image directory:

```bash
sudo docker run --rm --runtime nvidia --network none \
  -v "$HOME/htn26/vision/chessvision:/app:ro" \
  chessvision:tf217 /app/app/run_images.py \
  --assets /app --input /app/test_data/inputImg01.jpg
```

The deployed repository test image is expected to print:

```text
rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR
```

The port avoids `cv2.imshow()` and does not require Stockfish for static FEN
recognition. An ARM64 Stockfish package can be added later for move analysis.

## BracketBot cameras

This Jetson publishes frames through BBOS shared-memory topics, not through
`cv2.VideoCapture`. The provided launcher mounts the host's `/dev/shm` and the
read-only BBOS Python package into the TensorFlow container:

```bash
./run-bbos-container.sh /home/bracketbot/htn26/vision/chessvision head 1
```

The reader uses the same interface as `bbapps/examples/view_camera.py`:
`Reader("camera.head.jpeg")`, `jpeg_len`, and `jpeg`. It processes the newest
head/left/right JPEG frame, prints JSON, and never writes to or commands the
robot. BBOS needs shared `/dev/shm` for its time-log bookkeeping; the launcher
allows that IPC bookkeeping but mounts the application and model files
read-only. Use `--frames 0 --interval 0.5` only after validating the
single-frame result. The container has no network access at runtime and does
not include any robot-control writer.

## Move, capture, and infer

After the dry run reports a valid target, the combined command moves the right
arm to the camera pose, captures `camera.right`, writes the image and pose
metadata under `test_data/live_captures/`, and sends that image through the
same legacy TensorFlow pipeline:

```bash
uv run app/point_right_camera.py --assets .
uv run app/point_right_camera.py --assets . --confirm
```

The first command is a no-motion validation. `--confirm` is required for
physical movement. BBOS remains local shared-memory IPC on the Jetson; the IP
address is only for reaching the host over SSH.

## Validation gate

Compare the resulting `placement_fen` values for the included images with the
legacy x86 run before attaching a live camera. The model files are unchanged.
The port explicitly normalizes OpenCV's rotated-rectangle vertex order before
each perspective warp: OpenCV 4.11 changed that order from OpenCV 4.4, which
otherwise makes the legacy classifier receive different square crops.
