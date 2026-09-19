# chessboard2fen local setup

The upstream code was cloned to `~/Projects/chessboard2fen`.  Its pinned
TensorFlow 2.2 stack requires Python 3.7, so this directory provides an
isolated Docker setup rather than changing the host's Python 3.14 packages.

Build the image from this directory:

```bash
docker --context default build -t chessboard2fen:tf2.2 .
```

Run a complete, non-interactive inference against one included photo:

```bash
./run-smoke-test.sh
```

The upstream interactive viewer blocks for a key press on every image and
needs access to an X11 display.  Run it only from a desktop session with an
explicit display mount, for example (X11/Xwayland):

```bash
xhost +si:localuser:root
docker --context default run --rm -it \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$HOME/Projects/chessboard2fen:/repo:ro" \
  chessboard2fen:tf2.2 python detectionScript.py
xhost -si:localuser:root
```

`engine/stockfish` is already included upstream and is needed because the
application launches it during startup, even for static images.
