# Hacker Badge native bridge

This crate contains the first host-side transport primitives for the native
badge display project.

The intended pipeline is:

`Anchor Wayland screencopy -> downscale 320x240 -> RGB565 -> dirty 16x16 tiles -> TCP -> ESP32-C3 -> ST7789`

The bridge deliberately does not send H.264. The badge receives binary,
length-prefixed packets. Input packets travel in the reverse direction.

Button IDs are `A=0, B=1, HOME=2, DOWN=3, LEFT=4, RIGHT=5, UP=6,
AUX1=7, START=8`. The host maps A/B to left/right click, HOME to Escape,
START to middle click, and generates repeated mouse movement from held
direction buttons. The badge firmware only needs to send debounced button-edge
packets when a state changes.

The executable wires Anchor's reusable CPU buffer into
`xrgb8888_to_rgb565()` and sends the resulting dirty rectangles with
`write_frame()`. It also reuses Anchor's virtual pointer and keyboard input
backend.

For now the crate references the checked-out Anchor source at
`/home/hiatus/Projects/anchor`; that is intentional during bring-up so the
Wayland implementation is not duplicated. The next cleanup is to vendor this
small capture/input module set or make it a shared Anchor library.

## Badge firmware

The ESP-IDF project is in `badge-firmware/`. Configure it locally; do not
commit `sdkconfig` because it contains the Wi-Fi password:

```sh
. ~/.espressif/tools/activate_idf_v5.5.3.sh
cd badge-firmware
idf.py set-target esp32c3
idf.py menuconfig
```

Set `Badge bridge -> Wi-Fi password`, `Bridge host IPv4 address` to the
computer running this bridge, and leave the SSID as `nghs8133` if that is the
hotspot being used. Then build and flash:

```sh
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
```

The badge uses native USB-Serial-JTAG for logs. Close the host bridge's serial
tools before flashing or monitoring. If download mode is required, hold Start
(GPIO9) while plugging in USB, then release it and run the flash command.
