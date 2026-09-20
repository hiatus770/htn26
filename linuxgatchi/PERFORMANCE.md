# Linuxgatchi display performance

## Baseline before ESP32 tuning

Captured 2026-09-19 before changing the ESP32 transfer settings.

### ESP32-C3 firmware

- Target: ESP32-C3, 160 MHz, 4 MB flash
- Display: ST7789, 320x240 RGB565
- SPI host: SPI2
- SPI pins: MOSI GPIO10, CLK GPIO1, CS GPIO2, DC GPIO0, RESET GPIO4
- SPI clock: **40 MHz**
- SPI mode: 0
- LCD element order: **BGR**
- LCD inversion: **enabled** (`invert_color(true)`)
- DMA transfer size: `320 * 16 * 2` bytes
- LCD buffer: one 16-row stripe buffer
- Transfer behavior: receive one stripe, submit it, wait for DMA completion,
  then reuse the buffer
- Wi-Fi power save: ESP-IDF default

### Host bridge

- Capture: Wayland CPU screencopy
- Output: 320x240
- Pixel format: XRGB8888 to RGB565
- Transport: TCP port 8765, `TCP_NODELAY` enabled
- Dirty regions: 16x16 tiles, horizontally and vertically coalesced
- Full refresh: one 320x240 rectangle
- Unchanged frames: no rectangles sent
- Held-direction mouse speed: 3.0 pixels per 10 ms update

### Baseline observations

- Full frame payload: 153,600 pixel bytes, plus one packet header
- Unchanged frames: 0 pixel bytes sent
- The LCD bus is the likely limit for large updates.

## Test sequence

Test one change at a time and keep the display colors and stability checks:

1. Increase LCD DMA stripes from 16 to 32 rows at 40 MHz.
2. If stable, test 80 MHz SPI.
3. If needed, test double-buffered DMA transfers.
4. Optionally test disabling Wi-Fi power save, measuring latency against battery use.

Record the result and revert any setting that causes artifacts, resets, or lost
frames.

## Test result: 32-row DMA stripes

- Change: DMA transfer size and receive buffer increased from 16 to 32 rows.
- SPI clock: unchanged at 40 MHz.
- Result: build and flash succeeded; the badge booted, initialized the ST7789,
  joined Wi-Fi, and reconnected to the bridge without firmware errors.
- Visual result: compare scrolling and large screen changes on the badge before
  proceeding to the 80 MHz SPI test.

Follow-up: the 32-row setting felt less responsive during interactive use, so
the firmware was reverted to the 16-row baseline. It was not retained.

## Test result: 80 MHz SPI

- Change: ST7789 pixel clock increased from 40 MHz to 80 MHz.
- DMA stripe size: retained at 16 rows.
- This is an experimental test because 40 MHz is the documented stable speed.

Follow-up: 80 MHz caused display/connection instability, so the firmware was
reverted to the stable 40 MHz setting. It was not retained.

## Test: double-buffered 16-row DMA

- Keeps 40 MHz SPI and 16-row transfers.
- Uses two LCD DMA buffers so TCP reception and LCD transmission can overlap.
- The result should be judged by interactive responsiveness and visual
  stability; revert if tearing, corruption, or resets appear.

Follow-up: full-screen updates exposed that a binary completion semaphore could
drop a second DMA-completion callback. It was changed to a counting semaphore,
and the LCD transaction queue was increased from 4 to 10, matching Espressif's
SPI LCD guidance.
