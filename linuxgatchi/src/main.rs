use std::env;
use std::io;
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::{Duration, Instant};

use badge_bridge::anchorwayland::capture_backend::CaptureBackend;
use badge_bridge::anchorwayland::input::InputBackend;
use badge_bridge::anchorwayland::screencopy::WaylandScreencopyBackend;
use badge_bridge::dirty::{SCREEN_H, SCREEN_W, dirty_rects, xrgb8888_to_rgb565};
use badge_bridge::protocol::{InputEvent, Rect, decode_input, write_frame};

const LISTEN_ADDR: &str = "0.0.0.0:8765";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    eprintln!("badge bridge: initializing Wayland capture");
    let mut capture =
        WaylandScreencopyBackend::try_new().map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    capture
        .init()
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    capture
        .prefer_cpu_capture()
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    let outputs = capture.get_outputs();
    let output_name = env::var("BADGE_OUTPUT").unwrap_or_else(|_| "HEADLESS-1".to_owned());
    let output = outputs
        .iter()
        .find(|output| output.name == output_name)
        .ok_or_else(|| {
            let names = outputs.iter().map(|output| output.name.as_str()).collect::<Vec<_>>().join(", ");
            format!("Wayland output '{output_name}' not found (available: {names})")
        })?;
    eprintln!(
        "badge bridge: {} {}x{}",
        output.name, output.width, output.height
    );

    let listener = TcpListener::bind(LISTEN_ADDR)?;
    eprintln!("badge bridge: listening on {LISTEN_ADDR}");
    for incoming in listener.incoming() {
        match incoming {
            Ok(stream) => {
                eprintln!("badge bridge: badge connected");
                if let Err(error) = stream_loop(&mut capture, output.id, stream) {
                    eprintln!("badge bridge: connection ended: {error}");
                }
            }
            Err(error) => eprintln!("badge bridge: accept failed: {error}"),
        }
    }
    Ok(())
}

fn stream_loop(
    capture: &mut WaylandScreencopyBackend,
    output_id: usize,
    mut stream: TcpStream,
) -> Result<(), Box<dyn std::error::Error>> {
    stream.set_nodelay(true)?;
    let input_stream = stream.try_clone()?;
    let (input_tx, input_rx) = mpsc::channel();
    thread::spawn(move || read_inputs(input_stream, input_tx));
    thread::spawn(move || inject_inputs(input_rx));

    let mut previous = vec![0u16; SCREEN_W * SCREEN_H];
    let mut frame_id = 0u32;
    let mut sent_rects = 0usize;
    let mut sent_bytes = 0usize;

    loop {
        let frame = capture
            .capture_frame(output_id)
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
        let cpu = frame
            .cpu_buffer
            .as_ref()
            .ok_or("capture did not return CPU pixels")?;
        let pixels = xrgb8888_to_rgb565(
            cpu.as_slice(),
            frame.stride as usize,
            frame.width as usize,
            frame.height as usize,
        );

        let rects = if frame_id == 0 {
            vec![full_rect(&pixels)]
        } else {
            dirty_rects(&previous, &pixels)
        };
        let rect_count = rects.len();
        for rect in rects {
            sent_bytes += rect.pixels.len();
            write_frame(&mut stream, frame_id, rect.rect, &rect.pixels)?;
        }
        sent_rects += rect_count;
        if frame_id == 0 || frame_id % 60 == 0 {
            eprintln!(
                "badge bridge: frame={} rects={} bytes={} total_rects={} total_pixel_bytes={}",
                frame_id, rect_count, sent_bytes, sent_rects, sent_bytes
            );
            sent_bytes = 0;
        }
        previous.copy_from_slice(&pixels);
        frame_id = frame_id.wrapping_add(1);
    }
}

fn read_inputs(mut stream: TcpStream, tx: Sender<InputEvent>) {
    loop {
        match badge_bridge::protocol::read_packet(&mut stream).and_then(|p| decode_input(&p)) {
            Ok(event) => {
                eprintln!("badge bridge: input={event:?}");
                if tx.send(event).is_err() {
                    return;
                }
            }
            Err(error) => {
                eprintln!("badge bridge: input connection ended: {error}");
                return;
            }
        }
    }
}

fn inject_inputs(rx: Receiver<InputEvent>) {
    let mut input = match badge_bridge::anchorwayland::input::WaylandInput::new() {
        Ok(input) => input,
        Err(error) => {
            eprintln!("badge bridge: Wayland input unavailable: {error}");
            return;
        }
    };
    let started = Instant::now();
    let mut held = [false; 9];
    loop {
        match rx.recv_timeout(Duration::from_millis(10)) {
            Ok(InputEvent::Button { id, pressed }) if (id as usize) < held.len() => {
                held[id as usize] = pressed;
                let time = started.elapsed().as_millis() as u32;
                match id {
                    0 => input.pointer_button(time, 272, pressed), // A: left
                    1 => input.pointer_button(time, 273, pressed), // B: right
                    2 if pressed => {
                        input.key_special(time, "Escape");
                    }
                    8 => input.pointer_button(time, 274, pressed), // Start: middle
                    _ => {}
                }
                input.flush();
            }
            Ok(InputEvent::Motion { dx, dy }) => {
                input.pointer_motion(started.elapsed().as_millis() as u32, dx as f64, dy as f64);
                input.flush();
            }
            Ok(_) => {}
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => return,
        }

        // The badge only needs to send debounced button edges. Generate
        // repeat motion here so mouse speed and acceleration stay host-side.
        let dx = i16::from(held[5]) - i16::from(held[4]); // Right - Left
        let dy = i16::from(held[3]) - i16::from(held[6]); // Down - Up
        if dx != 0 || dy != 0 {
            let speed = if held[3] || held[4] || held[5] || held[6] {
                3.0
            } else {
                0.0
            };
            input.pointer_motion(
                started.elapsed().as_millis() as u32,
                f64::from(dx) * speed,
                f64::from(dy) * speed,
            );
            input.flush();
        }
    }
}

fn full_rect(pixels: &[u16]) -> badge_bridge::dirty::DirtyRect {
    let mut wire_pixels = Vec::with_capacity(pixels.len() * 2);
    for &pixel in pixels {
        wire_pixels.extend_from_slice(&pixel.to_be_bytes());
    }
    badge_bridge::dirty::DirtyRect {
        rect: Rect {
            x: 0,
            y: 0,
            w: SCREEN_W as u16,
            h: SCREEN_H as u16,
        },
        pixels: wire_pixels,
    }
}
