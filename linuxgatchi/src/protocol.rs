//! Length-prefixed badge bridge packets.

use std::io::{self, Read, Write};

pub const MAGIC: [u8; 4] = *b"B16\0";
pub const VERSION: u8 = 1;
pub const KIND_FRAME: u8 = 1;
pub const KIND_INPUT: u8 = 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rect {
    pub x: u16,
    pub y: u16,
    pub w: u16,
    pub h: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputEvent {
    Button { id: u8, pressed: bool },
    Motion { dx: i16, dy: i16 },
}

pub fn write_frame<W: Write>(
    writer: &mut W,
    frame_id: u32,
    rect: Rect,
    pixels_rgb565_be: &[u8],
) -> io::Result<()> {
    let payload_len = 4 + 2 + 2 + 2 + 2 + pixels_rgb565_be.len();
    let length = 1 + 1 + 2 + payload_len;
    let mut packet = Vec::with_capacity(4 + length);
    packet.extend_from_slice(&(length as u32).to_be_bytes());
    packet.push(VERSION);
    packet.push(KIND_FRAME);
    packet.extend_from_slice(&0u16.to_be_bytes());
    packet.extend_from_slice(&frame_id.to_be_bytes());
    packet.extend_from_slice(&rect.x.to_be_bytes());
    packet.extend_from_slice(&rect.y.to_be_bytes());
    packet.extend_from_slice(&rect.w.to_be_bytes());
    packet.extend_from_slice(&rect.h.to_be_bytes());
    packet.extend_from_slice(pixels_rgb565_be);
    writer.write_all(&packet)
}

pub fn write_input<W: Write>(writer: &mut W, event: InputEvent) -> io::Result<()> {
    let mut packet = Vec::with_capacity(16);
    packet.extend_from_slice(&7u32.to_be_bytes());
    packet.push(VERSION);
    packet.push(KIND_INPUT);
    packet.extend_from_slice(&0u16.to_be_bytes());
    match event {
        InputEvent::Button { id, pressed } => {
            packet.push(1);
            packet.push(id);
            packet.push(u8::from(pressed));
        }
        InputEvent::Motion { dx, dy } => {
            packet[0..4].copy_from_slice(&9u32.to_be_bytes());
            packet.push(2);
            packet.extend_from_slice(&dx.to_be_bytes());
            packet.extend_from_slice(&dy.to_be_bytes());
        }
    }
    writer.write_all(&packet)
}

pub fn read_packet<R: Read>(reader: &mut R) -> io::Result<Vec<u8>> {
    let mut length = [0u8; 4];
    reader.read_exact(&mut length)?;
    let length = u32::from_be_bytes(length) as usize;
    if !(4..=512 * 1024).contains(&length) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid badge packet length",
        ));
    }
    let mut packet = vec![0u8; length];
    reader.read_exact(&mut packet)?;
    if packet[0] != VERSION {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "unsupported badge protocol version",
        ));
    }
    Ok(packet)
}

pub fn decode_input(packet: &[u8]) -> io::Result<InputEvent> {
    if packet.len() < 5 || packet[1] != KIND_INPUT {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "not an input packet",
        ));
    }
    match packet[4] {
        1 if packet.len() == 7 => Ok(InputEvent::Button {
            id: packet[5],
            pressed: packet[6] != 0,
        }),
        2 if packet.len() == 9 => Ok(InputEvent::Motion {
            dx: i16::from_be_bytes([packet[5], packet[6]]),
            dy: i16::from_be_bytes([packet[7], packet[8]]),
        }),
        _ => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "malformed input packet",
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn button_packet_round_trips() {
        let mut bytes = Vec::new();
        write_input(
            &mut bytes,
            InputEvent::Button {
                id: 5,
                pressed: true,
            },
        )
        .unwrap();
        let mut reader = Cursor::new(bytes);
        let packet = read_packet(&mut reader).unwrap();
        assert_eq!(
            decode_input(&packet).unwrap(),
            InputEvent::Button {
                id: 5,
                pressed: true
            }
        );
    }

    #[test]
    fn motion_packet_round_trips() {
        let mut bytes = Vec::new();
        write_input(&mut bytes, InputEvent::Motion { dx: -6, dy: 12 }).unwrap();
        let mut reader = Cursor::new(bytes);
        let packet = read_packet(&mut reader).unwrap();
        assert_eq!(
            decode_input(&packet).unwrap(),
            InputEvent::Motion { dx: -6, dy: 12 }
        );
    }

    #[test]
    fn frame_packet_header_matches_firmware_parser() {
        let mut bytes = Vec::new();
        write_frame(
            &mut bytes,
            42,
            Rect {
                x: 16,
                y: 32,
                w: 16,
                h: 16,
            },
            &vec![0xaa; 16 * 16 * 2],
        )
        .unwrap();
        assert_eq!(&bytes[4..8], &[VERSION, KIND_FRAME, 0, 0]);
        assert_eq!(
            u32::from_be_bytes(bytes[0..4].try_into().unwrap()),
            16 + 16 * 16 * 2
        );
        assert_eq!(u32::from_be_bytes(bytes[8..12].try_into().unwrap()), 42);
        assert_eq!(u16::from_be_bytes(bytes[12..14].try_into().unwrap()), 16);
        assert_eq!(u16::from_be_bytes(bytes[14..16].try_into().unwrap()), 32);
        assert_eq!(u16::from_be_bytes(bytes[16..18].try_into().unwrap()), 16);
        assert_eq!(u16::from_be_bytes(bytes[18..20].try_into().unwrap()), 16);
    }
}
