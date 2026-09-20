//! Host-side wire primitives for the Hacker Badge display bridge.
//!
//! The wire format is deliberately smaller and simpler than the full Anchor
//! protocol: the ESP32 receives length-prefixed binary packets over one TCP
//! connection. Display packets contain only changed RGB565 rectangles.

pub mod dirty;
pub mod protocol;

// Reuse Anchor's already-tested Wayland screencopy implementation while this
// project is being developed alongside the Anchor checkout. These paths will
// become a vendored module once the bridge protocol stabilizes.
#[path = "anchorwayland.rs"]
pub mod anchorwayland;
