#![no_std]

extern crate alloc;

#[cfg(test)]
extern crate std;

pub mod color;
pub mod config;
pub mod key_event;
pub mod layer;
pub mod parallel;
pub mod screen;
pub mod subsystem;
pub mod text_plane;
pub mod frame;

pub use color::Color;
pub use key_event::KeyEvent;
pub use layer::LayerSystem;
pub use screen::{FramebufferInfo, PixelFormat, Screen};
