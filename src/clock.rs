use fugit::ExtU64;

use crate::{SubState, input::InputState, Duration};

pub enum ClockState {
    Time {
        frame: u8,
    },
}

impl SubState for ClockState {
    fn update(&mut self, input: &InputState) -> Duration {
        1000.millis()
    }

    fn render(&self, framebuffer: &mut crate::Framebuffer) {
        todo!()
    }
}

