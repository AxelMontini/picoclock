use embedded_time::duration::*;
use rp_pico::hal::rtc::DateTime;

use crate::{SubState, input::InputState};

pub enum ClockState {
    Time {
        frame: u8,
    },
}

impl SubState for ClockState {
    fn update(&mut self, input: &InputState) -> Milliseconds {
        1000.milliseconds()
    }

    fn render(&self, framebuffer: &mut crate::Framebuffer) {
        todo!()
    }
}

