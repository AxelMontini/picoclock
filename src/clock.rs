use embedded_time::duration::*;
use rp_pico::hal::rtc::DateTime;

use crate::{SubState, RTC};

pub enum ClockState {
    Time {},
}

impl SubState for ClockState {
    fn update(&mut self, input: &InputState) -> Milliseconds {
        cortex_m::interrupt::free(|cs| {
            let now = RTC.borrow(cs).now().unwra;
        });

        1000.milliseconds()
    }

    fn render(&self, framebuffer: &mut crate::Framebuffer) {
        todo!()
    }
}

