use core::fmt::Write;

use arrayvec::ArrayString;
use fugit::ExtU64;
use rp_pico::hal::rtc::{DateTime, RealTimeClock};

use crate::{input::InputState, Duration, SubState, text::render_text, Position, Color};


const RAINBOW: &[Color] = &[Color::new(148, 0, 211), Color::new(75, 0, 130), Color::new()];

pub enum ClockState {
    Time { frame: u8, datetime: DateTime },
}

impl<'d> SubState<'d> for ClockState {
    type Data = (&'d InputState, &'d mut RealTimeClock);

    fn update(&mut self, data: Self::Data) -> Duration {
        let (_input, rtc) = data;

        match self {
            ClockState::Time { frame, datetime } => {
                *datetime = rtc.now().expect("get datetime");
                *frame = (frame + 1) % ;
            }
        }

        1000.millis()
    }

    fn render(&self, framebuffer: &mut crate::Framebuffer) {
        let mut txt = ArrayString::<5>::new();

        match self {
            ClockState::Time { frame, datetime } => {
                // 5 characters (`HH:mm`)
                write!(&mut txt, "{}:{}", datetime.hour, datetime.minute).expect("formatting time");
                render_text(framebuffer, txt, Position::new(2, 2), [Color::new()].into_iter().cycle())
            }
        }
    }
}
