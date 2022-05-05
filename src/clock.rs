use core::fmt::Write;

use arrayvec::{ArrayString, ArrayVec};
use fugit::ExtU64;
use palette::{FromColor, Hsv, IntoColor, Srgb};
use rp_pico::hal::rtc::{DateTime, RealTimeClock};
use rtt_target::rprintln;

use crate::{
    input::InputState,
    text::{render_text, render_text_font, Font},
    Color, Draw, Duration, Framebuffer, Position, Shape, SubState,
};

pub enum ClockState {
    Time {
        frame: usize,
        datetime: DateTime,
        rainbow_steps: usize,
    },
}

/// Infinite! rainbow iterator with given color bands (`steps`), saturation and value in `Hsv` format
fn rainbow(steps: usize, saturation: f32, value: f32) -> impl Iterator<Item = Hsv> {
    (0..steps)
        .cycle()
        .map(move |i| Hsv::new(360.0 * i as f32 / steps as f32, saturation, value))
}

impl<'d> SubState<'d> for ClockState {
    type Data = (&'d InputState, &'d mut RealTimeClock);

    fn update(&mut self, data: Self::Data) -> Duration {
        let (_input, rtc) = data;

        match self {
            ClockState::Time {
                frame,
                datetime,
                rainbow_steps,
            } => {
                *datetime = rtc.now().expect("get datetime");
                *frame = (*frame + 1) % *rainbow_steps;

                2000.millis()
            }
        }
    }

    fn render(&self, framebuffer: &mut crate::Framebuffer) {
        // Clear framebuffer first, handled by renderer individually
        framebuffer
            .iter_mut()
            .for_each(|r| r.fill(Color::new(0, 0, 0)));

        match self {
            ClockState::Time {
                frame,
                datetime,
                rainbow_steps,
            } => {
                // Iterator with 4 rainbow elements and 2 white colors at the end
                let colors = rainbow(*rainbow_steps, 1.0, 0.5)
                    .skip(*frame)
                    .map(|c| Srgb::from_color(c).into_format())
                    .take(4)
                    .chain([Color::new(150, 150, 150), Color::new(150, 150, 150)]);

                render_time(framebuffer, datetime, colors);
            }
        }
    }
}

/// Renders current time simulating a segment display (big!)
/// Since the [`Framebuffer`] is 32x16, numbers can easily be 6x10 tall.
///
/// Each digit is rendered to a segment and a colon is used to separate `HH:mm`
///
/// # Color
///
/// The color iterator is used as follows: H1, H2, M1, M2, COLON_HIGH, COLON_LOW
fn render_time(fb: &mut Framebuffer, datetime: &DateTime, mut colors: impl Iterator<Item = Color>) {
    let (h1, h2) = (datetime.hour / 10, datetime.hour % 10);
    let (m1, m2) = (datetime.minute / 10, datetime.minute % 10);

    let positions: &[Position] = &[[0, 3].into(), [8, 3].into(), [18, 3].into(), [26, 3].into()];

    // Draw digits
    [h1, h2, m1, m2]
        .into_iter()
        .filter_map(Segment::from_digit)
        .zip(&mut colors)
        .zip(positions)
        .for_each(|((s, c), p)| s.render(fb, *p, &c));

    // Draw colon
    let colon_high_color = colors.next().unwrap();
    let colon_low_color = colors.next().unwrap();

    fb.draw(
        &Shape::Line([15, 10].into(), [15, 11].into()),
        &colon_high_color,
    );
    fb.draw(
        &Shape::Line([16, 10].into(), [16, 11].into()),
        &colon_high_color,
    );
    fb.draw(
        &Shape::Line([15, 5].into(), [15, 6].into()),
        &colon_low_color,
    );
    fb.draw(
        &Shape::Line([16, 5].into(), [16, 6].into()),
        &colon_low_color,
    );
}

/// Representation of a segment display (On/off for each segment).
///
/// Internally `LSB` is `a`, MSB is `g`.
struct Segment {
    code: u8,
}

impl Segment {
    pub fn from_digit(digit: u8) -> Option<Self> {
        let table: [u8; 10] = [
            0b0111111, 0b110, 0b1011011, 0b1001111, 0b1100110, 0b1101101, 0b1111101, 0b111, 0xFF,
            0b1100111,
        ];

        table.get(digit as usize).map(move |&c| Self::from_code(c))
    }

    /// Build from code directly. `LSB` is `a`, MSB is `g`
    fn from_code(code: u8) -> Self {
        Self { code }
    }

    /// Render in FB at position (from bottom-left corner of FB) with color
    pub fn render(&self, fb: &mut Framebuffer, position: Position, color: &Color) {
        // Lines of each segment (offset)
        let segments = [
            ([1, 10], [4, 10]), //A
            ([5, 9], [5, 6]),   //B
            ([5, 1], [5, 4]),   //C
            ([1, 0], [4, 0]),   //D
            ([0, 1], [0, 4]),   //E
            ([0, 6], [0, 9]),   //F
            ([1, 5], [4, 5]),   //G
        ];

        // Filter only the segments to be rendered (in order, a to g) and draw them with the given color
        segments
            .iter()
            .enumerate()
            .filter(|&(i, _)| self.is_on(i as u8))
            .map(|(_, &(a, b))| (a.into(), b.into()))
            .for_each(|(a, b): (Position, Position)| {
                fb.draw(&Shape::Line(position + a, position + b), color)
            });
    }

    /// Returns `true` whether segment with `id` (`a = 0`, `b = 1`, ...) is on, else `false`.
    pub fn is_on(&self, id: u8) -> bool {
        (self.code >> id) & 1 == 1
    }
}
