use core::fmt::Write;

use arrayvec::ArrayString;
use fugit::ExtU64;
use palette::{FromColor, Hsv, Srgb};
use rp_pico::hal::rtc::{DateTime, RealTimeClock};

use crate::{
    input::InputState, text::render_text, Color, Draw, Duration, Framebuffer, LcdConnection,
    Position, Shape, SubState,
};

pub(crate) const RAINBOW_STEPS: usize = 200;

/// Switch between states with Ok (deeper) or X (go to parent).
///
/// MainMenu -> Time -> Settings
///
pub enum ClockState {
    Time {
        frame: usize,
        datetime: DateTime,
    },
    Settings {
        step: EditStep,
        /// Datetime being edited
        edit: DateTime,
    },
}

#[derive(Copy, Clone)]
pub enum EditStep {
    Year,
    Month,
    Day,
    Hour,
    Minute,
    Save,
    Abort,
}

impl EditStep {
    pub fn go_next(&mut self) {
        *self = match self {
            EditStep::Year => EditStep::Month,
            EditStep::Month => EditStep::Day,
            EditStep::Day => EditStep::Hour,
            EditStep::Hour => EditStep::Minute,
            EditStep::Minute => EditStep::Save,
            ref x => **x,
        };
    }

    pub fn go_prev(&mut self) {
        *self = match self {
            EditStep::Year => EditStep::Abort,
            EditStep::Month => EditStep::Year,
            EditStep::Day => EditStep::Month,
            EditStep::Hour => EditStep::Day,
            EditStep::Minute => EditStep::Hour,
            ref x => **x,
        };
    }
}

/// Infinite! rainbow iterator with given color bands (`steps`), saturation and value in `Hsv` format
fn rainbow(steps: usize, saturation: f32, value: f32) -> impl Iterator<Item = Hsv> {
    (0..steps)
        .cycle()
        .map(move |i| Hsv::new(360.0 * i as f32 / steps as f32, saturation, value))
}

impl<'d> SubState<'d> for ClockState {
    type Data = (
        &'d InputState,
        &'d mut RealTimeClock,
        &'d mut LcdConnection,
        fn(u64),
    );

    fn update(&mut self, data: Self::Data) -> Duration {
        let (input, rtc, lcd, delay_us) = data;

        let mut driver = lcd.driver(delay_us);

        match self {
            ClockState::Time { frame, datetime } => {
                let txt = {
                    let mut s = arrayvec::ArrayString::<5>::new();
                    write!(&mut s, "{:02}:{:02}", datetime.hour, datetime.minute).unwrap();

                    s
                };

                if *frame == 0 {
                    driver.clear().unwrap();
                } else {
                    driver.return_home().unwrap();
                }

                driver.write_text(&txt).unwrap();

                *datetime = rtc.now().expect("get datetime");
                *frame = (*frame + 1) % RAINBOW_STEPS;

                match input {
                    x if x.is_just_back() => todo!("Main Menu?"),
                    x if x.is_just_confirm() => {
                        let new_state = ClockState::Settings {
                            step: EditStep::Year,
                            edit: rtc.now().unwrap(),
                        };

                        let _ = core::mem::replace(self, new_state);

                        2000.millis()
                    }
                    _ => 2000.millis(),
                }
            }
            ClockState::Settings { step, edit } => {
                // Set and go next, go back, or change value
                match input {
                    x if x.is_just_back() => step.go_prev(),
                    x if x.is_just_confirm() => step.go_next(),
                    x if x.is_just_left() || x.is_just_right() => {
                        let change = i32::from(x.is_just_right()) - i32::from(x.is_just_left()); // 1 or -1 (or 0, if both pressed)

                        match step {
                            EditStep::Year => edit.year = (edit.year as i32 + change) as _,
                            EditStep::Month => {
                                edit.month = ((edit.month as i32 + change) as u8).clamp(1, 12)
                            }
                            EditStep::Day => {
                                edit.day = ((edit.day as i32 + change) as u8).clamp(1, 31)
                            }
                            EditStep::Hour => {
                                edit.hour = ((edit.hour as i32 + change) as u8).clamp(0, 23)
                            }
                            EditStep::Minute => {
                                edit.minute = ((edit.minute as i32 + change) as u8).clamp(0, 59)
                            }
                            _ => (),
                        };
                    }
                    _ => (),
                };

                // Handle exit cases and render lcd value
                driver.clear().unwrap();
                delay_us(3000);

                let mut txt = ArrayString::<16>::new();

                let _ = match step {
                    EditStep::Year => write!(&mut txt, "Year: {}", edit.year).unwrap(),
                    EditStep::Month => write!(&mut txt, "Month: {}", edit.month).unwrap(),
                    EditStep::Day => write!(&mut txt, "Day: {}", edit.day).unwrap(),
                    EditStep::Hour => write!(&mut txt, "Hour: {}", edit.hour).unwrap(),
                    EditStep::Minute => write!(&mut txt, "Minute: {}", edit.minute).unwrap(),
                    EditStep::Abort => {
                        let _ = core::mem::replace(
                            self,
                            ClockState::Time {
                                datetime: rtc.now().unwrap(),
                                frame: 0,
                            },
                        );
                        txt.write_str("ABORT!").unwrap();
                    } // Go back to clock without applying changes
                    EditStep::Save => {
                        let old = core::mem::replace(
                            self,
                            ClockState::Time {
                                datetime: rtc.now().unwrap(),
                                frame: 0,
                            },
                        ); // Save and go back to clock after applying changes
                        match old {
                            ClockState::Settings { edit, .. } => rtc.set_datetime(edit).unwrap(),
                            _ => unreachable!("old is always Settings"),
                        }
                    }
                };

                driver.write_text(&txt).unwrap();

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
            ClockState::Time { frame, datetime } => {
                // value based on time of the day
                let ang = datetime.hour as f32 * core::f32::consts::PI / 12.0;
                let value = 0.2 + (1.0 - libm::cosf(ang)) * 0.2;

                let color_colon = Hsv::new(0.0, 0.0, value);

                // Iterator with 4 rainbow elements and 2 white colors at the end
                let colors = rainbow(RAINBOW_STEPS, 1.0, value)
                    .skip(*frame)
                    .take(4)
                    .chain([color_colon; 2])
                    .map(|c| Srgb::from_color(c).into_format());

                render_time(framebuffer, datetime, colors);
            }
            ClockState::Settings { step, .. } => {
                let colors = [Color::new(0, 255, 0)].into_iter().cycle();

                let txt = match step {
                    EditStep::Year => "SETYEAR",
                    EditStep::Month => "SETMONTH",
                    EditStep::Day => "SETDAY",
                    EditStep::Hour => "SETHOUR",
                    EditStep::Minute => "SETMINUTE",
                    EditStep::Save => "SAVE",
                    EditStep::Abort => "ABORT",
                };

                render_text(framebuffer, txt, [0, 0].into(), colors)
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
