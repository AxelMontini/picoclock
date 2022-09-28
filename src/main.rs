#![no_std]
#![no_main]

use core::fmt::{self};
use fugit::ExtU64;

use clock::ClockState;

// The macro for our start-up function

use input::InputState;
use nalgebra::Vector2;
use palette::rgb;
use panic_probe as _; // needed for panic probe stuff

use rp_pico::hal::rtc::DateTime;
use rp_pico::hal::{
    self,
    gpio::Pin,
    pac::{self},
    prelude::*,
};

use hal::rtc::RealTimeClock;
use rtic::Monotonic;
use rtt_target::{rprint, rprintln, rtt_init_print};
use snake::SnakeState;
use systick_monotonic::Systick;

mod clock;
mod input;
mod snake;
mod text;

#[link_section = ".boot_loader"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

type RightButtonPin = Pin<hal::gpio::bank0::Gpio18, hal::gpio::Input<hal::gpio::PullDown>>;
type LeftButtonPin = Pin<hal::gpio::bank0::Gpio17, hal::gpio::Input<hal::gpio::PullDown>>;
type BackButtonPin = Pin<hal::gpio::bank0::Gpio15, hal::gpio::Input<hal::gpio::PullDown>>;
type ConfirmButtonPin = Pin<hal::gpio::bank0::Gpio16, hal::gpio::Input<hal::gpio::PullDown>>;

pub(crate) type MonoSystick = Systick<1000>;
pub(crate) type Instant = <crate::MonoSystick as Monotonic>::Instant;
pub(crate) type Duration = <crate::MonoSystick as Monotonic>::Duration;
pub(crate) type Color = rgb::Srgb<u8>;

pub(crate) type LcdConnection = hd44780u::LcdConnection<
    hal::gpio::DynPin,
    hal::gpio::DynPin,
    hal::gpio::DynPin,
    hal::gpio::DynPin,
    hal::gpio::DynPin,
    hal::gpio::DynPin,
    hal::gpio::DynPin,
    hal::gpio::DynPin,
    hal::gpio::DynPin,
    hal::gpio::DynPin,
>;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [TIMER_IRQ_0, TIMER_IRQ_1])]
mod app {
    use super::*;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use embedded_time::fixed_point::FixedPoint;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = MonoSystick;

    #[local]
    struct Local {
        tx: hal::pio::Tx<(pac::PIO0, hal::pio::SM0)>,
        right_button: RightButtonPin,
        left_button: LeftButtonPin,
        confirm_button: ConfirmButtonPin,
        back_button: BackButtonPin,
    }

    #[shared]
    struct Shared {
        input_state: InputState,
        //alarm0: hal::timer::Alarm0,
        timer: hal::Timer,
        rtc: RealTimeClock,
        state: State,
        framebuffer: Framebuffer,
        lcd: crate::LcdConnection,
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        rprintln!("idle");

        loop {
            cortex_m::asm::nop() // Do nothing
        }
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Print to pico probe
        rtt_init_print!();

        // // Grab our singleton objects                        // Now in ctx
        // let mut pac = pac::Peripherals::take().unwrap();
        // let core = pac::CorePeripherals::take().unwrap();

        let mut pac = ctx.device;
        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);
        let mut mono = MonoSystick::new(ctx.core.SYST, rp_pico::XOSC_CRYSTAL_FREQ * 10);

        // Configure the clocks
        //
        // The default is to generate a 125 MHz system clock
        let clocks = hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // timer for periodic tasks, interrupts, ...
        let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
        let _alarm3 = timer.alarm_3().unwrap();

        // Real time clock should be initialized with default values,
        // we can then tune it through the Clock app
        let rtc =
            hal::rtc::RealTimeClock::new(pac.RTC, clocks.rtc_clock, &mut pac.RESETS, INITIAL_DATE)
                .unwrap();

        // The single-cycle I/O block controls our GPIO pins
        let sio = hal::sio::Sio::new(pac.SIO);

        // Set the pins up according to their function on this particular board
        let pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // Set the LED to be a PIO pin
        let _led_pin = pins.gpio19.into_mode::<hal::gpio::FunctionPio0>();
        let led_pin_id = 19;

        let side_set = pio::SideSet::new(false, 1, false);
        let mut a = pio::Assembler::new_with_side_set(side_set);

        const T1: u8 = 2; // start bit
        const T2: u8 = 2; // data bit
        const T3: u8 = 2; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;
        const FREQ: u32 = 800_000;

        // Configure the PIO state machine.
        let div =
            clocks.system_clock.freq().to_Hz() as f32 / (FREQ as f32 * CYCLES_PER_BIT as f32);

        // PIO Program for the leds
        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_zero = a.label();
        a.bind(&mut wrap_target);
        // Do stop bit
        a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
        // Do start bit
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
        // Do data bit = 1
        a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
        a.bind(&mut do_zero);
        // Do data bit = 0
        a.nop_with_delay_and_side_set(T2 - 1, 0);
        a.bind(&mut wrap_source);
        let program = a.assemble_with_wrap(wrap_source, wrap_target);

        // Initialize and start PIO
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let installed = pio.install(&program).unwrap();

        let (mut sm, _, tx) = hal::pio::PIOBuilder::from_program(installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_shift_direction(hal::pio::ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(24)
            .side_set_pin_base(led_pin_id)
            .clock_divisor(div)
            .build(sm0);
        // The GPIO pin needs to be configured as an output.
        sm.set_pindirs([(led_pin_id, hal::pio::PinDir::Output)]);
        sm.start();

        // Initial state
        let state = State::Clock(ClockState::Time {
            frame: 0,
            datetime: INITIAL_DATE,
        });

        // Control buttons (for interface).
        let left_button = pins.gpio17.into_pull_down_input();
        let right_button = pins.gpio18.into_pull_down_input();
        let confirm_button = pins.gpio16.into_pull_down_input();
        let back_button = pins.gpio15.into_pull_down_input();

        let input_state = InputState::new(mono.now()); // initial input last debounce

        let framebuffer = Framebuffer::default();

        let lcd = {
            let db7 = pins.gpio2.into_push_pull_output().into();
            let db6 = pins.gpio3.into_push_pull_output().into();
            let db5 = pins.gpio4.into_push_pull_output().into();
            let db4 = pins.gpio5.into_push_pull_output().into();
            let db3 = pins.gpio6.into_push_pull_output().into();
            let db2 = pins.gpio7.into_push_pull_output().into();
            let db1 = pins.gpio8.into_push_pull_output().into();
            let db0 = pins.gpio9.into_push_pull_output().into();
            let e = pins.gpio10.into_push_pull_output().into();
            let rs = pins.gpio11.into_push_pull_output().into();

            hd44780u::LcdConnection::new(rs, e, db0, db1, db2, db3, db4, db5, db6, db7)
        };

        let shared = Shared {
            timer,
            input_state,
            state,
            framebuffer,
            rtc,
            lcd,
        };

        let local = Local {
            tx,
            left_button,
            right_button,
            confirm_button,
            back_button,
        };

        lcd_init::spawn_after(100.millis()).expect("Ho test exp");
        // start update once
        update::spawn_after(2000.millis(), false).expect("update for the first time");
        debounce_input::spawn().expect("debouncing first time");

        (shared, local, init::Monotonics(mono))
    }

    #[task(priority = 1, shared = [lcd])]
    fn lcd_init(mut ctx: lcd_init::Context) {
        ctx.shared.lcd.lock(|lcd| {
            let delay_us = |us: u64| {
                let end = monotonics::now() + us.micros();
                while end > monotonics::now() {
                    cortex_m::asm::nop(); // busy wait
                }
            };

            let mut driver = lcd.driver(delay_us);

            driver.set_function().expect("lcd function");
            delay_us(20000);
            driver.set_function().expect("lcd function");
            delay_us(20000);
            driver.set_function().expect("lcd function");
            delay_us(20000);
            driver
                .set_display_control(true, true, true)
                .expect("lcd display control");
            delay_us(20000);
            driver.clear().expect("Clear display");
            delay_us(20000);
            driver.write_text("Booted!").expect("Write text");
            driver.set_line1().expect("Set line");
            driver.write_text("Clock is starting").expect("Write text");
        });
    }

    #[task(priority = 2, shared = [framebuffer], local = [tx])]
    fn draw(mut ctx: draw::Context) {
        cortex_m::interrupt::free(|_cs| {
            // An interrupt should never stop this, since delays in the signal could be interpreted as "stop" by the leds.
            let tx = ctx.local.tx;
            ctx.shared.framebuffer.lock(|framebuffer| {
                // drawing takes a while, the FIFO queue must be written to only when it's got space
                // Also the framebuffer rows are contiguous, but the matrices are connected in series
                // (writing to the first pixel of the second matrix requires all the pixels of the first
                // matrix to be written to)
                // So two loops are required (there could be a way to make it smarter?)
                // Write Matrix 1 (first 16 leds per row)
                framebuffer.iter().enumerate().for_each(|(i, row)| {
                    let row = &row[..16];
                    if i & 1 == 1 {
                        let row_iter = row.iter().rev();
                        row_iter.for_each(|color| {
                            while !tx.write(to_word(color)) {
                                cortex_m::asm::nop();
                            }
                        });
                    } else {
                        let row_iter = row.iter();
                        row_iter.for_each(|color| {
                            while !tx.write(to_word(color)) {
                                cortex_m::asm::nop(); // inefficient way to wait for the queue to have space in it
                            }
                        });
                    };
                });

                // Write matrix 2 (last 16 leds of row)
                framebuffer.iter().enumerate().for_each(|(i, row)| {
                    let row = &row[16..];
                    if i & 1 == 1 {
                        let row_iter = row.iter().rev();
                        row_iter.for_each(|color| {
                            while !tx.write(to_word(color)) {
                                cortex_m::asm::nop();
                            }
                        });
                    } else {
                        let row_iter = row.iter();
                        row_iter.for_each(|color| {
                            while !tx.write(to_word(color)) {
                                cortex_m::asm::nop(); // inefficient way to wait for the queue to have space in it
                            }
                        });
                    };
                });
            });
        });
    }

    /// Capacity must be 2, since this task spawns itself after some time
    /// `debounce = true` indicates that this update was spawned by debounce and thus no scheduling should be performed
    #[task(priority = 2, capacity = 2, shared = [state, input_state, rtc, lcd])]
    fn update(ctx: update::Context, debounce: bool) {
        let delay_us = |us: u64| {
            let end = monotonics::now() + us.micros();
            while end > monotonics::now() {
                cortex_m::asm::nop(); // busy wait
            }
        };

        rprintln!("Update");
        let next_update = (
            ctx.shared.input_state,
            ctx.shared.state,
            ctx.shared.rtc,
            ctx.shared.lcd,
        )
            .lock(|input, state, rtc, lcd| {
                let nu = match state {
                    State::Clock(clock) => clock.update((input, rtc, lcd, delay_us)),
                    State::Snake(snake) => snake.update(input),
                    State::Settings => todo!("Implement settings update"),
                };

                input.set_old(); // mark as already processed so that the next update cycle doesn't
                                 // have duplicated button pressed
                nu
            });

        render::spawn().unwrap();
        if !debounce {
            update::spawn_at(monotonics::now() + next_update, false).unwrap();
        }
    }

    #[task(priority = 2, shared = [state, framebuffer])]
    fn render(ctx: render::Context) {
        rprintln!("Render");
        // Lock the framebuffer to be sure
        (ctx.shared.state, ctx.shared.framebuffer).lock(|state, framebuffer| match state {
            State::Clock(clock) => clock.render(framebuffer),
            State::Snake(snake) => snake.render(framebuffer),
            State::Settings => todo!("Implement settings render"),
        });

        draw::spawn().unwrap();
    }

    /// debounce inputs with a task that runs every X milliseconds
    #[task(priority = 1, shared = [input_state], local = [left_button, right_button, confirm_button, back_button])]
    fn debounce_input(mut ctx: debounce_input::Context) {
        ctx.shared.input_state.lock(|input_state| {
            let (right_button, left_button, back_button, confirm_button) = (
                ctx.local.right_button,
                ctx.local.left_button,
                ctx.local.back_button,
                ctx.local.confirm_button,
            );

            let mut changed = false;
            let time = monotonics::now();
            let r_high = right_button.is_high().unwrap();
            let l_high = left_button.is_high().unwrap();
            let c_high = confirm_button.is_high().unwrap();
            let b_high = back_button.is_high().unwrap();

            // Check if state has changed (with a xor), then update state and enable the correct interrupt
            if input_state.right.is_pressed() ^ r_high {
                rprint!("R={} ", r_high);
                input_state.right.set_state(r_high, time);
                changed = true;
            }

            if input_state.left.is_pressed() ^ l_high {
                rprint!("L={} ", l_high);
                input_state.left.set_state(l_high, time);
                changed = true;
            }

            if input_state.confirm.is_pressed() ^ c_high {
                rprint!("C={} ", c_high);
                input_state.confirm.set_state(c_high, time);
                changed = true;
            }

            if input_state.back.is_pressed() ^ b_high {
                rprint!("B={} ", b_high);
                input_state.back.set_state(b_high, time);
                changed = true;
            }

            if changed {
                rprintln!("Change");
                update::spawn(true).unwrap(); // update (starts after this task ends due to priority diff)
            }

            let debounce_interval = 25.millis();
            debounce_input::spawn_after(debounce_interval).unwrap(); // next debounce
        });
    }
}

pub(crate) const INITIAL_DATE: DateTime = DateTime {
    year: 2022,
    month: 5,
    day: 5,
    day_of_week: hal::rtc::DayOfWeek::Thursday,
    hour: 22,
    minute: 55,
    second: 0,
};

/// framebuffer type. The bottom left corner is at position `(0,0)`
type Framebuffer = [[Color; 32]; 16];

pub enum Shape {
    /// Circle with center and radius
    Circle(Position, usize),
    /// Line between two points
    Line(Position, Position),
    Dot(Position),
}

/// Trait that allows to draw basic geometric shapes.
trait Draw {
    /// Draw a given shape on this thing, with given color
    fn draw(&mut self, shape: &Shape, color: &Color) {
        match shape {
            Shape::Circle(_c, _r) => {
                todo!("implement")
            }
            Shape::Line(a, b) => {
                // Check if vertical or horizontal (optimized)
                // Otherwise use Bresenham's algorithm
                if a.x == b.x {
                    (a.y.min(b.y)..=a.y.max(b.y))
                        .map(|y| Position::new(a.x, y))
                        .for_each(|p| {
                            self.get_pos_mut(p).map(|c| *c = *color);
                        });
                } else if a.y == b.y {
                    (a.x.min(b.x)..=a.x.max(b.x))
                        .map(|x| Position::new(x, a.y))
                        .for_each(|p| {
                            self.get_pos_mut(p).map(|c| *c = *color);
                        });
                } else {
                    todo!("Bresenham");
                }
            }
            Shape::Dot(p) => {
                self.get_pos_mut(*p).map(|c| *c = *color);
            }
        }
    }

    /// Get the color at position, with mutable access. Used for drawing
    fn get_pos_mut(&mut self, position: Position) -> Option<&mut Color>;
}

impl Draw for Framebuffer {
    fn get_pos_mut(&mut self, position: Position) -> Option<&mut Color> {
        self.get_mut(position.y as usize)
            .and_then(|row| row.get_mut(position.x as usize))
    }
}

/// Return `Some(p)` if the position is in bounds, `None` otherwise
pub fn bounded(p: Position) -> Option<Position> {
    match (p.x, p.y) {
        (0..=31, 0..=15) => Some(p),
        _ => None,
    }
}

pub type Position = Vector2<i8>;

/// Make this color into a word.
/// Intended to be passed to the PIO
/// green: `31:24`, red: `23:16`, blue: `15:8`, zero: `7:0`
pub fn to_word(c: &Color) -> u32 {
    u32::from_be_bytes([c.green, c.red, c.blue, 0])
}

pub(crate) trait SubState<'d> {
    type Data: 'd;
    /// Update cycle for this state.
    ///
    /// # Return
    ///
    /// The time after which the next update should occur, relative to the instant
    /// at which this function was called.
    ///
    /// E.g.: Call to `update` happens at `15ms`. It returns `50ms` after running for `10ms`.
    /// Then the next update should happen at time `65ms`, which is only `40ms` after execution of
    /// `update` stops.
    fn update(&mut self, data: Self::Data) -> Duration;

    /// Render the state to the framebuffer.
    /// This is called automatically after an `update`.
    fn render(&self, framebuffer: &mut Framebuffer);
}

struct ClockWrap<const FREQ: usize> {
    timer: hal::Timer,
}

impl<const FREQ: usize> fmt::Debug for ClockWrap<FREQ> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ClockWrap").field("freq", &FREQ).finish()
    }
}

impl<const FREQ: usize> ClockWrap<FREQ> {
    pub fn new(timer: hal::Timer) -> Self {
        Self { timer }
    }
}

pub enum State {
    Clock(ClockState),
    Snake(SnakeState),
    Settings,
}
