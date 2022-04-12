#![no_std]
#![no_main]

use core::borrow::BorrowMut;
use core::fmt::{self, Debug};
use core::ops::Deref;
use core::sync::atomic::Ordering;
use core::{cell::RefCell, ops::Add};

use atomic_polyfill::AtomicU8;
use clock::ClockState;
use cortex_m::interrupt::{CriticalSection, Mutex};
// The macro for our start-up function
use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use embedded_time::duration::*;
use input::InputState;
use panic_probe as _; // needed for panic probe stuff

// Time handling traits
use embedded_time::{rate::*, Clock};

use rp_pico::hal::rtc::DateTime;
use rp_pico::hal::{
    self,
    gpio::{Interrupt, Pin},
    pac::{self, interrupt},
    prelude::*,
};

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rtt_target::rprintln;
use rtt_target::rtt_init_print;
use snake::{Direction, SnakeState};

mod clock;
mod input;
mod snake;
mod text;

/// Fraction of the XOSC clock
const FRAC: Fraction = Fraction::new(1, rp_pico::XOSC_CRYSTAL_FREQ);

/// Clock is initialized at default frequency
type Instant = embedded_time::Instant<ClockWrap>;

type RightButtonPin = Pin<hal::gpio::bank0::Gpio18, hal::gpio::Input<hal::gpio::PullDown>>;
type LeftButtonPin = Pin<hal::gpio::bank0::Gpio17, hal::gpio::Input<hal::gpio::PullDown>>;
type BackButtonPin = Pin<hal::gpio::bank0::Gpio15, hal::gpio::Input<hal::gpio::PullDown>>;
type ConfirmButtonPin = Pin<hal::gpio::bank0::Gpio16, hal::gpio::Input<hal::gpio::PullDown>>;

pub(crate) static CONFIRM_BUTTON: Mutex<RefCell<Option<ConfirmButtonPin>>> =
    Mutex::new(RefCell::new(None));
pub(crate) static BACK_BUTTON: Mutex<RefCell<Option<BackButtonPin>>> =
    Mutex::new(RefCell::new(None));
pub(crate) static LEFT_BUTTON: Mutex<RefCell<Option<LeftButtonPin>>> =
    Mutex::new(RefCell::new(None));
pub(crate) static RIGHT_BUTTON: Mutex<RefCell<Option<RightButtonPin>>> =
    Mutex::new(RefCell::new(None));

pub(crate) static TX: Mutex<RefCell<Option<hal::pio::Tx<(pac::PIO0, hal::pio::SM0)>>>> =
    Mutex::new(RefCell::new(None));
pub(crate) static INPUT_STATE: Mutex<RefCell<Option<InputState>>> = Mutex::new(RefCell::new(None));
pub(crate) static ALARM0: Mutex<RefCell<Option<hal::timer::Alarm0>>> =
    Mutex::new(RefCell::new(None));
pub(crate) static CLOCK: Mutex<RefCell<Option<ClockWrap>>> = Mutex::new(RefCell::new(None));
pub(crate) static RTC: Mutex<RefCell<Option<hal::rtc::RealTimeClock>>> =
    Mutex::new(RefCell::new(None));

pub(crate) const INITIAL_DATE: DateTime = DateTime {
    year: 2022,
    month: 4,
    day: 12,
    day_of_week: hal::rtc::DayOfWeek::Tuesday,
    hour: 12,
    minute: 0,
    second: 0,
};

pub(crate) const FRAMEBUFFER: Mutex<RefCell<Framebuffer>> = Framebuffer::default();

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// framebuffer type. The bottom left corner is at position `(0,0)`
type Framebuffer = [[Color; 32]; 16];

pub(crate) trait SubState {
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
    fn update(&mut self, input: &InputState) -> embedded_time::duration::Milliseconds;

    /// Render the state to the framebuffer.
    /// This is called automatically after an `update`.
    fn render(&self, framebuffer: &mut Framebuffer);
}

#[entry]
fn main() -> ! {
    // Print to pico probe
    rtt_init_print!();

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

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

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // timer for periodic tasks, interrupts, ...
    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let alarm0 = timer.alarm_0().unwrap();

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
    let div = clocks.system_clock.freq().integer() as f32 / (FREQ as f32 * CYCLES_PER_BIT as f32);

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

    let (mut sm, _, mut tx) = hal::pio::PIOBuilder::from_program(installed)
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
    let mut state = State::Clock(ClockState);

    // Control buttons (for interface). They must be set globally to allow
    // the interrupt to read them
    let left_button = pins.gpio17.into_pull_down_input();
    let right_button = pins.gpio18.into_pull_down_input();
    let confirm_button = pins.gpio16.into_pull_down_input();
    let back_button = pins.gpio15.into_pull_down_input();

    let clock = ClockWrap::new(timer);
    let input_state = InputState::new(clock.try_now().unwrap());

    cortex_m::interrupt::free(|cs| {
        LEFT_BUTTON.borrow(cs).replace(Some(left_button)); // global button pins after configuring
        RIGHT_BUTTON.borrow(cs).replace(Some(right_button));
        CONFIRM_BUTTON.borrow(cs).replace(Some(confirm_button));
        BACK_BUTTON.borrow(cs).replace(Some(back_button));

        INPUT_STATE.borrow(cs).replace(Some(input_state)); // default input state

        // schedule next render and insert into global alarm for further scheduling
        ALARM0.borrow(cs).replace(Some(alarm0));

        CLOCK.borrow(cs).replace(Some(clock));

        // Make Real Time Clock global
        RTC.borrow(cs).replace(Some(rtc));

        // Make the TX queue (for pio) global
        TX.borrow(cs).replace(Some(tx));
    });

    // Run first update & render. The cycle function automatically schedules
    // the next update
    cycle();
    loop {}
}

/// A cycle consists in updating, rendering and drawing
fn cycle() {
    cortex_m::interrupt::free(|cs| {
        let clock = CLOCK.borrow(cs).borrow().as_ref().unwrap();
        let start_time = clock.try_now().unwrap();
        let framebuffer = FRAMEBUFFER.borrow(cs).get_mut();
        let input_state = INPUT_STATE.borrow(cs).borrow().as_ref().unwrap();
        let alarm = ALARM0.borrow(cs).as_ref().unwrap();

        // Update the state first. Takes little time and it should happen before the render
        let duration = update(&mut state, input_state);

        // render state onto the framebuffer and then draw it
        render(&state, framebuffer);

        draw(framebuffer);

        // Schedule next
        let end_time = clock.try_now().unwrap();
        alarm0.schedule();
    });
}

/// A position that is in range `0..16`
#[derive(Debug, Clone, Copy)]
pub struct Position {
    x: u8,
    y: u8,
}

impl Position {
    pub fn new(x: u8, y: u8) -> Self {
        Self { x, y }
    }

    pub fn add_dir(self, dir: Direction) -> Option<Self> {
        match dir {
            Direction::PosX => {
                ((0..15).contains(&self.x)).then(|| Position::new(self.x + 1, self.y))
            }
            Direction::NegX => {
                ((1..16).contains(&self.x)).then(|| Position::new(self.x - 1, self.y))
            }
            Direction::PosY => {
                ((0..15).contains(&self.y)).then(|| Position::new(self.x, self.y + 1))
            }
            Direction::NegY => {
                ((1..16).contains(&self.y)).then(|| Position::new(self.x, self.y - 1))
            }
        }
    }
}

impl Add for Position {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Position::new(self.x + rhs.x, self.y + rhs.y)
    }
}

fn draw(framebuffer: &Framebuffer) {
    // An interrupt should never stop this, since delays in the signal could be interpreted as "stop" by the leds.
    cortex_m::interrupt::free(|cs| {
        let tx = TX.borrow(cs).get_mut().as_mut().unwrap();

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
                    while !tx.write(color.to_word()) {
                        cortex_m::asm::nop();
                    }
                });
            } else {
                let row_iter = row.iter();
                row_iter.for_each(|color| {
                    while !tx.write(color.to_word()) {
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
                    while !tx.write(color.to_word()) {
                        cortex_m::asm::nop();
                    }
                });
            } else {
                let row_iter = row.iter();
                row_iter.for_each(|color| {
                    while !tx.write(color.to_word()) {
                        cortex_m::asm::nop(); // inefficient way to wait for the queue to have space in it
                    }
                });
            };
        });
    });
}

fn render(state: &State, framebuffer: &mut Framebuffer) {
    match state {
        State::Clock(clock) => clock.render(framebuffer),
        State::Snake(snake) => snake.render(framebuffer),
        State::Settings => todo!(),
    }
}

fn update(state: &mut State, input: &InputState) -> embedded_time::duration::Milliseconds {
    match state {
        State::Clock(clock) => clock.update(input),
        State::Snake(snake) => snake.update(input),
        State::Settings => todo!(),
    }
}

/// Interrupt that runs (`update`) and (`render`) for each frame
#[interrupt]
fn TIMER_IRQ_0() {
    cortex_m::interrupt::free(|cs| {
        // reschedule timer to run later
    });
}

/// Interrupt that runs on GPIO inputs.
/// Handles button presses for now.
#[interrupt]
fn IO_IRQ_BANK0() {
    // Once a button is pressed, we should debounce the signal.
    // If input state changes, then we should update and render.

    cortex_m::interrupt::free(|cs| {});
}

struct ClockWrap {
    timer: hal::Timer,
}

impl fmt::Debug for ClockWrap {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ClockWrap").finish()
    }
}

impl ClockWrap {
    pub fn new(timer: hal::Timer) -> Self {
        Self { timer }
    }
}

impl Clock for ClockWrap {
    type T = u64;

    const SCALING_FACTOR: Fraction = FRAC;

    fn try_now(&self) -> Result<embedded_time::Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::new(self.timer.get_counter()))
    }
}

enum State {
    Clock(ClockState),
    Snake(SnakeState),
    Settings,
}

/// GRB Color
#[derive(Copy, Clone, Debug, Default)]
pub struct Color {
    r: u8,
    g: u8,
    b: u8,
}

impl Color {
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    /// Make this color into a word.
    /// Intended to be passed to the PIO
    /// green: `31:24`, red: `23:16`, blue: `15:8`, zero: `7:0`
    pub fn to_word(&self) -> u32 {
        u32::from_be_bytes([self.g, self.r, self.b, 0])
    }

    // pub fn into_spi_bytes(&self) -> [u8; 9] {
    //     use bitvec::prelude::*;
    //     let color = [self.g, self.r, self.b];
    //     let color = color.view_bits::<Msb0>();

    //     let mut arr = bitarr![Msb0, u8; 0; 8*9];

    //     for i in 0..3 * 8 {
    //         if color.get(i).unwrap() == true {
    //             arr.set(i * 3, true);
    //             arr.set(i * 3 + 1, true);
    //             arr.set(i * 3 + 2, false);
    //         } else {
    //             arr.set(i * 3, true);
    //             arr.set(i * 3 + 1, false);
    //             arr.set(i * 3 + 2, false);
    //         };
    //     }

    //     arr.into_inner()
    // }
}
