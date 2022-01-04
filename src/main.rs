#![no_std]
#![no_main]

use core::ops::Add;
use core::sync::atomic::Ordering;

use atomic_polyfill::AtomicU8;
// The macro for our start-up function
use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use panic_probe as _; // needed for panic probe stuff

// Time handling traits
use embedded_time::rate::*;

use pico::hal::gpio::{Interrupt, Pin};
// Pull in any important traits
use pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pico::hal::{
    self,
    pac::{self, interrupt},
};

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rtt_target::rprintln;
use rtt_target::rtt_init_print;
use snake::{Direction, SnakeState};

mod snake;
mod text;

static mut CONFIRM_BUTTON: Option<
    Pin<hal::gpio::bank0::Gpio16, hal::gpio::Input<hal::gpio::PullDown>>,
> = None;
static mut BACK_BUTTON: Option<
    Pin<hal::gpio::bank0::Gpio15, hal::gpio::Input<hal::gpio::PullDown>>,
> = None;
static mut LEFT_BUTTON: Option<
    Pin<hal::gpio::bank0::Gpio17, hal::gpio::Input<hal::gpio::PullDown>>,
> = None;
static mut RIGHT_BUTTON: Option<
    Pin<hal::gpio::bank0::Gpio18, hal::gpio::Input<hal::gpio::PullDown>>,
> = None;

pub static INPUT: InputState = InputState {
    left: AtomicU8::new(0),
    right: AtomicU8::new(0),
    confirm: AtomicU8::new(0),
    back: AtomicU8::new(0),
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// framebuffer type. The bottom left corner is at position `(0,0)`
type Framebuffer = [[Color; 32]; 16];

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
        pico::XOSC_CRYSTAL_FREQ,
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::sio::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = pico::Pins::new(
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

    let mut framebuffer = Framebuffer::default();

    let mut state = State::Snake(SnakeState::menu());

    // Control buttons (for interface)
    let left_button = pins.gpio17.into_pull_down_input();
    let right_button = pins.gpio18.into_pull_down_input();
    let confirm_button = pins.gpio16.into_pull_down_input();
    let back_button = pins.gpio15.into_pull_down_input();

    rprintln!("HIGH: {}", right_button.is_high().unwrap());

    unsafe {
        // Interrupts not yet set, so this is safe
        let _ = LEFT_BUTTON.insert(left_button);
        let _ = RIGHT_BUTTON.insert(right_button);
        let _ = BACK_BUTTON.insert(back_button);
        let _ = CONFIRM_BUTTON.insert(confirm_button);
    }

    unsafe {
        // enable interrupts only after setting global buttons.
        // No writes happen to the pins before this, so this is also safe
        let p = LEFT_BUTTON.as_ref().unwrap();
        p.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        //p.set_interrupt_enabled(Interrupt::EdgeLow, true);
        let p = RIGHT_BUTTON.as_ref().unwrap();
        p.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        //p.set_interrupt_enabled(Interrupt::EdgeLow, true);
        let p = BACK_BUTTON.as_ref().unwrap();
        p.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        //p.set_interrupt_enabled(Interrupt::EdgeLow, true);
        let p = CONFIRM_BUTTON.as_ref().unwrap();
        p.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        //p.set_interrupt_enabled(Interrupt::EdgeLow, true);
    }

    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::IO_IRQ_BANK0);
    }

    loop {
        // Update the state first. Takes little time and it should happen before the render
        update(&mut state, &INPUT);

        // render state onto the framebuffer and then draw it
        render(&state, &mut framebuffer);

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
                        cortex_m::asm::nop();
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
                        cortex_m::asm::nop();
                    }
                });
            };
        });

        delay.delay_ms(500);
    }
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

fn render(state: &State, framebuffer: &mut Framebuffer) {
    match state {
        State::Clock => todo!(),
        State::Snake(snake) => snake::render(snake, framebuffer),
        State::Settings => todo!(),
    }
}

fn update(state: &mut State, input: &InputState) {
    match state {
        State::Clock => todo!(),
        State::Snake(snake) => snake::update(snake, input),
        State::Settings => todo!(),
    }
}

/// Current input state.
/// * `0` means not pressed.
/// * `1` means that the button was pressed and released _during this tick_.
/// * `2` means that the button is being held down (hasn't been released yet).
/// Values are set atomically by the interrupt
pub struct InputState {
    left: AtomicU8,
    right: AtomicU8,
    confirm: AtomicU8,
    back: AtomicU8,
}

/// Interrupt that changes the input state based on GPIO changes on the buttons
#[interrupt]
fn IO_IRQ_BANK0() {
    cortex_m::interrupt::free(|cs| {
        // debounce individual inputs

        let (confirm, back, left, right) = unsafe {
            // This can only be executed once the interrupts are set.
            // No writes happen after that => any amount of reads are safe now.w
            let confirm = CONFIRM_BUTTON.as_ref().unwrap().is_high().unwrap();
            let back = BACK_BUTTON.as_ref().unwrap().is_high().unwrap();
            let left = LEFT_BUTTON.as_ref().unwrap().is_high().unwrap();
            let right = RIGHT_BUTTON.as_ref().unwrap().is_high().unwrap();
            (confirm, back, left, right)
        };

        // check differences
        if confirm {
            INPUT.confirm.store(2, Ordering::Release);
        } else {
            let _ = INPUT
                .confirm
                .compare_exchange(2, 1, Ordering::Release, Ordering::Release);
        }

        if back {
            INPUT.back.store(2, Ordering::Release);
        } else {
            let _ = INPUT
                .back
                .compare_exchange(2, 1, Ordering::Release, Ordering::Release);
        }

        if left {
            INPUT.left.store(2, Ordering::Release);
        } else {
            let _ = INPUT
                .left
                .compare_exchange(2, 1, Ordering::Release, Ordering::Release);
        }

        if right {
            INPUT.right.store(2, Ordering::Release);
        } else {
            let _ = INPUT
                .right
                .compare_exchange(2, 1, Ordering::Release, Ordering::Release);
        }

        rprintln!(
            "Interrupt Triggered: Confirm: {}, Back: {}, Left: {}, Right: {}",
            INPUT.confirm.load(Ordering::Acquire),
            INPUT.back.load(Ordering::Acquire),
            INPUT.left.load(Ordering::Acquire),
            INPUT.right.load(Ordering::Acquire)
        );
    });
}

enum State {
    Clock,
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
