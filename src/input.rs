use rp_pico::hal::clocks::SystemClock;

use crate::{Instant, LEFT_BUTTON, LeftButtonPin, RightButtonPin, ConfirmButtonPin, BackButtonPin};

/// Update input state. Return 
pub fn update_input_state(left_button: &LeftButtonPin, right_button: &RightButtonPin, confirm_button: &ConfirmButtonPin, back_button: &BackButtonPin, clock: &SystemClock) {
    
}

/// should be greater than 10 according to [`the docs`](https://docs.rs/rp2040-hal/latest/rp2040_hal/timer/struct.Alarm0.html)
const BUTTON_DEBOUNCE_TIME: Milliseconds = Milliseconds(50);

/// What button has been pressed most recently
#[derive(PartialEq, Eq, Debug)]
pub enum ButtonState {
    /// Was pressed at instant
    Pressed(Instant),
    /// Was released at instant
    Released(Instant),
}

/// Current input state.
/// The right `ButtonState` is determined from the samples (LSB is newest sample).
#[derive(PartialEq, Eq, Debug)]
pub struct InputState {
    pub confirm: ButtonState,
    pub back: ButtonState,
    pub left: ButtonState,
    pub right: ButtonState,
}

impl InputState {
    pub fn new(time: Instant) -> Self {
        Self {
            confirm: ButtonState::Released(time),
            back: ButtonState::Released(time),
            left: ButtonState::Released(time),
            right: ButtonState::Released(time),
        }
    }
}