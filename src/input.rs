use embedded_time::duration::Milliseconds;

use crate::Instant;

/// should be greater than 10 according to [`the docs`](https://docs.rs/rp2040-hal/latest/rp2040_hal/timer/struct.Alarm0.html)
const BUTTON_DEBOUNCE_TIME: Milliseconds = Milliseconds(50);

/// What button has been pressed most recently
#[derive(PartialEq, Eq, Debug, Clone)]
pub enum ButtonState {
    /// Was pressed at a specific time and `true` if the action is new (becomes `false` after an update has ran).
    Pressed(bool, Instant),
    /// Was released at a specific time and `true` if the action is new (becomes `false` after an update has ran).
    Released(bool, Instant),
}

impl ButtonState {
    pub fn is_pressed(&self) -> bool {
        match self {
            Self::Pressed(_, _) => true,
            _ => false,
        }
    }

    pub fn is_released(&self) -> bool {
        !self.is_pressed()
    }

    pub fn is_just_pressed(&self) -> bool {
        match self {
            Self::Pressed(true, _) => true,
            _ => false,
        }
    }

    pub fn is_just_released(&self) -> bool {
        match self {
            Self::Released(true, _) => true,
            _ => false,
        }
    }

    pub fn set_new(&mut self, new: bool) {
        let v = match self {
            ButtonState::Pressed(v, _i) => v,
            ButtonState::Released(v, _i) => v,
        };

        *v = new;
    }

    /// Update this state with released/pressed and an instant
    pub fn set_state(&mut self, pressed: bool, time: Instant) {
        match pressed {
            true => *self = Self::Pressed(true, time),
            false => *self = Self::Released(true, time),
        }
    }
}

/// Current input state.
#[derive(PartialEq, Eq, Debug, Clone)]
pub struct InputState {
    pub confirm: ButtonState,
    pub back: ButtonState,
    pub left: ButtonState,
    pub right: ButtonState,
}

impl InputState {
    pub fn new(time: Instant) -> Self {
        Self {
            confirm: ButtonState::Released(false, time),
            back: ButtonState::Released(false, time),
            left: ButtonState::Released(false, time),
            right: ButtonState::Released(false, time),
        }
    }

    /// Turn the state into an "old state" (already went through an `update`)
    pub fn set_old(&mut self) {
        self.confirm.set_new(false);
        self.back.set_new(false);
        self.right.set_new(false);
        self.left.set_new(false);
    }

    /// `true` iff left is pressed and right is released
    pub fn is_left(&self) -> bool {
        self.left.is_pressed() && self.right.is_released()
    }

    /// `true` iff left was just pressed and right is released
    pub fn is_just_left(&self) -> bool {
        self.left.is_just_pressed() && self.right.is_released()
    }

    /// `true` iff right is pressed and left is released
    pub fn is_right(&self) -> bool {
        self.right.is_pressed() && self.left.is_released()
    }

    /// `true` iff right was just pressed and left is released
    pub fn is_just_right(&self) -> bool {
        self.right.is_just_pressed() && self.left.is_released()
    }

    /// `true` iff back is pressed and confirm is released
    pub fn is_back(&self) -> bool {
        self.back.is_pressed() && self.confirm.is_released()
    }

    /// `true` iff back was just pressed and confirm is released
    pub fn is_just_back(&self) -> bool {
        self.back.is_just_pressed() && self.confirm.is_released()
    }

    /// `true` iff confirm is pressed and back is released
    pub fn is_confirm(&self) -> bool {
        self.confirm.is_pressed() && self.back.is_released()
    }

    /// `true` iff confirm was just pressed and back is released
    pub fn is_just_confirm(&self) -> bool {
        self.confirm.is_just_pressed() && self.back.is_released()
    }
}
