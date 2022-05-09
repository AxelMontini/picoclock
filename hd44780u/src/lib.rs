#![no_std]

use embedded_hal::digital::v2::{InputPin, OutputPin, PinState};

pub type Result<T> = core::result::Result<T, ()>; //TODO: Actual error handling

/// LCD Struct. Provides methods to control the display.
pub struct Lcd<RW, RS, BF, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7> {
    rs: RS,
    bf: BF,
    rw: RW,
    e: E,
    db0: DB0,
    db1: DB1,
    db2: DB2,
    db3: DB3,
    db4: DB4,
    db5: DB5,
    db6: DB6,
    db7: DB7,
}

impl<RW, RS, BF, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7>
    Lcd<RW, RS, BF, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7>
where
    RW: OutputPin,
    RS: OutputPin,
    E: OutputPin,
    DB0: OutputPin + InputPin,
    DB1: OutputPin + InputPin,
    DB2: OutputPin + InputPin,
    DB3: OutputPin + InputPin,
    DB4: OutputPin + InputPin,
    DB5: OutputPin + InputPin,
    DB6: OutputPin + InputPin,
    DB7: OutputPin + InputPin,
    BF: InputPin,
{
    pub fn new(
        rs: RS,
        rw: RW,
        e: E,
        bf: BF,
        db0: DB0,
        db1: DB1,
        db2: DB2,
        db3: DB3,
        db4: DB4,
        db5: DB5,
        db6: DB6,
        db7: DB7,
    ) -> Self {
        Self {
            rs,
            bf,
            rw,
            e,
            db0,
            db1,
            db2,
            db3,
            db4,
            db5,
            db6,
            db7,
        }
    }

    pub fn clear(&mut self) -> Result<()> {
        self.write_rsrw(false, false);
        self.write_data(1);
        self.signal();

        Ok(())
    }

    pub fn set_display_control(
        &mut self,
        display_on: bool,
        cursor_on: bool,
        blink: bool,
    ) -> Result<()> {
        self.write_rsrw(false, false);
        let data = 0b_1000 | u8::from(display_on) << 2 | u8::from(cursor_on) << 1 | u8::from(blink);
        self.write_data(data);
        self.signal();

        Ok(())
    }

    /// Returns the Busy Flag and the address counter
    fn read_bf(&mut self) -> Result<(bool, u8)> {
        self.write_rsrw(false, true);
        self.write_data(0);
        self.signal();

        let bf = self.db7.is_high()?;
        let address = u8::from(self.db0.is_high()?)
            | u8::from(self.db1.is_high()?) << 1
            | u8::from(self.db2.is_high()?) << 2
            | u8::from(self.db3.is_high()?) << 3
            | u8::from(self.db4.is_high()?) << 4
            | u8::from(self.db5.is_high()?) << 5
            | u8::from(self.db6.is_high()?) << 6
            | u8::from(self.db7.is_high()?) << 7;

        Ok((bf, address));
    }

    /// Only 8-bit characters are supported since this is an LCD.
    /// Returns an error either if the character is not valid ASCII or if there was an IO error.
    pub fn write_char(&mut self, c: char) -> Result<()> {
        if !c.is_ascii() {
            return Err(());
        }

        self.write_rsrw(true, false);
        self.write_data(c as u8);
        self.signal();

        Ok(())
    }

    pub fn write_text(&mut self, text: &str) -> Result<()> {
        text.chars().try_for_each(|c| self.write_char(c));

        Ok(())
    }

    /// Writes data to the `data` pins. MSB is DB7, LSB is DB0.
    /// This does not change state of any other pins! (even `e`).
    fn write_data(&mut self, data: u8) -> Result<()> {
        let state = |data: u8, pin_num: u8| {
            if (data >> pin_num) & 1 == 1 {
                PinState::High
            } else {
                PinState::Low
            }
        };
        self.db0.set_state(state(data, 0));
        self.db1.set_state(state(data, 1));
        self.db2.set_state(state(data, 2));
        self.db3.set_state(state(data, 3));
        self.db4.set_state(state(data, 4));
        self.db5.set_state(state(data, 5));
        self.db6.set_state(state(data, 6));
        self.db7.set_state(state(data, 7));

        Ok(())
    }

    fn write_rsrw(&mut self, rs: bool, rw: bool) -> Result<()> {
        self.rs.set_state(PinState::from(rs));
        self.rw.set_state(PinState::from(rw));

        Ok(())
    }

    /// Signal a write by toggling `e`
    fn signal(&mut self) -> Result<()> {
        self.e.set_high();
        //TODO: Sleep?
        self.e.set_low();

        Ok(())
    }
}
