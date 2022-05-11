#![no_std]

use embedded_hal::digital::v2::{InputPin, OutputPin, PinState};

/// LCD Struct. Provides methods to control the display.
pub struct LcdConnection<RS, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7> {
    pub(crate) rs: RS,
    pub(crate) e: E,
    pub(crate) db0: DB0,
    pub(crate) db1: DB1,
    pub(crate) db2: DB2,
    pub(crate) db3: DB3,
    pub(crate) db4: DB4,
    pub(crate) db5: DB5,
    pub(crate) db6: DB6,
    pub(crate) db7: DB7,
}

impl<RS, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7>
    LcdConnection<RS, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7>
{
    pub fn new(
        rs: RS,
        e: E,
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

    pub fn driver<D: Fn(u64) -> ()>(
        &mut self,
        delay_us: D,
    ) -> LcdDriver<RS, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, D> {
        LcdDriver {
            conn: self,
            delay_us,
        }
    }
}

/// Struct potentially referencing an `Lcd` instance.
///
/// Designed so that it can be created on the fly, where access
/// to a timer is temporary (e.g. behind a lock).
pub struct LcdDriver<'c, RS, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, DELAY> {
    conn: &'c mut LcdConnection<RS, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7>,
    delay_us: DELAY,
}

impl<'c, RS, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, ERR, DELAY>
    LcdDriver<'c, RS, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, DELAY>
where
    RS: OutputPin<Error = ERR>,
    E: OutputPin<Error = ERR>,
    DB0: OutputPin<Error = ERR> + InputPin<Error = ERR>,
    DB1: OutputPin<Error = ERR> + InputPin<Error = ERR>,
    DB2: OutputPin<Error = ERR> + InputPin<Error = ERR>,
    DB3: OutputPin<Error = ERR> + InputPin<Error = ERR>,
    DB4: OutputPin<Error = ERR> + InputPin<Error = ERR>,
    DB5: OutputPin<Error = ERR> + InputPin<Error = ERR>,
    DB6: OutputPin<Error = ERR> + InputPin<Error = ERR>,
    DB7: OutputPin<Error = ERR> + InputPin<Error = ERR>,
    DELAY: Fn(u64) -> (),
{

    pub fn return_home(&mut self) -> Result<(), ERR> {
        self.write_rs(false)?;
        self.write_data(0b10)?;
        self.signal()?;
        (self.delay_us)(1520);
        Ok(())
    }

    pub fn clear(&mut self) -> Result<(), ERR> {
        self.write_rs(false)?;
        self.write_data(1)?;
        self.signal()?;

        (self.delay_us)(5_000); // 1.52ms reset time

        Ok(())
    }

    /// Fixed to 8 bit, 2 lines, 5x10 font
    pub fn set_function(&mut self) -> Result<(), ERR> {
        self.write_rs(false)?;
        let data = 0b111011;
        self.write_data(data)?;
        self.signal()?;
        (self.delay_us)(37);

        Ok(())
    }

    pub fn set_display_control(
        &mut self,
        display_on: bool,
        cursor_on: bool,
        blink: bool,
    ) -> Result<(), ERR> {
        self.write_rs(false)?;
        let data = 0b_1000 | u8::from(display_on) << 2 | u8::from(cursor_on) << 1 | u8::from(blink);
        self.write_data(data)?;
        self.signal()?;

        (self.delay_us)(37);

        Ok(())
    }

    /// Only 8-bit characters are supported since this is an LCD.
    /// Returns an error either if the character is not valid ASCII or if there was an IO error.
    pub fn write_char(&mut self, c: char) -> Result<(), ERR> {
        if !c.is_ascii() {
            panic!("TODO: GOOD ERROR HANDLING");
        }

        self.write_rs(true)?;
        self.write_data(c as u8)?;
        self.signal()?;

        (self.delay_us)(37);

        Ok(())
    }

    /// Set ddram address. Address should be 7 bits and it is clamped to fit.
    pub fn set_ddram_address(&mut self, addr: u8) -> Result<(), ERR> {
        let data = addr | 1 << 7; // Format 1 A A A A A A A
        self.write_rs(false)?;
        self.write_data(data)?;
        self.signal()?;
        (self.delay_us)(37);
        Ok(())
    }

    pub fn set_line0(&mut self) -> Result<(), ERR> {
        self.set_ddram_address(0)
    }
    pub fn set_line1(&mut self) -> Result<(), ERR> {
        self.set_ddram_address(0x40)
    }

    pub fn write_text(&mut self, text: &str) -> Result<(), ERR> {
        text.chars().try_for_each(|c| self.write_char(c))
    }

    /// Writes data to the `data` pins. MSB is DB7, LSB is DB0.
    /// This does not change state of any other pins! (even `e`).
    fn write_data(&mut self, data: u8) -> Result<(), ERR> {
        let state = |data: u8, pin_num: u8| {
            if (data >> pin_num) & 1 == 1 {
                PinState::High
            } else {
                PinState::Low
            }
        };

        self.conn.db0.set_state(state(data, 0))?;
        self.conn.db1.set_state(state(data, 1))?;
        self.conn.db2.set_state(state(data, 2))?;
        self.conn.db3.set_state(state(data, 3))?;
        self.conn.db4.set_state(state(data, 4))?;
        self.conn.db5.set_state(state(data, 5))?;
        self.conn.db6.set_state(state(data, 6))?;
        self.conn.db7.set_state(state(data, 7))?;

        Ok(())
    }

    fn write_rs(&mut self, rs: bool) -> Result<(), ERR> {
        self.conn.rs.set_state(PinState::from(rs))?;

        Ok(())
    }

    /// Signal a write by toggling `e`
    fn signal(&mut self) -> Result<(), ERR> {
        self.conn.e.set_high()?;
        (self.delay_us)(45);
        self.conn.e.set_low()?;

        Ok(())
    }
}
