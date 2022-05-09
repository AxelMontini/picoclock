#![no_std]

use embedded_hal::digital::v2::{InputPin, OutputPin, PinState};

pub type Result<T> = core::result::Result<T, ()>; //TODO: ACtual error handling

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

    pub fn write_text(&mut self, text: &str) -> Result<()> {
        self.
    }

    /// Writes data to the `data` pins. MSB is DB7, LSB is DB0.
    /// This does not change state of any other pins! (even `e`).
    fn write_data(&mut self, data: u8) -> Result<()> {
        let state = |data: u8, pin_num: u8| if (data >> pin_num) & 1 == 1 {PinState::High} else {PinState::Low};
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
