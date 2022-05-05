/// GRB Color
#[derive(Copy, Clone, Debug, Default)]
pub struct Color {
    r: u8,
    g: u8,
    b: u8,
}

impl Color {
    /// RGB Color
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self::rgb(r, g, b)
    }

    /// RGB Color
    pub const fn rgb(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    /// HSV Color (approximated to fit in RGB)
    /// Values in range `0.0..=1.0`
    pub const fn hsv(h: f32, s: f32, v: f32) -> Self {
        let c = s * v;
        let x = c * (1.0 - ((h * 6.0) % 2.0 - 1.0).abs());
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

#[cfg(test)]
mod tests {
    #[test]
    fn color_conversion() {
        
    }
}
