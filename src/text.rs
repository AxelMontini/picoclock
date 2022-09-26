use crate::{Color, Framebuffer, Position};

pub struct Letter<const W: usize, const H: usize> {
    /// Bitmap. index 0,0 is top left corner,
    /// for the ease of writing in Rust.
    data: &'static [[bool; W]; H],
}

impl<const W: usize, const H: usize> Letter<W, H> {
    pub const fn new(data: &'static [[bool; W]; H]) -> Self {
        Self { data }
    }
}

impl<const W: usize, const H: usize> ColorRender for Letter<W, H> {
    fn render(&self, framebuffer: &mut Framebuffer, origin: Position, color: Color) {
        let origin_x = origin.x as usize;
        let origin_y = origin.y as usize;

        // Reverse because the first row is the top one
        self.data
            .iter()
            .rev()
            .enumerate()
            .for_each(|(row_idx, row)| {
                row.iter()
                    .enumerate()
                    .filter(|(_c, &draw)| draw)
                    .for_each(|(col_idx, _draw)| {
                        framebuffer
                            .get_mut(origin_y + row_idx)
                            .and_then(|r| r.get_mut(origin_x + col_idx))
                            .map(|c| *c = color);
                    })
            });
    }

    fn bounds(&self) -> (usize, usize) {
        (W, H)
    }
}

pub trait ColorRender {
    /// Render at given position in the framebuffer with color
    fn render(&self, framebuffer: &mut Framebuffer, origin: Position, color: Color);
    /// (width, height)
    fn bounds(&self) -> (usize, usize);
}

/// Render some text on the screen.
/// Needs a framebuffer.
/// `origin` is from the bottom left corner of the framebuffer
/// to the bottom left corner of the text.
/// Letters are 5 tall and width is variable.
///
/// `colors` is a slice of colors for each letter.
/// If color is shorter than the characters, then the function panics.
pub fn render_text(
    framebuffer: &mut Framebuffer,
    text: &str,
    origin: Position,
    colors: impl Iterator<Item = impl Into<Color>>,
) {
    render_text_font(framebuffer, text, TABLE, origin, colors)
}

/// Render some text on the screen.
/// Needs a framebuffer.
/// `origin` is from the bottom left corner of the framebuffer
/// to the bottom left corner of the text.
/// Letters are 5 tall and width is variable.
///
/// `colors` is a slice of colors for each letter.
/// If color is shorter than the characters, then the function panics.
pub fn render_text_font<'f>(
    framebuffer: &mut Framebuffer,
    text: &str,
    font: &Font<'f>,
    origin: Position,
    mut colors: impl Iterator<Item = impl Into<Color>>,
) {
    text.chars().fold(origin, |cursor, c| {
        let bounds = render_char_font(framebuffer, c, font, cursor, colors.next().unwrap().into());
        Position::new(1 + cursor.x + bounds.0 as i8, cursor.y)
    });
}

/// Renders a char and returns the bounding box of the rendered char (width, height)
pub fn render_char(
    framebuffer: &mut Framebuffer,
    character: char,
    origin: Position,
    color: Color,
) -> (usize, usize) {
    render_char_font(framebuffer, character, TABLE, origin, color)
}

/// Font that starts from 0, ascii order
pub type Font<'f> = [&'f dyn ColorRender];

/// Renders a char in a custom font and returns the bounding box of the rendered char (width, height)
pub fn render_char_font<'f>(
    framebuffer: &mut Framebuffer,
    character: char,
    font: &Font<'f>,
    origin: Position,
    color: Color,
) -> (usize, usize) {
    if !character.is_ascii() {
        panic!("Not an ascii character: {:x}", character as u64);
    }

    let index = character as usize - '0' as usize;
    let letter = font.get(index).expect("unimplemented character");

    letter.render(framebuffer, origin, color);

    letter.bounds()
}

const A: Letter<3, 5> = Letter::new(&[
    [true; 3],
    [true, false, true],
    [true; 3],
    [true, false, true],
    [true, false, true],
]);
const B: Letter<3, 5> = Letter::new(&[
    [true, true, false],
    [true, false, true],
    [true, true, false],
    [true, false, true],
    [true, true, false],
]);
const C: Letter<2, 5> = Letter::new(&[
    [true, true],
    [true, false],
    [true, false],
    [true, false],
    [true, true],
]);
const D: Letter<3, 5> = Letter::new(&[
    [true, true, false],
    [true, false, true],
    [true, false, true],
    [true, false, true],
    [true, true, false],
]);
const E: Letter<2, 5> = Letter::new(&[
    [true, true],
    [true, false],
    [true, true],
    [true, false],
    [true, true],
]);
const F: Letter<2, 5> = Letter::new(&[
    [true, true],
    [true, false],
    [true, true],
    [true, false],
    [true, false],
]);
const G: Letter<3, 5> = Letter::new(&[
    [true, true, true],
    [true, false, false],
    [true, true, true],
    [true, false, true],
    [true, true, true],
]);
const H: Letter<3, 5> = Letter::new(&[
    [true, false, true],
    [true, false, true],
    [true, true, true],
    [true, false, true],
    [true, false, true],
]);
const I: Letter<1, 5> = Letter::new(&[[true], [true], [true], [true], [true]]);
const J: Letter<3, 5> = Letter::new(&[
    [false, false, true],
    [false, false, true],
    [false, false, true],
    [true, false, true],
    [false, true, false],
]);
const K: Letter<3, 5> = Letter::new(&[
    [true, false, true],
    [true, true, false],
    [true, true, false],
    [true, false, true],
    [true, false, true],
]);
const L: Letter<2, 5> = Letter::new(&[
    [true, false],
    [true, false],
    [true, false],
    [true, false],
    [true, true],
]);
const M: Letter<5, 5> = Letter::new(&[
    [true, true, true, true, true],
    [true, false, true, false, true],
    [true, false, true, false, true],
    [true, false, true, false, true],
    [true, false, true, false, true],
]);
const N: Letter<4, 5> = Letter::new(&[
    [true, false, false, true],
    [true, true, false, true],
    [true, false, false, false],
    [true, false, true, false],
    [true, false, false, true],
]);
const O: Letter<3, 5> = Letter::new(&[
    [true, true, true],
    [true, false, true],
    [true, false, true],
    [true, false, true],
    [true, true, true],
]);
const P: Letter<3, 5> = Letter::new(&[
    [true, true, true],
    [true, false, true],
    [true, true, true],
    [true, false, false],
    [true, false, false],
]);
const Q: Letter<3, 5> = Letter::new(&[
    [false, true, false],
    [true, false, true],
    [true, false, true],
    [false, true, false],
    [false, false, true],
]);
const R: Letter<3, 5> = Letter::new(&[
    [true, true, true],
    [true, false, true],
    [true, true, true],
    [true, true, false],
    [true, false, true],
]);
const S: Letter<3, 5> = Letter::new(&[
    [true, true, true],
    [true, false, false],
    [false, true, false],
    [false, false, true],
    [true, true, true],
]);
const T: Letter<3, 5> = Letter::new(&[
    [true, true, true],
    [false, true, false],
    [false, true, false],
    [false, true, false],
    [false, true, false],
]);
const U: Letter<3, 5> = Letter::new(&[
    [true, false, true],
    [true, false, true],
    [true, false, true],
    [true, false, true],
    [true, true, true],
]);
const V: Letter<3, 5> = Letter::new(&[
    [true, false, true],
    [true, false, true],
    [true, false, true],
    [true, false, true],
    [false, true, false],
]);
const W: Letter<5, 5> = Letter::new(&[
    [true, false, true, false, true],
    [true, false, true, false, true],
    [true, false, true, false, true],
    [true, false, true, false, true],
    [true, true, true, true, true],
]);
const X: Letter<3, 5> = Letter::new(&[
    [true, false, true],
    [true, false, true],
    [false, true, false],
    [true, false, true],
    [true, false, true],
]);
const Y: Letter<3, 5> = Letter::new(&[
    [true, false, true],
    [true, false, true],
    [false, true, false],
    [false, true, false],
    [false, true, false],
]);
const Z: Letter<3, 5> = Letter::new(&[
    [true, true, true],
    [false, false, true],
    [false, true, false],
    [true, false, false],
    [true, true, true],
]);
const ZERO: Letter<3, 5> = Letter::new(&[
    [false, true, false],
    [true, false, true],
    [true, false, true],
    [true, false, true],
    [false, true, false],
]);
const ONE: Letter<1, 5> = Letter::new(&[[true], [true], [true], [true], [true]]);
const TWO: Letter<3, 5> = Letter::new(&[
    [false, true, true],
    [true, false, true],
    [false, false, true],
    [false, true, false],
    [true, true, true],
]);
const THREE: Letter<2, 5> = Letter::new(&[
    [true, true],
    [false, true],
    [true, true],
    [false, true],
    [true, true],
]);
const FOUR: Letter<3, 5> = Letter::new(&[
    [false, true, true],
    [true, false, true],
    [true, true, true],
    [false, false, true],
    [false, false, true],
]);
const FIVE: Letter<2, 5> = Letter::new(&[
    [true, true],
    [true, false],
    [true, true],
    [false, true],
    [true, false],
]);
const SIX: Letter<3, 5> = Letter::new(&[
    [false, true, true],
    [true, false, false],
    [true, true, false],
    [true, false, true],
    [false, true, false],
]);
const SEVEN: Letter<2, 5> = Letter::new(&[
    [true, true],
    [false, true],
    [false, true],
    [true, false],
    [true, false],
]);
const EIGHT: Letter<3, 5> = Letter::new(&[
    [true, true, true],
    [true, false, true],
    [true, true, true],
    [true, false, true],
    [true, true, true],
]);
const NINE: Letter<3, 5> = Letter::new(&[
    [false, true, false],
    [true, false, true],
    [false, true, true],
    [false, false, true],
    [true, true, true],
]);
const COLON: Letter<1, 5> = Letter::new(&[[false], [true], [false], [true], [false]]);
const SEMICOLON: Letter<1, 5> = Letter::new(&[[false], [true], [false], [true], [true]]);
const LT: Letter<2, 5> = Letter::new(&[
    [false, false],
    [false, true],
    [true, false],
    [false, true],
    [false, true],
]);
const GT: Letter<2, 5> = Letter::new(&[
    [false, false],
    [true, false],
    [false, true],
    [true, false],
    [false, true],
]);
const EQ: Letter<2, 5> = Letter::new(&[
    [false, false],
    [true, true],
    [false, false],
    [true, true],
    [false, false],
]);
const QUESTION: Letter<3, 5> = Letter::new(&[
    [false, true, false],
    [true, false, true],
    [false, true, false],
    [false, false, false],
    [false, true, false],
]);
const AT: Letter<5, 5> = Letter::new(&[
    [false, true, true, true, false],
    [true, false, false, false, true],
    [true, false, true, false, true],
    [true, false, false, true, true],
    [false, true, true, true, false],
]);
/// Table mapping ascii characters to their bitmap representation.
const TABLE: &[&'static dyn ColorRender] = &[
    &ZERO, &ONE, &TWO, &THREE, &FOUR, &FIVE, &SIX, &SEVEN, &EIGHT, &NINE, &COLON, &SEMICOLON, &LT,
    &EQ, &GT, &QUESTION, &AT, &A, &B, &C, &D, &E, &F, &G, &H, &I, &J, &K, &L, &M, &N, &O, &P, &Q,
    &R, &S, &T, &U, &V, &W, &X, &Y, &Z,
];
