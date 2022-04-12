use atomic_polyfill::Ordering;
use pio::ArrayVec;

use crate::{text::render_text, Color, Framebuffer, InputState, Position, SubState};

#[derive(Copy, Clone, Debug)]
pub enum Direction {
    PosX,
    NegX,
    PosY,
    NegY,
}

impl Direction {
    pub fn rotate_cwise(self) -> Self {
        match self {
            Direction::PosX => Direction::NegY,
            Direction::NegX => Direction::PosY,
            Direction::PosY => Direction::PosX,
            Direction::NegY => Direction::NegX,
        }
    }

    pub fn rotate_ccwise(self) -> Self {
        match self {
            Direction::PosX => Direction::PosY,
            Direction::NegX => Direction::NegY,
            Direction::PosY => Direction::NegX,
            Direction::NegY => Direction::PosX,
        }
    }
}

#[derive(Debug)]
pub enum SnakeState {
    Menu {
        selected: MenuItem,
        frame: usize,
    },
    Play {
        direction: Direction,
        /// Snake tiles in an array vec. Last element is the head
        snake: ArrayVec<Position, { 16 * 16 }>,
        apple: Position,
    },
}

impl SubState for SnakeState {
    fn update(&mut self, input: &InputState) -> embedded_time::duration::Milliseconds {
        match self {
            SnakeState::Menu { selected, frame } => {
                *frame = (*frame + 1) % 16;
                match (
                    input.confirm.load(Ordering::Acquire),
                    input.back.load(Ordering::Acquire),
                    input.left.load(Ordering::Acquire),
                    input.right.load(Ordering::Acquire),
                ) {
                    // Start playing if button is pressed
                    (1 | 2, 0, 0, 0) => *state = SnakeState::play(),
                    _ => (),
                }
            }
            SnakeState::Play {
                direction,
                snake,
                apple,
            } => {
                let new_direction = match (
                    input.left.load(Ordering::Acquire),
                    input.right.load(Ordering::Acquire),
                    input.confirm.load(Ordering::Acquire),
                    input.back.load(Ordering::Acquire),
                ) {
                    (1 | 2, 0, _, _) => direction.rotate_ccwise(),
                    (0, 1 | 2, _, _) => direction.rotate_cwise(),
                    _ => *direction,
                };

                *direction = new_direction;

                // move snake in the direction
                let new_head = snake.last().unwrap().add_dir(new_direction);

                // CHeck position is whithin bounds
                if let Some(new_head) = new_head {
                    let snake_len = snake.len();
                    // "shift" snake forward, then add new head
                    for i in 0..(snake_len - 1) {
                        snake[i] = snake[i + 1];
                    }

                    snake[snake_len - 1] = new_head;
                } else {
                    *state = SnakeState::menu();
                }
            }
        }
    }

    fn render(&self, framebuffer: &mut Framebuffer) {
        match self {
            SnakeState::Menu { frame, selected } => match selected {
                MenuItem::Start => {
                    render_text(
                        framebuffer,
                        "START",
                        Position::new(1, 6),
                        [[Color::new(0x0F, 0, 0), Color::new(0, 0x0F, 0)][frame / 8]]
                            .into_iter()
                            .cycle(),
                    );
                }
            },
            SnakeState::Play {
                snake,
                apple,
                direction,
            } => {
                *framebuffer = Framebuffer::default();

                let head_idx = snake.len() - 1;
                snake.iter().enumerate().for_each(|(i, pos)| {
                    // color snake green, head blueish
                    framebuffer[pos.y as usize][pos.x as usize] =
                        Color::new(0, 0x0a, ((i == head_idx) as u8) << 4)
                });

                framebuffer[apple.y as usize][apple.y as usize] = Color::new(0x0a, 0, 0);
            }
        }
    }
}

#[derive(Debug)]
pub enum MenuItem {
    Start,
}

impl SnakeState {
    /// Return the initial [`SnakeState::Play`] state
    pub fn play() -> Self {
        let mut snake = ArrayVec::new();
        snake.push(Position::new(0, 5));
        snake.push(Position::new(1, 5));
        snake.push(Position::new(2, 5));
        snake.push(Position::new(3, 5));

        SnakeState::Play {
            direction: Direction::NegX,
            snake,
            apple: Position::new(2, 2),
        }
    }
    /// Return the initial [`SnakeState::Menu`] state
    pub fn menu() -> Self {
        SnakeState::Menu {
            frame: 0,
            selected: MenuItem::Start,
        }
    }
}
