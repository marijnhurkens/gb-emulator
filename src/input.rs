use ggez::input::keyboard::KeyCode;

pub type KeyMapping = (KeyCode, Key);
pub const KEY_MAP: [KeyMapping; 8] = [
    (KeyCode::Z, Key::A),
    (KeyCode::X, Key::B),
    (KeyCode::Up, Key::Up),
    (KeyCode::Down, Key::Down),
    (KeyCode::Left, Key::Left),
    (KeyCode::Right, Key::Right),
    (KeyCode::Return, Key::Start),
    (KeyCode::Backslash, Key::Select),
];

#[derive(Debug, Copy, Clone)]
pub struct KeyMessage {
    pub key: Key,
    pub key_position: KeyPosition,
}

#[derive(Debug, Copy, Clone)]
pub enum Key {
    Left,
    Right,
    Up,
    Down,
    A,
    B,
    Start,
    Select,
}

#[derive(Debug, Copy, Clone)]
pub enum KeyPosition {
    Pressed,
    Released,
}

impl From<KeyPosition> for bool {
    fn from(value: KeyPosition) -> Self {
        match value {
            KeyPosition::Pressed => true,
            KeyPosition::Released => false,
        }
    }
}

impl Default for KeyPosition {
    fn default() -> Self {
        Self::Released
    }
}
