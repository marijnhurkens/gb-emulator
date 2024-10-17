use ggez::input::keyboard::KeyCode;
use ggez::winit::keyboard::PhysicalKey;

pub type KeyMapping = (PhysicalKey, Button);
pub const KEY_MAP: [KeyMapping; 8] = [
    (PhysicalKey::Code(KeyCode::KeyZ), Button::A),
    (PhysicalKey::Code(KeyCode::KeyX), Button::B),
    (PhysicalKey::Code(KeyCode::ArrowUp), Button::Up),
    (PhysicalKey::Code(KeyCode::ArrowDown), Button::Down),
    (PhysicalKey::Code(KeyCode::ArrowLeft), Button::Left),
    (PhysicalKey::Code(KeyCode::ArrowRight), Button::Right),
    (PhysicalKey::Code(KeyCode::Enter), Button::Start),
    (PhysicalKey::Code(KeyCode::Backslash), Button::Select),
];

#[derive(Debug, Copy, Clone)]
pub struct KeyMessage {
    pub key: Button,
    pub key_position: ButtonPosition,
}

#[derive(Debug, Copy, Clone)]
pub enum Button {
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
pub enum ButtonPosition {
    Pressed,
    Released,
}

impl From<ButtonPosition> for bool {
    fn from(value: ButtonPosition) -> Self {
        match value {
            ButtonPosition::Pressed => true,
            ButtonPosition::Released => false,
        }
    }
}

impl Default for ButtonPosition {
    fn default() -> Self {
        Self::Released
    }
}
