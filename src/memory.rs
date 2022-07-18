use std::io::Cursor;

use bitflags::bitflags;
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};

const MEM_SIZE: usize = 1024 * 128;

#[derive(Debug)]
pub struct Memory {
    storage: Cursor<Vec<u8>>,
    pub video: Video,
}

impl Memory {
    pub fn new() -> Self {
        Self {
            storage: Cursor::new(vec![0x0; MEM_SIZE]),
            video: Video::new(),
        }
    }

    pub fn read_byte(&mut self, pos: u16) -> u8 {
        match pos {
            0x0000 ..= 0x3FFF => {
                // Memory bank 0
                self.read_byte_from_storage(pos)
            }
            0x4000 ..= 0x7FFF => {
                // Switchable memory bank 01..max, todo
                self.read_byte_from_storage(pos)
            }
            VRAM_START..=0x9FFF => {
                unimplemented!("Reading vram")
            }
            0xA000 ..= 0xBFFF => {
                self.read_byte_from_storage(pos)
            }
            0xFF00 ..= 0xFF7F => {
                self.read_io_register(pos)
            }
            _ => {
                unimplemented!("Memory map not implemented for {:#04X}", pos)
            }
        }

    }

    fn read_byte_from_storage(&mut self, pos: u16) -> u8 {
        self.storage.set_position(pos as u64);
        self.storage.read_u8().unwrap()
    }

    pub fn read_word(&mut self, pos: u16) -> u16 {
        self.storage.set_position(pos as u64);
        self.storage.read_u16::<LittleEndian>().unwrap()
    }

    pub fn write_byte(&mut self, pos: u16, byte: u8) {
        self.storage.set_position(pos as u64);
        self.storage.write_u8(byte).unwrap()
    }

    pub fn write_word(&mut self, pos: u16, word: u16) {
        self.storage.set_position(pos as u64);
        self.storage.write_u16::<LittleEndian>(word).unwrap()
    }

    fn read_io_register(&self, pos: u16) -> u8 {
        match pos {
            0xFF41 => {
                self.video.lcd_status.bits
            }
            0xFF44 => {
                self.video.line // LY, LCD y coordinate
            }
            0xFF45 => {
                unimplemented!()
            }
            _ => {
                unimplemented!("IO register not implemented")
            }
        }
    }
}

const SCREEN_WIDTH: usize = 160;
const SCREEN_HEIGHT: usize = 144;
const VRAM_START: u16 = 0x8000;

#[derive(Debug)]
pub struct Video {
    pixels: [u8; SCREEN_WIDTH * SCREEN_HEIGHT * 4],
    cycle_step: usize,
    mode_step: usize,
    line: u8,
    mode: Mode,
    // lcdc
    lcd_control: LcdControl,
    // stat
    lcd_status: LcdStatus,
}


#[derive(Debug)]
enum Mode {
    OamRead,
    VramRead,
    HBlank,
    VBlank,
}

bitflags! {
    struct LcdStatus: u8 {
        const LYC = 0b00000100;
    }

    struct LcdControl: u8 {
        const LCD_ENABLE =              0b10000000;
        const WINDOW_TILE_MAP =         0b01000000;
        const WINDOW_ENABLE =           0b00100000;
        const WINDOW_BG_ADDRES_MODE =   0b00010000;
        const BG_TILE_MAP =             0b00001000;
        const OBJ_SIZE =                0b00000100;
        const OBJ_ENABLE =              0b00000010;
        const WINDOW_BG_DISPLAY =       0b00000001;

    }
}

impl Video {
    pub fn new() -> Self {
        Self {
            pixels: [0;SCREEN_WIDTH * SCREEN_HEIGHT * 4],
            mode: Mode::OamRead,
            cycle_step: 0,
            mode_step: 0,
            line: 0,
            lcd_control: LcdControl::empty(),
            lcd_status: LcdStatus::empty(),
        }
    }

    pub fn step(&mut self) {
        //self.cycle_step += 1;
        self.mode_step += 1;

        match self.mode {
            Mode::OamRead => {
                if self.mode_step >= 80 {
                    self.mode_step = 0;
                    self.mode = Mode::VramRead
                }
            }
            Mode::VramRead => {
                if self.mode_step >= 172
                {
                    self.mode_step = 0;
                    self.mode = Mode::HBlank;
                    // write scanline
                }
            }
            Mode::HBlank => {
                if self.mode_step >= 204
                {
                    self.mode_step = 0;
                    self.line += 1;

                    if self.line == 143 {
                        self.mode = Mode::VBlank;
                    } else {
                        self.mode = Mode::OamRead;
                    }
                }
            }
            Mode::VBlank => {
                if self.mode_step >= 456 {
                    self.mode_step = 0;
                    self.line += 1;

                    if self.line > 153 {
                        self.mode = Mode::OamRead;
                        self.line = 0;
                    }
                }
            }
        }
    }
}
