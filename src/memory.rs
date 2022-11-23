use std::io::{Cursor, Read};

use bitflags::bitflags;
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};

use crate::SCREEN_BUFFER_SIZE;

const MEM_SIZE: usize = 1024 * 128;
const VRAM_START: u16 = 0x8000;

#[derive(Debug)]
pub struct Memory {
    storage: Cursor<Vec<u8>>,
    rom: Vec<u8>,
    pub video: Video,
    interrupt_flags: InterruptFlags,
    interrupt_enable: InterruptFlags,
}

bitflags! {
    struct InterruptFlags: u8 {
        const VBLANK   =  0b00000001;
        const LCD_STAT =  0b00000010;
        const TIMER    =  0b00000100;
        const SERIAL   =  0b00001000;
        const JOYPAD   =  0b00010000;
    }
}

impl Memory {
    pub fn new(rom: Vec<u8>) -> Self {
        Self {
            storage: Cursor::new(vec![0x0; MEM_SIZE]),
            rom,
            video: Video::new(),
            interrupt_flags: InterruptFlags::empty(),
            interrupt_enable: InterruptFlags::empty(),
        }
    }

    pub fn read_byte(&mut self, pos: u16) -> u8 {
        let res = match pos {
            0x0000..=0x3FFF => {
                // Memory bank 0
                self.read_byte_from_rom(pos)
            }
            0x4000..=0x7FFF => {
                // Switchable memory bank 01..max, todo
                self.read_byte_from_storage(pos)
            }
            VRAM_START..=0x9FFF => self.read_byte_from_vram(pos - VRAM_START),
            0xA000..=0xBFFF => self.read_byte_from_storage(pos),
            0xC000..=0xCFFF => self.read_byte_from_storage(pos),
            0xD000..=0xDFFF => self.read_byte_from_storage(pos),
            0xE000..=0xFDFF => self.read_byte_from_storage(pos - 0x2000),
            0xFF00..=0xFF07 => self.read_io_register(pos),
            0xFF0F => self.interrupt_flags.bits,
            0xFF10..=0xFF7F => self.read_io_register(pos),
            0xFF80..=0xFFFE => {
                // High ram, todo
                self.read_byte_from_storage(pos)
            }
            0xFFFF => self.interrupt_enable.bits,
            _ => {
                unimplemented!("Memory map not implemented for {:#04X}", pos)
            }
        };

        print!("read byte {:#04X} -> {:#04X} | ", pos, res);

        res
    }

    pub fn read_word(&mut self, pos: u16) -> u16 {
        let lo = self.read_byte(pos) as u16;
        let hi = self.read_byte(pos + 1) as u16;
        hi << 8 | lo
    }

    pub fn read_vram(&mut self) -> [u8; SCREEN_BUFFER_SIZE] {
        self.video.vram.set_position(0);
        let mut buffer = [0; SCREEN_BUFFER_SIZE];
        self.video.vram.read_exact(&mut buffer).unwrap();

        buffer
    }

    fn read_byte_from_storage(&mut self, pos: u16) -> u8 {
        self.storage.set_position(pos as u64);
        self.storage.read_u8().unwrap()
    }

    fn read_byte_from_rom(&self, pos: u16) -> u8 {
        self.rom[pos as usize]
    }

    fn read_byte_from_vram(&mut self, pos: u16) -> u8 {
        self.video.vram.set_position(pos as u64);
        self.video.vram.read_u8().unwrap()
    }

    fn write_byte_to_vram(&mut self, pos: u16, byte: u8) {
        self.video.vram.set_position(pos as u64);
        self.video.vram.write_u8(byte).unwrap();
    }

    pub fn write_byte(&mut self, pos: u16, byte: u8) {
        match pos {
            0x0000..=0x7FFF => print!("write rom, not implemented"),
            0x8000..=0x9FFF => self.write_byte_to_vram(pos - VRAM_START, byte),
            0xC000..=0xDFFF => self.write_byte_to_storage(pos, byte), // wram
            0xFE00 ..= 0xFE9F => self.write_byte_to_storage(pos, byte), // sprite attr table
            0xFEA0 ..= 0xFEFF => self.write_byte_to_storage(pos, byte), // not usable
            0xFF01 ..= 0xFF02 => self.write_io_register(pos, byte),
            0xFF0F => self.interrupt_flags = InterruptFlags::from_bits(byte).unwrap(),
            0xFF10 ..= 0xFF26 => self.write_io_register(pos, byte), // audio
            0xFF40 ..= 0xFF4F => self.write_io_register(pos, byte),
            0xFF7F..=0xFFFE => self.write_byte_to_storage(pos, byte), // high ram
            0xFFFF => self.interrupt_enable = InterruptFlags::from_bits(byte).unwrap(),
            _ => panic!(
                "Unimplemented memory write to addr {:#08X} -> {:#04X}",
                pos, byte
            ),
        }
    }

    fn write_byte_to_storage(&mut self, pos: u16, byte: u8) {
        print!("write byte {:#04X} -> {:#04X} | ", pos, byte);
        self.storage.set_position(pos as u64);
        self.storage.write_u8(byte).unwrap()
    }

    pub fn write_word(&mut self, pos: u16, word: u16) {
        print!("write word {:#04X} -> {:#08X} | ", pos, word);

        let bytes = pos.to_le_bytes();
        self.write_byte(pos, bytes[0]);
        self.write_byte(pos + 1, bytes[1]);
    }

    fn read_io_register(&self, pos: u16) -> u8 {
        match pos {
            0xFF00 => {
                // joypad input
                0x0
            }
            0xFF41 => self.video.lcd_status.bits,
            0xFF42 => self.video.scy,
            0xFF43 => self.video.scx,
            0xFF44 => {
                self.video.line // LY, LCD y coordinate
            }
            0xFF45 => {
                unimplemented!()
            }
            _ => {
                unimplemented!("IO register not implemented for {:#04X}", pos)
            }
        }
    }

    fn write_io_register(&mut self, pos: u16, byte: u8) {
        match pos {
            0xFF01 | 0xFF02 => print!("Serial transfer register set, not implemented"),
            0xFF10 ..= 0xFF26 => print!("Audio register set, not implemented"),
            0xFF40 => self.video.lcd_control = LcdControl::from_bits(byte).unwrap(),
            0xFF41 => self.video.lcd_status = LcdStatus::from_bits(byte).unwrap(),
            0xFF42 => self.video.scy = byte,
            0xFF43 => self.video.scx = byte,
            0xFF47 => self.video.bg_palette = byte,
            0xFF48 => self.video.obj_0_palette = byte,
            0xFF49 => self.video.obj_1_palette = byte,
            _ => {
                unimplemented!("IO register not implemented for {:#04X}", pos)
            }
        }
    }
}

#[derive(Debug)]
pub struct Video {
    cycle_step: usize,
    mode_step: usize,
    vram: Cursor<Vec<u8>>,
    line: u8,
    scy: u8,
    scx: u8,
    mode: Mode,
    // lcdc
    lcd_control: LcdControl,
    // stat
    lcd_status: LcdStatus,
    bg_palette: u8,
    obj_0_palette: u8,
    obj_1_palette: u8,
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
            mode: Mode::OamRead,
            vram: Cursor::new(vec![0; SCREEN_BUFFER_SIZE]),
            cycle_step: 0,
            mode_step: 0,
            line: 0,
            scy: 0,
            scx: 0,
            lcd_control: LcdControl::empty(),
            lcd_status: LcdStatus::empty(),
            bg_palette: 0,
            obj_0_palette: 0,
            obj_1_palette: 0,
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
                if self.mode_step >= 172 {
                    self.mode_step = 0;
                    self.mode = Mode::HBlank;
                    // write scanline
                }
            }
            Mode::HBlank => {
                if self.mode_step >= 204 {
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
