use std::io::{Cursor, Read, Write};

use bitflags::bitflags;
use bitvec::macros::internal::funty::Fundamental;
use byteorder::{ReadBytesExt, WriteBytesExt};
use tracing::{event, Level};

use crate::cpu::CPU_FREQ;
use crate::video::{LcdControl, LcdStatus, Video, VRAM_START};


const MEM_SIZE: usize = 1024 * 128;
const DIVIDER_REG_CYCLES_PER_STEP: u32 = ((16_384.0 / CPU_FREQ) * CPU_FREQ) as u32;

bitflags! {
    pub struct InterruptFlags: u8 {
        const VBLANK   =  0b00000001;
        const LCD_STAT =  0b00000010;
        const TIMER    =  0b00000100;
        const SERIAL   =  0b00001000;
        const JOYPAD   =  0b00010000;
    }

    struct TimerControl: u8 {
        const TIMER_ENABLE = 0b00000100;
        const TIMER_1024   = 0b00000000;
        const TIMER_16     = 0b00000001;
        const TIMER_64     = 0b00000010;
        const TIMER_256    = 0b00000011;
    }

    struct SerialControl: u8 {
        const TRANSFER_START    = 0b10000000;
        const SHIFT_CLOCK       = 0b00000001;
    }
}

#[derive(Debug)]
pub struct Memory {
    storage: Cursor<Vec<u8>>,
    rom: Vec<u8>,
    pub video: Video,
    pub interrupt_flags: InterruptFlags,
    pub interrupt_enable: InterruptFlags,
    div_step: u32,
    div: u8, // divider register
    tima_step: u32,
    tima: u8, // timer counter
    tma: u8,  // timer counter modulo
    tac: TimerControl,
    buttons: u8,
    bcps: u8,
    bcpd: u8,
    sb: u8,
    sc: SerialControl,
}

impl Memory {
    pub fn new(rom: Vec<u8>) -> Self {
        Self {
            storage: Cursor::new(vec![0x0; MEM_SIZE]),
            rom,
            video: Video::new(),
            interrupt_flags: InterruptFlags::VBLANK,
            interrupt_enable: InterruptFlags::empty(),
            div_step: 0,
            div: 0x18,
            tima_step: 0,
            tima: 0,
            tma: 0,
            tac: TimerControl::empty(),
            buttons: 0xFF,
            bcps: 0,
            bcpd: 0,
            sb: 0,
            sc: SerialControl::empty(),
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
                self.read_byte_from_rom(pos)
            }
            VRAM_START..=0x9FFF => self.video.read_byte(pos),
            0xA000..=0xBFFF => self.read_byte_from_storage(pos),
            0xC000..=0xCFFF => self.read_byte_from_storage(pos),
            0xD000..=0xDFFF => self.read_byte_from_storage(pos),
            0xE000..=0xFDFF => self.read_byte_from_storage(pos - 0x2000),
            0xFE00..=0xFE9F => self.video.read_byte_oam(pos),
            0xFF00..=0xFF07 => self.read_io_register(pos),
            0xFF0F => self.interrupt_flags.bits,
            0xFF10..=0xFF7F => self.read_io_register(pos),
            0xFF80..=0xFFFE => {
                // High ram, todo
                self.read_byte_from_storage(pos)
            }
            0xFFFF => self.interrupt_enable.bits,
            _ => {
                unimplemented!("Memory map not implemented for {:#08X}", pos)
            }
        };

        //print!("read byte {:#08X} -> {:#04X} | ", pos, res);

        res
    }

    fn read_byte_from_storage(&mut self, pos: u16) -> u8 {
        self.storage.set_position(pos as u64);
        self.storage.read_u8().unwrap()
    }

    fn read_byte_from_rom(&self, pos: u16) -> u8 {
        self.rom[pos as usize]
    }

    pub fn write_byte(&mut self, pos: u16, byte: u8) {
        event!(
            Level::DEBUG,
            "memory write byte {:#04X} at {:#06X}",
            byte,
            pos
        );
        match pos {
            0x0000..=0x7FFF => (), //self.write_byte_to_storage(pos, byte), //print!("write rom, not implemented |"),
            VRAM_START..=0x9FFF => self.video.write_byte(pos, byte),
            0xA000..=0xBFFF => self.write_byte_to_storage(pos, byte), // ??
            0xC000..=0xDFFF => self.write_byte_to_storage(pos, byte), // wram
            0xE000..=0xFDFF => self.write_byte_to_storage(pos - 0x2000, byte), //echo ram
            0xFE00..=0xFE9F => self.video.write_byte_oam(pos, byte),
            0xFEA0..=0xFEFF => self.write_byte_to_storage(pos, byte), // not usable
            0xFF00..=0xFF07 => self.write_io_register(pos, byte),
            0xFF0F => self.write_interrupt_flags(byte),
            0xFF10..=0xFF26 => self.write_io_register(pos, byte), // audio
            0xFF40..=0xFF55 => self.write_io_register(pos, byte),
            0xFF68 => self.write_bcps_palette(byte),
            0xFF69 => self.write_bcpd_palette(byte),
            0xFF7F..=0xFFFE => self.write_byte_to_storage(pos, byte), // high ram
            0xFFFF => self.write_interrupt_enable(byte),
            _ => panic!(
                "Unimplemented memory write to addr {:#04X} -> {:#04X}",
                pos, byte
            ),
        }
    }

    fn write_interrupt_flags(&mut self, byte: u8) {
        self.interrupt_flags = InterruptFlags::from_bits(byte).unwrap_or_else(|| {
            event!(Level::ERROR, "Wrong IF write: {:#08b}", byte);
            self.interrupt_flags
        });
    }

    fn write_interrupt_enable(&mut self, byte: u8) {
        self.interrupt_enable = InterruptFlags::from_bits(byte).unwrap_or_else(|| {
            event!(Level::ERROR, "Wrong IE write: {:#08b}", byte);
            self.interrupt_enable
        });
    }

    fn write_byte_to_storage(&mut self, pos: u16, byte: u8) {
        self.storage.set_position(pos as u64);
        self.storage.write_u8(byte).unwrap()
    }

    pub fn read_word(&mut self, pos: u16) -> u16 {
        let lo = self.read_byte(pos) as u16;
        let hi = self.read_byte(pos + 1) as u16;
        lo | (hi << 8)
    }

    pub fn write_word(&mut self, pos: u16, word: u16) {
        let bytes = word.to_le_bytes();
        self.write_byte(pos, bytes[0]);
        self.write_byte(pos + 1, bytes[1]);
    }

    fn read_io_register(&self, pos: u16) -> u8 {
        match pos {
            0xFF00 => {
                // joypad input
                self.buttons
            }
            0xFF01 => {
                event!(Level::WARN, "SB read, not implemented");
                0
            } // serial transfer not implemented
            0xFF02 => {
                event!(Level::WARN, "SC read, not implemented");
                0
            } // serial control not implemented
            0xFF04 => self.div,
            0xFF40 => self.video.lcd_control.bits(),
            0xFF41 => self.video.lcd_status.bits(),
            0xFF42 => self.video.scy,
            0xFF43 => self.video.scx,
            0xFF44 => {
                self.video.line // LY, LCD y coordinate
                // 0x90
            }
            0xFF45 => {
                unimplemented!()
            }
            _ => {
                unimplemented!("IO register not implemented for {:#06X}", pos)
            }
        }
    }

    fn write_io_register(&mut self, pos: u16, byte: u8) {
        match pos {
            0xFF00 => self.buttons = byte,
            0xFF01 => self.sb = byte,
            0xFF02 => self.sc = SerialControl::from_bits(byte).unwrap(),
            0xFF04 => self.div = 0, // always resets when written
            0xFF05 => self.tima = byte,
            0xFF06 => self.tma = byte,
            0xFF07 => self.tac = TimerControl::from_bits(byte).unwrap(),
            0xFF10..=0xFF26 => event!(Level::WARN, "Audio register set, not implemented"),
            0xFF40 => self.video.lcd_control = LcdControl::from_bits(byte).unwrap(),
            0xFF41 => self.video.lcd_status = LcdStatus::from_bits(byte).unwrap(),
            0xFF42 => self.video.scy = byte,
            0xFF43 => self.video.scx = byte,
            0xFF45 => self.video.lyc = byte,
            0xFF46 => self.oam_transfer(byte),
            0xFF47 => self.video.bg_palette = byte,
            0xFF48 => self.video.obj_0_palette = byte,
            0xFF49 => self.video.obj_1_palette = byte,
            0xFF4A => self.video.window_y = byte,
            0xFF4B => self.video.window_x = byte,
            0xFF4F => self.video.bank_select = byte,
            0xFF50 => event!(Level::WARN, "Disable boot rom, not implemented"),
            _ => {
                unimplemented!("IO register not implemented for {:#08X}", pos)
            }
        }
    }

    fn write_bcps_palette(&mut self, byte: u8) {
        self.bcps = byte;
    }

    fn write_bcpd_palette(&mut self, byte: u8) {
        self.bcpd = byte;
    }

    fn oam_transfer(&mut self, byte: u8) {
        let source_start = (byte as u16) << 8;
        println!("OAM TRANSFER: {:#06X}", source_start);
        let mut buf = [0_u8; 0x9f];

        self.storage.set_position(source_start as u64);
        self.storage.read_exact(&mut buf).unwrap();

        self.video.write_oam_transfer(&buf);

        // todo: implement 160 machine cycles
    }

    pub fn step(&mut self) {
        self.div_step += 1;
        if self.div_step > DIVIDER_REG_CYCLES_PER_STEP {
            self.div_step = 0;
            (self.div, _) = self.div.overflowing_add(1);
        }

        self.step_timers();
    }

    fn step_timers(&mut self) {
        if !self.tac.contains(TimerControl::TIMER_ENABLE) {
            return;
        }

        self.tima_step += 1;
        let step = match self.tac.difference(TimerControl::TIMER_ENABLE) {
            TimerControl::TIMER_16 => 16,
            TimerControl::TIMER_64 => 64,
            TimerControl::TIMER_256 => 256,
            TimerControl::TIMER_1024 => 1024,
            _ => panic!("should not happen"),
        };

        if self.tima_step > step {
            let (new_tima, overflow) = self.tima.overflowing_add(1);
            self.tima = new_tima;
            if overflow {
                self.tima = self.tma;
                self.interrupt_flags |= InterruptFlags::TIMER;
            }
        }
    }

    pub fn handle_serial(&mut self) {
        if self.sc.contains(SerialControl::TRANSFER_START) {
            let char = self.sb.as_char().unwrap();
            if char == '\n' {
                println!();
            } else {
                print!("{}", char);
            }
            self.sc -= SerialControl::TRANSFER_START;
        }
    }
}
