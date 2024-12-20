use std::io::{Cursor, Read};
use std::sync::mpsc::Receiver;

use bitflags::bitflags;
use bitvec::macros::internal::funty::Fundamental;
use byteorder::{ReadBytesExt, WriteBytesExt};
use tracing::{event, Level};

use crate::apu::Apu;
use crate::cpu::CPU_FREQ;
use crate::input::Button;
use crate::mbc::Mbc;
use crate::ppu::{LcdControl, LcdStatus, Ppu, VRAM_START};
use crate::{ButtonPosition, KeyMessage};

const MEM_SIZE: usize = 1024 * 128;
const DIVIDER_REG_CYCLES_PER_STEP: u32 = (16_384 / CPU_FREQ) * CPU_FREQ;

bitflags! {
    #[derive(PartialEq, Debug, Copy, Clone)]
    pub struct InterruptFlags: u8 {
        const VBLANK   =  0b00000001;
        const LCD_STAT =  0b00000010;
        const TIMER    =  0b00000100;
        const SERIAL   =  0b00001000;
        const JOYPAD   =  0b00010000;
    }

    #[derive(PartialEq, Debug, Copy, Clone)]
    struct TimerControl: u8 {
        const TIMER_ENABLE = 0b00000100;
        const TIMER_1024   = 0b00000000;
        const TIMER_16     = 0b00000001;
        const TIMER_64     = 0b00000010;
        const TIMER_256    = 0b00000011;
    }

    struct SerialControl: u8 {
        const TRANSFER_START    = 0b10000000;
        const CLOCK_SPEED       = 0b00000010;
        const SHIFT_CLOCK       = 0b00000001;
    }
}

#[derive(Debug, Default)]
struct KeyState {
    up: ButtonPosition,
    down: ButtonPosition,
    left: ButtonPosition,
    right: ButtonPosition,
    start: ButtonPosition,
    select: ButtonPosition,
    a: ButtonPosition,
    b: ButtonPosition,
}

pub struct Mmu {
    mbc: Box<dyn Mbc>,
    storage: Cursor<Vec<u8>>,
    pub ppu: Ppu,
    pub apu: Apu,
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
    key_state: KeyState,
    key_receiver: Receiver<KeyMessage>,
}

impl Mmu {
    pub fn new(mbc: Box<dyn Mbc>, apu: Apu, ppu: Ppu, key_receiver: Receiver<KeyMessage>) -> Self {
        Self {
            mbc,
            ppu,
            apu,
            key_state: KeyState::default(),
            key_receiver,
            storage: Cursor::new(vec![0x0; MEM_SIZE]),
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
        match pos {
            0x0000..=0x7FFF => self.mbc.read_rom(pos),
            VRAM_START..=0x9FFF => self.ppu.read_byte(pos),
            0xA000..=0xBFFF => self.read_byte_from_storage(pos),
            0xC000..=0xCFFF => self.read_byte_from_storage(pos),
            0xD000..=0xDFFF => self.read_byte_from_storage(pos),
            0xE000..=0xFDFF => self.read_byte_from_storage(pos - 0x2000),
            0xFE00..=0xFE9F => self.ppu.read_byte_oam(pos),
            0xFEA0..=0xFEFF => {
                event!(Level::ERROR, "Prohibited memory access {:#04X}", pos);
                panic!();
            }
            0xFF00..=0xFF02 => self.read_io_register(pos),
            0xFF04..=0xFF07 => self.read_io_register(pos),
            0xFF30..=0xFF3F => self.read_io_register(pos),
            0xFF0F => self.interrupt_flags.bits(),
            0xFF10..=0xFF7F => self.read_io_register(pos),
            0xFF80..=0xFFFE => {
                // High ram, todo
                self.read_byte_from_storage(pos)
            }
            0xFFFF => self.interrupt_enable.bits(),
            _ => {
                //event!(Level::ERROR, "Memory map not implemented for {:#04X}", pos);
                0xFF
            }
        }
    }

    fn read_byte_from_storage(&mut self, pos: u16) -> u8 {
        self.storage.set_position(pos as u64);
        self.storage.read_u8().unwrap()
    }

    pub fn write_byte(&mut self, pos: u16, byte: u8) {
        match pos {
            0x0000..=0x7FFF => self.mbc.write_rom(pos, byte),
            VRAM_START..=0x9FFF => self.ppu.write_byte(pos, byte),
            0xA000..=0xBFFF => self.write_byte_to_storage(pos, byte), // ??
            0xC000..=0xDFFF => self.write_byte_to_storage(pos, byte), // wram
            0xE000..=0xFDFF => self.write_byte_to_storage(pos - 0x2000, byte), //echo ram
            0xFE00..=0xFE9F => self.ppu.write_byte_oam(pos, byte),
            0xFEA0..=0xFEFF => self.write_byte_to_storage(pos, byte), // not usable
            0xFF00..=0xFF07 => self.write_io_register(pos, byte),
            0xFF0F => self.write_interrupt_flags(byte),
            0xFF10..=0xFF26 => self.write_io_register(pos, byte), // audio
            0xFF30..=0xFF3F => self.write_io_register(pos, byte), // audio
            0xFF40..=0xFF55 => self.write_io_register(pos, byte),
            0xFF68 => self.write_bcps_palette(byte),
            0xFF69 => self.write_bcpd_palette(byte),
            0xFF6A => self.write_io_register(pos, byte),
            0xFF6B => self.write_io_register(pos, byte),
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
            panic!();
        });
    }

    fn write_interrupt_enable(&mut self, byte: u8) {
        self.interrupt_enable = InterruptFlags::from_bits(byte).unwrap_or_else(|| {
            event!(Level::ERROR, "Wrong IE write: {:#08b}", byte);
            panic!();
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

    fn update_key_state(&mut self) {
        if let Ok(key_message) = self.key_receiver.try_recv() {
            match key_message.key {
                Button::Left => self.key_state.left = key_message.key_position,
                Button::Right => self.key_state.right = key_message.key_position,
                Button::Up => self.key_state.up = key_message.key_position,
                Button::Down => self.key_state.down = key_message.key_position,
                Button::A => self.key_state.a = key_message.key_position,
                Button::B => self.key_state.b = key_message.key_position,
                Button::Start => self.key_state.start = key_message.key_position,
                Button::Select => self.key_state.select = key_message.key_position,
            }
        }
    }

    /// Returns the key state and if it has changed
    fn read_key_state(&mut self) -> (u8, bool) {
        let new_state = if self.buttons & 0x20 == 0x0 {
            (self.buttons & 0xf0)
                | ((!bool::from(self.key_state.start) as u8) << 3)
                | ((!bool::from(self.key_state.select) as u8) << 2)
                | ((!bool::from(self.key_state.b) as u8) << 1)
                | (!bool::from(self.key_state.a) as u8)
        } else if self.buttons & 0x10 == 0x0 {
            (self.buttons & 0xf0)
                | ((!bool::from(self.key_state.down) as u8) << 3)
                | ((!bool::from(self.key_state.up) as u8) << 2)
                | ((!bool::from(self.key_state.left) as u8) << 1)
                | (!bool::from(self.key_state.right) as u8)
        } else {
            return (0xFF, false);
        };

        if self.buttons != new_state {
            self.buttons = new_state;
            return (self.buttons, true);
        }

        (self.buttons, false)
    }

    fn read_io_register(&mut self, pos: u16) -> u8 {
        match pos {
            0xFF00 => {
                // joypad input
                self.read_key_state().0
            }
            0xFF01 => {
                //event!(Level::WARN, "SB read, not implemented");
                0
            } // serial transfer not implemented
            0xFF02 => {
                //event!(Level::WARN, "SC read, not implemented");
                0
            } // serial control not implemented
            0xFF04 => self.div,
            0xFF05 => self.tima,
            0xFF06 => self.tma,
            0xFF07 => self.tac.bits(),
            0xFF10..=0xFF26 => self.apu.read_register(pos),
            0xFF30..=0xFF3F => {
                //event!(Level::WARN, "Audio wave read, not implemented");
                0
            }
            0xFF40 => self.ppu.lcd_control.bits(),
            0xFF41 => self.ppu.lcd_status.bits(),
            0xFF42 => self.ppu.scy,
            0xFF43 => self.ppu.scx,
            0xFF44 => {
                self.ppu.line // LY, LCD y coordinate
                              // 0x90
            }
            0xFF45 => {
                unimplemented!()
            }
            0xFF48 => self.ppu.obj_0_palette,
            0xFF49 => self.ppu.obj_1_palette,
            0xFF4A => self.ppu.window_y,
            0xFF4B => self.ppu.window_x,
            0xFF4D => {
                // CGB speed switch
                0xFF
            }
            0xFF6A | 0xFF6B => {
                //event!(Level::WARN, "CGB only not supported");
                0
            }
            _ => {
                unimplemented!("IO register not implemented for {:#06X}", pos)
            }
        }
    }

    fn write_io_register(&mut self, pos: u16, byte: u8) {
        match pos {
            0xFF00 => {
                self.buttons = (self.buttons & 0xcf) | (byte & 0x30) // button states are read only
            }
            0xFF01 => self.sb = byte,
            0xFF02 => self.sc = SerialControl::from_bits(byte).unwrap(),
            0xFF04 => self.div = 0, // always resets when written
            0xFF05 => self.tima = byte,
            0xFF06 => self.tma = byte,
            0xFF07 => self.tac = TimerControl::from_bits(byte).unwrap(),
            0xFF10..=0xFF26 => self.apu.write_register(pos, byte),
            0xFF30..=0xFF3F => (), //event!(Level::WARN, "Audio wave write, not implemented"),
            0xFF40 => {
                self.ppu.lcd_control = LcdControl::from_bits(byte).unwrap();
            }
            0xFF41 => self.ppu.lcd_status = LcdStatus::from_bits(byte & 0x78).unwrap(),
            0xFF42 => self.ppu.scy = byte,
            0xFF43 => self.ppu.scx = byte,
            0xFF45 => {
                self.ppu.lyc = byte;
            }
            0xFF46 => self.oam_transfer(byte),
            0xFF47 => {
                self.ppu.bg_palette = byte;
            }
            0xFF48 => self.ppu.obj_0_palette = byte,
            0xFF49 => self.ppu.obj_1_palette = byte,
            0xFF4A => self.ppu.window_y = byte,
            0xFF4B => self.ppu.window_x = byte,
            0xFF4D => (), //event!(Level::WARN,"KEY1 prepare speed switch (CGB only) not supported"),
            0xFF4F => self.ppu.bank_select = byte,
            0xFF50 => (), //event!(Level::WARN, "Disable boot rom, not implemented"),
            0xFF6A | 0xFF6B => (), //event!(Level::WARN, "CGB only not supported"),
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
        let mut buf = [0_u8; 0x9f];

        self.storage.set_position(source_start as u64);
        self.storage.read_exact(&mut buf).unwrap();

        self.ppu.write_oam_transfer(&buf);

        // todo: implement 160 machine cycles
    }

    pub fn step(&mut self) {
        self.div_step += 1;
        if self.div_step > DIVIDER_REG_CYCLES_PER_STEP {
            self.div_step = 0;
            self.div = self.div.wrapping_add(1);

            // check for new key events
            self.update_key_state();
        }

        if self.read_key_state().1 {
            self.interrupt_flags |= InterruptFlags::JOYPAD;
        }

        self.step_timers();

        self.interrupt_flags = self.ppu.step(self.interrupt_flags);
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
            self.tima_step = 0;
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
