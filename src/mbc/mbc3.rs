use std::io::Cursor;

use byteorder::ReadBytesExt;

use crate::mbc::Mbc;

#[derive(Debug)]
pub struct Mbc3 {
    storage: Cursor<Vec<u8>>,
    rom_bank: u16,
    ram_bank: u8,
    ram_rtc_enable: bool,
    banking_mode: bool,
    rom_size: u64,
    rtc_reg: u8,
}

impl Mbc3 {
    pub fn new(rom: Vec<u8>, header_rom_size: u8) -> Self {
        Self {
            storage: Cursor::new(rom),
            rom_bank: 1,
            ram_bank: 1,
            rtc_reg: 0x08,
            ram_rtc_enable: false,
            banking_mode: false,
            rom_size: 32 * 1024 * (1 << header_rom_size),
        }
    }
}

impl Mbc for Mbc3 {
    fn read_rom(&mut self, pos: u16) -> u8 {
        let index: u64 = if pos < 0x4000 {
            pos as u64
        } else {
            (pos as u64 & 0x3FFF) | (self.rom_bank as u64 * 0x4000)
        };

        self.storage.set_position(index);
        self.storage.read_u8().unwrap()
    }

    fn read_ram(&mut self, _: u16) -> u8 {
        panic!()
    }

    fn write_rom(&mut self, pos: u16, val: u8) {
        match pos {
            0x0000..=0x1FFF => {
                self.ram_rtc_enable = (val & 0xF) == 0xA;
            }
            0x2000..=0x3FFF => {
                if val == 0x0 {
                    self.rom_bank = 1;
                }

                self.rom_bank = val as u16;
            }
            0x4000..=0x5FFF => {
                if val <= 0x03 {
                    self.ram_bank = val;
                }

                if (0x08..=0x0c).contains(&val) {
                    self.rtc_reg = val;
                }
            }
            0x6000..=0x7FFF => {
                self.banking_mode = (val & 0x01) == 0x01;
            }
            _ => panic!("not implemented"),
        }
    }

    fn write_ram(&mut self, _: u16, _: u8) {}
}
