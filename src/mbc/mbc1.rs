use std::io::Cursor;

use byteorder::ReadBytesExt;

use crate::mbc::Mbc;

#[derive(Debug)]
pub struct Mbc1 {
    storage: Cursor<Vec<u8>>,
    rom_bank: u16,
    ram_enable: bool,
    banking_mode: bool,
    rom_size: u64,
}

impl Mbc1 {
    pub fn new(rom: Vec<u8>, header_rom_size: u8) -> Self {
        Self {
            storage: Cursor::new(rom),
            rom_bank: 1,
            ram_enable: false,
            banking_mode: false,
            rom_size: 32 * 1024 * (1 << header_rom_size),
        }
    }
}

impl Mbc for Mbc1 {
    fn read_rom(&mut self, pos: u16) -> u8 {
        let index = if pos < 0x4000 {
            pos
        } else {
            (pos & 0x3FFF) | (self.rom_bank * 0x4000)
        };

        self.storage.set_position(index as u64);
        self.storage.read_u8().unwrap()
    }

    fn read_ram(&mut self, _: u16) -> u8 {
        panic!()
    }

    fn write_rom(&mut self, pos: u16, val: u8) {
        match pos {
            0x0000..=0x1FFF => {
                if (val & 0xF) == 0xA {
                    self.ram_enable = true;
                } else {
                    self.ram_enable = false;
                }
            }
            0x2000..=0x3FFF => {
                // check if the first 5 bits are 0, and set to bank 1 if this is the case
                let lower_bits = (val & 0x1F).max(0x1);

                // mask val to amount of bits needed to represent the amount of banks in the MBC
                let lower_bits = match self.rom_size / (16 * 1024) {
                    0..=2 => lower_bits & 0x1,
                    3..=4 => lower_bits & 0x3,
                    5..=8 => lower_bits & 0x7,
                    9..=16 => lower_bits & 0xf,
                    _ => lower_bits,
                };

                self.rom_bank = (self.rom_bank & 0x60) | lower_bits as u16;
            }
            0x4000..=0x5FFF => {
                panic!("not implemented");
                // self.rom_bank = self.rom_bank & 0x1F | ((val & 0x03) << 5) as u16
            }
            0x6000..=0x7FFF => {
                self.banking_mode = (val & 0x01) == 0x01;
            }
            _ => panic!("not implemented"),
        }
    }

    fn write_ram(&mut self, _: u16, _: u8) {}
}
