use std::io::Cursor;

use byteorder::ReadBytesExt;

use crate::mbc::Mbc;

#[derive(Debug)]
pub struct Mbc1 {
    storage: Cursor<Vec<u8>>,
    rom_bank: u16,
    ram_enable: bool,
}

impl Mbc1 {
    pub fn new(rom: Vec<u8>) -> Self {
        Self {
            storage: Cursor::new(rom),
            rom_bank: 0,
            ram_enable: false,
        }
    }
}

impl Mbc for Mbc1 {
    fn read_rom(&mut self, pos: u16) -> u8 {
        let index = if pos < 0x4000 {
            pos
        } else {
            (pos & 0x3FFF) + ((self.rom_bank | 0x1) * 0x4000)
        };

        self.storage.set_position(index as u64);
        self.storage.read_u8().unwrap()
    }

    fn read_ram(&mut self, _: u16) -> u8 {
        0
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
                self.rom_bank = (self.rom_bank & 0x60) | (val & 0x1F) as u16;
            }
            0x4000..=0x5FFF => {
                panic!();
                self.rom_bank = self.rom_bank & 0x1F | ((val & 0x03) << 5) as u16
            },
            0x6000..=0x7FFF => {
                panic!("not implemented");
                // self.ram_mode = (v & 0x01) == 0x01;
            }
            _ => panic!("not implemented"),
        }
    }

    fn write_ram(&mut self, _: u16, _: u8) {}
}
