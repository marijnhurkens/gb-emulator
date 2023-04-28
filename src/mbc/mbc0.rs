use std::io::Cursor;

use byteorder::ReadBytesExt;

use crate::mbc::Mbc;

#[derive(Debug)]
pub struct Mbc0 {
    storage: Cursor<Vec<u8>>,
}

impl Mbc0 {
    pub fn new(rom: Vec<u8>) -> Self {
        Self {
            storage: Cursor::new(rom),
        }
    }
}

impl Mbc for Mbc0 {
    fn read_rom(&mut self, pos: u16) -> u8 {
        self.storage.set_position(pos as u64);
        self.storage.read_u8().unwrap()
    }

    fn read_ram(&mut self, _: u16) -> u8 {
        0
    }

    fn write_rom(&mut self, _: u16, _: u8) {}

    fn write_ram(&mut self, _: u16, _: u8) {}
}
