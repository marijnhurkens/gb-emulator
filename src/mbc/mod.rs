use tracing::{event, Level};

use crate::cartridge::Cartridge;
use crate::mbc::mbc0::Mbc0;
use crate::mbc::mbc1::Mbc1;

mod mbc0;
mod mbc1;

pub trait Mbc: Send {
    fn read_rom(&mut self, pos: u16) -> u8;
    fn read_ram(&mut self, pos: u16) -> u8;
    fn write_rom(&mut self, pos: u16, data: u8);
    fn write_ram(&mut self, pos: u16, data: u8);
}

pub fn from_cartridge(cartridge: Cartridge) -> Box<dyn Mbc> {
    event!(Level::INFO, "Running {:}", cartridge.header.title);
    event!(
        Level::INFO,
        "Cartridge type {:#04X}",
        cartridge.header.cartridge_type
    );

    match cartridge.header.cartridge_type {
        0x00 => Box::new(Mbc0::new(cartridge.data)),
        0x01..=0x03 => Box::new(Mbc1::new(cartridge.data, cartridge.header.rom_size)),
        _ => panic!(
            "Unknown cartridge type {:#04X}",
            cartridge.header.cartridge_type
        ),
    }
}
