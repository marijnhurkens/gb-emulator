extern crate core;

use std::ffi::OsString;
use std::fs::File;
use std::io::Read;

use clap::Parser;

use crate::cartridge::Cartridge;
use crate::cpu::CPU;

mod cartridge;
mod cpu;
mod helpers;
mod memory;

#[derive(Parser, Debug)]
struct Args {
    rom_file: OsString,
}

fn main() {
    let args = Args::parse();

    let mut rom = File::open(&args.rom_file).unwrap();
    let mut data = vec![];

    rom.read_to_end(&mut data).unwrap();

    let cartridge = Cartridge::load_rom(data);
    let mut cpu = CPU::load_cartridge(cartridge);

    cpu.run();
}
