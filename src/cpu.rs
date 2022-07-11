use std::io::Cursor;
use std::thread::sleep;
use std::time;

use bitvec::prelude::*;
use byteorder::{LittleEndian, WriteBytesExt};

const MEM_SIZE: usize = 1024 * 8;

#[derive(Debug)]
pub struct CPU {
    pc: u16,
    sp: u16,
    a: u8,
    // accumulator
    bc: [u8; 2],
    de: [u8; 2],
    hl: [u8; 2],
    flags: Flags,
    interrupts_enabled: bool,
    memory: Cursor<Vec<u8>>,
    cartridge: Cartridge,
}

#[derive(Debug)]
pub struct Flags {
    z: bool,
    // zero
    c: bool,
    // CY or carry
    n: bool,
    h: bool,
}

impl CPU {
    pub fn load_cartridge(cartridge: Cartridge) -> Self {
        CPU {
            pc: 0x0100,
            sp: 0xfffe,
            a: 0x01,
            bc: [0xff, 0x13],
            de: [0x00, 0xc1],
            hl: [0x74, 0x03],
            flags: Flags {
                z: false,
                c: false,
                n: false,
                h: false,
            },
            interrupts_enabled: false,
            memory: Cursor::new(vec![0x0; MEM_SIZE]),
            cartridge,
        }
    }

    pub fn run(&mut self)
    {
        let title = &self.cartridge.header.title;
        println!("Running {title}");

        loop {
            let instruction = self.cartridge.data[self.pc as usize];
            println!("{:x}", instruction);

            match instruction {
                0x00 => self.pc += 1,
                0x21 => {
                    // load the next word (16 bits) into the HL register
                    let data: [u8; 2] = self.next_word();
                    self.hl = data;
                    self.pc += 1;
                }
                0x6f => {
                    self.hl[1] = self.a;
                    self.pc += 1;
                }
                0x99 => {
                    let b = self.bc[1];
                    self.flags.n = true;

                    let mut overflow = false;
                    (self.a, overflow) = self.a.overflowing_sub(b);
                    (self.a, overflow) = self.a.overflowing_sub(1);
                    self.flags.c = overflow;

                    self.flags.z = self.a == 0;
                    self.pc += 1;
                }
                0xAF => {
                    // XOR the A register with itself
                    self.a = self.a ^ self.a;
                    self.pc += 1;
                }
                0xC3 => {
                    // Jump to the address immediately after the current instruction
                    let data: [u8; 2] = self.next_word();
                    let pos: u16 = u16::from_le_bytes(data);
                    self.pc = pos;
                }
                0xFF => {
                    self.push_word(self.pc);
                    self.pc = self.cartridge.data[0x38] as u16;
                }
                _ => {
                    self.debug_registers();
                    panic!("Unknown instruction {:x}", instruction)
                }
            }

            sleep(time::Duration::from_millis(100));
        }
    }

    fn next_byte(&self) -> u8
    {
        let pos = (self.pc + 1) as usize;
        self.cartridge.data[pos]
    }

    fn next_word(&self) -> [u8; 2]
    {
        let a = self.next_byte();
        let b = self.next_byte();

        [b, a]
    }

    fn push_word(&mut self, word: u16) {
        self.sp -= 2;

        self.memory.set_position(self.sp as u64);
        self.memory.write_u16::<LittleEndian>(word).unwrap();
    }

    fn debug_registers(&self)
    {
        println!("Program counter: {:x}", self.pc);
        println!("Stack pointer: {:x}", self.sp);
        println!("A: {:x}", self.a);
        println!("BC: {:x} {:x}", self.bc[0], self.bc[1]);
        println!("DE: {:x} {:x}", self.de[0], self.de[1]);
        println!("HL: {:x} {:x}", self.hl[0], self.hl[1]);
        println!("Flags: {:?}", self.flags);
    }
}


#[derive(Debug)]
pub struct Cartridge {
    pub header: CartridgeHeader,
    pub data: Vec<u8>,
}

#[derive(Debug)]
pub struct CartridgeHeader {
    pub title: String,
    pub cartridge_type: u8,
    pub licensee_code: u8,
    pub rom_size: u8,
    pub ram_size: u8,
}


impl Cartridge {
    pub fn load_rom(data: Vec<u8>) -> Self {
        verify_header(&data).unwrap();

        let header = load_header(&data);

        Cartridge {
            header,
            data,
        }
    }
}


fn load_header(data: &[u8]) -> CartridgeHeader {
    let title = &data[0x0134..=0x0143];
    let cartridge_type = data[0x0147];
    let rom_size = data[0x0148];
    let ram_size = data[0x0149];
    let licensee_code = data[0x014B];

    CartridgeHeader {
        title: String::from_utf8(title.to_vec()).unwrap(),
        cartridge_type,
        rom_size,
        ram_size,
        licensee_code,
    }
}

fn verify_header(data: &[u8]) -> Result<(), &str>
{
    let checksum = &data[0x014D];
    println!("Verifying header checksum..");
    let mut x: u8 = 0;

    for i in 0x0134..=0x014C {
        x = x.overflowing_sub(data[i]).0.overflowing_sub(1).0;
    }

    if x == *checksum {
        Ok(())
    } else {
        Err("Header checksum not valid.")
    }
}