use std::io::Cursor;
use std::thread::sleep;
use std::time;

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};

use crate::cartridge::Cartridge;

const MEM_SIZE: usize = 1024 * 128;

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
    // zero
    z: bool,
    // CY or carry
    c: bool,
    // previous instruction was a subtraction
    n: bool,
    // todo
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

    pub fn run(&mut self) {
        let title = &self.cartridge.header.title;
        println!("Running {title}");

        loop {
            if self.pc as usize >= self.cartridge.data.len() {
                self.debug_registers();
                panic!("PC out of bounds");
            }

            let instruction = self.cartridge.data[self.pc as usize];
            println!("{:X} | {:#04X}", self.pc, instruction);

            match instruction {
                0x00 => self.pc += 1,
                0x01 => {
                    // load next 2 bytes in bc
                    let data: [u8; 2] = self.next_word();
                    self.bc = data;
                    self.pc += 1;
                }
                0x02 => {
                    // write a to memory on location bc
                    self.write_memory_byte(u16::from_le_bytes(self.bc), self.a);
                    self.pc += 1;
                }
                0x03 => {
                    // increment BE by 1
                    self.bc = (u16::from_le_bytes(self.bc) + 1).to_le_bytes();
                    self.pc += 1;
                }
                0x04 => {
                    // increment B by 1
                    self.bc[0] = self.increment(self.bc[0]);
                    self.pc += 1;
                }
                0x05 => {
                    // decrement B by 1
                    self.bc[0] = self.decrement(self.bc[0]);
                    self.pc += 1;
                }
                0x06 => {
                    // load next byte to b
                    let data = self.next_byte();
                    self.bc[0] = data;
                    self.pc += 1;
                }
                0x07 => {
                    self.rlca();
                    self.pc +=1 ;
                }
                0x0E => {
                    // load next byte to c
                    let data = self.next_byte();
                    self.bc[1] = data;
                    self.pc += 1;
                }
                0x14 => {
                    // increment D by 1
                    self.de[0] = self.increment(self.de[0]);
                    self.pc += 1;
                }
                0x15 => {
                    // decrement D by 1
                    self.de[0] = self.decrement(self.de[0]);
                    self.pc += 1;
                }
                0x16 => {
                    // load next byte to d
                    let data = self.next_byte();
                    self.de[0] = data;
                    self.pc += 1;
                }
                0x18 => {
                    // relative jump
                    let pos = self.next_byte();
                    self.pc += pos as u16;
                }
                0x19 => {
                    // add DE to HL
                    self.add_to_reg_hl(self.de);
                    self.pc += 1;
                }
                0x1D => {
                    // decrement E by 1
                    self.de[1] = self.decrement(self.de[1]);
                    self.pc += 1;
                }
                0x1F => {
                    self.rra();
                    self.pc += 1;
                }
                0x20 => {
                    // if z is 0, jump next byte relative steps, otherwise skip next byte
                    if !self.flags.z {
                        let current_addr = self.pc; // store because next_byte increases pc
                        self.pc = current_addr + self.next_byte() as u16;
                    } else {
                        self.pc += 2; // skip data
                    }
                }
                0x21 => {
                    // load the next word (16 bits) into the HL register
                    let data: [u8; 2] = self.next_word();
                    self.hl = data;
                    self.pc += 1;
                }
                0x24 => {
                    // increment H by 1
                    self.hl[0] = self.increment(self.hl[0]);
                    self.pc += 1;
                }
                0x25 => {
                    // decrement H by 1
                    self.hl[0] = self.decrement(self.hl[0]);
                    self.pc += 1;
                }
                0x29 => {
                    // Add HL to HL
                    self.add_to_reg_hl(self.hl);
                    self.pc += 1;
                }
                0x2C => {
                    // increment L by 1
                    self.hl[1] = self.increment(self.hl[1]);
                    self.pc += 1;
                }
                0x2D => {
                    // decrement L by 1
                    self.hl[1] = self.decrement(self.hl[1]);
                    self.pc += 1;
                }
                0x2E => {
                    // set L to next byte
                    self.hl[1] = self.next_byte();
                    self.pc += 1;
                }
                0x2F => {
                    // complement / invert A
                    self.a = !self.a;
                    self.pc += 1;
                }
                0x32 => {
                    // store a in location of hl, and decrement hl
                    let hl_u16 = u16::from_le_bytes(self.hl);
                    self.write_memory_byte(hl_u16, self.a);
                    self.hl = (hl_u16 - 1).to_le_bytes();
                    self.pc += 1;
                }
                0x4A => {
                    // load d to c
                    self.de[0] = self.bc[1];
                    self.pc += 1;
                }
                0x4B => {
                    // load c to e
                    self.bc[1] = self.de[1];
                    self.pc += 1;
                }
                0x4C => {
                    // load h to c
                    self.hl[0] = self.bc[1];
                    self.pc += 1;
                }
                0x4D => {
                    // load l to c
                    self.hl[1] = self.bc[1];
                    self.pc += 1;
                }
                0x4E => {
                    // load memory at position hl to c
                    self.bc[1] = self.read_memory_byte(u16::from_le_bytes(self.hl));
                    self.pc += 1;
                }
                0x4F => {
                    // load a to c
                    self.a = self.bc[1];
                    self.pc += 1;
                }
                0x50 => {
                    // load b to d
                    self.bc[0] = self.de[0];
                    self.pc += 1;
                }
                0x51 => {
                    // load c to d
                    self.bc[1] = self.de[0];
                    self.pc += 1;
                }
                0x52 => {
                    // load d to d, no-op
                    self.de[0] = self.de[0];
                    self.pc += 1;
                }
                0x53 => {
                    // load e to d
                    self.de[1] = self.de[0];
                    self.pc += 1;
                }
                0x56 => {
                    // load memory at position hl to d
                    self.de[0] = self.read_memory_byte(u16::from_le_bytes(self.hl));
                    self.pc += 1;
                }
                0x58 => {
                    // load b to e
                    self.bc[0] = self.de[1];
                    self.pc += 1;
                }
                0x5A => {
                    // load d to e
                    self.de[0] = self.de[1];
                    self.pc += 1;
                }
                0x5B => {
                    // load e to e, no-op
                    // self.de[1] = self.de[1];
                    self.pc += 1;
                }
                0x5C => {
                    // load h to e
                    self.de[1] = self.hl[0];
                    self.pc += 1;
                }
                0x60 => {
                    // load b to h
                    self.bc[0] = self.hl[0];
                    self.pc += 1;
                }
                0x6B => {
                    // load h to l
                    self.de[1] = self.hl[1];
                    self.pc += 1;
                }
                0x6E => {
                    // load memory at position hl to l
                    self.hl[1] = self.read_memory_byte(u16::from_le_bytes(self.hl));
                    self.pc += 1;
                }
                0x6F => {
                    // load a to l
                    self.hl[1] = self.a;
                    self.pc += 1;
                }
                0x77 => {
                    self.write_memory_byte(u16::from_le_bytes(self.hl), self.a);
                    self.pc += 1;
                }
                0x7B => {
                    // load e to a
                    self.a = self.de[1];
                    self.pc += 1;
                }
                0x93 => {
                    // sub e from a
                    self.subtract(self.de[1]);
                    self.pc += 1;
                }
                0x94 => {
                    // sub h from a
                    self.subtract(self.hl[0]);
                    self.pc += 1;
                }
                0x95 => {
                    // sub l from a
                    self.subtract(self.hl[1]);
                    self.pc += 1;
                }
                0x99 => {
                    // subtract c + 1 from a and store in a
                    let reg_c = self.bc[1];
                    self.flags.n = true;

                    let mut overflow = false;
                    (self.a, overflow) = self.a.overflowing_sub(reg_c);
                    (self.a, overflow) = self.a.overflowing_sub(1);
                    self.flags.c = overflow;

                    self.flags.z = self.a == 0;
                    self.pc += 1;
                }
                0xAF => {
                    // XOR the A register with itself
                    self.a ^= self.a;
                    self.pc += 1;
                }
                0xB0 => {
                    // XOR B with A and store in A
                    self.a ^= self.bc[0];
                    self.pc += 1;
                }
                0xBF => {
                    // compare A with A, set flags
                    self.compare_a(self.a);
                    self.pc += 1;
                }
                0xC3 => {
                    // Jump to the address immediately after the current instruction
                    let data: [u8; 2] = self.next_word();
                    let pos: u16 = u16::from_be_bytes(data); // be because already switched
                    self.pc = pos;
                }
                0xCD => {
                    let address = self.next_word();
                    self.push_word(self.pc);
                    self.pc = u16::from_le_bytes(address);
                }
                0xFF => {
                    self.push_word(self.pc);
                    self.pc = self.cartridge.data[0x38] as u16;
                }
                _ => {
                    self.debug_registers();
                    panic!("Unknown instruction {:#04X}", instruction)
                }
            }

            sleep(time::Duration::from_millis(5));
        }
    }

    /** Increments program counter */
    fn next_byte(&mut self) -> u8 {
        self.pc += 1;
        self.cartridge.data[self.pc as usize]
    }

    /** Increments program counter twice */
    fn next_word(&mut self) -> [u8; 2] {
        let a = self.next_byte();
        let b = self.next_byte();

        [b, a]
    }

    fn push_word(&mut self, word: u16) {
        self.sp -= 2;

        self.memory.set_position(self.sp as u64);
        self.memory.write_u16::<LittleEndian>(word).unwrap();
    }

    fn read_memory_byte(&mut self, pos: u16) -> u8 {
        self.memory.set_position(pos as u64);
        self.memory.read_u8().unwrap()
    }

    fn write_memory_byte(&mut self, pos: u16, byte: u8) {
        self.memory.set_position(pos as u64);
        self.memory.write_u8(byte).unwrap()
    }

    /** add 1 to byte, does not set carry flag */
    fn increment(&mut self, byte: u8) -> u8 {
        self.flags.h = (byte & 0xF) == 0xF;
        let (byte, _) = byte.overflowing_add(1);
        self.flags.z = byte == 0;
        self.flags.n = false;

        byte
    }

    /** subtract 1 from byte, does not set carry flag */
    fn decrement(&mut self, byte: u8) -> u8 {
        let (byte, _) = byte.overflowing_sub(1);
        self.flags.z = byte == 0;
        self.flags.n = true;
        self.flags.h = (byte & 0xF) == 0xF;

        byte
    }

    /**
     * Add register B to HL
     */
    fn add_to_reg_hl(&mut self, b: [u8; 2]) {
        let hl_u16 = u16::from_le_bytes(self.hl);
        let b_u16 = u16::from_le_bytes(b);
        let temp = hl_u16 as u32 + b_u16 as u32;
        self.flags.n = false;
        self.flags.c = temp > 0xFFFF;
        self.flags.h = ((hl_u16 & 0xFFF) + (b_u16 & 0xFFF)) > 0xFFF;
        self.hl = ((temp ^ 0xFFFF) as u16).to_le_bytes();
    }

    /** subtract byte from register A */
    fn subtract(&mut self, byte: u8) {
        self.compare_a(byte);
        (self.a, _) = self.a.overflowing_sub(byte);
    }

    fn compare_a(&mut self, byte: u8) {
        self.flags.z = self.a == byte;
        self.flags.n = true;
        self.flags.h = (self.a & 0xF) < (byte & 0xF);
        self.flags.c = self.a < byte;
    }

    /**
     * Shift A right, placing the rightmost bit in the carry flag. Set the leftmost
     * bit of A to the old value of the carry flag.
     */
    fn rra(&mut self) {
        let mut byte = self.a;
        let carry = self.flags.c;
        self.flags.c = (byte & 0x1) == 0x1;
        byte >>= 1;
        if carry {
            byte |= 0x80;
        }
        self.a = byte;
    }

    /**
     * Shift A left, placing the leftmost bit (bit 7) in the carry flag and bit 0 of A.
     */
    fn rlca(&mut self) {
        let mut byte = self.a;
        self.flags.c = (byte & 0xF) == 0x1;
        byte <<= 1;
        if self.flags.c {
            byte |= 0x01;
        }
        self.a = byte;
    }

    fn debug_registers(&self) {
        println!("Program counter: {:x}", self.pc);
        println!("Stack pointer: {:x}", self.sp);
        println!("A: {:x}", self.a);
        println!("BC: {:x} {:x}", self.bc[0], self.bc[1]);
        println!("DE: {:x} {:x}", self.de[0], self.de[1]);
        println!("HL: {:x} {:x}", self.hl[0], self.hl[1]);
        println!("Flags: {:?}", self.flags);
    }
}
