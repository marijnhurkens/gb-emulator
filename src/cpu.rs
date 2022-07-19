use std::io::Cursor;
use std::ops::Sub;
use std::thread::sleep;
use std::time::{Duration, Instant};

use byteorder::LittleEndian;

use crate::cartridge::Cartridge;
use crate::helpers::signed_add;
use crate::memory::Memory;

const CPU_FREQ: f64 = 4_194_304.0;

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
    memory: Memory,
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
            memory: Memory::new(),
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

            let start_time = Instant::now();
            let t_cycles = self.process_instruction(instruction) * 4;

            for _ in 0..t_cycles {
                self.memory.video.step();
            }

            let end_time = Instant::now();

            let expected_time = Duration::from_secs_f64(t_cycles as f64 / CPU_FREQ);
            let time_elapsed = end_time.duration_since(start_time);

            if time_elapsed.lt(&expected_time) {
                let sleep_for = expected_time.sub(time_elapsed);
                sleep(sleep_for);
            }
        }
    }

    /**
     * Runs the next instruction and returns the cycle cost divided by 4 (M-cycles)
     */
    fn process_instruction(&mut self, instruction: u8) -> usize {
        print!("{:#08X} | {:#04X} | ", self.pc, instruction);

        let m_cycles = match instruction {
            0x00 => {
                self.pc += 1;
                0
            }
            0x01 => {
                // load next 2 bytes in bc
                let data: [u8; 2] = self.next_word();
                self.bc = data;
                self.pc += 1;
                3
            }
            0x02 => {
                // write a to memory on location bc
                self.memory.write_byte(u16::from_le_bytes(self.bc), self.a);
                self.pc += 1;
                2
            }
            0x03 => {
                // increment BE by 1
                self.bc = (u16::from_le_bytes(self.bc) + 1).to_le_bytes();
                self.pc += 1;
                2
            }
            0x04 => {
                // increment B by 1
                self.bc[0] = self.increment(self.bc[0]);
                self.pc += 1;
                1
            }
            0x05 => {
                // decrement B by 1
                self.bc[0] = self.decrement(self.bc[0]);
                self.pc += 1;
                1
            }
            0x06 => {
                // load next byte to b
                let data = self.next_byte();
                self.bc[0] = data;
                self.pc += 1;
                2
            }
            0x07 => {
                self.rlca();
                self.pc += 1;
                1
            }
            0x08 => {
                // write stack pointer to memory at next word location
                let pos = u16::from_le_bytes(self.next_word());
                self.memory.write_word(pos, self.sp);
                self.pc += 1;
                5
            }
            0x0A => {
                // load memory at address BC into A
                self.a = self.memory.read_byte(u16::from_le_bytes(self.bc));
                self.pc += 1;
                2
            }
            0x0B => {
                // decrement BC by 1
                let data = u16::from_le_bytes(self.bc);
                self.bc = (data - 1).to_le_bytes();
                self.pc += 1;
                2
            }
            0x0C => {
                // increment C by 1
                self.bc[1] = self.increment(self.bc[1]);
                self.pc += 1;
                1
            }
            0x0D => {
                // decrement C by 1
                self.bc[1] = self.decrement(self.bc[1]);
                self.pc += 1;
                1
            }
            0x0E => {
                // load next byte to c
                let data = self.next_byte();
                self.bc[1] = data;
                self.pc += 1;
                2
            }
            0x11 => {
                // load next word into DE
                self.de = self.next_word();
                self.pc += 1;
                3
            }
            0x12 => {
                // store A to memeory location of DE
                let address = u16::from_le_bytes(self.de);
                self.memory.write_byte(address, self.a);
                self.pc += 1;

                2
            }
            0x14 => {
                // increment D by 1
                self.de[0] = self.increment(self.de[0]);
                self.pc += 1;
                1
            }
            0x15 => {
                // decrement D by 1
                self.de[0] = self.decrement(self.de[0]);
                self.pc += 1;
                1
            }
            0x16 => {
                // load next byte to d
                let data = self.next_byte();
                self.de[0] = data;
                self.pc += 1;
                2
            }
            0x18 => {
                // relative jump
                let pos = self.next_byte();
                self.pc += pos as u16;
                3
            }
            0x19 => {
                // add DE to HL
                self.add_to_reg_hl(self.de);
                self.pc += 1;
                2
            }
            0x1D => {
                // decrement E by 1
                self.de[1] = self.decrement(self.de[1]);
                self.pc += 1;
                1
            }
            // todo: set clockcycles from here on
            0x1E => {
                self.de[1] = self.next_byte();
                self.pc += 1;
                1
            }
            0x1F => {
                self.rra();
                self.pc += 1;
                1
            }
            0x20 => {
                // if z is 0, jump next byte relative steps, otherwise skip next byte
                if !self.flags.z {
                    //let current_addr = self.pc; // store because next_byte increases pc
                    //let new_addr = current_addr + self.next_byte() as u16 + 2;
                    let data = self.next_byte();
                    self.relative_jump(data);
                    self.pc += 1;
                    print!("JR NZ, s8 | Jump to {:X}", self.pc);
                } else {
                    print!("JR NZ, s8 | Skip");
                    self.pc += 2; // skip data
                }
                1
            }
            0x21 => {
                // load the next word (16 bits) into the HL register
                let data: [u8; 2] = self.next_word();
                self.hl = data;
                self.pc += 1;
                1
            }
            0x22 => {
                // write value of A to memory location HL and increment HL
                self.memory.write_byte(u16::from_le_bytes(self.hl), self.a);
                self.hl = (u16::from_le_bytes(self.hl) + 1).to_le_bytes();
                self.pc += 1;
                1
            }
            0x23 => {
                // increment HL by 1
                self.hl = (u16::from_le_bytes(self.hl) + 1).to_le_bytes();
                self.pc += 1;
                1
            }
            0x24 => {
                // increment H by 1
                self.hl[0] = self.increment(self.hl[0]);
                self.pc += 1;
                1
            }
            0x25 => {
                // decrement H by 1
                self.hl[0] = self.decrement(self.hl[0]);
                self.pc += 1;
                1
            }
            0x26 => {
                // Load next byte to H
                let data = self.next_byte();
                self.hl[0] = data;
                self.pc += 1;
                2
            }
            0x27 => {
                // // note: assumes a is a uint8_t and wraps from 0xff to 0
                // if (!n_flag) {  // after an addition, adjust if (half-)carry occurred or if result is out of bounds
                //   if (c_flag || a > 0x99) { a += 0x60; c_flag = 1; }
                //   if (h_flag || (a & 0x0f) > 0x09) { a += 0x6; }
                // } else {  // after a subtraction, only adjust if (half-)carry occurred
                //   if (c_flag) { a -= 0x60; }
                //   if (h_flag) { a -= 0x6; }
                // }
                // // these flags are always updated
                // z_flag = (a == 0); // the usual z flag
                // h_flag = 0; // h flag is always cleared

                if !self.flags.n {
                    if self.flags.c || self.a > 0x99 {
                        self.a += 0x60;
                        self.flags.c = true;
                    }
                    if self.flags.h || (self.a & 0x0f) > 0x9 {
                        self.a += 0x6;
                    }
                } else {
                    if self.flags.c {
                        self.a -= 0x60;
                    }
                    if self.flags.h {
                        self.a -= 0x6;
                    }
                }

                self.flags.z = self.a == 0;
                self.flags.h = false;
                self.pc += 1;

                1
            }
            0x28 => {
                // if z is 1, jump next byte relative steps, otherwise skip next byte
                if self.flags.z {
                    // let current_addr = self.pc; // store because next_byte increases pc
                    // let new_addr = current_addr + self.next_byte() as u16 + 2;
                    let data = self.next_byte();
                    self.relative_jump(data);
                    self.pc += 1;
                    print!("JR Z, s8 | Jump to {:X}", self.pc);
                } else {
                    print!("JR Z, s8 | Skip");
                    self.pc += 2; // skip data
                }
                1
            }
            0x29 => {
                // Add HL to HL
                self.add_to_reg_hl(self.hl);
                self.pc += 1;
                1
            }
            0x2A => {
                // Load contents of memory at HL to A, and incrment HL
                let address = u16::from_le_bytes(self.hl);
                self.a = self.memory.read_byte(address);
                self.hl = (address + 1).to_le_bytes();
                self.pc += 1;
                2
            }
            0x2C => {
                // increment L by 1
                self.hl[1] = self.increment(self.hl[1]);
                self.pc += 1;
                1
            }
            0x2D => {
                // decrement L by 1
                self.hl[1] = self.decrement(self.hl[1]);
                self.pc += 1;
                1
            }
            0x2E => {
                // set L to next byte
                self.hl[1] = self.next_byte();
                self.pc += 1;
                1
            }
            0x2F => {
                // complement / invert A
                self.a = !self.a;
                self.pc += 1;
                1
            }
            0x31 => {
                // load next byte into stack pointer
                self.sp = u16::from_le_bytes(self.next_word());
                self.pc += 1;
                1
            }
            0x32 => {
                // store a in memory at location hl, and decrement hl
                let hl_u16 = u16::from_le_bytes(self.hl);
                self.memory.write_byte(hl_u16, self.a);
                self.hl = (hl_u16 - 1).to_le_bytes();
                self.pc += 1;
                1
            }
            0x35 => {
                // decrement memory at location HL by 1
                let hl_u16 = u16::from_le_bytes(self.hl);
                let mut data = self.memory.read_byte(hl_u16);
                data = self.decrement(data);
                self.memory.write_byte(hl_u16, data);
                self.pc += 1;
                3
            }
            0x36 => {
                // load next byte into memory at HL
                let hl_u16 = u16::from_le_bytes(self.hl);
                let data = self.next_byte();
                self.memory.write_byte(hl_u16, data);
                self.pc += 1;
                3
            }
            0x3E => {
                // Load next byte to A
                self.a = self.next_byte();
                self.pc += 1;
                1
            }
            0x41 => {
                self.bc[0] = self.bc[1];
                self.pc += 1;
                1
            }
            0x4A => {
                // load d to c
                self.de[0] = self.bc[1];
                self.pc += 1;
                1
            }
            0x4B => {
                // load c to e
                self.bc[1] = self.de[1];
                self.pc += 1;
                1
            }
            0x4C => {
                // load h to c
                self.hl[0] = self.bc[1];
                self.pc += 1;
                1
            }
            0x4D => {
                // load l to c
                self.hl[1] = self.bc[1];
                self.pc += 1;
                1
            }
            0x4E => {
                // load memory at position hl to c
                self.bc[1] = self.memory.read_byte(u16::from_le_bytes(self.hl));
                self.pc += 1;
                1
            }
            0x4F => {
                // load a to c
                self.a = self.bc[1];
                self.pc += 1;
                1
            }
            0x50 => {
                // load b to d
                self.bc[0] = self.de[0];
                self.pc += 1;
                1
            }
            0x51 => {
                // load c to d
                self.bc[1] = self.de[0];
                self.pc += 1;
                1
            }
            0x52 => {
                // load d to d, no-op
                //self.de[0] = self.de[0];
                self.pc += 1;
                1
            }
            0x53 => {
                // load e to d
                self.de[1] = self.de[0];
                self.pc += 1;
                1
            }
            0x56 => {
                // load memory at position hl to d
                self.de[0] = self.memory.read_byte(u16::from_le_bytes(self.hl));
                self.pc += 1;
                1
            }
            0x58 => {
                // load b to e
                self.bc[0] = self.de[1];
                self.pc += 1;
                1
            }
            0x5A => {
                // load d to e
                self.de[0] = self.de[1];
                self.pc += 1;
                1
            }
            0x5B => {
                // load e to e, no-op
                // self.de[1] = self.de[1];
                self.pc += 1;
                1
            }
            0x5C => {
                // load h to e
                self.de[1] = self.hl[0];
                self.pc += 1;
                1
            }
            0x60 => {
                // load b to h
                self.bc[0] = self.hl[0];
                self.pc += 1;
                1
            }
            0x67 => {
                // load a to h
                self.hl[0] = self.a;
                self.pc += 1;
                1
            }
            0x6B => {
                // load h to l
                self.de[1] = self.hl[1];
                self.pc += 1;
                1
            }
            0x6E => {
                // load memory at position hl to l
                self.hl[1] = self.memory.read_byte(u16::from_le_bytes(self.hl));
                self.pc += 1;
                1
            }
            0x6F => {
                // load a to l
                self.hl[1] = self.a;
                self.pc += 1;
                1
            }
            0x77 => {
                self.memory.write_byte(u16::from_le_bytes(self.hl), self.a);
                self.pc += 1;
                1
            }
            0x78 => {
                self.a = self.bc[0];
                self.pc += 1;
                1
            }
            0x7B => {
                // load e to a
                self.a = self.de[1];
                self.pc += 1;
                1
            }
            0x7F => {
                // load a to a, no-op
                self.pc += 1;
                1
            }
            0x8A => {
                self.add_to_a_with_carry(self.de[0]);
                self.pc += 1;
                1
            }
            0x93 => {
                // sub e from a
                self.subtract(self.de[1]);
                self.pc += 1;
                1
            }
            0x94 => {
                // sub h from a
                self.subtract(self.hl[0]);
                self.pc += 1;
                1
            }
            0x95 => {
                // sub l from a
                self.subtract(self.hl[1]);
                self.pc += 1;
                1
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
                1
            }
            0xAA => {
                // XOR A with D and store in A
                self.a ^= self.de[0];
                self.pc += 1;
                1
            }
            0xAF => {
                // XOR the A register with itself
                self.a ^= self.a;
                self.pc += 1;
                1
            }
            0xB0 => {
                // XOR B with A and store in A
                self.a ^= self.bc[0];
                self.pc += 1;
                1
            }
            0xB1 => {
                // xor C with C and store in A
                self.a ^= self.bc[1];
                self.pc += 1;
                1
            }
            0xBF => {
                // compare A with A, set flags
                self.compare_a(self.a);
                self.pc += 1;
                1
            }
            0xC3 => {
                // Jump to the address immediately after the current instruction
                let data: [u8; 2] = self.next_word();
                let pos: u16 = u16::from_be_bytes(data); // be because already switched
                self.pc = pos;
                1
            }
            0xC9 => {
                // RET - pop word from stack and set PC to that
                let address = self.pop_word();
                print!("RET | {:#08X}", address);
                self.pc = address;
                1
            }
            0xCB => {
                println!();
                unimplemented!("16 bit instruction prefix");
            }
            0xCD => {
                // CALL a16 = push PC to stack and jump to next byte
                let address = u16::from_be_bytes(self.next_word()); // BigEndian because we already switched
                print!("CALL a16 | {:#08X}", address);
                self.push_word(self.pc);
                self.pc = address;
                1
            }
            0xD2 => {
                // Jump to address in next word if carry flag is true, or continue
                if self.flags.c {
                    self.pc = u16::from_le_bytes(self.next_word());
                } else {
                    self.pc += 1;
                }
                1
            }
            0xD6 => {
                // subtract net byte from A and saver to A
                let data = self.next_byte();
                self.subtract(data);
                self.pc += 1;
                1
            }
            0xE0 => {
                // Store A to address at memory 0xFFxx, where xx is the next byte
                let address = self.next_byte();
                self.memory
                    .write_byte(u16::from_le_bytes([address, 0xff]), self.a);
                self.pc += 1;
                1
            }
            0xE2 => {
                // Store A to address at memory 0xFFxx, where xx is register C
                let address = self.bc[1];
                self.memory
                    .write_byte(u16::from_le_bytes([address, 0xff]), self.a);
                self.pc += 1;
                2
            }
            0xE6 => {
                let data = self.next_byte();
                self.a &= data;
                self.pc += 1;
                2
            }
            0xEA => {
                // Store A to address at memory 0xFFxx, where xx is the next byte
                let address = u16::from_le_bytes(self.next_word());
                self.memory.write_byte(address, self.a);
                self.pc += 4;
                1
            }
            0xF0 => {
                // Load address at memory 0xFFxx into A, where xx is the next byte
                let data = self.next_byte();
                self.a = self.memory.read_byte(u16::from_le_bytes([data, 0xff]));
                self.pc += 1;
                1
            }
            0xF3 => {
                self.interrupts_enabled = false;
                self.pc += 1;
                1
            }
            0xF9 => {
                // Load HL to SP
                self.sp = u16::from_le_bytes(self.hl);
                self.pc += 1;
                1
            }
            0xFB => {
                self.interrupts_enabled = true;
                self.pc += 1;
                1
            }
            0xFE => {
                // Set Z flag if A and next byte are equal by calculating A - next byte == 0
                let data = self.next_byte();
                self.compare_a(data);
                self.pc += 1;
                1
            }
            0xFF => {
                self.push_word(self.pc);
                self.pc = self.cartridge.data[0x38] as u16;
                1
            }
            _ => {
                println!();
                self.debug_registers();
                panic!("Unknown instruction {:#04X}", instruction)
            }
        };

        println!();

        m_cycles
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

        self.memory.write_word(self.sp, word);
    }

    fn pop_word(&mut self) -> u16 {
        let data = self.memory.read_word(self.sp);

        self.sp += 2;

        data
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

    fn add_to_a_with_carry(&mut self, byte: u8) {
        let mut n = self.a as u16 + byte as u16;
        if self.flags.c {
            n += 1;
        }

        self.flags.z = (n & 0xff) == 0;
        self.flags.h = ((self.a & 0x0F) + (byte & 0x0F)) & 0x10 == 0x10;
        self.flags.c = n > 0xff;
        self.flags.n = false;
        self.a = (n & 0xff) as u8;
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

    fn relative_jump(&mut self, data: u8) {
        self.pc = signed_add(self.pc, data);
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
