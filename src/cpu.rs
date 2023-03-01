use std::io;
use std::ops::Sub;
use std::thread::sleep;
use std::time::{Duration, Instant};

use crate::cartridge::Cartridge;
use crate::helpers::signed_add;
use crate::instructions::{
    decode, Condition, ImmediateOperand, Instruction, Load, MemoryLocation, Operand, Register,
    RegisterPair,
};
use crate::memory::{InterruptFlags, Memory, VideoMode};
use crate::ScreenBuffer;

pub const CPU_FREQ: f64 = 4_194_304.0;
const STACK_START: u16 = 0xfffe;

#[derive(Debug)]
pub struct Cpu {
    pc: u16,
    sp: u16,
    a: u8, // accumulator
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
    flags: CpuFlags,
    interrupts_enabled: bool,
    memory: Memory,
    cartridge: Cartridge,
    state: CpuState,
    break_point: Option<u16>,
}

#[derive(Debug)]
pub struct CpuFlags {
    // zero
    z: bool,
    // CY or carry
    c: bool,
    // previous instruction was a subtraction
    n: bool,
    // todo
    h: bool,
}

impl CpuFlags {
    pub fn to_bits(&self) -> u8 {
        self.c as u8 | (self.h as u8) << 1 | (self.n as u8) << 2 | (self.z as u8) << 3
    }
}

#[derive(Eq, PartialEq, Debug)]
enum CpuState {
    Running,
    Stopped,
}

impl Cpu {
    pub fn load_cartridge(cartridge: Cartridge) -> Self {
        Cpu {
            pc: 0x0100,
            sp: STACK_START,
            a: 0x01,
            b: 0xff,
            c: 0x13,
            d: 0x00,
            e: 0xc1,
            h: 0x74,
            l: 0x03,
            flags: CpuFlags {
                z: false,
                c: false,
                n: false,
                h: false,
            },
            interrupts_enabled: false,
            memory: Memory::new(cartridge.data.clone()),
            cartridge,
            state: CpuState::Stopped,
            break_point: None,
        }
    }

    pub fn run(&mut self, screen_buffer: Option<ScreenBuffer>, break_point: Option<u16>) {
        let title = &self.cartridge.header.title;
        println!("Running {:}", title);

        self.break_point = break_point;
        self.state = CpuState::Running;

        loop {
            // Draw to screen
            if let Some(buffer) = &screen_buffer {
                let mut guard = buffer.lock().unwrap();
                (*guard) = self.memory.read_vram();
                drop(guard);
            }

            // Catch PC out of bounds
            if self.pc as usize >= self.cartridge.data.len() {
                self.debug_registers();
                self.debug_stack();
                panic!("PC out of bounds");
            }

            let (instruction, length) = decode(&self.cartridge.data, self.pc);
            print!("{:#08X} | {} | ", self.pc, instruction);
            self.pc = self.pc.wrapping_add(length);

            let start_time = Instant::now();
            let t_cycles = self.process_instruction(instruction) * 4;

            for _ in 0..t_cycles {
                self.memory.step();
                let new_mode = self.memory.video.step();

                // we've entered vblank, set Interrupt Flag
                if let Some(VideoMode::VBlank) = new_mode {
                    let interrupt_flag = self.memory.read_byte(0xFF0F);
                    self.memory.write_byte(0xFF0F, interrupt_flag | 0b0000_0001);

                    if self.interrupts_enabled
                        && self
                            .memory
                            .interrupt_enable
                            .intersects(InterruptFlags::VBLANK)
                    {
                        self.interrupts_enabled = false;
                        let interrupt_flag = self.memory.read_byte(0xFF0F);
                        self.memory.write_byte(0xFF0F, interrupt_flag & 0b1111_1110);

                        self.push_word(self.pc + 1);
                        let address = 0x0040;
                        println!("INT 40 VBLANK | {:#08X}", address);
                        self.pc = address;
                    }
                }
            }

            // handle timing
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
    fn process_instruction(&mut self, instruction: Instruction) -> usize {
        if let Some(break_point) = self.break_point {
            if break_point == self.pc {
                self.state = CpuState::Stopped;
            }
        }

        let m_cycles = match instruction {
            Instruction::LD(load) => self.ld(load),
            Instruction::NOP => 1,
            Instruction::HALT => panic!("HALT"),
            Instruction::DEC(operand) => self.decrement(operand),
            Instruction::JP(operand) => self.jump(operand),
            Instruction::XOR(source) => self.xor_a(source),
            Instruction::JR(operand, condition) => self.jump_relative(operand, condition),
            Instruction::DI => {
                self.interrupts_enabled = false;
                1
            }
            Instruction::CP(operand) => self.compare_a(operand),
            // 0x00 => {
            //     self.pc += 1;
            //     0
            // }
            // 0x01 => {
            //     // load next 2 bytes in bc
            //     let data: [u8; 2] = self.next_word();
            //     self.bc = data;
            //     self.pc += 1;
            //     3
            // }
            // 0x02 => {
            //     // write a to memory on location bc
            //     self.memory.write_byte(u16::from_le_bytes(self.bc), self.a);
            //     self.pc += 1;
            //     2
            // }
            // 0x03 => {
            //     // increment BE by 1
            //     self.bc = (u16::from_le_bytes(self.bc) + 1).to_le_bytes();
            //     self.pc += 1;
            //     2
            // }
            // 0x04 => {
            //     // increment B by 1
            //     self.bc[0] = self.increment(self.bc[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x05 => {
            //     // decrement B by 1
            //     self.bc[0] = self.decrement(self.bc[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x06 => {
            //     // load next byte to b
            //     let data = self.next_byte();
            //     self.bc[0] = data;
            //     self.pc += 1;
            //     2
            // }
            // 0x07 => {
            //     self.rlca();
            //     self.pc += 1;
            //     1
            // }
            // 0x08 => {
            //     // write stack pointer to memory at next word location
            //     let pos = u16::from_le_bytes(self.next_word());
            //     self.memory.write_word(pos, self.sp);
            //     self.pc += 1;
            //     5
            // }
            // 0x0A => {
            //     // load memory at address BC into A
            //     self.a = self.memory.read_byte(u16::from_le_bytes(self.bc));
            //     self.pc += 1;
            //     2
            // }
            // 0x0B => {
            //     // decrement BC by 1
            //     let data = u16::from_le_bytes(self.bc);
            //     self.bc = (data - 1).to_le_bytes();
            //     self.pc += 1;
            //     2
            // }
            // 0x0C => {
            //     // increment C by 1
            //     self.bc[1] = self.increment(self.bc[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x0D => {
            //     // decrement C by 1
            //     self.bc[1] = self.decrement(self.bc[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x0E => {
            //     // load next byte to c
            //     let data = self.next_byte();
            //     self.bc[1] = data;
            //     self.pc += 1;
            //     2
            // }
            // 0x11 => {
            //     // load next word into DE
            //     self.de = self.next_word();
            //     self.pc += 1;
            //     3
            // }
            // 0x12 => {
            //     // store A to mememory location of DE
            //     let address = u16::from_le_bytes(self.de);
            //     self.memory.write_byte(address, self.a);
            //     self.pc += 1;
            //
            //     2
            // }
            // 0x13 => {
            //     // increment DE by 1
            //     self.de = (u16::from_le_bytes(self.de) + 1).to_le_bytes();
            //     self.pc += 1;
            //     2
            // }
            // 0x14 => {
            //     // increment D by 1
            //     self.de[0] = self.increment(self.de[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x15 => {
            //     // decrement D by 1
            //     self.de[0] = self.decrement(self.de[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x16 => {
            //     // load next byte to d
            //     let data = self.next_byte();
            //     print!("ld d, ${:#02X} | ", data);
            //     self.de[0] = data;
            //     self.pc += 1;
            //     2
            // }
            // 0x18 => {
            //     // relative jump
            //     let pos = self.next_byte();
            //     self.pc += pos as u16;
            //     3
            // }
            // 0x19 => {
            //     // add DE to HL
            //     print!("add hl, de | ");
            //     self.add_to_reg_hl(self.de);
            //     self.pc += 1;
            //     2
            // }
            // 0x1C => {
            //     // increment E by 1
            //     self.de[1] = self.increment(self.de[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x1D => {
            //     // decrement E by 1
            //     self.de[1] = self.decrement(self.de[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x1A => {
            //     self.a = self.memory.read_byte(u16::from_le_bytes(self.de));
            //     self.pc += 1;
            //     2
            // }
            // // todo: set clock cycles from here on
            // 0x1E => {
            //     self.de[1] = self.next_byte();
            //     self.pc += 1;
            //     1
            // }
            // 0x1F => {
            //     self.rra();
            //     self.pc += 1;
            //     1
            // }
            // 0x20 => {
            //     // if z is 0, jump next byte relative steps, otherwise skip next byte
            //     // self.debug_registers();
            //     if !self.flags.z {
            //         //let current_addr = self.pc; // store because next_byte increases pc
            //         //let new_addr = current_addr + self.next_byte() as u16 + 2;
            //         let data = self.next_byte();
            //         self.relative_jump(data);
            //         self.pc += 1;
            //         print!("JR NZ, s8 | Jump to {:#04X}", self.pc);
            //     } else {
            //         print!("JR NZ, s8 | Skip");
            //         self.pc += 2; // skip data
            //     }
            //     1
            // }
            // 0x21 => {
            //     // load the next word (16 bits) into the HL register
            //     let data: [u8; 2] = self.next_word();
            //     print!("LD HL, d16 | {:#04X}", u16::from_le_bytes(data));
            //     self.hl = data;
            //     self.pc += 1;
            //     1
            // }
            // 0x22 => {
            //     // write value of A to memory location HL and increment HL
            //     self.memory.write_byte(u16::from_le_bytes(self.hl), self.a);
            //     self.hl = (u16::from_le_bytes(self.hl) + 1).to_le_bytes();
            //     self.pc += 1;
            //     1
            // }
            // 0x23 => {
            //     // increment HL by 1
            //     self.hl = (u16::from_le_bytes(self.hl) + 1).to_le_bytes();
            //     self.pc += 1;
            //     1
            // }
            // 0x24 => {
            //     // increment H by 1
            //     self.hl[0] = self.increment(self.hl[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x25 => {
            //     // decrement H by 1
            //     self.hl[0] = self.decrement(self.hl[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x26 => {
            //     // Load next byte to H
            //     let data = self.next_byte();
            //     self.hl[0] = data;
            //     self.pc += 1;
            //     2
            // }
            // 0x27 => {
            //     // // note: assumes a is a uint8_t and wraps from 0xff to 0
            //     // if (!n_flag) {  // after an addition, adjust if (half-)carry occurred or if result is out of bounds
            //     //   if (c_flag || a > 0x99) { a += 0x60; c_flag = 1; }
            //     //   if (h_flag || (a & 0x0f) > 0x09) { a += 0x6; }
            //     // } else {  // after a subtraction, only adjust if (half-)carry occurred
            //     //   if (c_flag) { a -= 0x60; }
            //     //   if (h_flag) { a -= 0x6; }
            //     // }
            //     // // these flags are always updated
            //     // z_flag = (a == 0); // the usual z flag
            //     // h_flag = 0; // h flag is always cleared
            //
            //     if !self.flags.n {
            //         if self.flags.c || self.a > 0x99 {
            //             self.a += 0x60;
            //             self.flags.c = true;
            //         }
            //         if self.flags.h || (self.a & 0x0f) > 0x9 {
            //             self.a += 0x6;
            //         }
            //     } else {
            //         if self.flags.c {
            //             self.a = self.a.overflowing_sub(0x60).0;
            //         }
            //         if self.flags.h {
            //             self.a -= 0x6;
            //         }
            //     }
            //
            //     self.flags.z = self.a == 0;
            //     self.flags.h = false;
            //     self.pc += 1;
            //
            //     1
            // }
            // 0x28 => {
            //     // if z is 1, jump next byte relative steps, otherwise skip next byte
            //     if self.flags.z {
            //         // let current_addr = self.pc; // store because next_byte increases pc
            //         // let new_addr = current_addr + self.next_byte() as u16 + 2;
            //         let data = self.next_byte();
            //         self.relative_jump(data);
            //         self.pc += 1;
            //         print!("JR Z, s8 | Jump to {:X}", self.pc);
            //     } else {
            //         print!("JR Z, s8 | Skip");
            //         self.pc += 2; // skip data
            //     }
            //     1
            // }
            // 0x29 => {
            //     // Add HL to HL
            //     self.add_to_reg_hl(self.hl);
            //     self.pc += 1;
            //     1
            // }
            // 0x2A => {
            //     // Load contents of memory at HL to A, and incrment HL
            //     let address = u16::from_le_bytes(self.hl);
            //     self.a = self.memory.read_byte(address);
            //     self.hl = (address + 1).to_le_bytes();
            //     self.pc += 1;
            //     2
            // }
            // 0x2C => {
            //     // increment L by 1
            //     self.hl[1] = self.increment(self.hl[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x2D => {
            //     // decrement L by 1
            //     self.hl[1] = self.decrement(self.hl[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x2E => {
            //     // set L to next byte
            //     self.hl[1] = self.next_byte();
            //     self.pc += 1;
            //     1
            // }
            // 0x2F => {
            //     // complement / invert A
            //     self.a = !self.a;
            //     self.pc += 1;
            //     1
            // }
            // 0x31 => {
            //     // load next byte into stack pointer
            //     self.sp = u16::from_le_bytes(self.next_word());
            //     self.pc += 1;
            //     1
            // }
            // 0x32 => {
            //     // store a in memory at location hl, and decrement hl
            //     let hl_u16 = u16::from_le_bytes(self.hl);
            //     self.memory.write_byte(hl_u16, self.a);
            //     self.hl = (hl_u16 - 1).to_le_bytes();
            //     self.pc += 1;
            //     1
            // }
            // 0x35 => {
            //     // decrement memory at location HL by 1
            //     let hl_u16 = u16::from_le_bytes(self.hl);
            //     let mut data = self.memory.read_byte(hl_u16);
            //     data = self.decrement(data);
            //     self.memory.write_byte(hl_u16, data);
            //     self.pc += 1;
            //     3
            // }
            // 0x36 => {
            //     // load next byte into memory at HL
            //     let hl_u16 = u16::from_le_bytes(self.hl);
            //     let data = self.next_byte();
            //     self.memory.write_byte(hl_u16, data);
            //     self.pc += 1;
            //     3
            // }
            // 0x37 => {
            //     self.flags.c = true;
            //     self.pc += 1;
            //     1
            // }
            // 0x3B => {
            //     // decrement SP by 1
            //     self.sp -= 1;
            //     self.pc += 1;
            //     2
            // }
            // 0x3C => {
            //     // increment A by 1
            //     self.a = self.increment(self.a);
            //     self.pc += 1;
            //     1
            // }
            // 0x3D => {
            //     // decrement L by 1
            //     self.a = self.decrement(self.a);
            //     self.pc += 1;
            //     1
            // }
            // 0x3E => {
            //     // Load next byte to A
            //     self.a = self.next_byte();
            //     self.pc += 1;
            //     1
            // }
            // 0x41 => {
            //     self.bc[0] = self.bc[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x47 => {
            //     // Load A to B
            //     self.bc[0] = self.a;
            //     self.pc += 1;
            //     1
            // }
            // 0x48 => {
            //     // load B to C
            //     self.bc[1] = self.bc[0];
            //     self.pc += 1;
            //     1
            // }
            // 0x49 => {
            //     // load C to C, no-op
            //     self.pc += 1;
            //     1
            // }
            // 0x4A => {
            //     // load d to c
            //     self.de[0] = self.bc[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x4B => {
            //     // load c to e
            //     self.bc[1] = self.de[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x4C => {
            //     // load h to c
            //     self.hl[0] = self.bc[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x4D => {
            //     // load l to c
            //     self.hl[1] = self.bc[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x4E => {
            //     // load memory at position hl to c
            //     self.bc[1] = self.memory.read_byte(u16::from_le_bytes(self.hl));
            //     self.pc += 1;
            //     1
            // }
            // 0x4F => {
            //     // load a to c
            //     self.a = self.bc[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x50 => {
            //     // load b to d
            //     self.bc[0] = self.de[0];
            //     self.pc += 1;
            //     1
            // }
            // 0x51 => {
            //     // load c to d
            //     self.bc[1] = self.de[0];
            //     self.pc += 1;
            //     1
            // }
            // 0x52 => {
            //     // load d to d, no-op
            //     //self.de[0] = self.de[0];
            //     self.pc += 1;
            //     1
            // }
            // 0x53 => {
            //     // load e to d
            //     self.de[1] = self.de[0];
            //     self.pc += 1;
            //     1
            // }
            // 0x56 => {
            //     // load memory at position hl to d
            //     self.de[0] = self.memory.read_byte(u16::from_le_bytes(self.hl));
            //     self.pc += 1;
            //     1
            // }
            // 0x58 => {
            //     // load b to e
            //     self.bc[0] = self.de[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x5A => {
            //     // load D to E
            //     self.de[0] = self.de[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x5B => {
            //     // load E to E, no-op
            //     // self.de[1] = self.de[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x5C => {
            //     // load H to E
            //     self.de[1] = self.hl[0];
            //     self.pc += 1;
            //     1
            // }
            // 0x5D => {
            //     // load L to E
            //     self.de[1] = self.hl[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x5E => {
            //     // load memory at HL to E
            //     print!("ld e, (hl) | ");
            //     self.de[1] = self.memory.read_byte(u16::from_le_bytes(self.hl));
            //     self.pc += 1;
            //     1
            // }
            // 0x5F => {
            //     // load A to E
            //     print!("ld e, a | ");
            //     self.de[1] = self.a;
            //     self.pc += 1;
            //     1
            // }
            // 0x60 => {
            //     // load b to h
            //     self.bc[0] = self.hl[0];
            //     self.pc += 1;
            //     1
            // }
            // 0x67 => {
            //     // load a to h
            //     self.hl[0] = self.a;
            //     self.pc += 1;
            //     1
            // }
            // 0x6B => {
            //     // load h to l
            //     self.de[1] = self.hl[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x6E => {
            //     // load memory at position hl to l
            //     self.hl[1] = self.memory.read_byte(u16::from_le_bytes(self.hl));
            //     self.pc += 1;
            //     1
            // }
            // 0x6F => {
            //     // load a to l
            //     self.hl[1] = self.a;
            //     self.pc += 1;
            //     1
            // }
            // 0x70 => {
            //     self.memory
            //         .write_byte(u16::from_le_bytes(self.hl), self.bc[0]);
            //     self.pc += 1;
            //     2
            // }
            // 0x71 => {
            //     self.memory
            //         .write_byte(u16::from_le_bytes(self.hl), self.bc[1]);
            //     self.pc += 1;
            //     2
            // }
            // 0x72 => {
            //     self.memory
            //         .write_byte(u16::from_le_bytes(self.hl), self.de[0]);
            //     self.pc += 1;
            //     2
            // }
            // 0x73 => {
            //     self.memory
            //         .write_byte(u16::from_le_bytes(self.hl), self.de[1]);
            //     self.pc += 1;
            //     2
            // }
            // 0x74 => {
            //     self.memory
            //         .write_byte(u16::from_le_bytes(self.hl), self.hl[0]);
            //     self.pc += 1;
            //     2
            // }
            // 0x75 => {
            //     self.memory
            //         .write_byte(u16::from_le_bytes(self.hl), self.hl[1]);
            //     self.pc += 1;
            //     2
            // }
            // 0x77 => {
            //     self.memory.write_byte(u16::from_le_bytes(self.hl), self.a);
            //     self.pc += 1;
            //     2
            // }
            // 0x78 => {
            //     self.a = self.bc[0];
            //     self.pc += 1;
            //     1
            // }
            // 0x79 => {
            //     self.a = self.bc[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x7B => {
            //     // load e to a
            //     self.a = self.de[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x7C => {
            //     // load h to a
            //     self.a = self.hl[0];
            //     self.pc += 1;
            //     1
            // }
            // 0x7D => {
            //     // load l to a
            //     self.a = self.hl[1];
            //     self.pc += 1;
            //     1
            // }
            // 0x7E => {
            //     // load memory at hl to a
            //     self.a = self.memory.read_byte(u16::from_le_bytes(self.hl));
            //     self.pc += 1;
            //     1
            // }
            // 0x7F => {
            //     // load a to a, no-op
            //     self.pc += 1;
            //     2
            // }
            // 0x80 => {
            //     self.add(self.bc[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x81 => {
            //     self.add(self.bc[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x82 => {
            //     self.add(self.de[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x83 => {
            //     self.add(self.de[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x84 => {
            //     self.add(self.hl[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x85 => {
            //     self.add(self.hl[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x87 => {
            //     print!("add a, a | ");
            //     self.add(self.a);
            //     self.pc += 1;
            //     1
            // }
            // 0x88 => {
            //     self.add_to_a_with_carry(self.bc[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x89 => {
            //     self.add_to_a_with_carry(self.bc[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x8A => {
            //     self.add_to_a_with_carry(self.de[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x8C => {
            //     self.add_to_a_with_carry(self.hl[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x90 => {
            //     // sub b from a
            //     self.subtract(self.bc[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x91 => {
            //     // sub c from a
            //     self.subtract(self.bc[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x92 => {
            //     // sub d from a
            //     self.subtract(self.de[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x93 => {
            //     // sub e from a
            //     self.subtract(self.de[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x94 => {
            //     // sub h from a
            //     self.subtract(self.hl[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x95 => {
            //     // sub l from a
            //     self.subtract(self.hl[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x98 => {
            //     self.sub_from_a_with_carry(self.bc[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x99 => {
            //     self.sub_from_a_with_carry(self.bc[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x9A => {
            //     self.sub_from_a_with_carry(self.de[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x9B => {
            //     self.sub_from_a_with_carry(self.de[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x9C => {
            //     self.sub_from_a_with_carry(self.hl[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0x9D => {
            //     self.sub_from_a_with_carry(self.hl[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0x9E => {
            //     let data = self.memory.read_byte(u16::from_le_bytes(self.hl));
            //     self.sub_from_a_with_carry(data);
            //     self.pc += 1;
            //     2
            // }
            // 0xA0 => {
            //     // AND A with B and store in A
            //     self.a &= self.bc[0];
            //     self.pc += 1;
            //     1
            // }
            // 0xA1 => {
            //     // AND A with C and store in A
            //     self.a &= self.bc[1];
            //     self.pc += 1;
            //     1
            // }
            // 0xA2 => {
            //     // AND A with D and store in A
            //     self.a &= self.de[0];
            //     self.pc += 1;
            //     1
            // }
            // 0xA3 => {
            //     // AND A with E and store in A
            //     self.a &= self.de[1];
            //     self.pc += 1;
            //     1
            // }
            // 0xA4 => {
            //     // AND A with H and store in A
            //     self.a &= self.hl[0];
            //     self.pc += 1;
            //     1
            // }
            // 0xA5 => {
            //     // AND A with L and store in A
            //     self.a &= self.hl[1];
            //     self.pc += 1;
            //     1
            // }
            // 0xA6 => {
            //     // AND A with memory at HL and store in A
            //     self.a &= self.memory.read_byte(u16::from_le_bytes(self.hl));
            //     self.pc += 1;
            //     2
            // }
            // 0xA7 => {
            //     // AND A with A and store in A
            //     self.a &= self.a;
            //     self.pc += 1;
            //     1
            // }
            // 0xA8 => {
            //     // XOR A with B and store in A
            //     self.xor_a(self.bc[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0xA9 => {
            //     // XOR A with C and store in A
            //     self.xor_a(self.bc[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0xAA => {
            //     // XOR A with D and store in A
            //     self.xor_a(self.de[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0xAB => {
            //     // XOR A with E and store in A
            //     self.xor_a(self.de[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0xAC => {
            //     // XOR A with H and store in A
            //     self.xor_a(self.hl[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0xAD => {
            //     // XOR A with L and store in A
            //     self.xor_a(self.hl[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0xAF => {
            //     // XOR the A register with itself
            //     self.xor_a(self.a);
            //     self.pc += 1;
            //     1
            // }
            // 0xB0 => {
            //     // OR A with B and store in A
            //     self.or_a(self.bc[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0xB1 => {
            //     // OR A with C and store in A
            //     self.or_a(self.bc[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0xB2 => {
            //     // OR A with D and store in A
            //     self.or_a(self.de[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0xB3 => {
            //     // OR A with E and store in A
            //     self.or_a(self.de[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0xB4 => {
            //     // OR A with H and store in A
            //     self.or_a(self.hl[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0xB5 => {
            //     // OR A with L and store in A
            //     self.or_a(self.hl[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0xB6 => {
            //     // OR A with (HL) and store in A
            //     let data = self.memory.read_byte(u16::from_le_bytes(self.hl));
            //     self.or_a(data);
            //     self.pc += 1;
            //     2
            // }
            // 0xB7 => {
            //     // OR A with A and store in A
            //     self.or_a(self.a);
            //     self.pc += 1;
            //     1
            // }
            // 0xB8 => {
            //     // compare A with B, set flags
            //     self.compare_a(self.bc[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0xB9 => {
            //     // compare A with C, set flags
            //     self.compare_a(self.bc[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0xBA => {
            //     // compare A with D, set flags
            //     self.compare_a(self.de[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0xBB => {
            //     // compare A with E, set flags
            //     self.compare_a(self.de[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0xBC => {
            //     // compare A with H, set flags
            //     self.compare_a(self.hl[0]);
            //     self.pc += 1;
            //     1
            // }
            // 0xBD => {
            //     // compare A with L, set flags
            //     self.compare_a(self.hl[1]);
            //     self.pc += 1;
            //     1
            // }
            // 0xBF => {
            //     // compare A with A, set flags
            //     self.compare_a(self.a);
            //     self.pc += 1;
            //     1
            // }
            // 0xC0 => {
            //     // RET NZ - if flag z = 0, pop word from stack and set PC to that
            //     print!("RET NZ | ");
            //     if !self.flags.z {
            //         let address = self.pop_word();
            //         print!("jump to {:#08X}", address);
            //         self.pc = address;
            //     } else {
            //         print!("SKIP");
            //         self.pc += 1;
            //     }
            //
            //     1
            // }
            // 0xC3 => {
            //     // Jump to the address immediately after the current instruction
            //     let data: [u8; 2] = self.next_word();
            //     let pos: u16 = u16::from_le_bytes(data);
            //     self.pc = pos;
            //     1
            // }
            // 0xC5 => {
            //     self.push_word(u16::from_le_bytes(self.bc));
            //     self.pc += 1;
            //     4
            // }
            // 0xC9 => {
            //     // RET - pop word from stack and set PC to that
            //     print!("RET NZ | ");
            //     let address = self.pop_word();
            //     print!("jump to {:#08X}", address);
            //     self.pc = address;
            //     1
            // }
            // 0xCC => {
            //     // CALL z,a16 = push PC to stack and jump to next byte IF z = 1
            //     if self.flags.z {
            //         self.push_word(self.pc + 1);
            //         let address = u16::from_le_bytes(self.next_word());
            //         print!("CALL z,a16 | {:#08X}", address);
            //         self.pc = address;
            //         6
            //     } else {
            //         print!("CALL z,a16 | SKIP");
            //         self.pc += 2;
            //         3
            //     }
            // }
            // 0xD5 => {
            //     self.push_word(u16::from_le_bytes(self.de));
            //     self.pc += 1;
            //     4
            // }
            // 0xCB => {
            //     self.pc += 1;
            //     self.handle_word_opcode()
            // }
            // 0xCD => {
            //     // CALL a16 = push PC to stack and jump to next byte
            //     self.push_word(self.pc + 1);
            //     let address = u16::from_le_bytes(self.next_word());
            //     print!("CALL a16 | {:#08X}", address);
            //     self.pc = address;
            //     1
            // }
            // 0xD2 => {
            //     // Jump to address in next word if carry flag is true, or continue
            //     if self.flags.c {
            //         self.pc = u16::from_le_bytes(self.next_word());
            //     } else {
            //         self.pc += 1;
            //     }
            //     1
            // }
            // 0xD6 => {
            //     // subtract net byte from A and saver to A
            //     let data = self.next_byte();
            //     self.subtract(data);
            //     self.pc += 1;
            //     1
            // }
            // 0xE0 => {
            //     // Store A to address at memory 0xFFxx, where xx is the next byte
            //     let address = self.next_byte();
            //     self.memory
            //         .write_byte(u16::from_le_bytes([address, 0xff]), self.a);
            //     self.pc += 1;
            //     3
            // }
            // 0xE1 => {
            //     // pop stack to HL
            //     print!("pop hl | ");
            //     self.hl = self.pop_word().to_le_bytes();
            //     self.pc += 1;
            //     3
            // }
            // 0xE2 => {
            //     // Store A to address at memory 0xFFxx, where xx is register C
            //     let address = self.bc[1];
            //     self.memory
            //         .write_byte(u16::from_le_bytes([address, 0xff]), self.a);
            //     self.pc += 1;
            //     2
            // }
            // 0xE5 => {
            //     self.push_word(u16::from_le_bytes(self.hl));
            //     self.pc += 1;
            //     4
            // }
            // 0xE6 => {
            //     let data = self.next_byte();
            //     self.a &= data;
            //     self.pc += 1;
            //     2
            // }
            // 0xE9 => {
            //     self.pc = u16::from_le_bytes(self.hl);
            //     1
            // }
            //
            // 0xEA => {
            //     // Store A to address at memory 0xFFxx, where xx is the next byte
            //     let address = u16::from_le_bytes(self.next_word());
            //     self.memory.write_byte(address, self.a);
            //     self.pc += 4;
            //     1
            // }
            // 0xEF => {
            //     // RST 5
            //     self.push_word(self.pc);
            //     self.pc = 0x28;
            //     4
            // }
            // 0xF0 => {
            //     // Load address at memory 0xFFxx into A, where xx is the next byte
            //     let data = self.next_byte();
            //     self.a = self.memory.read_byte(u16::from_le_bytes([data, 0xff]));
            //     self.pc += 1;
            //     3
            // }
            // 0xF3 => {
            //     self.interrupts_enabled = false;
            //     self.pc += 1;
            //     1
            // }
            // 0xF5 => {
            //     self.push_word(u16::from_le_bytes([self.flags.to_bits(), self.a]));
            //     self.pc += 1;
            //     4
            // }
            // 0xF8 => {
            //     // LD HL, SP+s8
            //     let int = i8::from_le_bytes([self.memory.read_byte(self.pc + 1)]);
            //     let val = (self.sp as i16).checked_add(int as i16).expect("Fail");
            //
            //     self.hl = (val as u16).to_le_bytes();
            //     self.pc += 2;
            //     3
            // }
            // 0xF9 => {
            //     // Load HL to SP
            //     self.sp = u16::from_le_bytes(self.hl);
            //     self.pc += 1;
            //     1
            // }
            // 0xFB => {
            //     self.interrupts_enabled = true;
            //     self.pc += 1;
            //     1
            // }
            // 0xFE => {
            //     // Set Z flag if A and next byte are equal by calculating A - next byte == 0
            //     let data = self.next_byte();
            //     self.compare_a(data);
            //     self.pc += 1;
            //     1
            // }
            // 0xFF => {
            //     // RST 7, we should never hit this
            //     print!("RST 7");
            //     println!();
            //     self.debug_registers();
            //     panic!();
            //     // self.push_word(self.pc);
            //     // self.pc = 0x38;
            //     // 4
            // }
        };

        println!();

        if self.state == CpuState::Stopped {
            println!("--------- BREAK -----------");
            self.debug_all();

            let mut input = String::new(); // Take user input (to be parsed as clap args)
            io::stdin()
                .read_line(&mut input)
                .expect("Error reading input.");

            match input.as_str() {
                "N" => (),
                "C" => {
                    self.state = CpuState::Running;
                }
                _ => println!("C to continue or N for next"),
            }
        }

        m_cycles
    }

    // fn handle_word_opcode(&mut self) -> usize {
    //     let instruction = self.cartridge.data[self.pc as usize];
    //     print!("{:#04X} | ", instruction);
    //
    //     let register_byte = instruction & 0x7;
    //     let register = match register_byte {
    //         0 => Register::B,
    //         1 => Register::C,
    //         2 => Register::D,
    //         3 => Register::E,
    //         4 => Register::H,
    //         5 => Register::L,
    //         6 => Register::HL,
    //         7 => Register::A,
    //         _ => {
    //             println!();
    //             self.debug_registers();
    //             panic!("Unknown 16-bit instruction register {:?}", register_byte);
    //         }
    //     };
    //
    //     print!("reg {:?} | ", register);
    //
    //     match instruction & 0b11000000 {
    //         0x00 => {
    //             // Shift / rotate
    //             let opcode_byte = (instruction & 0b111000) >> 3; // bit 3, 4 and 5 give the opcode
    //
    //             match opcode_byte {
    //                 // 0 => {
    //                 //     // RLC
    //                 //     1
    //                 // }
    //                 // 1 => {
    //                 //     // RRC
    //                 //     1
    //                 // }
    //                 // 2 => {
    //                 //     // RL
    //                 //     1
    //                 // }
    //                 // 3 => {
    //                 //     // RR
    //                 //     1
    //                 // }
    //                 // 4 => {
    //                 //     // SLA
    //                 //     1
    //                 // }
    //                 // 5 => {
    //                 //     // SRA
    //                 //     1
    //                 // }
    //                 6 => {
    //                     // SWAP
    //                     if register == Register::HL {
    //                         self.swap_memory(u16::from_le_bytes(self.hl));
    //                         self.pc += 1;
    //                         4
    //                     } else {
    //                         self.swap_register(register);
    //                         self.pc += 1;
    //                         1
    //                     }
    //                 }
    //                 // 7 => {
    //                 //     // SRL
    //                 //     1
    //                 // }
    //                 _ => {
    //                     println!();
    //                     self.debug_registers();
    //                     panic!(
    //                         "Unknown 16-bit instruction opcode {:#08b} {:?}",
    //                         opcode_byte, opcode_byte
    //                     );
    //                 }
    //             }
    //         }
    //         0x40 => {
    //             // BIT
    //             // divide by 8 and take remainder of mod 8
    //             let bit = (instruction >> 3) & 0x7;
    //             print!("bit {:#04X}", bit);
    //
    //             self.flags.z = (*self.get_register(register) >> bit) & 0x1 == 0x0;
    //             self.flags.n = false;
    //             self.flags.h = true;
    //
    //             self.pc += 1;
    //             4
    //         }
    //         // 0x80 => {
    //         //     // RES
    //         //     1
    //         // }
    //         // 0xC0 => {
    //         //     // SET
    //         //     1
    //         // }
    //         _ => {
    //             println!();
    //             self.debug_registers();
    //             panic!("Unknown 16-bit instruction type");
    //         }
    //     }
    // }

    fn ld(&mut self, load: Load) -> usize {
        match load.source {
            Operand::Register(source) => {
                let source_data = self.get_register(source);
                match load.target {
                    Operand::Register(target) => {
                        self.set_register(target, source_data);
                        1
                    }
                    Operand::MemoryLocation(memory_location) => match memory_location {
                        MemoryLocation::RegisterPair(pair) => {
                            let location = self.get_register_pair(pair);
                            self.memory.write_byte(location, source_data);
                            2
                        }
                        MemoryLocation::ImmediateOperand(operand) => match operand {
                            ImmediateOperand::A8(operand) => {
                                self.memory
                                    .write_byte(u16::from_le_bytes([operand, 0xff]), source_data);
                                3
                            }
                            _ => panic!("not implemented"),
                        },
                        MemoryLocation::HLplus => {
                            let pair = RegisterPair(Register::H, Register::L);
                            let location = self.get_register_pair(pair);
                            self.memory.write_byte(location, source_data);
                            self.set_register_pair(pair, location.overflowing_add(1).0);
                            2
                        }
                        MemoryLocation::HLmin => {
                            let pair = RegisterPair(Register::H, Register::L);
                            let location = self.get_register_pair(pair);
                            self.memory.write_byte(location, source_data);
                            self.set_register_pair(pair, location.overflowing_sub(1).0);
                            2
                        }
                    },
                    _ => panic!("not implemented"),
                }
            }
            Operand::MemoryLocation(MemoryLocation::ImmediateOperand(ImmediateOperand::A8(
                operand,
            ))) => {
                let data = self.memory.read_byte(u16::from_le_bytes([operand, 0xff]));
                match load.target {
                    Operand::Register(register) => {
                        self.set_register(register, data);
                        2
                    }
                    _ => panic!("not implemented"),
                }
            }
            Operand::RegisterPair(source) => match load.target {
                Operand::RegisterPair(target) => {
                    self.set_register_pair(target, self.get_register_pair(source));
                    2
                }
                _ => panic!("not implemented"),
            },
            Operand::ImmediateOperand(ImmediateOperand::D8(operand)) => match load.target {
                Operand::Register(target) => {
                    self.set_register(target, operand);
                    2
                }
                _ => panic!("not implemented"),
            },
            Operand::ImmediateOperand(ImmediateOperand::A16(operand)) => match load.target {
                Operand::RegisterPair(target) => {
                    self.set_register_pair(target, operand);
                    3
                }
                _ => panic!("not implemented"),
            },
            Operand::ImmediateOperand(ImmediateOperand::D16(operand)) => match load.target {
                Operand::RegisterPair(target) => {
                    self.set_register_pair(target, operand);
                    3
                }
                _ => panic!("not implemented"),
            },
            _ => panic!("not implemented"),
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

        [a, b]
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
    fn decrement(&mut self, operand: Operand) -> usize {
        let mut byte = 0;
        let mut cycles = 1;
        match operand {
            Operand::Register(register) => {
                byte = self.get_register(register);
                byte = byte.overflowing_sub(1).0;
                self.set_register(register, byte);

                self.flags.z = byte == 0;
                self.flags.n = true;
                self.flags.h = (byte & 0xF) == 0xF;
            }
            Operand::RegisterPair(_) => panic!("not implemented"),
            Operand::MemoryLocation(_) => panic!("not implemented"),
            Operand::ImmediateOperand(_) => panic!("not implemented"),
            Operand::StackPointer => {
                self.sp = self.sp.wrapping_sub(1);
                cycles = 2;
            }
        }

        cycles
    }

    fn xor_a(&mut self, source: Operand) -> usize {
        let (source_data, cycles) = match source {
            Operand::Register(source) => (self.get_register(source), 1),
            Operand::MemoryLocation(_) => panic!("not implemented"), // 2 cycles
            Operand::RegisterPair(_) => panic!("Should not happen"),
            Operand::ImmediateOperand(_) => panic!("not implemented"),
            Operand::StackPointer => panic!("not implemented"),
        };

        self.a ^= source_data;
        self.flags.z = self.a == 0;
        cycles
    }

    fn or_a(&mut self, byte: u8) {
        self.a |= byte;
        self.flags.z = self.a == 0;
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

    fn sub_from_a_with_carry(&mut self, byte: u8) {
        let carry: u16 = u16::from(self.flags.c);

        let res = (self.a as u16) - (byte as u16 + carry);

        self.flags.z = res == 0x0;
        self.flags.h = ((self.a & 0x0F) - (byte & 0x0F) - (carry as u8)) > 0xf;
        self.flags.c = res > 0xff;
        self.flags.n = true;
        self.a = (res & 0xff) as u8;
    }

    /**
     * Add register to HL
     */
    // fn add_to_reg_hl(&mut self, b: [u8; 2]) {
    //     let hl_u16 = u16::from_le_bytes(self.hl);
    //     let b_u16 = u16::from_le_bytes(b);
    //     let temp = hl_u16 as u32 + b_u16 as u32;
    //     self.flags.n = false;
    //     self.flags.c = temp > 0xFFFF;
    //     self.flags.h = ((hl_u16 & 0xFFF) + (b_u16 & 0xFFF)) > 0xFFF;
    //     self.hl = ((temp & 0xFFFF) as u16).to_le_bytes();
    // }

    /** add bye to register a */
    fn add(&mut self, byte: u8) {
        let n: u16 = self.a as u16 + byte as u16;
        self.flags.z = (n & 0xFF) == 0;
        self.flags.h = ((self.a & 0x0F) + (byte & 0x0F)) & 0x10 == 0x10;
        self.flags.c = n > 0xFF;
        self.flags.n = false;
        self.a = (n & 0xFF) as u8;
    }

    /** subtract byte from register A */
    // fn subtract(&mut self, byte: u8) {
    //     self.compare_a(byte);
    //     (self.a, _) = self.a.overflowing_sub(byte);
    // }

    fn compare_a(&mut self, operand: Operand) -> usize {
        let mut cycles = 1;

        let data = match operand {
            Operand::Register(register) => self.get_register(register),
            Operand::ImmediateOperand(ImmediateOperand::D8(operand)) => {
                cycles = 2;
                operand
            }
            _ => panic!("not implemented"),
        };

        self.flags.z = self.a == data;
        self.flags.n = true;
        self.flags.h = (self.a & 0xF) < (data & 0xF);
        self.flags.c = self.a < data;

        cycles
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

    fn jump(&mut self, operand: ImmediateOperand) -> usize {
        match operand {
            ImmediateOperand::A16(a16) => self.pc = a16,
            _ => panic!("Unknown operand type for JP instruction"),
        }
        4
    }

    fn jump_relative(&mut self, operand: ImmediateOperand, condition: Option<Condition>) -> usize {
        let ImmediateOperand::S8(operand) = operand else {
            panic!("Invalid operand for JR");
        };

        if let Some(condition) = condition {
            match condition {
                Condition::NZ => {
                    if self.flags.z == false {
                        self.pc = self.pc.wrapping_add_signed(operand as i16)
                    } else {
                        return 2;
                    }
                }
                Condition::NC => {
                    if self.flags.c == false {
                        self.pc = self.pc.wrapping_add_signed(operand as i16)
                    } else {
                        return 2;
                    }
                }
            }
        } else {
            self.pc = self.pc.wrapping_add_signed(operand as i16);
        }

        3
    }

    fn debug_registers(&self) {
        println!("Program counter: {:#08X}", self.pc);
        println!("Stack pointer: {:#04X}", self.sp);
        println!("A: {:#04X}", self.a);
        println!("BC: {:#04X} {:#04X}", self.b, self.c);
        println!("DE: {:#04X} {:#04X}", self.d, self.e);
        println!("HL: {:#04X} {:#04X}", self.h, self.l);
        println!("Flags: {:?}", self.flags);
    }

    fn debug_flags(&self) {
        println!("Interrupts enabled: {:?}", self.interrupts_enabled);
        println!("Interrupt enable flags: {:?}", self.memory.interrupt_enable);
        println!("Interrupt flags: {:?}", self.memory.interrupt_flags);
    }

    fn debug_video(&self) {
        println!("Video mode: {:?}", self.memory.video.mode);
        println!("Video lcd control: {:?}", self.memory.video.lcd_control);
        println!("Video lcd status: {:?}", self.memory.video.lcd_status);
    }

    fn debug_stack(&mut self) {
        println!("--- STACK START ---");
        for i in self.sp..=STACK_START {
            println!("{:#04X}", self.memory.read_byte(i));
        }
        println!("--- STACK END ---");
    }

    fn debug_all(&mut self) {
        println!("------- REGISTERS ----------");
        self.debug_registers();
        println!("--------- FLAGS ------------");
        self.debug_flags();
        println!("--------- VIDEO ------------");
        self.debug_video();
    }

    fn swap_memory(&mut self, pos: u16) {
        let data = self.memory.read_byte(pos);

        let upper = data & 0b11110000;
        let lower = data & 0b00001111;

        self.memory.write_byte(pos, upper | lower);
    }

    // fn swap_register(&mut self, reg: Register) {
    //     let reg = self.get_register(reg);
    //
    //     let upper = *reg & 0b11110000;
    //     let lower = *reg & 0b00001111;
    //
    //     *reg = upper | lower;
    // }

    fn get_register(&self, reg: Register) -> u8 {
        match reg {
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::H => self.h,
            Register::L => self.l,
            Register::A => self.a,
        }
    }

    fn get_register_pair(&self, reg_pair: RegisterPair) -> u16 {
        u16::from_le_bytes([self.get_register(reg_pair.0), self.get_register(reg_pair.1)])
    }

    fn set_register(&mut self, reg: Register, data: u8) {
        match reg {
            Register::B => self.b = data,
            Register::C => self.c = data,
            Register::D => self.d = data,
            Register::E => self.e = data,
            Register::H => self.h = data,
            Register::L => self.l = data,
            Register::A => self.a = data,
        };
    }

    fn set_register_pair(&mut self, reg_pair: RegisterPair, data: u16) {
        let data = u16::to_le_bytes(data);

        self.set_register(reg_pair.0, data[0]);
        self.set_register(reg_pair.1, data[1]);
    }
}

#[cfg(test)]
mod tests {
    use crate::cartridge::CartridgeHeader;
    use crate::{Cartridge, Cpu};

    #[test]
    fn it_handles_registers() {
        let cartridge = Cartridge {
            header: CartridgeHeader {
                title: "test".to_string(),
                cgb_flag: 0,
                cartridge_type: 0,
                licensee_code: 0,
                rom_size: 0,
                ram_size: 0,
            },
            data: vec![0x21, 0x01, 0x00],
        };

        let mut cpu = Cpu::load_cartridge(cartridge);
        cpu.pc = 0;
        cpu.run(None, None);
        dbg!(cpu);
    }
}
