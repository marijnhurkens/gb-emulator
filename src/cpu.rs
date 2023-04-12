use std::io;
use std::ops::Sub;
use std::thread::sleep;
use std::time::{Duration, Instant};

use bitvec::macros::internal::funty::Fundamental;
use tracing::{event, Level};

use crate::cartridge::Cartridge;
use crate::instructions::{
    decode, Add, Condition, ImmediateOperand, Instruction, InstructionCB, Load, MemoryLocation,
    Operand, Register, RegisterPair,
};
use crate::memory::{InterruptFlags, Memory};
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
    current_opcode: Option<u8>,
    current_instruction: Option<Instruction>,
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
        (self.c as u8) << 4 | (self.h as u8) << 5 | (self.n as u8) << 6 | (self.z as u8) << 7
    }

    pub fn set_bits(&mut self, bits: u8) {
        self.c = bits & 0x10 == 0x10;
        self.h = bits & 0x20 == 0x20;
        self.n = bits & 0x40 == 0x40;
        self.z = bits & 0x80 == 0x80;
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
            b: 0x00,
            c: 0x13,
            d: 0x00,
            e: 0xD8,
            h: 0x01,
            l: 0x4D,
            flags: CpuFlags {
                z: true,
                c: true,
                n: false,
                h: true,
            },
            interrupts_enabled: false,
            memory: Memory::new(cartridge.data.clone()),
            cartridge,
            state: CpuState::Stopped,
            break_point: None,
            current_opcode: None,
            current_instruction: None,
        }
    }

    pub fn run(&mut self, screen_buffer: Option<ScreenBuffer>, break_point: Option<u16>) {
        let title = &self.cartridge.header.title;
        event!(Level::INFO, "Running {:}", title);

        self.break_point = break_point;
        self.state = CpuState::Running;

        loop {
            self.cycle(&screen_buffer);
        }
    }

    pub fn cycle(&mut self, screen_buffer: &Option<ScreenBuffer>) {
        // Draw to screen
        if let Some(buffer) = &screen_buffer {
            let mut guard = buffer.lock().unwrap();
            (*guard) = self.memory.read_vram();
            drop(guard);
        }

        // Catch PC out of bounds
        if self.pc as usize > 0xFFFF {
            panic!("PC out of bounds");
        }

        self.current_opcode = Some(self.memory.read_byte(self.pc));
        let (instruction, length) = decode(&mut self.memory, self.pc);
        self.current_instruction = Some(instruction);

        self.log_doctor();

        event!(
            Level::DEBUG,
            "{:#08X} | {:#04X} | {}",
            self.pc,
            self.memory.read_byte(self.pc),
            instruction
        );

        if let Some(break_point) = self.break_point {
            if break_point == self.pc {
                self.state = CpuState::Stopped;
            }
        }

        self.pc = self.pc.wrapping_add(length);

        let start_time = Instant::now();
        let t_cycles = self.process_instruction(instruction) * 4;

        for _ in 0..t_cycles {
            self.memory.step();

            let interrupt_flags = self.memory.video.step(self.memory.interrupt_flags);
            self.memory.interrupt_flags = interrupt_flags;

            // INTERRUPTS
            if self.interrupts_enabled {
                if self.memory.interrupt_flags.contains(InterruptFlags::VBLANK)
                    && self
                        .memory
                        .interrupt_enable
                        .intersects(InterruptFlags::VBLANK)
                {
                    self.interrupts_enabled = false;
                    let interrupt_flags =
                        InterruptFlags::from_bits(self.memory.read_byte(0xFF0F)).unwrap();
                    self.memory
                        .write_byte(0xFF0F, (interrupt_flags - InterruptFlags::VBLANK).bits());

                    self.push_word(self.pc + 1);
                    let address = 0x0040;
                    event!(Level::INFO, "INT 40 VBLANK | {:#08X}", address);
                    self.pc = address;
                }

                if self.memory.interrupt_flags.contains(InterruptFlags::TIMER)
                    && self
                        .memory
                        .interrupt_enable
                        .intersects(InterruptFlags::TIMER)
                {
                    self.interrupts_enabled = false;
                    let interrupt_flags =
                        InterruptFlags::from_bits(self.memory.read_byte(0xFF0F)).unwrap();
                    self.memory
                        .write_byte(0xFF0F, (interrupt_flags - InterruptFlags::TIMER).bits());

                    self.push_word(self.pc + 1);
                    let address = 0x0050;
                    event!(Level::INFO, "INT 50 TIMER | {:#08X}", address);
                    self.pc = address;
                }

                if self
                    .memory
                    .interrupt_flags
                    .contains(InterruptFlags::LCD_STAT)
                    && self
                        .memory
                        .interrupt_enable
                        .intersects(InterruptFlags::LCD_STAT)
                {
                    self.interrupts_enabled = false;
                    let interrupt_flags =
                        InterruptFlags::from_bits(self.memory.read_byte(0xFF0F)).unwrap();
                    self.memory
                        .write_byte(0xFF0F, (interrupt_flags - InterruptFlags::LCD_STAT).bits());

                    self.push_word(self.pc + 1);
                    let address = 0x0048;
                    event!(Level::INFO, "INT 48 STAT | {:#08X}", address);
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

    /**
     * Runs the next instruction and returns the cycle cost divided by 4 (M-cycles)
     */
    fn process_instruction(&mut self, instruction: Instruction) -> usize {
        let m_cycles = match instruction {
            Instruction::NOP => 1,
            Instruction::HALT => panic!("HALT"),
            Instruction::INC(operand) => self.increment(operand),
            Instruction::DEC(operand) => self.decrement(operand),
            Instruction::LD(load) => self.ld(load),
            Instruction::ADD(add) => self.add(add),
            Instruction::ADC(operand) => self.adc(operand),
            Instruction::SUB(operand) => self.sub(operand),
            Instruction::SBC(operand) => self.sbc(operand),
            Instruction::JP(operand, condition) => self.jump(operand, condition),
            Instruction::XOR(source) => self.xor_a(source),
            Instruction::OR(source) => self.or_a(source),
            Instruction::AND(operand) => self.and(operand),
            Instruction::JR(operand, condition) => self.jump_relative(operand, condition),
            Instruction::DI => {
                self.interrupts_enabled = false;
                1
            }
            Instruction::CP(operand) => self.compare_a(operand),
            Instruction::Call(operand, condition) => self.call(operand, condition),
            Instruction::RET(condition) => self.ret(condition),
            Instruction::RETI => self.reti(),
            Instruction::EI => {
                self.interrupts_enabled = true;
                1
            }
            Instruction::CPL => self.cpl(),
            Instruction::CB(instruction) => self.handle_cb(instruction),
            Instruction::RST(number) => {
                self.push_word(self.pc);
                self.pc = (number as u16) << 3;
                4
            }
            Instruction::PUSH(source) => self.push(source),
            Instruction::POP(target) => self.pop(target),
            Instruction::RLCA => self.rlca(),
            Instruction::DAA => self.daa(),
            Instruction::SCF => self.scf(),
            Instruction::RRA => self.rra(),
        };

        if self.state == CpuState::Stopped {
            event!(Level::ERROR, "--------- BREAK -----------");

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

    fn handle_cb(&mut self, instruction: InstructionCB) -> usize {
        match instruction {
            InstructionCB::SWAP(source) => self.swap(source),
            InstructionCB::BIT(bit, target) => self.bit(bit, target),
            InstructionCB::RR(source) => self.rr(source),
            InstructionCB::SRL(source) => self.srl(source),
            InstructionCB::RES(bit, target) => self.res(bit, target),
            InstructionCB::SET(bit, target) => self.set(bit, target),
        }
    }

    fn call(&mut self, operand: ImmediateOperand, condition: Option<Condition>) -> usize {
        match condition {
            Some(Condition::Z) => {
                if !self.flags.z {
                    return 3;
                }
            }
            Some(Condition::C) => {
                if !self.flags.c {
                    return 3;
                }
            }
            Some(Condition::NZ) => {
                if self.flags.z {
                    return 3;
                }
            }
            Some(Condition::NC) => {
                if self.flags.c {
                    return 3;
                }
            }
            _ => (),
        };

        self.push_word(self.pc);

        let address = match operand {
            ImmediateOperand::A16(operand) => operand,
            _ => panic!("should not happen"),
        };

        self.pc = address;
        6
    }

    fn ret(&mut self, condition: Option<Condition>) -> usize {
        let mut cycles = 5;
        match condition {
            Some(Condition::Z) => {
                if !self.flags.z {
                    return 2;
                }
            }
            Some(Condition::C) => {
                if !self.flags.c {
                    return 2;
                }
            }
            Some(Condition::NZ) => {
                if self.flags.z {
                    return 2;
                }
            }
            Some(Condition::NC) => {
                if self.flags.c {
                    return 2;
                }
            }
            _ => {
                cycles = 4;
            }
        };

        let address = self.pop_word();
        self.pc = address;

        cycles
    }

    fn reti(&mut self) -> usize {
        let address = self.pop_word();
        self.pc = address;

        self.interrupts_enabled = true;

        4
    }

    fn daa(&mut self) -> usize {
        if !self.flags.n {
            if self.flags.c || self.a > 0x99 {
                self.a = self.a.wrapping_add(0x60);
                self.flags.c = true;
            }
            if self.flags.h || (self.a & 0x0f) > 0x9 {
                self.a = self.a.wrapping_add(0x6);
            }
        } else {
            if self.flags.c {
                self.a = self.a.wrapping_sub(0x60);
            }
            if self.flags.h {
                self.a = self.a.wrapping_sub(0x6);
            }
        }

        self.flags.z = self.a == 0;
        self.flags.h = false;

        1
    }

    fn scf(&mut self) -> usize {
        self.flags.h = false;
        self.flags.n = false;
        self.flags.c = true;

        1
    }

    fn cpl(&mut self) -> usize {
        self.a = !self.a;
        self.flags.n = true;
        self.flags.h = true;
        1
    }

    fn swap(&mut self, source: Operand) -> usize {
        let mut cycles = 2;
        let result = match source {
            Operand::Register(register) => {
                let data = self.get_register(register);
                let result = ((data & 0b00001111) << 4) & (data >> 4);
                self.set_register(register, result);
                result
            }
            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    let memory_pos = self.get_register_pair(pair);
                    let data = self.memory.read_byte(memory_pos);
                    let result = ((data & 0b00001111) << 4) & (data >> 4);
                    self.memory.write_byte(memory_pos, result);
                    cycles = 4;
                    result
                }
                _ => panic!("should not happen"),
            },
            _ => panic!("should not happen"),
        };

        self.flags.z = result == 0;
        self.flags.n = false;
        self.flags.h = false;
        self.flags.c = false;

        cycles
    }

    fn rr(&mut self, source: Operand) -> usize {
        let mut cycles = 2;
        let result = match source {
            Operand::Register(register) => {
                let data = self.get_register(register);
                let carry_prev = self.flags.c as u8;
                self.flags.c = data & 0x1 == 0x1;
                let result = (data >> 1) | carry_prev << 7;
                self.set_register(register, result);
                result
            }
            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    let memory_pos = self.get_register_pair(pair);
                    let data = self.memory.read_byte(memory_pos);

                    let carry_prev = self.flags.c as u8;
                    self.flags.c = data & 0x1 == 0x1;
                    let result = (data >> 1) | carry_prev << 7;

                    self.memory.write_byte(memory_pos, result);
                    cycles = 4;
                    result
                }
                _ => panic!("should not happen"),
            },
            _ => panic!("should not happen"),
        };

        self.flags.z = result == 0;
        self.flags.n = false;
        self.flags.h = false;

        cycles
    }

    fn srl(&mut self, source: Operand) -> usize {
        let mut cycles = 2;
        let result = match source {
            Operand::Register(register) => {
                let data = self.get_register(register);
                self.flags.c = data & 0x1 == 0x1;
                let result = data >> 1;
                self.set_register(register, result);
                result
            }
            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    let memory_pos = self.get_register_pair(pair);
                    let data = self.memory.read_byte(memory_pos);

                    self.flags.c = data & 0x1 == 0x1;
                    let result = data >> 1;

                    self.memory.write_byte(memory_pos, result);
                    cycles = 4;
                    result
                }
                _ => panic!("should not happen"),
            },
            _ => panic!("should not happen"),
        };

        self.flags.z = result == 0;
        self.flags.n = false;
        self.flags.h = false;

        cycles
    }

    fn bit(&mut self, bit: u8, target: Operand) -> usize {
        let mut cycles = 2;

        match target {
            Operand::Register(target) => {
                self.flags.z = (self.get_register(target) >> bit) & 0x1 == 0x0;
            }

            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    let memory_pos = self.get_register_pair(pair);
                    let data = self.memory.read_byte(memory_pos);
                    self.flags.z = (data >> bit) & 0x1 == 0x0;
                    cycles = 3;
                }
                _ => panic!("should not happen"),
            },
            _ => panic!("should not happen"),
        }

        self.flags.n = false;
        self.flags.h = true;

        cycles
    }

    fn res(&mut self, bit: u8, target: Operand) -> usize {
        let mut cycles = 2;

        match target {
            Operand::Register(target) => {
                self.set_register(target, self.get_register(target) & !(0x1 << bit));
            }

            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    let memory_pos = self.get_register_pair(pair);
                    let data = self.memory.read_byte(memory_pos);
                    self.memory.write_byte(memory_pos, data & !(0x1 << bit));

                    cycles = 4;
                }
                _ => panic!("should not happen"),
            },
            _ => panic!("should not happen"),
        }

        cycles
    }

    fn set(&mut self, bit: u8, target: Operand) -> usize {
        let mut cycles = 2;

        match target {
            Operand::Register(target) => {
                self.set_register(target, self.get_register(target) & (0x1 << bit));
            }

            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    let memory_pos = self.get_register_pair(pair);
                    let data = self.memory.read_byte(memory_pos);
                    self.memory.write_byte(memory_pos, data & (0x1 << bit));

                    cycles = 4;
                }
                _ => panic!("should not happen"),
            },
            _ => panic!("should not happen"),
        }

        cycles
    }

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
                            ImmediateOperand::A16(operand) => {
                                self.memory.write_byte(operand, source_data);
                                4
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
                        MemoryLocation::Register(register) => {
                            self.memory.write_byte(
                                u16::from_le_bytes([self.get_register(register), 0xFF]),
                                source_data,
                            );
                            2
                        }
                    },
                    _ => panic!("not implemented"),
                }
            }
            Operand::MemoryLocation(memory_location) => match memory_location {
                MemoryLocation::ImmediateOperand(operand) => match operand {
                    ImmediateOperand::A8(operand) => {
                        let data = self.memory.read_byte(u16::from_le_bytes([operand, 0xff]));
                        match load.target {
                            Operand::Register(register) => {
                                self.set_register(register, data);
                                3
                            }
                            _ => panic!("not implemented"),
                        }
                    }
                    ImmediateOperand::A16(operand) => {
                        let data = self.memory.read_byte(operand);
                        match load.target {
                            Operand::Register(register) => {
                                self.set_register(register, data);
                                4
                            }
                            _ => panic!("not implemented"),
                        }
                    }
                    _ => panic!("not implemented"),
                },
                MemoryLocation::RegisterPair(register_pair) => {
                    let data = self.memory.read_byte(self.get_register_pair(register_pair));
                    match load.target {
                        Operand::Register(register) => {
                            self.set_register(register, data);
                            2
                        }
                        _ => panic!("not implemented"),
                    }
                }
                MemoryLocation::HLplus => {
                    let register_pair = RegisterPair(Register::H, Register::L);
                    let memory_location = self.get_register_pair(register_pair);
                    let data = self.memory.read_byte(memory_location);
                    self.set_register_pair(register_pair, memory_location.wrapping_add(1));
                    match load.target {
                        Operand::Register(register) => {
                            self.set_register(register, data);
                            2
                        }
                        _ => panic!("not implemented"),
                    }
                }
                MemoryLocation::HLmin => {
                    let register_pair = RegisterPair(Register::H, Register::L);
                    let memory_location = self.get_register_pair(register_pair);
                    let data = self.memory.read_byte(memory_location);
                    self.set_register_pair(register_pair, memory_location.wrapping_sub(1));
                    match load.target {
                        Operand::Register(register) => {
                            self.set_register(register, data);
                            2
                        }
                        _ => panic!("not implemented"),
                    }
                }
                _ => panic!("not implemented"),
            },
            Operand::RegisterPair(source) => match load.target {
                Operand::RegisterPair(target) => {
                    self.set_register_pair(target, self.get_register_pair(source));
                    2
                }
                Operand::StackPointer => {
                    self.sp = self.get_register_pair(source);
                    2
                }
                _ => panic!("not implemented"),
            },
            Operand::ImmediateOperand(ImmediateOperand::D8(operand)) => match load.target {
                Operand::Register(target) => {
                    self.set_register(target, operand);
                    2
                }
                Operand::MemoryLocation(MemoryLocation::RegisterPair(register_pair)) => {
                    self.memory
                        .write_byte(self.get_register_pair(register_pair), operand);
                    3
                }
                _ => {
                    println!();
                    panic!("not implemented")
                }
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
                Operand::StackPointer => {
                    self.sp = operand;
                    3
                }
                _ => {
                    println!();
                    panic!("not implemented")
                }
            },
            Operand::StackPointer => match load.target {
                Operand::ImmediateOperand(ImmediateOperand::A16(operand)) => {
                    self.memory.write_word(operand, self.sp);
                    5
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

    fn push(&mut self, source: RegisterPair) -> usize {
        self.push_word(self.get_register_pair(source));
        4
    }

    fn pop(&mut self, target: RegisterPair) -> usize {
        let data = self.pop_word();
        self.set_register_pair(target, data);
        3
    }

    fn push_word(&mut self, word: u16) {
        self.sp -= 1;
        self.memory
            .write_byte(self.sp, ((word & 0xFF00) >> 8) as u8);

        self.sp -= 1;
        self.memory.write_byte(self.sp, (word & 0xFF) as u8);
    }

    fn pop_word(&mut self) -> u16 {
        let data_low = self.memory.read_byte(self.sp) as u16;
        self.sp += 1;
        let data_high = self.memory.read_byte(self.sp) as u16;
        self.sp += 1;

        data_low | (data_high << 8)
    }

    /** add 1 to byte, does not set carry flag */
    fn increment(&mut self, operand: Operand) -> usize {
        let mut cycles = 1;
        match operand {
            Operand::Register(register) => {
                let byte = self.get_register(register);
                let result = byte.wrapping_add(1);
                self.set_register(register, result);

                self.flags.z = result == 0;
                self.flags.n = false;
                self.flags.h = (byte & 0xF) == 0xF;
            }
            Operand::RegisterPair(pair) => {
                let word = self.get_register_pair(pair).wrapping_add(1);
                self.set_register_pair(pair, word);
                cycles = 2;
            }
            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    let pos = self.get_register_pair(pair);
                    let byte = self.memory.read_byte(pos).wrapping_add(1);
                    self.memory.write_byte(pos, byte);

                    self.flags.z = byte == 0;
                    self.flags.n = false;
                    self.flags.h = (byte & 0xF) == 0xF;

                    cycles = 3;
                }
                _ => panic!("not implemented"),
            },
            Operand::ImmediateOperand(_) => panic!("not implemented"),
            Operand::StackPointer => {
                self.sp = self.sp.wrapping_add(1);
                cycles = 2;
            }
        }

        cycles
    }

    /** subtract 1 from byte, does not set carry flag */
    fn decrement(&mut self, operand: Operand) -> usize {
        let mut cycles = 1;
        match operand {
            Operand::Register(register) => {
                let byte = self.get_register(register).wrapping_sub(1);
                self.set_register(register, byte);

                self.flags.z = byte == 0;
                self.flags.n = true;
                self.flags.h = (byte & 0xF) == 0xF;
            }
            Operand::RegisterPair(pair) => {
                let word = self.get_register_pair(pair).wrapping_sub(1);
                self.set_register_pair(pair, word);
                cycles = 2;
            }
            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    let byte = self
                        .memory
                        .read_byte(self.get_register_pair(pair))
                        .wrapping_sub(1);

                    self.memory.write_byte(self.get_register_pair(pair), byte);
                    self.flags.z = byte == 0;
                    self.flags.n = true;
                    self.flags.h = (byte & 0xF) == 0xF;
                    cycles = 3;
                }
                _ => panic!("not implemented"),
            },
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
            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    (self.memory.read_byte(self.get_register_pair(pair)), 2)
                }
                _ => panic!("Should not happen"),
            },
            Operand::RegisterPair(_) => panic!("Should not happen"),
            Operand::ImmediateOperand(immediate_operand) => match immediate_operand {
                ImmediateOperand::D8(operand) => (operand, 2),
                _ => panic!("Should not happen"),
            },
            Operand::StackPointer => panic!("not implemented"),
        };

        self.a ^= source_data;
        self.flags.z = self.a == 0;
        self.flags.n = false;
        self.flags.h = false;
        self.flags.c = false;

        cycles
    }

    fn or_a(&mut self, source: Operand) -> usize {
        let (source_data, cycles) = match source {
            Operand::Register(source) => (self.get_register(source), 1),
            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    (self.memory.read_byte(self.get_register_pair(pair)), 2)
                }
                _ => panic!("not implemented"),
            }, // 2 cycles
            Operand::RegisterPair(_) => panic!("Should not happen"),
            Operand::ImmediateOperand(_) => panic!("not implemented"),
            Operand::StackPointer => panic!("not implemented"),
        };

        self.a |= source_data;
        self.flags.z = self.a == 0;
        self.flags.n = false;
        self.flags.h = false;
        self.flags.c = false;

        cycles
    }

    fn and(&mut self, source: Operand) -> usize {
        let (source_data, cycles) = match source {
            Operand::Register(source) => (self.get_register(source), 1),
            Operand::MemoryLocation(location) => match location {
                MemoryLocation::RegisterPair(pair) => {
                    (self.memory.read_byte(self.get_register_pair(pair)), 2)
                }
                _ => panic!("should not happend"),
            },
            Operand::RegisterPair(_) => panic!("Should not happen"),
            Operand::ImmediateOperand(operand) => match operand {
                ImmediateOperand::D8(operand) => (operand, 2),
                _ => panic!("should not happend"),
            },
            Operand::StackPointer => panic!("not implemented"),
        };

        self.a &= source_data;
        self.flags.z = self.a == 0;
        self.flags.n = false;
        self.flags.h = true;
        self.flags.c = false;

        cycles
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

    fn sub(&mut self, operand: Operand) -> usize {
        let mut cycles = 1;
        let byte = match operand {
            Operand::Register(register) => self.get_register(register),
            Operand::MemoryLocation(memory_location) => match memory_location {
                MemoryLocation::RegisterPair(pair) => {
                    cycles = 2;
                    let location = self.get_register_pair(pair);
                    self.memory.read_byte(location)
                }
                _ => panic!("should not happen"),
            },
            Operand::ImmediateOperand(operand) => match operand {
                ImmediateOperand::D8(byte) => {
                    cycles = 2;
                    byte
                }
                _ => panic!("should not happen"),
            },
            _ => panic!("not implemented"),
        };

        let res = self.a.wrapping_sub(byte);

        self.flags.z = res == 0x0;
        self.flags.n = true;

        self.flags.c = self.a < byte;
        self.flags.h = (self.a & 0x0F) < (byte & 0x0F);
        self.a = res;

        cycles
    }

    fn sbc(&mut self, operand: Operand) -> usize {
        let mut cycles = 1;
        let byte = match operand {
            Operand::Register(register) => self.get_register(register),
            Operand::MemoryLocation(memory_location) => match memory_location {
                MemoryLocation::RegisterPair(pair) => {
                    cycles = 2;
                    let location = self.get_register_pair(pair);
                    self.memory.read_byte(location)
                }
                _ => panic!("should not happen"),
            },
            Operand::ImmediateOperand(operand) => match operand {
                ImmediateOperand::D8(byte) => {
                    cycles = 2;
                    byte
                }
                _ => panic!("should not happen"),
            },
            _ => panic!("not implemented"),
        };

        let carry = self.flags.c.as_u8();
        let res = self.a.wrapping_sub(byte + carry);

        self.flags.z = res == 0x0;
        self.flags.n = true;

        self.flags.c = (self.a as u16) < (byte as u16 + carry as u16);
        self.flags.h = (self.a & 0x0F) < ((byte & 0x0F) + carry);
        self.a = res;

        cycles
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
    fn add(&mut self, add: Add) -> usize {
        let mut cycles = 1;
        let result = match add.source {
            Operand::ImmediateOperand(source_operand) => match source_operand {
                ImmediateOperand::D8(source_data) => match add.target {
                    Operand::Register(target_register) => {
                        cycles = 2;
                        let current_register = self.get_register(target_register);
                        let result = current_register as u32 + source_data as u32;
                        self.flags.h =
                            ((current_register & 0x0F) + (source_data & 0x0F)) & 0x10 == 0x10;
                        self.flags.c = result > 0xFF;
                        self.flags.z = (result & 0xFF) == 0;
                        result
                    }
                    _ => panic!("not implemented"),
                },
                _ => panic!("not implemented"),
            },
            Operand::Register(source_register) => match add.target {
                Operand::Register(target_register) => {
                    let current_register = self.get_register(target_register);
                    let source_data = self.get_register(source_register);
                    let result = current_register as u32 + source_data as u32;
                    self.flags.h =
                        ((current_register & 0x0F) + (source_data & 0x0F)) & 0x10 == 0x10;
                    self.flags.c = result > 0xFF;
                    self.flags.z = (result & 0xFF) == 0;
                    result
                }
                _ => panic!("not implemented"),
            },
            Operand::MemoryLocation(memory_location) => match memory_location {
                MemoryLocation::RegisterPair(source_pair) => match add.target {
                    Operand::Register(target_register) => {
                        cycles = 2;
                        let current_register = self.get_register(target_register);
                        let source_data =
                            self.memory.read_byte(self.get_register_pair(source_pair));
                        let result = current_register as u32 + source_data as u32;
                        self.flags.h =
                            ((current_register & 0x0F) + (source_data & 0x0F)) & 0x10 == 0x10;
                        self.flags.c = result > 0xFF;
                        self.flags.z = (result & 0xFF) == 0;
                        result
                    }
                    _ => panic!("not implemented"),
                },
                _ => panic!("not implemented"),
            },
            Operand::RegisterPair(source_pair) => match add.target {
                Operand::RegisterPair(target_pair) => {
                    cycles = 2;
                    let current_data = self.get_register_pair(target_pair) as u32;
                    let source_data = self.get_register_pair(source_pair) as u32;
                    let result = current_data.wrapping_add(source_data);

                    self.flags.h =
                        ((current_data & 0x0FFF) + (source_data & 0x0FFF)) & 0x01000 == 0x01000;
                    self.flags.c = result > 0xFFFF;

                    result
                }
                _ => panic!("not implemented"),
            },
            _ => {
                event!(Level::INFO, "{:#04X}", self.pc);
                self.debug_all();
                panic!("not implemented")
            }
        };

        match add.target {
            Operand::Register(target_register) => {
                self.set_register(target_register, (result & 0xFF) as u8);
            }
            Operand::RegisterPair(target_pair) => {
                self.set_register_pair(target_pair, (result & 0xFFFF) as u16);
            }
            _ => panic!("should not happen"),
        }

        self.flags.n = false;

        cycles
    }

    fn adc(&mut self, operand: Operand) -> usize {
        let mut cycles = 1;
        let result = match operand {
            Operand::ImmediateOperand(source_operand) => match source_operand {
                ImmediateOperand::D8(source_data) => {
                    cycles = 2;
                    let current_register = self.a;
                    let result = current_register as u16 + source_data as u16 + self.flags.c as u16;
                    self.flags.h =
                        ((current_register & 0x0F) + (source_data & 0x0F) + self.flags.c as u8)
                            & 0x10
                            == 0x10;
                    result
                }
                _ => panic!("not implemented"),
            },
            Operand::Register(source_register) => {
                let current_register = self.a;
                let source_data = self.get_register(source_register);
                let result = current_register as u16 + source_data as u16 + self.flags.c as u16;
                self.flags.h =
                    ((current_register & 0x0F) + (source_data & 0x0F) + self.flags.c as u8) & 0x10
                        == 0x10;
                result
            }
            Operand::MemoryLocation(memory_location) => match memory_location {
                MemoryLocation::RegisterPair(source_pair) => {
                    cycles = 2;

                    let current_register = self.a;
                    let source_data = self.memory.read_byte(self.get_register_pair(source_pair));
                    let result = current_register as u16 + source_data as u16 + self.flags.c as u16;
                    self.flags.h =
                        ((current_register & 0x0F) + (source_data & 0x0F) + self.flags.c as u8)
                            & 0x10
                            == 0x10;
                    result
                }
                _ => panic!("not implemented"),
            },
            _ => {
                event!(Level::INFO, "{:#04X}", self.pc);
                self.debug_all();
                panic!("not implemented")
            }
        };

        self.flags.c = result > 0xFF;

        self.a = (result & 0xFF) as u8;

        self.flags.z = (result & 0xFF) == 0;
        self.flags.n = false;

        cycles
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
    fn rra(&mut self) -> usize {
        self.flags.h = false;
        self.flags.z = false;
        self.flags.n = false;

        let mut byte = self.a;
        let carry = self.flags.c;
        self.flags.c = (byte & 0x1) == 0x1;
        byte >>= 1;
        if carry {
            byte |= 0x80;
        }
        self.a = byte;

        1
    }

    /**
     * Shift A left, placing the leftmost bit (bit 7) in the carry flag and bit 0 of A.
     */
    fn rlca(&mut self) -> usize {
        let mut byte = self.a;
        self.flags.c = (byte & 0xF) == 0x1;
        byte <<= 1;
        if self.flags.c {
            byte |= 0x01;
        }
        self.a = byte;

        1
    }

    fn jump(&mut self, operand: Operand, condition: Option<Condition>) -> usize {
        let mut cycles = 4;

        let operand = match operand {
            Operand::RegisterPair(pair) => {
                cycles = 1;
                self.get_register_pair(pair)
            }
            Operand::ImmediateOperand(ImmediateOperand::A16(operand)) => operand,
            _ => panic!("should not happend"),
        };

        if let Some(condition) = condition {
            match condition {
                Condition::Z => {
                    if !self.flags.z {
                        return 3;
                    }
                }
                Condition::C => {
                    if !self.flags.c {
                        return 3;
                    }
                }
                Condition::NZ => {
                    if self.flags.z {
                        return 3;
                    }
                }
                Condition::NC => {
                    if self.flags.c {
                        return 3;
                    }
                }
            }
        }

        self.pc = operand;

        cycles
    }

    fn jump_relative(&mut self, operand: ImmediateOperand, condition: Option<Condition>) -> usize {
        let ImmediateOperand::S8(operand) = operand else {
            panic!("Invalid operand for JR");
        };

        if let Some(condition) = condition {
            match condition {
                Condition::Z => {
                    if !(self.flags.z) {
                        return 2;
                    }
                }
                Condition::C => {
                    if !(self.flags.c) {
                        return 2;
                    }
                }
                Condition::NZ => {
                    if self.flags.z {
                        return 2;
                    }
                }
                Condition::NC => {
                    if self.flags.c {
                        return 2;
                    }
                }
            }
        }

        self.pc = self.pc.wrapping_add_signed(operand as i16);

        3
    }

    fn debug_registers(&self) {
        event!(Level::ERROR, "--------- REGISTERS -----------");
        event!(Level::ERROR, "Program counter: {:#08X}", self.pc);
        event!(Level::ERROR, "Stack pointer: {:#04X}", self.sp);
        event!(Level::ERROR, "A: {:#04X}", self.a);
        event!(Level::ERROR, "BC: {:#04X} {:#04X}", self.b, self.c);
        event!(Level::ERROR, "DE: {:#04X} {:#04X}", self.d, self.e);
        event!(Level::ERROR, "HL: {:#04X} {:#04X}", self.h, self.l);
        event!(Level::ERROR, "Flags: {:?}", self.flags);
    }

    fn debug_flags(&self) {
        event!(Level::ERROR, "--------- FLAGS -----------");
        event!(
            Level::ERROR,
            "Interrupts enabled: {:?}",
            self.interrupts_enabled
        );
        event!(
            Level::ERROR,
            "Interrupt enable flags: {:?}",
            self.memory.interrupt_enable
        );
        event!(
            Level::ERROR,
            "Interrupt flags: {:?}",
            self.memory.interrupt_flags
        );
    }

    fn debug_video(&self) {
        event!(Level::ERROR, "--------- VIDEO -----------");
        event!(Level::ERROR, "Video mode: {:?}", self.memory.video.mode);
        event!(
            Level::ERROR,
            "Video lcd control: {:?}",
            self.memory.video.lcd_control
        );
        event!(
            Level::ERROR,
            "Video lcd status: {:?}",
            self.memory.video.lcd_status
        );
    }

    fn debug_stack(&mut self) {
        event!(Level::ERROR, "--- STACK START ---");
        for i in self.sp..=STACK_START {
            event!(Level::ERROR, "{:#04X}", self.memory.read_byte(i));
        }
        event!(Level::ERROR, "--- STACK END ---");
    }

    fn debug_current_instruction(&self) {
        event!(Level::ERROR, "--- CURRENT INSTRUCTION ---");
        event!(
            Level::ERROR,
            "Current opcode: {:#04X}",
            self.current_opcode.unwrap(),
        );
        event!(
            Level::ERROR,
            "Current instruction: {}",
            self.current_instruction.unwrap(),
        );
    }

    fn debug_all(&mut self) {
        self.debug_registers();
        self.debug_flags();
        self.debug_video();
        self.debug_current_instruction();
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
            Register::F => self.flags.to_bits(),
        }
    }

    fn get_register_pair(&self, reg_pair: RegisterPair) -> u16 {
        ((self.get_register(reg_pair.0) as u16) << 8) | (self.get_register(reg_pair.1) as u16)
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
            Register::F => self.flags.set_bits(data),
        };
    }

    fn set_register_pair(&mut self, reg_pair: RegisterPair, data: u16) {
        self.set_register(reg_pair.0, (data >> 8) as u8);
        self.set_register(reg_pair.1, (data & 0x00FF) as u8);
    }

    fn log_doctor(&mut self) {
        // gameboy doctor
        event!(
            Level::TRACE,
            "A:{:02X} F:{:02X} B:{:02X} C:{:02X} D:{:02X} E:{:02X} H:{:02X} L:{:02X} SP:{:04X} PC:{:04X} PCMEM:{:02X},{:02X},{:02X},{:02X}",
            self.a,
            self.flags.to_bits(),
            self.b,
            self.c,
            self.d,
            self.e,
            self.h,
            self.l,
            self.sp,
            self.pc,
            self.memory.read_byte(self.pc),
            self.memory.read_byte(self.pc+1),
            self.memory.read_byte(self.pc+2),
            self.memory.read_byte(self.pc+3),
        );
    }
}

#[cfg(test)]
mod tests {
    use crate::cartridge::CartridgeHeader;
    use crate::cpu::{CpuState, STACK_START};
    use crate::{Cartridge, Cpu};

    #[test]
    fn it_executes_call() {
        let cartridge = Cartridge {
            header: CartridgeHeader {
                title: "test".to_string(),
                ..Default::default()
            },
            data: vec![0xCD, 0x03, 0x00],
        };

        let mut cpu = Cpu::load_cartridge(cartridge);
        cpu.state = CpuState::Running;
        cpu.pc = 0;
        cpu.cycle(&None);

        assert_eq!(cpu.pc, 0x03);
        assert_eq!(cpu.sp, STACK_START - 2);
    }

    #[test]
    fn it_executes_ret() {
        let cartridge = Cartridge {
            header: CartridgeHeader {
                title: "test".to_string(),
                ..Default::default()
            },
            data: vec![0xCD, 0x04, 0x00, 0x00, 0xC9],
        };

        let mut cpu = Cpu::load_cartridge(cartridge);
        cpu.state = CpuState::Running;
        cpu.pc = 0;
        cpu.cycle(&None);
        cpu.cycle(&None);

        assert_eq!(cpu.pc, 0x03);
        assert_eq!(cpu.sp, STACK_START);
    }
}
