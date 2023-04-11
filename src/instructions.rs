use std::fmt::{Display, Formatter};

use crate::helpers;
use crate::memory::Memory;

pub fn decode(memory: &mut Memory, pc: u16) -> (Instruction, u16) {
    let opcode = memory.read_byte(pc);

    match opcode {
        0x00 => (Instruction::NOP, 1),
        0x01 | 0x11 | 0x21 | 0x31 => {
            let target = match opcode {
                0x01 => Operand::RegisterPair(RegisterPair(Register::B, Register::C)),
                0x11 => Operand::RegisterPair(RegisterPair(Register::D, Register::E)),
                0x21 => Operand::RegisterPair(RegisterPair(Register::H, Register::L)),
                0x31 => Operand::StackPointer,
                _ => panic!("Should not happend"),
            };
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::LD(Load {
                    source: Operand::ImmediateOperand(ImmediateOperand::D16(operand)),
                    target,
                }),
                3,
            )
        }
        0x02 | 0x12 | 0x22 | 0x32 => {
            let target = match opcode {
                0x02 => Operand::MemoryLocation(MemoryLocation::RegisterPair(RegisterPair(
                    Register::B,
                    Register::C,
                ))),
                0x12 => Operand::MemoryLocation(MemoryLocation::RegisterPair(RegisterPair(
                    Register::D,
                    Register::E,
                ))),
                0x22 => Operand::MemoryLocation(MemoryLocation::HLplus),
                0x32 => Operand::MemoryLocation(MemoryLocation::HLmin),
                _ => panic!("Should not happend"),
            };
            (
                Instruction::LD(Load {
                    source: Operand::Register(Register::A),
                    target,
                }),
                1,
            )
        }
        0x03 | 0x13 | 0x23 | 0x33 => {
            let target = match opcode {
                0x03 => Operand::RegisterPair(RegisterPair(Register::B, Register::C)),
                0x13 => Operand::RegisterPair(RegisterPair(Register::D, Register::E)),
                0x23 => Operand::RegisterPair(RegisterPair(Register::H, Register::L)),
                0x33 => Operand::StackPointer,
                _ => panic!("Should not happend"),
            };
            (Instruction::INC(target), 1)
        }
        0x04 | 0x14 | 0x24 | 0x34 | 0x0C | 0x1C | 0x2C | 0x3C => {
            let target = int_to_register((opcode + 3) >> 3).unwrap();
            (Instruction::INC(target), 1)
        }
        0x05 | 0x15 | 0x25 | 0x35 | 0x0D | 0x1D | 0x2D | 0x3D => {
            let target = int_to_register((opcode + 2) >> 3).unwrap();
            (Instruction::DEC(target), 1)
        }
        0x06 | 0x16 | 0x26 | 0x36 | 0x0E | 0x1E | 0x2E | 0x3E => {
            let target = int_to_register((opcode + 1) >> 3).unwrap();
            let operand = memory.read_byte(pc + 1);
            (
                Instruction::LD(Load {
                    source: Operand::ImmediateOperand(ImmediateOperand::D8(operand)),
                    target,
                }),
                2,
            )
        }
        0x07 => (Instruction::RLCA, 1),
        0x09 | 0x19 | 0x29 | 0x39 => {
            let target = Operand::RegisterPair(RegisterPair(Register::H, Register::L));
            let source = match opcode {
                0x09 => Operand::RegisterPair(RegisterPair(Register::B, Register::C)),
                0x19 => Operand::RegisterPair(RegisterPair(Register::D, Register::E)),
                0x29 => Operand::RegisterPair(RegisterPair(Register::H, Register::L)),
                0x39 => Operand::StackPointer,
                _ => panic!("Should not happend"),
            };

            (Instruction::LD(Load { target, source }), 1)
        }
        0x0A | 0x1A | 0x2A | 0x3A => {
            let source = match opcode {
                0x0A => Operand::MemoryLocation(MemoryLocation::RegisterPair(RegisterPair(
                    Register::B,
                    Register::C,
                ))),
                0x1A => Operand::MemoryLocation(MemoryLocation::RegisterPair(RegisterPair(
                    Register::D,
                    Register::E,
                ))),
                0x2A => Operand::MemoryLocation(MemoryLocation::HLplus),
                0x3A => Operand::MemoryLocation(MemoryLocation::HLmin),
                _ => panic!("Should not happend"),
            };
            (
                Instruction::LD(Load {
                    source,
                    target: Operand::Register(Register::A),
                }),
                1,
            )
        }
        0x0B | 0x1B | 0x2B | 0x3B => {
            let target = match opcode {
                0x0B => Operand::RegisterPair(RegisterPair(Register::B, Register::C)),
                0x1B => Operand::RegisterPair(RegisterPair(Register::D, Register::E)),
                0x2B => Operand::RegisterPair(RegisterPair(Register::H, Register::L)),
                0x3B => Operand::StackPointer,
                _ => panic!("Should not happend"),
            };
            (Instruction::DEC(target), 1)
        }
        0x18 => {
            let operand = helpers::u8_to_i8(memory.read_byte(pc + 1));
            (Instruction::JR(ImmediateOperand::S8(operand), None), 2)
        }
        0x1F => (Instruction::RRA, 1),
        0x20 => {
            let operand = helpers::u8_to_i8(memory.read_byte(pc + 1));
            (
                Instruction::JR(ImmediateOperand::S8(operand), Some(Condition::NZ)),
                2,
            )
        }
        0x27 => (Instruction::DAA, 1),
        0x28 => {
            let operand = helpers::u8_to_i8(memory.read_byte(pc + 1));
            (
                Instruction::JR(ImmediateOperand::S8(operand), Some(Condition::Z)),
                2,
            )
        }
        0x30 => {
            let operand = helpers::u8_to_i8(memory.read_byte(pc + 1));
            (
                Instruction::JR(ImmediateOperand::S8(operand), Some(Condition::NC)),
                2,
            )
        }
        0x37 => (Instruction::SCF, 1),
        0x38 => {
            let operand = helpers::u8_to_i8(memory.read_byte(pc + 1));
            (
                Instruction::JR(ImmediateOperand::S8(operand), Some(Condition::C)),
                2,
            )
        }
        0x2F => (Instruction::CPL, 1),
        0x76 => (Instruction::HALT, 1),
        0x40..=0x7F => {
            let target = int_to_register(((opcode - 0x40) & 0xf8) >> 3)
                .expect(&format!("Unknown opcode {:#04X}", opcode));

            let source = int_to_register(opcode & 0x07)
                .unwrap_or_else(|_| panic!("Unknown opcode {:#04X}", opcode));

            (Instruction::LD(Load { target, source }), 1)
        }
        0x80..=0x87 => {
            let source = int_to_register(opcode & 0x07)
                .unwrap_or_else(|_| panic!("Unknown opcode {:#04X}", opcode));

            (
                Instruction::ADD(Add {
                    source,
                    target: Operand::Register(Register::A),
                }),
                1,
            )
        }
        0x88..=0x8F => {
            let source = int_to_register(opcode & 0x07)
                .unwrap_or_else(|_| panic!("Unknown opcode {:#04X}", opcode));

            (Instruction::ADC(source), 1)
        }
        0x90..=0x97 => {
            let source = int_to_register(opcode & 0x07)
                .unwrap_or_else(|_| panic!("Unknown opcode {:#04X}", opcode));

            (Instruction::SUB(source), 1)
        }
        0x98..=0x9F => {
            let source = int_to_register(opcode & 0x07)
                .unwrap_or_else(|_| panic!("Unknown opcode {:#04X}", opcode));

            (Instruction::SBC(source), 1)
        }
        0xA0..=0xA7 => {
            let source = int_to_register(opcode & 0x07)
                .unwrap_or_else(|_| panic!("Unknown opcode {:#04X}", opcode));

            (Instruction::AND(source), 1)
        }
        0xA8..=0xAF => {
            let source = int_to_register(opcode & 0x07)
                .unwrap_or_else(|_| panic!("Unknown opcode {:#04X}", opcode));

            (Instruction::XOR(source), 1)
        }
        0xB0..=0xB7 => {
            let source = int_to_register(opcode & 0x07)
                .unwrap_or_else(|_| panic!("Unknown opcode {:#04X}", opcode));
            (Instruction::OR(source), 1)
        }
        0xB8..=0xBF => {
            let source = int_to_register(opcode & 0x07)
                .unwrap_or_else(|_| panic!("Unknown opcode {:#04X}", opcode));
            (Instruction::CP(source), 1)
        }
        0xC0 => (Instruction::RET(Some(Condition::NZ)), 1),
        0xC1 | 0xD1 | 0xE1 | 0xF1 => {
            let target = match opcode {
                0xC1 => RegisterPair(Register::B, Register::C),
                0xD1 => RegisterPair(Register::D, Register::E),
                0xE1 => RegisterPair(Register::H, Register::L),
                0xF1 => RegisterPair(Register::A, Register::F),
                _ => panic!("should not happen"),
            };

            (Instruction::POP(target), 1)
        }
        0xC2 => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::JP(
                    Operand::ImmediateOperand(ImmediateOperand::A16(operand)),
                    Some(Condition::NZ),
                ),
                3,
            )
        }
        0xC3 => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::JP(
                    Operand::ImmediateOperand(ImmediateOperand::A16(operand)),
                    None,
                ),
                3,
            )
        }
        0xC5 | 0xD5 | 0xE5 | 0xF5 => {
            let source = match opcode {
                0xC5 => RegisterPair(Register::B, Register::C),
                0xD5 => RegisterPair(Register::D, Register::E),
                0xE5 => RegisterPair(Register::H, Register::L),
                0xF5 => RegisterPair(Register::A, Register::F),
                _ => panic!("should not happen"),
            };

            (Instruction::PUSH(source), 1)
        }
        0xC7 | 0xCF | 0xD7 | 0xDF | 0xE7 | 0xEF | 0xF7 | 0xFF => {
            let number = (opcode - 0xc7) >> 3;
            (Instruction::RST(number), 1)
        }
        0xC8 => (Instruction::RET(Some(Condition::Z)), 1),
        0xC9 => (Instruction::RET(None), 1),

        0xC4 => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::Call(ImmediateOperand::A16(operand), Some(Condition::NZ)),
                3,
            )
        }
        0xC6 => (
            Instruction::ADD(Add {
                source: Operand::ImmediateOperand(ImmediateOperand::D8(memory.read_byte(pc + 1))),
                target: Operand::Register(Register::A),
            }),
            2,
        ),
        0xCA => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::JP(
                    Operand::ImmediateOperand(ImmediateOperand::A16(operand)),
                    Some(Condition::Z),
                ),
                3,
            )
        }
        0xCB => {
            let instruction = decode_cb(memory.read_byte(pc + 1));
            (Instruction::CB(instruction), 2)
        }
        0xCC => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::Call(ImmediateOperand::A16(operand), Some(Condition::Z)),
                3,
            )
        }
        0xCE => (
            Instruction::ADC(Operand::ImmediateOperand(ImmediateOperand::D8(
                memory.read_byte(pc + 1),
            ))),
            2,
        ),
        0xD2 => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::JP(
                    Operand::ImmediateOperand(ImmediateOperand::A16(operand)),
                    Some(Condition::NC),
                ),
                3,
            )
        }
        0xD4 => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::Call(ImmediateOperand::A16(operand), Some(Condition::NC)),
                3,
            )
        }
        0xD6 => (
            Instruction::SUB(Operand::ImmediateOperand(ImmediateOperand::D8(
                memory.read_byte(pc + 1),
            ))),
            2,
        ),
        0xDC => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::Call(ImmediateOperand::A16(operand), Some(Condition::C)),
                3,
            )
        }
        0xDE => (
            Instruction::SBC(Operand::ImmediateOperand(ImmediateOperand::D8(
                memory.read_byte(pc + 1),
            ))),
            2,
        ),
        0xCD => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (Instruction::Call(ImmediateOperand::A16(operand), None), 3)
        }
        0xD0 => (Instruction::RET(Some(Condition::NC)), 1),
        0xD8 => (Instruction::RET(Some(Condition::C)), 1),
        0xD9 => (Instruction::RETI, 1),
        0xE0 => (
            Instruction::LD(Load {
                target: Operand::MemoryLocation(MemoryLocation::ImmediateOperand(
                    ImmediateOperand::A8(memory.read_byte(pc + 1)),
                )),
                source: Operand::Register(Register::A),
            }),
            2,
        ),
        0xE2 => (
            Instruction::LD(Load {
                target: Operand::MemoryLocation(MemoryLocation::Register(Register::C)),
                source: Operand::Register(Register::A),
            }),
            1,
        ),
        0xE6 => (
            Instruction::AND(Operand::ImmediateOperand(ImmediateOperand::D8(
                memory.read_byte(pc + 1),
            ))),
            2,
        ),
        0xE9 => (
            Instruction::JP(
                Operand::RegisterPair(RegisterPair(Register::H, Register::L)),
                None,
            ),
            1,
        ),
        0xEA => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::LD(Load {
                    target: Operand::MemoryLocation(MemoryLocation::ImmediateOperand(
                        ImmediateOperand::A16(operand),
                    )),
                    source: Operand::Register(Register::A),
                }),
                3,
            )
        }
        0xEE => {
            let operand = memory.read_byte(pc + 1);
            (
                Instruction::XOR(Operand::ImmediateOperand(ImmediateOperand::D8(operand))),
                2,
            )
        }
        0xF0 => {
            let operand = memory.read_byte(pc + 1);
            (
                Instruction::LD(Load {
                    target: Operand::Register(Register::A),
                    source: Operand::MemoryLocation(MemoryLocation::ImmediateOperand(
                        ImmediateOperand::A8(operand),
                    )),
                }),
                2,
            )
        }
        0xF2 => (
            Instruction::LD(Load {
                target: Operand::Register(Register::A),
                source: Operand::MemoryLocation(MemoryLocation::Register(Register::C)),
            }),
            1,
        ),
        0xF3 => (Instruction::DI, 1),
        0xFA => {
            let operand = u16::from_le_bytes([memory.read_byte(pc + 1), memory.read_byte(pc + 2)]);
            (
                Instruction::LD(Load {
                    target: Operand::Register(Register::A),
                    source: Operand::MemoryLocation(MemoryLocation::ImmediateOperand(
                        ImmediateOperand::A16(operand),
                    )),
                }),
                3,
            )
        }
        0xFB => (Instruction::EI, 1),
        0xFE => {
            let operand = memory.read_byte(pc + 1);
            (
                Instruction::CP(Operand::ImmediateOperand(ImmediateOperand::D8(operand))),
                2,
            )
        }
        _ => panic!("Unknown opcode {:#04X}", opcode),
    }
}

fn decode_cb(opcode: u8) -> InstructionCB {
    match opcode {
        0x18..=0x1f => {
            let source =
                int_to_register(opcode & 0x07).expect(&format!("Unknown opcode {:#04X}", opcode));

            InstructionCB::RR(source)
        }
        0x30..=0x37 => {
            let source =
                int_to_register(opcode & 0x07).expect(&format!("Unknown opcode {:#04X}", opcode));

            InstructionCB::SWAP(source)
        }
        0x38..=0x3F => {
            let source =
                int_to_register(opcode & 0x07).expect(&format!("Unknown opcode {:#04X}", opcode));

            InstructionCB::SRL(source)
        }
        0x40..=0x7F => {
            // BIT
            // divide by 8 and take remainder of mod 8
            let bit = (opcode >> 3) & 0x7;
            let target =
                int_to_register(opcode & 0x07).expect(&format!("Unknown opcode {:#04X}", opcode));

            InstructionCB::BIT(bit, target)
        }
        _ => panic!("Unknown CB opcode 0xCB{:X}", opcode),
    }
}

fn int_to_register(int: u8) -> Result<Operand, ()> {
    match int {
        0x0 => Ok(Operand::Register(Register::B)),
        0x1 => Ok(Operand::Register(Register::C)),
        0x2 => Ok(Operand::Register(Register::D)),
        0x3 => Ok(Operand::Register(Register::E)),
        0x4 => Ok(Operand::Register(Register::H)),
        0x5 => Ok(Operand::Register(Register::L)),
        0x6 => Ok(Operand::MemoryLocation(MemoryLocation::RegisterPair(
            RegisterPair(Register::H, Register::L),
        ))),
        0x7 => Ok(Operand::Register(Register::A)),
        _ => Err(()),
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Instruction {
    NOP,
    HALT,
    CPL,
    DAA,
    SCF,
    INC(Operand),
    DEC(Operand),
    ADD(Add),
    ADC(Operand),
    SUB(Operand),
    SBC(Operand),
    LD(Load),
    XOR(Operand),
    OR(Operand),
    AND(Operand),
    JP(Operand, Option<Condition>),
    JR(ImmediateOperand, Option<Condition>),
    DI,
    CP(Operand),
    Call(ImmediateOperand, Option<Condition>),
    RET(Option<Condition>),
    RETI,
    EI,
    CB(InstructionCB),
    PUSH(RegisterPair),
    POP(RegisterPair),
    RST(u8),
    RLCA,
    RRA,
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Instruction::NOP => write!(f, "NOP"),
            Instruction::HALT => write!(f, "HALT"),
            Instruction::DAA => write!(f, "DAA"),
            Instruction::SCF => write!(f, "SCF"),
            Instruction::DEC(operand) => write!(f, "DEC {}", operand),
            Instruction::INC(operand) => write!(f, "INC {}", operand),
            Instruction::ADD(add) => write!(f, "ADD {}", add),
            Instruction::ADC(operand) => write!(f, "ADC A, {}", operand),
            Instruction::SUB(operand) => write!(f, "SUB {}", operand),
            Instruction::SBC(operand) => write!(f, "SBC {}", operand),
            Instruction::LD(load) => write!(f, "LD {}", load),
            Instruction::XOR(source) => write!(f, "XOR {}", source),
            Instruction::OR(source) => write!(f, "OR {}", source),
            Instruction::AND(source) => write!(f, "AND {}", source),
            Instruction::JP(operand, condition) => {
                if let Some(condition) = condition {
                    write!(f, "JP {}, {}", condition, operand)
                } else {
                    write!(f, "JP {}", operand)
                }
            }
            Instruction::JR(operand, condition) => {
                if let Some(condition) = condition {
                    write!(f, "JR {}, {}", condition, operand)
                } else {
                    write!(f, "JR {}", operand)
                }
            }
            Instruction::DI => write!(f, "DI"),
            Instruction::CP(operand) => write!(f, "CP {}", operand),
            Instruction::Call(operand, condition) => {
                if let Some(condition) = condition {
                    write!(f, "CALL {}, {}", condition, operand)
                } else {
                    write!(f, "CALL {}", operand)
                }
            }
            Instruction::RET(condition) => {
                if let Some(condition) = condition {
                    write!(f, "RET {}", condition)
                } else {
                    write!(f, "RET")
                }
            }
            Instruction::RETI => write!(f, "RETI"),
            Instruction::EI => write!(f, "EI"),
            Instruction::CPL => write!(f, "CPL"),
            Instruction::RLCA => write!(f, "RLCA"),
            Instruction::CB(instruction) => write!(f, "{}", instruction),
            Instruction::RST(number) => write!(f, "RST {}", number),
            Instruction::PUSH(pair) => write!(f, "PUSH {}", pair),
            Instruction::POP(pair) => write!(f, "POP {}", pair),
            Instruction::RRA => write!(f, "RRA"),
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum InstructionCB {
    SWAP(Operand),
    RR(Operand),
    SRL(Operand),
    BIT(u8, Operand),
}

impl Display for InstructionCB {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            InstructionCB::SWAP(operand) => write!(f, "SWAP {}", operand),
            InstructionCB::RR(operand) => write!(f, "RR {}", operand),
            InstructionCB::SRL(operand) => write!(f, "SRL {}", operand),
            InstructionCB::BIT(bit, operand) => write!(f, "BIT {}, {}", bit, operand),
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub struct Load {
    pub target: Operand,
    pub source: Operand,
}

impl Display for Load {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}, {}", self.target, self.source)
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub struct Add {
    pub target: Operand,
    pub source: Operand,
}

impl Display for Add {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}, {}", self.target, self.source)
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Operand {
    Register(Register),
    RegisterPair(RegisterPair),
    MemoryLocation(MemoryLocation),
    ImmediateOperand(ImmediateOperand),
    StackPointer,
}

impl Display for Operand {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Operand::Register(register) => write!(f, "{}", register),
            Operand::MemoryLocation(memory) => write!(f, "{}", memory),
            Operand::ImmediateOperand(immediate_operand) => write!(f, "{}", immediate_operand),
            Operand::RegisterPair(register_pair) => write!(f, "{}", register_pair),
            Operand::StackPointer => write!(f, "SP"),
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Register {
    B,
    C,
    D,
    E,
    H,
    L,
    A,
    F,
}

impl Display for Register {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub struct RegisterPair(pub Register, pub Register);

impl Display for RegisterPair {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}{}", self.0, self.1)
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum MemoryLocation {
    Register(Register),
    RegisterPair(RegisterPair),
    ImmediateOperand(ImmediateOperand),
    HLplus,
    HLmin,
}

impl Display for MemoryLocation {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let display_name = match self {
            MemoryLocation::HLplus => "HL+",
            MemoryLocation::HLmin => "HL-",
            MemoryLocation::ImmediateOperand(operand) => return write!(f, "({})", operand),
            MemoryLocation::RegisterPair(register_pair) => return write!(f, "({})", register_pair),
            MemoryLocation::Register(register) => return write!(f, "({})", register),
        };
        write!(f, "({})", display_name)
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum ImmediateOperand {
    A16(u16),
    D16(u16),
    S8(i8),
    D8(u8),
    A8(u8),
}

impl Display for ImmediateOperand {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        return match self {
            ImmediateOperand::A16(a16) => write!(f, "a16[{:04X}, {}]", a16, a16),
            ImmediateOperand::D16(d16) => write!(f, "d16[{:04X}, {}]", d16, d16),
            ImmediateOperand::S8(s8) => write!(f, "s8[{:02X}, {}]", s8, s8),
            ImmediateOperand::D8(d8) => write!(f, "d8[{:02X}, {}]", d8, d8),
            ImmediateOperand::A8(a8) => write!(f, "a8[{:02X}, {}]", a8, a8),
        };
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Condition {
    Z,
    C,
    NZ,
    NC,
}

impl Display for Condition {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}
