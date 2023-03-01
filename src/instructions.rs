use std::fmt::{Display, Formatter};

use crate::helpers;

pub fn decode(data: &[u8], pc: u16) -> (Instruction, u16) {
    let opcode = data[pc as usize];

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
            let operand = u16::from_le_bytes([data[pc as usize + 1], data[pc as usize + 2]]);
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
        0x05 | 0x15 | 0x25 | 0x35 | 0x0D | 0x1D | 0x2D | 0x3D => {
            let target = int_to_register((opcode + 2) >> 3).unwrap();
            (Instruction::DEC(target), 1)
        }
        0x06 | 0x16 | 0x26 | 0x36 | 0x0E | 0x1E | 0x2E | 0x3E => {
            let target = int_to_register((opcode + 1) >> 3).unwrap();
            let operand = data[pc as usize + 1];
            (
                Instruction::LD(Load {
                    source: Operand::ImmediateOperand(ImmediateOperand::D8(operand)),
                    target,
                }),
                2,
            )
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
        0x20 => {
            let operand = helpers::u8_to_i8(data[pc as usize + 1]);
            (
                Instruction::JR(ImmediateOperand::S8(operand), Some(Condition::NZ)),
                2,
            )
        }
        0x76 => (Instruction::HALT, 1),
        0x40..=0x80 => {
            let target =
                int_to_register((opcode & 0xf0) >> 4).expect(&format!("Unknown opcode {:#04X}", opcode));

            let source =
                int_to_register(opcode & 0x07).expect(&format!("Unknown opcode {:#04X}", opcode));

            (Instruction::LD(Load { target, source }), 1)
        }
        0xA8..=0xAF => {
            let source =
                int_to_register(opcode & 0x07).expect(&format!("Unknown opcode {:#04X}", opcode));

            (Instruction::XOR(source), 1)
        }
        0xB0 ..= 0xB7 => {
            let source =
                int_to_register(opcode & 0x07).expect(&format!("Unknown opcode {:#04X}", opcode));
            (Instruction::OR(source), 1)
        }
        0xC3 => {
            let operand = u16::from_le_bytes([data[pc as usize + 1], data[pc as usize + 2]]);
            (Instruction::JP(ImmediateOperand::A16(operand)), 3)
        }
        0xCD => {
            let operand = u16::from_le_bytes([data[pc as usize + 1], data[pc as usize + 2]]);
            (Instruction::Call(ImmediateOperand::A16(operand), None), 3)
        }
        0xE0 => {
            let operand = data[pc as usize + 1];
            (
                Instruction::LD(Load {
                    target: Operand::MemoryLocation(MemoryLocation::ImmediateOperand(
                        ImmediateOperand::A8(operand),
                    )),
                    source: Operand::Register(Register::A),
                }),
                2,
            )
        }
        0xEA => {
            let operand = u16::from_le_bytes([data[pc as usize + 1], data[pc as usize + 2]]);
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
        0xE2 => (
            Instruction::LD(Load {
                target: Operand::MemoryLocation(MemoryLocation::Register(Register::C)),
                source: Operand::Register(Register::A),
            }),
            2,
        ),
        0xF2 => (
            Instruction::LD(Load {
                target: Operand::Register(Register::A),
                source: Operand::MemoryLocation(MemoryLocation::Register(Register::C)),
            }),
            2,
        ),
        0xF0 => {
            let operand = data[pc as usize + 1];
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
        0xFA => {
            let operand = u16::from_le_bytes([data[pc as usize + 1], data[pc as usize + 2]]);
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
        0xF3 => (Instruction::DI, 1),
        0xFE => {
            let operand = data[pc as usize + 1];
            (
                Instruction::CP(Operand::ImmediateOperand(ImmediateOperand::D8(operand))),
                2,
            )
        }
        _ => panic!("Unknown opcode {:#04X}", opcode),
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
    DEC(Operand),
    LD(Load),
    XOR(Operand),
    OR(Operand),
    JP(ImmediateOperand),
    JR(ImmediateOperand, Option<Condition>),
    DI,
    CP(Operand),
    Call(ImmediateOperand, Option<Condition>),
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Instruction::NOP => write!(f, "NOP"),
            Instruction::HALT => write!(f, "HALT"),
            Instruction::DEC(operand) => write!(f, "DEC {}", operand),
            Instruction::LD(load) => write!(f, "LD {}", load),
            Instruction::XOR(target) => write!(f, "XOR {}", target),
            Instruction::OR(target) => write!(f, "OR {}", target),
            Instruction::JP(operand) => write!(f, "JP {}", operand),
            Instruction::JR(operand, condition) => {
                if let Some(condition) = condition {
                    write!(f, "JR {} {}", condition, operand)
                } else {
                    write!(f, "JR {}", operand)
                }
            }
            Instruction::DI => write!(f, "DI"),
            Instruction::CP(operand) => write!(f, "CP {}", operand),
            Instruction::Call(operand, condition) => {
                if let Some(condition) = condition {
                    write!(f, "CALL {} {}", condition, operand)
                } else {
                    write!(f, "CALL {}", operand)
                }
            }
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
