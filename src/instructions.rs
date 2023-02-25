use std::fmt::{Display, Formatter};

pub fn decode(data: &[u8], pc: u16) -> (Instruction, u16) {
    let opcode = data[pc as usize];

    match opcode {
        0x00 => (Instruction::NOP, pc),
        0x76 => (Instruction::HALT, pc),
        // decode LD
        0x40..=0x80 => {
            let target = match (opcode & 0x38) >> 3 {
                0x0 => InstructionTarget::RegisterTarget(RegisterTarget::B),
                0x1 => InstructionTarget::RegisterTarget(RegisterTarget::C),
                0x2 => InstructionTarget::RegisterTarget(RegisterTarget::D),
                0x3 => InstructionTarget::RegisterTarget(RegisterTarget::E),
                0x4 => InstructionTarget::RegisterTarget(RegisterTarget::H),
                0x5 => InstructionTarget::RegisterTarget(RegisterTarget::L),
                0x6 => InstructionTarget::MemoryTarget(MemoryTarget::HL),
                0x7 => InstructionTarget::RegisterTarget(RegisterTarget::A),
                _ => panic!("Unknown opcode {:#04X}", opcode),
            };

            let source = match opcode & 0x07 {
                0x0 => InstructionTarget::RegisterTarget(RegisterTarget::B),
                0x1 => InstructionTarget::RegisterTarget(RegisterTarget::C),
                0x2 => InstructionTarget::RegisterTarget(RegisterTarget::D),
                0x3 => InstructionTarget::RegisterTarget(RegisterTarget::E),
                0x4 => InstructionTarget::RegisterTarget(RegisterTarget::H),
                0x5 => InstructionTarget::RegisterTarget(RegisterTarget::L),
                0x6 => InstructionTarget::MemoryTarget(MemoryTarget::HL),
                0x7 => InstructionTarget::RegisterTarget(RegisterTarget::A),
                _ => panic!("Unknown opcode {:#04X}", opcode),
            };

            (Instruction::LD(Load { target, source }), pc)
        }
        0xA8..=0xAF => {
            let source = match opcode & 0x07 {
                0x0 => InstructionTarget::RegisterTarget(RegisterTarget::B),
                0x1 => InstructionTarget::RegisterTarget(RegisterTarget::C),
                0x2 => InstructionTarget::RegisterTarget(RegisterTarget::D),
                0x3 => InstructionTarget::RegisterTarget(RegisterTarget::E),
                0x4 => InstructionTarget::RegisterTarget(RegisterTarget::H),
                0x5 => InstructionTarget::RegisterTarget(RegisterTarget::L),
                0x6 => InstructionTarget::MemoryTarget(MemoryTarget::HL),
                0x7 => InstructionTarget::RegisterTarget(RegisterTarget::A),
                _ => panic!("Unknown opcode {:#04X}", opcode),
            };

            (Instruction::XOR(source), pc)
        }
        0xC3 => {
            let operand = u16::from_le_bytes([data[pc as usize +1], data[pc as usize+2]]);
            (Instruction::JP(Operand::A16(operand)), pc+2)
        },
        _ => panic!("Unknown opcode {:#04X}", opcode),
    }
}

#[derive(Debug, Eq, PartialEq)]
pub enum Instruction {
    NOP,
    HALT,
    LD(Load),
    XOR(InstructionTarget),
    JP(Operand),
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Instruction::NOP => write!(f, "NOP"),
            Instruction::HALT => write!(f, "NOP"),
            Instruction::LD(load) => write!(f, "LD {}", load),
            Instruction::XOR(target) => write!(f, "XOR {}", target),
            Instruction::JP(operand) => write!(f, "JP {}", operand),
        }
    }
}

#[derive(Debug, Eq, PartialEq)]
pub struct Load {
    pub target: InstructionTarget,
    pub source: InstructionTarget,
}

impl Display for Load {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}, {}", self.target, self.source)
    }
}


#[derive(Debug, Eq, PartialEq)]
pub enum InstructionTarget {
    RegisterTarget(RegisterTarget),
    MemoryTarget(MemoryTarget),
}

impl Display for InstructionTarget {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            InstructionTarget::RegisterTarget(target) => write!(f, "{}", target),
            InstructionTarget::MemoryTarget(target) => write!(f, "{}", target),
        }
    }
}

#[derive(Debug, Eq, PartialEq)]
pub enum RegisterTarget {
    B,
    C,
    D,
    E,
    H,
    L,
    A,
}

impl Display for RegisterTarget {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

#[derive(Debug, Eq, PartialEq)]
pub enum MemoryTarget {
    HL,
    HLplus,
    HLmin,
    BC,
    DE,
    Operand(Operand),
}

impl Display for MemoryTarget {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let display_name = match self {
            MemoryTarget::HL => "HL",
            MemoryTarget::HLplus => "HL+",
            MemoryTarget::HLmin => "HL-",
            MemoryTarget::BC => "BC",
            MemoryTarget::DE => "DE",
            MemoryTarget::Operand(operand) => return write!(f, "({})", operand),
        };
        write!(f, "({})", display_name)
    }
}

#[derive(Debug, Eq, PartialEq)]
pub enum Operand {
    A16(u16),
    D16,
    S8,
    D8,
}

impl Display for Operand {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let display_name = match self {
            Operand::A16(a16) => return write!(f,"a16[{:04X}]", a16),
            Operand::S8 => "s8",
            Operand::D8 => "d8",
            Operand::D16 => "d16",
        };
        write!(f, "({})", display_name)
    }
}
