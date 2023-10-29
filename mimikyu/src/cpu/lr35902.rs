use bitflags::bitflags;
use log::debug;
use core::fmt;
use std::ops;
use yaxpeax_arch::{Decoder, ReadError, Reader};
use yaxpeax_sm83::{InstDecoder, Instruction, Operand};

// afaict LR35902/ Sharp SM83 only has one bank of gprs
bitflags! {
    #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
    pub struct Flags: u8 {
        const Z = 1 << 7;
        const N = 1 << 6;
        const H = 1 << 5;
        const CY = 1 << 4;
    }
}

impl Flags {
    fn check_condition(&self, cc: Condition) -> bool {
        match cc {
            Condition::NZ => !self.contains(Self::Z),
            Condition::Z => self.contains(Self::Z),
            Condition::NC => !self.contains(Self::CY),
            Condition::C => self.contains(Self::CY),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Condition {
    NZ,
    Z,
    NC,
    C,
}

// 7 0  7  0
//   A  |  F
//   B  |  C
//   D  |  E
//   H  |  L
// 15      0

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Registers {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub f: Flags,
    pub h: u8,
    pub l: u8,
}

impl fmt::Display for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "a: {:#04x}  f: {:#04x}\n", self.a, self.f)?;
        write!(f, "b: {:#04x}  c: {:#04x}\n", self.b, self.c)?;
        write!(f, "d: {:#04x}  e: {:#04x}\n", self.d, self.e)?;
        write!(f, "h: {:#04x}  l: {:#04x}", self.h, self.l)?;

        Ok(())
    }
}

impl Registers {
    pub fn af(&self) -> u16 {
        (self.a as u16) << 8 | (self.f.bits() as u16)
    }

    pub fn set_af(&mut self, val: u16) {
        self.a = ((val & 0xff00) >> 8) as u8;
        self.f = Flags::from_bits_retain((val & 0xff) as u8);
    }

    pub fn bc(&self) -> u16 {
        (self.b as u16) << 8 | (self.c as u16)
    }

    pub fn set_bc(&mut self, val: u16) {
        self.b = ((val & 0xff00) >> 8) as u8;
        self.c = (val & 0xff) as u8;
    }

    pub fn de(&self) -> u16 {
        (self.d as u16) << 8 | (self.e as u16)
    }

    pub fn set_de(&mut self, val: u16) {
        self.d = ((val & 0xff00) >> 8) as u8;
        self.e = (val & 0xff) as u8;
    }

    pub fn hl(&self) -> u16 {
        (self.h as u16) << 8 | (self.l as u16)
    }

    pub fn set_hl(&mut self, val: u16) {
        self.h = ((val & 0xff00) >> 8) as u8;
        self.l = (val & 0xff) as u8;
    }

    pub fn by_operand(&self, operand: &Operand) -> Option<u8> {
        match operand {
            Operand::A => Some(self.a),
            Operand::B => Some(self.b),
            Operand::C => Some(self.c),
            Operand::D => Some(self.d),
            Operand::E => Some(self.e),
            Operand::H => Some(self.h),
            Operand::L => Some(self.l),
            _ => None,
        }
    }

    pub fn by_operand_mut(&mut self, operand: &Operand) -> Option<&mut u8> {
        match operand {
            Operand::A => Some(&mut self.a),
            Operand::B => Some(&mut self.b),
            Operand::C => Some(&mut self.c),
            Operand::D => Some(&mut self.d),
            Operand::E => Some(&mut self.e),
            Operand::H => Some(&mut self.h),
            Operand::L => Some(&mut self.l),
            _ => None,
        }
    }
}

pub struct Lr35902 {
    pub pc: u16,
    pub sp: u16,
    pub regs: Registers,

    pub memory: [u8; 0xffff],

    reader_mark: u16,
}

/// hack: yaxpeax is actually extremely bad at this type of modeling.
impl Reader<u16, u8> for Lr35902 {
    fn next(&mut self) -> Result<u8, ReadError> {
        let opc = self.instr_read(self.pc);
        self.pc += 1;
        Ok(opc)
    }

    fn next_n(&mut self, buf: &mut [u8]) -> Result<(), ReadError> {
        if buf.len() > 2 {
            return Err(ReadError::ExhaustedInput);
        }

        for v in buf.iter_mut() {
            *v = self.instr_read(self.pc);
            self.pc += 1;
        }

        Ok(())
    }

    fn mark(&mut self) {
        self.reader_mark = self.pc;
    }

    fn offset(&mut self) -> u16 {
        self.pc - self.reader_mark
    }

    fn total_offset(&mut self) -> u16 {
        self.pc
    }
}

trait OperandProperties {
    fn is_imm8(&self) -> bool;
    fn is_imm16(&self) -> bool;
    fn is_reg8(&self) -> bool;
    fn is_indirect(&self) -> bool;
    fn as_condition(&self) -> Option<Condition>;
}

impl OperandProperties for Operand {
    fn is_imm8(&self) -> bool {
        match self {
            Self::D8(_) => true,
            Self::DerefHighD8(_) => true,
            _ => false,
        }
    }

    fn is_imm16(&self) -> bool {
        match self {
            Self::D16(_) => true,
            Self::A16(_) => true,
            _ => false,
        }
    }

    fn is_reg8(&self) -> bool {
        match self {
            Self::A => true,
            Self::B => true,
            Self::C => true,
            Self::D => true,
            Self::E => true,
            Self::H => true,
            Self::L => true,
            _ => false,
        }
    }

    fn is_indirect(&self) -> bool {
        match self {
            Self::DerefBC | Self::DerefDE | Self::DerefHL | Self::DerefDecHL | Self::DerefIncHL => {
                true
            }
            _ => false,
        }
    }

    fn as_condition(&self) -> Option<Condition> {
        match self {
            Operand::CondC => Some(Condition::C),
            Operand::CondNC => Some(Condition::NC),
            Operand::CondZ => Some(Condition::Z),
            Operand::CondNZ => Some(Condition::NZ),
            _ => None,
        }
    }
}

impl Lr35902 {
    pub fn new() -> Self {
        Self {
            pc: 0,
            sp: 0,
            regs: Registers::default(),
            memory: [0; 0xffff],

            reader_mark: 0,
        }
    }

    pub fn mem_write(&self, addr: u16, val: u8) {
        debug!("memory: write {:#06x} <- {:#04x}", addr, val);
    }
    pub fn mem_read(&self, addr: u16) -> u8 {
        let val = self.memory[addr as usize];
        debug!("memory: read {:#06x} -> {:#04x}", addr, val);
        val
    }

    pub fn instr_read(&self, addr: u16) -> u8 {
        return self.mem_read(addr);
    }

    pub fn run_one_instr(&mut self, instr: &Instruction) -> u32 {
        use yaxpeax_sm83::Opcode::*;
        match instr.opcode() {
            NOP => {
                // :)
                1 // latency
            }

            LD => {
                // load. there are 8 bit and 16 bit loads.
                // -- 8 bit
                //  the LHS is generally reg8, including [HL] (meaning *HL = RHS).
                //  the RHS can be a reg8, an [HL] indirect, an 8-bit immediate.
                // -- 16 bit
                //  the LHS is {BC, DE, HL, SP}
                //  the RHS is imm16, ..

                let [dest, src] = instr.operands();
                match dest {
                    Operand::A
                    | Operand::B
                    | Operand::C
                    | Operand::D
                    | Operand::E
                    | Operand::H
                    | Operand::L => {
                        // 8 bits to registers
                        let val = self.read_src8_opr(src);
                        self.write_reg8_opr(dest, val);

                        if src.is_imm8() || src.is_indirect() {
                            2
                        } else {
                            1
                        }
                    }
                    Operand::AF | Operand::BC | Operand::DE | Operand::HL | Operand::SP => {
                        // 16 bits to registers
                        let val = self.read_src16_opr(src);
                        self.write_reg16_opr(dest, val);

                        let mut cycles = 1;
                        if src.is_imm16() {
                            cycles += 2;
                        }
                        if dest.is_imm16() {
                            cycles += 2;
                        }

                        cycles
                    }
                    Operand::DerefHL
                    | Operand::DerefBC
                    | Operand::DerefDE
                    | Operand::DerefDecHL
                    | Operand::DerefIncHL => {
                        // 8 bits to memory, full 16b address
                        let val = self.read_src8_opr(src);
                        let addr = self.compute_addr(dest);
                        self.mem_write(addr, val);

                        if src.is_imm8() {
                            3
                        } else {
                            2
                        } // cy
                    }
                    Operand::A16(_) => todo!(), // 16 bits to memory
                    _ => unreachable!("bad LD dest operand"),
                }
            }
            DEC => todo!(),
            INC => todo!(),
            ADD => todo!(),
            ADC => todo!(),
            SBC => todo!(),
            SUB => todo!(),
            AND => {
                // AND A, x
                let operand = instr.operands()[0];
                let other = self.read_src8_opr(&operand);

                self.regs.a &= other;
                if self.regs.a == 0 {
                    self.regs.f = Flags::Z;
                }
                self.regs.f |= Flags::H;

                if operand.is_indirect() || operand.is_imm8() {
                    2
                } else {
                    1
                }
            }
            XOR => {
                let operand = instr.operands()[0];
                let other = self.read_src8_opr(&operand);
                self.regs.a &= other;
                self.regs.f = Flags::empty();
                if self.regs.a == 0 {
                    self.regs.f |= Flags::Z;
                }

                if operand.is_indirect() || operand.is_imm8() {
                    2
                } else {
                    1
                }
            }
            OR => todo!(),
            CP => todo!(),
            POP => todo!(),
            PUSH => todo!(),
            JP => todo!(),
            JR => {
                // PC-relative jump, possibly conditional

                let (taken, o_offs) = if let Some(cc) = instr.operands()[0].as_condition() {
                    let o_offs = instr.operands()[1];
                    let taken = self.regs.f.check_condition(cc);

                    (taken, o_offs)
                } else {
                    let o_offs = instr.operands()[0];
                    (true, o_offs)
                };

                log::trace!("branch: taken={}", taken);
                if taken {
                    let offs = if let Operand::R8(offs) = o_offs {
                        offs
                    } else { unreachable!(); };

                    self.pc = self.pc.wrapping_add_signed(offs as i16);

                    4
                } else {
                    3
                }
            },
            CALL => todo!(),
            RET => todo!(),
            RETI => todo!(),
            HALT => todo!(),
            RST => todo!(),
            STOP => todo!(),
            RLCA => todo!(),
            RRCA => todo!(),
            RLA => todo!(),
            RRA => todo!(),
            DAA => {
                // decimal adjust accumulator. annoying BCD shit
                todo!()
            }
            CPL => {
                // complement accumulator; sets N, H
                self.regs.a = !self.regs.a;
                self.regs.f |= Flags::H;
                self.regs.f |= Flags::N;

                1
            }
            SCF => {
                // set carry flag; clears N, H
                self.regs.f |= Flags::CY;
                self.regs.f -= Flags::H;
                self.regs.f -= Flags::N;

                1
            }
            CCF => {
                // complement carry flag; clears N, H
                self.regs.f.toggle(Flags::CY);
                self.regs.f -= Flags::H;
                self.regs.f -= Flags::N;

                1
            }
            DI => todo!(),
            EI => todo!(),
            LDH => todo!(),
            RLC => todo!(),
            RRC => todo!(),
            RL => todo!(),
            RR => todo!(),
            SLA => todo!(),
            SRA => todo!(),
            SWAP => todo!(),
            SRL => todo!(),
            BIT => {
                // Set Z based on bit n in an 8-bit value
                let [o_bit, o_src] = instr.operands();
                let val = self.read_src8_opr(o_src);

                let is_set = if let Operand::Bit(n) = o_bit {
                    (val & (1 << n)) != 0
                } else {
                    unreachable!();
                };

                self.regs.f.set(Flags::Z, !is_set);

                if o_src.is_indirect() {
                    3
                } else {
                    2
                }
            }
            RES => todo!(),
            SET => todo!(),
        }
    }

    pub fn execute(&mut self, mut cycles: i32) {
        let decoder = InstDecoder::default();
        loop {
            debug!("run instruction at pc={:#06x}", self.pc);
            let instr = decoder.decode(self).unwrap();
            debug!("decoded: {} ({:?})", instr, instr);

            let latency = self.run_one_instr(&instr);
            debug!("completed: {} cycles", latency);

            {
                debug!("pc: {:#06x} sp: {:#06x}", self.pc, self.sp);
                debug!("regs:\n{}", self.regs);
            }

            cycles -= latency as i32;
            if cycles < 0 {
                break;
            }
        }
    }

    fn write_reg8_opr(&mut self, dest: &Operand, val: u8) {
        let reg = self.regs.by_operand_mut(dest).unwrap();
        *reg = val;
    }

    fn write_reg16_opr(&mut self, dest: &Operand, val: u16) {
        match dest {
            Operand::AF => self.regs.set_af(val),
            Operand::BC => self.regs.set_bc(val),
            Operand::DE => self.regs.set_de(val),
            Operand::HL => self.regs.set_hl(val),
            Operand::SP => self.sp = val,
            _ => unreachable!(),
        };
    }

    fn compute_addr(&mut self, dest: &Operand) -> u16 {
        match dest {
            Operand::DerefHL => self.regs.hl(),
            Operand::DerefBC => self.regs.bc(),
            Operand::DerefDE => self.regs.de(),
            Operand::DerefDecHL => {
                let a = self.regs.hl();
                self.regs.set_hl(a - 1);
                a
            }
            Operand::DerefIncHL => {
                let a = self.regs.hl();
                self.regs.set_hl(a + 1);
                a
            }
            Operand::DerefHighC => 0xff00 | (self.regs.c as u16),
            Operand::DerefHighD8(imm) => 0xff00 | (*imm as u16),
            _ => unreachable!(),
        }
    }

    pub fn read_src8_opr(&mut self, operand: &Operand) -> u8 {
        if let &Operand::D8(imm) = operand {
            imm
        } else if operand.is_reg8() {
            self.regs.by_operand(operand).unwrap()
        } else if operand.is_indirect() {
            let addr = self.compute_addr(&operand);
            self.mem_read(addr)
        } else {
            unimplemented!()
        }
    }

    fn read_src16_opr(&self, src: &Operand) -> u16 {
        match src {
            Operand::HL => self.regs.hl(),
            Operand::SP => self.sp,
            Operand::D16(imm) => *imm,
            Operand::SPWithOffset(offs) => self.sp.wrapping_add_signed(*offs as i16), // LDHL SP, e
            _ => unimplemented!("invalid src16 opr {:?}", src),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn paired_registers() {
        let mut registers: Registers = Default::default();
        registers.set_af(0x1337);
        registers.set_bc(0xcafe);
        registers.set_de(0xb0fa);
        registers.set_hl(0xd335);

        assert_eq!(0x1337, registers.af());
        assert_eq!(0xcafe, registers.bc());
        assert_eq!(0xb0fa, registers.de());
        assert_eq!(0xd335, registers.hl());

        assert_eq!(registers.a, 0x13);
        assert_eq!(registers.f.bits, 0x37);
        assert_eq!(registers.b, 0xca);
        assert_eq!(registers.c, 0xfe);
        assert_eq!(registers.d, 0xb0);
        assert_eq!(registers.e, 0xfa);
        assert_eq!(registers.h, 0xd3);
        assert_eq!(registers.l, 0x35);
    }
}
