use bitflags::bitflags;
use core::fmt;
use log::{debug, trace};
use yaxpeax_arch::{Decoder, ReadError, Reader};
use yaxpeax_sm83::{InstDecoder, Instruction, Operand};

// afaict LR35902/ Sharp SM83 only has one bank of gprs
bitflags! {
    #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
    pub struct Flags: u8 {
        /// set when result is zero
        const Z = 1 << 7;
        /// set after subtraction ops
        const N = 1 << 6;
        /// half-carry (did [3:0] carry to [7:4]?)
        const H = 1 << 5;
        /// carry
        const CY = 1 << 4;
    }
}

impl Flags {
    /// returns true if the [`Condition`] is fulfilled by this set of flags.
    fn check_condition(&self, cc: Condition) -> bool {
        match cc {
            Condition::NZ => !self.contains(Self::Z),
            Condition::Z => self.contains(Self::Z),
            Condition::NC => !self.contains(Self::CY),
            Condition::C => self.contains(Self::CY),
        }
    }
}

/// Condition codes for control flow (jumps, calls, etc.)
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
        writeln!(f, "a: {:#04x}  f: {:#04x}", self.a, self.f)?;
        writeln!(f, "b: {:#04x}  c: {:#04x}", self.b, self.c)?;
        writeln!(f, "d: {:#04x}  e: {:#04x}", self.d, self.e)?;
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

    pub fn pair_by_operand(&self, operand: &Operand) -> Option<u16> {
        match operand {
            Operand::AF => Some(self.af()),
            Operand::BC => Some(self.bc()),
            Operand::DE => Some(self.de()),
            Operand::HL => Some(self.hl()),
            _ => None,
        }
    }

    pub fn set_pair_by_operand(&mut self, operand: &Operand, val: u16) {
        match operand {
            Operand::AF => self.set_af(val),
            Operand::BC => self.set_bc(val),
            Operand::DE => self.set_de(val),
            Operand::HL => self.set_hl(val),
            _ => unreachable!(),
        }
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

trait OperandExt {
    fn is_imm8(&self) -> bool;
    fn is_imm16(&self) -> bool;
    fn is_reg8(&self) -> bool;
    fn is_reg16(&self) -> bool;
    fn is_indirect(&self) -> bool;
    fn as_condition(&self) -> Option<Condition>;
}

impl OperandExt for Operand {
    fn is_imm8(&self) -> bool {
        matches!(self, Self::D8(_) | Self::DerefHighD8(_))
    }

    fn is_imm16(&self) -> bool {
        matches!(self, Self::D16(_) | Self::A16(_))
    }

    fn is_reg8(&self) -> bool {
        matches!(
            self,
            Self::A | Self::B | Self::C | Self::D | Self::E | Self::H | Self::L
        )
    }

    fn is_reg16(&self) -> bool {
        matches!(self, Self::AF | Self::BC | Self::DE | Self::HL)
    }

    fn is_indirect(&self) -> bool {
        matches!(
            self,
            Self::DerefBC | Self::DerefDE | Self::DerefHL | Self::DerefDecHL | Self::DerefIncHL
        )
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

/*
 -- SoC registers --
 reg  addr    d7   | d6   |  d5   |  d4   |  d3   |  d2   |  d1    | d0     | dir      comment
 P1   FF00     \   |  \   |  P15  |  P14  |  P13  |  P12  |  P11   | P10    | R/W      "control of transfer data by P14, P15"
 SB   FF01         |      |       |       |       |       |        |        | R/W      transfer data
 SC   FF02    txs  |  \   |  \    |  \    |  \    |  \    |  cksp  | cksh   | R/W      tx speed, clock speed, shift clock {ext, int}
 DIV  FF04    64hz |  ... | ..... | ..... | ..... | ..... | ...... | 8192hz | R/W      clock divider
 TIMA FF05         |      |       |       |       |       |        |        | R/W      timer
 TMA  FF06         |      |       |       |       |       |        |        | R/W      timer preset
 TAC  FF07         |      |       |       |       |       |  stop  | freq   | R/W      timer ctl
 IF   FF0F         |      |       |       |       |       |        |        | R/W      interrupt req
 IF   FFFF         |      |       |       |       |       |        |        | R/W      interrupt enable
 IME          ---- | ---- | --- 1 |  bit  | cpu r | eg -- | ------ | -----  | R/W*     global int enable; DI/EI
 LCDC FF40    x    | x    | x     |  x    | x     | x     | x      | x      | R/W      LCD controller
 STAT FF41    \    | x    | x     |  x    | x     | x     | x      | x      | R/(W3-5) LCD controller status
 SCY  FF42    .... | .... | ..... | ..... | ..... | ..... | ...... | ...... | R/W      scroll Y reg
 SCX  FF43    .... | .... | ..... | ..... | ..... | ..... | ...... | ...... | R/W      scroll X reg
 LY   FF44    .... | .... | ..... | ..... | ..... | ..... | ...... | ...... | R        y-coord during displ
 LYC  FF44    .... | .... | ..... | ..... | ..... | ..... | ...... | ...... | R/W      LY compare reg
 DMA  FF46    .... | .... | ..... | ..... | ..... | ..... | ...... | ...... | W        W initiates txn. $00--$DF
 BGP  FF47    .... | .... | ..... | ..... | ..... | ..... | ...... | ...... | W        bg palette data

 TODO: continue

 -- register descriptions --
 IF ::::::::
   d7 d6 d5   d4                       d3                d2          d1               d0
   \  \  \  | terminals P10-P13 HIGH | end of serial tx | timer ovf | LCDC ctl STAT | vblank

 IE ::::::::
   ibid

i dont wanna be stuck in your passenger seat ! just let me out ! i gotta cry a while and i cant tell you whyyyyyy
*/

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

    pub fn mem_write(&mut self, addr: u16, val: u8) {
        debug!("memory: write {:#06x} <- {:#04x}", addr, val);
        self.memory[addr as usize] = val;
    }
    pub fn mem_read(&self, addr: u16) -> u8 {
        let val = self.memory[addr as usize];
        debug!("memory: read {:#06x} -> {:#04x}", addr, val);
        val
    }

    pub fn instr_read(&self, addr: u16) -> u8 {
        self.mem_read(addr)
    }

    /// Unpack yaxpeax operand array, figure out which is the offset/addr and which is the condition (if present),
    /// and return `(taken, offset_or_addr_operand)`.
    fn compute_branch(&mut self, operands: &[Operand; 2]) -> (bool, Operand) {
        if let Some(cc) = operands[0].as_condition() {
            let o_offs = operands[1];
            let taken = self.regs.f.check_condition(cc);

            (taken, o_offs)
        } else {
            let o_offs = operands[0];
            (true, o_offs)
        }
    }

    fn stack_push(&mut self, val: u16) {
        self.sp -= 2;
        //  /new sp /old sp
        // v       v
        // msb lsb ...
        self.mem_write(self.sp, ((0xff00 & val) >> 8) as u8); // msb
        self.mem_write(self.sp + 1, (0xff & val) as u8); // lsb
    }

    fn stack_pop(&mut self) -> u16 {
        let val = ((self.mem_read(self.sp) as u16) << 8) | (self.mem_read(self.sp + 1) as u16);
        self.sp += 2;

        val
    }

    // TODO?
    fn clear_hn(&mut self) {
        self.regs.f -= Flags::H;
        self.regs.f -= Flags::N;
    }


    // insn helpers
    /// computes dec and sets flags
    fn dec_8bit(&mut self, val: u8) -> u8 {
        let res = val.wrapping_sub(1);
        self.regs.f &= Flags::CY; // wipe everything else; preserve cy
        self.regs.f |= Flags::N;
        self.regs.f.set(Flags::Z, res == 0);
        self.regs.f.set(Flags::H, (res & 0xf) == 0xf);

        res
    }

    fn inc_8bit(&mut self, val: u8) -> u8 {
        let res = val.wrapping_add(1);
        self.regs.f &= Flags::CY; // wipe everything else; preserve cy
        self.regs.f |= Flags::N;
        self.regs.f.set(Flags::Z, res == 0);
        self.regs.f.set(Flags::H, (res & 0xf) == 0x0);

        res
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
            INC => {
                let [opr, _] = instr.operands();
                if opr.is_reg16() {
                    // TODO: simulate $ff00 inc/dec OAM bug
                    let val = self.read_src16_opr(opr);
                    self.write_reg16_opr(opr, val.wrapping_add(1));

                    2
                } else if opr.is_indirect() {
                    let addr = self.compute_addr(opr);
                    let val = self.mem_read(addr);
                    let res = self.inc_8bit(val);
                    self.mem_write(addr, res);

                    3
                } else {
                    let val = self.read_src8_opr(opr);
                    let res = self.inc_8bit(val);
                    self.write_reg8_opr(opr, res);

                    1
                }
            }
            DEC => {
                let [opr, _] = instr.operands();
                if opr.is_reg16() {
                    // TODO: simulate $ff00 inc/dec OAM bug
                    let val = self.read_src16_opr(opr);
                    self.write_reg16_opr(opr, val.wrapping_sub(1));

                    2
                } else if opr.is_indirect() {
                    let addr = self.compute_addr(opr);
                    let val = self.mem_read(addr);
                    let res = self.dec_8bit(val);
                    self.mem_write(addr, res);

                    3
                } else {
                    let val = self.read_src8_opr(opr);
                    let res = self.dec_8bit(val);
                    self.write_reg8_opr(opr, res);

                    1
                }
            }

            ADC => todo!(),
            SBC => todo!(),
            SUB => todo!(),
            AND => {
                // AND A, x
                let [operand, _] = instr.operands();
                let other = self.read_src8_opr(operand);

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
                let [operand, _] = instr.operands();
                let other = self.read_src8_opr(operand);
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
            POP => {
                let [o_dest, _] = instr.operands();
                let val = self.stack_pop();
                self.regs.set_pair_by_operand(o_dest, val);

                4
            }
            PUSH => {
                let [o_src, _] = instr.operands();
                let val = self.regs.pair_by_operand(o_src).unwrap();
                self.stack_push(val);

                4
            }
            JP => todo!(),
            JR => {
                // PC-relative jump, possibly conditional
                let (taken, o_offs) = self.compute_branch(instr.operands());
                trace!("JR: taken={}", taken);
                if taken {
                    let offs = if let Operand::R8(offs) = o_offs {
                        offs
                    } else {
                        unreachable!();
                    };

                    self.pc = self.pc.wrapping_add_signed(offs as i16);

                    4
                } else {
                    3
                }
            }
            CALL => {
                let (taken, o_addr) = self.compute_branch(instr.operands());
                trace!("CALL: taken={}", taken);
                if taken {
                    let addr = if let Operand::D16(imm) = o_addr {
                        imm
                    } else {
                        unreachable!()
                    };
                    self.stack_push(self.pc);
                    self.pc = addr;

                    6
                } else {
                    3
                }
            }
            RET => {
                if let Some(cc) = instr.operands()[0].as_condition() {
                    if self.regs.f.check_condition(cc) {
                        // taken
                        self.pc = self.stack_pop();

                        5
                    } else {
                        2
                    }
                } else {
                    self.pc = self.stack_pop();

                    4
                }
            }
            RETI => todo!(),
            HALT => todo!(),
            RST => todo!(),
            STOP => todo!(),
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
                self.clear_hn();

                1
            }
            CCF => {
                // complement carry flag; clears N, H
                self.regs.f.toggle(Flags::CY);
                self.clear_hn();

                1
            }
            DI => todo!(),
            EI => todo!(),
            LDH => {
                let [dest, src] = instr.operands();
                match dest {
                    Operand::DerefHighD8(_) | Operand::DerefHighC => {
                        // going to memory
                        let addr = self.compute_addr(dest);
                        let val = self.read_src8_opr(src);
                        self.mem_write(addr, val);
                    }
                    Operand::A => {
                        let addr = self.compute_addr(src);
                        self.regs.a = self.mem_read(addr);
                    }
                    _ => unreachable!(),
                }

                if dest.is_imm8() || src.is_imm8() {
                    3
                } else {
                    2
                }
            }
            RLC => todo!(),
            RRC => todo!(),
            RL => {
                // rotate left through carry; arbitrary register or [HL]
                let [o_x, _] = instr.operands();
                let mut x = self.read_src8_opr(o_x);
                let cy = (x & 0x80) != 0;
                x = (x << 1) | (cy as u8);
                self.regs.f.set(Flags::CY, cy);

                if o_x.is_indirect() {
                    let addr = self.compute_addr(o_x);
                    self.mem_write(addr, x);

                    4
                } else {
                    self.write_reg8_opr(o_x, x);
                    2
                }
            }
            RLA => {
                let mut x = self.regs.a;
                let cy = (x & 0x80) != 0;
                x = (x << 1) | (cy as u8);
                self.regs.f.set(Flags::CY, cy);
                self.regs.a = x;

                1
            }
            RR => todo!(),
            RLCA => todo!(),
            RRCA => todo!(),
            RRA => todo!(),
            SLA => todo!(),
            SRA => todo!(),
            SRL => todo!(),
            SWAP => todo!(),
            RES => todo!(),
            SET => todo!(),
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
        }
    }

    pub fn execute(&mut self, mut cycles: i32) {
        let decoder = InstDecoder::default();
        loop {
            trace!("run instruction at pc={:#06x}", self.pc);
            let instr = decoder.decode(self).unwrap();
            trace!("decoded: {} ({:?})", instr, instr);

            let latency = self.run_one_instr(&instr);
            trace!("completed: {} cycles", latency);

            {
                trace!("pc: {:#06x} sp: {:#06x}", self.pc, self.sp);
                trace!("regs:\n{}", self.regs);
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
                self.regs.set_hl(a.wrapping_sub(1));
                a
            }
            Operand::DerefIncHL => {
                let a = self.regs.hl();
                self.regs.set_hl(a.wrapping_add(1));
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
            let addr = self.compute_addr(operand);
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
        assert_eq!(registers.f.bits(), 0x37);
        assert_eq!(registers.b, 0xca);
        assert_eq!(registers.c, 0xfe);
        assert_eq!(registers.d, 0xb0);
        assert_eq!(registers.e, 0xfa);
        assert_eq!(registers.h, 0xd3);
        assert_eq!(registers.l, 0x35);
    }
}
