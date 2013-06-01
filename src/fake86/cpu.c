/*
  Fake86: A portable, open-source 8086 PC emulator.
  Copyright (C)2010-2012 Mike Chambers

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

/* cpu.c: functions to emulate the 8086/V20 CPU in software. the heart of Fake86. */

#include "config.h"
#include <stdint.h>
#include <stdio.h>
#include "cpu.h"
#include "i8259.h"
#include "i8253.h"

extern struct i8253_s i8253;

extern struct structpic i8259;
uint64_t curtimer, lasttimer, timerfreq;

uint8_t byteregtable[8] = { regal, regcl, regdl, regbl, regah, regch, regdh, regbh };

static const uint8_t parity[0x100] = {
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1
};

uint8_t	RAM[0x100000], readonly[0x100000];
uint8_t	opcode, segoverride, reptype, bootdrive = 0, hdcount = 0;
uint16_t segregs[4], savecs, saveip, ip, useseg, oldsp;
uint8_t	tempcf, oldcf, cf, pf, af, zf, sf, tf, ifl, df, of, mode, reg, rm;
uint16_t oper1, oper2, res16, disp16, temp16, dummy, stacksize, frametemp;
uint8_t	oper1b, oper2b, res8, disp8, temp8, nestlev, addrbyte;
uint32_t temp1, temp2, temp3, temp4, temp5, temp32, tempaddr32, ea;
int32_t	result;
uint64_t totalexec;

extern uint16_t	VGA_SC[0x100], VGA_CRTC[0x100], VGA_ATTR[0x100], VGA_GC[0x100];
extern uint8_t updatedscreen;
union _bytewordregs_ regs;

uint8_t	portram[0x10000];
uint8_t	running = 0, debugmode, showcsip, verbose, mouseemu, didbootstrap = 0;
uint8_t	ethif;

extern uint8_t	vidmode;
extern uint8_t verbose;

extern void vidinterrupt();

extern uint8_t readVGA (uint32_t addr32);

void intcall86 (uint8_t intnum);

#define makeflagsword() \
    ( \
    2 | (uint16_t) cf | ((uint16_t) pf << 2) | ((uint16_t) af << 4) | ((uint16_t) zf << 6) | ((uint16_t) sf << 7) | \
    ((uint16_t) tf << 8) | ((uint16_t) ifl << 9) | ((uint16_t) df << 10) | ((uint16_t) of << 11) \
    )

#define decodeflagsword(x) { \
    temp16 = x; \
    cf = temp16 & 1; \
    pf = (temp16 >> 2) & 1; \
    af = (temp16 >> 4) & 1; \
    zf = (temp16 >> 6) & 1; \
    sf = (temp16 >> 7) & 1; \
    tf = (temp16 >> 8) & 1; \
    ifl = (temp16 >> 9) & 1; \
    df = (temp16 >> 10) & 1; \
    of = (temp16 >> 11) & 1; \
    }

extern void	writeVGA (uint32_t addr32, uint8_t value);
extern void	portout (uint16_t portnum, uint8_t value);
extern void	portout16 (uint16_t portnum, uint16_t value);
extern uint8_t	portin (uint16_t portnum);
extern uint16_t portin16 (uint16_t portnum);

void write86 (uint32_t addr32, uint8_t value) {
    tempaddr32 = addr32 & 0xFFFFF;
    if (readonly[tempaddr32] || (tempaddr32 >= 0xC0000) ) {
        return;
    }

    if ( (tempaddr32 >= 0xA0000) && (tempaddr32 <= 0xBFFFF) ) {
        if ( (vidmode != 0x13) && (vidmode != 0x12) && (vidmode != 0xD) && (vidmode != 0x10) ) {
            RAM[tempaddr32] = value;
            updatedscreen = 1;
        }
        else if ( ( (VGA_SC[4] & 6) == 0) && (vidmode != 0xD) && (vidmode != 0x10) && (vidmode != 0x12) ) {
            RAM[tempaddr32] = value;
            updatedscreen = 1;
        }
        else {
            writeVGA (tempaddr32 - 0xA0000, value);
        }

        updatedscreen = 1;
    }
    else {
        RAM[tempaddr32] = value;
    }
}

void writew86 (uint32_t addr32, uint16_t value) {
    write86 (addr32, (uint8_t) value);
    write86 (addr32 + 1, (uint8_t) (value >> 8) );
}

uint8_t read86 (uint32_t addr32) {
    addr32 &= 0xFFFFF;
    if ( (addr32 >= 0xA0000) && (addr32 <= 0xBFFFF) ) {
        if ( (vidmode == 0xD) || (vidmode == 0xE) || (vidmode == 0x10) ) return (readVGA (addr32 - 0xA0000) );
        if ( (vidmode != 0x13) && (vidmode != 0x12) && (vidmode != 0xD) ) return (RAM[addr32]);
        if ( (VGA_SC[4] & 6) == 0)
            return (RAM[addr32]);
        else
            return (readVGA (addr32 - 0xA0000) );
    }

    if (!didbootstrap) {
        RAM[0x410] = 0x41; //ugly hack to make BIOS always believe we have an EGA/VGA card installed
        RAM[0x475] = hdcount; //the BIOS doesn't have any concept of hard drives, so here's another hack
    }

    return (RAM[addr32]);
}

uint16_t readw86 (uint32_t addr32) {
    return ( (uint16_t) read86 (addr32) | (uint16_t) (read86 (addr32 + 1) << 8) );
}

void flag_szp8 (uint8_t value) {
    if (!value) {
        zf = 1;
    }
    else {
        zf = 0;	/* set or clear zero flag */
    }

    if (value & 0x80) {
        sf = 1;
    }
    else {
        sf = 0;	/* set or clear sign flag */
    }

    pf = parity[value]; /* retrieve parity state from lookup table */
}

void flag_szp16 (uint16_t value) {
    if (!value) {
        zf = 1;
    }
    else {
        zf = 0;	/* set or clear zero flag */
    }

    if (value & 0x8000) {
        sf = 1;
    }
    else {
        sf = 0;	/* set or clear sign flag */
    }

    pf = parity[value & 255];	/* retrieve parity state from lookup table */
}

void flag_log8 (uint8_t value) {
    flag_szp8 (value);
    cf = 0;
    of = 0; /* bitwise logic ops always clear carry and overflow */
}

void flag_log16 (uint16_t value) {
    flag_szp16 (value);
    cf = 0;
    of = 0; /* bitwise logic ops always clear carry and overflow */
}

void flag_adc8 (uint8_t v1, uint8_t v2, uint8_t v3) {

    /* v1 = destination operand, v2 = source operand, v3 = carry flag */
    uint16_t	dst;

    dst = (uint16_t) v1 + (uint16_t) v2 + (uint16_t) v3;
    flag_szp8 ( (uint8_t) dst);
    if ( ( (dst ^ v1) & (dst ^ v2) & 0x80) == 0x80) {
        of = 1;
    }
    else {
        of = 0; /* set or clear overflow flag */
    }

    if (dst & 0xFF00) {
        cf = 1;
    }
    else {
        cf = 0; /* set or clear carry flag */
    }

    if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
        af = 1;
    }
    else {
        af = 0; /* set or clear auxilliary flag */
    }
}

void flag_adc16 (uint16_t v1, uint16_t v2, uint16_t v3) {

    uint32_t	dst;

    dst = (uint32_t) v1 + (uint32_t) v2 + (uint32_t) v3;
    flag_szp16 ( (uint16_t) dst);
    if ( ( ( (dst ^ v1) & (dst ^ v2) ) & 0x8000) == 0x8000) {
        of = 1;
    }
    else {
        of = 0;
    }

    if (dst & 0xFFFF0000) {
        cf = 1;
    }
    else {
        cf = 0;
    }

    if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
        af = 1;
    }
    else {
        af = 0;
    }
}

void flag_add8 (uint8_t v1, uint8_t v2) {
    /* v1 = destination operand, v2 = source operand */
    uint16_t	dst;

    dst = (uint16_t) v1 + (uint16_t) v2;
    flag_szp8 ( (uint8_t) dst);
    if (dst & 0xFF00) {
        cf = 1;
    }
    else {
        cf = 0;
    }

    if ( ( (dst ^ v1) & (dst ^ v2) & 0x80) == 0x80) {
        of = 1;
    }
    else {
        of = 0;
    }

    if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
        af = 1;
    }
    else {
        af = 0;
    }
}

void flag_add16 (uint16_t v1, uint16_t v2) {
    /* v1 = destination operand, v2 = source operand */
    uint32_t	dst;

    dst = (uint32_t) v1 + (uint32_t) v2;
    flag_szp16 ( (uint16_t) dst);
    if (dst & 0xFFFF0000) {
        cf = 1;
    }
    else {
        cf = 0;
    }

    if ( ( (dst ^ v1) & (dst ^ v2) & 0x8000) == 0x8000) {
        of = 1;
    }
    else {
        of = 0;
    }

    if ( ( (v1 ^ v2 ^ dst) & 0x10) == 0x10) {
        af = 1;
    }
    else {
        af = 0;
    }
}

void flag_sbb8 (uint8_t v1, uint8_t v2, uint8_t v3) {

    /* v1 = destination operand, v2 = source operand, v3 = carry flag */
    uint16_t	dst;

    v2 += v3;
    dst = (uint16_t) v1 - (uint16_t) v2;
    flag_szp8 ( (uint8_t) dst);
    if (dst & 0xFF00) {
        cf = 1;
    }
    else {
        cf = 0;
    }

    if ( (dst ^ v1) & (v1 ^ v2) & 0x80) {
        of = 1;
    }
    else {
        of = 0;
    }

    if ( (v1 ^ v2 ^ dst) & 0x10) {
        af = 1;
    }
    else {
        af = 0;
    }
}

void flag_sbb16 (uint16_t v1, uint16_t v2, uint16_t v3) {

    /* v1 = destination operand, v2 = source operand, v3 = carry flag */
    uint32_t	dst;

    v2 += v3;
    dst = (uint32_t) v1 - (uint32_t) v2;
    flag_szp16 ( (uint16_t) dst);
    if (dst & 0xFFFF0000) {
        cf = 1;
    }
    else {
        cf = 0;
    }

    if ( (dst ^ v1) & (v1 ^ v2) & 0x8000) {
        of = 1;
    }
    else {
        of = 0;
    }

    if ( (v1 ^ v2 ^ dst) & 0x10) {
        af = 1;
    }
    else {
        af = 0;
    }
}

void flag_sub8 (uint8_t v1, uint8_t v2) {

    /* v1 = destination operand, v2 = source operand */
    uint16_t	dst;

    dst = (uint16_t) v1 - (uint16_t) v2;
    flag_szp8 ( (uint8_t) dst);
    if (dst & 0xFF00) {
        cf = 1;
    }
    else {
        cf = 0;
    }

    if ( (dst ^ v1) & (v1 ^ v2) & 0x80) {
        of = 1;
    }
    else {
        of = 0;
    }

    if ( (v1 ^ v2 ^ dst) & 0x10) {
        af = 1;
    }
    else {
        af = 0;
    }
}

void flag_sub16 (uint16_t v1, uint16_t v2) {

    /* v1 = destination operand, v2 = source operand */
    uint32_t	dst;

    dst = (uint32_t) v1 - (uint32_t) v2;
    flag_szp16 ( (uint16_t) dst);
    if (dst & 0xFFFF0000) {
        cf = 1;
    }
    else {
        cf = 0;
    }

    if ( (dst ^ v1) & (v1 ^ v2) & 0x8000) {
        of = 1;
    }
    else {
        of = 0;
    }

    if ( (v1 ^ v2 ^ dst) & 0x10) {
        af = 1;
    }
    else {
        af = 0;
    }
}

void op_adc8() {
    res8 = oper1b + oper2b + cf;
    flag_adc8 (oper1b, oper2b, cf);
}

void op_adc16() {
    res16 = oper1 + oper2 + cf;
    flag_adc16 (oper1, oper2, cf);
}

void op_add8() {
    res8 = oper1b + oper2b;
    flag_add8 (oper1b, oper2b);
}

void op_add16() {
    res16 = oper1 + oper2;
    flag_add16 (oper1, oper2);
}

void op_and8() {
    res8 = oper1b & oper2b;
    flag_log8 (res8);
}

void op_and16() {
    res16 = oper1 & oper2;
    flag_log16 (res16);
}

void op_or8() {
    res8 = oper1b | oper2b;
    flag_log8 (res8);
}

void op_or16() {
    res16 = oper1 | oper2;
    flag_log16 (res16);
}

void op_xor8() {
    res8 = oper1b ^ oper2b;
    flag_log8 (res8);
}

void op_xor16() {
    res16 = oper1 ^ oper2;
    flag_log16 (res16);
}

void op_sub8() {
    res8 = oper1b - oper2b;
    flag_sub8 (oper1b, oper2b);
}

void op_sub16() {
    res16 = oper1 - oper2;
    flag_sub16 (oper1, oper2);
}

void op_sbb8() {
    res8 = oper1b - (oper2b + cf);
    flag_sbb8 (oper1b, oper2b, cf);
}

void op_sbb16() {
    res16 = oper1 - (oper2 + cf);
    flag_sbb16 (oper1, oper2, cf);
}

#define modregrm() { \
    addrbyte = getmem8(segregs[regcs], ip); \
    StepIP(1); \
    mode = addrbyte >> 6; \
    reg = (addrbyte >> 3) & 7; \
    rm = addrbyte & 7; \
    switch(mode) \
{ \
    case 0: \
    if(rm == 6) { \
    disp16 = getmem16(segregs[regcs], ip); \
    StepIP(2); \
    } \
    if(((rm == 2) || (rm == 3)) && !segoverride) { \
    useseg = segregs[regss]; \
    } \
    break; \
    \
    case 1: \
    disp16 = signext(getmem8(segregs[regcs], ip)); \
    StepIP(1); \
    if(((rm == 2) || (rm == 3) || (rm == 6)) && !segoverride) { \
    useseg = segregs[regss]; \
    } \
    break; \
    \
    case 2: \
    disp16 = getmem16(segregs[regcs], ip); \
    StepIP(2); \
    if(((rm == 2) || (rm == 3) || (rm == 6)) && !segoverride) { \
    useseg = segregs[regss]; \
    } \
    break; \
    \
    default: \
    disp8 = 0; \
    disp16 = 0; \
    } \
    }

void getea (uint8_t rmval) {
    uint32_t	tempea;

    tempea = 0;
    switch (mode) {
    case 0:
        switch (rmval) {
        case 0:
            tempea = regs.wordregs[regbx] + regs.wordregs[regsi];
            break;
        case 1:
            tempea = regs.wordregs[regbx] + regs.wordregs[regdi];
            break;
        case 2:
            tempea = regs.wordregs[regbp] + regs.wordregs[regsi];
            break;
        case 3:
            tempea = regs.wordregs[regbp] + regs.wordregs[regdi];
            break;
        case 4:
            tempea = regs.wordregs[regsi];
            break;
        case 5:
            tempea = regs.wordregs[regdi];
            break;
        case 6:
            tempea = disp16;
            break;
        case 7:
            tempea = regs.wordregs[regbx];
            break;
        }
        break;

    case 1:
    case 2:
        switch (rmval) {
        case 0:
            tempea = regs.wordregs[regbx] + regs.wordregs[regsi] + disp16;
            break;
        case 1:
            tempea = regs.wordregs[regbx] + regs.wordregs[regdi] + disp16;
            break;
        case 2:
            tempea = regs.wordregs[regbp] + regs.wordregs[regsi] + disp16;
            break;
        case 3:
            tempea = regs.wordregs[regbp] + regs.wordregs[regdi] + disp16;
            break;
        case 4:
            tempea = regs.wordregs[regsi] + disp16;
            break;
        case 5:
            tempea = regs.wordregs[regdi] + disp16;
            break;
        case 6:
            tempea = regs.wordregs[regbp] + disp16;
            break;
        case 7:
            tempea = regs.wordregs[regbx] + disp16;
            break;
        }
        break;
    }

    ea = (tempea & 0xFFFF) + (useseg << 4);
}

void push (uint16_t pushval) {
    putreg16 (regsp, getreg16 (regsp) - 2);
    putmem16 (segregs[regss], getreg16 (regsp), pushval);
}

uint16_t pop() {

    uint16_t	tempval;

    tempval = getmem16 (segregs[regss], getreg16 (regsp) );
    putreg16 (regsp, getreg16 (regsp) + 2);
    return tempval;
}

void reset86() {
    segregs[regcs] = 0xFFFF;
    ip = 0x0000;
    //regs.wordregs[regsp] = 0xFFFE;
}

uint16_t readrm16 (uint8_t rmval) {
    if (mode < 3) {
        getea (rmval);
        return read86 (ea) | ( (uint16_t) read86 (ea + 1) << 8);
    }
    else {
        return getreg16 (rmval);
    }
}

uint8_t readrm8 (uint8_t rmval) {
    if (mode < 3) {
        getea (rmval);
        return read86 (ea);
    }
    else {
        return getreg8 (rmval);
    }
}

void writerm16 (uint8_t rmval, uint16_t value) {
    if (mode < 3) {
        getea (rmval);
        write86 (ea, value & 0xFF);
        write86 (ea + 1, value >> 8);
    }
    else {
        putreg16 (rmval, value);
    }
}

void writerm8 (uint8_t rmval, uint8_t value) {
    if (mode < 3) {
        getea (rmval);
        write86 (ea, value);
    }
    else {
        putreg8 (rmval, value);
    }
}

uint8_t op_grp2_8 (uint8_t cnt) {

    uint16_t	s;
    uint16_t	shift;
    uint16_t	oldcf;
    uint16_t	msb;

    s = oper1b;
    oldcf = cf;
#ifdef CPU_V20 //80186/V20 class CPUs limit shift count to 31
    cnt &= 0x1F;
#endif
    switch (reg) {
    case 0: /* ROL r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            if (s & 0x80) {
                cf = 1;
            }
            else {
                cf = 0;
            }

            s = s << 1;
            s = s | cf;
        }

        if (cnt == 1) {
            of = cf ^ ( (s >> 7) & 1);
        }
        break;

    case 1: /* ROR r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            cf = s & 1;
            s = (s >> 1) | (cf << 7);
        }

        if (cnt == 1) {
            of = (s >> 7) ^ ( (s >> 6) & 1);
        }
        break;

    case 2: /* RCL r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            oldcf = cf;
            if (s & 0x80) {
                cf = 1;
            }
            else {
                cf = 0;
            }

            s = s << 1;
            s = s | oldcf;
        }

        if (cnt == 1) {
            of = cf ^ ( (s >> 7) & 1);
        }
        break;

    case 3: /* RCR r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            oldcf = cf;
            cf = s & 1;
            s = (s >> 1) | (oldcf << 7);
        }

        if (cnt == 1) {
            of = (s >> 7) ^ ( (s >> 6) & 1);
        }
        break;

    case 4:
    case 6: /* SHL r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            if (s & 0x80) {
                cf = 1;
            }
            else {
                cf = 0;
            }

            s = (s << 1) & 0xFF;
        }

        if ( (cnt == 1) && (cf == (s >> 7) ) ) {
            of = 0;
        }
        else {
            of = 1;
        }

        flag_szp8 ( (uint8_t) s);
        break;

    case 5: /* SHR r/m8 */
        if ( (cnt == 1) && (s & 0x80) ) {
            of = 1;
        }
        else {
            of = 0;
        }

        for (shift = 1; shift <= cnt; shift++) {
            cf = s & 1;
            s = s >> 1;
        }

        flag_szp8 ( (uint8_t) s);
        break;

    case 7: /* SAR r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            msb = s & 0x80;
            cf = s & 1;
            s = (s >> 1) | msb;
        }

        of = 0;
        flag_szp8 ( (uint8_t) s);
        break;
    }

    return s & 0xFF;
}

uint16_t op_grp2_16 (uint8_t cnt) {

    uint32_t	s;
    uint32_t	shift;
    uint32_t	oldcf;
    uint32_t	msb;

    s = oper1;
    oldcf = cf;
#ifdef CPU_V20 //80186/V20 class CPUs limit shift count to 31
    cnt &= 0x1F;
#endif
    switch (reg) {
    case 0: /* ROL r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            if (s & 0x8000) {
                cf = 1;
            }
            else {
                cf = 0;
            }

            s = s << 1;
            s = s | cf;
        }

        if (cnt == 1) {
            of = cf ^ ( (s >> 15) & 1);
        }
        break;

    case 1: /* ROR r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            cf = s & 1;
            s = (s >> 1) | (cf << 15);
        }

        if (cnt == 1) {
            of = (s >> 15) ^ ( (s >> 14) & 1);
        }
        break;

    case 2: /* RCL r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            oldcf = cf;
            if (s & 0x8000) {
                cf = 1;
            }
            else {
                cf = 0;
            }

            s = s << 1;
            s = s | oldcf;
        }

        if (cnt == 1) {
            of = cf ^ ( (s >> 15) & 1);
        }
        break;

    case 3: /* RCR r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            oldcf = cf;
            cf = s & 1;
            s = (s >> 1) | (oldcf << 15);
        }

        if (cnt == 1) {
            of = (s >> 15) ^ ( (s >> 14) & 1);
        }
        break;

    case 4:
    case 6: /* SHL r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            if (s & 0x8000) {
                cf = 1;
            }
            else {
                cf = 0;
            }

            s = (s << 1) & 0xFFFF;
        }

        if ( (cnt == 1) && (cf == (s >> 15) ) ) {
            of = 0;
        }
        else {
            of = 1;
        }

        flag_szp16 ( (uint16_t) s);
        break;

    case 5: /* SHR r/m8 */
        if ( (cnt == 1) && (s & 0x8000) ) {
            of = 1;
        }
        else {
            of = 0;
        }

        for (shift = 1; shift <= cnt; shift++) {
            cf = s & 1;
            s = s >> 1;
        }

        flag_szp16 ( (uint16_t) s);
        break;

    case 7: /* SAR r/m8 */
        for (shift = 1; shift <= cnt; shift++) {
            msb = s & 0x8000;
            cf = s & 1;
            s = (s >> 1) | msb;
        }

        of = 0;
        flag_szp16 ( (uint16_t) s);
        break;
    }

    return (uint16_t) s & 0xFFFF;
}

void op_div8 (uint16_t valdiv, uint8_t divisor) {
    if (divisor == 0) {
        intcall86 (0);
        return;
    }

    if ( (valdiv / (uint16_t) divisor) > 0xFF) {
        intcall86 (0);
        return;
    }

    regs.byteregs[regah] = valdiv % (uint16_t) divisor;
    regs.byteregs[regal] = valdiv / (uint16_t) divisor;
}

void op_idiv8 (uint16_t valdiv, uint8_t divisor) {

    uint16_t	s1;
    uint16_t	s2;
    uint16_t	d1;
    uint16_t	d2;
    int	sign;

    if (divisor == 0) {
        intcall86 (0);
        return;
    }

    s1 = valdiv;
    s2 = divisor;
    sign = ( ( (s1 ^ s2) & 0x8000) != 0);
    s1 = (s1 < 0x8000) ? s1 : ( (~s1 + 1) & 0xffff);
    s2 = (s2 < 0x8000) ? s2 : ( (~s2 + 1) & 0xffff);
    d1 = s1 / s2;
    d2 = s1 % s2;
    if (d1 & 0xFF00) {
        intcall86 (0);
        return;
    }

    if (sign) {
        d1 = (~d1 + 1) & 0xff;
        d2 = (~d2 + 1) & 0xff;
    }

    regs.byteregs[regah] = (uint8_t) d2;
    regs.byteregs[regal] = (uint8_t) d1;
}

void op_grp3_8() {
    oper1 = signext (oper1b);
    oper2 = signext (oper2b);
    switch (reg) {
    case 0:
    case 1: /* TEST */
        flag_log8 (oper1b & getmem8 (segregs[regcs], ip) );
        StepIP (1);
        break;

    case 2: /* NOT */
        res8 = ~oper1b;
        break;

    case 3: /* NEG */
        res8 = (~oper1b) + 1;
        flag_sub8 (0, oper1b);
        if (res8 == 0) {
            cf = 0;
        }
        else {
            cf = 1;
        }
        break;

    case 4: /* MUL */
        temp1 = (uint32_t) oper1b * (uint32_t) regs.byteregs[regal];
        putreg16 (regax, temp1 & 0xFFFF);
        flag_szp8 ( (uint8_t) temp1);
        if (regs.byteregs[regah]) {
            cf = 1;
            of = 1;
        }
        else {
            cf = 0;
            of = 0;
        }
#ifndef CPU_V20
        zf = 0;
#endif
        break;

    case 5: /* IMUL */
        oper1 = signext (oper1b);
        temp1 = signext (regs.byteregs[regal]);
        temp2 = oper1;
        if ( (temp1 & 0x80) == 0x80) {
            temp1 = temp1 | 0xFFFFFF00;
        }

        if ( (temp2 & 0x80) == 0x80) {
            temp2 = temp2 | 0xFFFFFF00;
        }

        temp3 = (temp1 * temp2) & 0xFFFF;
        putreg16 (regax, temp3 & 0xFFFF);
        if (regs.byteregs[regah]) {
            cf = 1;
            of = 1;
        }
        else {
            cf = 0;
            of = 0;
        }
#ifndef CPU_V20
        zf = 0;
#endif
        break;

    case 6: /* DIV */
        op_div8 (getreg16 (regax), oper1b);
        break;

    case 7: /* IDIV */
        op_idiv8 (getreg16 (regax), oper1b);
        break;
    }
}

void op_div16 (uint32_t valdiv, uint16_t divisor) {
    if (divisor == 0) {
        intcall86 (0);
        return;
    }

    if ( (valdiv / (uint32_t) divisor) > 0xFFFF) {
        intcall86 (0);
        return;
    }

    putreg16 (regdx, valdiv % (uint32_t) divisor);
    putreg16 (regax, valdiv / (uint32_t) divisor);
}

void op_idiv16 (uint32_t valdiv, uint16_t divisor) {

    uint32_t	d1;
    uint32_t	d2;
    uint32_t	s1;
    uint32_t	s2;
    int	sign;

    if (divisor == 0) {
        intcall86 (0);
        return;
    }

    s1 = valdiv;
    s2 = divisor;
    s2 = (s2 & 0x8000) ? (s2 | 0xffff0000) : s2;
    sign = ( ( (s1 ^ s2) & 0x80000000) != 0);
    s1 = (s1 < 0x80000000) ? s1 : ( (~s1 + 1) & 0xffffffff);
    s2 = (s2 < 0x80000000) ? s2 : ( (~s2 + 1) & 0xffffffff);
    d1 = s1 / s2;
    d2 = s1 % s2;
    if (d1 & 0xFFFF0000) {
        intcall86 (0);
        return;
    }

    if (sign) {
        d1 = (~d1 + 1) & 0xffff;
        d2 = (~d2 + 1) & 0xffff;
    }

    putreg16 (regax, d1);
    putreg16 (regdx, d2);
}

void op_grp3_16() {
    switch (reg) {
    case 0:
    case 1: /* TEST */
        flag_log16 (oper1 & getmem16 (segregs[regcs], ip) );
        StepIP (2);
        break;

    case 2: /* NOT */
        res16 = ~oper1;
        break;

    case 3: /* NEG */
        res16 = (~oper1) + 1;
        flag_sub16 (0, oper1);
        if (res16) {
            cf = 1;
        }
        else {
            cf = 0;
        }
        break;

    case 4: /* MUL */
        temp1 = (uint32_t) oper1 * (uint32_t) getreg16 (regax);
        putreg16 (regax, temp1 & 0xFFFF);
        putreg16 (regdx, temp1 >> 16);
        flag_szp16 ( (uint16_t) temp1);
        if (getreg16 (regdx) ) {
            cf = 1;
            of = 1;
        }
        else {
            cf = 0;
            of = 0;
        }
#ifndef CPU_V20
        zf = 0;
#endif
        break;

    case 5: /* IMUL */
        temp1 = getreg16 (regax);
        temp2 = oper1;
        if (temp1 & 0x8000) {
            temp1 |= 0xFFFF0000;
        }

        if (temp2 & 0x8000) {
            temp2 |= 0xFFFF0000;
        }

        temp3 = temp1 * temp2;
        putreg16 (regax, temp3 & 0xFFFF);	/* into register ax */
        putreg16 (regdx, temp3 >> 16);	/* into register dx */
        if (getreg16 (regdx) ) {
            cf = 1;
            of = 1;
        }
        else {
            cf = 0;
            of = 0;
        }
#ifndef CPU_V20
        zf = 0;
#endif
        break;

    case 6: /* DIV */
        op_div16 ( ( (uint32_t) getreg16 (regdx) << 16) + getreg16 (regax), oper1);
        break;

    case 7: /* DIV */
        op_idiv16 ( ( (uint32_t) getreg16 (regdx) << 16) + getreg16 (regax), oper1);
        break;
    }
}

void op_grp5() {
    switch (reg) {
    case 0: /* INC Ev */
        oper2 = 1;
        tempcf = cf;
        op_add16();
        cf = tempcf;
        writerm16 (rm, res16);
        break;

    case 1: /* DEC Ev */
        oper2 = 1;
        tempcf = cf;
        op_sub16();
        cf = tempcf;
        writerm16 (rm, res16);
        break;

    case 2: /* CALL Ev */
        push (ip);
        ip = oper1;
        break;

    case 3: /* CALL Mp */
        push (segregs[regcs]);
        push (ip);
        getea (rm);
        ip = (uint16_t) read86 (ea) + (uint16_t) read86 (ea + 1) * 256;
        segregs[regcs] = (uint16_t) read86 (ea + 2) + (uint16_t) read86 (ea + 3) * 256;
        break;

    case 4: /* JMP Ev */
        ip = oper1;
        break;

    case 5: /* JMP Mp */
        getea (rm);
        ip = (uint16_t) read86 (ea) + (uint16_t) read86 (ea + 1) * 256;
        segregs[regcs] = (uint16_t) read86 (ea + 2) + (uint16_t) read86 (ea + 3) * 256;
        break;

    case 6: /* PUSH Ev */
        push (oper1);
        break;
    }
}

uint8_t dolog = 0, didintr = 0;
FILE	*logout;
uint8_t printops = 0;

#ifdef NETWORKING_ENABLED
extern void nethandler();
#endif
extern void diskhandler();
extern void readdisk (uint8_t drivenum, uint16_t dstseg, uint16_t dstoff, uint16_t cyl, uint16_t sect, uint16_t head, uint16_t sectcount);

void intcall86 (uint8_t intnum) {
    static uint16_t lastint10ax;
    uint16_t oldregax;
    didintr = 1;

    if (intnum == 0x19) didbootstrap = 1;

    switch (intnum) {
    case 0x10:
        updatedscreen = 1;
        if ( (regs.byteregs[regah]==0x00) || (regs.byteregs[regah]==0x10) ) {
            oldregax = regs.wordregs[regax];
            vidinterrupt();
            regs.wordregs[regax] = oldregax;
            if (regs.byteregs[regah]==0x10) return;
            if (vidmode==9) return;
        }
        if ( (regs.byteregs[regah]==0x1A) && (lastint10ax!=0x0100) ) { //the 0x0100 is a cheap hack to make it not do this if DOS EDIT/QBASIC
            regs.byteregs[regal] = 0x1A;
            regs.byteregs[regbl] = 0x8;
            return;
        }
        lastint10ax = regs.wordregs[regax];
        break;

#ifndef DISK_CONTROLLER_ATA
    case 0x19: //bootstrap
        if (bootdrive<255) { //read first sector of boot drive into 07C0:0000 and execute it
            regs.byteregs[regdl] = bootdrive;
            readdisk (regs.byteregs[regdl], 0x07C0, 0x0000, 0, 1, 0, 1);
            segregs[regcs] = 0x0000;
            ip = 0x7C00;
        }
        else {
            segregs[regcs] = 0xF600;	//start ROM BASIC at bootstrap if requested
            ip = 0x0000;
        }
        return;

    case 0x13:
    case 0xFD:
        diskhandler();
        return;
#endif
#ifdef NETWORKING_OLDCARD
    case 0xFC:
#ifdef NETWORKING_ENABLED
        nethandler();
#endif
        return;
#endif
    }

    push (makeflagsword() );
    push (segregs[regcs]);
    push (ip);
    segregs[regcs] = getmem16 (0, (uint16_t) intnum * 4 + 2);
    ip = getmem16 (0, (uint16_t) intnum * 4);
    ifl = 0;
    tf = 0;
}

#if defined(NETWORKING_ENABLED)
extern struct netstruct {
    uint8_t	enabled;
    uint8_t	canrecv;
    uint16_t	pktlen;
} net;
#endif

uint64_t	frametimer = 0, didwhen = 0, didticks = 0;
uint32_t	makeupticks = 0;
extern float	timercomp;
uint64_t	timerticks = 0, realticks = 0;
uint64_t	lastcountertimer = 0, counterticks = 10000;
extern uint8_t	nextintr();
extern void	timing();

uint32_t loopcount;
uint8_t	docontinue;
uint16_t firstip;
uint16_t trap_toggle = 0;

void (*opcodeTable[256])() = {0};

void opAdd8() {
    /* 00 ADD Eb Gb */
    modregrm();
    oper1b = readrm8 (rm);
    oper2b = getreg8 (reg);
    op_add8();
    writerm8 (rm, res8);
}

void opAdd16(){
    /* 01 ADD Ev Gv */
    modregrm();
    oper1 = readrm16 (rm);
    oper2 = getreg16 (reg);
    op_add16();
    writerm16 (rm, res16);
}

void opAdd8_2(){
    /* 02 ADD Gb Eb */
    modregrm();
    oper1b = getreg8 (reg);
    oper2b = readrm8 (rm);
    op_add8();
    putreg8 (reg, res8);
}

void opAdd16_2(){
    /* 03 ADD Gv Ev */
    modregrm();
    oper1 = getreg16 (reg);
    oper2 = readrm16 (rm);
    op_add16();
    putreg16 (reg, res16);
}

void opAdd8_3(){
    /* 04 ADD regs.byteregs[regal] Ib */
    oper1b = regs.byteregs[regal];
    oper2b = getmem8 (segregs[regcs], ip);
    StepIP (1);
    op_add8();
    regs.byteregs[regal] = res8;
}

void opAdd16_3() {
    oper1 = (getreg16 (regax) );
    oper2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    op_add16();
    putreg16 (regax, res16);
}

void opPush(){
    /* 06 PUSH segregs[reges] */
    push (segregs[reges]);
}

void opPop(){
    /* 07 POP segregs[reges] */
    segregs[reges] = pop();
}

void opOr8(){
    /* 08 OR Eb Gb */
    modregrm();
    oper1b = readrm8 (rm);
    oper2b = getreg8 (reg);
    op_or8();
    writerm8 (rm, res8);
}

void opOr16(){
    /* 09 OR Ev Gv */
    modregrm();
    oper1 = readrm16 (rm);
    oper2 = getreg16 (reg);
    op_or16();
    writerm16 (rm, res16);
}

void opOr8_2() {
    /* 0A OR Gb Eb */
    modregrm();
    oper1b = getreg8 (reg);
    oper2b = readrm8 (rm);
    op_or8();
    putreg8 (reg, res8);
}

void opOr16_2() {
    /* 0B OR Gv Ev */
    modregrm();
    oper1 = getreg16 (reg);
    oper2 = readrm16 (rm);
    op_or16();

    if ( (oper1 == 0xF802) && (oper2 == 0xF802) ) {
        sf = 0;	/* cheap hack to make Wolf 3D think we're a 286 so it plays */
    }
    putreg16 (reg, res16);
}

void opOr8_3(){
    /* 0C OR regs.byteregs[regal] Ib */
    oper1b = regs.byteregs[regal];
    oper2b = getmem8 (segregs[regcs], ip);
    StepIP (1);
    op_or8();
    regs.byteregs[regal] = res8;
}

void opOr16_3(){
    /* 0D OR eAX Iv */
    oper1 = getreg16 (regax);
    oper2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    op_or16();
    putreg16 (regax, res16);
}

void opPush_2(){
    /* 0E PUSH segregs[regcs] */
    push (segregs[regcs]);
}

void opPop_2(){
    //0F POP CS
#ifndef CPU_V20
    segregs[regcs] = pop();
#endif
}

void opAdc8(){
    /* 10 ADC Eb Gb */
    modregrm();
    oper1b = readrm8 (rm);
    oper2b = getreg8 (reg);
    op_adc8();
    writerm8 (rm, res8);
}

void opAdc16(){
    /* 11 ADC Ev Gv */
    modregrm();
    oper1 = readrm16 (rm);
    oper2 = getreg16 (reg);
    op_adc16();
    writerm16 (rm, res16);
}

void opAdc8_2(){
    /* 12 ADC Gb Eb */
    modregrm();
    oper1b = getreg8 (reg);
    oper2b = readrm8 (rm);
    op_adc8();
    putreg8 (reg, res8);
}

void opAdc16_2(){
    /* 13 ADC Gv Ev */
    modregrm();
    oper1 = getreg16 (reg);
    oper2 = readrm16 (rm);
    op_adc16();
    putreg16 (reg, res16);
}

void opAdc8_3(){
    /* 14 ADC regs.byteregs[regal] Ib */
    oper1b = regs.byteregs[regal];
    oper2b = getmem8 (segregs[regcs], ip);
    StepIP (1);
    op_adc8();
    regs.byteregs[regal] = res8;
}

void opAdc16_3(){
    /* 15 ADC eAX Iv */
    oper1 = getreg16 (regax);
    oper2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    op_adc16();
    putreg16 (regax, res16);
}

void opPush_3(){
    /* 16 PUSH segregs[regss] */
    push (segregs[regss]);
}

void opPop_3(){
    /* 17 POP segregs[regss] */
    segregs[regss] = pop();
}

void opSbb8(){
    /* 18 SBB Eb Gb */
    modregrm();
    oper1b = readrm8 (rm);
    oper2b = getreg8 (reg);
    op_sbb8();
    writerm8 (rm, res8);
}

void opSbb16(){
    /* 19 SBB Ev Gv */
    modregrm();
    oper1 = readrm16 (rm);
    oper2 = getreg16 (reg);
    op_sbb16();
    writerm16 (rm, res16);
}

void opSbb8_2(){
    /* 1A SBB Gb Eb */
    modregrm();
    oper1b = getreg8 (reg);
    oper2b = readrm8 (rm);
    op_sbb8();
    putreg8 (reg, res8);
}

void opSbb16_2(){
    /* 1B SBB Gv Ev */
    modregrm();
    oper1 = getreg16 (reg);
    oper2 = readrm16 (rm);
    op_sbb16();
    putreg16 (reg, res16);
}

void opSbb8_3(){
    /* 1C SBB regs.byteregs[regal] Ib */
    oper1b = regs.byteregs[regal];
    oper2b = getmem8 (segregs[regcs], ip);
    StepIP (1);
    op_sbb8();
    regs.byteregs[regal] = res8;
}

void opSbb16_3(){
    /* 1D SBB eAX Iv */
    oper1 = getreg16 (regax);
    oper2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    op_sbb16();
    putreg16 (regax, res16);
}

void opPush_4(){
    /* 1E PUSH segregs[regds] */
    push (segregs[regds]);
}

void opPop_4(){
    /* 1F POP segregs[regds] */
    segregs[regds] = pop();
}

void opAnd8(){
    /* 20 AND Eb Gb */
    modregrm();
    oper1b = readrm8 (rm);
    oper2b = getreg8 (reg);
    op_and8();
    writerm8 (rm, res8);
}

void opAnd16(){
    /* 21 AND Ev Gv */
    modregrm();
    oper1 = readrm16 (rm);
    oper2 = getreg16 (reg);
    op_and16();
    writerm16 (rm, res16);
}

void opAnd8_2(){
    /* 22 AND Gb Eb */
    modregrm();
    oper1b = getreg8 (reg);
    oper2b = readrm8 (rm);
    op_and8();
    putreg8 (reg, res8);
}

void opAnd16_2(){
    /* 23 AND Gv Ev */
    modregrm();
    oper1 = getreg16 (reg);
    oper2 = readrm16 (rm);
    op_and16();
    putreg16 (reg, res16);
}

void opAnd8_3(){
    /* 24 AND regs.byteregs[regal] Ib */
    oper1b = regs.byteregs[regal];
    oper2b = getmem8 (segregs[regcs], ip);
    StepIP (1);
    op_and8();
    regs.byteregs[regal] = res8;
}

void opAnd16_3(){
    /* 25 AND eAX Iv */
    oper1 = getreg16 (regax);
    oper2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    op_and16();
    putreg16 (regax, res16);
}

void opDaa(){
    /* 27 DAA */
    if ( ( (regs.byteregs[regal] & 0xF) > 9) || (af == 1) ) {
        oper1 = regs.byteregs[regal] + 6;
        regs.byteregs[regal] = oper1 & 255;
        if (oper1 & 0xFF00) {
            cf = 1;
        }
        else {
            cf = 0;
        }

        af = 1;
    }
    else {
        af = 0;
    }

    if ( ( (regs.byteregs[regal] & 0xF0) > 0x90) || (cf == 1) ) {
        regs.byteregs[regal] = regs.byteregs[regal] + 0x60;
        cf = 1;
    }
    else {
        cf = 0;
    }

    regs.byteregs[regal] = regs.byteregs[regal] & 255;
    flag_szp8 (regs.byteregs[regal]);
}

void opSub8(){
    /* 28 SUB Eb Gb */
    modregrm();
    oper1b = readrm8 (rm);
    oper2b = getreg8 (reg);
    op_sub8();
    writerm8 (rm, res8);
}

void opSub16(){
    /* 29 SUB Ev Gv */
    modregrm();
    oper1 = readrm16 (rm);
    oper2 = getreg16 (reg);
    op_sub16();
    writerm16 (rm, res16);
}

void opSub8_2(){
    /* 2A SUB Gb Eb */
    modregrm();
    oper1b = getreg8 (reg);
    oper2b = readrm8 (rm);
    op_sub8();
    putreg8 (reg, res8);
}

void opSub16_2(){
    /* 2B SUB Gv Ev */
    modregrm();
    oper1 = getreg16 (reg);
    oper2 = readrm16 (rm);
    op_sub16();
    putreg16 (reg, res16);
}

void opSub8_3(){
    /* 2C SUB regs.byteregs[regal] Ib */
    oper1b = regs.byteregs[regal];
    oper2b = getmem8 (segregs[regcs], ip);
    StepIP (1);
    op_sub8();
    regs.byteregs[regal] = res8;
}

void opSub16_3(){
    /* 2D SUB eAX Iv */
    oper1 = getreg16 (regax);
    oper2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    op_sub16();
    putreg16 (regax, res16);
}

void opDas(){
    /* 2F DAS */
    if ( ( (regs.byteregs[regal] & 15) > 9) || (af == 1) ) {
        oper1 = regs.byteregs[regal] - 6;
        regs.byteregs[regal] = oper1 & 255;
        if (oper1 & 0xFF00) {
            cf = 1;
        }
        else {
            cf = 0;
        }

        af = 1;
    }
    else {
        af = 0;
    }

    if ( ( (regs.byteregs[regal] & 0xF0) > 0x90) || (cf == 1) ) {
        regs.byteregs[regal] = regs.byteregs[regal] - 0x60;
        cf = 1;
    }
    else {
        cf = 0;
    }

    flag_szp8 (regs.byteregs[regal]);
}

void opXor8(){
    /* 30 XOR Eb Gb */
    modregrm();
    oper1b = readrm8 (rm);
    oper2b = getreg8 (reg);
    op_xor8();
    writerm8 (rm, res8);
}

void opXor16(){
    /* 31 XOR Ev Gv */
    modregrm();
    oper1 = readrm16 (rm);
    oper2 = getreg16 (reg);
    op_xor16();
    writerm16 (rm, res16);
}

void opXor8_2(){
    /* 32 XOR Gb Eb */
    modregrm();
    oper1b = getreg8 (reg);
    oper2b = readrm8 (rm);
    op_xor8();
    putreg8 (reg, res8);
}

void opXor16_2(){
    /* 33 XOR Gv Ev */
    modregrm();
    oper1 = getreg16 (reg);
    oper2 = readrm16 (rm);
    op_xor16();
    putreg16 (reg, res16);
}

void opXor8_3(){
    /* 34 XOR regs.byteregs[regal] Ib */
    oper1b = regs.byteregs[regal];
    oper2b = getmem8 (segregs[regcs], ip);
    StepIP (1);
    op_xor8();
    regs.byteregs[regal] = res8;
}

void opXor16_3(){
    /* 35 XOR eAX Iv */
    oper1 = getreg16 (regax);
    oper2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    op_xor16();
    putreg16 (regax, res16);
}

void opAaa(){
    /* 37 AAA ASCII */
    if ( ( (regs.byteregs[regal] & 0xF) > 9) || (af == 1) ) {
        regs.byteregs[regal] = regs.byteregs[regal] + 6;
        regs.byteregs[regah] = regs.byteregs[regah] + 1;
        af = 1;
        cf = 1;
    }
    else {
        af = 0;
        cf = 0;
    }

    regs.byteregs[regal] = regs.byteregs[regal] & 0xF;
}

void opCmp8(){
    /* 38 CMP Eb Gb */
    modregrm();
    oper1b = readrm8 (rm);
    oper2b = getreg8 (reg);
    flag_sub8 (oper1b, oper2b);
}

void opCmp16(){
    /* 39 CMP Ev Gv */
    modregrm();
    oper1 = readrm16 (rm);
    oper2 = getreg16 (reg);
    flag_sub16 (oper1, oper2);
}

void opCmp8_2(){
    /* 3A CMP Gb Eb */
    modregrm();
    oper1b = getreg8 (reg);
    oper2b = readrm8 (rm);
    flag_sub8 (oper1b, oper2b);
}

void opCmp16_2(){
    /* 3B CMP Gv Ev */
    modregrm();
    oper1 = getreg16 (reg);
    oper2 = readrm16 (rm);
    flag_sub16 (oper1, oper2);
}

void opCmp8_3(){
    /* 3C CMP regs.byteregs[regal] Ib */
    oper1b = regs.byteregs[regal];
    oper2b = getmem8 (segregs[regcs], ip);
    StepIP (1);
    flag_sub8 (oper1b, oper2b);
}

void opCmp16_3(){
    /* 3D CMP eAX Iv */
    oper1 = getreg16 (regax);
    oper2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    flag_sub16 (oper1, oper2);
}

void opAas() {
    /* 3F AAS ASCII */
    if ( ( (regs.byteregs[regal] & 0xF) > 9) || (af == 1) ) {
        regs.byteregs[regal] = regs.byteregs[regal] - 6;
        regs.byteregs[regah] = regs.byteregs[regah] - 1;
        af = 1;
        cf = 1;
    }
    else {
        af = 0;
        cf = 0;
    }

    regs.byteregs[regal] = regs.byteregs[regal] & 0xF;
}

void opIncEax(){
    /* 40 INC eAX */
    oldcf = cf;
    oper1 = getreg16 (regax);
    oper2 = 1;
    op_add16();
    cf = oldcf;
    putreg16 (regax, res16);
}

void opIncEcx(){
/* 41 INC eCX */
    oldcf = cf;
    oper1 = getreg16 (regcx);
    oper2 = 1;
    op_add16();
    cf = oldcf;
    putreg16 (regcx, res16);
}

void opIncEdx(){
    /* 42 INC eDX */
    oldcf = cf;
    oper1 = getreg16 (regdx);
    oper2 = 1;
    op_add16();
    cf = oldcf;
    putreg16 (regdx, res16);
}

void opIncEbx(){
    /* 43 INC eBX */
    oldcf = cf;
    oper1 = getreg16 (regbx);
    oper2 = 1;
    op_add16();
    cf = oldcf;
    putreg16 (regbx, res16);
}

void opIncEsp(){
    /* 44 INC eSP */
    oldcf = cf;
    oper1 = getreg16 (regsp);
    oper2 = 1;
    op_add16();
    cf = oldcf;
    putreg16 (regsp, res16);
}

void opIncEbp(){
    /* 45 INC eBP */
    oldcf = cf;
    oper1 = getreg16 (regbp);
    oper2 = 1;
    op_add16();
    cf = oldcf;
    putreg16 (regbp, res16);
}

void opIncEsi(){
    /* 46 INC eSI */
    oldcf = cf;
    oper1 = getreg16 (regsi);
    oper2 = 1;
    op_add16();
    cf = oldcf;
    putreg16 (regsi, res16);
}

void opIncEdi(){
    /* 47 INC eDI */
    oldcf = cf;
    oper1 = getreg16 (regdi);
    oper2 = 1;
    op_add16();
    cf = oldcf;
    putreg16 (regdi, res16);
}

void opDecEax(){
    /* 48 DEC eAX */
    oldcf = cf;
    oper1 = getreg16 (regax);
    oper2 = 1;
    op_sub16();
    cf = oldcf;
    putreg16 (regax, res16);
}

void opDecEcx(){
    /* 49 DEC eCX */
    oldcf = cf;
    oper1 = getreg16 (regcx);
    oper2 = 1;
    op_sub16();
    cf = oldcf;
    putreg16 (regcx, res16);
}

void opDecEdx(){
    /* 4A DEC eDX */
    oldcf = cf;
    oper1 = getreg16 (regdx);
    oper2 = 1;
    op_sub16();
    cf = oldcf;
    putreg16 (regdx, res16);
}

void opDecEbx(){
    /* 4B DEC eBX */
    oldcf = cf;
    oper1 = getreg16 (regbx);
    oper2 = 1;
    op_sub16();
    cf = oldcf;
    putreg16 (regbx, res16);
}

void opDecEsp(){
    /* 4C DEC eSP */
    oldcf = cf;
    oper1 = getreg16 (regsp);
    oper2 = 1;
    op_sub16();
    cf = oldcf;
    putreg16 (regsp, res16);
}

void opDecEbp(){
    /* 4D DEC eBP */
    oldcf = cf;
    oper1 = getreg16 (regbp);
    oper2 = 1;
    op_sub16();
    cf = oldcf;
    putreg16 (regbp, res16);
}

void opDecEsi(){
    /* 4E DEC eSI */
    oldcf = cf;
    oper1 = getreg16 (regsi);
    oper2 = 1;
    op_sub16();
    cf = oldcf;
    putreg16 (regsi, res16);
}

void opDecEdi(){
    /* 4F DEC eDI */
    oldcf = cf;
    oper1 = getreg16 (regdi);
    oper2 = 1;
    op_sub16();
    cf = oldcf;
    putreg16 (regdi, res16);
}

void opPushEax(){
    /* 50 PUSH eAX */
    push (getreg16 (regax) );
}

void opPushEcx(){
    /* 51 PUSH eCX */
    push (getreg16 (regcx) );
}

void opPushEdx(){
    /* 52 PUSH eDX */
    push (getreg16 (regdx) );
}

void opPushEbx(){
    /* 53 PUSH eBX */
    push (getreg16 (regbx) );
}

void opPushEsp(){
    /* 54 PUSH eSP */
    push (getreg16 (regsp) - 2);
}

void opPushEbp(){
    /* 55 PUSH eBP */
    push (getreg16 (regbp) );
}

void opPushEsi(){
    /* 56 PUSH eSI */
    push (getreg16 (regsi) );
}

void opPushEdi(){
    /* 57 PUSH eDI */
    push (getreg16 (regdi) );
}

void opPopEax(){
    /* 58 POP eAX */
    putreg16 (regax, pop() );
}

void opPopEcx(){
    /* 59 POP eCX */
    putreg16 (regcx, pop() );
}

void opPopEdx(){
    /* 5A POP eDX */
    putreg16 (regdx, pop() );
}

void opPopEbx(){
    /* 5B POP eBX */
    putreg16 (regbx, pop() );
}

void opPopEsp(){
    /* 5C POP eSP */
    putreg16 (regsp, pop() );
}

void opPopEbp(){
    /* 5D POP eBP */
    putreg16 (regbp, pop() );
}

void opPopEsi(){
    /* 5E POP eSI */
    putreg16 (regsi, pop() );
}

void opPopEdi(){
    /* 5F POP eDI */
    putreg16 (regdi, pop() );
}

void opPushA(){
    /* 60 PUSHA (80186+) */
    oldsp = getreg16 (regsp);
    push (getreg16 (regax) );
    push (getreg16 (regcx) );
    push (getreg16 (regdx) );
    push (getreg16 (regbx) );
    push (oldsp);
    push (getreg16 (regbp) );
    push (getreg16 (regsi) );
    push (getreg16 (regdi) );
}

void opPopA(){
    /* 61 POPA (80186+) */
    putreg16 (regdi, pop() );
    putreg16 (regsi, pop() );
    putreg16 (regbp, pop() );
    dummy = pop();
    putreg16 (regbx, pop() );
    putreg16 (regdx, pop() );
    putreg16 (regcx, pop() );
    putreg16 (regax, pop() );
}

void opBound(){
    /* 62 BOUND Gv, Ev (80186+) */
    modregrm();
    getea (rm);
    if (signext32 (getreg16 (reg) ) < signext32 ( getmem16 (ea >> 4, ea & 15) ) ) {
        intcall86 (5); //bounds check exception
    }
    else {
        ea += 2;
        if (signext32 (getreg16 (reg) ) > signext32 ( getmem16 (ea >> 4, ea & 15) ) ) {
            intcall86(5); //bounds check exception
        }
    }
}

void opPushIv(){
    /* 68 PUSH Iv (80186+) */
    push (getmem16 (segregs[regcs], ip) );
    StepIP (2);
}

void opImul(){
    /* 69 IMUL Gv Ev Iv (80186+) */
    modregrm();
    temp1 = readrm16 (rm);
    temp2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    if ( (temp1 & 0x8000L) == 0x8000L) {
        temp1 = temp1 | 0xFFFF0000L;
    }

    if ( (temp2 & 0x8000L) == 0x8000L) {
        temp2 = temp2 | 0xFFFF0000L;
    }

    temp3 = temp1 * temp2;
    putreg16 (reg, temp3 & 0xFFFFL);
    if (temp3 & 0xFFFF0000L) {
        cf = 1;
        of = 1;
    }
    else {
        cf = 0;
        of = 0;
    }
}

void opPushIb(){
    /* 6A PUSH Ib (80186+) */
    push (getmem8 (segregs[regcs], ip) );
    StepIP (1);
}

void opImul_2(){
    /* 6B IMUL Gv Eb Ib (80186+) */
    modregrm();
    temp1 = readrm16 (rm);
    temp2 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if ( (temp1 & 0x8000L) == 0x8000L) {
        temp1 = temp1 | 0xFFFF0000L;
    }

    if ( (temp2 & 0x8000L) == 0x8000L) {
        temp2 = temp2 | 0xFFFF0000L;
    }

    temp3 = temp1 * temp2;
    putreg16 (reg, temp3 & 0xFFFFL);
    if (temp3 & 0xFFFF0000L) {
        cf = 1;
        of = 1;
    }
    else {
        cf = 0;
        of = 0;
    }
}

void opInsb(){
    /* 6E INSB */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    putmem8 (useseg, getreg16 (regsi) , portin (regs.wordregs[regdx]) );
    if (df) {
        putreg16 (regsi, getreg16 (regsi) - 1);
        putreg16 (regdi, getreg16 (regdi) - 1);
    }
    else {
        putreg16 (regsi, getreg16 (regsi) + 1);
        putreg16 (regdi, getreg16 (regdi) + 1);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opInsw(){
    /* 6F INSW */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    putmem16 (useseg, getreg16 (regsi) , portin16 (regs.wordregs[regdx]) );
    if (df) {
        putreg16 (regsi, getreg16 (regsi) - 2);
        putreg16 (regdi, getreg16 (regdi) - 2);
    }
    else {
        putreg16 (regsi, getreg16 (regsi) + 2);
        putreg16 (regdi, getreg16 (regdi) + 2);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opOutsb(){
    /* 6E OUTSB */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    portout (regs.wordregs[regdx], getmem8 (useseg, getreg16 (regsi) ) );
    if (df) {
        putreg16 (regsi, getreg16 (regsi) - 1);
        putreg16 (regdi, getreg16 (regdi) - 1);
    }
    else {
        putreg16 (regsi, getreg16 (regsi) + 1);
        putreg16 (regdi, getreg16 (regdi) + 1);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opOutsw(){
    /* 6F OUTSW */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    portout16 (regs.wordregs[regdx], getmem16 (useseg, getreg16 (regsi) ) );
    if (df) {
        putreg16 (regsi, getreg16 (regsi) - 2);
        putreg16 (regdi, getreg16 (regdi) - 2);
    }
    else {
        putreg16 (regsi, getreg16 (regsi) + 2);
        putreg16 (regdi, getreg16 (regdi) + 2);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opJo(){
    /* 70 JO Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (of) {
        ip = ip + temp16;
    }
}

void opJno(){
    /* 71 JNO Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (!of) {
        ip = ip + temp16;
    }
}

void opJb(){	/* 72 JB Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (cf) {
        ip = ip + temp16;
    }
}

void opJnb(){
    /* 73 JNB Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (!cf) {
        ip = ip + temp16;
    }
}

void opJz(){
    /* 74 JZ Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (zf) {
        ip = ip + temp16;
    }
}

void opJnz(){
    /* 75 JNZ Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (!zf) {
        ip = ip + temp16;
    }
}

void opJbe(){
    /* 76 JBE Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (cf || zf) {
        ip = ip + temp16;
    }
}

void opJa(){
    /* 77 JA Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (!cf && !zf) {
        ip = ip + temp16;
    }
}

void opJs(){
    /* 78 JS Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (sf) {
        ip = ip + temp16;
    }
}

void opJns(){
    /* 79 JNS Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (!sf) {
        ip = ip + temp16;
    }
}

void opJpe(){
    /* 7A JPE Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (pf) {
        ip = ip + temp16;
    }
}

void opJpo(){
    /* 7B JPO Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (!pf) {
        ip = ip + temp16;
    }
}

void opJl(){
    /* 7C JL Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (sf != of) {
        ip = ip + temp16;
    }
}

void opJge(){
    /* 7D JGE Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (sf == of) {
        ip = ip + temp16;
    }
}

void opJle(){
    /* 7E JLE Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if ( (sf != of) || zf) {
        ip = ip + temp16;
    }
}

void opJg(){
    /* 7F JG Jb */
    temp16 = signext (getmem8 (segregs[regcs], ip) );
    StepIP (1);
    if (!zf && (sf == of) ) {
        ip = ip + temp16;
    }
}

void opGrp1(){
    /* 80/82 GRP1 Eb Ib */
    modregrm();
    oper1b = readrm8 (rm);
    oper2b = getmem8 (segregs[regcs], ip);
    StepIP (1);
    switch (reg) {
    case 0:
        op_add8();
        break;
    case 1:
        op_or8();
        break;
    case 2:
        op_adc8();
        break;
    case 3:
        op_sbb8();
        break;
    case 4:
        op_and8();
        break;
    case 5:
        op_sub8();
        break;
    case 6:
        op_xor8();
        break;
    case 7:
        flag_sub8 (oper1b, oper2b);
        break;
    default:
        break;	/* to avoid compiler warnings */
    }

    if (reg < 7) {
        writerm8 (rm, res8);
    }
}

void opGrp1_2(){
    /* 81 GRP1 Ev Iv */
    /* 83 GRP1 Ev Ib */
    modregrm();
    oper1 = readrm16 (rm);
    if (opcode == 0x81) {
        oper2 = getmem16 (segregs[regcs], ip);
        StepIP (2);
    }
    else {
        oper2 = signext (getmem8 (segregs[regcs], ip) );
        StepIP (1);
    }

    switch (reg) {
    case 0:
        op_add16();
        break;
    case 1:
        op_or16();
        break;
    case 2:
        op_adc16();
        break;
    case 3:
        op_sbb16();
        break;
    case 4:
        op_and16();
        break;
    case 5:
        op_sub16();
        break;
    case 6:
        op_xor16();
        break;
    case 7:
        flag_sub16 (oper1, oper2);
        break;
    default:
        break;	/* to avoid compiler warnings */
    }

    if (reg < 7) {
        writerm16 (rm, res16);
    }
}

void opTest(){
    /* 84 TEST Gb Eb */
    modregrm();
    oper1b = getreg8 (reg);
    oper2b = readrm8 (rm);
    flag_log8 (oper1b & oper2b);
}

void opTest_2()
{
    /* 85 TEST Gv Ev */
    modregrm();
    oper1 = getreg16 (reg);
    oper2 = readrm16 (rm);
    flag_log16 (oper1 & oper2);
}

void opXchg(){
    /* 86 XCHG Gb Eb */
    modregrm();
    oper1b = getreg8 (reg);
    putreg8 (reg, readrm8 (rm) );
    writerm8 (rm, oper1b);
}

void opXchg_2(){
    /* 87 XCHG Gv Ev */
    modregrm();
    oper1 = getreg16 (reg);
    putreg16 (reg, readrm16 (rm) );
    writerm16 (rm, oper1);
}

void opMov8(){
    /* 88 MOV Eb Gb */
    modregrm();
    writerm8 (rm, getreg8 (reg) );
}

void opMov16(){
    /* 89 MOV Ev Gv */
    modregrm();
    writerm16 (rm, getreg16 (reg) );
}

void opMovReg8(){
    /* 8A MOV Gb Eb */
    modregrm();
    putreg8 (reg, readrm8 (rm) );
}

void opMovReg16(){
    /* 8B MOV Gv Ev */
    modregrm();
    putreg16 (reg, readrm16 (rm) );
}

void opMov16_2(){
    /* 8C MOV Ew Sw */
    modregrm();
    writerm16 (rm, getsegreg (reg) );
}

void opLea(){
    /* 8D LEA Gv M */
    modregrm();
    getea (rm);
    putreg16 (reg, ea - segbase (useseg) );
}

void opMov(){
    /* 8E MOV Sw Ew */
    modregrm();
    putsegreg (reg, readrm16 (rm) );
}

void opPop_5(){
    /* 8F POP Ev */
    modregrm();
    writerm16 (rm, pop() );
}

void opNop(){
    /* 90 NOP */
}

void opXchgEcxEax(){
    /* 91 XCHG eCX eAX */
    oper1 = getreg16 (regcx);
    putreg16 (regcx, getreg16 (regax) );
    putreg16 (regax, oper1);
}

void opXchgEdxEax(){
    /* 92 XCHG eDX eAX */
    oper1 = getreg16 (regdx);
    putreg16 (regdx, getreg16 (regax) );
    putreg16 (regax, oper1);
}

void opXchgEbxEax(){
    /* 93 XCHG eBX eAX */
    oper1 = getreg16 (regbx);
    putreg16 (regbx, getreg16 (regax) );
    putreg16 (regax, oper1);
}

void opXchgEspEax(){
    /* 94 XCHG eSP eAX */
    oper1 = getreg16 (regsp);
    putreg16 (regsp, getreg16 (regax) );
    putreg16 (regax, oper1);
}

void opXchgEbpEax(){
    /* 95 XCHG eBP eAX */
    oper1 = getreg16 (regbp);
    putreg16 (regbp, getreg16 (regax) );
    putreg16 (regax, oper1);
}

void opXchgEsiEax(){
    /* 96 XCHG eSI eAX */
    oper1 = getreg16 (regsi);
    putreg16 (regsi, getreg16 (regax) );
    putreg16 (regax, oper1);
}

void opXchgEdiEax(){
    /* 97 XCHG eDI eAX */
    oper1 = getreg16 (regdi);
    putreg16 (regdi, getreg16 (regax) );
    putreg16 (regax, oper1);
}

void opCbw(){
    /* 98 CBW */
    if ( (regs.byteregs[regal] & 0x80) == 0x80) {
        regs.byteregs[regah] = 0xFF;
    }
    else {
        regs.byteregs[regah] = 0;
    }
}

void opCwd(){
    /* 99 CWD */
    if ( (regs.byteregs[regah] & 0x80) == 0x80) {
        putreg16 (regdx, 0xFFFF);
    }
    else {
        putreg16 (regdx, 0);
    }
}

void opCall(){
    /* 9A CALL Ap */
    oper1 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    oper2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    push (segregs[regcs]);
    push (ip);
    ip = oper1;
    segregs[regcs] = oper2;
}

void opWait(){
    /* 9B WAIT */
}

void opPushF(){
    /* 9C PUSHF */
    push (makeflagsword() | 0xF800);
}

void opPopF(){
    /* 9D POPF */
    temp16 = pop();
    decodeflagsword (temp16);
}

void opSahF(){
    /* 9E SAHF */
    decodeflagsword ( (makeflagsword() & 0xFF00) | regs.byteregs[regah]);
}

void opLahF(){
    /* 9F LAHF */
    regs.byteregs[regah] = makeflagsword() & 0xFF;
}

void opMov_2(){
    /* A0 MOV regs.byteregs[regal] Ob */
    regs.byteregs[regal] = getmem8 (useseg, getmem16 (segregs[regcs], ip) );
    StepIP (2);
}

void opMovEaxOv(){
    /* A1 MOV eAX Ov */
    oper1 = getmem16 (useseg, getmem16 (segregs[regcs], ip) );
    StepIP (2);
    putreg16 (regax, oper1);
}

void opMovOb(){
    /* A2 MOV Ob regs.byteregs[regal] */
    putmem8 (useseg, getmem16 (segregs[regcs], ip), regs.byteregs[regal]);
    StepIP (2);
}

void opMovOvEax(){
    /* A3 MOV Ov eAX */
    putmem16 (useseg, getmem16 (segregs[regcs], ip), getreg16 (regax) );
    StepIP (2);
}

void opMovsb(){
    /* A4 MOVSB */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    putmem8 (segregs[reges], getreg16 (regdi), getmem8 (useseg, getreg16 (regsi) ) );
    if (df) {
        putreg16 (regsi, getreg16 (regsi) - 1);
        putreg16 (regdi, getreg16 (regdi) - 1);
    }
    else {
        putreg16 (regsi, getreg16 (regsi) + 1);
        putreg16 (regdi, getreg16 (regdi) + 1);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opMovsw(){
    /* A5 MOVSW */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    putmem16 (segregs[reges], getreg16 (regdi), getmem16 (useseg, getreg16 (regsi) ) );
    if (df) {
        putreg16 (regsi, getreg16 (regsi) - 2);
        putreg16 (regdi, getreg16 (regdi) - 2);
    }
    else {
        putreg16 (regsi, getreg16 (regsi) + 2);
        putreg16 (regdi, getreg16 (regdi) + 2);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opCmpsb(){
    /* A6 CMPSB */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    oper1b = getmem8 (useseg, getreg16 (regsi) );
    oper2b = getmem8 (segregs[reges], getreg16 (regdi) );
    if (df) {
        putreg16 (regsi, getreg16 (regsi) - 1);
        putreg16 (regdi, getreg16 (regdi) - 1);
    }
    else {
        putreg16 (regsi, getreg16 (regsi) + 1);
        putreg16 (regdi, getreg16 (regdi) + 1);
    }

    flag_sub8 (oper1b, oper2b);
    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    if ( (reptype == 1) && !zf) {
        return;
    }
    else if ( (reptype == 2) && (zf == 1) ) {
        return;
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opCmpsw(){
    /* A7 CMPSW */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    oper1 = getmem16 (useseg, getreg16 (regsi) );
    oper2 = getmem16 (segregs[reges], getreg16 (regdi) );
    if (df) {
        putreg16 (regsi, getreg16 (regsi) - 2);
        putreg16 (regdi, getreg16 (regdi) - 2);
    }
    else {
        putreg16 (regsi, getreg16 (regsi) + 2);
        putreg16 (regdi, getreg16 (regdi) + 2);
    }

    flag_sub16 (oper1, oper2);
    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    if ( (reptype == 1) && !zf) {
        return;
    }

    if ( (reptype == 2) && (zf == 1) ) {
        return;
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opTest_3(){
    /* A8 TEST regs.byteregs[regal] Ib */
    oper1b = regs.byteregs[regal];
    oper2b = getmem8 (segregs[regcs], ip);
    StepIP (1);
    flag_log8 (oper1b & oper2b);
}

void opTestEax(){
    /* A9 TEST eAX Iv */
    oper1 = getreg16 (regax);
    oper2 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    flag_log16 (oper1 & oper2);
}

void opStosb(){
    /* AA STOSB */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    putmem8 (segregs[reges], getreg16 (regdi), regs.byteregs[regal]);
    if (df) {
        putreg16 (regdi, getreg16 (regdi) - 1);
    }
    else {
        putreg16 (regdi, getreg16 (regdi) + 1);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void cmpStosw(){
    /* AB STOSW */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    putmem16 (segregs[reges], getreg16 (regdi), getreg16 (regax) );
    if (df) {
        putreg16 (regdi, getreg16 (regdi) - 2);
    }
    else {
        putreg16 (regdi, getreg16 (regdi) + 2);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opLodsb(){
    /* AC LODSB */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    regs.byteregs[regal] = getmem8 (useseg, getreg16 (regsi) );
    if (df) {
        putreg16 (regsi, getreg16 (regsi) - 1);
    }
    else {
        putreg16 (regsi, getreg16 (regsi) + 1);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opLodsw(){
    /* AD LODSW */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    oper1 = getmem16 (useseg, getreg16 (regsi) );
    putreg16 (regax, oper1);
    if (df) {
        putreg16 (regsi, getreg16 (regsi) - 2);
    }
    else {
        putreg16 (regsi, getreg16 (regsi) + 2);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opScasb(){
    /* AE SCASB */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    oper1b = getmem8 (segregs[reges], getreg16 (regdi) );
    oper2b = regs.byteregs[regal];
    flag_sub8 (oper1b, oper2b);
    if (df) {
        putreg16 (regdi, getreg16 (regdi) - 1);
    }
    else {
        putreg16 (regdi, getreg16 (regdi) + 1);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    if ( (reptype == 1) && !zf) {
        return;
    }
    else if ( (reptype == 2) && (zf == 1) ) {
        return;
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opScasw(){
    /* AF SCASW */
    if (reptype && (getreg16 (regcx) == 0) ) {
        return;
    }

    oper1 = getmem16 (segregs[reges], getreg16 (regdi) );
    oper2 = getreg16 (regax);
    flag_sub16 (oper1, oper2);
    if (df) {
        putreg16 (regdi, getreg16 (regdi) - 2);
    }
    else {
        putreg16 (regdi, getreg16 (regdi) + 2);
    }

    if (reptype) {
        putreg16 (regcx, getreg16 (regcx) - 1);
    }

    if ( (reptype == 1) && !zf) {
        return;
    }
    else if ( (reptype == 2) & (zf == 1) ) {
        return;
    }

    totalexec++;
    loopcount++;
    if (!reptype) {
        return;
    }

    ip = firstip;
}

void opMovAlIv(){
    /* B0 MOV regs.byteregs[regal] Ib */
    regs.byteregs[regal] = getmem8 (segregs[regcs], ip);
    StepIP (1);
}

void opMovClIb(){
    /* B1 MOV regs.byteregs[regcl] Ib */
    regs.byteregs[regcl] = getmem8 (segregs[regcs], ip);
    StepIP (1);
}

void opMovDlIb(){
    /* B2 MOV regs.byteregs[regdl] Ib */
    regs.byteregs[regdl] = getmem8 (segregs[regcs], ip);
    StepIP (1);
}

void opMovBlIb(){
    /* B3 MOV regs.byteregs[regbl] Ib */
    regs.byteregs[regbl] = getmem8 (segregs[regcs], ip);
    StepIP (1);
}

void opMovAhIb(){
    /* B4 MOV regs.byteregs[regah] Ib */
    regs.byteregs[regah] = getmem8 (segregs[regcs], ip);
    StepIP (1);
}

void opMovChIb(){
    /* B5 MOV regs.byteregs[regch] Ib */
    regs.byteregs[regch] = getmem8 (segregs[regcs], ip);
    StepIP (1);
}

void opMovDhIb(){
    /* B6 MOV regs.byteregs[regdh] Ib */
    regs.byteregs[regdh] = getmem8 (segregs[regcs], ip);
    StepIP (1);
}

void opMovBhIb(){
    /* B7 MOV regs.byteregs[regbh] Ib */
    regs.byteregs[regbh] = getmem8 (segregs[regcs], ip);
    StepIP (1);
}

void opMovEaxIv(){
    /* B8 MOV eAX Iv */
    oper1 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    putreg16 (regax, oper1);
}

void opMovEcxIv(){
    /* B9 MOV eCX Iv */
    oper1 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    putreg16 (regcx, oper1);
}

void opMovEdxIv(){
    /* BA MOV eDX Iv */
    oper1 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    putreg16 (regdx, oper1);
}

void opMovEbxIv(){
    /* BB MOV eBX Iv */
    oper1 = getmem16 (segregs[regcs], ip);
    StepIP (2);
    putreg16 (regbx, oper1);
}

void opMovEspIv(){
    /* BC MOV eSP Iv */
    putreg16 (regsp, getmem16 (segregs[regcs], ip) );
    StepIP (2);
}

void opMovEbpIv(){
    /* BD MOV eBP Iv */
    putreg16 (regbp, getmem16 (segregs[regcs], ip) );
    StepIP (2);
}

void opMovEsiIv(){
    /* BE MOV eSI Iv */
    putreg16 (regsi, getmem16 (segregs[regcs], ip) );
    StepIP (2);
}

void opMovEdiIv(){
    /* BF MOV eDI Iv */
    putreg16 (regdi, getmem16 (segregs[regcs], ip) );
    StepIP (2);
}


void initOpcodeTable() {
    opcodeTable[0x00] = &opAdd8;
    opcodeTable[0x01] = &opAdd16;
    opcodeTable[0x02] = &opAdd8_2;
    opcodeTable[0x03] = &opAdd16_2;
    opcodeTable[0x04] = &opAdd8_3;
    opcodeTable[0x05] = &opAdd16_3;

    opcodeTable[0x06] = &opPush;
    opcodeTable[0x07] = &opPop;

    opcodeTable[0x08] = &opOr8;
    opcodeTable[0x09] = &opOr16;
    opcodeTable[0x0A] = &opOr8_2;
    opcodeTable[0x0B] = &opOr16_2;
    opcodeTable[0x0C] = &opOr8_3;
    opcodeTable[0x0D] = &opOr16_3;

    opcodeTable[0x0E] = &opPush_2;
    opcodeTable[0x0F] = &opPop_2;

    opcodeTable[0x10] = &opAdc8;
    opcodeTable[0x11] = &opAdc16;
    opcodeTable[0x12] = &opAdc8_2;
    opcodeTable[0x13] = &opAdc16_2;
    opcodeTable[0x14] = &opAdc8_3;
    opcodeTable[0x15] = &opAdc16_3;

    opcodeTable[0x16] = &opPush_3;
    opcodeTable[0x17] = &opPop_3;

    opcodeTable[0x18] = &opSbb8;
    opcodeTable[0x19] = &opSbb16;
    opcodeTable[0x1A] = &opSbb8_2;
    opcodeTable[0x1B] = &opSbb16_2;
    opcodeTable[0x1C] = &opSbb8_3;
    opcodeTable[0x1D] = &opSbb16_3;

    opcodeTable[0x1E] = &opPush_4;
    opcodeTable[0x1F] = &opPop_4;

    opcodeTable[0x20] = &opAnd8;
    opcodeTable[0x21] = &opAnd16;
    opcodeTable[0x22] = &opAnd8_2;
    opcodeTable[0x23] = &opAnd16_2;
    opcodeTable[0x24] = &opAnd8_3;
    opcodeTable[0x25] = &opAnd16_3;

    opcodeTable[0x27] = &opDaa;

    opcodeTable[0x28] = &opSub8;
    opcodeTable[0x29] = &opSub16;
    opcodeTable[0x2A] = &opSub8_2;
    opcodeTable[0x2B] = &opSub16_2;
    opcodeTable[0x2C] = &opSub8_3;
    opcodeTable[0x2D] = &opSub16_3;

    opcodeTable[0x2F] = &opDas;

    opcodeTable[0x30] = &opXor8;
    opcodeTable[0x31] = &opXor16;
    opcodeTable[0x32] = &opXor8_2;
    opcodeTable[0x33] = &opXor16_2;
    opcodeTable[0x34] = &opXor8_3;
    opcodeTable[0x35] = &opXor16_3;

    opcodeTable[0x37] = &opAaa;

    opcodeTable[0x38] = &opCmp8;
    opcodeTable[0x39] = &opCmp16;
    opcodeTable[0x3A] = &opCmp8_2;
    opcodeTable[0x3B] = &opCmp16_2;
    opcodeTable[0x3C] = &opCmp8_3;
    opcodeTable[0x3D] = &opCmp16_3;

    opcodeTable[0x3F] = &opAas;

    opcodeTable[0x40] = &opIncEax;
    opcodeTable[0x41] = &opIncEcx;
    opcodeTable[0x42] = &opIncEdx;
    opcodeTable[0x43] = &opIncEbx;
    opcodeTable[0x44] = &opIncEsp;
    opcodeTable[0x45] = &opIncEbp;
    opcodeTable[0x46] = &opIncEsi;
    opcodeTable[0x47] = &opIncEdi;

    opcodeTable[0x48] = &opDecEax;
    opcodeTable[0x49] = &opDecEcx;
    opcodeTable[0x4A] = &opDecEdx;
    opcodeTable[0x4B] = &opDecEbx;
    opcodeTable[0x4C] = &opDecEsp;
    opcodeTable[0x4D] = &opDecEbp;
    opcodeTable[0x4E] = &opDecEsi;
    opcodeTable[0x4F] = &opDecEdi;

    opcodeTable[0x50] = &opPushEax;
    opcodeTable[0x51] = &opPushEcx;
    opcodeTable[0x52] = &opPushEdx;
    opcodeTable[0x53] = &opPushEbx;
    opcodeTable[0x54] = &opPushEsp;
    opcodeTable[0x55] = &opPushEbp;
    opcodeTable[0x56] = &opPushEsi;
    opcodeTable[0x57] = &opPushEdi;

    opcodeTable[0x58] = &opPopEax;
    opcodeTable[0x59] = &opPopEcx;
    opcodeTable[0x5A] = &opPopEdx;
    opcodeTable[0x5B] = &opPopEbx;
    opcodeTable[0x5C] = &opPopEsp;
    opcodeTable[0x5D] = &opPopEbp;
    opcodeTable[0x5E] = &opPopEsi;
    opcodeTable[0x5F] = &opPopEdi;

    opcodeTable[0x60] = &opPushA;
    opcodeTable[0x61] = &opPopA;

    opcodeTable[0x62] = &opBound;
    opcodeTable[0x68] = &opPushIv;
    opcodeTable[0x69] = &opImul;
    opcodeTable[0x6A] = &opPushIb;
    opcodeTable[0x6B] = &opImul_2;
    opcodeTable[0x6C] = &opInsb;
    opcodeTable[0x6D] = &opInsw;
    opcodeTable[0x6E] = &opOutsb;
    opcodeTable[0x6F] = &opOutsw;

    opcodeTable[0x70] = &opJo;
    opcodeTable[0x71] = &opJno;
    opcodeTable[0x72] = &opJb;
    opcodeTable[0x73] = &opJnb;
    opcodeTable[0x74] = &opJz;
    opcodeTable[0x75] = &opJnz;
    opcodeTable[0x76] = &opJbe;
    opcodeTable[0x77] = &opJa;
    opcodeTable[0x78] = &opJs;
    opcodeTable[0x79] = &opJns;
    opcodeTable[0x7A] = &opJpe;
    opcodeTable[0x7B] = &opJpo;
    opcodeTable[0x7C] = &opJl;
    opcodeTable[0x7D] = &opJge;
    opcodeTable[0x7E] = &opJle;
    opcodeTable[0x7F] = &opJg;

    opcodeTable[0x80] = &opGrp1;
    opcodeTable[0x81] = &opGrp1_2;
    opcodeTable[0x82] = &opGrp1;
    opcodeTable[0x83] = &opGrp1_2;
    opcodeTable[0x84] = &opTest;
    opcodeTable[0x85] = &opTest_2;
    opcodeTable[0x86] = &opXchg;
    opcodeTable[0x87] = &opXchg_2;

    opcodeTable[0x88] = &opMov8;
    opcodeTable[0x89] = &opMov16;
    opcodeTable[0x8A] = &opMovReg8;
    opcodeTable[0x8B] = &opMovReg16;
    opcodeTable[0x8C] = &opMov16_2;
    opcodeTable[0x8D] = &opLea;
    opcodeTable[0x8E] = &opMov;
    opcodeTable[0x8F] = &opPop_5;
    opcodeTable[0x90] = &opNop;

    opcodeTable[0x91] = &opXchgEcxEax;
    opcodeTable[0x92] = &opXchgEdxEax;
    opcodeTable[0x93] = &opXchgEbxEax;
    opcodeTable[0x94] = &opXchgEspEax;
    opcodeTable[0x95] = &opXchgEbpEax;
    opcodeTable[0x96] = &opXchgEsiEax;
    opcodeTable[0x97] = &opXchgEdiEax;

    opcodeTable[0x98] = &opCbw;
    opcodeTable[0x99] = &opCwd;
    opcodeTable[0x9A] = &opCall;
    opcodeTable[0x9B] = &opWait;
    opcodeTable[0x9C] = &opPushF;
    opcodeTable[0x9D] = &opPopF;
    opcodeTable[0x9E] = &opSahF;
    opcodeTable[0x9F] = &opLahF;
    opcodeTable[0xA0] = &opMov_2;
    opcodeTable[0xA1] = &opMovEaxOv;
    opcodeTable[0xA2] = &opMovOb;
    opcodeTable[0xA3] = &opMovOvEax;
    opcodeTable[0xA4] = &opMovsb;
    opcodeTable[0xA5] = &opMovsw;

    opcodeTable[0xA6] = &opCmpsb;
    opcodeTable[0xA7] = &opCmpsw;
    opcodeTable[0xA8] = &opTest_3;
    opcodeTable[0xA9] = &opTestEax;
    opcodeTable[0xAA] = &opStosb;
    opcodeTable[0xAB] = &cmpStosw;
    opcodeTable[0xAC] = &opLodsb;
    opcodeTable[0xAD] = &opLodsw;
    opcodeTable[0xAE] = &opScasb;
    opcodeTable[0xAF] = &opScasw;

    opcodeTable[0xB0] = &opMovAlIv;
    opcodeTable[0xB1] = &opMovClIb;
    opcodeTable[0xB2] = &opMovDlIb;
    opcodeTable[0xB3] = &opMovBlIb;
    opcodeTable[0xB4] = &opMovAhIb;
    opcodeTable[0xB5] = &opMovChIb;
    opcodeTable[0xB6] = &opMovDhIb;
    opcodeTable[0xB7] = &opMovBhIb;
    opcodeTable[0xB8] = &opMovEaxIv;
    opcodeTable[0xB9] = &opMovEcxIv;
    opcodeTable[0xBA] = &opMovEdxIv;
    opcodeTable[0xBB] = &opMovEbxIv;
    opcodeTable[0xBC] = &opMovEspIv;
    opcodeTable[0xBD] = &opMovEbpIv;
    opcodeTable[0xBE] = &opMovEsiIv;
    opcodeTable[0xBF] = &opMovEdiIv;
}

void initLookupTables() {
    initOpcodeTable();
}

void exec86 (uint32_t execloops) {
    counterticks = (uint64_t) ( (double) timerfreq / (double) 65536.0);

    for (loopcount = 0; loopcount < execloops; loopcount++) {

        if ( (totalexec & 31) == 0) {
            timing();
        }
        if (trap_toggle) {
            intcall86 (1);
        }
        if (tf) {
            trap_toggle = 1;
        } else {
            trap_toggle = 0;
        }
        if (!trap_toggle && (ifl && (i8259.irr & (~i8259.imr) ) ) ) {
            intcall86 (nextintr() );	/* get next interrupt from the i8259, if any */
        }

        reptype = 0;
        segoverride = 0;
        useseg = segregs[regds];
        docontinue = 0;
        firstip = ip;

        if ( (segregs[regcs] == 0xF000) && (ip == 0xE066) )
            didbootstrap = 0; //detect if we hit the BIOS entry point to clear didbootstrap because we've rebooted

        while (!docontinue) {
            segregs[regcs] = segregs[regcs] & 0xFFFF;
            ip = ip & 0xFFFF;
            savecs = segregs[regcs];
            saveip = ip;
            opcode = getmem8 (segregs[regcs], ip);
            StepIP (1);

            switch (opcode) {
            /* segment prefix check */
            case 0x2E:	/* segment segregs[regcs] */
                useseg = segregs[regcs];
                segoverride = 1;
                break;

            case 0x3E:	/* segment segregs[regds] */
                useseg = segregs[regds];
                segoverride = 1;
                break;

            case 0x26:	/* segment segregs[reges] */
                useseg = segregs[reges];
                segoverride = 1;
                break;

            case 0x36:	/* segment segregs[regss] */
                useseg = segregs[regss];
                segoverride = 1;
                break;

                /* repetition prefix check */
            case 0xF3:	/* REP/REPE/REPZ */
                reptype = 1;
                break;

            case 0xF2:	/* REPNE/REPNZ */
                reptype = 2;
                break;

            default:
                docontinue = 1;
                break;
            }
        }

        totalexec++;

        /*todo remove*/
        if(opcodeTable[opcode] != NULL) {
            opcodeTable[opcode]();
        }
        switch (opcode) {

        case 0xC0:	/* C0 GRP2 byte imm8 (80186+) */
            modregrm();
            oper1b = readrm8 (rm);
            oper2b = getmem8 (segregs[regcs], ip);
            StepIP (1);
            writerm8 (rm, op_grp2_8 (oper2b) );
            break;

        case 0xC1:	/* C1 GRP2 word imm8 (80186+) */
            modregrm();
            oper1 = readrm16 (rm);
            oper2 = getmem8 (segregs[regcs], ip);
            StepIP (1);
            writerm16 (rm, op_grp2_16 ( (uint8_t) oper2) );
            break;

        case 0xC2:	/* C2 RET Iw */
            oper1 = getmem16 (segregs[regcs], ip);
            ip = pop();
            putreg16 (regsp, getreg16 (regsp) + oper1);
            break;

        case 0xC3:	/* C3 RET */
            ip = pop();
            break;

        case 0xC4:	/* C4 LES Gv Mp */
            modregrm();
            getea (rm);
            putreg16 (reg, read86 (ea) + read86 (ea + 1) * 256);
            segregs[reges] = read86 (ea + 2) + read86 (ea + 3) * 256;
            break;

        case 0xC5:	/* C5 LDS Gv Mp */
            modregrm();
            getea (rm);
            putreg16 (reg, read86 (ea) + read86 (ea + 1) * 256);
            segregs[regds] = read86 (ea + 2) + read86 (ea + 3) * 256;
            break;

        case 0xC6:	/* C6 MOV Eb Ib */
            modregrm();
            writerm8 (rm, getmem8 (segregs[regcs], ip) );
            StepIP (1);
            break;

        case 0xC7:	/* C7 MOV Ev Iv */
            modregrm();
            writerm16 (rm, getmem16 (segregs[regcs], ip) );
            StepIP (2);
            break;

        case 0xC8:	/* C8 ENTER (80186+) */
            stacksize = getmem16 (segregs[regcs], ip);
            StepIP (2);
            nestlev = getmem8 (segregs[regcs], ip);
            StepIP (1);
            push (getreg16 (regbp) );
            frametemp = getreg16 (regsp);
            if (nestlev) {
                for (temp16 = 1; temp16 < nestlev; temp16++) {
                    putreg16 (regbp, getreg16 (regbp) - 2);
                    push (getreg16 (regbp) );
                }

                push (getreg16 (regsp) );
            }

            putreg16 (regbp, frametemp);
            putreg16 (regsp, getreg16 (regbp) - stacksize);

            break;

        case 0xC9:	/* C9 LEAVE (80186+) */
            putreg16 (regsp, getreg16 (regbp) );
            putreg16 (regbp, pop() );

            break;

        case 0xCA:	/* CA RETF Iw */
            oper1 = getmem16 (segregs[regcs], ip);
            ip = pop();
            segregs[regcs] = pop();
            putreg16 (regsp, getreg16 (regsp) + oper1);
            break;

        case 0xCB:	/* CB RETF */
            ip = pop();;
            segregs[regcs] = pop();
            break;

        case 0xCC:	/* CC INT 3 */
            intcall86 (3);
            break;

        case 0xCD:	/* CD INT Ib */
            oper1b = getmem8 (segregs[regcs], ip);
            StepIP (1);
            intcall86 (oper1b);
            break;

        case 0xCE:	/* CE INTO */
            if (of) {
                intcall86 (4);
            }
            break;

        case 0xCF:	/* CF IRET */
            ip = pop();
            segregs[regcs] = pop();
            decodeflagsword (pop() );

            /*
                         * if (net.enabled) net.canrecv = 1;
                         */
            break;

        case 0xD0:	/* D0 GRP2 Eb 1 */
            modregrm();
            oper1b = readrm8 (rm);
            writerm8 (rm, op_grp2_8 (1) );
            break;

        case 0xD1:	/* D1 GRP2 Ev 1 */
            modregrm();
            oper1 = readrm16 (rm);
            writerm16 (rm, op_grp2_16 (1) );
            break;

        case 0xD2:	/* D2 GRP2 Eb regs.byteregs[regcl] */
            modregrm();
            oper1b = readrm8 (rm);
            writerm8 (rm, op_grp2_8 (regs.byteregs[regcl]) );
            break;

        case 0xD3:	/* D3 GRP2 Ev regs.byteregs[regcl] */
            modregrm();
            oper1 = readrm16 (rm);
            writerm16 (rm, op_grp2_16 (regs.byteregs[regcl]) );
            break;

        case 0xD4:	/* D4 AAM I0 */
            oper1 = getmem8 (segregs[regcs], ip);
            StepIP (1);
            if (!oper1) {
                intcall86 (0);
                break;
            }	/* division by zero */

            regs.byteregs[regah] = (regs.byteregs[regal] / oper1) & 255;
            regs.byteregs[regal] = (regs.byteregs[regal] % oper1) & 255;
            flag_szp16 (getreg16 (regax) );
            break;

        case 0xD5:	/* D5 AAD I0 */
            oper1 = getmem8 (segregs[regcs], ip);
            StepIP (1);
            regs.byteregs[regal] = (regs.byteregs[regah] * oper1 + regs.byteregs[regal]) & 255;
            regs.byteregs[regah] = 0;
            flag_szp16 (regs.byteregs[regah] * oper1 + regs.byteregs[regal]);
            sf = 0;
            break;

        case 0xD6:	/* D6 XLAT on V20/V30, SALC on 8086/8088 */
#ifndef CPU_V20
            regs.byteregs[regal] = cf ? 0xFF : 0x00;
            break;
#endif

        case 0xD7:	/* D7 XLAT */
            regs.byteregs[regal] = read86(useseg * 16 + (regs.wordregs[regbx]) + regs.byteregs[regal]);
            break;

        case 0xD8:
        case 0xD9:
        case 0xDA:
        case 0xDB:
        case 0xDC:
        case 0xDE:
        case 0xDD:
        case 0xDF:	/* escape to x87 FPU (unsupported) */
            modregrm();
            break;

        case 0xE0:	/* E0 LOOPNZ Jb */
            temp16 = signext (getmem8 (segregs[regcs], ip) );
            StepIP (1);
            putreg16 (regcx, getreg16 (regcx) - 1);
            if ( (getreg16 (regcx) ) && !zf) {
                ip = ip + temp16;
            }
            break;

        case 0xE1:	/* E1 LOOPZ Jb */
            temp16 = signext (getmem8 (segregs[regcs], ip) );
            StepIP (1);
            putreg16 (regcx, (getreg16 (regcx) ) - 1);
            if ( (getreg16 (regcx) ) && (zf == 1) ) {
                ip = ip + temp16;
            }
            break;

        case 0xE2:	/* E2 LOOP Jb */
            temp16 = signext (getmem8 (segregs[regcs], ip) );
            StepIP (1);
            putreg16 (regcx, (getreg16 (regcx) ) - 1);
            if (getreg16 (regcx) ) {
                ip = ip + temp16;
            }
            break;

        case 0xE3:	/* E3 JCXZ Jb */
            temp16 = signext (getmem8 (segregs[regcs], ip) );
            StepIP (1);
            if (! (getreg16 (regcx) ) ) {
                ip = ip + temp16;
            }
            break;

        case 0xE4:	/* E4 IN regs.byteregs[regal] Ib */
            oper1b = getmem8 (segregs[regcs], ip);
            StepIP (1);
            regs.byteregs[regal] = (uint8_t) portin (oper1b);
            break;

        case 0xE5:	/* E5 IN eAX Ib */
            oper1b = getmem8 (segregs[regcs], ip);
            StepIP (1);
            putreg16 (regax, portin16 (oper1b) );
            break;

        case 0xE6:	/* E6 OUT Ib regs.byteregs[regal] */
            oper1b = getmem8 (segregs[regcs], ip);
            StepIP (1);
            portout (oper1b, regs.byteregs[regal]);
            break;

        case 0xE7:	/* E7 OUT Ib eAX */
            oper1b = getmem8 (segregs[regcs], ip);
            StepIP (1);
            portout16 (oper1b, (getreg16 (regax) ) );
            break;

        case 0xE8:	/* E8 CALL Jv */
            oper1 = getmem16 (segregs[regcs], ip);
            StepIP (2);
            push (ip);
            ip = ip + oper1;
            break;

        case 0xE9:	/* E9 JMP Jv */
            oper1 = getmem16 (segregs[regcs], ip);
            StepIP (2);
            ip = ip + oper1;
            break;

        case 0xEA:	/* EA JMP Ap */
            oper1 = getmem16 (segregs[regcs], ip);
            StepIP (2);
            oper2 = getmem16 (segregs[regcs], ip);
            ip = oper1;
            segregs[regcs] = oper2;
            break;

        case 0xEB:	/* EB JMP Jb */
            oper1 = signext (getmem8 (segregs[regcs], ip) );
            StepIP (1);
            ip = ip + oper1;
            break;

        case 0xEC:	/* EC IN regs.byteregs[regal] regdx */
            oper1 = (getreg16 (regdx) );
            regs.byteregs[regal] = (uint8_t) portin (oper1);
            break;

        case 0xED:	/* ED IN eAX regdx */
            oper1 = (getreg16 (regdx) );
            putreg16 (regax, portin16 (oper1) );
            break;

        case 0xEE:	/* EE OUT regdx regs.byteregs[regal] */
            oper1 = (getreg16 (regdx) );
            portout (oper1, regs.byteregs[regal]);
            break;

        case 0xEF:	/* EF OUT regdx eAX */
            oper1 = (getreg16 (regdx) );
            portout16 (oper1, (getreg16 (regax) ) );
            break;

        case 0xF0:	/* F0 LOCK */
            break;

        case 0xF4:	/* F4 HLT */
            ip--;
            break;

        case 0xF5:	/* F5 CMC */
            if (!cf) {
                cf = 1;
            }
            else {
                cf = 0;
            }
            break;

        case 0xF6:	/* F6 GRP3a Eb */
            modregrm();
            oper1b = readrm8 (rm);
            op_grp3_8();
            if ( (reg > 1) && (reg < 4) ) {
                writerm8 (rm, res8);
            }
            break;

        case 0xF7:	/* F7 GRP3b Ev */
            modregrm();
            oper1 = readrm16 (rm);
            op_grp3_16();
            if ( (reg > 1) && (reg < 4) ) {
                writerm16 (rm, res16);
            }
            break;

        case 0xF8:	/* F8 CLC */
            cf = 0;
            break;

        case 0xF9:	/* F9 STC */
            cf = 1;
            break;

        case 0xFA:	/* FA CLI */
            ifl = 0;
            break;

        case 0xFB:	/* FB STI */
            ifl = 1;
            break;

        case 0xFC:	/* FC CLD */
            df = 0;
            break;

        case 0xFD:	/* FD STD */
            df = 1;
            break;

        case 0xFE:	/* FE GRP4 Eb */
            modregrm();
            oper1b = readrm8 (rm);
            oper2b = 1;
            if (!reg) {
                tempcf = cf;
                res8 = oper1b + oper2b;
                flag_add8 (oper1b, oper2b);
                cf = tempcf;
                writerm8 (rm, res8);
            }
            else {
                tempcf = cf;
                res8 = oper1b - oper2b;
                flag_sub8 (oper1b, oper2b);
                cf = tempcf;
                writerm8 (rm, res8);
            }
            break;

        case 0xFF:	/* FF GRP5 Ev */
            modregrm();
            oper1 = readrm16 (rm);
            op_grp5();
            break;
        }

        if (!running) {
            return;
        }
    }
}

