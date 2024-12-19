/*
 * nema is a Wii U Game Pad UIC emulator written in C++.
 * Copyright (C) 2024  noumidev
 */

#include "hw/cpu.hpp"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <type_traits>

#include "common/types.hpp"

#include "sys/memory.hpp"

using namespace common;

using sys::memory::read;
using sys::memory::write;

namespace hw::cpu {

constexpr int SIZE_INSTRUCTION_TABLE = 0x100;

typedef void (*InstructionHandler)(const u8);
typedef InstructionHandler InstructionTable[SIZE_INSTRUCTION_TABLE];

constexpr u32 INITIAL_PC = 0x8000;

enum class AddressingMode : int {
    DirectByte,
    DirectLong,
    DirectExtended,
    DirectIndexedX,
    DirectIndexedY,
    DirectIndexedShortX,
    DirectIndexedShortY,
    DirectIndexedShortSP,
    Immediate,
    SP,
    A,
    X,
    XL,
    Y,
    YL,
};

constexpr bool isRegister(const AddressingMode mode) {
    return (int)mode >= (int)AddressingMode::SP;
}

enum class Condition {
    True,
    Equal,
    NotEqual,
    Carry,
    NotCarry,
};

struct Context {
    u32 pc, cpc;
    u8 a;
    u16 x, y;
    u16 sp;

    union CCR {
        u8 raw;

        struct {
            u8 c : 1;
            u8 z : 1;
            u8 n : 1;
            u8 i0 : 1;
            u8 h : 1;
            u8 i1 : 1;
            u8 : 1;
            u8 v : 1;
        };
    } ccr;

    u32 ea;

    void setA(const u8 data) {
        a = data;

        std::printf("[  CPU  ] A = %02X\n", data);
    }

    void setX(const u16 data) {
        x = data;

        std::printf("[  CPU  ] X = %04X\n", data);
    }

    void setY(const u16 data) {
        y = data;

        std::printf("[  CPU  ] Y = %04X\n", data);
    }

    void setSp(const u16 data) {
        sp = data;

        std::printf("[  CPU  ] SP = %04X\n", data);
    }

    void setPc(const u32 addr) {
        pc = (addr & 0xFFFFFF);

        std::printf("[  CPU  ] Jump to %06X\n", addr);
    }
} ctx;

InstructionTable instrTable, precodeTable, prefix72Table;

// Instruction handlers
template<AddressingMode dstMode, AddressingMode srcMode, bool word>
void ADD(const u8);
template<AddressingMode mode>
void AND(const u8);
template<AddressingMode mode>
void BCP(const u8);
void BRES(const u8);
void BSET(const u8);
template<AddressingMode mode>
void CALL(const u8);
template<AddressingMode mode, bool word>
void CLR(const u8);
template<AddressingMode dstMode, AddressingMode srcMode, bool word>
void CP(const u8);
template<AddressingMode mode, bool word>
void CPL(const u8);
template<AddressingMode mode, bool word>
void DEC(const u8);
template<AddressingMode mode, bool word>
void INC(const u8);
void INT(const u8);
void JPF(const u8);
template<Condition cc>
void JRcc(const u8);
template<AddressingMode dstMode, AddressingMode srcMode, bool word>
void LD(const u8);
template<AddressingMode mode>
void OR(const u8);
template<AddressingMode mode, bool word>
void POP(const u8);
void PRECODE(const u8);
void PREFIX72(const u8);
template<AddressingMode mode, bool word>
void PUSH(const u8);
void RET(const u8);
template<AddressingMode mode>
void RLWA(const u8);
template<AddressingMode mode, bool word>
void RRC(const u8);
template<AddressingMode mode>
void RRWA(const u8);
void SIM(const u8);
template<AddressingMode mode, bool word>
void SRL(const u8);
template<AddressingMode dstMode, AddressingMode srcMode, bool word>
void SUB(const u8);
template<AddressingMode mode, bool word>
void TNZ(const u8);
template<AddressingMode mode>
void XOR(const u8);

void UNIMPLEMENTED(const u8);
void UNIMPLEMENTED_PRECODE(const u8);
void UNIMPLEMENTED_PREFIX72(const u8);

void initializeTables() {
    for (auto &i : instrTable) {
        i = UNIMPLEMENTED;
    }

    instrTable[0x01] = RRWA<AddressingMode::X>;
    instrTable[0x02] = RLWA<AddressingMode::X>;
    instrTable[0x04] = SRL<AddressingMode::DirectIndexedShortSP, 0>;
    instrTable[0x06] = RRC<AddressingMode::DirectIndexedShortSP, 0>;
    instrTable[0x0C] = INC<AddressingMode::DirectIndexedShortSP, 0>;
    instrTable[0x0F] = CLR<AddressingMode::DirectIndexedShortSP, 0>;
    instrTable[0x11] = CP<AddressingMode::A, AddressingMode::DirectIndexedShortSP, 0>;
    instrTable[0x13] = CP<AddressingMode::X, AddressingMode::DirectIndexedShortSP, 1>;
    instrTable[0x18] = XOR<AddressingMode::DirectIndexedShortSP>;
    instrTable[0x1A] = OR<AddressingMode::DirectIndexedShortSP>;
    instrTable[0x1B] = ADD<AddressingMode::A, AddressingMode::DirectIndexedShortSP, 0>;
    instrTable[0x1C] = ADD<AddressingMode::X, AddressingMode::Immediate, 1>;
    instrTable[0x1E] = LD<AddressingMode::X, AddressingMode::DirectIndexedShortSP, 1>;
    instrTable[0x1F] = LD<AddressingMode::DirectIndexedShortSP, AddressingMode::X, 1>;
    instrTable[0x20] = JRcc<Condition::True>;
    instrTable[0x24] = JRcc<Condition::NotCarry>;
    instrTable[0x25] = JRcc<Condition::Carry>;
    instrTable[0x26] = JRcc<Condition::NotEqual>;
    instrTable[0x27] = JRcc<Condition::Equal>;
    instrTable[0x43] = CPL<AddressingMode::A, 0>;
    instrTable[0x4A] = DEC<AddressingMode::A, 0>;
    instrTable[0x4B] = PUSH<AddressingMode::Immediate, 0>;
    instrTable[0x4D] = TNZ<AddressingMode::A, 0>;
    instrTable[0x52] = SUB<AddressingMode::SP, AddressingMode::Immediate, 1>;
    instrTable[0x54] = SRL<AddressingMode::X, 1>;
    instrTable[0x5B] = ADD<AddressingMode::SP, AddressingMode::Immediate, 1>;
    instrTable[0x5C] = INC<AddressingMode::X, 1>;
    instrTable[0x5F] = CLR<AddressingMode::X, 1>;
    instrTable[0x6B] = LD<AddressingMode::DirectIndexedShortSP, AddressingMode::A, 0>;
    instrTable[0x72] = PREFIX72;
    instrTable[0x7B] = LD<AddressingMode::A, AddressingMode::DirectIndexedShortSP, 0>;
    instrTable[0x81] = RET;
    instrTable[0x82] = INT;
    instrTable[0x84] = POP<AddressingMode::A, 0>;
    instrTable[0x85] = POP<AddressingMode::X, 1>;
    instrTable[0x88] = PUSH<AddressingMode::A, 0>;
    instrTable[0x89] = PUSH<AddressingMode::X, 1>;
    instrTable[0x90] = PRECODE;
    instrTable[0x94] = LD<AddressingMode::SP, AddressingMode::X, 1>;
    instrTable[0x96] = LD<AddressingMode::X, AddressingMode::SP, 1>;
    instrTable[0x97] = LD<AddressingMode::XL, AddressingMode::A, 0>;
    instrTable[0x9B] = SIM;
    instrTable[0xA1] = CP<AddressingMode::A, AddressingMode::Immediate, 0>;
    instrTable[0xA3] = CP<AddressingMode::X, AddressingMode::Immediate, 1>;
    instrTable[0xA5] = BCP<AddressingMode::Immediate>;
    instrTable[0xA6] = LD<AddressingMode::A, AddressingMode::Immediate, 0>;
    instrTable[0xA8] = XOR<AddressingMode::Immediate>;
    instrTable[0xAA] = OR<AddressingMode::Immediate>;
    instrTable[0xAC] = JPF;
    instrTable[0xAE] = LD<AddressingMode::X, AddressingMode::Immediate, 1>;
    instrTable[0xB3] = CP<AddressingMode::X, AddressingMode::DirectByte, 1>;
    instrTable[0xB6] = LD<AddressingMode::A, AddressingMode::DirectByte, 0>;
    instrTable[0xBE] = LD<AddressingMode::X, AddressingMode::DirectByte, 1>;
    instrTable[0xBF] = LD<AddressingMode::DirectByte, AddressingMode::X, 1>;
    instrTable[0xC6] = LD<AddressingMode::A, AddressingMode::DirectLong, 0>;
    instrTable[0xC7] = LD<AddressingMode::DirectLong, AddressingMode::A, 0>;
    instrTable[0xCD] = CALL<AddressingMode::DirectLong>;
    instrTable[0xCE] = LD<AddressingMode::X, AddressingMode::DirectLong, 1>;
    instrTable[0xE4] = AND<AddressingMode::DirectIndexedShortX>;
    instrTable[0xE7] = LD<AddressingMode::DirectIndexedShortX, AddressingMode::A, 0>;
    instrTable[0xEE] = LD<AddressingMode::X, AddressingMode::DirectIndexedShortX, 1>;
    instrTable[0xF4] = AND<AddressingMode::DirectIndexedX>;
    instrTable[0xF6] = LD<AddressingMode::A, AddressingMode::DirectIndexedX, 0>;
    instrTable[0xF7] = LD<AddressingMode::DirectIndexedX, AddressingMode::A, 0>;

    for (auto &i : precodeTable) {
        i = UNIMPLEMENTED_PRECODE;
    }

    precodeTable[0x01] = RRWA<AddressingMode::Y>;
    precodeTable[0x02] = RRWA<AddressingMode::X>;
    precodeTable[0x54] = SRL<AddressingMode::Y, 1>;
    precodeTable[0x5C] = INC<AddressingMode::Y, 1>;
    precodeTable[0x5F] = CLR<AddressingMode::Y, 1>;
    precodeTable[0x85] = POP<AddressingMode::Y, 1>;
    precodeTable[0x89] = PUSH<AddressingMode::Y, 1>;
    precodeTable[0x94] = LD<AddressingMode::SP, AddressingMode::Y, 1>;
    precodeTable[0x96] = LD<AddressingMode::Y, AddressingMode::SP, 1>;
    precodeTable[0x97] = LD<AddressingMode::YL, AddressingMode::A, 0>;
    precodeTable[0xA3] = CP<AddressingMode::Y, AddressingMode::Immediate, 1>;
    precodeTable[0xAE] = LD<AddressingMode::Y, AddressingMode::Immediate, 1>;
    precodeTable[0xB3] = CP<AddressingMode::Y, AddressingMode::DirectByte, 1>;
    precodeTable[0xBE] = LD<AddressingMode::Y, AddressingMode::DirectByte, 1>;
    precodeTable[0xBF] = LD<AddressingMode::DirectByte, AddressingMode::Y, 1>;
    precodeTable[0xCE] = LD<AddressingMode::Y, AddressingMode::DirectLong, 1>;
    precodeTable[0xE4] = AND<AddressingMode::DirectIndexedShortY>;
    precodeTable[0xEE] = LD<AddressingMode::Y, AddressingMode::DirectIndexedShortY, 1>;
    precodeTable[0xF4] = AND<AddressingMode::DirectIndexedY>;
    precodeTable[0xF6] = LD<AddressingMode::A, AddressingMode::DirectIndexedY, 0>;
    precodeTable[0xF7] = LD<AddressingMode::DirectIndexedY, AddressingMode::A, 0>;

    for (auto &i : prefix72Table) {
        i = UNIMPLEMENTED_PREFIX72;
    }

    for (int i = 0; i < 0x10; i += 2) {
        prefix72Table[0x10 | i] = BSET;
        prefix72Table[0x11 | i] = BRES;
    }

    prefix72Table[0x5F] = CLR<AddressingMode::DirectLong, 0>;
}

void initialize() {
    std::memset(&ctx, 0, sizeof(Context));

    initializeTables();

    ctx.pc = INITIAL_PC;
}

void shutdown() {}

template<typename T>
T fetch() {
    static_assert(std::is_same_v<T, u8> || std::is_same_v<T, u16>);

    u32 &pc = ctx.pc;

    const T data = read<T>(pc);

    pc += sizeof(T);

    return data;
}

template<typename T>
T pop() {
    static_assert(std::is_same_v<T, u8> || std::is_same_v<T, u16>);

    u16 &sp = ctx.sp;

    sp += sizeof(T);

    return read<T>(sp);
}

template<typename T>
void push(const T data) {
    static_assert(std::is_same_v<T, u8> || std::is_same_v<T, u16>);

    u16 &sp = ctx.sp;

    write<T>(sp, data);

    sp -= sizeof(T);
}

u8 fetchOpcode() {
    ctx.cpc = ctx.pc;

    return fetch<u8>();
}

template<AddressingMode mode>
u32 getEffectiveAddress() {
    switch (mode) {
        case AddressingMode::DirectByte:
            return fetch<u8>();
        case AddressingMode::DirectLong:
            return fetch<u16>();
        case AddressingMode::DirectExtended:
            return ((u32)fetch<u8>() << 16) | ((u32)fetch<u8>() << 8) | (u32)fetch<u8>();
        case AddressingMode::DirectIndexedX:
            return ctx.x;
        case AddressingMode::DirectIndexedY:
            return ctx.y;
        case AddressingMode::DirectIndexedShortX:
            return ctx.x + fetch<u8>();
        case AddressingMode::DirectIndexedShortY:
            return ctx.y + fetch<u8>();
        case AddressingMode::DirectIndexedShortSP:
            return ctx.sp + fetch<u8>();
        default:
            assert(false);
    }
}

template<AddressingMode mode, bool word>
u16 getOperand() {
    switch (mode) {
        case AddressingMode::Immediate:
            if constexpr (word) {
                return fetch<u16>();
            }

            return fetch<u8>();
        case AddressingMode::SP:
            assert(word);

            return ctx.sp;
        case AddressingMode::A:
            assert(!word);

            return ctx.a;
        case AddressingMode::X:
            assert(word);

            return ctx.x;
        case AddressingMode::XL:
            assert(!word);

            return (u8)ctx.x;
        case AddressingMode::Y:
            assert(word);

            return ctx.y;
        case AddressingMode::YL:
            assert(!word);

            return (u8)ctx.y;
        default:
            {
                u32 &ea = ctx.ea;

                ea = getEffectiveAddress<mode>();

                if constexpr (word) {
                    return read<u16>(ea);
                }

                return read<u8>(ea);
            }
    }
}

template<AddressingMode mode, bool word>
void setOperand(const u16 data, const bool useCachedEa = false) {
    switch (mode) {
        case AddressingMode::Immediate:
            assert(false);
        case AddressingMode::SP:
            assert(word);

            ctx.setSp(data);
            break;
        case AddressingMode::A:
            assert(!word);
            
            ctx.setA(data);
            break;
        case AddressingMode::X:
            assert(word);

            ctx.setX(data);
            break;
        case AddressingMode::XL:
            assert(!word);

            ctx.setX((ctx.x & 0xFF00) | data);
            break;
        case AddressingMode::Y:
            assert(word);

            ctx.setY(data);
            break;
        case AddressingMode::YL:
            assert(!word);

            ctx.setY((ctx.y & 0xFF00) | data);
            break;
        default:
            {
                u32 ea = ctx.ea;

                if (!useCachedEa) {
                    ea = getEffectiveAddress<mode>();
                }

                if constexpr (word) {
                    return write<u16>(ea, data);
                }

                return write<u8>(ea, (u8)data);
            }
    }
}

template<bool word>
bool addAuxiliaryCarry(const u16 a, const u16 b, const u16 r) {
    const u16 sgn = 3 + 4 * word;

    const bool sgnA = a >> sgn;
    const bool sgnB = b >> sgn;
    const bool sgnR = r >> sgn;

    return (sgnA && sgnB) || (sgnB && !sgnR) || (sgnA && !sgnR);
}

template<bool word>
bool addCarry(const u16 a, const u16 b, const u16 r) {
    const u16 sgn = 7 + 8 * word;

    const bool sgnA = a >> sgn;
    const bool sgnB = b >> sgn;
    const bool sgnR = r >> sgn;

    return (sgnA && sgnB) || (sgnB && !sgnR) || (sgnA && !sgnR);
}

template<bool word>
bool subCarry(const u16 a, const u16 b, const u16 r) {
    const u16 sgn = 7 + 8 * word;

    const bool sgnA = a >> sgn;
    const bool sgnB = b >> sgn;
    const bool sgnR = r >> sgn;

    return (!sgnA && sgnB) || (!sgnA && sgnR) || (sgnA && sgnB && sgnR);
}

template<bool word>
bool addOverflow(const u16 a, const u16 b, const u16 r) {
    const u16 sgn = 7 + 8 * word;

    const bool sgnA = a >> sgn;
    const bool sgnB = b >> sgn;
    const bool sgnR = r >> sgn;

    const bool msbA = a >> (sgn - 1);
    const bool msbB = b >> (sgn - 1);
    const bool msbR = r >> (sgn - 1);

    return ((sgnA && sgnB) || (sgnB && !sgnR) || (sgnA && !sgnR)) ^
           ((msbA && msbB) || (msbB && !msbR) || (msbA && !msbR));
}

template<bool word>
bool subOverflow(const u16 a, const u16 b, const u16 r) {
    const u16 sgn = 7 + 8 * word;

    const bool sgnA = a >> sgn;
    const bool sgnB = b >> sgn;
    const bool sgnR = r >> sgn;

    const bool msbA = a >> (sgn - 1);
    const bool msbB = b >> (sgn - 1);
    const bool msbR = r >> (sgn - 1);

    return ((!sgnA && sgnB) || (!sgnA && sgnR) || (sgnA && sgnB && sgnR)) ^
           ((!msbA && msbB) || (!msbA && msbR) || (msbA && msbB && msbR));
}

template<bool word>
void setBitFlags(const u16 result) {
    ctx.ccr.z = result == 0;
    ctx.ccr.n = result >> (7 + 8 * word);
}

template<AddressingMode dstMode, AddressingMode srcMode, bool word>
void ADD(const u8 opcode) {
    (void)opcode;

    const u16 a = getOperand<dstMode, word>();

    u16 b;

    if constexpr (dstMode == AddressingMode::SP) {
        assert(srcMode == AddressingMode::Immediate);

        b = getOperand<AddressingMode::Immediate, 0>();
    } else {
        b = getOperand<srcMode, word>();
    }

    const u16 result = (a + b) & ((1 << (8 + 8 * word)) - 1);

    if constexpr(dstMode != AddressingMode::SP) {
        ctx.ccr.c = addCarry<word>(a, b, result);
        ctx.ccr.z = result == 0;
        ctx.ccr.n = result >> (7 + 8 * word);
        ctx.ccr.h = addAuxiliaryCarry<word>(a, b, result);
        ctx.ccr.v = addOverflow<word>(a, b, result);
    }

    setOperand<dstMode, word>(result);
}

template<AddressingMode mode>
void AND(const u8 opcode) {
    (void)opcode;

    const u8 result = ctx.a & getOperand<mode, 0>();

    setBitFlags<0>(result);

    ctx.setA(result);
}

template<AddressingMode mode>
void BCP(const u8 opcode) {
    (void)opcode;

    const u8 result = ctx.a & getOperand<mode, 0>();

    setBitFlags<0>(result);
}

void BRES(const u8 opcode) {
    setOperand<AddressingMode::DirectLong, 0>(getOperand<AddressingMode::DirectLong, 0>() & ~(1 << ((opcode >> 1) & 7)), 1);
}

void BSET(const u8 opcode) {
    setOperand<AddressingMode::DirectLong, 0>(getOperand<AddressingMode::DirectLong, 0>() | (1 << ((opcode >> 1) & 7)), 1);
}

template<AddressingMode mode>
void CALL(const u8 opcode) {
    (void)opcode;

    const u32 addr = getEffectiveAddress<mode>();

    push<u16>((u16)ctx.pc);

    ctx.setPc((ctx.pc & ~0xFFFF) | addr);
}

template<AddressingMode mode, bool word>
void CLR(const u8 opcode) {
    (void)opcode;

    setBitFlags<0>(0);
    setOperand<mode, word>(0);
}

template<AddressingMode dstMode, AddressingMode srcMode, bool word>
void CP(const u8 opcode) {
    (void)opcode;

    const u16 a = getOperand<dstMode, word>();
    const u16 b = getOperand<srcMode, word>();

    const u16 result = (a - b) & ((1 << (8 + 8 * word)) - 1);

    ctx.ccr.c = subCarry<word>(a, b, result);
    ctx.ccr.z = result == 0;
    ctx.ccr.n = result >> (7 + 8 * word);
    ctx.ccr.v = subOverflow<word>(a, b, result);
}

template<AddressingMode mode, bool word>
void CPL(const u8 opcode) {
    (void)opcode;

    const u16 result = (~getOperand<mode, word>()) & ((1 << (8 + 8 * word)) - 1);

    ctx.ccr.c = 1;
    ctx.ccr.z = result == 0;
    ctx.ccr.n = result >> (7 + 8 * word);

    setOperand<mode, word>(result, 1);
}

template<AddressingMode mode, bool word>
void DEC(const u8 opcode) {
    (void)opcode;

    const u16 a = getOperand<mode, word>();
    const u16 b = 1;

    const u16 result = (a - b) & ((1 << (8 + 8 * word)) - 1);

    ctx.ccr.z = result == 0;
    ctx.ccr.n = result >> (7 + (8 * word));
    ctx.ccr.v = subOverflow<word>(a, b, result);

    setOperand<mode, word>(result, 1);
}

template<AddressingMode mode, bool word>
void INC(const u8 opcode) {
    (void)opcode;

    const u16 a = getOperand<mode, word>();
    const u16 b = 1;

    const u16 result = (a + b) & ((1 << (8 + 8 * word)) - 1);

    ctx.ccr.z = result == 0;
    ctx.ccr.n = result >> (7 + (8 * word));
    ctx.ccr.v = addOverflow<word>(a, b, result);

    setOperand<mode, word>(result, 1);
}

void INT(const u8 opcode) {
    (void)opcode;

    ctx.setPc(getEffectiveAddress<AddressingMode::DirectExtended>());
};

template<Condition cc>
void JRcc(const u8 opcode) {
    (void)opcode;

    const auto &ccr = ctx.ccr;

    const u32 offset = (u32)(i8)fetch<u8>();

    bool condition;

    switch (cc) {
        case Condition::True:
            condition = true;
            break;
        case Condition::Equal:
            condition = ccr.z != 0;
            break;
        case Condition::NotEqual:
            condition = ccr.z == 0;
            break;
        case Condition::Carry:
            condition = ccr.c != 0;
            break;
        case Condition::NotCarry:
            condition = ccr.c == 0;
            break;
        default:
            std::puts("[  CPU  ] Unimplemented condition code");

            exit(1);
    }

    if (condition) {
        ctx.setPc(ctx.pc + offset);
    }
}

void JPF(const u8 opcode) {
    (void)opcode;

    ctx.setPc(getEffectiveAddress<AddressingMode::DirectExtended>());
};

template<AddressingMode dstMode, AddressingMode srcMode, bool word>
void LD(const u8 opcode) {
    (void)opcode;

    const u16 data = getOperand<srcMode, word>();

    if constexpr (!isRegister(dstMode) || !isRegister(srcMode)) {
        setBitFlags<word>(data);
    }

    setOperand<dstMode, word>(data);
}

template<AddressingMode mode>
void OR(const u8 opcode) {
    (void)opcode;

    const u8 result = ctx.a | getOperand<mode, 0>();

    setBitFlags<0>(result);

    ctx.setA(result);
}

template<AddressingMode mode, bool word>
void POP(const u8 opcode) {
    (void)opcode;

    u16 data;

    if constexpr (word) {
        data = pop<u16>();
    } else {
        data = pop<u8>();
    }

    setOperand<mode, word>(data);
}

template<AddressingMode mode, bool word>
void PUSH(const u8 opcode) {
    (void)opcode;

    if constexpr (word) {
        push<u16>(getOperand<mode, word>());
    } else {
        push<u8>((u8)getOperand<mode, word>());
    }
}

void PRECODE(const u8 opcode) {
    (void)opcode;

    const u8 nextOpcode = fetchOpcode();

    return precodeTable[nextOpcode](nextOpcode);
}

void PREFIX72(const u8 opcode) {
    (void)opcode;

    const u8 nextOpcode = fetchOpcode();

    return prefix72Table[nextOpcode](nextOpcode);
}

void RET(const u8 opcode) {
    (void)opcode;

    ctx.setPc((ctx.pc & ~0xFFFF) | pop<u16>());
}

template<AddressingMode mode>
void RLWA(const u8 opcode) {
    (void)opcode;

    const u16 a = ctx.a;
    const u16 x = getOperand<mode, 1>();

    const u16 result = (x << 8) | a;

    setBitFlags<1>(result);
    setOperand<mode, 1>(result);

    ctx.setA(x >> 8);
}

template<AddressingMode mode, bool word>
void RRC(const u8 opcode) {
    (void)opcode;

    const u16 a = getOperand<mode, word>();
    const u16 c = ctx.ccr.c;

    const u16 result = (a >> 1) | (c << (7 + 8 * word));

    ctx.ccr.c = a & 1;

    setBitFlags<word>(result);
    setOperand<mode, word>(result, 1);
}

template<AddressingMode mode>
void RRWA(const u8 opcode) {
    (void)opcode;

    const u16 a = ctx.a;
    const u16 x = getOperand<mode, 1>();

    const u16 result = (x >> 8) | (a << 8);

    setBitFlags<1>(result);
    setOperand<mode, 1>(result);

    ctx.setA(x & 0xFF);
}

void SIM(const u8 opcode) {
    (void)opcode;

    // Disables interrupts
    ctx.ccr.i0 = 1;
    ctx.ccr.i1 = 1;
}

template<AddressingMode mode, bool word>
void SRL(const u8 opcode) {
    (void)opcode;

    const u16 result = getOperand<mode, word>() >> 1;

    setBitFlags<word>(result);
    setOperand<mode, word>(result, 1);
}

template<AddressingMode dstMode, AddressingMode srcMode, bool word>
void SUB(const u8 opcode) {
    (void)opcode;

    const u16 a = getOperand<dstMode, word>();

    u16 b;

    if constexpr (dstMode == AddressingMode::SP) {
        assert(srcMode == AddressingMode::Immediate);

        b = getOperand<AddressingMode::Immediate, 0>();
    } else {
        b = getOperand<srcMode, word>();
    }

    const u16 result = (a - b) & ((1 << (8 + 8 * word)) - 1);

    if constexpr(dstMode != AddressingMode::SP) {
        ctx.ccr.c = subCarry<word>(a, b, result);
        ctx.ccr.z = result == 0;
        ctx.ccr.n = result >> (7 + 8 * word);
        ctx.ccr.v = subOverflow<word>(a, b, result);

        if constexpr (word) {
            ctx.ccr.h = subAuxiliaryCarry<word>(a, b, result);
        }
    }

    setOperand<dstMode, word>(result);
}

template<AddressingMode mode, bool word>
void TNZ(const u8 opcode) {
    (void)opcode;

    const u16 data = getOperand<mode, word>();

    setBitFlags<word>(data);
}

template<AddressingMode mode>
void XOR(const u8 opcode) {
    (void)opcode;

    const u8 result = ctx.a ^ getOperand<mode, 0>();

    setBitFlags<0>(result);

    ctx.setA(result);
}

void UNIMPLEMENTED(const u8 opcode) {
    std::printf("[  CPU  ] Unimplemented opcode %02X (PC = %06X)\n", opcode, ctx.cpc);

    exit(1);
}

void UNIMPLEMENTED_PRECODE(const u8 opcode) {
    std::printf("[  CPU  ] Unimplemented PRECODE opcode %02X (PC = %06X)\n", opcode, ctx.cpc);

    exit(1);
}

void UNIMPLEMENTED_PREFIX72(const u8 opcode) {
    std::printf("[  CPU  ] Unimplemented PREFIX 72 opcode %02X (PC = %06X)\n", opcode, ctx.cpc);

    exit(1);
}

void run() {
    const u8 opcode = fetchOpcode();

    instrTable[opcode](opcode);
}

}
