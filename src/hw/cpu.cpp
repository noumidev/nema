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
    DirectIndexed,
    Immediate,
    SP,
    A,
    X,
    Y,
};

constexpr bool isRegister(const AddressingMode mode) {
    return (int)mode >= (int)AddressingMode::SP;
}

enum class Condition {
    Equal,
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
        pc = addr;

        std::printf("[  CPU  ] Jump to %06X\n", addr);
    }
} ctx;

InstructionTable instrTable, precodeTable;

// Instruction handlers
template<AddressingMode mode>
void BCP(const u8);
void INT(const u8);
template<Condition cc>
void JRcc(const u8);
template<AddressingMode dstMode, AddressingMode srcMode, bool word>
void LD(const u8);
void PRECODE(const u8);
void UNIMPLEMENTED(const u8);
void UNIMPLEMENTED_PRECODE(const u8);

void initializeTables() {
    for (auto &i : instrTable) {
        i = UNIMPLEMENTED;
    }

    instrTable[0x27] = JRcc<Condition::Equal>;
    instrTable[0x82] = INT;
    instrTable[0x90] = PRECODE;
    instrTable[0x94] = LD<AddressingMode::SP, AddressingMode::X, 1>;
    instrTable[0xA5] = BCP<AddressingMode::Immediate>;
    instrTable[0xAE] = LD<AddressingMode::X, AddressingMode::Immediate, 1>;
    instrTable[0xBF] = LD<AddressingMode::DirectByte, AddressingMode::X, 1>;
    instrTable[0xCE] = LD<AddressingMode::X, AddressingMode::DirectLong, 1>;
    instrTable[0xF6] = LD<AddressingMode::A, AddressingMode::DirectIndexed, 0>;

    for (auto &i : precodeTable) {
        i = UNIMPLEMENTED_PRECODE;
    }

    precodeTable[0x94] = LD<AddressingMode::SP, AddressingMode::Y, 1>;
    precodeTable[0xAE] = LD<AddressingMode::Y, AddressingMode::Immediate, 1>;
    precodeTable[0xBF] = LD<AddressingMode::DirectByte, AddressingMode::Y, 1>;
    precodeTable[0xCE] = LD<AddressingMode::Y, AddressingMode::DirectLong, 1>;
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
        case AddressingMode::DirectIndexed:
            return ctx.x;
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
        case AddressingMode::Y:
            assert(word);

            return ctx.y;
        default:
            {
                const u32 ea = getEffectiveAddress<mode>();

                if constexpr (word) {
                    return read<u16>(ea);
                }

                return read<u8>(ea);
            }
    }
}

template<AddressingMode mode, bool word>
void setOperand(const u16 data) {
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
        case AddressingMode::Y:
            assert(word);

            ctx.setY(data);
            break;
        default:
            {
                const u32 ea = getEffectiveAddress<mode>();

                if constexpr (word) {
                    return write<u16>(ea, data);
                }

                return write<u8>(ea, (u8)data);
            }
    }
}

template<AddressingMode mode>
void BCP(const u8 opcode) {
    (void)opcode;

    const u8 result = ctx.a & getOperand<mode, 0>();

    ctx.ccr.z = result == 0;
    ctx.ccr.n = result >> 7;
}

template<AddressingMode dstMode, AddressingMode srcMode, bool word>
void LD(const u8 opcode) {
    (void)opcode;

    const u16 data = getOperand<srcMode, word>();

    if constexpr (!isRegister(dstMode) || !isRegister(srcMode)) {
        ctx.ccr.z = data == 0;
        ctx.ccr.n = data >> (7 + (8 * word));
    }

    setOperand<dstMode, word>(data);
}

template<Condition cc>
void JRcc(const u8 opcode) {
    (void)opcode;

    const auto &ccr = ctx.ccr;

    const u32 offset = (u32)(i8)fetch<u8>();

    bool condition;

    switch (cc) {
        case Condition::Equal:
            condition = ccr.z != 0;
            break;
        default:
            std::puts("[  CPU  ] Unimplemented condition code");

            exit(1);
    }

    if (condition) {
        ctx.setPc(ctx.pc + offset);
    }
}

void INT(const u8 opcode) {
    (void)opcode;

    ctx.setPc(getEffectiveAddress<AddressingMode::DirectExtended>());
};

void PRECODE(const u8 opcode) {
    (void)opcode;

    const u8 nextOpcode = fetchOpcode();

    return precodeTable[nextOpcode](nextOpcode);
}

void UNIMPLEMENTED(const u8 opcode) {
    std::printf("[  CPU  ] Unimplemented opcode %02X (PC = %06X)\n", opcode, ctx.cpc);

    exit(1);
}

void UNIMPLEMENTED_PRECODE(const u8 opcode) {
    std::printf("[  CPU  ] Unimplemented PRECODE opcode %02X (PC = %06X)\n", opcode, ctx.cpc);

    exit(1);
}

void run() {
    const u8 opcode = fetchOpcode();

    instrTable[opcode](opcode);
}

}
