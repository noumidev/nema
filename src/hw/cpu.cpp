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

typedef void (*InstructionHandler)(const u8);

constexpr int SIZE_INSTRUCTION_TABLE = 0x100;

constexpr u32 INITIAL_PC = 0x8000;

enum class AddressingMode {
    DirectExtended,
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

    void setPc(const u32 addr) {
        pc = addr;

        std::printf("[  CPU  ] Jump to %06X\n", addr);
    }
} ctx;

InstructionHandler instrTable[SIZE_INSTRUCTION_TABLE];

// Instruction handlers
void INT(const u8);
void UNIMPLEMENTED(const u8);

void initializeTables() {
    for (auto &i : instrTable) {
        i = UNIMPLEMENTED;
    }

    instrTable[0x82] = INT;
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

template<AddressingMode mode>
u32 getEffectiveAddress() {
    switch (mode) {
        case AddressingMode::DirectExtended:
            return ((u32)fetch<u8>() << 16) | ((u32)fetch<u8>() << 8) | (u32)fetch<u8>();
    }
}

void INT(const u8 opcode) {
    (void)opcode;

    ctx.setPc(getEffectiveAddress<AddressingMode::DirectExtended>());
};

void UNIMPLEMENTED(const u8 opcode) {
    std::printf("[  CPU  ] Unimplemented opcode %02X (PC = %06X)\n", opcode, ctx.cpc);

    exit(1);
}

void run() {
    ctx.cpc = ctx.pc;

    const u8 opcode = fetch<u8>();

    instrTable[opcode](opcode);
}

}
