/*
 * nema is a Wii U Game Pad UIC emulator written in C++.
 * Copyright (C) 2024  noumidev
 */

#include "sys/memory.hpp"

#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <type_traits>

#include "common/bswap.hpp"
#include "common/file.hpp"

using namespace common;

namespace sys::memory {

// Constants for software page table
constexpr u32 SHIFT_PAGE = 12;
constexpr u32 SIZE_PAGE = 1 << SHIFT_PAGE;
constexpr u32 MASK_PAGE = SIZE_PAGE - 1;

constexpr u32 NUM_PAGES = MemorySize::AddressSpace >> SHIFT_PAGE;

struct PageTable {
    std::array<u8*, NUM_PAGES> readTable, writeTable;

    void clear() {
        for (u32 i = 0; i < NUM_PAGES; i++) {
            readTable[i] = writeTable[i] = NULL;
        }
    }

    void map(const u32 addr, const u32 size, u8 *mem, const bool read, const bool write) {
        assert(addr < MemorySize::AddressSpace);

        const u32 page = addr >> SHIFT_PAGE;
        const u32 num = size >> SHIFT_PAGE;

        for (u32 i = 0; i < num; i++) {
            if (read) {
                readTable[page + i] = &mem[i * SIZE_PAGE];
            }

            if (write) {
                writeTable[page + i] = &mem[i * SIZE_PAGE];
            }
        }
    }
} pageTable;

struct Context {
    u8 ram[MemorySize::RAM];
    u8 flash[MemorySize::FLASH];
    u8 eeprom[MemorySize::EEPROM];
} ctx;

void initialize(const Config &config) {
    std::memset(&ctx, 0, sizeof(Context));

    pageTable.clear();

    // Load files
    load(config.flashPath, ctx.flash, 0);
    load(config.eepromPath, ctx.eeprom, MemorySize::EEPROM);

    pageTable.map(MemoryBase::RAM, MemorySize::RAM, ctx.ram, true, true);
    pageTable.map(MemoryBase::FLASH, MemorySize::FLASH, ctx.flash, true, false);
    pageTable.map(MemoryBase::EEPROM, MemorySize::EEPROM, ctx.eeprom, true, false);
}

void shutdown() {}

template<typename T>
T read(const u32 addr) {
    static_assert(std::is_same_v<T, u8> || std::is_same_v<T, u16>);

    assert(addr < MemorySize::AddressSpace);

    const u32 page = addr >> SHIFT_PAGE;

    if (pageTable.readTable[page] != NULL) {
        T data;

        std::memcpy(&data, &pageTable.readTable[page][addr & MASK_PAGE], sizeof(T));

        return bswap<T>(data);
    }

    std::printf("[  MEM  ] Unmapped read%lu (address = %08X)\n", 8 * sizeof(T), addr);

    return 0;
}

template u8 read(const u32 addr);
template u16 read(const u32 addr);

template<typename T>
void write(const u32 addr, const T data) {
    static_assert(std::is_same_v<T, u8> || std::is_same_v<T, u16>);

    assert(addr < MemorySize::AddressSpace);

    const u32 page = addr >> SHIFT_PAGE;

    if (pageTable.writeTable[page] != NULL) {
        const T swappedData = bswap<T>(data);

        std::memcpy(&pageTable.readTable[page][addr & MASK_PAGE], &swappedData, sizeof(T));

        return;
    }

    std::printf("[  MEM  ] Unmapped write%lu (address = %08X, data = %02X)\n", 8 * sizeof(T), addr, data);
}

template void write(const u32 addr, const u8 data);
template void write(const u32 addr, const u16 data);

}
