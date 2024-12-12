/*
 * nema is a Wii U Game Pad UIC emulator written in C++.
 * Copyright (C) 2024  noumidev
 */

#pragma once

#include "common/types.hpp"

#include "sys/config.hpp"

namespace sys::memory {

namespace MemoryBase {
    enum : common::u32 {
        EEPROM = 0x1000,
        FLASH = 0x8000,
    };
}

namespace MemorySize {
    enum : common::u32 {
        EEPROM = 0x1000,
        FLASH = 0x8000,
        AddressSpace = 0x1000000,
    };
}

void initialize(const Config &config);
void shutdown();

template<typename T>
T read(const common::u32 addr);

template<typename T>
void write(const common::u32 addr, const T data);

}
