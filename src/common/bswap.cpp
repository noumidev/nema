/*
 * nema is a Wii U Game Pad UIC emulator written in C++.
 * Copyright (C) 2024  noumidev
 */

#include "common/bswap.hpp"

namespace common {

template<>
u8 bswap(const u8 data) {
    return data;
}

template<>
u16 bswap(const u16 data) {
    return (data >> 8) | (data << 8);
}

}
