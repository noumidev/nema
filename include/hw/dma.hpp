/*
 * nema is a Wii U Game Pad UIC emulator written in C++.
 * Copyright (C) 2024  noumidev
 */

#pragma once

#include "common/types.hpp"

namespace hw::dma {

void initialize();
void shutdown();

template<typename T>
T read(const common::u32 addr);

template<typename T>
void write(const common::u32 addr, const T data);

}
