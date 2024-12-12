/*
 * nema is a Wii U Game Pad UIC emulator written in C++.
 * Copyright (C) 2024  noumidev
 */

#include "sys/emulator.hpp"

#include <cstdio>

#include "hw/cpu.hpp"

#include "sys/memory.hpp"

namespace sys::emulator {

void initialize(const Config &config) {
    std::printf("[  EMU  ] FLASH path = \"%s\"\n", config.flashPath);
    std::printf("[  EMU  ] EEPROM path = \"%s\"\n", config.eepromPath);

    memory::initialize(config);

    hw::cpu::initialize();
}

void shutdown() {
    hw::cpu::shutdown();

    memory::shutdown();
}

void run() {
    while (true) {
        hw::cpu::run();
    }
}

}
