/*
 * nema is a Wii U Game Pad UIC emulator written in C++.
 * Copyright (C) 2024  noumidev
 */

#pragma once

#include "sys/config.hpp"

namespace sys::emulator {

void initialize(const Config &config);
void shutdown();

void run();

}
