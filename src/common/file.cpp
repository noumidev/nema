/*
 * nema is a Wii U Game Pad UIC emulator written in C++.
 * Copyright (C) 2024  noumidev
 */

#include "common/file.hpp"

#include <cstdio>
#include <cstdlib>

namespace common {

void load(const char *path, u8 *buf, const int size) {
    FILE *file = std::fopen(path, "rb");

    if (file == NULL) {
        std::printf("Failed to open file \"%s\"\n", path);

        exit(1);
    }

    std::fseek(file, 0, SEEK_END);
    const int filesize = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);

    // NOTE: size == 0 implies no size check
    if ((size != 0) && (filesize != size)) {
        std::printf("Size mismatch (%d %d)\n", filesize, size);

        exit(1);
    }

    std::fread(buf, sizeof(u8), filesize, file);
    std::fclose(file);
}

}
