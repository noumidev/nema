#include <cstdio>

#include "sys/config.hpp"
#include "sys/emulator.hpp"

constexpr int NUM_ARGS = 2 + 1;

int main(int argc, char **argv) {
    if (argc < NUM_ARGS) {
        std::puts("Usage: nema [path to FLASH dump] [path to EEPROM dump]");

        return 1;
    }

    const sys::Config config{.flashPath = argv[1], .eepromPath = argv[2]};

    sys::emulator::initialize(config);
    sys::emulator::run();
    sys::emulator::shutdown();

    return 0;
}
