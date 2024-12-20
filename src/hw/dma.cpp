/*
 * nema is a Wii U Game Pad UIC emulator written in C++.
 * Copyright (C) 2024  noumidev
 */

#include "hw/dma.hpp"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <type_traits>

#include "sys/memory.hpp"

using namespace common;
using namespace sys;

namespace hw::dma {

constexpr int NUM_CHANNELS = 4;
constexpr int NUM_ADDRESS_REGISTERS = 2;

namespace Address {
    enum : u32 {
        GCSR = 0x5070,
        C3CR = 0x5093,
        C3SPR = 0x5094,
        C3NDTR = 0x5095,
        C3PARH = 0x5096,
        C3PARL = 0x5097,
        C3M0ARH = 0x5099,
        C3M0ARL = 0x509A,
    };
}

namespace AddressOffset {
    enum : u32 {
        Low = 0,
        High = 8,
        Extended = 16,
    };
}

namespace AddressRegister {
    enum : int {
        Memory = 0,
        Peripheral = 1,
        Source = 0,
        Destination = 1,
    };
}

struct Context {
    union GlobalConfiguration {
        u8 raw;

        struct {
            u8 gen : 1;
            u8 gb : 1;
            u8 to : 6;
        };
    } gcsr;

    struct {
        union ChannelControl {
            u8 raw;

            struct {
                u8 en : 1;
                u8 tcie : 1;
                u8 htie : 1;
                u8 dir : 1;
                u8 circ : 1;
                u8 mincdec : 1;
                u8 mem : 1;
                u8 : 1;
            };
        } ccr;

        union ChannelPriority {
            u8 raw;

            struct {
                u8 : 1;
                u8 tcif : 1;
                u8 hcif : 1;
                u8 tsize : 1;
                u8 pl : 2;
                u8 pend : 1;
                u8 busy : 1;
            };
        } cspr;

        u8 ndtr;

        u32 mar[NUM_ADDRESS_REGISTERS];

        bool isEnabled() const {
            return ccr.en != 0;
        }

        bool isMemoryCopy() const {
            return ccr.mem != 0;
        }

        bool isByteTransfer() const {
            return cspr.tsize == 0;
        }

        u32 getDestinationOffset() const {
            if (ccr.mincdec != 0) {
                return 1;
            }

            return -1;
        }

        void transferFinished() {
            cspr.tcif = 1;

            if (ccr.tcie != 0) {
                std::puts("[  DMA  ] Unimplemented transfer completion interrupt");
            }

            cspr.busy = 0;
        }
    } channels[NUM_CHANNELS];

    u8 getGlobalConfiguration() {
        return gcsr.raw;
    }

    void setGlobalConfiguration(const u8 data) {
        gcsr.raw &= ~0xFD;
        gcsr.raw |= data & 0xFD;
    }

    u8 getChannelControl(const int idx) {
        assert(idx < NUM_CHANNELS);

        return channels[idx].ccr.raw;
    }

    void setChannelControl(const int idx, const u8 data) {
        assert(idx < NUM_CHANNELS);

        auto &ccr = channels[idx].ccr;

        u8 mask = 0x3F;

        if (idx == 3) {
            mask |= 0x40;
        }

        ccr.raw &= ~mask;
        ccr.raw |= data & mask;
    }

    u8 getChannelPriority(const int idx) {
        assert(idx < NUM_CHANNELS);

        return channels[idx].cspr.raw;
    }

    void setChannelPriority(const int idx, const u8 data) {
        assert(idx < NUM_CHANNELS);

        auto &cspr = channels[idx].cspr;

        cspr.raw &= ~0x38;
        cspr.raw |= data & 0x38;

        if ((data & 2) == 0) {
            cspr.tcif = 0;
        }

        if ((data & 4) == 0) {
            cspr.hcif = 0;
        }
    }

    void setTransferCount(const int idx, const u8 data) {
        assert(idx < NUM_CHANNELS);

        channels[idx].ndtr = data;
    }

    u32 getMemoryAddress(const int idx, const int reg) {
        assert(idx < NUM_CHANNELS);
        assert(reg < NUM_ADDRESS_REGISTERS);

        return channels[idx].mar[reg];
    }

    void setMemoryAddress(const int idx, const int reg, const u32 offset, const u8 data) {
        assert(idx < NUM_CHANNELS);
        assert(reg < NUM_ADDRESS_REGISTERS);

        auto &mar = channels[idx].mar[reg];

        mar &= ~(0xFF << offset);
        mar |= (u32)data << offset;
    }

    bool isAnyChannelEnabled() {
        if (gcsr.gen == 0) {
            return false;
        }

        for (int i = 0; i < NUM_CHANNELS; i++) {
            const auto &channel = channels[i];

            if (channel.isEnabled()) {
                return true;
            }
        }

        return false;
    }
} ctx;

void initialize() {
    std::memset(&ctx, 0, sizeof(Context));

    ctx.setGlobalConfiguration(0xFC);
}

void shutdown() {}

void runMemoryCopy() {
    auto &channel = ctx.channels[3];

    u32 dad = ctx.getMemoryAddress(3, AddressRegister::Destination);
    u32 sad = ctx.getMemoryAddress(3, AddressRegister::Source);

    assert(channel.isByteTransfer());

    const u32 offset = channel.getDestinationOffset();

    // channel.cspr.busy = 1;

    u8 &ndtr = channel.ndtr;

    std::printf("[  DMA  ] Channel 3 memory copy (M0AR = %08X, M1AR = %08X, NDTR = %02X)\n", sad, dad, ndtr);

    for (; ndtr > 0; ndtr--) {
        memory::write<u8>(dad, memory::read<u8>(sad));

        dad += offset;
        sad++;
    }

    channel.transferFinished();
}

void runChannels() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        const auto &channel = ctx.channels[i];

        if (channel.isEnabled()) {
            if ((i == 3) && (channel.isMemoryCopy())) {
                runMemoryCopy();

                break;
            }

            std::printf("[  DMA  ] Unimplemented Channel %d transfer\n", i);

            exit(1);
        }
    }
}

template<typename T>
T read(const u32 addr) {
    static_assert(std::is_same_v<T, u8> || std::is_same_v<T, u16>);

    if constexpr (std::is_same_v<T, u16>) {
        assert(false);
    }

    switch (addr) {
        case Address::GCSR:
            std::printf("[  DMA  ] GCSR read%lu\n", 8 * sizeof(T));

            return ctx.getGlobalConfiguration();
        case Address::C3CR:
            std::printf("[  DMA  ] C3CR read%lu\n", 8 * sizeof(T));

            return ctx.getChannelControl(3);
        case Address::C3SPR:
            std::printf("[  DMA  ] C3SPR read%lu\n", 8 * sizeof(T));

            return ctx.getChannelPriority(3);
        default:
            std::printf("[  DMA  ] Unimplemented read%lu (address = %08X)\n", 8 * sizeof(T), addr);

            exit(1);
    }
}

template u8 read(const u32 addr);
template u16 read(const u32 addr);

template<typename T>
void write(const u32 addr, const T data) {
    static_assert(std::is_same_v<T, u8> || std::is_same_v<T, u16>);

    if constexpr (std::is_same_v<T, u16>) {
        assert(false);
    }

    switch (addr) {
        case Address::GCSR:
            std::printf("[  DMA  ] GCSR write%lu (data = %02X)\n", 8 * sizeof(T), data);

            ctx.setGlobalConfiguration(data);
            break;
        case Address::C3CR:
            std::printf("[  DMA  ] C3CR write%lu (data = %02X)\n", 8 * sizeof(T), data);

            ctx.setChannelControl(3, data);
            break;
        case Address::C3SPR:
            std::printf("[  DMA  ] C3SPR write%lu (data = %02X)\n", 8 * sizeof(T), data);

            ctx.setChannelPriority(3, data);
            break;
        case Address::C3NDTR:
            std::printf("[  DMA  ] C3NDTR write%lu (data = %02X)\n", 8 * sizeof(T), data);

            ctx.setTransferCount(3, data);
            break;
        case Address::C3PARH:
            std::printf("[  DMA  ] C3PARH write%lu (data = %02X)\n", 8 * sizeof(T), data);

            ctx.setMemoryAddress(3, AddressRegister::Peripheral, AddressOffset::High, data);
            break;
        case Address::C3PARL:
            std::printf("[  DMA  ] C3PARL write%lu (data = %02X)\n", 8 * sizeof(T), data);

            ctx.setMemoryAddress(3, AddressRegister::Peripheral, AddressOffset::Low, data);
            break;
        case Address::C3M0ARH:
            std::printf("[  DMA  ] C3M0ARH write%lu (data = %02X)\n", 8 * sizeof(T), data);

            ctx.setMemoryAddress(3, AddressRegister::Memory, AddressOffset::High, data);
            break;
        case Address::C3M0ARL:
            std::printf("[  DMA  ] C3M0ARL write%lu (data = %02X)\n", 8 * sizeof(T), data);

            ctx.setMemoryAddress(3, AddressRegister::Memory, AddressOffset::Low, data);
            break;
        default:
            std::printf("[  DMA  ] Unimplemented write%lu (address = %08X, data = %02X)\n", 8 * sizeof(T), addr, data);

            exit(1);
    }

    if (ctx.isAnyChannelEnabled()) {
        runChannels();
    }
}

template void write(const u32 addr, const u8 data);
template void write(const u32 addr, const u16 data);

}
