/**
 * CAN MCP2515_nb
 * Copyright 2020 WitchCraftWorks Team, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#ifndef MCP2515_H
#define MCP2515_H

#undef max
#undef min

#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
# include <queue>
#endif

#include <Arduino.h>
#include <SPI.h>

#include "ErrorCodes.hpp"

// According to VS there is a "OVERFLOW" macro defined in corecrt_math.h
#undef OVERFLOW

#if defined(ARDUINO_ARCH_SAMD)
# define __MCP2515_MULTI_INTERRUPTS_ENABLE__ 1
#else
# undef __MCP2515_MULTI_INTERRUPTS_ENABLE__
#endif

enum MCP2515_CAN_MASK {
    MASK0 = 0,
    MASK1 = 1
};

enum MCP2515_CAN_RXF {
    RXF0 = 0,
    RXF1 = 1,
    RXF2 = 2,
    RXF3 = 3,
    RXF4 = 4,
    RXF5 = 5
};

enum MCP2515_MODES {
    NORMAL = 0,
    LOOPBACK = 1,
    LISTEN = 2,
    CONFIG = 3,
    SLEEP = 4
};

// defer include to MCP2515_nb.h
class CANPacket;

#ifndef MCP2515_CANPACKET_TX_QUEUE_SIZE
# define MCP2515_CANPACKET_TX_QUEUE_SIZE 16
#endif

#define MCP2515_DEFAULT_CS_PIN  10
#define MCP2515_DEFAULT_INT_PIN 2

using packetCallback = void(*)(CANPacket&&);

class MCP2515 {
public:
    enum MCP2515_CAN_SPEED {
        CAN_1000KBPS = 0,
        CAN_500KBPS,
        CAN_250KBPS,
        CAN_200KBPS,
        CAN_125KBPS,
        CAN_100KBPS,
        CAN_80KBPS,
        CAN_50KBPS,
        CAN_40KBPS,
        CAN_20KBPS,
        CAN_10KBPS,
        CAN_5KBPS,

        MCP_SPEED_max
    };

    enum MCP2515_CAN_CLOCK {
        MCP_8MHZ = 0,
        MCP_16MHZ,

        MCP_CLOCK_max
    };

public:
    MCP2515(MCP2515_CAN_CLOCK clk = MCP2515_CAN_CLOCK::MCP_8MHZ, SPIClass &spi = SPI);
    ~MCP2515() = default;

    MCP2515Error begin(MCP2515_CAN_SPEED baudRate);
    void end();

    uint8_t getStatus();
    uint8_t getErrorFlags();

    void setPins(int cs = MCP2515_DEFAULT_CS_PIN, int irq = MCP2515_DEFAULT_INT_PIN);
    void setSPIFrequency(uint32_t frequency);

    MCP2515Error setMask(const MCP2515_CAN_MASK num, bool extended, uint32_t mask);
    MCP2515Error setFilter(const MCP2515_CAN_RXF num, bool extended, uint32_t filter);

    int getMode();
    MCP2515Error setConfigMode();
    MCP2515Error setListenMode();
    MCP2515Error setLoopbackMode();
    MCP2515Error setSleepMode();
    MCP2515Error setNormalMode();

    MCP2515Error setWakeupFilter(bool enable);
    MCP2515Error setOneShotMode(bool enable);

    MCP2515Error receivePacket(CANPacket &packet);

    void onReceivePacket(packetCallback callback);

    size_t getTxQueueLength();
    void processTxQueue();

    MCP2515Error writePacket(const CANPacket &packet, bool nowait = false);
    MCP2515Error abortPacket(CANPacket* packet, bool nowait = false);
    MCP2515Error waitForPacketStatus(CANPacket* packet, unsigned long status, bool nowait = false, unsigned long timeout = 0);

    static void onInterrupt();
    void _handleInterruptPacket();

private:
    void reset();

    uint8_t readRegister(uint8_t address);
    void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t getMCP2515Status();

    MCP2515Error handleMessageTransmit(int txBuf, bool cond);
    int8_t getFreeTxBuffer();

    struct _mcp_cnf_frequency {
        uint8_t one{0x00};
        uint8_t two{0x00};
        uint8_t three{0x00};

        constexpr _mcp_cnf_frequency() = default;
        constexpr _mcp_cnf_frequency(uint8_t one, uint8_t two, uint8_t three) : one(one), two(two), three(three) {}

        constexpr _mcp_cnf_frequency(const _mcp_cnf_frequency &l) {
            this->one = l.one;
            this->two = l.two;
            this->three = l.three;
        }

        explicit operator bool() const {
            return one && two && three;
        }

        static _mcp_cnf_frequency copy_P(const _mcp_cnf_frequency *pmem) {
            return _mcp_cnf_frequency{pgm_read_byte(pmem->one), pgm_read_byte(pmem->two), pgm_read_byte(pmem->three)};
        }
    };

    static _mcp_cnf_frequency getCnfForClockFrequency(MCP2515_CAN_CLOCK clock, MCP2515_CAN_SPEED baudRate) {
        static const _mcp_cnf_frequency configs [][MCP2515_CAN_SPEED::MCP_SPEED_max] PROGMEM = {
            { // 8MHz
                {0x00, 0x80, 0x00},     // CAN_1000KBPS
                {0x00, 0x90, 0x02},     // CAN_500KBPS
                {0x00, 0xB1, 0x05},     // CAN_250KBPS
                {0x00, 0xB4, 0x06},     // CAN_200KBPS
                {0x01, 0xB1, 0x05},     // CAN_125KBPS
                {0x01, 0xB4, 0x06},     // CAN_100KBPS
                {0x01, 0xBF, 0x07},     // CAN_80KBPS
                {0x03, 0xB4, 0x06},     // CAN_50KBPS
                {0x03, 0xBF, 0x07},     // CAN_40KBPS
                {0x07, 0xBF, 0x07},     // CAN_20KBPS
                {0x0F, 0xBF, 0x07},     // CAN_10KBPS
                {0x1F, 0xBF, 0x07},     // CAN_5KBPS
            },
            { // 16 MHz
                {0x00, 0xD0, 0x82},     // CAN_1000KBPS
                {0x00, 0xF0, 0x86},     // CAN_500KBPS
                {0x41, 0xF1, 0x85},     // CAN_250KBPS
                {0x01, 0xFA, 0x87},     // CAN_200KBPS
                {0x03, 0xF0, 0x86},     // CAN_125KBPS
                {0x03, 0xFA, 0x87},     // CAN_100KBPS
                {0x03, 0xFF, 0x87},     // CAN_80KBPS
                {0x07, 0xFA, 0x87},     // CAN_50KBPS
                {0x07, 0xFF, 0x87},     // CAN_40KBPS
                {0x0F, 0xFF, 0x87},     // CAN_20KBPS
                {0x1F, 0xFF, 0x87},     // CAN_10KBPS
                {0x3F, 0xFF, 0x87},     // CAN_5KBPS
            }
        };
        if (baudRate > MCP2515_CAN_SPEED::MCP_SPEED_max || clock > MCP2515_CAN_CLOCK::MCP_CLOCK_max)
            return {};
        return _mcp_cnf_frequency::copy_P(&configs[clock][baudRate]);
    }
    
    uint8_t _csPin{MCP2515_DEFAULT_CS_PIN};
    uint8_t _intPin{MCP2515_DEFAULT_INT_PIN};
    MCP2515_CAN_CLOCK _clockFrequency;
    SPISettings _spiSettings{10000000, MSBFIRST, SPI_MODE0};
    SPIClass &_spi;

    packetCallback _onReceivePacket;

    bool _oneShotMode{false};
    uint8_t _rxErrorCount{0};

#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    std::queue<CANPacket> _canpacketTxQueue{};
#endif
};

#endif
