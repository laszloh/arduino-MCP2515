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

#include <array>
#include <Arduino.h>
#include <SPI.h>

#include "ErrorCodes.hpp"

// According to VS there is a "OVERFLOW" macro defined in corecrt_math.h
#undef OVERFLOW

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

enum MCP2515_MODES: uint8_t {
    NORMAL = 0,
    SLEEP,
    LOOPBACK,
    LISTEN,
    CONFIG,
};

// defer include to MCP2515_nb.h
class CANPacket;

#ifndef MCP2515_CANPACKET_TX_QUEUE_SIZE
# define MCP2515_CANPACKET_TX_QUEUE_SIZE 16
#endif

#define MCP2515_DEFAULT_CS_PIN  10
#define MCP2515_DEFAULT_INT_PIN 2

using packetCallback = void(*)(CANPacket);

class MCP2515 {
public:
    enum CAN_SPEED: uint8_t {
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

    enum CAN_CLOCK: uint8_t {
        MCP_8MHZ = 0,
        MCP_16MHZ,

        MCP_CLOCK_max
    };

    struct ErrorFlags {
        enum CAN_EFLAG: uint8_t {
            MCP_EFLAG_EWARN     = 0x01,
            MCP_EFLAG_RXWARN    = 0x02,
            MCP_EFLAG_TXWARN    = 0x04,
            MCP_EFLAG_RXEP      = 0x08,
            MCP_EFLAG_TXEP      = 0x10,
            MCP_EFLAG_TXBO      = 0x20,
            MCP_EFLAG_RX0_OVF   = 0x40,
            MCP_EFLAG_RX1_OVF   = 0x80,
        };

        const uint8_t flags;

        ErrorFlags(uint8_t flags): flags(flags) { }
    };


public:
    MCP2515(int cs = MCP2515_DEFAULT_CS_PIN, CAN_CLOCK clk = CAN_CLOCK::MCP_8MHZ, SPIClass &spi = SPI);

    MCP2515Error begin(CAN_SPEED baudRate);
    void loop() { processTxQueue(); }
    void end();

    ErrorFlags getErrorFlags();

    void setSPIFrequency(uint32_t frequency);

    MCP2515Error setMask(const MCP2515_CAN_MASK num, bool extended, uint32_t mask);
    MCP2515Error setFilter(const MCP2515_CAN_RXF num, bool extended, uint32_t filter);

    MCP2515_MODES getMode();
    MCP2515Error setConfigMode();
    MCP2515Error setListenMode();
    MCP2515Error setLoopbackMode();
    MCP2515Error setSleepMode();
    MCP2515Error setNormalMode();

    MCP2515Error setWakeupFilter(bool enable);
    MCP2515Error setOneShotMode(bool enable);

    MCP2515Error receivePacket(CANPacket &packet);
    void onReceivePacket(packetCallback callback)  { _onReceivePacket = callback; }

    size_t getTxQueueLength();
    void processTxQueue();

    MCP2515Error writePacket(const CANPacket &packet, bool nowait = false);

    void handleInterrupt();

private:
    void reset();

    uint8_t readRegister(uint8_t address);
    void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readStatus();
    inline MCP2515Error setMode(MCP2515_MODES mode);

    template<size_t N> void writeRegisters(uint8_t address, const std::array<uint8_t, N> &data, uint8_t size) {
        _spi.beginTransaction(_spiSettings);
        digitalWrite(_csPin, LOW);

        uint8_t cmd = 0x00;
        switch(address) {
            case 0x31:
                cmd = 0x40;
                break;
            case 0x36:
                cmd = 0x41;
                break;
            case 0x41:
                cmd = 0x42;
                break;
            case 0x46:
                cmd = 0x43;
                break;
            case 0x51:
                cmd = 0x44;
                break;
            case 0x56:
                cmd = 0x45;
                break;
        }

        if(cmd) {
            // send data
            _spi.transfer(cmd);
            for(uint8_t i = 0; i < size; i++) {
                _spi.transfer(data[i]);
            }
        }

        digitalWrite(_csPin, HIGH);
        _spi.endTransaction();
    }

    MCP2515Error handleMessageTransmit(int txBuf, bool cond);
    int8_t getFreeTxBuffer();

    static std::array<uint8_t, 8 + 5> serialize(const CANPacket &packet);

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

    static _mcp_cnf_frequency getCnfForClockFrequency(CAN_CLOCK clock, CAN_SPEED baudRate);
    
    uint8_t _csPin;
    CAN_CLOCK _clockFrequency;
    SPISettings _spiSettings{10000000, MSBFIRST, SPI_MODE0};
    SPIClass &_spi;

    packetCallback _onReceivePacket{nullptr};

    bool _oneShotMode{false};
    uint8_t _rxErrorCount{0};

#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    std::queue<CANPacket> _canpacketTxQueue{};
#endif
};

#endif
