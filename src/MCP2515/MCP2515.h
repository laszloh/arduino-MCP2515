/**
 * CAN MCP2515_nb
 * Copyright 2020 WitchCraftWorks Team, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#ifndef MCP2515_H
#define MCP2515_H

#include <array>
#include <Arduino.h>
#include <SPI.h>

#include "CANPacket.hpp"
#include "ErrorCodes.hpp"
#include "mcp2515_def.h"

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

#define MCP2515_DEFAULT_CS_PIN  10
#define MCP2515_DEFAULT_INT_PIN 2

class MCP2515 {
public:
    enum CanSpeed: uint8_t {
        CAN_1KBPS,
        CAN_5KBPS,
        CAN_10KBPS,
        CAN_12K5BPS,
        CAN_16KBPS,
        CAN_20KBPS,
        CAN_25KBPS,
        CAN_31K25BPS,
        CAN_33KBPS,
        CAN_40KBPS,
        CAN_50KBPS,
        CAN_80KBPS,
        CAN_83K3BPS,
        CAN_95KBPS,
        CAN_100KBPS,
        CAN_125KBPS,
        CAN_200KBPS,
        CAN_250KBPS,
        CAN_500KBPS,
        CAN_800KBPS,
        CAN_1000KBPS
    };

    enum CanClock: uint8_t {
        MCP_20MHZ,
        MCP_16MHZ,
        MCP_12MHZ,
        MCP_8MHZ,
    };

    enum CanClkOut {
        CLKOUT_DISABLE = -1,
        CLKOUT_DIV1 = 0x0,
        CLKOUT_DIV2 = 0x1,
        CLKOUT_DIV4 = 0x2,
        CLKOUT_DIV8 = 0x3,
    };

    enum CanModes : uint8_t {
        MCP_NORMAL = 0x00,
        MCP_SLEEP = 0x20,
        MCP_LOOPBACK = 0x40,
        MCP_LISTENONLY = 0x60,
        MCP_CONFIG = 0x80
    };

    struct ErrorFlags {
        enum CAN_EFLAG: uint16_t {
            MCP_EFLAG_EWARN     = 0x0001,
            MCP_EFLAG_RXWARN    = 0x0002,
            MCP_EFLAG_TXWARN    = 0x0004,
            MCP_EFLAG_RXEP      = 0x0008,
            MCP_EFLAG_TXEP      = 0x0010,
            MCP_EFLAG_TXBO      = 0x0020,
            MCP_EFLAG_RX0_OVF   = 0x0040,
            MCP_EFLAG_RX1_OVF   = 0x0080,
            MCP_EFLAG_MERR      = 0x0100,
            MCP_EFLAG_ERR       = 0x0200,

        };

        const uint16_t flags;

        ErrorFlags(uint16_t flags): flags(flags) { }

        explicit operator bool() const {
            return flags;
        }
    };


public:
    MCP2515(int cs = MCP2515_DEFAULT_CS_PIN, CanClock clk = CanClock::MCP_8MHZ, SPIClass &spi = SPI);

    MCP2515Error begin(CanSpeed baudRate);
    void end();

    ErrorFlags getErrorFlags();
    void clearErrorFlags();

    void setSPIFrequency(uint32_t frequency);
    MCP2515Error setBitrate(CanSpeed speed, CanClock clock = MCP_16MHZ);

    MCP2515Error setMask(const MCP2515_CAN_MASK num, bool extended, uint32_t mask);
    MCP2515Error setFilter(const MCP2515_CAN_RXF num, bool extended, uint32_t filter);

    CanModes getMode();
    MCP2515Error setConfigMode();
    MCP2515Error setListenMode();
    MCP2515Error setLoopbackMode();
    MCP2515Error setSleepMode();
    MCP2515Error setNormalMode();

    MCP2515Error setWakeupFilter(bool enable);
    MCP2515Error setOneShotMode(bool enable);
    MCP2515Error setClockOut(const CanClkOut divisor);

    bool checkMessage();
    MCP2515Error readMessage(CANPacket &packet);

    MCP2515Error sendMessage(const CANPacket &packet);

private:
    inline void spiEnable();
    inline void spiDisable();
    MCP2515Error reset();

    uint8_t readRegister(const uint8_t address);
    void readRegisters(const uint8_t address, uint8_t val[], const uint8_t n);
    void setRegister(const uint8_t address, const uint8_t value);
    void setRegisters(const uint8_t address, const uint8_t values[], const uint8_t n);
    void modifyRegister(const uint8_t address, const uint8_t mask, const uint8_t value);
    uint8_t getStatus();

    inline MCP2515Error setMode(const internal::CanctrlReqopMode mode);

    MCP2515Error readMessage(internal::RXBn rxbn, CANPacket &packet);
    MCP2515Error sendMessage(internal::TXBn txbn, const CANPacket &packet);

    static std::array<uint8_t, CANPacket::MAX_DATA_LENGTH + 5> serialize(const CANPacket &packet);
    
    static constexpr size_t nTxBuffers = 3;
    static constexpr struct TxBnRegs {
        uint8_t CTRL;
        uint8_t SIDH;
        uint8_t DATA;
    } TXB[nTxBuffers] = {
        {internal::MCP_TXB0CTRL, internal::MCP_TXB0SIDH, internal::MCP_TXB0DATA},
        {internal::MCP_TXB1CTRL, internal::MCP_TXB1SIDH, internal::MCP_TXB1DATA},
        {internal::MCP_TXB2CTRL, internal::MCP_TXB2SIDH, internal::MCP_TXB2DATA},
    };

    static constexpr size_t nRxBuffers = 2;
    static constexpr struct RxBnRegs {
        uint8_t CTRL;
        uint8_t SIDH;
        uint8_t DATA;
        uint8_t CANINTF_RXnIF;
    } RXB[nRxBuffers] = {
        {internal::MCP_RXB0CTRL, internal::MCP_RXB0SIDH, internal::MCP_RXB0DATA, internal::CANINTF_RX0IF},
        {internal::MCP_RXB1CTRL, internal::MCP_RXB1SIDH, internal::MCP_RXB1DATA, internal::CANINTF_RX1IF}
    };

    uint8_t _csPin;
    CanClock _clockFrequency;
    SPISettings _spiSettings{10000000, MSBFIRST, SPI_MODE0};
    SPIClass &_spi;
};

#endif
