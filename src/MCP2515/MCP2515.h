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

#define MCP2515_DEFAULT_CS_PIN  10
#define MCP2515_DEFAULT_INT_PIN 2

/// @brief MCP2515 driver class
class MCP2515 {
public:
     /// @brief CAN baudrate configration values
     /// @attention Not all combination of MCP2515 clock & baudrate are supported
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

    /// @brief Oscillator frequency of the MCP2515
    enum CanClock: uint8_t {
        MCP_20MHZ,
        MCP_16MHZ,
        MCP_12MHZ,
        MCP_8MHZ,
    };

    /// @brief CLKOUT pin configrations
    enum CanClkOut {
        CLKOUT_DISABLE = -1,
        CLKOUT_DIV1 = 0x0,
        CLKOUT_DIV2 = 0x1,
        CLKOUT_DIV4 = 0x2,
        CLKOUT_DIV8 = 0x3,
    };

    /// @brief CAN protocol engine modes
    enum CanModes : uint8_t {
        MCP_NORMAL = 0x00,
        MCP_SLEEP = 0x20,
        MCP_LOOPBACK = 0x40,
        MCP_LISTENONLY = 0x60,
        MCP_CONFIG = 0x80
    };

    /// @brief Selection values for the CAN-ID filter masks
    enum MASK: uint8_t {
        MASK0 = 0,
        MASK1 = 1
    };

    /// @brief Selection values for the CAN-ID filters
    enum RXF: uint8_t {
        RXF0 = 0,
        RXF1 = 1,
        RXF2 = 2,
        RXF3 = 3,
        RXF4 = 4,
        RXF5 = 5
    };

    /// @brief All error related flags
    /// See http://www.can-wiki.info/doku.php?id=can_faq:can_faq_erors for further 
    /// information about the CAN warning and error states
    struct ErrorFlags {
        enum CAN_EFLAG: uint16_t {
            MCP_EFLG_EWARN     = 0x0001,
            MCP_EFLG_RXWARN    = 0x0002,
            MCP_EFLG_TXWARN    = 0x0004,
            MCP_EFLG_RXEP      = 0x0008,
            MCP_EFLG_TXEP      = 0x0010,
            MCP_EFLG_TXBO      = 0x0020,
            MCP_EFLG_RX0_OVF   = 0x0040,
            MCP_EFLG_RX1_OVF   = 0x0080,
            MCP_EFLG_MERR      = 0x0100,
            MCP_EFLG_ERR       = 0x0200,

        };

        /// @brief Access to the raw value
        /// @return a reference to the underlying error value
        const uint8_t &raw() const { return flags; }

        /// @brief Create a new ErrorFlags object
        /// @param flags The value of the error flags
        /// @param tec transmit error counter value
        /// @param rec receive error counter value
        constexpr ErrorFlags(uint16_t flags, uint8_t tec, uint8_t rec): flags(flags), tec(tec), rec(rec) { }

        /// @brief returns true if any error is present 
        explicit operator bool() const {
            return flags;
        }

        /// @brief Status of the EWARN bit in EFLG register
        /// This bit is set, when the TEC or REC is equal or greater than 96
        /// @return true if the bit is set, false otherwise
        bool errorWarning() const { return flags & MCP_EFLG_EWARN; }

        /// @brief Status of the RXWARN bit in EFLG regsiter
        /// This bit is set, when REC equal or greater than 96
        /// @return true if the bit is set, false otherwise
        bool rxErrorWarning() const { return flags & MCP_EFLG_RXWARN; }

        /// @brief Status of the TXWARN bit in EFLG regsiter
        /// This bit is set, when TEC equal or greater than 96
        /// @return true if the bit is set, false otherwise
        bool txErrorWarning() const { return flags & MCP_EFLG_TXWARN; }

        /// @brief Status of the RXEP bit in EFLG regsiter
        /// This bit is set, when REC equal or greater than 128
        /// @return true if the bit is set, false otherwise
        bool rxErrorPassive() const { return flags & MCP_EFLG_RXEP; }

        /// @brief Status of the TXEP bit in EFLG regsiter
        /// This bit is set, when TEC equal or greater than 128
        /// @return true if the bit is set, false otherwise
        bool txErrorPassive() const { return flags & MCP_EFLG_TXEP; }

        /// @brief Status of the TXBO bit in EFLG register
        /// this bit is set, when TEC is greater than 255
        /// @return true if the bit is set, false otherwise
        bool txBusOff() const { return flags & MCP_EFLG_TXBO; }

        /// @brief Number of transmit errors detected by this node
        /// @return The value of the TEC register at the time of the ErrorFlags creation
        const uint8_t &txErrorCounter() const { return tec; }

        /// @brief Number of receive errors detected by this node
        /// @return The value of the REC register at the time of the ErrorFlags creation
        const uint8_t &rxErrorCounter() const { return rec; }

        // RX buffer overflows
        bool rxBufferOverflow() const { return flags & (MCP_EFLG_RX0_OVF | MCP_EFLG_RX1_OVF); }
        bool rxBuffer0Overflow() const { return flags & MCP_EFLG_RX0_OVF; }
        bool rxBuffer1Overflow() const { return flags & MCP_EFLG_RX1_OVF; }

        // error interrupt flags
        bool generalErrorIntFlags() const { return flags & MCP_EFLG_ERR; }
        bool messageErrorIntFlag() const { return flags & MCP_EFLG_MERR; }

    protected:
        const uint16_t flags;
        const uint8_t tec;
        const uint8_t rec;
   };


public:
    MCP2515(int cs = MCP2515_DEFAULT_CS_PIN, CanClock clk = CanClock::MCP_8MHZ, SPIClass &spi = SPI);

    MCP2515Error begin(CanSpeed baudRate);
    MCP2515Error begin(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);
    void end();

    ErrorFlags getErrorFlags();
    uint8_t getTxErrorCount();
    uint8_t getRxErrorCount();
    void clearErrorFlags();

    void setSPIFrequency(uint32_t frequency);
    MCP2515Error setBitrate(CanSpeed speed);

    void setBitrate(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);
    MCP2515Error setMask(const MASK num, bool extended, uint32_t mask);
    MCP2515Error setFilter(const RXF num, bool extended, uint32_t filter);

    CanModes getMode();
    MCP2515Error setConfigMode();
    MCP2515Error setListenMode();
    MCP2515Error setLoopbackMode();
    MCP2515Error setSleepMode();
    MCP2515Error setNormalMode();

    void setWakeupFilter(bool enable);
    void setOneShotMode(bool enable);
    void setClockOut(const CanClkOut divisor);
    void setRxBufferRollover(bool enable);

    bool checkMessage();
    MCP2515Error readMessage(CANPacket &packet);

    MCP2515Error sendMessage(const CANPacket &packet);

protected:
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
    SPISettings _spiSettings{4000000, MSBFIRST, SPI_MODE0};
    SPIClass &_spi;
};

#endif
