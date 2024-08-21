/**
 * CAN MCP2515_nb
 * Copyright 2020 WitchCraftWorks Team, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#include "MCP2515.h"
#include "CANPacket.hpp"

#define SPI_CMD_RST 0x0C
#define SPI_CMD_RD 0x03
#define SPI_CMD_WR 0x02
#define SPI_CMD_ST 0xA0
#define SPI_CMD_RXST 0xB0
#define SPI_CMD_BMD 0x05 

#define REG_BFPCTRL 0x0C
#define REG_TXRTSCTRL 0x0D

#define REG_CANSTAT 0x0E
#define REG_CANCTRL 0x0F

#define REG_TX_ERROR_COUNTER 0x1C
#define REG_RX_ERROR_COUNTER 0x1D

#define REG_CNF3 0x28
#define REG_CNF2 0x29
#define REG_CNF1 0x2A

#define REG_CANINTE 0x2B
#define REG_CANINTF 0x2C
#define REG_EFLG 0x2D

#define FLAG_RXnIE(n) (0x01 << n)
#define FLAG_RXnIF(n) (0x01 << n)
#define FLAG_TXnIF(n) (0x04 << n)

#define REG_RXFnSIDH(n) ((n) < 3 ? (0x00 + (n) * 4) : (0x10 + ((n) - 3) * 4))
#define REG_RXFnSIDL(n) ((n) < 3 ? (0x01 + (n) * 4) : (0x11 + ((n) - 3) * 4))
#define REG_RXFnEID8(n) ((n) < 3 ? (0x02 + (n) * 4) : (0x12 + ((n) - 3) * 4))
#define REG_RXFnEID0(n) ((n) < 3 ? (0x03 + (n) * 4) : (0x13 + ((n) - 3) * 4))

#define REG_RXMnSIDH(n) (0x20 + (n * 0x04))
#define REG_RXMnSIDL(n) (0x21 + (n * 0x04))
#define REG_RXMnEID8(n) (0x22 + (n * 0x04))
#define REG_RXMnEID0(n) (0x23 + (n * 0x04))

#define REG_TXB_COUNT 3
#define REG_TXBnCTRL(n) (0x30 + (n * 0x10))
#define FLAG_TXREQ  0x08
#define FLAG_TXERR  0x10
#define FLAG_TXMLOA 0x20
#define FLAG_TXABTF 0x40

#define REG_TXBnSIDH(n) (0x31 + (n * 0x10))
#define REG_TXBnSIDL(n) (0x32 + (n * 0x10))
#define REG_TXBnEID8(n) (0x33 + (n * 0x10))
#define REG_TXBnEID0(n) (0x34 + (n * 0x10))
#define REG_TXBnDLC(n) (0x35 + (n * 0x10))
#define FLAG_RTR 0x40

#define REG_TXBnD0(n) (0x36 + (n * 0x10))

#define REG_RXBnCTRL(n) (0x60 + (n * 0x10))
#define REG_RXBnSIDH(n) (0x61 + (n * 0x10))
#define REG_RXBnSIDL(n) (0x62 + (n * 0x10))
#define REG_RXBnEID8(n) (0x63 + (n * 0x10))
#define REG_RXBnEID0(n) (0x64 + (n * 0x10))
#define REG_RXBnDLC(n) (0x65 + (n * 0x10))
#define REG_RXBnD0(n) (0x66 + (n * 0x10))

#define FLAG_IDE 0x08
#define FLAG_SRR 0x10
#define FLAG_RTR 0x40
#define FLAG_EXIDE 0x08

#define FLAG_RXM0 0x20
#define FLAG_RXM1 0x40

#define FLAG_ST_RX0IF   0x01
#define FLAG_ST_RX1IF   0x02
#define FLAG_ST_TX0REQ  0x04
#define FLAG_ST_TX0IF   0x08
#define FLAG_ST_TX1REQ  0x10
#define FLAG_ST_TX1IF   0x20
#define FLAG_ST_TX2REQ  0x40
#define FLAG_ST_TX2IF   0x80

MCP2515::MCP2515(int cs, CAN_CLOCK clk, SPIClass &spi) :
    _csPin(cs),
    _clockFrequency(clk),
    _spi(spi)
{
}

MCP2515Error MCP2515::begin(CAN_SPEED baudRate) {
    pinMode(_csPin, OUTPUT);

    SPI.begin();
    reset();

    writeRegister(REG_CANCTRL, 0x80);
    if (readRegister(REG_CANCTRL) != 0x80) {
        return MCP2515Error::BADF;
    }

    _mcp_cnf_frequency cnf = getCnfForClockFrequency(_clockFrequency, baudRate);
    if (!cnf)
        return MCP2515Error::INVAL;

    writeRegister(REG_CNF1, cnf.one);
    writeRegister(REG_CNF2, cnf.two);
    writeRegister(REG_CNF3, cnf.three);

    writeRegister(REG_CANINTE, (FLAG_RXnIE(1) | FLAG_RXnIE(0)));
    writeRegister(REG_BFPCTRL, 0x00);
    writeRegister(REG_TXRTSCTRL, 0x00);
    writeRegister(REG_RXBnCTRL(0), 0x00);
    writeRegister(REG_RXBnCTRL(1), 0x00);

    return setNormalMode();
}

void MCP2515::end() {
    reset();
    SPI.end();
}

MCP2515::ErrorFlags MCP2515::getErrorFlags() {
    return ErrorFlags{readRegister(REG_EFLG)};
}

void MCP2515::setSPIFrequency(uint32_t frequency) {
    _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

MCP2515Error MCP2515::setMask(const MCP2515_CAN_MASK num, bool extended, uint32_t mask) {
    auto err = setConfigMode();
    if(err)
        return err;

    uint8_t eid0 = 0;
    uint8_t eid8 = 0;
    uint8_t sidh;
    uint8_t sidl;

    if (extended) {
        uint16_t canfilter = mask & 0x0FFFF;
        eid0 = canfilter & 0xFF;
        eid8 = canfilter >> 8;

        uint16_t canid = mask >> 16;
        sidh = canid >> 5;
        sidl = ((canid & 0x03) + ((canid & 0x1C) << 3)) | FLAG_EXIDE;
    } else {
        uint16_t canfilter = mask & 0x0FFFF;
        sidh = canfilter >> 3;
        sidl = (canfilter & 0x07) << 5;
    }

    writeRegister(REG_RXMnEID0(num), eid0);
    writeRegister(REG_RXMnEID8(num), eid8);
    writeRegister(REG_RXMnSIDH(num), sidh);
    writeRegister(REG_RXMnSIDL(num), sidl);

    return setNormalMode();
}

MCP2515Error MCP2515::setFilter(const MCP2515_CAN_RXF num, bool extended, uint32_t filter) {
    auto err = setConfigMode();
    if(err)
        return err;

    uint8_t eid0 = 0;
    uint8_t eid8 = 0;
    uint8_t sidh;
    uint8_t sidl;

    if (extended) {
        uint16_t canfilter = filter & 0x0FFFF;
        eid0 = canfilter & 0xFF;
        eid8 = canfilter >> 8;

        uint16_t canid = filter >> 16;
        sidh = canid >> 5;
        sidl = ((canid & 0x03) + ((canid & 0x1C) << 3)) | FLAG_EXIDE;
    } else {
        uint16_t canfilter = filter & 0x0FFFF;
        sidh = canfilter >> 3;
        sidl = (canfilter & 0x07) << 5;
    }

    writeRegister(REG_RXFnEID0(num), eid0);
    writeRegister(REG_RXFnEID8(num), eid8);
    writeRegister(REG_RXFnSIDH(num), sidh);
    writeRegister(REG_RXFnSIDL(num), sidl);

    return setNormalMode();
}

MCP2515_MODES MCP2515::getMode() {
    return static_cast<MCP2515_MODES>(readRegister(REG_CANCTRL) >> 5);
}

MCP2515Error MCP2515::setConfigMode() {
    return setMode(MCP2515_MODES::CONFIG);
}

MCP2515Error MCP2515::setListenMode() {
    return setMode(MCP2515_MODES::LISTEN);
}

MCP2515Error MCP2515::setLoopbackMode() {
    return setMode(MCP2515_MODES::LOOPBACK);
}

MCP2515Error MCP2515::setSleepMode() {
    auto err = setMode(MCP2515_MODES::SLEEP);
    if(err)
        return err;

    // Block until CAN controller goes into sleep.
    // "These bits should be read after sending the SLEEP command to the MCP2515.
    // The MCP2515 is active and has not yet entered Sleep mode until these bits
    // indicate that Sleep mode has been entered."
    while ((readRegister(REG_CANCTRL) & 0xE0) != 0x20) {
        yield();
    }

    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setNormalMode() {
    return setMode(MCP2515_MODES::NORMAL);
}

MCP2515Error MCP2515::setMode(MCP2515_MODES mode) {
    uint8_t raw = static_cast<uint8_t>(mode) << 5;
    modifyRegister(REG_CANCTRL, 0xE0, raw);
    if( (readRegister(REG_CANCTRL) & 0xE0) != raw)
        return MCP2515Error::BADF;
    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setWakeupFilter(bool enable) {
    uint8_t envalue = (enable ? 0x40 : 0);
    modifyRegister(REG_CNF3, 0x40, envalue);

    if ((readRegister(REG_CNF3) & 0x40) != envalue) {
        return MCP2515Error::BADF;
    }

    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setOneShotMode(bool enable) {
    uint8_t envalue = (enable ? 0x08 : 0);
    modifyRegister(REG_CANCTRL, 0x08, envalue);

    if ((readRegister(REG_CANCTRL) & 0x08) != envalue) {
        return MCP2515Error::BADF;
    }

    _oneShotMode = enable;
    return MCP2515Error::OK;
}

MCP2515Error MCP2515::receivePacket(CANPacket &packet) {
    int rxBuf;
    uint8_t intf = readStatus();

    if (intf & FLAG_ST_RX0IF) {
        rxBuf = 0;
    } else if (intf & FLAG_ST_RX1IF) {
        rxBuf = 1;
    } else {
        return MCP2515Error::NOENT;
    }

    packet._extended = (readRegister(REG_RXBnSIDL(rxBuf)) & FLAG_IDE ? true : false);

    uint8_t sidh = readRegister(REG_RXBnSIDH(rxBuf));
    uint8_t sidl = readRegister(REG_RXBnSIDL(rxBuf));

    uint32_t idA = ((sidh << 3) & 0x07F8) | ((sidl >> 5) & 0x07);

    if (packet._extended) {
        uint32_t idB = (((uint32_t)(sidl & 0x03) << 16) & 0x30000) | ((readRegister(REG_RXBnEID8(rxBuf)) << 8) & 0xFF00) | readRegister(REG_RXBnEID0(rxBuf));

        packet._id = (idA << 18) | idB;
        packet._rtr = (readRegister(REG_RXBnDLC(rxBuf)) & FLAG_RTR ? true : false);
    } else {
        packet._id = idA;
        packet._rtr = (sidl & FLAG_SRR ? true : false);
    }

    packet._dlc = readRegister(REG_RXBnDLC(rxBuf)) & 0x0F;

    if (!packet._rtr) {
        // READ RX BUFFER
        SPI.beginTransaction(_spiSettings);
        digitalWrite(_csPin, LOW);

        SPI.transfer((0x92 | (rxBuf << 2)));

        for (uint8_t i = 0; i < packet._dlc; i++) {
            packet.writeData(SPI.transfer(0x00));
        }

        digitalWrite(_csPin, HIGH);
        SPI.endTransaction();
    }
    modifyRegister(REG_CANINTF, FLAG_RXnIF(rxBuf), 0x00);

    return MCP2515Error::OK;
}

size_t MCP2515::getTxQueueLength() {
#ifdef MCP2515_DISABLE_ASYNC_TX_QUEUE
    return 0;
#else
    return _canpacketTxQueue.size();
#endif
}

void MCP2515::processTxQueue() {
#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
    if (_canpacketTxQueue.empty()) {
        return;
    }

    if(getFreeTxBuffer() < 0) {
        // All TX buffers have packets pending
        return;
    }

    // send packet
    const CANPacket &packet = _canpacketTxQueue.front();

    auto err = writePacket(packet, true);
    if(!err) {
        // send was successful
        _canpacketTxQueue.pop();
    }
#endif
}

MCP2515Error MCP2515::writePacket(const CANPacket &packet, bool nowait) {
    if (!packet) {
        return MCP2515Error::INVAL;
    }
    // MCP controller is NOT in normal or loopback mode
    uint8_t canctrl = readRegister(REG_CANCTRL) & 0xE0;
    if (canctrl != 0x00 && canctrl != 0x40) {
        return MCP2515Error::COMM;
    }

    int8_t txBuf = getFreeTxBuffer();
    if(txBuf < 0) {
        // there were no empty tx buffers, push packet into queue
#ifndef MCP2515_DISABLE_ASYNC_TX_QUEUE
        if(getTxQueueLength() > MCP2515_CANPACKET_TX_QUEUE_SIZE) {
            return MCP2515Error::OVERFLOW;
        }

        _canpacketTxQueue.push(packet);
        return MCP2515Error::OK;
#else
        return MCP2515Error::AGAIN;
#endif
    }

    // we have an empty buffer, write it
    noInterrupts();

    // Clear TxControl register
    writeRegister(REG_TXBnCTRL(txBuf), 0x00);

    // Clear abort bit (might have been set previously)
    modifyRegister(REG_CANCTRL, 0x10, 0x00);

    auto data = serialize(packet);

    // write the raw data directly to the tx buffer
    writeRegisters(REG_TXBnSIDH(txBuf), data, packet._dlc + 5);

    // trigger tx ready for this buffer
    modifyRegister(REG_TXBnCTRL(txBuf), FLAG_TXREQ, FLAG_TXREQ);

    interrupts();

    if (nowait) {
        return MCP2515Error::OK;
    }

    return handleMessageTransmit(txBuf, true);
}

MCP2515Error MCP2515::handleMessageTransmit(int txBuf, bool cond) {
    MCP2515Error status = MCP2515Error::AGAIN;

    do {
        uint8_t txbctrl = readRegister(REG_TXBnCTRL(txBuf));

        if (txbctrl & FLAG_TXABTF) {
            modifyRegister(REG_CANCTRL, 0x10, 0x00);
            status = MCP2515Error::INTR;
            break;
        } else if (txbctrl & FLAG_TXERR) {
            status = MCP2515Error::BADF;
            break;
        } else if ((txbctrl & FLAG_TXREQ) == 0) {
            status = MCP2515Error::OK;
            break;
        }
        yield();

        if (cond) {
            delayMicroseconds(10);
        }
    } while (cond);

    if (status != MCP2515Error::AGAIN && (_oneShotMode || status != MCP2515Error::BADF)) {
        modifyRegister(REG_TXBnCTRL(txBuf), FLAG_TXREQ, 0x00);
        modifyRegister(REG_CANINTF, FLAG_TXnIF(txBuf), 0x00);
    }

    return status;
}

void MCP2515::handleInterrupt() {
    uint8_t intf = readRegister(REG_CANINTF);

    if (intf == 0) {
        return;
    }

    // process receive queue
    if (intf & (FLAG_RXnIF(0) | FLAG_RXnIF(1))) {
        CANPacket packet;
        while(receivePacket(packet) != MCP2515Error::NOENT) {
            if(packet && _onReceivePacket)
                _onReceivePacket(packet);
        }
    }

    // process tx queue
    if(intf & (FLAG_TXnIF(0) | FLAG_TXnIF(1) | FLAG_TXnIF(2)))
        processTxQueue();

    // Clear all TXnIF + MERRF bits
    modifyRegister(REG_CANINTF, 0x9C, 0x00);
}

MCP2515::_mcp_cnf_frequency MCP2515::getCnfForClockFrequency(CAN_CLOCK clock, CAN_SPEED baudRate) {
    static const _mcp_cnf_frequency configs [][CAN_SPEED::MCP_SPEED_max] PROGMEM = {
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
    if (baudRate > CAN_SPEED::MCP_SPEED_max || clock > CAN_CLOCK::MCP_CLOCK_max)
        return {};
    return _mcp_cnf_frequency::copy_P(&configs[clock][baudRate]);
}

int8_t MCP2515::getFreeTxBuffer() {
    uint8_t status = readStatus();

    // find the first free buffer
    if(!(status & FLAG_ST_TX0REQ))
        return 0;
    else if(!(status & FLAG_ST_TX1REQ))
        return 1;
    else if(!(status & FLAG_ST_TX2REQ))
        return 2;
    else 
        return -1;
}

void MCP2515::reset() {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    SPI.transfer(SPI_CMD_RST);

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();

    delay(10);
}

uint8_t MCP2515::readRegister(uint8_t address) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    SPI.transfer(SPI_CMD_RD);
    SPI.transfer(address);
    uint8_t value = SPI.transfer(0x00);

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();

    return value;
}

void MCP2515::modifyRegister(uint8_t address, uint8_t mask, uint8_t value) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    SPI.transfer(SPI_CMD_BMD);
    SPI.transfer(address);
    SPI.transfer(mask);
    SPI.transfer(value);

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}

void MCP2515::writeRegister(uint8_t address, uint8_t value) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    SPI.transfer(SPI_CMD_WR);
    SPI.transfer(address);
    SPI.transfer(value);

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}

uint8_t MCP2515::readStatus() {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    SPI.transfer(SPI_CMD_ST);
    uint8_t val = SPI.transfer(0x00);

    digitalWrite(_csPin, HIGH);
    return val;
}

std::array<uint8_t, 8 + 5> MCP2515::serialize(const CANPacket &packet) {
    std::array<uint8_t, 8+5> dat;

    if(packet._extended) {
        dat[0] = (packet._id >> 21);
        dat[1] = ((((packet._id >> 18) & 0x07) << 5) | FLAG_EXIDE | ((packet._id >> 16) & 0x03));
        dat[2] = ((packet._id >> 8) & 0xFF);
        dat[3] = (packet._id & 0xFF);
    } else {
        dat[0] = (packet._id >> 3);
        dat[1] = (packet._id << 5);
        dat[2] = 0x00;
        dat[3] = 0x00;
    }

    dat[4] = packet._dlc;

    std::copy(packet._data.begin(), packet._data.begin() + packet._dlc, dat.begin() + 5);
    return dat;
}
