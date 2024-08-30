/**
 * CAN MCP2515_nb
 * Copyright 2020 WitchCraftWorks Team, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#include "MCP2515.h"
#include "CANPacket.hpp"

using namespace internal; 

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

MCP2515::MCP2515(int cs, CanClock clk, SPIClass &spi) :
    _csPin(cs),
    _clockFrequency(clk),
    _spi(spi)
{
}

MCP2515Error MCP2515::begin(CanSpeed baudRate) {
    pinMode(_csPin, OUTPUT);
    _spi.begin();
    
    if(reset())
        return MCP2515Error::FAILINIT;
    if(setBitrate(baudRate)) 
        return MCP2515Error::FAILINIT;
    return setNormalMode();
}

MCP2515Error MCP2515::begin(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3) {
    auto err = begin(CAN_1000KBPS);
    if(err)
        return err;
    
    setBitrate(cnf1, cnf2, cnf3);
    return setNormalMode();
}

void MCP2515::end() {
    reset();
    setSleepMode();
}

MCP2515::ErrorFlags MCP2515::getErrorFlags() {
    uint16_t flags = readRegister(MCP_EFLG);
    
    uint8_t canIntF = readRegister(MCP_CANINTF);
    flags |= (canIntF & CANINTF_MERRF) ? ErrorFlags::MCP_EFLG_MERR : 0x00;
    flags |= (canIntF & CANINTF_ERRIF) ? ErrorFlags::MCP_EFLG_ERR : 0x00;

    uint8_t tec = readRegister(MCP_TEC);
    uint8_t rec = readRegister(MCP_REC);

    return ErrorFlags{flags, tec, rec};
}

uint8_t MCP2515::getTxErrorCount() {
    return readRegister(MCP_TEC);
}

uint8_t MCP2515::getRxErrorCount() {
    return readRegister(MCP_REC);
}

void MCP2515::clearErrorFlags() {
    modifyRegister(MCP_EFLG, EFLG_RX1OVR | EFLG_RX0OVR, 0x00);
    modifyRegister(MCP_CANINTF, CANINTF_MERRF | CANINTF_ERRIF, 0x00);
}

void MCP2515::setSPIFrequency(uint32_t frequency) {
    _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

MCP2515Error MCP2515::setMask(const MASK num, bool extended, uint32_t mask) {
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

    setRegister(REG_RXMnEID0(num), eid0);
    setRegister(REG_RXMnEID8(num), eid8);
    setRegister(REG_RXMnSIDH(num), sidh);
    setRegister(REG_RXMnSIDL(num), sidl);

    return setNormalMode();
}

MCP2515Error MCP2515::setFilter(const RXF num, bool extended, uint32_t filter) {
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

    setRegister(REG_RXFnEID0(num), eid0);
    setRegister(REG_RXFnEID8(num), eid8);
    setRegister(REG_RXFnSIDH(num), sidh);
    setRegister(REG_RXFnSIDL(num), sidl);

    return setNormalMode();
}

MCP2515::CanModes MCP2515::getMode() {
    return static_cast<CanModes>(readRegister(MCP_CANSTAT) & CANSTAT_OPMOD);
}

MCP2515Error MCP2515::setConfigMode() {
    return setMode(CanctrlReqopMode::CANCTRL_REQOP_CONFIG);
}

MCP2515Error MCP2515::setListenMode() {
    return setMode(CanctrlReqopMode::CANCTRL_REQOP_LISTENONLY);
}

MCP2515Error MCP2515::setLoopbackMode() {
    return setMode(CanctrlReqopMode::CANCTRL_REQOP_LOOPBACK);
}

MCP2515Error MCP2515::setSleepMode() {
    return setMode(CanctrlReqopMode::CANCTRL_REQOP_SLEEP);
}

MCP2515Error MCP2515::setNormalMode() {
    return setMode(CanctrlReqopMode::CANCTRL_REQOP_NORMAL);
}

MCP2515Error MCP2515::setMode(const CanctrlReqopMode mode) {
    modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);

    uint32_t timeout = millis() + 10;
    while(millis() < timeout) {
        if( (readRegister(MCP_CANSTAT) & CANSTAT_OPMOD) == mode) 
            return MCP2515Error::OK;
    }
    return MCP2515Error::FAIL;
}

void MCP2515::setWakeupFilter(bool enable) {
    uint8_t envalue = (enable ? CNF3_WAKFIL : 0x00);
    modifyRegister(MCP_CNF3, CNF3_WAKFIL, envalue);
}

void MCP2515::setOneShotMode(bool enable) {
    uint8_t envalue = (enable ? CANCTRL_OSM : 0x00);
    modifyRegister(MCP_CANCTRL, CANCTRL_OSM, envalue);
}

void MCP2515::setClockOut(const CanClkOut divisor) {
    if(divisor == CLKOUT_DISABLE) {
        // disable CLKEN
        modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

        // enable SOF for CLKOUT pin
        modifyRegister(MCP_CNF3, CNF3_SOF, CNF3_SOF);
    }

    // set prescaler
    modifyRegister(MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

    // enable clock out
    modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN);

    // disable SOF for clockout pin
    modifyRegister(MCP_CNF3, CNF3_SOF, 0x00);
}

void MCP2515::setRxBufferRollover(bool enable) {
    modifyRegister(MCP_RXB0CTRL, RXB_0_CTRL_BUKT, (enable) ? 0xFF : 0x00);
}

MCP2515Error MCP2515::readMessage(RXBn rxbn, MCP2515CanPaket &packet) {
    const struct RxBnRegs *rxb = &RXB[rxbn];

    packet._rxBuffer = rxbn;

    uint8_t tbufdata[5];
    readRegisters(rxb->SIDH, tbufdata, sizeof(tbufdata));

    uint32_t id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);
    if(tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) {
        id = (id << 2) + (tbufdata[MCP_SIDL] & 0x03);
        id = (id << 8) + tbufdata[MCP_EID8];
        id = (id << 8) + tbufdata[MCP_EID0];
        packet._extended = true;
    }
    packet._id = id;

    packet._dlc = (tbufdata[MCP_DLC] & DLC_MASK);
    if(packet._dlc > CANPacket::MAX_DATA_LENGTH)
        return MCP2515Error::FAIL;
    
    uint8_t ctrl = readRegister(rxb->CTRL);
    if(rxbn == RXB0) {
        packet._filHit = (ctrl & RXB_0_CTRL_FILHIT);
    } else {
        packet._filHit = (ctrl & RXB_1_CTRL_FILHIT);
    }
    packet._rtr = (ctrl & RXB_CTRL_RTR);

    readRegisters(rxb->DATA, packet._data.data(), packet._dlc);

    modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

    return MCP2515Error::OK;
}

bool MCP2515::checkMessage() {
    return getStatus() & STAT_RXIF_MASK;
}

MCP2515Error MCP2515::readMessage(MCP2515CanPaket &packet) {
    MCP2515Error rc;
    uint8_t stat = getStatus();

    if(stat & STAT_RX0IF)
        rc = readMessage(RXB0, packet);
    else if(stat & STAT_RX1IF)
        rc = readMessage(RXB1, packet);
    else
        rc = MCP2515Error::NOMSG;

    return rc;
}

MCP2515Error MCP2515::sendMessage(TXBn txbn, const CANPacket &packet) {
    const struct TxBnRegs *txbuf = &TXB[txbn];

    auto data = serialize(packet);
    setRegisters(txbuf->SIDH, data.data(), 5 + packet._dlc);
    modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

    return MCP2515Error::OK;
}

MCP2515Error MCP2515::sendMessage(const CANPacket &packet) {
    if (!packet)
        return MCP2515Error::FAILTX;

    MCP2515Error rc;

    uint8_t stat = getStatus();
    if(!(stat & STAT_TXREQ0))
        rc = sendMessage(TXB0, packet);
    else if(!(stat & STAT_TXREQ1))
        rc = sendMessage(TXB1, packet);
    else if(!(stat & STAT_TXREQ2))
        rc = sendMessage(TXB2, packet);
    else
        rc = MCP2515Error::ALLTXBUSY;
    
    return rc;
}

void MCP2515::spiEnable() {
    _spi.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
}

void MCP2515::spiDisable() {
    digitalWrite(_csPin, HIGH);
    _spi.endTransaction();
}

MCP2515Error MCP2515::reset() {
    spiEnable();
    _spi.transfer(INSTRUCTION_RESET);
    spiDisable();

    delay(10);

    // clearing all tx buffers
    uint8_t zeros[14] = {};
    setRegisters(MCP_TXB0CTRL, zeros, sizeof(zeros));
    setRegisters(MCP_TXB1CTRL, zeros, sizeof(zeros));
    setRegisters(MCP_TXB2CTRL, zeros, sizeof(zeros));

    setRegister(MCP_RXB0CTRL, 0x00);
    setRegister(MCP_RXB1CTRL, 0x00);

    setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    modifyRegister(MCP_RXB0CTRL, RXB_CTRL_RXM_MASK, RXB_CTRL_RXM_STDEXT);
    modifyRegister(MCP_RXB1CTRL, RXB_CTRL_RXM_MASK, RXB_CTRL_RXM_STDEXT);

    // clear all filters
    const RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for(uint8_t i = 0; i < sizeof(filters); i++) {
        bool ext = (i == 1);
        auto rc = setFilter(filters[i], ext, 0);
        if(rc)
            return rc;
    }

    // clear all masks
    const MASK masks[] = {MASK0, MASK1};
    for(uint8_t i=0;i<sizeof(masks);i++){
        auto rc = setMask(masks[i], true, 0);
        if(rc)
            return rc;
    }

    return MCP2515Error::OK;
}

uint8_t MCP2515::readRegister(const uint8_t address) {
    spiEnable();
    _spi.transfer(INSTRUCTION_READ);
    _spi.transfer(address);
    uint8_t value = _spi.transfer(0x00);
    spiDisable();

    return value;
}

void MCP2515::readRegisters(const uint8_t address, uint8_t val[], const uint8_t n) {
    spiEnable();
    _spi.transfer(INSTRUCTION_READ);
    _spi.transfer(address);
    // MCP2515 has auto increment of address pointer
    for(uint8_t i = 0; i < n; i++) {
        val[i] = _spi.transfer(0x00);
    }
    spiDisable();
}

void MCP2515::setRegister(const uint8_t address, const uint8_t value) {
    spiEnable();
    _spi.transfer(INSTRUCTION_WRITE);
    _spi.transfer(address);
    _spi.transfer(value);
    spiDisable();
}

void MCP2515::setRegisters(const uint8_t address, const uint8_t val[], const uint8_t n) {
    spiEnable();
    _spi.transfer(INSTRUCTION_WRITE);
    _spi.transfer(address);
    // MCP2515 has auto increment of address pointer
    for(uint8_t i = 0; i < n; i++) {
        _spi.transfer(val[i]);
    }
    spiDisable();
}

void MCP2515::modifyRegister(const uint8_t address, const uint8_t mask, const uint8_t value) {
    spiEnable();
    _spi.transfer(INSTRUCTION_BITMOD);
    _spi.transfer(address);
    _spi.transfer(mask);
    _spi.transfer(value);
    spiDisable();
}

uint8_t MCP2515::getStatus() {
    spiEnable();
    _spi.transfer(INSTRUCTION_READ_STATUS);
    uint8_t ret = _spi.transfer(0x00);
    spiDisable();

    return ret;
}

std::array<uint8_t, 8 + 5> MCP2515::serialize(const CANPacket &packet) {
    std::array<uint8_t, 8+5> dat;

    uint16_t canid = packet._id & 0x0FFFF;
    if(packet._extended) {
        dat[MCP_EID0] = canid & 0xFF;
        dat[MCP_EID8] = canid >> 8;
        canid = packet._id >> 16;
        dat[MCP_SIDL] = canid & 0x03;
        dat[MCP_SIDL] += ((canid & 0x1C) << 3);
        dat[MCP_SIDL] |= TXB_EXIDE_MASK;
        dat[MCP_SIDH] = canid >> 5;
    } else {
        dat[MCP_SIDH] = canid >> 3;
        dat[MCP_SIDL] = ((canid & 0x07) << 5);
        dat[MCP_EID0] = 0x00;
        dat[MCP_EID8] = 0x00;
    }

    dat[MCP_DLC] = packet._dlc;
    dat[MCP_DLC] |= (packet._rtr) ? RTR_MASK : 0x00;

    std::copy(packet._data.begin(), packet._data.begin() + packet._dlc, dat.begin() + MCP_DATA);
    return dat;
}

MCP2515Error MCP2515::setBitrate(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3) {
    auto err = setConfigMode();
    if(err)
        return err;

    setRegister(MCP_CNF1, cnf1);
    setRegister(MCP_CNF2, cnf2);
    setRegister(MCP_CNF3, cnf3);
    return MCP2515Error::OK;
}

MCP2515Error MCP2515::setBitrate(CanSpeed speed) {
    auto err = setConfigMode();
    if(err)
        return err;

    mcp_baud_cfg cfg{};
    switch(_clockFrequency){
        case CanClock::MCP_8MHZ :
            switch(speed) {
                case CAN_1000KBPS:
                    cfg = MCP_8MHZ_1000KBPS;
                    break;
                case CAN_500KBPS:
                    cfg = MCP_8MHZ_500KBPS;
                    break;
                case CAN_250KBPS:
                    cfg = MCP_8MHZ_250KBPS;
                    break;
                case CAN_200KBPS:
                    cfg = MCP_8MHZ_200KBPS;
                    break;
                case CAN_125KBPS:
                    cfg = MCP_8MHZ_125KBPS;
                    break;
                case CAN_100KBPS:
                    cfg = MCP_8MHZ_100KBPS;
                    break;
                case CAN_80KBPS:
                    cfg = MCP_8MHZ_80KBPS;
                    break;
                case CAN_50KBPS:
                    cfg = MCP_8MHZ_50KBPS;
                    break;
                case CAN_40KBPS:
                    cfg = MCP_8MHZ_40KBPS;
                    break;
                case CAN_33KBPS:
                    cfg = MCP_8MHZ_33K3BPS;
                    break;
                case CAN_31K25BPS:
                    cfg = MCP_8MHZ_31K25BPS;
                    break;
                case CAN_20KBPS:
                    cfg = MCP_8MHZ_20KBPS;
                    break;
                case CAN_10KBPS:
                    cfg = MCP_8MHZ_10KBPS;
                    break;
                case CAN_5KBPS:
                    cfg = MCP_8MHZ_5KBPS;
                    break;
                default:
                    break;
            }
            break;
        case CanClock::MCP_12MHZ:
            switch(speed) {
                case CAN_1000KBPS:
                    cfg = MCP_12MHZ_1000KBPS;
                    break;
                case CAN_500KBPS:
                    cfg = MCP_12MHZ_500KBPS;
                    break;
                case CAN_250KBPS:
                    cfg = MCP_12MHZ_250KBPS;
                    break;
                case CAN_200KBPS:
                    cfg = MCP_12MHZ_200KBPS;
                    break;
                case CAN_125KBPS:
                    cfg = MCP_12MHZ_125KBPS;
                    break;
                case CAN_100KBPS:
                    cfg = MCP_12MHZ_100KBPS;
                    break;
                case CAN_80KBPS:
                    cfg = MCP_12MHZ_80KBPS;
                    break;
                case CAN_50KBPS:
                    cfg = MCP_12MHZ_50KBPS;
                    break;
                case CAN_40KBPS:
                    cfg = MCP_12MHZ_40KBPS;
                    break;
                case CAN_33KBPS:
                    cfg = MCP_12MHZ_33K3BPS;
                    break;
                case CAN_20KBPS:
                    cfg = MCP_12MHZ_20KBPS;
                    break;
                case CAN_10KBPS:
                    cfg = MCP_12MHZ_10KBPS;
                    break;
                case CAN_5KBPS:
                    cfg = MCP_12MHZ_5KBPS;
                    break;
                default:
                    break;
            }
            break;
        case CanClock::MCP_16MHZ:
            switch(speed) {
                case CAN_1000KBPS:
                    cfg = MCP_16MHZ_1000KBPS;
                    break;
                case CAN_500KBPS:
                    cfg = MCP_16MHZ_500KBPS;
                    break;
                case CAN_250KBPS:
                    cfg = MCP_16MHZ_250KBPS;
                    break;
                case CAN_200KBPS:
                    cfg = MCP_16MHZ_200KBPS;
                    break;
                case CAN_125KBPS:
                    cfg = MCP_16MHZ_125KBPS;
                    break;
                case CAN_100KBPS:
                    cfg = MCP_16MHZ_100KBPS;
                    break;
                case CAN_80KBPS:
                    cfg = MCP_16MHZ_80KBPS;
                    break;
                case CAN_83K3BPS:
                    cfg = MCP_16MHZ_83K3BPS;
                    break;
                case CAN_50KBPS:
                    cfg = MCP_16MHZ_50KBPS;
                    break;
                case CAN_40KBPS:
                    cfg = MCP_16MHZ_40KBPS;
                    break;
                case CAN_33KBPS:
                    cfg = MCP_16MHZ_33K3BPS;
                    break;
                case CAN_20KBPS:
                    cfg = MCP_16MHZ_20KBPS;
                    break;
                case CAN_10KBPS:
                    cfg = MCP_16MHZ_10KBPS;
                    break;
                case CAN_5KBPS:
                    cfg = MCP_16MHZ_5KBPS;
                    break;
                default:
                    break;
            }
            break;
        case CanClock::MCP_20MHZ:
            switch(speed) {
                case CAN_1000KBPS:
                    cfg = MCP_20MHZ_1000KBPS;
                    break;
                case CAN_500KBPS:
                    cfg = MCP_20MHZ_500KBPS;
                    break;
                case CAN_250KBPS:
                    cfg = MCP_20MHZ_250KBPS;
                    break;
                case CAN_200KBPS:
                    cfg = MCP_20MHZ_200KBPS;
                    break;
                case CAN_125KBPS:
                    cfg = MCP_20MHZ_125KBPS;
                    break;
                case CAN_100KBPS:
                    cfg = MCP_20MHZ_100KBPS;
                    break;
                case CAN_83K3BPS:
                    cfg = MCP_20MHZ_83K3BPS;
                    break;
                case CAN_80KBPS:
                    cfg = MCP_20MHZ_80KBPS;
                    break;
                case CAN_50KBPS:
                    cfg = MCP_20MHZ_50KBPS;
                    break;
                case CAN_40KBPS:
                    cfg = MCP_20MHZ_40KBPS;
                    break;
                case CAN_33KBPS:
                    cfg = MCP_20MHZ_33K3BPS;
                    break;
                default:
                    break;
            }
            break;
    }

    if(cfg) {
        setRegister(MCP_CNF1, cfg.cnf1);
        setRegister(MCP_CNF2, cfg.cnf2);
        setRegister(MCP_CNF3, cfg.cnf3);

        return MCP2515Error::OK;
    } else {
        return MCP2515Error::FAIL;
    }
}
