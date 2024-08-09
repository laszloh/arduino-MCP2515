/**
 * CAN MCP2515_nb
 * Copyright 2020 WitchCraftWorks Team, All Rights Reserved
 * Copyright 2024 laszloh, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#ifndef CANPACKET_H
#define CANPACKET_H

#include <Arduino.h>
#include "MCP2515.h"
#include "ErrorCodes.hpp"

#include <array>
#include <cstdint>

class CANPacket {
    friend class MCP2515;

public:
    enum Status : uin16_t {
        STATUS_RX_OK = (1 << 0),
        STATUS_RX_INVALID_MESSAGE = (1 << 1),
        STATUS_TX_PENDING = (1 << 2),
        STATUS_TX_WRITTEN = (1 << 3), // packet written to CAN controller, confirmation pending
        STATUS_TX_SENT = (1 << 4),
        STATUS_TX_ABORT_REQUESTED = (1 << 5),
        STATUS_TX_ABORTED = (1 << 6),
        STATUS_TX_ERROR = (1 << 7),
    };

    CANPacket() : _extended(false), _rtr(false) { }
    ~CANPacket() = default;

    // copy operators
    CANPacket(const CANPacket&) = default;
    CANPacket &operator =(const CANPacket&) = default;

    // move operators
    CANPacket(CANPacket&&) = default;
    CANPacket &operator =(CANPacket&&) = default;

    bool isValid() const { return _lifetime == Lifetime::ended; }
    bool isExtended() const {return _extended; }
    Status getStatus() const {return _status; }

    uint32_t getId() const { return _id; }
    uint8_t getDlc() const { return _dlc; }
    bool getRtr() const { return _rtr; }
    std::array<uint8_t, 8> getData() const { return _data; }

    MCP2515Error startStandard(uint16_t id, bool rtr = false) { return startPacket(id, false, rtr); }
    MCP2515Error startExtended(uint32_t id, bool rtr = false) { return startPacket(id, true, rtr); }

    MCP2515Error writeData(uint8_t byte) { return writeData(&byte, sizeof(byte)); }
    MCP2515Error writeData(const uint8_t* buffer, size_t size) {
        if(_lifetime != Lifetime::started)
            return MCP2515Error::PERM;

        if(_dlc > 8 || size > _data.size() - _dlc)
            return MCP2515Error::OVERFLOW;

        std::copy(buffer, buffer + size, _data.begin() + _dlc);
        dlc += size;

        return MCP2515Error::OK;
    }

    MCP2515Error end() {
        if(_lifetime != Lifetime::started)
            return MCP2515Error::PERM;
        
        _lifetime = Lifetime::ended;
        return MCP2515Error::OK;
    }

private:
    enum class Lifetime: uint8_t {
        undefined = 0,
        started,
        ended,
        aborted
    };

    Lifetime _lifetime{Lifetime::undefined};
    Status _status{0x00};

    bool _extended : 1;
    bool _rtr : 1;
    uint32_t _id{0};
    std::array<uint8_t, 8> _data{0};
    uint8_t _dlc{0};

    MCP2515Error startPacket(uint32_t id, bool extended, bool rtr) {
        if(_lifetime >= Lifetime::started)
            return MCP2515Error::PERM;
        
        if( (!extended && id > 0x7FF) || (extended && id > 0x1FFFFFFF) )
            return MCP2515Error::INVAL;
        
        _lifetime = Lifetime::started;
        _extended = extended;
        _id = id;
        _rtr = rtr;

        return MCP2515Error::OK;
    }
};

#endif
