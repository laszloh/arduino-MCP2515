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
#include <algorithm>

/// @brief Class representing a CAN packet
class CANPacket {
    friend class MCP2515;

public:
    /// @brief Default constructor
    CANPacket() : _extended(false), _rtr(false) { }
    virtual ~CANPacket() = default;

    // copy operators
    CANPacket(const CANPacket&) = default;
    CANPacket &operator =(const CANPacket&) = default;

    // move operators
    CANPacket(CANPacket&&) = default;
    CANPacket &operator =(CANPacket&&) = default;

    /// @brief Returns true if the packet is valid
    explicit operator bool() const { return isValid(); }

    /// @brief Check if the packet is valid
    /// @return true if the packet is valid, false otherwise
    bool isValid() const { 
        // check id range
        if((!_extended && _id > 0x7FF) || (_extended && _id > 0x1FFFFFFF))
            return false;

        // check dlc range
        if(_dlc > MAX_DATA_LENGTH)
            return false;
        
        // all checks passed
        return true;
    }

    /// @brief Return the ID of the packet
    /// @return The ID of the packet
    uint32_t id() const { return _id; }

    /// @brief Return true if the packet is extended
    /// @return true if the packet is extended, false otherwise
    bool extended() const { return _extended; }

    /// @brief Return the data lenght of the packet
    /// @return The number of bytes in the packet
    uint8_t dlc() const { return _dlc; }

    /// @brief Return true if the packet is a RTR request
    /// @return true if the packet is a RTR request, false otherwise
    bool rtr() const { return _rtr; }

    /// @brief Return the data in the packet
    /// @return The data in the packet
    const std::array<uint8_t, 8> &data() const { return _data; }

    /// @brief Start a new packet with the standard ID
    /// @param id The ID of the packet
    /// @param rtr true if the packet is a RTR request
    /// @return MCP2515Error::OK if the packet is ID is valid
    MCP2515Error startStandard(uint16_t id, bool rtr = false) { return startPacket(id, false, rtr); }

    /// @brief Start a new packet with the extended ID
    /// @param id The ID of the packet
    /// @param rtr true if the packet is a RTR request
    /// @return MCP2515Error::OK if the packet is ID is valid
    MCP2515Error startExtended(uint32_t id, bool rtr = false) { return startPacket(id, true, rtr); }

    /// @brief Start a new packet
    /// @param id The ID of the packet
    /// @param extended True if the packet is extended
    /// @param rtr True if the packet is a RTR request
    /// @return MCP2515Error::OK if the packet is ID is valid
    MCP2515Error startPacket(uint32_t id, bool extended, bool rtr) {
        if( (!extended && id > 0x7FF) || (extended && id > 0x1FFFFFFF) )
            return MCP2515Error::FAIL;
        
        _extended = extended;
        _id = id;
        _rtr = rtr;
        _dlc = 0;

        return MCP2515Error::OK;
    }
    
    /// @brief Write a byte to the packet
    /// @param byte The value to write
    /// @return MCP2515Error::OK if the byte was written
    MCP2515Error writeData(uint8_t byte) { return writeData(&byte, sizeof(byte)); }

    /// @brief Write a buffer to the packet
    /// @param buffer The buffer to write
    /// @param size TThe number of bytes to write
    /// @return MCP2515Error::OK if the buffer was written
    MCP2515Error writeData(const uint8_t* buffer, size_t size) {
        if(_dlc > 8 || size > _data.size() - _dlc)
            return MCP2515Error::FAIL;

        std::copy(buffer, buffer + size, _data.begin() + _dlc);
        _dlc += size;

        return MCP2515Error::OK;
    }

    /// @brief Write a string to the packet
    /// @param str The string to write
    /// @return MCP2515Error::OK if the string was written
    MCP2515Error writeData(const char *str) { return writeData(reinterpret_cast<const uint8_t*>(str), strlen(str)); }

    /// @brief Write an std::array to the packet
    /// @tparam N The size of the array
    /// @param data The array to write
    /// @param size The number of bytes to write
    /// @return MCP2515Error::OK if the array was written
    template<size_t N> MCP2515Error writeData(const std::array<uint8_t, N> &data, uint8_t size = N) { return writeData(data.data(), N); }

    /// @brief Maximum data length in a packet
    static constexpr uint8_t MAX_DATA_LENGTH = 8;

protected:
    bool _extended:1;
    bool _rtr:1;
    uint8_t _dlc{0};
    uint32_t _id{UINT32_MAX};
    std::array<uint8_t, MAX_DATA_LENGTH> _data{0};
};

#endif
