/**
 * CAN MCP2515_nb
 * Copyright 2024 laszloh, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */
#pragma once

#include <avr/pgmspace.h>

class __FlashStringHelper;

/// @brief MCP2515 and CANPacket Error codes
class MCP2515Error {
public:
    enum Code{
        OK = 0,     ///< Operation was successful
        FAIL,       ///< Operation failed
        ALLTXBUSY,  ///< All TX buffers are busy
        FAILINIT,   ///< Failed to initialize MCP2515
        FAILTX,     ///< Failed to transmit message
        NOMSG,      ///< No messages available
    };

    /// @brief Default constructor
    MCP2515Error() = default;

    /// @brief Constructor with error code
    /// @param c error code
    MCP2515Error(Code c) : _code(c) {}

    // Compare with MCP2515Error
    friend bool operator==(const MCP2515Error& lhs,
                            const MCP2515Error& rhs) {
        return lhs._code == rhs._code;
    }
    friend bool operator!=(const MCP2515Error& lhs,
                            const MCP2515Error& rhs) {
        return lhs._code != rhs._code;
    }

    // Compare with Code
    friend bool operator==(const MCP2515Error& lhs, Code rhs) {
        return lhs._code == rhs;
    }
    friend bool operator==(Code lhs, const MCP2515Error& rhs) {
        return lhs == rhs._code;
    }
    friend bool operator!=(const MCP2515Error& lhs, Code rhs) {
        return lhs._code != rhs;
    }
    friend bool operator!=(Code lhs, const MCP2515Error& rhs) {
        return lhs != rhs._code;
    }

    /// @brief Returns true if the error code is not OK
    /// @return true if the error code is not OK
    explicit operator bool() const { return _code != Code::OK; }

    /// @brief Returns the error code
    /// @return the error code
    Code code() const { return _code; }

    /// @brief Returns the error code as a string
    /// @return the error code as a string
    const char *c_str() const {
        static constexpr const char *messages[] = {
            "OK", "FAIL", "ALLTXBUSY", "FAILINIT", "FAILTX",
            "NOMSG"
        };
        Code c = _code;
        if(_code >= sizeof(messages) / sizeof(messages[0]))
            c = FAIL;
        return messages[c];
    }

    /// @brief Returns the error code as a flash string
    /// @return the error code as a flash string
    const __FlashStringHelper* f_str() const {
        static const char s0[] PROGMEM = "OK";
        static const char s1[] PROGMEM = "FAIL";
        static const char s2[] PROGMEM = "ALLTXBUSY";
        static const char s3[] PROGMEM = "FAILINIT";
        static const char s4[] PROGMEM = "FAILTX";
        static const char s5[] PROGMEM = "NOMSG";
        static const char* const messages[] PROGMEM = {s0, s1, s2, s3, s4, s5};
 
        Code c = _code;
        if(_code >= sizeof(messages) / sizeof(messages[0]))
            c = FAIL;
        return reinterpret_cast<const __FlashStringHelper*>(pgm_read_word(&(messages[c])));
    }

protected:
    Code _code{Code::OK};
};