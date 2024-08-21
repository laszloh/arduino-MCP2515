/**
 * CAN MCP2515_nb
 * Copyright 2024 laszloh, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */
#pragma once

#include <avr/pgmspace.h>

class __FlashStringHelper;

class MCP2515Error {
public:
    enum Code{
        OK = 0,
        PERM,
        NOENT,
        INTR,
        BADF,
        AGAIN,
        BUSY,
        INVAL,
        COMM,
        OVERFLOW
    };

    MCP2515Error() = default;
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

    // return true if there is an error
    explicit operator bool() const { return _code != Code::OK; }

    // return internal enum
    Code code() const { return _code; }

    const char * c_str() const {
        static constexpr const char *messages[] = {
            "OK", "PERM", "NOENT", "INTR", "BADF",
            "AGAIN", "BUSY", "INVAL", "COMM", "OVERFLOW"
        };
        Code c = _code;
        if(_code >= sizeof(messages) / sizeof(messages[0]))
            c = INVAL;
        return messages[c];
    }

    const __FlashStringHelper* f_str() const {
        static const char s0[] PROGMEM = "OK";
        static const char s1[] PROGMEM = "PERM";
        static const char s2[] PROGMEM = "NOENT";
        static const char s3[] PROGMEM = "INTR";
        static const char s4[] PROGMEM = "BADF";
        static const char s5[] PROGMEM = "AGAIN";
        static const char s6[] PROGMEM = "BUSY";
        static const char s7[] PROGMEM = "INVAL";
        static const char s8[] PROGMEM = "COMM";
        static const char s9[] PROGMEM = "OVERFLOW";
        static const char* const messages[] PROGMEM = {s0, s1, s2, s3, s4, s5, s6, s7, s8, s9};
 
        Code c = _code;
        if(_code >= sizeof(messages) / sizeof(messages[0]))
            c = INVAL;
        return reinterpret_cast<const __FlashStringHelper*>(pgm_read_word(&(messages[c])));
    }

private:
    Code _code{Code::OK};
};