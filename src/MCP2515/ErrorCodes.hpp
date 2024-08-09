/**
 * CAN MCP2515_nb
 * Copyright 2024 laszloh, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */
#pragma once

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
        return lhs.code_ == rhs.code_;
    }
    friend bool operator!=(const MCP2515Error& lhs,
                            const MCP2515Error& rhs) {
        return lhs.code_ != rhs.code_;
    }

    // Compare with Code
    friend bool operator==(const MCP2515Error& lhs, Code rhs) {
        return lhs.code_ == rhs;
    }
    friend bool operator==(Code lhs, const MCP2515Error& rhs) {
        return lhs == rhs.code_;
    }
    friend bool operator!=(const MCP2515Error& lhs, Code rhs) {
        return lhs.code_ != rhs;
    }
    friend bool operator!=(Code lhs, const MCP2515Error& rhs) {
        return lhs != rhs.code_;
    }

    // return true if there is an error
    explicit operator bool() const { return _code != Code::Ok; }

    // return internal enum
    Code code() const { return _code; }

    const char * c_str() const {
        static constexpr const char *messages[] = {
            "OK", "PERM", "NOENT", "INTR", "BADF",
            "AGAIN", "BUSY", "INVAL", "COMM", "OVERFLOW"
        };
        if(_code < sizeof(messages) / sizeof(messages[0]))
            return messages[_code];
        return "";
    }

private:
    Code _code{Code::Ok};
};