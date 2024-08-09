
#include <cstdint>
#include <array>

#include "MCP2515.h"

struct Protocol {
    struct Direction {
        enum BitVal : uint8_t {
            Master = 0,
            Slave = 1
        };

        BitVal sender{BitVal::Master};
        BitVal receiver{BitVal::Master};

    };
    enum Command : uint16_t {
        Hello = 0,
        Config = 1,


        FragmentedHeader = 0x800,
        Command_max,
    };

    Protocol(CANPacket &p) : packet(p) {
        uint32_t id = p.getId();

        command = id & 0x3FF;
        id = (id >> 10) & 0xFFFF;
        fragmented = (id >> 26) & 0x01;
        direction.sender = (id >> 27) & 0x01;
        direction.receiver = (id >> 28) & 0x01;

    }

    uint32_t generateId() const {
        uint32_t ret = command;
        ret |= (id << 10);
        ret |= (fragmented << 26);
        ret |= (direction.receiver) ? (1 << 27) : 0;
        ret |= (direction.sender) ? (1 << 28) : 0;
        return ret;
    }

    explicit operator bool() const {
        return command < Command_max && !(direction.sender == Direction::Master && direction.receiver == Direction::Master);
    }

    CANPacket &packet;
    Direction direction{Direction::Master, Direction::Slave};
    uint16_t id{0x00};
    uint16_t command{0x00};
    bool fragmented{false};

};

void x() {
    Protocol p;

    int x = 10;

    p = Protocol::parseCANPacket(d);

    if(p) {
        // es ist valide
    }

}
