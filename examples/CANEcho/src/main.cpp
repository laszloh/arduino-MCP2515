/**
 * CAN Echo example
 * Copyright 2024 lazloh, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#include "Arduino.h"
#include "MCP2515.h"

MCP2515 MCP = MCP2515(10, MCP2515::MCP_8MHZ);

void setup() {
    Serial.begin(9600);
    while(!Serial) { }

    Serial.println(F("CAN Echo example"));
    Serial.println(F("Sends back all packets inverted on ID + 1"));

    // start the CAN bus at 250 kbps
    if(!MCP.begin(MCP2515::CAN_250KBPS)) {
        Serial.println(F("Starting CAN failed!"));
        while(true) { }
    }
}

void loop() {
    CANPacket rxPacket;

    auto rxErr = MCP.receivePacket(rxPacket);
    if(rxErr == MCP2515Error::OK) {
        // packet received
        Serial.printf(F("Received packet, id: 0x%08x, extended: %d, dlc: %d"), rxPacket.id(),
                      rxPacket.extended(), rxPacket.dlc());

        CANPacket txPacket;
        txPacket.startPacket(rxPacket.id() + 1, rxPacket.extended(), rxPacket.rtr());
        if(!txPacket.rtr()){
            const auto &rxData = rxPacket.data();
            for(uint8_t i=0;i<rxPacket.dlc();i++)
                txPacket.writeData(rxData[i]^0xFF);
        }

        auto txErr = MCP.writePacket(txPacket);
        if(txErr) {
            Serial.print(F("Tx Error: "));
            Serial.println(txErr.f_str());
        }
    } else if(rxErr != MCP2515Error::NOENT) {
        Serial.print(F("Rx Error: "));
        Serial.println(rxErr.f_str());
    }
}
