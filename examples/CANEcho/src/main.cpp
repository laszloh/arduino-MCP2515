/**
 * CAN Echo example
 * Copyright 2024 lazloh, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#include "Arduino.h"
#include "MCP2515.h"

MCP2515 MCP = MCP2515(PIN_PD7, MCP2515::MCP_8MHZ);
CANPacket txPacket;

constexpr uint8_t resetPin = PIN_PD6;
constexpr uint8_t stbPin = PIN_PD5;

void setup() {
    Serial.begin(38400);
    while(!Serial) { }

    pinMode(stbPin, OUTPUT);
    digitalWrite(stbPin, LOW);

    Serial.println(F("CAN Echo example"));
    Serial.println(F("Sends back all packets inverted on ID + 1"));

    txPacket.startExtended(0xF6);
    for(uint8_t i=0;i<8;i++)
        txPacket.writeData(0xA0 + i);

    // start the CAN bus
    // for custom baudrate settings from here: https://kvaser.com/support/calculators/bit-timing-calculator/
    //        MCP.begin(cnf1, cnf2, cnf3);
    //  f.e.: MCP.begin(0xC0, 0xAD, 0x02);  // 250kbps@8MHZ (T1=13, T2=3, SP=81,25%, SJW=4)
    if(MCP.begin(MCP2515::CAN_500KBPS)) {
        Serial.println(F("Starting CAN failed!"));
        while(true) { }
    }
    Serial.println("------- CAN Read ----------");
    Serial.println("ID  DLC   DATA");
}

void loop() {
    MCP2515CanPaket rxPacket;

    auto rxErr = MCP.readMessage(rxPacket);
    if(rxErr == MCP2515Error::OK) {
        // packet received
        Serial.print(rxPacket.id(), HEX); // print ID
        Serial.print(' ');
        Serial.print(rxPacket.extended(), HEX); // print DLC
        Serial.print(' ');
        Serial.print(rxPacket.dlc(), HEX); // print DLC
        Serial.print(' ');

        for(int i = 0; i < rxPacket.dlc(); i++) { // print the data
            Serial.printf("%02x ", rxPacket.data()[i]);
        }

        Serial.println();

        CANPacket txPacket;
        txPacket.startPacket(rxPacket.id() + 1, rxPacket.extended(), rxPacket.rtr());
        if(!txPacket.rtr()) {
            const auto &dat = rxPacket.data();
            for(uint8_t i=0;i<rxPacket.dlc();i++)
                txPacket.writeData(dat[i] ^ 0xFF);
        }

        auto txErr = MCP.sendMessage(txPacket);
        if(txErr) {
            Serial.print(F("Tx Error: "));
            Serial.println(txErr.f_str());
        }
    } else if(rxErr != MCP2515Error::NOMSG) {
        Serial.print(F("Rx Error: "));
        Serial.println(rxErr.f_str());
    } else {
        static uint32_t lastHeartbeat;
        static constexpr uint32_t heartbeatInterval = 1000;

        if(millis() - lastHeartbeat > heartbeatInterval) {
            lastHeartbeat = millis();
            Serial.print('.');

            auto errFlags = MCP.getErrorFlags();
            if(errFlags){
                MCP.clearErrorFlags();
                Serial.printf(F("Error flags: 0x%03x (REC: %d, TEC: %d)\n"), errFlags.raw(), errFlags.rxErrorCounter(), errFlags.txErrorCounter());
            }
        }
    }
}
