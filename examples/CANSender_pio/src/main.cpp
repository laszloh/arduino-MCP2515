/**
 * CAN MCP2515_nb
 * Copyright 2020 WitchCraftWorks Team, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */

#include "Arduino.h"
#include "MCP2515.h"

MCP2515 MCP = MCP2515();

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ;
    }

    Serial.println("CAN Receiver Callback");

    // start the CAN bus at 50 kbps
    if (!MCP.begin(MCP2515::CAN_250KBPS)) {
        Serial.println("Starting CAN failed!");
        while (true);
    }
}

void loop() {
    Serial.println("Sending 11 bit standard packet");

    CANPacket std = CANPacket();

    std.startStandard(0x80);
    std.writeData('a'); // up to 8 data bytes
    std.writeData('b');
    std.writeData('c');
    std.writeData('d');

    MCP2515Error result = MCP.writePacket(std);
    Serial.print("Write standard package error code: ");
    Serial.println(result.f_str());

    Serial.println("Sending 29 bit extended packet");

    CANPacket ext = CANPacket();
    ext.startExtended(0xABCDEF);
    ext.writeData("hell0");

    MCP2515Error result2 = MCP.writePacket(ext);
    Serial.print("Write extended package error code: ");
    Serial.println(result2.f_str());

    delay(1000);
}
