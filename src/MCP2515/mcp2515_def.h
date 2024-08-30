/**
 * CAN MCP2515_nb
 * Copyright 2024 laszloh, All Rights Reserved
 *
 * Licensed under Apache 2.0
 */
#pragma once

#include <cstdint>

namespace internal {

enum INSTRUCTION : uint8_t {
  INSTRUCTION_WRITE = 0x02,
  INSTRUCTION_READ = 0x03,
  INSTRUCTION_BITMOD = 0x05,
  INSTRUCTION_LOAD_TX0 = 0x40,
  INSTRUCTION_LOAD_TX1 = 0x42,
  INSTRUCTION_LOAD_TX2 = 0x44,
  INSTRUCTION_RTS_TX0 = 0x81,
  INSTRUCTION_RTS_TX1 = 0x82,
  INSTRUCTION_RTS_TX2 = 0x84,
  INSTRUCTION_RTS_ALL = 0x87,
  INSTRUCTION_READ_RX0 = 0x90,
  INSTRUCTION_READ_RX1 = 0x94,
  INSTRUCTION_READ_STATUS = 0xA0,
  INSTRUCTION_RX_STATUS = 0xB0,
  INSTRUCTION_RESET = 0xC0
};

enum CanctrlReqopMode : uint8_t {
  CANCTRL_REQOP_NORMAL = 0x00,
  CANCTRL_REQOP_SLEEP = 0x20,
  CANCTRL_REQOP_LOOPBACK = 0x40,
  CANCTRL_REQOP_LISTENONLY = 0x60,
  CANCTRL_REQOP_CONFIG = 0x80,
  CANCTRL_REQOP_POWERUP = 0xE0
};
static constexpr uint8_t CANCTRL_REQOP = 0xE0;
static constexpr uint8_t CANCTRL_ABAT = 0x10;
static constexpr uint8_t CANCTRL_OSM = 0x08;
static constexpr uint8_t CANCTRL_CLKEN = 0x04;
static constexpr uint8_t CANCTRL_CLKPRE = 0x03;

enum REGISTER : uint8_t {
  MCP_RXF0SIDH = 0x00,
  MCP_RXF0SIDL = 0x01,
  MCP_RXF0EID8 = 0x02,
  MCP_RXF0EID0 = 0x03,
  MCP_RXF1SIDH = 0x04,
  MCP_RXF1SIDL = 0x05,
  MCP_RXF1EID8 = 0x06,
  MCP_RXF1EID0 = 0x07,
  MCP_RXF2SIDH = 0x08,
  MCP_RXF2SIDL = 0x09,
  MCP_RXF2EID8 = 0x0A,
  MCP_RXF2EID0 = 0x0B,
  MCP_CANSTAT = 0x0E,
  MCP_CANCTRL = 0x0F,
  MCP_RXF3SIDH = 0x10,
  MCP_RXF3SIDL = 0x11,
  MCP_RXF3EID8 = 0x12,
  MCP_RXF3EID0 = 0x13,
  MCP_RXF4SIDH = 0x14,
  MCP_RXF4SIDL = 0x15,
  MCP_RXF4EID8 = 0x16,
  MCP_RXF4EID0 = 0x17,
  MCP_RXF5SIDH = 0x18,
  MCP_RXF5SIDL = 0x19,
  MCP_RXF5EID8 = 0x1A,
  MCP_RXF5EID0 = 0x1B,
  MCP_TEC = 0x1C,
  MCP_REC = 0x1D,
  MCP_RXM0SIDH = 0x20,
  MCP_RXM0SIDL = 0x21,
  MCP_RXM0EID8 = 0x22,
  MCP_RXM0EID0 = 0x23,
  MCP_RXM1SIDH = 0x24,
  MCP_RXM1SIDL = 0x25,
  MCP_RXM1EID8 = 0x26,
  MCP_RXM1EID0 = 0x27,
  MCP_CNF3 = 0x28,
  MCP_CNF2 = 0x29,
  MCP_CNF1 = 0x2A,
  MCP_CANINTE = 0x2B,
  MCP_CANINTF = 0x2C,
  MCP_EFLG = 0x2D,
  MCP_TXB0CTRL = 0x30,
  MCP_TXB0SIDH = 0x31,
  MCP_TXB0SIDL = 0x32,
  MCP_TXB0EID8 = 0x33,
  MCP_TXB0EID0 = 0x34,
  MCP_TXB0DLC = 0x35,
  MCP_TXB0DATA = 0x36,
  MCP_TXB1CTRL = 0x40,
  MCP_TXB1SIDH = 0x41,
  MCP_TXB1SIDL = 0x42,
  MCP_TXB1EID8 = 0x43,
  MCP_TXB1EID0 = 0x44,
  MCP_TXB1DLC = 0x45,
  MCP_TXB1DATA = 0x46,
  MCP_TXB2CTRL = 0x50,
  MCP_TXB2SIDH = 0x51,
  MCP_TXB2SIDL = 0x52,
  MCP_TXB2EID8 = 0x53,
  MCP_TXB2EID0 = 0x54,
  MCP_TXB2DLC = 0x55,
  MCP_TXB2DATA = 0x56,
  MCP_RXB0CTRL = 0x60,
  MCP_RXB0SIDH = 0x61,
  MCP_RXB0SIDL = 0x62,
  MCP_RXB0EID8 = 0x63,
  MCP_RXB0EID0 = 0x64,
  MCP_RXB0DLC = 0x65,
  MCP_RXB0DATA = 0x66,
  MCP_RXB1CTRL = 0x70,
  MCP_RXB1SIDH = 0x71,
  MCP_RXB1SIDL = 0x72,
  MCP_RXB1EID8 = 0x73,
  MCP_RXB1EID0 = 0x74,
  MCP_RXB1DLC = 0x75,
  MCP_RXB1DATA = 0x76,
};

enum CANINTF : uint8_t {
  CANINTF_RX0IF = 0x01,
  CANINTF_RX1IF = 0x02,
  CANINTF_TX0IF = 0x04,
  CANINTF_TX1IF = 0x08,
  CANINTF_TX2IF = 0x10,
  CANINTF_ERRIF = 0x20,
  CANINTF_WAKIF = 0x40,
  CANINTF_MERRF = 0x80,
};

enum TxbNCtrl : uint8_t {
  TXB_ABTF = 0x40,
  TXB_MLOA = 0x20,
  TXB_TXERR = 0x10,
  TXB_TXREQ = 0x08,
  TXB_TXIE = 0x04,
  TXB_TXP = 0x03
};


enum EFLG : uint8_t {
  EFLG_EWARN = 0x01,
  EFLG_RXWAR = 0x02,
  EFLG_TXWAR = 0x04,
  EFLG_RXEP = 0x08,
  EFLG_TXEP = 0x10,
  EFLG_TXBO = 0x20,
  EFLG_RX0OVR = 0x40,
  EFLG_RX1OVR = 0x80,
};

static constexpr uint8_t CANSTAT_OPMOD = 0xE0;
static constexpr uint8_t CANSTAT_ICOD = 0x0E;

static constexpr uint8_t CNF3_SOF = 0x80;
static constexpr uint8_t CNF3_WAKFIL = 0x40;

static constexpr uint8_t TXB_EXIDE_MASK = 0x08;
static constexpr uint8_t DLC_MASK = 0x0F;
static constexpr uint8_t RTR_MASK = 0x40;

static constexpr uint8_t RXB_CTRL_RXM_STD = 0x20;
static constexpr uint8_t RXB_CTRL_RXM_EXT = 0x40;
static constexpr uint8_t RXB_CTRL_RXM_STDEXT = 0x00;
static constexpr uint8_t RXB_CTRL_RXM_MASK = 0x60;
static constexpr uint8_t RXB_CTRL_RTR = 0x08;
static constexpr uint8_t RXB_0_CTRL_BUKT = 0x04;
static constexpr uint8_t RXB_0_CTRL_FILHIT = 0x01;
static constexpr uint8_t RXB_1_CTRL_FILHIT = 0x07;

static constexpr uint8_t MCP_SIDH = 0;
static constexpr uint8_t MCP_SIDL = 1;
static constexpr uint8_t MCP_EID8 = 2;
static constexpr uint8_t MCP_EID0 = 3;
static constexpr uint8_t MCP_DLC = 4;
static constexpr uint8_t MCP_DATA = 5;

enum STAT: uint8_t { 
    STAT_RX0IF  = 0x01, 
    STAT_RX1IF  = 0x02,
    STAT_TXREQ0 = 0x04,
    STAT_TX0IF  = 0x08,
    STAT_TXREQ1 = 0x10,
    STAT_TX1IF  = 0x20,
    STAT_TXREQ2 = 0x40,
    STAT_TX2IF  = 0x80,

    STAT_RXIF_MASK = 0x03,
};

enum RXBn: uint8_t { RXB0 = 0, RXB1 };
enum TXBn: uint8_t { TXB0 = 0, TXB1, TXB2 };

struct mcp_baud_cfg {
    bool set{false};
    uint8_t cnf1{0};
    uint8_t cnf2{0};
    uint8_t cnf3{0};

    constexpr mcp_baud_cfg() = default;
    constexpr mcp_baud_cfg(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3): set(true), cnf1(cnf1), cnf2(cnf2), cnf3(cnf3) { }

    explicit operator bool() const {return set;}
};

/*
 * Speed 8M
 */
static constexpr mcp_baud_cfg MCP_8MHZ_1000KBPS {0x00, 0x80, 0x80};
static constexpr mcp_baud_cfg MCP_8MHZ_500KBPS  {0x00, 0x90, 0x82};
static constexpr mcp_baud_cfg MCP_8MHZ_250KBPS  {0x00, 0xB1, 0x85};
static constexpr mcp_baud_cfg MCP_8MHZ_200KBPS  {0x00, 0xB4, 0x86};
static constexpr mcp_baud_cfg MCP_8MHZ_125KBPS  {0x01, 0xB1, 0x85};
static constexpr mcp_baud_cfg MCP_8MHZ_100KBPS  {0x01, 0xB4, 0x86};
static constexpr mcp_baud_cfg MCP_8MHZ_80KBPS   {0x01, 0xBF, 0x87};
static constexpr mcp_baud_cfg MCP_8MHZ_50KBPS   {0x03, 0xB4, 0x86};
static constexpr mcp_baud_cfg MCP_8MHZ_40KBPS   {0x03, 0xBF, 0x87};
static constexpr mcp_baud_cfg MCP_8MHZ_33K3BPS  {0x47, 0xE2, 0x85};
static constexpr mcp_baud_cfg MCP_8MHZ_31K25BPS {0x07, 0xA4, 0x84};
static constexpr mcp_baud_cfg MCP_8MHZ_20KBPS   {0x07, 0xBF, 0x87};
static constexpr mcp_baud_cfg MCP_8MHZ_10KBPS   {0x0F, 0xBF, 0x87};
static constexpr mcp_baud_cfg MCP_8MHZ_5KBPS    {0x1F, 0xBF, 0x87};

/*
 * Speed 12M
 */
static constexpr mcp_baud_cfg MCP_12MHZ_1000KBPS {0x00, 0x88, 0x81};
static constexpr mcp_baud_cfg MCP_12MHZ_500KBPS  {0x00, 0x9B, 0x82};
static constexpr mcp_baud_cfg MCP_12MHZ_250KBPS  {0x01, 0x9B, 0x82};
static constexpr mcp_baud_cfg MCP_12MHZ_200KBPS  {0x01, 0xA4, 0xB3};
static constexpr mcp_baud_cfg MCP_12MHZ_125KBPS  {0x03, 0x9B, 0x82};
static constexpr mcp_baud_cfg MCP_12MHZ_100KBPS  {0x03, 0xA4, 0x83};
static constexpr mcp_baud_cfg MCP_12MHZ_80KBPS   {0x04, 0xA4, 0x83};
static constexpr mcp_baud_cfg MCP_12MHZ_50KBPS   {0x07, 0xA4, 0x83};
static constexpr mcp_baud_cfg MCP_12MHZ_40KBPS   {0x09, 0xA4, 0x83};
static constexpr mcp_baud_cfg MCP_12MHZ_33K3BPS  {0x0E, 0xB6, 0x84};
static constexpr mcp_baud_cfg MCP_12MHZ_20KBPS   {0x07, 0xBF, 0x84};
static constexpr mcp_baud_cfg MCP_12MHZ_10KBPS   {0x31, 0x9B, 0x82};
static constexpr mcp_baud_cfg MCP_12MHZ_5KBPS    {0x3B, 0xB6, 0x84};

/*
 *  speed 16M
 */
static constexpr mcp_baud_cfg MCP_16MHZ_1000KBPS    {0x00, 0xD0, 0x82};
static constexpr mcp_baud_cfg MCP_16MHZ_500KBPS     {0x00, 0xF0, 0x86};
static constexpr mcp_baud_cfg MCP_16MHZ_250KBPS     {0x41, 0xF1, 0x85};
static constexpr mcp_baud_cfg MCP_16MHZ_200KBPS     {0x01, 0xFA, 0x87};
static constexpr mcp_baud_cfg MCP_16MHZ_125KBPS     {0x03, 0xF0, 0x86};
static constexpr mcp_baud_cfg MCP_16MHZ_100KBPS     {0x03, 0xFA, 0x87};
static constexpr mcp_baud_cfg MCP_16MHZ_80KBPS      {0x03, 0xFF, 0x87};
static constexpr mcp_baud_cfg MCP_16MHZ_83K3BPS     {0x03, 0xBE, 0x07};
static constexpr mcp_baud_cfg MCP_16MHZ_50KBPS      {0x07, 0xFA, 0x87};
static constexpr mcp_baud_cfg MCP_16MHZ_40KBPS      {0x07, 0xFF, 0x87};
static constexpr mcp_baud_cfg MCP_16MHZ_33K3BPS     {0x4E, 0xF1, 0x85};
static constexpr mcp_baud_cfg MCP_16MHZ_20KBPS      {0x0F, 0xFF, 0x87};
static constexpr mcp_baud_cfg MCP_16MHZ_10KBPS      {0x1F, 0xFF, 0x87};
static constexpr mcp_baud_cfg MCP_16MHZ_5KBPS       {0x3F, 0xFF, 0x87};

/*
 *  speed 20M
 */
static constexpr mcp_baud_cfg MCP_20MHZ_1000KBPS    {0x00, 0xD9, 0x82};
static constexpr mcp_baud_cfg MCP_20MHZ_500KBPS     {0x00, 0xFA, 0x87};
static constexpr mcp_baud_cfg MCP_20MHZ_250KBPS     {0x41, 0xFB, 0x86};
static constexpr mcp_baud_cfg MCP_20MHZ_200KBPS     {0x01, 0xFF, 0x87};
static constexpr mcp_baud_cfg MCP_20MHZ_125KBPS     {0x03, 0xFA, 0x87};
static constexpr mcp_baud_cfg MCP_20MHZ_100KBPS     {0x04, 0xFA, 0x87};
static constexpr mcp_baud_cfg MCP_20MHZ_83K3BPS     {0x04, 0xFE, 0x87};
static constexpr mcp_baud_cfg MCP_20MHZ_80KBPS      {0x04, 0xFF, 0x87};
static constexpr mcp_baud_cfg MCP_20MHZ_50KBPS      {0x09, 0xFA, 0x87};
static constexpr mcp_baud_cfg MCP_20MHZ_40KBPS      {0x09, 0xFF, 0x87};
static constexpr mcp_baud_cfg MCP_20MHZ_33K3BPS     {0x0B, 0xFF, 0x87};

} // namespace internal 
