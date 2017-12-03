#pragma once

#include "roboy_plexus/i2c.hpp"
#include <iostream>
#include <ros/ros.h>

using namespace std;

#define FLAGS_STRLEN 10
#define ANGLES_FLAGS 0
#define STATUS_FLAGS 1
#define ERROR_FLAGS 2
#define XERROR_FLAGS 3

struct A1335State{
    uint8_t address;
    bool isOK;
    float angle; // in deg
    uint8_t angle_flags : 2; // error, new
    uint8_t status_flags : 4;
    uint16_t err_flags : 12;
    uint16_t xerr_flags : 12;
    float temp; // in °C
    float fieldStrength; // in mT

    uint8_t rawData[8][2];
};

class A1335{
public:

    A1335(int32_t* i2c_base, vector<int32_t> &deviceIDs);

    ~A1335();

    bool readAngleData(vector<A1335State> &states);

    string decodeFlag(int type, uint16_t code);

private:
    bool writeMemoryCheck(uint8_t deviceaddress, uint8_t eeaddress, uint8_t* wdata);

    bool clearStatusRegisters(uint8_t deviceaddress);

    bool checkDefaultSettings(A1335State* state);

    bool readDeviceState(uint8_t deviceaddress, A1335State* state);

    bool searchAddressSpace();

private:
    vector<int32_t> deviceIDs;
    const uint8_t start_register = 0x20;
    const uint8_t num_registers = 6;
    const uint8_t start_register2 = 0x34; // Error Mask registers
    const uint8_t num_registers2 = 2;

/**
 * Content of the registers that should be on every chip
 * Starts on ANG register (0x20:21)
 */
    const uint8_t expected_registers[8][2] = {
            {0b00000000, 0b00000000}, // ANG: Excpect RID & EF to be 0
            {0b10000000, 0b00010001}, // STA: RIDC, Error:0, Status Processing
            {0b10100000, 0b00000000}, // ERR: RIDC, all errors 0
            {0b10110000, 0b00000000}, // XERR: RIDC, all errors 0
            {0b11110000, 0b00000000}, // TSEN: RIDC
            {0b11100000, 0b00000000}, // FIELD: RIDC
            {0b11000000, 0b00000000}, // ERM: RIDC & All Errors enabled
            {0b11010000, 0b00000000}  // XERM: RIDC & All Errors enabled
    };

/**
 * 1 marks the bits that belong to the expected values
 * (to check the expected content)
 */
    const uint8_t expected_registers_mask[8][2] = {
            {0b11000000, 0b00000000}, // ANG: RIDC & EF
            {0b11110001, 0b11111111}, // STA: RIDC, Error & Status
            {0b11111100, 0b11111111}, // ERR: All except Interface & CRC Errors
            {0b11111111, 0b11111111}, // XERR: Check all errors
            {0b11110000, 0b00000000}, // TSEN: Only RIDC (Register IDentifier Code)
            {0b11110000, 0b00000000}, // FIELD: RIDC
            {0b11110100, 0b01111111}, // ERM: RIDC & All Errors except Protocol Erros
            {0b11111111, 0b11111111}  // XERM: RIDC & All Errors
    };

    const vector<const char*> angle_flags = {
            "NEW ",
            "ERR "
    };

    const vector<const char*> status_flags = {
            "ERR ",
            "NEW ",
            "Soft_Rst ",
            "PwON_Rst "
    };

    const vector<const char*> error_flags = {
            "MagLow ",
            "MagHigh ",
            "UnderVolt ",
            "OverVolt ",
            "AngleLow ",
            "AngleHigh ",
            "ProcError ",
            "NoRunMode ",
            "(CRC_Err) ",
            "(INTFErr) ",
            "(XOV) ",
            "XERR "
    };

    const vector<const char*> xerror_flags = {
            "SelfTest ",
            "MemAddr ",
            "Execute ",
            "ResetCond ",
            "WTD_Timer ",
            "WTD_Halt ",
            "EEPR_Hard ",
            "SRAM_Hard ",
            "Temp_Err ",
            "AngleWarn ",
            "EEPR_Soft ",
            "SRAM_Soft "
    };

    I2C *i2c;
    uint8_t deviceID;
    string str;
};