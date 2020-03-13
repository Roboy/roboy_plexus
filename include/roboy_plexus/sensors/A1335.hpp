/*
    BSD 3-Clause License

    Copyright (c) 2017, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Daniel Schubert, Simon Trendel ( simon.trendel@tum.de ), 2018
    description: A1335 - Precision Hall-Effect Angle Sensor IC
*/

#pragma once

#include "interfaces/i2c.hpp"
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

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
    /**
     * Constructor
     * @param i2c_base i2c base (cf hps_0.h )
     * @param deviceIDs active device ids
     */
    A1335(int32_t* i2c_base, vector<uint8_t> &deviceIDs);

    ~A1335();
    /**
     * Reads the angle data
     * @param states will be filled with angle data
     * @return success
     */
    bool readAngleData(vector<A1335State> &states);

    /**
     * Decodes a status flag
     * @param type the type
     * @param code thsi code
     * @return human readable string
     */
    string decodeFlag(int type, uint16_t code);

private:
    bool writeMemoryCheck(uint8_t deviceaddress, uint8_t eeaddress, uint8_t* wdata);

    bool clearStatusRegisters(uint8_t deviceaddress);

    bool checkDefaultSettings(A1335State* state);

    bool readDeviceState(uint8_t deviceaddress, A1335State* state);

    bool searchAddressSpace();

private:
    vector<uint8_t> deviceIDs;
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

typedef std::shared_ptr<A1335> A1335Ptr;
