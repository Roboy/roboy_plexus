/*
    BSD 3-Clause License

    Copyright (c) 2019, Roboy
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

    author: Simon Trendel ( st@gi.ai ), 2019
    description: Class for configuring and reading 3d magnetic sensor TLV493d
*/
#pragma  once

#include <stdint.h>
#include "interfaces/i2c.hpp"
#include <vector>
#include <ros/ros.h>

using namespace std;

#define bitRead(byte,pos) ((byte) & (1<<(pos)))
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')



class TLV493D{
public:
    TLV493D(int32_t *i2c_base);
    ~TLV493D();
    bool initTLV(uint8_t &deviceaddress, int devicepin);
    void reset();
    float convertToMilliTesla(uint8_t MSB, uint8_t LSB) ;
    float convertToMilliTesla(uint32_t data) ;
    void readAllRegisters(int deviceaddress, vector<uint8_t> &reg, bool print=true);
    bool read(float &fx, float &fy, float &fz);
    void updateData();
    bool readData100Hz(float &fx, float &fy, float &fz);
private:
    /// checks the parity of 32 bit val
    /// \param val
    /// \return true if odd, false if even
    bool checkParity(uint32_t val);
    vector<uint8_t> deviceAddress;
    uint8_t gpioreg = 0;
    uint8_t frameCounter = 0;
public:
    boost::shared_ptr<I2C> i2c;
    int32_t *i2c_base;
};