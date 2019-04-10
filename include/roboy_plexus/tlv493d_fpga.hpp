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
    description: Class for reading 3d magnetic sensor TLV493d, with automatic configuration and readout implemented in fpga
*/
#pragma  once

#define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)

#include <stdint.h>
#include <vector>
#include <ros/ros.h>

using namespace std;

class TLV493D_FPGA{
public:
    TLV493D_FPGA(int32_t *i2c_base);
    ~TLV493D_FPGA();
    float convertToMilliTesla(int32_t data) ;
    bool read(float &fx, float &fy, float &fz);
public:
    int32_t *tlv_base;
};