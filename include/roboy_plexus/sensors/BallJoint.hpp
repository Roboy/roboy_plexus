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

#include <iostream>
#include <bitset>
#include <ros/ros.h>

#define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)

// the upper 8 bit define which register, the lower 8 bit define which sensor
#define BALL_JOINT_READ_mag_x(base, sensor) IORD(base, (uint32_t)(0x00<<2|sensor&0x3) )
#define BALL_JOINT_READ_mag_y(base, sensor) IORD(base, (uint32_t)(0x01<<2|sensor&0x3) )
#define BALL_JOINT_READ_mag_z(base, sensor) IORD(base, (uint32_t)(0x02<<2|sensor&0x3) )
#define BALL_JOINT_READ_temperature(base, sensor) IORD(base, (uint32_t)(0x03<<2|sensor&0x3) )

#define BALL_JOINT_WRITE_update_frequency(base, data) IOWR(base, (uint32_t)(0x00<<2|0x00), data )
#define BALL_JOINT_WRITE_reset(base, data) IOWR(base, (uint32_t)(0x01<<2|0x00), data )

using namespace std;

class BallJoint{
public:
    /**
     * Constructor
     * @param base base (cf hps_0.h )
     */
    BallJoint(int32_t* base, int number_of_sensors=4);

    void readMagneticData(vector<uint8_t> &sensor_id, vector<float> &mx,vector<float> &my,vector<float> &mz);

private:
  float convertToMilliTesla(uint32_t data);

  int32_t *base;
  int number_of_sensors = 4;
};

typedef boost::shared_ptr<BallJoint> BallJointPtr;
