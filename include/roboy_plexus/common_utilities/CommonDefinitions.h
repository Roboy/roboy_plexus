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

    author: Simon Trendel ( simon.trendel@tum.de ), 2018
    description: common defintions shared by all teams
*/

#pragma once

#include <stdint.h>
#include <map>
#include <vector>

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * (float)M_PI / 180.0f)
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0f / (float)M_PI)

#define myoMuscleMeterPerEncoderTick(encoderTicks) ((encoderTicks)/(2096.0*53.0)*(2.0*M_PI*0.0045))
#define myoMuscleEncoderTicksPerMeter(meter) ((meter)*(2096.0*53.0)/(2.0*M_PI*0.0045))
#define myoBrick100NMeterPerEncoderTick(encoderTicks) ((encoderTicks)/(256.0*35.0)*(2.0*M_PI*0.003))
#define myoBrick100NEncoderTicksPerMeter(meter) ((meter)*(256.0*35.0)/(2.0*M_PI*0.003))
#define myoBrick300NMeterPerEncoderTick(encoderTicks) ((encoderTicks)/(1024.0*62.0)*(2.0*M_PI*0.003))
#define myoBrick300NEncoderTicksPerMeter(meter) ((meter)*(1024.0*62.0)/(2.0*M_PI*0.003))

#define springEncoderTicksPerMeter(meter) (10*1000*meter)
#define springMeterPerEncoderTicks(encoderTicks) (encoderTicks/(10*1000))

enum ENCODERS{
    ENCODER0,
    ENCODER1
};

enum CONTROL_MODES{
    ENCODER0_POSITION = 0,
    ENCODER1_POSITION = 1,
    DISPLACEMENT = 2,
    DIRECT_PWM = 3
};

typedef struct {
    int control_mode = 3;
    float PWMLimit = 500; /*!< maximum control output in the positive direction in counts, max 1000000*/
    float Kp = 1;/*!<Gain of the proportional component*/
    float Ki = 0;/*!<Gain of the integral component*/
    float Kd = 0;/*!<Gain of the differential component*/
    float deadband = 0;/*!<Optional deadband threshold for the control response*/
    float IntegralLimit = 50; /*!<Integral maximum*/
} control_Parameters_t;

[[deprecated("will be obsoulted by icebus")]]
typedef struct {
    float control_mode;
    float outputPosMax = 500; /*!< maximum control output in the positive direction in counts, max 4000*/
    float outputNegMax = -500; /*!< maximum control output in the negative direction in counts, max -4000*/
    float spPosMax;/*<!Positive limit for the set point.*/
    float spNegMax;/*<!Negative limit for the set point.*/
    float Kp = 1;/*!<Gain of the proportional component*/
    float Ki = 0;/*!<Gain of the integral component*/
    float Kd = 0;/*!<Gain of the differential component*/
    float forwardGain = 0; /*!<Gain of  the feed-forward term*/
    float deadBand = 0;/*!<Optional deadband threshold for the control response*/
    float IntegralPosMax; /*!<Integral positive component maximum*/
    float IntegralNegMax; /*!<Integral negative component maximum*/
    float radPerEncoderCount = {2 * 3.14159265359f / (2000.0f * 53.0f)};
    float outputDivider = 5; /*! This divides the output of the PID controllers */
} control_Parameters_legacy;

typedef struct {
    uint16_t fw_version;
    uint32_t ID;
    float fcal_0_phase = 0.0f;
    float fcal_1_phase = 0.0f;
    float fcal_0_tilt = 0.0f;
    float fcal_1_tilt = 0.0f;
    uint8_t unlock_count;
    uint8_t hw_version;
    float fcal_0_curve = 0.0f;
    float fcal_1_curve = 0.0f;
    float accel_dir_x;
    float accel_dir_y;
    float accel_dir_z;
    float fcal_0_gibphase = 0.0f;
    float fcal_1_gibphase = 0.0f;
    float fcal_0_gibmag = 0.0f;
    float fcal_1_gibmag = 0.0f;
    uint8_t mode;
    uint8_t faults;
} OOTXframe;
