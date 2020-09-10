/*
    BSD 3-Clause License

    Copyright (c) 2020, Roboy
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

    author: Simon Trendel ( st@gi.ai ), 2020
    description: Class for interfacing fancontrol
*/

#pragma once

#include <ros/ros.h>

#define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)

class FanControl {
public:
    /** Constructor
    * @param base address
    */
    FanControl(int32_t* base);
    /** Gets if auto fan is enabled or not
    * @return enabled/disabled
    */
    bool GetAutoFan();
    /**
    * Gets the average current fed to fancontrol in fpga
    * @return amps
    */
    float GetCurrentAverage();
    /**
    * Gets the duty precentage
    * @return pwm percent
    */
    float GetDuty();
    /**
    * Gets the pwm frequency
    * @return Hz
    */
    int GetPWMFrequency();
    /**
    * Gets the sensitivity towards the average current
    * the smallest value is 1
    * @return value
    */
    int GetSensitivity();
    /** Sets auto fan
    * @param autofan enable/disable
    */
    void SetAutoFan(bool autofan);
    /** Sets the pwm duty percentage
    * @param duty percent
    */
    void SetDuty(int duty);
    /** Sets the pwm frequency
    * @param freq in Hz, should be higher than audible frequency (>16kHz)
    */
    void SetPWMFrequency(int freq);
    /** Sets the sensitivity towards the average current
    * @param sensitivity 
    */
    void SetSensitivity(int sensitivity);
private:
    ros::NodeHandlePtr nh;
    int32_t* base;
};

typedef boost::shared_ptr<FanControl> FanControlPtr;
