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
    description: Interface class for motorcontrol, these are the functions any kind
                  of motor control class needs to provide, eg icebus and myocontrol
*/

#pragma once

#include <ros/ros.h>
#include <interfaces/NeoPixel.hpp>
#include <common_utilities/MotorConfig.hpp>

class MotorControl {
public:
    MotorControl();
    ~MotorControl();
    /**
     * This function returns a comprehensive desciption of the bus in use
     * @return
     */
    virtual string whoami(){};

    /**
     * Changes the control mode for ALL motors and applies the ew setpoint
     * @param control_mode change to this control_mode
     * @param setpoint apply this new setPoint
     * #return success
     */
    virtual bool AllToSetpoint(int control_mode, int32_t setpoint){};

    /**
     * Gets the communication Quality of a motor
     * @param motor
     * @return quality in percent
     */
    virtual int32_t GetCommunicationQuality(int motor){};

    /**
      * Gets the controller parameters of a motor
      */
    virtual void GetControllerParameter(int motor, int32_t &Kp, int32_t &Ki, int32_t &Kd,
                                        int32_t &deadband, int32_t &IntegralLimit, float &PWMLimit){};

    /**
     * Gets the current control_mode of a motor
     * @param motor for this motor
     * @return control_mode
     */
    virtual uint8_t GetControlMode(int motor){};

    /**
        * Gets the current in Ampere
        * @param motor for this motor
        */
    virtual float GetCurrent(int motor){};

    /**
        * Gets the current limit in Ampere
        * @param motor for this motor
        */
    virtual float GetCurrentLimit(int motor){};

    /**
     * Gets the current displacement of a motor
     * @param motor for this motor
     */
    virtual int32_t GetDisplacement(int motor){};

    /**
     * Gets the current position of a motor in radians
     * @param motor for this motor
     * @param encoder of this encoder
     */
    virtual int32_t GetEncoderPosition(int motor, int encoder){};

    /**
     * Gets the current velocity of a motor in radians/seconds
     * @param motor for this motor
     * @param encoder of this encoder
     */
    virtual int32_t GetEncoderVelocity(int motor, int encoder){};

    /**
     * Gets the error code of a motor
     * @param motor
     * @return error code
     */
    virtual string GetErrorCode(int motor){};

    /**
     * Get the neopixel color of a motor
     * @param motor
     * @return muscleType
     */
    virtual string GetMuscleType(int motor){};

    /**
     * Get the neopixel color of a motor
     * @param motor
     * @return color
     */
    virtual int32_t GetNeopixelColor(int motor){};

    /**
     * Get the power sense
     * @return true (power on), false (power off)
     */
    virtual bool GetPowerSense(){};

    /**
     * Gets the current pwm of a motor
     * @param motor for this motor
     */
    virtual float GetPWM(int motor){};

    /**
     * Gets the current setpoint of a motor
     * @param motor for this motor
     */
    virtual float GetSetPoint(int motor){};

    /**
     * Getting default parameters for a control mode
     * @param params
     * @param control_mode
     */
    virtual void GetDefaultControlParams(control_Parameters_t *params, int control_mode){};

    /**
     * Sets the motor update frequency, this is done per bus,
     * so the motor id is only used to check which bus is controlling it
     * @param motor
     * @param freq
     */
    virtual void SetMotorUpdateFrequency(int motor, int32_t freq){};

    /**
     * Returns if this motor is part of this bus
     * @param return true if the motor belongs to this bus
    */
    virtual bool MyMotor(int motor){};
    /**
     * records positions of motors in Displacement mode
     * @param samplingTime
     * @param recordTime
     * @param trajectories will be filled with positions
     * @param idList record these motors
     * @param controlmode in this mode
     * @param name filename
     */
    virtual float RecordTrajectories(
            float samplingTime, float recordTime,
            map<int, vector<float>> &trajectories, vector<int> &idList,
            vector<int> &controlmode, string name){};

    /**
     * Changes the controller of a motor with the saved controller parameters
     * @param motor for this motor
     * @param mode choose from Position, Velocity or Displacement
     */
    virtual bool SetControlMode(int motor, int mode){};

    /**
    * Changes the controller of a motor
    * @param motor for this motor
    * @param mode choose from Position, Velocity or Displacement
    * @param params with these controller parameters
    */
    virtual bool SetControlMode(int motor, int mode, control_Parameters_t &params){};

    /**
	 * Changes the controller of a motor
	 * @param motor for this motor
	 * @param mode choose from Position, Velocity or Displacement
	 * @param params with these controller parameters
     * @param setPoint new setPoint
	 */
    virtual bool SetControlMode(int motor, int mode, control_Parameters_t &params, float setPoint){};

    /**
     * Changes the controller of ALL motors with the saved controller parameters
     * @param mode choose from Position, Velocity or Displacement
     */
    virtual bool SetControlMode(int mode){};

    /**
      * Sets the current limit in Ampere
      * @param motor for this motor
      * @param limit in ampere
      */
    virtual bool SetCurrentLimit(int motor, float limit){};

    /**
	 * Sets the setpoint of a motor
	 * @param motor for this motor
	 */
    virtual void SetPoint(int motor, float setPoint){};

    /**
   * Sets the color of a motorboard
   * @param motor for this motor
   * @param color rgb color
   */
    virtual void SetNeopixelColor(int motor, int32_t color){};

    /**
	 * starts recording positions of motors in Displacement mode
	 * @param samplingTime
	 * @param trajectories will be filled with positions
	 * @param idList record these motors
	 * @param name filename
	 */
    virtual float StartRecordTrajectories(
            float samplingTime, map<int, vector<float>> &trajectories,
            vector<int> &idList, string name){};

    /**
     * stops recording a trajectory and writes to file
     */
    virtual void StopRecordTrajectories(){};

    /**
     * Plays back a trajectory
     * @param file
     * @return success
     */
    virtual bool PlayTrajectory(const char *file){};

    /**
     * Sets predisplacement for recording trajectories (50 by default)
     * @param value
     */
    virtual void SetPredisplacement(int value){};

    /**
     * Enables/disables replaying trajectory
     * @param replay
     */
    virtual void SetReplay(bool status){};

    string trajectories_folder = "/home/root/trajectories";
    MotorConfigPtr motor_config;
};

typedef boost::shared_ptr<MotorControl> MotorControlPtr;
