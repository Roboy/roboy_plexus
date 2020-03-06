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
    description: Class for interfacing iceboards connected via icebus
*/

#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <math.h>
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <tinyxml.h>
#include <string>
#include <sstream>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/MotorConfig.hpp>
#include "interfaces/motorControl.hpp"
#include <utility/timer.hpp>
#include <ros/ros.h>
#include <interfaces/NeoPixel.hpp>

#define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)

// the upper 8 bit define which register, the lower 8 bit define which motor
#define ICEBUS_CONTROL_READ_id(base, motor) IORD(base, (uint32_t)(0x00<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_Kp(base, motor) IORD(base, (uint32_t)(0x01<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_Ki(base, motor) IORD(base, (uint32_t)(0x02<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_Kd(base, motor) IORD(base, (uint32_t)(0x03<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_encoder0_position(base, motor) IORD(base, (uint32_t)(0x04<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_encoder1_position(base, motor) IORD(base, (uint32_t)(0x05<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_PWMLimit(base, motor) IORD(base, (uint32_t)(0x08<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_IntegralLimit(base, motor) IORD(base, (uint32_t)(0x09<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_deadband(base, motor) IORD(base, (uint32_t)(0x0A<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_control_mode(base, motor) IORD(base, (uint32_t)(0x0B<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_sp(base, motor) IORD(base, (uint32_t)(0x0C<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_error_code(base, motor) IORD(base, (uint32_t)(0x0D<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_update_frequency_Hz(base) IORD(base, (uint32_t)(0x11<<8|0) )
#define ICEBUS_CONTROL_READ_crc_checksum(base, motor) IORD(base, (uint32_t)(0x15<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_communication_quality(base, motor) IORD(base, (uint32_t)(0x16<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_pwm(base, motor) IORD(base, (uint32_t)(0x17<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_displacement(base, motor) IORD(base, (uint32_t)(0x18<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_current(base, motor) IORD(base, (uint32_t)(0x19<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_neopxl_color(base, motor) IORD(base, (uint32_t)(0x1A<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_current_limit(base, motor) IORD(base, (uint32_t)(0x1B<<8|motor&0xff) )
#define ICEBUS_CONTROL_READ_current_average(base) IORD(base, (uint32_t)(0x1C<<8|0x00) )
#define ICEBUS_CONTROL_READ_baudrate(base) IORD(base, (uint32_t)(0x1D<<8|0x00) )

#define ICEBUS_CONTROL_WRITE_id(base, motor, data) IOWR(base, (uint32_t)(0x00<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_Kp(base, motor, data) IOWR(base, (uint32_t)(0x01<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_Ki(base, motor, data) IOWR(base, (uint32_t)(0x02<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_Kd(base, motor, data) IOWR(base, (uint32_t)(0x03<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_PWMLimit(base, motor, data) IOWR(base, (uint32_t)(0x08<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_IntegralLimit(base, motor, data) IOWR(base, (uint32_t)(0x09<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_deadband(base, motor, data) IOWR(base, (uint32_t)(0x0A<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_control_mode(base, motor, data) IOWR(base, (uint32_t)(0x0B<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_sp(base, motor, data) IOWR(base, (uint32_t)(0x0C<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_update_frequency_Hz(base, data) IOWR(base, (uint32_t)(0x11<<8|0), data )
#define ICEBUS_CONTROL_WRITE_neopxl_color(base, motor, data) IOWR(base, (uint32_t)(0x12<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_current_limit(base, motor, data) IOWR(base, (uint32_t)(0x13<<8|motor&0xff), data )
#define ICEBUS_CONTROL_WRITE_baudrate(base, data) IOWR(base, (uint32_t)(0x14<<8|0), data )

#define NUMBER_OF_ADC_SAMPLES 50
#define NUM_SENSORS 0
#define NUMBER_OF_LOADCELLS 1

using namespace std;
using namespace std::chrono;

class IcebusControl: public MotorControl {
public:
    /**
     * Constructor
     * @param myo_base vector of myo base addresses (cf hps_0.h)
     * @param adc_base adc base address (cf hps_0.h) [OPTIONAL]
     * @param neopixel neopixel base address (cf hps_0.h) [OPTIONAL]
     */
    IcebusControl(string motor_config_filepath, vector<int32_t *> &myo_base, int32_t *adc_base = nullptr, NeoPixelPtr neopixel = nullptr);

    ~IcebusControl();

    bool AllToSetpoint(int control_mode, int32_t setPoint) override;

    string whoami() override{
        return "icebus";
    }

    /**
     * Estimates the spring parameters of a motor by pulling with variable forces
     * keeping track of displacement and weight, it will either timeout or stop when the
     * requested number of samples was reached
     * @param motor for this motor
     * @param degree the degree for the polynomial regression
     * @param coeffs these are the result from the polynomial regression
     * @param timeout in milliseconds
     * @param numberOfDataPoints how many samples do you wanne collect
     * @param displacement_min the minimal displacement to be sampled from
     * @param displacement_min the maximal displacement to be sampled from
     * @param load will be filled with the load cell data
     * @param displacement will be filled with the displacement
     */
    void EstimateSpringParameters(int motor, int degree, vector<float> &coeffs, int timeout,
                                  uint numberOfDataPoints, float displacement_min,
                                  float displacement_max, vector<double> &load, vector<double> &displacement);

    /**
	 * Estimates the linearisation parameters of a myobrick by turning the motor and measuring the positions
     * from the motor angle sensor. The optical encoder data is then used to estimate linearisation parameters
	 * @param motor for this motor
	 * @param degree the degree for the polynomial regression
	 * @param coeffs these are the result from the polynomial regression
	 * @param timeout in milliseconds
	 * @param numberOfDataPoints how many samples do you wanne collect
	 * @param delta_revolution_negative value to turn the motor to into negative direction in degrees
	 * @param delta_revolution_positive value to turn the motor to into negative direction in degrees
	 * @param motor_angle will be filled with the a1339 motor_angle data (values between 0-4095)
	 * @param motor_encoder will be filled with positions from the optical encoder (values between 0-4095)
	 */
    void EstimateMotorAngleLinearisationParameters(int motor, int degree, vector<float> &coeffs, int timeout,
                                                   uint numberOfDataPoints, float delta_revolution_negative,
                                                   float delta_revolution_positive, vector<double> &motor_angle,
                                                   vector<double> &motor_encoder);

   /**
    * Gets the baudrate for a motor
   */
   int32_t GetBaudrate(int motor);

    /**
     * Gets the communication Quality of a motor
     * @param motor
     * @return quality in percent
     */
    int32_t GetCommunicationQuality(int motor);

    /**
     * Gets the error code of a motor
     * @param motor
     * @return error code
     */
    string GetErrorCode(int motor);

    void GetControllerParameter(int motor, int32_t &Kp, int32_t &Ki, int32_t &Kd,
            int32_t &deadband, int32_t &IntegralLimit, int32_t &PWMLimit);

    /**
     * Gets the current control_mode of a motor
     * @param motor for this motor
     */
    uint8_t GetControlMode(int motor) override;

    /**
     * Gets the current of a motor
     * @param motor for this motor
     */
    float GetCurrent(int motor) override;

    /**
     * Gets the average current of all motors connected on the bus
     */
    int GetCurrentAverage();

    /**
     * Gets the current limit of a motor
     * @param motor for this motor
     * return the current_limit in ampere
     */
    float GetCurrentLimit(int motor) override;

    /**
     * Getting default parameters for a control mode
     * @param params
     * @param control_mode
     */
    void GetDefaultControlParams(control_Parameters_t *params, int control_mode);

    /**
     * Get the displacement of a muscle
     * @param muscle
     * @return displacement
     */
    int16_t GetDisplacement(int muscle);

    /**
     * Gets the current position of a motor in encoder ticks
     * @param motor for this motor
     * @param encoder of this encoder
     */
    int32_t GetEncoderPosition(int motor,int encoder) override;

    /**
     * Gets the current position of a motor in encoder ticks
     * @param motor for this motor
     * @param encoder of this encoder
     */
    int32_t GetEncoderVelocity(int motor,int encoder) override;

    /**
     * Gets the bus_id of a motor
     * @param motor
     * @return id
     */
    int32_t GetID(int motor);

    /**
     * Get the neopixel color of a motor
     * @param motor
     * @return color
     */
    int32_t GetNeopixelColor(int motor);

    bool GetPowerSense() override{
        return false;
    }

    /**
     * Get PWM of motor
     * @param pwm
     */
    int32_t GetPWM(int motor) override;

    /**
     * Gets the setpoint of a motor
     * @param motor
     * @return
     */
    int32_t GetSetPoint(int motor);

    /**
     * Returns the current weight according to adc_weight_parameters of a load_cell
     * @param load_cell for this load cell
     * @return the load
     */
    float GetWeight(int load_cell);

    /**
     * Returns the current weight according to adc_weight_parameters of a load_cell
     * @param load_cell for this load cell
     * @param the adc value
     * @return the load
     */
    float GetWeight(int load_cell, uint32_t &adc_value);

    /**
     * Returns if this motor is part of this bus
     * @param return true if the motor belongs to this bus
    */
    bool MyMotor(int motor);

    /**
     * Sets the baudrate for a motor
    */
    void SetBaudrate(int motor, int baudrate);

    /**
     * Changes the controller of ALL motors with the saved controller parameters
     * @param mode choose from Position, Velocity or Displacement
     * @return success
     */
    bool SetControlMode(int mode) override;

    /**
     * Changes the controller of a motor with the saved controller parameters
     * @param motor for this motor
     * @param mode choose from Position, Velocity or Displacement
     */
    bool SetControlMode(int motor, int mode) override;

    /**
    * Changes the controller of a motor
    * @param motor for this motor
    * @param mode choose from Position, Velocity or Displacement
    * @param params with these controller parameters
    */
    bool SetControlMode(int motor, int mode, control_Parameters_t &params);

    /**
	 * Changes the controller of a motor
	 * @param motor for this motor
	 * @param mode choose from Position, Velocity or Displacement
	 * @param params with these controller parameters
     * @param setPoint new setPoint
	 */
    bool SetControlMode(int motor, int mode, control_Parameters_t &params, int32_t setPoint);

    /**
     * Gets the current limit of a motor
     * @param motor for this motor
     * @param limit in ampere
     */
    bool SetCurrentLimit(int motor, float limit) override;

    /**
     * Sets the bus_id of a motor
     * @param motor
     * @param id
     * @return success
     */
    bool SetID(int motor, int id);

    /**
     * Sets the neopixel color of a motor
     * @param motor
     * @param color
     */
    void SetNeopixelColor(int motor, int32_t color) override;

    /**
     * Sets a new setpoint for a motor
     * @param motor
     * @param setpoint
     */
    void SetPoint(int motor, int32_t setpoint) override;

    /**
     * Sets the motor update frequency, this is done per bus,
     * so the motor id is only used to check which bus is controlling it
     * @param motor
     * @param freq
     */
    void SetMotorUpdateFrequency(int motor, int32_t freq);

    void SetPredisplacement(int value) override;

    /**
     * Enables/disables replaying trajectory
     * @param replay
     */
    void SetReplay(bool status) override;

    /**
	 * starts recording positions of motors in Displacement mode
	 * @param samplingTime
	 * @param trajectories will be filled with positions
	 * @param idList record these motors
	 * @param name filename
	 */
    float StartRecordTrajectories(
            float samplingTime, map<int, vector<float>> &trajectories,
            vector<int> &idList, string name) override;

    /**
     * stops recording a trajectory and writes to file
     */
    void StopRecordTrajectories() override;

    /**
     * Plays back a trajectory
     * @param file
     * @return success
     */
    bool PlayTrajectory(const char *file) override;

    /**
     * Prints all information about a motor
     * @param motor_id_global
     */
    void PrintStatus(int motor_id_global);

    /**
     * Performs polynomial regression (http://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/)
     * @param degree (e.g. 2 -> a * x^0 + b * x^1 + c * x^2)
     * @param coeffs the estimated coefficients
     * @param X the x-data
     * @param Y the y-data
     */
    void PolynomialRegression(int degree, vector<double> &x, vector<double> &y,
                              vector<float> &coeffs);

    /**
	 * Returns the current adc of the load cell
	 * @param load_cell for this load cell
     * @return the adc value
	 */
    uint32_t ReadADC(int load_cell);

    /**
     * records positions of motors in Displacement mode
     * @param samplingTime
     * @param recordTime
     * @param trajectories will be filled with positions
     * @param idList record these motors
     * @param controlmode in this mode
     * @param name filename
     */
    float RecordTrajectories(
            float samplingTime, float recordTime,
            map<int, vector<float>> &trajectories, vector<int> &idList,
            vector<int> &controlmode, string name) override;

    /**
     * Zeros the weight for a load cell
     * @param load_cell for this load cell
     */
    void ZeroWeight(int load_cell = 0);

    map<int,int> myo_base_of_motor, motor_offset;
    map<int, map<int, control_Parameters_t>> control_params;
    int32_t *adc_base;
    float weight_offset = 0;
    float adc_weight_parameters[2] = {-89.6187, 0.1133}; // b + a*x = y
    uint numberOfMotors;
    const string trajectories_folder = "/home/root/trajectories/";
    const string behaviors_folder = "/home/root/behaviors/";
    MotorConfigPtr motor_config;
private:
    Timer timer;
    vector<int32_t *> icebus_base, myobus_base;
    int iter = 0;
    bool recording = false; // keeps track of recording status
    bool replay = true;
    int predisplacement = 100;
    NeoPixelPtr neopixel;
};

typedef boost::shared_ptr<IcebusControl> IcebusControlPtr;
