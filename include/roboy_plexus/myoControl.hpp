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
#include <roboy_plexus/timer.hpp>
#include <ros/ros.h>
#include <roboy_plexus/NeoPixel.hpp>

#define MOTORS_PER_MYOCONTROL 9

#define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)

// the upper 8 bit define which register, the lower 8 bit define which motor
#define MYO_READ_Kp(base, motor) IORD(base, (uint32_t)(0x01<<8|motor&0xff) )
#define MYO_READ_Ki(base, motor) IORD(base, (uint32_t)(0x02<<8|motor&0xff) )
#define MYO_READ_Kd(base, motor) IORD(base, (uint32_t)(0x03<<8|motor&0xff) )
#define MYO_READ_encoder0_position(base, motor) IORD(base, (uint32_t)(0x04<<8|motor&0xff) )
#define MYO_READ_encoder1_position(base, motor) IORD(base, (uint32_t)(0x05<<8|motor&0xff) )
#define MYO_READ_encoder0_velocity(base, motor) IORD(base, (uint32_t)(0x06<<8|motor&0xff) )
#define MYO_READ_encoder1_velocity(base, motor) IORD(base, (uint32_t)(0x07<<8|motor&0xff) )
#define MYO_READ_PWMLimit(base, motor) IORD(base, (uint32_t)(0x08<<8|motor&0xff) )
#define MYO_READ_IntegralLimit(base, motor) IORD(base, (uint32_t)(0x09<<8|motor&0xff) )
#define MYO_READ_deadband(base, motor) IORD(base, (uint32_t)(0x0A<<8|motor&0xff) )
#define MYO_READ_control_mode(base, motor) IORD(base, (uint32_t)(0x0B<<8|motor&0xff) )
#define MYO_READ_sp(base, motor) IORD(base, (uint32_t)(0x0C<<8|motor&0xff) )
#define MYO_READ_error_code(base, motor) IORD(base, (uint32_t)(0x0D<<8|motor&0xff) )
#define MYO_READ_update_frequency_Hz(base) IORD(base, (uint32_t)(0x11<<8|0) )
#define MYO_READ_current_phase1(base, motor) IORD(base, (uint32_t)(0x12<<8|motor&0xff) )
#define MYO_READ_current_phase2(base, motor) IORD(base, (uint32_t)(0x13<<8|motor&0xff) )
#define MYO_READ_current_phase3(base, motor) IORD(base, (uint32_t)(0x14<<8|motor&0xff) )
#define MYO_READ_crc_checksum(base, motor) IORD(base, (uint32_t)(0x15<<8|motor&0xff) )
#define MYO_READ_communication_quality(base, motor) IORD(base, (uint32_t)(0x16<<8|motor&0xff) )

#define MYO_WRITE_Kp(base, motor, data) IOWR(base, (uint32_t)(0x01<<8|motor&0xff), data )
#define MYO_WRITE_Ki(base, motor, data) IOWR(base, (uint32_t)(0x02<<8|motor&0xff), data )
#define MYO_WRITE_Kd(base, motor, data) IOWR(base, (uint32_t)(0x03<<8|motor&0xff), data )
#define MYO_WRITE_PWMLimit(base, motor, data) IOWR(base, (uint32_t)(0x08<<8|motor&0xff), data )
#define MYO_WRITE_IntegralLimit(base, motor, data) IOWR(base, (uint32_t)(0x09<<8|motor&0xff), data )
#define MYO_WRITE_deadband(base, motor, data) IOWR(base, (uint32_t)(0x0A<<8|motor&0xff), data )
#define MYO_WRITE_control_mode(base, motor, data) IOWR(base, (uint32_t)(0x0B<<8|motor&0xff), data )
#define MYO_WRITE_sp(base, motor, data) IOWR(base, (uint32_t)(0x0C<<8|motor&0xff), data )
#define MYO_WRITE_update_frequency_Hz(base, data) IOWR(base, (uint32_t)(0x11<<8|0), data )

#define NUMBER_OF_ADC_SAMPLES 50

#define MOTOR_ENCODER 0
#define DISPLACEMENT_ENCODER 1

using namespace std;
using namespace std::chrono;

class MyoControl {
public:
    /**
     * Constructor
     * @param myo_base vector of myo base addresses (cf hps_0.h)
     * @param adc_base adc base address (cf hps_0.h) [OPTIONAL]
     * @param neopixel neopixel base address (cf hps_0.h) [OPTIONAL]
     */
    MyoControl(string motor_config_filepath, vector<int32_t *> &myo_base, int32_t *adc_base = nullptr, NeoPixelPtr neopixel = nullptr);

    ~MyoControl();

    /**
	 * Changes the controller of a motor
	 * @param motor for this motor
	 * @param mode choose from Position, Velocity or Displacement
	 * @param params with these controller parameters
     * @param setPoint new setPoint
	 */
    void changeControl(int motor, int mode, control_Parameters_t &params, int32_t setPoint);

    /**
     * Changes the controller of a motor
     * @param motor for this motor
     * @param mode choose from Position, Velocity or Displacement
     * @param params with these controller parameters
     */
    void changeControl(int motor, int mode, control_Parameters_t &params);

    /**
     * Changes the controller of a motor with the saved controller parameters
     * @param motor for this motor
     * @param mode choose from Position, Velocity or Displacement
     */
    void changeControl(int motor, int mode);

    /**
     * Changes the controller of ALL motors with the saved controller parameters
     * @param mode choose from Position, Velocity or Displacement
     */
    void changeControl(int mode);

    /**
    * Changes the controller parameters of a motor
    * @param motor for this motor
    * @param params with these controller parameters
    */
    void changeControlParameters(int motor, control_Parameters_t &params);

    /**
     * Gets the current control_mode of a motor
     * @param motor for this motor
     */
    uint16_t getControlMode(int motor);

    /**
     * Gets the current position of a motor in encoder ticks
     * @param motor for this motor
     * @param encoder of this encoder
     */
    int32_t getEncoderPosition(int motor,int encoder);

    /**
     * Gets the current velcity of a motor in encoder ticks/s
     * @param motor for this motor
     * @param encoder of this encoder
     */
    int32_t getEncoderVelocity(int motor,int encoder);

    /**
     * Sets a new setpoint for a motor
     * @param motor
     * @param setpoint
     */
    void setPoint(int motor, int32_t setpoint);

    int16_t getCurrent(int motor, int phase);

    /**
     * Fills the given params with default values for the corresponding control mode
     * @param params pointer to control struct
     * @param control_mode Position, Velocity, Force
     */
    void getDefaultControlParams(control_Parameters_t *params, int control_mode);

    /**
     * Changes the control mode for all motors to Position
     * @param pos new setPoint
     */
    void allToPosition(int32_t pos);

    /**
     * Changes the control mode for all motors to Velocity
     * @param pos new setPoint
     */
    void allToVelocity(int32_t vel);

    /**
     * Changes the control mode for all motors to Displacement
     * @param displacement new setPoint
     */
    void allToDisplacement(int32_t displacement);

    /**
     * Changes the control mode for all motors to Direct PWM
     * @param pwm new setPoint
     */
    void allToDirectPWM(int32_t pwm);

    /**
     * Zeros the weight for a load cell
     * @param load_cell for this load cell
     */
    void zeroWeight(int load_cell = 0);

    /**
	 * Returns the current adc of the load cell
	 * @param load_cell for this load cell
     * @return the adc value
	 */
    uint32_t readADC(int load_cell);

    /**
	 * Returns the current weight according to adc_weight_parameters of a load_cell
	 * @param load_cell for this load cell
	 * @return the load
	 */
    float getWeight(int load_cell);

    /**
     * Returns the current weight according to adc_weight_parameters of a load_cell
     * @param load_cell for this load cell
     * @param the adc value
     * @return the load
     */
    float getWeight(int load_cell, uint32_t &adc_value);

    /**
     * records positions of motors in Displacement mode
     * @param samplingTime
     * @param recordTime
     * @param trajectories will be filled with positions
     * @param idList record these motors
     * @param controlmode in this mode
     * @param name filename
     */
    float recordTrajectories(
            float samplingTime, float recordTime,
            map<int, vector<float>> &trajectories, vector<int> &idList,
            vector<int> &controlmode, string name);

    /**
	 * starts recording positions of motors in Displacement mode
	 * @param samplingTime
	 * @param trajectories will be filled with positions
	 * @param idList record these motors
	 * @param name filename
	 */
    float startRecordTrajectories(
            float samplingTime, map<int, vector<float>> &trajectories,
            vector<int> &idList, string name);

    /**
     * stops recording a trajectory and writes to file
     */
    void stopRecordTrajectories();

    /**
     * Plays back a trajectory
     * @param file
     * @return success
     */
    bool playTrajectory(const char *file);

    /**
     * Sets predisplacement for recording trajectories (50 by default)
     * @param value
     */
    void setPredisplacement(int value);

    /**
     * Enables/disables replaying trajectory
     * @param replay
     */
    void setReplay(bool status);

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
    void estimateSpringParameters(int motor, int degree, vector<float> &coeffs, int timeout,
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
    void estimateMotorAngleLinearisationParameters(int motor, int degree, vector<float> &coeffs, int timeout,
                                                   uint numberOfDataPoints, float delta_revolution_negative,
                                                   float delta_revolution_positive, vector<double> &motor_angle,
                                                   vector<double> &motor_encoder);

    /**
     * Performs polynomial regression (http://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/)
     * @param degree (e.g. 2 -> a * x^0 + b * x^1 + c * x^2)
     * @param coeffs the estimated coefficients
     * @param X the x-data
     * @param Y the y-data
     */
    void polynomialRegression(int degree, vector<double> &x, vector<double> &y,
                              vector<float> &coeffs);


    map<int,int> myo_base_of_motor, motor_offset;
    map<int, map<int, control_Parameters_t>> control_params;
    int32_t *adc_base;
    float weight_offset = 0;
    float adc_weight_parameters[2] = {-89.6187, 0.1133}; // b + a*x = y
    uint numberOfMotors;
    const string trajectories_folder = "/home/root/trajectories/";
    const string behaviors_folder = "/home/root/behaviors/";
private:
    Timer timer;
    vector<int32_t *> myo_base;
    int iter = 0;
    bool recording = false; // keeps track of recording status
    bool replay = true;
    int predisplacement = 100;
    NeoPixelPtr neopixel;
};

typedef boost::shared_ptr<MyoControl> MyoControlPtr;