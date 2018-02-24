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
    description: Class for interfacing motor status and PID controllers running in fpga
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
#include <roboy_plexus/timer.hpp>

#define MOTORS_PER_MYOCONTROL 6

#define IORD(base,reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile int32_t*)base)+reg)=data)

// the upper 8 bit define which value, the lower 8 bit define which motor
#define MYO_READ_Kp(base,motor) IORD(base, (uint32_t)(0x00<<8|motor&0xff) )
#define MYO_READ_Ki(base,motor) IORD(base, (uint32_t)(0x01<<8|motor&0xff) )
#define MYO_READ_Kd(base,motor) IORD(base, (uint32_t)(0x02<<8|motor&0xff) )
#define MYO_READ_sp(base,motor) IORD(base, (uint32_t)(0x03<<8|motor&0xff) )
#define MYO_READ_forwardGain(base,motor) IORD(base, (uint32_t)(0x04<<8|motor&0xff) )
#define MYO_READ_outputPosMax(base,motor) IORD(base, (uint32_t)(0x05<<8|motor&0xff) )
#define MYO_READ_outputNegMax(base,motor) IORD(base, (uint32_t)(0x06<<8|motor&0xff) )
#define MYO_READ_IntegralPosMax(base,motor) IORD(base, (uint32_t)(0x07<<8|motor&0xff) )
#define MYO_READ_IntegralNegMax(base,motor) IORD(base, (uint32_t)(0x08<<8|motor&0xff) )
#define MYO_READ_deadBand(base,motor) IORD(base, (uint32_t)(0x09<<8|motor&0xff) )
#define MYO_READ_control(base,motor) IORD(base, (uint32_t)(0x0A<<8|motor&0xff) )
#define MYO_READ_position(base,motor) IORD(base, (uint32_t)(0x0B<<8|motor&0xff) )
#define MYO_READ_velocity(base,motor) IORD(base, (uint32_t)(0x0C<<8|motor&0xff) )
#define MYO_READ_current(base,motor) IORD(base, (uint32_t)(0x0D<<8|motor&0xff) )
#define MYO_READ_displacement(base,motor) IORD(base, (uint32_t)(0x0E<<8|motor&0xff) )
#define MYO_READ_pwmRef(base,motor) IORD(base, (uint32_t)(0x0F<<8|motor&0xff) )
#define MYO_READ_update_frequency(base) IORD(base, (uint32_t)(0x10<<8|0) )

#define MYO_WRITE_Kp(base,motor,data) IOWR(base, (uint32_t)(0x00<<8|motor&0xff), data )
#define MYO_WRITE_Ki(base,motor,data) IOWR(base, (uint32_t)(0x01<<8|motor&0xff), data )
#define MYO_WRITE_Kd(base,motor,data) IOWR(base, (uint32_t)(0x02<<8|motor&0xff), data )
#define MYO_WRITE_sp(base,motor,data) IOWR(base, (uint32_t)(0x03<<8|motor&0xff), data )
#define MYO_WRITE_forwardGain(base,motor,data) IOWR(base, (uint32_t)(0x04<<8|motor&0xff), data )
#define MYO_WRITE_outputPosMax(base,motor,data) IOWR(base, (uint32_t)(0x05<<8|motor&0xff), data )
#define MYO_WRITE_outputNegMax(base,motor,data) IOWR(base, (uint32_t)(0x06<<8|motor&0xff), data )
#define MYO_WRITE_IntegralPosMax(base,motor,data) IOWR(base, (uint32_t)(0x07<<8|motor&0xff), data )
#define MYO_WRITE_IntegralNegMax(base,motor,data) IOWR(base, (uint32_t)(0x08<<8|motor&0xff), data )
#define MYO_WRITE_deadBand(base,motor,data) IOWR(base, (uint32_t)(0x09<<8|motor&0xff), data )
#define MYO_WRITE_control(base,motor,data) IOWR(base, (uint32_t)(0x0A<<8|motor&0xff), data )
#define MYO_WRITE_reset_myo_control(base,data) IOWR(base, (uint32_t)(0x0B<<8|0), data )
#define MYO_WRITE_spi_activated(base,data) IOWR(base, (uint32_t)(0x0C<<8|0), data )
#define MYO_WRITE_reset_controller(base,motor) IOWR(base, (uint32_t)(0x0D<<8|motor&0xff), 1 )
#define MYO_WRITE_update_frequency(base, data) IOWR(base, (uint32_t)(0x0E<<8|0), data )

#define NUMBER_OF_ADC_SAMPLES 10
#define MOTOR_BOARD_COMMUNICATION_FREQUENCY 2500 // in Hz, used to scale the motor velocity

using namespace std;
using namespace std::chrono;

//typedef struct
//{
//    int32_t outputPosMax; /*!< maximum control output in the positive direction in counts, max 4000*/
//    int32_t outputNegMax; /*!< maximum control output in the negative direction in counts, max -4000*/
//    int32_t spPosMax;/*<!Positive limit for the set point.*/
//    int32_t spNegMax;/*<!Negative limit for the set point.*/
//	uint16_t Kp;/*!<Gain of the proportional component*/
//	uint16_t Ki;/*!<Gain of the integral component*/
//	uint16_t Kd;/*!<Gain of the differential component*/
//	int16_t forwardGain; /*!<Gain of  the feed-forward term*/
//	uint16_t deadBand;/*!<Optional deadband threshold for the control response*/
//	int16_t IntegralPosMax; /*!<Integral positive component maximum*/
//	int16_t IntegralNegMax; /*!<Integral negative component maximum*/
//	vector<float> polyPar; /*! polynomial fit from displacement (d)  to tendon force (f)
//				 f=polyPar[0]+polyPar[1]*d +polyPar[2]*d^2+ +polyPar[3]*d^3 + ... */
//	float radPerEncoderCount;
//}control_Parameters_t;
//
//enum CONTROLMODE{
//	POSITION = 0,
//	VELOCITY = 1,
//	DISPLACEMENT = 2,
//	FORCE
//};

class MyoControl{
public:
    /**
     * Constructor
     * @param myo_base vector of myo base addresses (cf hps_0.h)
     */
	MyoControl(vector<int32_t*> &myo_base);
    /**
     * Alternative Constructor
     * @param myo_base vector of myo base addresses (cf hps_0.h)
     * @param adc_base adc base address (cf hps_0.h)
     */
	MyoControl(vector<int32_t*> &myo_base, int32_t *adc_base);
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
	 * Resets all myo controllers
	 */
	void reset();
	/**
	 * Sets the spi state for the interface of a motor
	 * @param motor
	 * @param active
	 * @param active/not active
	 */
	bool setSPIactive(int motor, bool active);
	/**
	 * Get the parameters for the PID controller of a motor
	 * @param motor for this motor
	 */
	void getPIDcontrollerParams(int &Pgain, int &Igain, int &Dgain, int &forwardGain, int &deadband,
									int &setPoint, int &setPointMin, int &setPointMax, int motor);
	/**
	 * Get the parameters for the PID controller of a motor
	 * @param motor for this motor
	 */
	void setPIDcontrollerParams(uint16_t Pgain, uint16_t Igain, uint16_t Dgain, uint16_t forwardGain, uint16_t deadband, int motor, int mode);
	/**
	 * Gets the current control_mode of a motor
	 * @param motor for this motor
	 */
	uint16_t getControlMode(int motor);
	/**
	 * Gets the current pwm of a motor
	 * @param motor for this motor
	 */
	int16_t getPWM(int motor);
	/**
	 * Gets the current position of a motor in radians
	 * @param motor for this motor
	 */
	int32_t getPosition(int motor);
	/**
	 * Gets the current velocity of a motor in radians/seconds
	 * @param motor for this motor
	 */
	int32_t getVelocity(int motor);
	/**
	 * Gets the displacement in encoder ticks
	 * @param motor for this motor
	 */
	int16_t getDisplacement(int motor);
    /**
	 * Sets the position of a motor in radians
	 * @param motor for this motor
	 */
    void setPosition(int motor, int32_t setPoint);
    /**
     * Sets the current velocity of a motor in radians/seconds
     * @param motor for this motor
     */
    void setVelocity(int motor, int16_t setPoint);
    /**
     * Set the displacement in encoder ticks
     * @param motor for this motor
     */
    void setDisplacement(int motor, int16_t setPoint);
	/**
	 * Gets the current in Ampere
	 * @param motor for this motor
	 */
	int16_t getCurrent(int motor);
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
	void allToVelocity(int16_t vel);
	/**
	 * Changes the control mode for all motors to Displacement
	 * @param force new setPoint
	 */
	void allToDisplacement(int16_t displacement);
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
	 * records positions of motors in Displacment mode
	 * @param samplingTime
	 * @param recordTime
	 * @param trajectories will be filled with positions
	 * @param idList record these motors
	 * @param controlmode in this mode
	 * @param name filename
	 */
	float recordTrajectories(
	        float samplingTime, float recordTime,
			map<int,vector<float>> &trajectories, vector<int> &idList,
	        vector<int> &controlmode, string name);

	/**
	 * Plays back a trajectory
	 * @param file
	 * @return success
	 */
	bool playTrajectory(const char* file);
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
	 * Performs polynomial regression (http://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/)
	 * @param degree (e.g. 2 -> a * x^0 + b * x^1 + c * x^2)
	 * @param coeffs the estimated coefficients
	 * @param X the x-data
	 * @param Y the y-data
	 */
	void polynomialRegression(int degree, vector<double> &x, vector<double> &y,
			vector<float> &coeffs);

	map<int,map<int,control_Parameters_t>> control_params;
	int32_t* adc_base;
	float weight_offset = 0;
	float adc_weight_parameters[2] = {-1.3070e+01, 6.8629e-03}; // b + a*x = y
	uint numberOfMotors;
private:
	Timer timer;
	vector<int32_t*> myo_base;
	int iter = 0;
};
