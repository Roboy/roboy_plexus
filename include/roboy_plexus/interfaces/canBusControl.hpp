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
    description: Class for interfacing icebus
*/

#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <math.h>
#include <chrono>
#include <utility>

#include <fstream>
#include <unistd.h>
#include <tinyxml.h>
#include <string>
#include <sstream>
#include <thread>
#include <tuple>
#include <common_utilities/CommonDefinitions.h>
#include "interfaces/motorControl.hpp"
#include <utility/timer.hpp>
#include "interfaces/canSocket.hpp"
#include <ros/ros.h>

using namespace std;
using namespace std::chrono;

// definition of different control Protocols

#define RMD_X6_STATUS_REQUEST_FRAME {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_MOTOR_START_FRAME {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_MOTOR_OFF_FRAME {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_MOTOR_STOP_FRAME {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_TORQUE_CONTROL_ID 0xA1
#define RMD_X6_SPEED_CONTROL_ID 0xA2
#define RMD_X6_POSITION_CONTROL_2_ID 0xA4
#define RMD_X6_WRITE_KP_POSITION_RAM 0x36
#define RMD_X6_WRITE_KI_POSITION_RAM 0x37
#define RMD_X6_WRITE_KP_SPEED_RAM 0x38
#define RMD_X6_WRITE_KI_SPEED_RAM 0x39
#define RMD_X6_WRITE_KP_TORQUE_RAM 0x3A
#define RMD_X6_WRITE_KI_TORQUE_RAM 0x3B
#define RMD_X6_READ_KP_POSITION_FRAME {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_READ_KI_POSITION_FRAME {0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_READ_KP_SPEED_FRAME {0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_READ_KI_SPEED_FRAME {0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_READ_KP_TORQUE_FRAME {0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_READ_KI_TORQUE_FRAME {0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_WRITE_CURRENT_POSITION_AS_ZERO_ROM_FRAME {0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} //motor needs to be repowerd
#define RMD_X6_GET_OPERATION_MODE_FRAME {0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_OBTAIN_BATTERY_VOLTAGE_FRAME {0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_RESET_SYSTEM_FRAME {0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_BRAKE_OPEN_FRAME {0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_BRAKE_CLOSE_FRAME {0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_SET_CAN_ID 0x79
#define RMD_X6_STATUS_RESPONSE_ID 0x9C
#define RMD_X6_READ_KP_POSITION_ID 0x30
#define RMD_X6_READ_KI_POSITION_ID 0x31
#define RMD_X6_READ_KP_SPEED_ID 0x32
#define RMD_X6_READ_KI_SPEED_ID 0x33
#define RMD_X6_READ_KP_TORQUE_ID 0x34
#define RMD_X6_READ_KI_TORQUE_ID 0x35
#define RMD_X6_POWER_AQUISITION_FRAME {0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_POWER_AQUISITION_ID 0x71
#define RMD_X6_READ_MULTITURN_ANGLE {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_READ_MULTITURN_ANGLE_ID 0x92
#define RMD_X6_READ_PID_VALUE_ID 0x30
#define RMD_X6_READ_PID_VALUE {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define RMD_X6_WRITE_PID_VALUE_ID 0x31
#define SAFTY_TEMP_OFF 79

class CanMotor{
    public:
        CanMotor(int bus_id, int motor_global_id, string bus_socket_name, string muscleType):
        bus_id(bus_id), motor_global_id(motor_global_id), bus_socket_name(bus_socket_name), muscleType(muscleType){};
        //~CanMotor();
        int encoderPosition, torque, speed, temperature, setpoint, bus_id, motor_global_id;
        string bus_socket_name, muscleType;
        CanSocketPtr socket_;
        uint8_t control_mode;
        bool is_on;
};

typedef boost::shared_ptr<CanMotor> CanMotorPtr;

class CanBusControl: public MotorControl {
public:
    /**
     * Constructor
     * @param motor_config as loaded from roboy3.yaml config file, the base addresses
     * of the icebusses
     */
    CanBusControl(MotorConfigPtr motor_config);

    ~CanBusControl();

    /**
    * Sets all motors to a control_mode and applies the setpoint to all
    * @param control_mode to change to
    * @param setPoint the setpoint to apply
    * @return success
    */
    bool AllToSetpoint(int control_mode, int32_t setPoint) override;

    /*
    * hard-coded name of the motorcontrol
    */
    string whoami() override{
        return "canBus";
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
    * Gets the baudrate for a motor
    */
    int32_t GetBaudrate(int motor);

    /**
    * Gets the control parameter of a motor
    * @param motor
    * @param Kp P-gain of controller
    * @param Ki I-gain of controller
    * @param Kd D-gain of controller
    * @param deadband deadband of controller
    * @param IntegralLimit IntegralLimit of controller
    * @param PWMLimit PWMLimit of controller
    */
    void GetControllerParameter(int motor, int32_t &Kp, int32_t &Ki, int32_t &Kd,
            int32_t &deadband, int32_t &IntegralLimit, float &PWMLimit);

    /**
     * Gets the current control_mode of a motor
     * @param motor for this motor
     * @return control_mode
     */
    uint8_t GetControlMode(int motor) override;

    /**
     * Gets the current of a motor in Ampere
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
     * @return the current_limit in Ampere
     */
    float GetCurrentLimit(int motor) override;

    /**
     * Getting default parameters for a control mode
     * @param params
     * @param control_mode
     * @param muscleType
     * @param motor
     */
    void GetDefaultControlParams(control_Parameters_t *params, int control_mode, string muscleType, int motor);

    /**
     * Get the displacement of a muscle
     * @param muscle
     * @return displacement
     */
    int32_t GetDisplacement(int muscle);

    /**
     * Gets the current position of a motor in encoder ticks
     * @param motor for this motor
     * @param encoder of this encoder
     */
    int32_t GetEncoderPosition(int motor,int encoder) override;

    /**
     * Gets the current velocity of a motor in encoder ticks/s (not implemented yet)
     * @param motor for this motor
     * @param encoder of this encoder
     */
    int32_t GetEncoderVelocity(int motor,int encoder) override;

    /**
     * Gets the bus_id of a motor
     * @param motor
     * @return bus_id
     */
    int32_t GetID(int motor);

    /**
     * Gets the muscleType of a motor
     * @param motor
     * @return muscleType
     */
    string GetMuscleType(int motor) override;

    /**
     * Get the neopixel color of a motor
     * @param motor
     * @return color
     */
    int32_t GetNeopixelColor(int motor);

    /**
    * Gets the target update frequency of a motor, ie the frequency at which a bus
    * should talk to a motor.
    */
    int32_t GetMotorUpdateFrequency(int motor);

    /**
    * not implemented, or rather obsoleted by optocouplers on myo_shield_rev0.6
    */
    bool GetPowerSense() override{
        return false;
    }

    /**
     * Get PWM of motor
     * @param pwm in percent
     */
    float GetPWM(int motor) override;

    /**
     * Gets the setpoint of a motor
     * @param motor
     * @return
     */
    float GetSetPoint(int motor);

    /**
     * Returns if this motor is part of this bus
     * @param return true if the motor belongs to this bus
    */
    bool MyMotor(int motor);

    /**
     * Sets the baudrate for a motor
     * @param motor
     * @param baudrate
    */
    void SetBaudrate(int motor, int baudrate);

    /**
     * Changes the controller of ALL motors with the saved controller parameters
     * @param mode choose from ENCODER0_POSITION, ENCODER1_POSITION, DISPLACEMENT, DIRECT_PWM
     * @return success
     */
    bool SetControlMode(int mode) override;

    /**
     * Changes the controller of a motor with the saved controller parameters
     * @param motor for this motor
     * @param mode choose from ENCODER0_POSITION, ENCODER1_POSITION, DISPLACEMENT, DIRECT_PWM
     */
    bool SetControlMode(int motor, int mode) override;

    /**
    * Changes the controller of a motor
    * @param motor for this motor
    * @param mode choose from ENCODER0_POSITION, ENCODER1_POSITION, DISPLACEMENT, DIRECT_PWM
    * @param params with these controller parameters
    */
    bool SetControlMode(int motor, int mode, control_Parameters_t &params) override;

    /**
	 * Changes the controller of a motor
	 * @param motor for this motor
	 * @param mode choose from ENCODER0_POSITION, ENCODER1_POSITION, DISPLACEMENT, DIRECT_PWM
	 * @param params with these controller parameters
     * @param setPoint new setPoint
	 */
    bool SetControlMode(int motor, int mode, control_Parameters_t &params, float setPoint) override;

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
    void SetPoint(int motor, float setpoint) override;

    /**
     * Sets the motor update frequency, this is done per bus,
     * so the motor id is only used to check which bus is controlling it
     * @param motor
     * @param freq
     */
    void SetMotorUpdateFrequency(int motor, int32_t freq) override;

    /**
    * Displacement setpoint used when recording trajectories
    * @param value
    */
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
     * Performs polynomial regression (http://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/)
     * @param degree (e.g. 2 -> a * x^0 + b * x^1 + c * x^2)
     * @param coeffs the estimated coefficients
     * @param X the x-data
     * @param Y the y-data
     */
    void PolynomialRegression(int degree, vector<double> &x, vector<double> &y,
                              vector<float> &coeffs);

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
            float samplingTime, float ,
            map<int, vector<float>> &trajectrecordTimeories, vector<int> &idList,
            vector<int> &controlmode, string name) override;

     /**
     * starts a motor
     * @param motor
     */
    void StartMotor(int motor);

     /**
     * stop a motor keep last motor command
     * @param motor
     */
    void StopMotor(int motor);

    /**
     * turns off a motor
     * @param motor
     */
    void OffMotor(int motor);

    /**
     * Get the Kp for the closed loop torque control
     * @param control_params
     * @param motor
     */
    void GetKpTorqueControl(control_Parameters_t *params, int motor);

    /**
     * Get the Ki for the closed loop torque control
     * @param control_params
     * @param motor
     */
    void GetKiTorqueControl(control_Parameters_t *params, int motor);

    /**
     * Get the Kp for the closed loop speed control
     * @param control_params
     * @param motor
     */
    void GetKpSpeedControl(control_Parameters_t *params, int motor);

    /**
     * Get the Ki for the closed loop speed control
     * @param control_params
     * @param motor
     */
    void GetKiSpeedControl(control_Parameters_t *params, int motor);

    /**
     * Get the Kp for the closed loop position control
     * @param control_params
     * @param motor
     */
    void GetKpPositionControl(control_Parameters_t *params, int motor);

    /**
     * Get the Ki for the closed loop position control
     * @param control_params
     * @param motor
     */
    void GetKiPositionControl(control_Parameters_t *params, int motor);

    /**
     * Set the Kp for the closed loop torque control to Ram
     * @param control_params
     * @param motor
     * @param mode
     */
    void SetKpTorqueControlRam(control_Parameters_t *params, int motor, int mode);

    /**
     * Set the Ki for the closed loop torque control to Ram
     * @param control_params
     * @param motor
     * @param mode
     */
    void SetKiTorqueControlRam(control_Parameters_t *params, int motor, int mode);

    /**
     * Set the Kp for the closed loop speed control to Ram
     * @param control_params
     * @param motor
     * @param mode
     */
    void SetKpSpeedControlRam(control_Parameters_t *params, int motor, int mode);

    /**
     * Set the Ki for the closed loop speed control to Ram
     * @param control_params
     * @param motor
     * @param mode
     */
    void SetKiSpeedControlRam(control_Parameters_t *params, int motor, int mode);

    /**
     * Set the Kp for the closed loop position control to Ram
     * @param control_params
     * @param motor
     * @param mode
     */
    void SetKpPositionControlRam(control_Parameters_t *params, int motor, int mode);

    /**
     * Set the Ki for the closed loop position control to Ram
     * @param control_params
     * @param motor
     * @param mode
     */
    void SetKiPositionControlRam(control_Parameters_t *params, int motor, int mode);

    /**
     * Set the Kp parameter to Ram
     * @param control_params
     * @param motor
     * @param mode
     */
    void SetKpParamRam(control_Parameters_t *params, int motor, int mode);

    /**
     * Set the Ki parameter to Ram
     * @param control_params
     * @param motor
     * @param mode
     */
    void SetKiParamRam(control_Parameters_t *params, int motor, int mode);

    /**
     * init all sockets
     */
    void StartStatusRequestThreads();

     /**
     * Get any control parameters
     * @param control_params
     * @param motor
     * @param mode
     */
    void GetPIDValues(control_Parameters_t *params, int motor, int mode);
     
     /**
     * Get any control parameters
     * @param control_params
     * @param motor
     * @param mode
     */
    void SetPIDParamRam(control_Parameters_t *params, int motor, int mode);

    vector<map<int, CanMotorPtr>> canMotorOnSocket;
    map<int, CanMotorPtr> canMotor;
    vector<tuple<string,int>> can_bus_names;
    vector<CanSocketPtr> canSockets;
    vector<boost::shared_ptr<std::thread>> statusRequestThreads;
    map<int, map<int, control_Parameters_t>> control_params;
    const string trajectories_folder = "/home/root/trajectories/";
    const string behaviors_folder = "/home/root/behaviors/";
private:
    /**
     * init all sockets
     */
    void CreateSockets();

    /**
     * init all sockets
     */
    void StatusRequestLoop();

    /**
     * add to each motor a pointer to corresponding socket
     */
    void SetSocketsForMotor();

    /**
     * Uses a given socket to send data and recieve the response
     * @param can_id
     * @param data
     * @param socket
     */
    void UseSocket(int can_id, uint8_t * data, CanSocketPtr socket);
    
    Timer timer;
    vector<int32_t *> base;
    int iter = 0;
    bool recording = false; // keeps track of recording status
    bool replay = true;
    int predisplacement = 100;
    bool finished_init_threads = false;
};

typedef boost::shared_ptr<CanBusControl> CanBusControlPtr;

