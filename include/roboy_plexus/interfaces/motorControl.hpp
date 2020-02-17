#pragma once

#include <ros/ros.h>
#include <interfaces/NeoPixel.hpp>

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
     * Get the power sense
     * @return true (power on), false (power off)
     */
    virtual bool GetPowerSense(){};

    /**
     * Gets the current pwm of a motor
     * @param motor for this motor
     */
    virtual int32_t GetPWM(int motor){};

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
    virtual void SetPoint(int motor, int32_t setPoint){};

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
};

typedef boost::shared_ptr<MotorControl> MotorControlPtr;
