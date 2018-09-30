#pragma once

#include <ros/ros.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/XL320.h>

#define IORD(base,reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile int32_t*)base)+reg)=data)
#define XL320_write(base,motor,address,data) IOWR(base, (uint32_t)(0x46<<8|motor&0xff), (uint32_t)(address<<16|data&0xffff) )
#define XL320_read(base,motor,address) IOWR(base, (uint32_t)(0x45<<8|motor&0xff), (uint32_t)(address<<16|0) )

class XL320 {
public:
    XL320(int32_t *xl320_base);

    enum struct Address : uint8_t
    {
        /*EEPROM Area*/
        MODEL_NUMBER             = 0, /**< Model number [R] (default=350) */
        VERSION                  = 2, /**< Information on the version of firmware [R] */
        ID                       = 3, /**< ID of Dynamixel [RW] (default=1 ; min=0 ; max=252) */
        BAUD_RATE                = 4, /**< Baud Rate of Dynamixel [RW] (default=3 ; min=0 ; max=3) 0: 9600, 1:57600, 2:115200, 3:1Mbps*/
        RETURN_DELAY_TIME        = 5, /**< Return Delay Time [RW] (default=250 ; min=0 ; max=254) */
        CW_ANGLE_LIMIT           = 6, /**< clockwise Angle Limit [RW] (default=0 ; min=0 ; max=1023) */
        CCW_ANGLE_LIMIT          = 8, /**< counterclockwise Angle Limit [RW] (default=1023 ; min=0 ; max=1023) */
        CONTROL_MODE             = 11, /**< Control Mode [RW] (default=2 ; min=1 ; max=2) */
        LIMIT_TEMPERATURE        = 12, /**< Internal Limit Temperature [RW] (default=65 ; min=0 ; max=150) */
        LOWER_LIMIT_VOLTAGE      = 13, /**< Lowest Limit Voltage [RW] (default=60 ; min=50 ; max=250) */
        UPPPER_LIMIT_VOLTAGE     = 14, /**< Upper Limit Voltage [RW] (default=90 ; min=50 ; max=250) */
        MAX_TORQUE               = 15, /**< Lowest byte of Max. Torque [RW] (default=1023 ; min=0 ; max=1023) */
        RETURN_LEVEL             = 17, /**< Return Level [RW] (default=2 ; min=0 ; max=2) */
        ALARM_SHUTDOWN           = 18, /**< Shutdown for Alarm [RW] (default=3 ; min=0 ; max=7) */
        /*RAM Area*/
        TORQUE_ENABLE            = 24, /**< Torque On/Off [RW] (default=0 ; min=0 ; max=1) */
        LED                      = 25, /**< LED On/Off [RW] (default=0 ; min=0 ; max=7) */
        D_GAIN    				 = 27, /**< D Gain [RW] (default=0 ; min=0 ; max=254) */
        I_GAIN      			 = 28, /**< I Gain [RW] (default=0 ; min=0 ; max=254) */
        P_GAIN    				 = 29, /**< P Gain [RW] (default=32 ; min=0 ; max=254) */
        GOAL_POSITION            = 30, /**< Goal Position [RW] (min=0 ; max=1023) */
        GOAL_SPEED               = 32, /**< Goal Speed [RW] (min=0 ; max=2047) */
        GOAL_TORQUE 		     = 35, /**< Goal Torque [RW] (min=0 ; max=1023) */
        PRESENT_POSITION         = 37, /**< Current Position [R] */
        PRESENT_SPEED            = 39, /**< Current Speed [R] */
        PRESENT_LOAD             = 41, /**< Current Load [R] */
        PRESENT_VOLTAGE          = 45, /**< Current Voltage [R] */
        PRESENT_TEMPERATURE      = 46, /**< Present temperature [R] */
        REGISTERED_INSTRUCTION   = 47, /**< Registered Instruction [R] (default=0) */
        MOVING                   = 49, /**< Moving [R] (default=0) */
        HARDWARE_ERROR           = 50, /**< Hardware error status [R] (default=0) */
        PUNCH                    = 51  /**< Punch [RW] (default=32 ; min=0 ; max=1023) */
    };

    void write(uint8_t motor, Address address, int16_t value);
private:
    void MotorCommandCB(const roboy_communication_middleware::MotorCommandConstPtr &msg);
    bool Service(roboy_communication_middleware::XL320::Request &req,
                       roboy_communication_middleware::XL320::Response &res);
private:
    ros::NodeHandlePtr nh;
    ros::Subscriber motor_command;
    ros::ServiceServer xl320_srv;
    int32_t *xl320_base;
    int id = 5;
};