#pragma once

#define NUMBER_OF_MOTORS 8

#define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)

#define MSJ_READ_Kp(base, motor) IORD(base, (uint32_t)(0x00<<8|motor&0xff) )
#define MSJ_READ_Kd(base, motor) IORD(base, (uint32_t)(0x01<<8|motor&0xff) )
#define MSJ_READ_sp(base, motor) IORD(base, (uint32_t)(0x02<<8|motor&0xff) )
#define MSJ_READ_control_mode(base, motor) IORD(base, (uint32_t)(0x03<<8|motor&0xff) )
#define MSJ_READ_outputPosMax(base, motor) IORD(base, (uint32_t)(0x04<<8|motor&0xff) )
#define MSJ_READ_outputNegMax(base, motor) IORD(base, (uint32_t)(0x05<<8|motor&0xff) )
#define MSJ_READ_deadBand(base, motor) IORD(base, (uint32_t)(0x06<<8|motor&0xff) )
#define MSJ_READ_outputDivider(base, motor) IORD(base, (uint32_t)(0x07<<8|motor&0xff) )
#define MSJ_READ_sensor_angle(base, motor) IORD(base, (uint32_t)(0x08<<8|motor&0xff) )
#define MSJ_READ_sensor_angle_absolute(base, motor) IORD(base, (uint32_t)(0x09<<8|motor&0xff) )
#define MSJ_READ_sensor_angle_offset(base, motor) IORD(base, (uint32_t)(0x0A<<8|motor&0xff) )
#define MSJ_READ_sensor_angle_relative(base, motor) IORD(base, (uint32_t)(0x0B<<8|motor&0xff) )
#define MSJ_READ_sensor_angle_velocity(base, motor) IORD(base, (uint32_t)(0x0C<<8|motor&0xff) )
#define MSJ_READ_sensor_revolution_counter(base, motor) IORD(base, (uint32_t)(0x0D<<8|motor&0xff) )
#define MSJ_READ_dutys(base, motor) IORD(base, (uint32_t)(0x0E<<8|motor&0xff) )
#define MSJ_READ_zero_speed(base, motor) IORD(base, (uint32_t)(0x0F<<8|motor&0xff) )

#define MSJ_WRITE_Kp(base, motor, data) IOWR(base, (uint32_t)(0x00<<8|motor&0xff), data )
#define MSJ_WRITE_Kd(base, motor, data) IOWR(base, (uint32_t)(0x01<<8|motor&0xff), data )
#define MSJ_WRITE_sp(base, motor, data) IOWR(base, (uint32_t)(0x02<<8|motor&0xff), data )
#define MSJ_WRITE_control_mode(base, motor, data) IOWR(base, (uint32_t)(0x03<<8|motor&0xff), data )
#define MSJ_WRITE_reset_control(base, data) IOWR(base, (uint32_t)(0x04<<8|0), data )
#define MSJ_WRITE_outputDivider(base, motor, data) IOWR(base, (uint32_t)(0x05<<8|motor&0xff), data )
#define MSJ_WRITE_outputPosMax(base, motor, data) IOWR(base, (uint32_t)(0x06<<8|motor&0xff), data )
#define MSJ_WRITE_outputNegMax(base, motor, data) IOWR(base, (uint32_t)(0x07<<8|motor&0xff), data )
#define MSJ_WRITE_deadBand(base, motor, data) IOWR(base, (uint32_t)(0x08<<8|motor&0xff), data )
#define MSJ_WRITE_zero_speed(base, motor, data) IOWR(base, (uint32_t)(0x09<<8|motor&0xff), data )
#define MSJ_WRITE_pwm_mute(base, data) IOWR(base, (uint32_t)(0x0A<<8|0), data )

#include <ros/ros.h>
#include <roboy_middleware_msgs/MagneticSensor.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <thread>
#include <vector>
#include "roboy_plexus/tlv493d.hpp"

#define SPINDLE_RADIUS 0.0055
#define msjMeterPerEncoderTick(encoderTicks) (((encoderTicks)/4096.0*2.0*M_PI)*(2.0*M_PI*SPINDLE_RADIUS))

using namespace std;

class MSJPlatform{
public:
    MSJPlatform(int32_t *msj_platform_base, int32_t *switch_base, vector<int32_t*> i2c_base);
    void publishStatus();
    void publishMagneticSensors();
    void MotorCommand(const roboy_middleware_msgs::MotorCommandConstPtr &msg);
    bool EmergencyStop(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool Zero(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

private:
    ros::NodeHandlePtr nh;
    ros::Publisher motor_status, magnetic_sensor;
    ros::Subscriber motor_command;
    ros::ServiceServer emergency_stop, zero;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    int32_t *msj_platform_base, *switch_base;
    boost::shared_ptr<std::thread> status_thread, magnetic_thread;
    vector<int32_t> zero_speed = {304,305,312,312,308,305,312,305};
    vector<boost::shared_ptr<TLV493D>> tlv;
    vector<int32_t*> i2c_base;
};