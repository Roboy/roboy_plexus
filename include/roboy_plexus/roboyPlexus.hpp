#pragma once

#include <ros/ros.h>
#include <vector>
#include <roboy_plexus/myoControl.hpp>
#include <roboy_plexus/am4096.hpp>
#include <roboy_plexus/Adafruit_LSM9DS1.hpp>
#include <roboy_communication_middleware/ADCvalue.h>
#include <roboy_communication_middleware/ControlMode.h>
#include <roboy_communication_middleware/DarkRoom.h>
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_communication_middleware/MotorCalibrationService.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/MotorConfigService.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Imu.h>
#include <thread>
#include <map>
#include <chrono>
#include <roboy_plexus/ADXL345.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "hwlib.h"

#define NUM_SENSORS 32
#define NUMBER_OF_MOTORS_PER_FPGA 14
#define NUMBER_OF_LOADCELLS 8

using namespace std;
using namespace chrono;

class RoboyPlexus{
public:
    RoboyPlexus(vector<int32_t *> &myo_base, vector<int32_t*> &i2c_base,
                vector<int> &deviceIDs, int32_t* darkroom_base = nullptr,
                uint32_t *adc_base = nullptr);
    ~RoboyPlexus();
private:
    void gsensorPublisher();
    void adcPublisher();
    void darkRoomPublisher();
    void jointStatusPublisher();
    void motorStatusPublisher();
    void motorCommandCB(const roboy_communication_middleware::MotorCommand::ConstPtr &msg);
    bool MotorConfigService(roboy_communication_middleware::MotorConfigService::Request  &req,
                            roboy_communication_middleware::MotorConfigService::Response &res);
    bool ControlModeService(roboy_communication_middleware::ControlMode::Request  &req,
                            roboy_communication_middleware::ControlMode::Response &res);
    bool MotorCalibrationService(roboy_communication_middleware::MotorCalibrationService::Request  &req,
                            roboy_communication_middleware::MotorCalibrationService::Response &res);
    bool EmergencyStopService(std_srvs::SetBool::Request  &req,
                              std_srvs::SetBool::Response &res);
    bool ADXL345_REG_WRITE(int file, uint8_t address, uint8_t value);
    bool ADXL345_REG_READ(int file, uint8_t address,uint8_t *value);
    bool ADXL345_REG_MULTI_READ(int file, uint8_t readaddr,uint8_t readdata[], uint8_t len);
    bool ADXL345_Init(int file);
    bool ADXL345_IsDataReady(int file);
    bool ADXL345_XYZ_Read(int file, uint16_t szData16[3]);
    bool ADXL345_IdRead(int file, uint8_t *pId);
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber motorCommand_sub;
    ros::Publisher motorStatus_pub, jointStatus_pub, darkroom_pub, adc_pub, gsensor_pub;
    ros::ServiceServer motorConfig_srv, controlMode_srv, emergencyStop_srv, motorCalibration_srv;
    map<int, int> setPoint_backup;
    map<int,map<int,control_Parameters_t>> control_params_backup;
    map<int, int> control_mode, control_mode_backup;
    boost::shared_ptr<MyoControl> myoControl;
    boost::shared_ptr<std::thread> adcThread, darkRoomThread, jointStatusThread, motorStatusThread, gsensor_thread;
    bool keep_publishing = true;
    int32_t *darkroom_base;
    bool emergency_stop = false;
    vector<boost::shared_ptr<AM4096>> jointAngle;
    int file;
    const char *filename = "/dev/i2c-0";
    uint8_t id;
    bool bSuccess;
    const int mg_per_digi = 4;
    uint16_t szXYZ[3];
    int cnt=0, max_cnt=0;
};