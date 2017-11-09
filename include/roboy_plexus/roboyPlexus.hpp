#pragma once

#include <ros/ros.h>
#include <vector>
#include <roboy_plexus/myoControl.hpp>
#include <roboy_plexus/am4096.hpp>
#include <roboy_communication_middleware/ADCvalue.h>
#include <roboy_communication_middleware/ControlMode.h>
#include <roboy_communication_middleware/DarkRoom.h>
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_communication_middleware/MotorCalibrationService.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/MotorConfigService.h>
#include <std_srvs/SetBool.h>
#include <thread>
#include <map>
#include <chrono>

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
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber motorCommand_sub;
    ros::Publisher motorStatus_pub, jointStatus_pub, darkroom_pub, adc_pub;
    ros::ServiceServer motorConfig_srv, controlMode_srv, emergencyStop_srv, motorCalibration_srv;
    map<int, int> setPoint_backup;
    map<int,map<int,control_Parameters_t>> control_params_backup;
    map<int, int> control_mode, control_mode_backup;
    boost::shared_ptr<MyoControl> myoControl;
    boost::shared_ptr<std::thread> adcThread, darkRoomThread, jointStatusThread, motorStatusThread;
    bool keep_publishing = true;
    int32_t *darkroom_base;
    bool emergency_stop = false;
    vector<boost::shared_ptr<AM4096>> jointAngle;
};