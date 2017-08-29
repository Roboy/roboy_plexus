#pragma once

#include <ros/ros.h>
#include <vector>
#include <roboy_plexus/myoControl.hpp>
#include <roboy_communication_middleware/DarkRoom.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/MotorConfigService.h>
#include <thread>

#define NUM_SENSORS 32

using namespace std;

class RoboyPlexus{
public:
    RoboyPlexus(vector<int32_t *> &myo_base, int32_t* darkroom_base);
    ~RoboyPlexus();
private:
    void motorStatusPublisher();
    void motorCommandCB(const roboy_communication_middleware::MotorCommand::ConstPtr &msg);
    bool MotorConfigService(roboy_communication_middleware::MotorConfigService::Request  &req,
                            roboy_communication_middleware::MotorConfigService::Response &res);
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber motorCommand_sub;
    ros::Publisher motorStatus_pub, darkroom_pub;
    ros::ServiceServer motorConfig_srv;
    boost::shared_ptr<MyoControl> myoControl;
    boost::shared_ptr<std::thread> motorStatusThread;
    bool keep_publishing_motor_status = true;
    int32_t *darkroom_base;
};