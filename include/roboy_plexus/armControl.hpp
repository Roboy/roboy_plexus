#pragma once

#include <ros/ros.h>
#include <thread>
#include <roboy_middleware_msgs/FingerCommand.h>
#include <roboy_middleware_msgs/ArmStatus.h>
#include <roboy_middleware_msgs/HandCommand.h>
#include <roboy_middleware_msgs/JointController.h>
#include <roboy_control_msgs/SetMode.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include "roboy_plexus/myoControl.hpp"

#define THUMB 0
#define INDEXFINGER 1
#define MIDDLEFINGER 2
#define RINGLITTLEFINGER 3

#define OPEN 0
#define CLOSE 1

class ArmControl {
public:
    ArmControl(int32_t *myo_base);

    ~ArmControl();

    /**
     * Callback for hand command
     * @param msg hand command
     */
    void handCommandCB(const roboy_middleware_msgs::HandCommand::ConstPtr &msg);

    /**
     * Callback for finger command
     * @param msg finger command
     */
    void fingerCommandCB(const roboy_middleware_msgs::FingerCommand::ConstPtr &msg);

private:
    ros::NodeHandlePtr nh;
    ros::Subscriber handCommand_sub, fingerCommand_sub;
    ros::Publisher armStatus_pub;
    bool keep_publishing = true;
    int32_t *myo_base;
    int elbow_agonist = 1, elbow_antagonist = 0;
    vector<uint8_t> handDeviceIDs;
    uint8_t id = 0;
    bool elbow_joint_controller_active = true;
    float elbow_joint_angle = 0;
    string hand;
};

typedef boost::shared_ptr<ArmControl> ArmControlPtr;