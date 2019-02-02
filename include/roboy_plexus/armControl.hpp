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
    ArmControl(int32_t *myo_base, uint8_t elbowDeviceID, uint8_t wristDeviceID, vector<uint8_t> handDeviceIDs,
                bool id = false, bool hand_control_active = true, bool elbow_joint_controller_active = true);

    ~ArmControl();

    /**
     * Callback for hand command
     * @param msg hand command
     */
    void handCommandCB(const roboy_middleware_msgs::HandCommand::ConstPtr &msg);

    /**
     * Callback for elbow joint
     * @param msg elbow joint angle command
     */
    void elbowCommandCB(const std_msgs::Float32::ConstPtr &msg);

    /**
     * Callback for wrist joint
     * @param msg wrist joint angle command
     */
    void wristCommandCB(const std_msgs::Float32::ConstPtr &msg);

    /**
     * Callback for finger command
     * @param msg finger command
     */
    void fingerCommandCB(const roboy_middleware_msgs::FingerCommand::ConstPtr &msg);

    /**
     * Service to close/open hand
     * @param req
     * @param res
     */
    bool setHandModeService(roboy_control_msgs::SetModeRequest &req,
                            roboy_control_msgs::SetModeResponse &res);

    /**
      * Service to close/open hand
      * @param req
      * @param res
      */
    bool JointControllerService(roboy_middleware_msgs::JointControllerRequest &req,
                                roboy_middleware_msgs::JointControllerResponse &res);

    bool ElbowJointControllerService(std_srvs::SetBoolRequest &req,
                                     std_srvs::SetBoolResponse &res);

    /**
     * Publishes hand status data
     */
    void armStatusPublisher();


    void closeHand();

    void openHand();

private:
    ros::NodeHandlePtr nh;
    ros::Subscriber handCommand_sub, fingerCommand_sub, elbowCommand_sub;
    ros::Publisher armStatus_pub;
    ros::ServiceServer setMode_srv, jointController_srv, elbow_joint_controller_srv;
    boost::shared_ptr<std::thread> armStatusThread;
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