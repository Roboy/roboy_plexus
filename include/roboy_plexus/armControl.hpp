#pragma once

#include <ros/ros.h>
#include <thread>
#include <roboy_communication_middleware/FingerCommand.h>
#include <roboy_communication_middleware/ArmStatus.h>
#include <roboy_communication_middleware/HandCommand.h>
#include <roboy_communication_middleware/JointController.h>
#include <roboy_communication_control/SetMode.h>
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
                bool id = false, bool hand_control_active = true, bool elbow_joint_controller_active = true,
               bool wrist_joint_controller_active = true );

    ~ArmControl();

    union CommandFrame {
        struct {
            uint8_t angleCommand[5];
            uint8_t gpioControl[6];
        };
        uint8_t data[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    };

    union SensorFrame {
        struct {
            uint16_t current[6];
            float gyro[3];
            float acc[3];
        };
        uint8_t data[36] = {0};
    };

    /**
     * Callback for hand command
     * @param msg hand command
     */
    void handCommandCB(const roboy_communication_middleware::HandCommand::ConstPtr &msg);

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
    void fingerCommandCB(const roboy_communication_middleware::FingerCommand::ConstPtr &msg);

    /**
     * Service to close/open hand
     * @param req
     * @param res
     */
    bool setHandModeService(roboy_communication_control::SetModeRequest &req,
                            roboy_communication_control::SetModeResponse &res);

    /**
      * Service to close/open hand
      * @param req
      * @param res
      */
    bool JointControllerService(roboy_communication_middleware::JointControllerRequest &req,
                                roboy_communication_middleware::JointControllerResponse &res);

    bool ElbowJointControllerService(std_srvs::SetBoolRequest &req,
                                     std_srvs::SetBoolResponse &res);

    bool WristJointControllerService(std_srvs::SetBoolRequest &req,
                                     std_srvs::SetBoolResponse &res);

    /**
     * Publishes hand status data
     */
    void armStatusPublisher();

    bool fingerControl(uint8_t finger, uint8_t alpha, uint8_t beta, uint8_t gamma, uint8_t zeta = 90);

    void neutralHand();

    void closeHand();

    void openHand();

    bool command(vector<uint8_t> &setPoint);

    bool command(vector<uint8_t> &setPoint, int board);

    bool readSensorData(vector<SensorFrame> &data);

    bool readSensorData(SensorFrame &data, int board);

    bool test();

private:
    ros::NodeHandlePtr nh;
    ros::Subscriber handCommand_sub, fingerCommand_sub, elbowCommand_sub, wristCommand_sub;
    ros::Publisher armStatus_pub;
    ros::ServiceServer setMode_srv, jointController_srv, elbow_joint_controller_srv, wrist_joint_controller_srv;
    boost::shared_ptr<std::thread> armStatusThread;
    bool keep_publishing = true;
    int32_t *myo_base;
    int elbow_agonist = 1, elbow_antagonist = 0, wrist_agonist = 3, wrist_antagonist = 2;
    uint8_t elbowDeviceID, wristDeviceID;
    vector<uint8_t> handDeviceIDs;
    uint8_t id = 0;
    ArmControl::CommandFrame frame[4];
    bool hand_control_active = false, elbow_joint_controller_active = true, wrist_joint_controller_active = true;
    float elbow_joint_angle = 0;
    string hand;
};

typedef boost::shared_ptr<ArmControl> ArmControlPtr;