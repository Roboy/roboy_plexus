#pragma once

#include <ros/ros.h>
#include <thread>
#include <roboy_communication_middleware/FingerCommand.h>
#include <roboy_communication_middleware/HandStatus.h>
#include <roboy_communication_middleware/HandCommand.h>
#include <roboy_communication_middleware/JointController.h>
#include <roboy_communication_control/SetMode.h>
#include <std_msgs/Float32.h>
#include "roboy_plexus/myoControl.hpp"

#define THUMB 0
#define INDEXFINGER 1
#define MIDDLEFINGER 2
#define RINGLITTLEFINGER 3

#define OPEN 0
#define CLOSE 1

class HandControl{
public:
    HandControl(int32_t *myo_base, uint8_t elbowDeviceID, vector<uint8_t> handDeviceIDs, bool id = false);

    ~HandControl();

    union CommandFrame{
        struct{
            uint8_t angleCommand[5];
            uint8_t gpioControl[6];
        };
        uint8_t data[11] = {0,0,0,0,0,0,0,0,0,0,0};
    };

    union SensorFrame{
        struct{
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
    /**
     * Publishes hand status data
     */
    void handStatusPublisher();

    bool fingerControl(uint8_t finger, uint8_t alpha, uint8_t beta, uint8_t gamma, uint8_t zeta = 90);

    void neutralHand();
    void closeHand();
    void openHand();

    bool command(vector<uint8_t> &setPoint);
    bool command(vector<uint8_t> &setPoint, int board);
    bool readSensorData(vector<SensorFrame> &data);
    bool readSensorData(SensorFrame &data, int board);
    bool write(vector<CommandFrame> &command);
    bool write(CommandFrame &command, int board);
    void test();

private:
    ros::NodeHandlePtr nh;
    ros::Subscriber handCommand_sub, fingerCommand_sub, elbowCommand_sub;
    ros::Publisher handStatus_pub;
    ros::ServiceServer setMode_srv, jointController_srv;
    boost::shared_ptr<std::thread> handIMUThread;
    bool keep_publishing = true;
    int32_t *myo_base;
    int agonist = 1, antagonist = 0;
    uint8_t elbowDeviceID;
    vector<uint8_t> handDeviceIDs;
    uint8_t id = 0;
    HandControl::CommandFrame frame[4];
    bool hand_control_active = true, elbow_joint_controller_active = false;
    float elbow_joint_angle = 0;
    string hand;
};

typedef boost::shared_ptr<HandControl> HandControlPtr;