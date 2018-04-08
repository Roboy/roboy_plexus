#pragma once

#include "roboy_plexus/i2c.hpp"
#include <ros/ros.h>
#include <thread>
#include <roboy_communication_middleware/FingerCommand.h>
#include <roboy_communication_middleware/HandStatus.h>
#include <roboy_communication_middleware/HandCommand.h>

#define THUMB 0
#define INDEXFINGER 1
#define MIDDLEFINGER 2
#define RINGLITTLEFINGER 3

class HandControl{
public:
    HandControl(int32_t *i2c_base, vector<uint8_t> deviceIDs, bool id = false);

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
     * Callback for finger command
     * @param msg finger command
     */
    void fingerCommandCB(const roboy_communication_middleware::FingerCommand::ConstPtr &msg);

    /**
     * Publishes hand status data
     */
    void handStatusPublisher();

    bool fingerControl(uint8_t finger, uint8_t alpha, uint8_t beta, uint8_t gamma, uint8_t zeta = 90);

    void neutralHand();

    bool command(vector<uint8_t> &setPoint);
    bool command(vector<uint8_t> &setPoint, int board);
    bool readSensorData(vector<SensorFrame> &data);
    bool readSensorData(SensorFrame &data, int board);
    bool write(vector<CommandFrame> &command);
    bool write(CommandFrame &command, int board);
    void test();

private:
    ros::NodeHandlePtr nh;
    ros::Subscriber handCommand_sub, fingerCommand_sub;
    ros::Publisher handStatus_pub;
    boost::shared_ptr<std::thread> handIMUThread;
    bool keep_publishing = true;
    I2C *i2c;
    vector<uint8_t> deviceIDs;
    uint8_t id = 0;
    HandControl::CommandFrame frame[4];
};

typedef boost::shared_ptr<HandControl> HandControlPtr;