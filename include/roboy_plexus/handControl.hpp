#pragma once

#include "roboy_plexus/i2c.hpp"
#include <ros/ros.h>

class HandControl{
public:
    HandControl(int32_t *i2c_base, vector<uint8_t> deviceIDs);

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

    bool command(vector<uint8_t> &setPoint);
    bool command(vector<uint8_t> &setPoint, int board);
    bool readSensorData(vector<SensorFrame> &data);
    bool readSensorData(SensorFrame &data, int board);
    bool write(vector<CommandFrame> &command);
    bool write(CommandFrame &command, int board);
    void test();

private:
    I2C *i2c;
    vector<uint8_t> deviceIDs;
};

typedef boost::shared_ptr<HandControl> HandControlPtr;