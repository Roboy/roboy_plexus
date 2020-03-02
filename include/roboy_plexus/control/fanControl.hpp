#pragma once

#include <ros/ros.h>

#define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)

class FanControl {
public:
    FanControl(int32_t* base);

    ~FanControl(){};

    bool GetAutoFan();
    int GetCurrentAverage();
    int GetDuty();
    int GetPWMFrequency();
    int GetSensitivity();
    void SetAutoFan(bool autofan);
    void SetDuty(int duty);
    void SetPWMFrequency(int freq);
    void SetSensitivity(int sensitivity);
private:
    ros::NodeHandlePtr nh;
    int32_t* base;
};

typedef boost::shared_ptr<FanControl> FanControlPtr;
