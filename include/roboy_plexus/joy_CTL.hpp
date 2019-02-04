#pragma once

#include <vector>
//#include <iostream>
#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#ifndef IORD
  #define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#endif
#ifndef IOWR
  #define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)
#endif

class rickshaw_CTL
{
  public:
    rickshaw_CTL(int32_t *base_addr = nullptr);

    int32_t readThrottle(void);
    double readAngleSensor(void);
    int32_t readAngleSensor_raw(void);
    void writeThrottle(int32_t throttl_GPIO);

    int32_t *rickshaw_base;

  //private:
    //void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    //ros::NodeHandle nh_;
    //ros::Subscriber joy_sub_;
};
