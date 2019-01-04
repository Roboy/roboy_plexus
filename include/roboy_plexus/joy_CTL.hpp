#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class Joy_CTL
{
  public:
    Joy_CTL(int32_t *throttl_GPIO=nullptr);

    //~Joy_CTL();

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;

    int32_t *throttl_GPIO;
};
