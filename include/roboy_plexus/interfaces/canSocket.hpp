#include <iostream>
#include <string>
#include <stdint.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <roboy_middleware_msgs/CanMotorStatus.h>
#include <roboy_middleware_msgs/CanFrame.h>

#include <ros/ros.h>

class CanSocket{
    public:
        CanSocket();
        int initInterface(std::string interface);
        void canTransmit(const roboy_middleware_msgs::CanFrame::ConstPtr &msg);
        int canRensieve(roboy_middleware_msgs::CanFrame* msg);
    private:
        int s;
};