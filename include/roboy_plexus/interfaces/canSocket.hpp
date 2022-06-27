#include <iostream>
#include <string>
#include <stdint.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <tuple>
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

#include <ros/ros.h>
using namespace std;
class CanSocket{
    public:
        CanSocket(std::tuple<string, int> interface):
        name(std::get<0>(interface)), update_frequency(std::get<1>(interface)){
            initInterface(std::get<0>(interface));
            in_use_by = -1;
        };
        
        void canTransmit(int can_id, uint8_t *data);
        int canRensieve(uint8_t *data);
        string name;
        int update_frequency;
        int in_use_by;
        ~CanSocket();
    private:
        int s;
        int initInterface(string interface);
    
};

typedef boost::shared_ptr<CanSocket> CanSocketPtr;