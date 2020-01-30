#pragma once

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <unistd.h>
#include "hwlib.h"
#include "hps_0.h"
#include <vector>
#include <thread>


using namespace std;

#define TX_REG 0
#define BAUD_REG 1

#ifndef IORD
  #define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
  #define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)
#endif


class UART {
  public:
    UART(void * baseAddr, uint32_t BAUD);
    ~UART();

    void send(uint8_t tx_data);
    void set_baud(uint32_t baud_rate);
    uint8_t receive();

  private:
    void receive_pos(uint8_t * position);

    void * h2p_lw_uart_addr;
    ros::NodeHandlePtr nh;
    ros::Publisher pub_position_left, pub_position_right;
    ros::Subscriber sub_hand_left,sub_hand_right;

    boost::shared_ptr<ros::AsyncSpinner> spinner;
    boost::shared_ptr<std::thread> handStateThread_left, handStateThread_right;

    void RightHandPublisher();
    void LeftHandPublisher();

};
/*

*/
