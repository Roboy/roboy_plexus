#include "interfaces/hand.hpp"

UART::UART(void * baseAddr, uint32_t BAUD):h2p_lw_uart_addr(baseAddr){

  set_baud(9600);

  IOWR(h2p_lw_uart_addr, 0, TX_REG);

  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "hand_node");
  }
  nh = ros::NodeHandlePtr(new ros::NodeHandle);

  ROS_INFO("init...hand_node");

  ros::Publisher pub_position_left = nh->advertise<std_msgs::String>("/roboy/hand/left/position", 1);
  //ros::Subscriber sub_hand_left = nh.subscribe("/roboy/hand/left/command", 1, sub_callback);
  ros::Publisher pub_position_right = nh->advertise<std_msgs::String>("/roboy/hand/right/position", 1);
  //ros::Subscriber sub_hand_right = nh.subscribe("/roboy/hand/right/command", 1, sub_callback);

  spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
  spinner->start();

  handStateThread_right = boost::shared_ptr<std::thread>(new std::thread(&UART::RightHandPublisher, this));
  handStateThread_right->detach();
  handStateThread_left = boost::shared_ptr<std::thread>(new std::thread(&UART::LeftHandPublisher, this));
  handStateThread_left->detach();

}

UART::~UART() {
    if (handStateThread_left->joinable())
        handStateThread_left->join();
    if (handStateThread_right->joinable())
        handStateThread_right->join();
}

void UART::set_baud(uint32_t baud_rate){
  IOWR(h2p_lw_uart_addr, baud_rate, BAUD_REG);
}

void UART::send(uint8_t tx_data){

}
uint8_t UART::receive(){
  return (uint8_t)IORD(h2p_lw_uart_addr,0);
}

void UART::receive_pos(uint8_t * position){
  position[0] = (uint8_t)IORD(h2p_lw_uart_addr,1);
  position[1] = (uint8_t)IORD(h2p_lw_uart_addr,2);
  position[2] = (uint8_t)IORD(h2p_lw_uart_addr,3);
  position[3] = (uint8_t)IORD(h2p_lw_uart_addr,4);
  position[4] = (uint8_t)IORD(h2p_lw_uart_addr,5);
}

void UART::RightHandPublisher() {
    ros::Rate rate(200);
    uint8_t position[5];

    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;

        receive_pos(&position[0]);

        ss << "Hello world right" << position[0];

        msg.data = ss.str();

        pub_position_right.publish(msg);
        rate.sleep();
    }
}
void UART::LeftHandPublisher() {
  ros::Rate rate(200);
  while (ros::ok()) {
      std_msgs::String msg;
      std::stringstream ss;

      ss << "Hello world left";

      msg.data = ss.str();
      pub_position_left.publish(msg);
      rate.sleep();
  }
}
