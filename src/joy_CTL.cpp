#include <roboy_plexus/joy_CTL.hpp>

Joy_CTL::Joy_CTL(int32_t *throttl_GPIO):
  throttl_GPIO(throttl_GPIO)
{
   //ROS_INFO("Debug: %d", (IORD(throttl_GPIO, 0) & 0x1));
   joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy_CTL::joyCallback, this);

}

void Joy_CTL::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  //ROS_INFO("show it to me baby: %d", joy.);
  ROS_INFO("TODO:");
}
