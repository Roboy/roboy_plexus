#include <roboy_plexus/joy_CTL.hpp>


rickshaw_CTL::rickshaw_CTL(int32_t *base_addr)
{
  rickshaw_base=base_addr;
   //ROS_INFO("Debug: %d", (IORD(throttl_GPIO, 0) & 0x1));
   //joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy_CTL::joyCallback, this);
}

/*void rickshaw_CTL::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  //ROS_INFO("show it to me baby: %d", joy->buttons[0]); //test
  ROS_INFO("TODO:");
}*/


int32_t rickshaw_CTL::readThrottle(void) {
    return (IORD(rickshaw_base, (uint32_t)(0x01<<8|0)));
}
int32_t rickshaw_CTL::readAngleSensor_raw(void) {
    return(IORD(rickshaw_base, (uint32_t)(0x00<<8|0)));
}
int32_t rickshaw_CTL::readAngleSensor_absolute(void) {
    return(IORD(rickshaw_base, (uint32_t)(0x03<<8|0)));
}
int32_t rickshaw_CTL::readAngleSensor_offset(void) {
    return(IORD(rickshaw_base, (uint32_t)(0x04<<8|0)));
}
int32_t rickshaw_CTL::readAngleSensor_relative(void) {
    return(IORD(rickshaw_base, (uint32_t)(0x05<<8|0)));
}
int32_t rickshaw_CTL::readAngleSensor_velocity(void) {
    return(IORD(rickshaw_base, (uint32_t)(0x06<<8|0)));
}
int32_t rickshaw_CTL::readAngleSensor_counter(void) {
    return(IORD(rickshaw_base, (uint32_t)(0x07<<8|0)));
}
int32_t rickshaw_CTL::readSpeedData(void) {
    return(IORD(rickshaw_base, (uint32_t)(0x08<<8|0)));
}


double rickshaw_CTL::readAngleSensor(void) {
    double angle=double(readAngleSensor_raw()) / 4096.0 * 360.0;
    return angle;
}

void rickshaw_CTL::writeThrottle(int32_t throttl_GPIO) {
    IOWR(rickshaw_base, (uint32_t)(0x00<<8|0), throttl_GPIO);
}
