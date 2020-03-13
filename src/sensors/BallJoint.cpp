#include "sensors/BallJoint.hpp"
#include <vector>
#include <rclcpp/rclcpp.hpp>

BallJoint::BallJoint(int32_t* base):base(base){
  BALL_JOINT_WRITE_reset(base,1);
  BALL_JOINT_WRITE_update_frequency(base,100);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"update frequency %d",BALL_JOINT_READ_update_frequency(base));
}

void BallJoint::readMagneticData(vector<float> &mx,vector<float> &my,vector<float> &mz){
    float fx,fy,fz;
    mx.push_back(fx);
    my.push_back(fy);
    mz.push_back(fz);
}
