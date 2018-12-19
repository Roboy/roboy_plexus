#include <roboyPlexus/joy_CTL.hpp>

Joy_CTL::Joy_CTL(int32_t *throttl_GPIO):
  linear_(1),
  angular_(2),
  throttl_GPIO(throttl_GPIO)
{

   ROS_INFO("Debug: %d", (IORD(throttl_GPIO, 0) & 0x1));

   nh_.param("axis_linear", linear_, linear_);
   nh_.param("axis_angular", angular_, angular_);
   nh_.param("scale_angular", a_scale_, a_scale_);
   nh_.param("scale_linear", l_scale_, l_scale_);


   //vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);


   joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy_CTL::joyCallback, this);

 }

void Joy_CTL::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  //geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  //vel_pub_.publish(twist);
  ROS_INFO("show it to me baby: %d", twist.linear.x);
  ROS_INFO("show it to me baby: %d", twist.angular.z);
}


//Joy_CTL teleop_turtle;
