#include "sensors/BallJoint.hpp"

BallJoint::BallJoint(int32_t* base, int number_of_sensors):base(base),number_of_sensors(number_of_sensors){
  BALL_JOINT_WRITE_reset(base,1);
  BALL_JOINT_WRITE_update_frequency(base,100);
  ROS_INFO("balljoint with %d sensors and update frequency 100",number_of_sensors);
}

void BallJoint::readMagneticData(vector<uint8_t> &sensor_id, vector<float> &mx,vector<float> &my,vector<float> &mz){
  for(int i=0;i<number_of_sensors;i++){
    float fx,fy,fz;
    fx = convertToMilliTesla(BALL_JOINT_READ_mag_x(base,i));
    fy = convertToMilliTesla(BALL_JOINT_READ_mag_y(base,i));
    fz = convertToMilliTesla(BALL_JOINT_READ_mag_z(base,i));
    sensor_id.push_back(i);
    mx.push_back(fx);
    my.push_back(fy);
    mz.push_back(fz);
  }
}

float BallJoint::convertToMilliTesla(uint16_t data) {
    int val = 0;
    for(int i=11;i>=0;i--){
        if(i==11){
            if((data>>i)&0x1)
                val = -2048;
        }else{
            if((data>>i)&0x1)
                val += (1<<i);
        }
    }
    return val*0.098;
}
