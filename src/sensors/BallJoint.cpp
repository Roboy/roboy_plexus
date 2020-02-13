#include "sensors/BallJoint.hpp"

BallJoint::BallJoint(vector<int32_t*> i2c_base){
  for(int32_t* base:i2c_base){
    tle.push_back(TLE493DPtr(new TLE493D(base)));
  }
}

void BallJoint::readMagneticData(vector<float> &mx,vector<float> &my,vector<float> &mz){
  for(auto t:tle){
    float fx,fy,fz;
    t->read(fx,fy,fz);
    mx.push_back(fx);
    my.push_back(fy);
    mz.push_back(fz);
  }
}
