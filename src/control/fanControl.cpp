#include "control/fanControl.hpp"

FanControl::FanControl(int32_t* base):base(base){

}

bool FanControl::GetAutoFan(){
  return IORD(base,2);
}

int FanControl::GetCurrentAverage() {
    return IORD(base,4);
}

int FanControl::GetDuty(){
  return IORD(base,1);
}

int FanControl::GetPWMFrequency(){
  return IORD(base,0);
}

int FanControl::GetSensitivity(){
  return IORD(base,3);
}

void FanControl::SetAutoFan(bool autofan){
  IOWR(base,2,autofan);
}

void FanControl::SetDuty(int duty){
  if(duty<0||duty>100){
    ROS_ERROR("exceeding duty range [0-100]: %d", duty);
    return;
  }
  IOWR(base,1,duty);
}

void FanControl::SetPWMFrequency(int freq){
  if(freq==0){
    ROS_ERROR("a frequency of 0 does not make sense friend, keeping current frequency %d",GetPWMFrequency());
    return;
  }
  IOWR(base,0,freq);
}

void FanControl::SetSensitivity(int sensitivity){
  IOWR(base,3,sensitivity);
}
