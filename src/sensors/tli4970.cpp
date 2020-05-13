#include "sensors/tli4970.hpp"

TLI4970::TLI4970(int32_t *base, int number_of_sensors):base(base),number_of_sensors(number_of_sensors){
  ROS_INFO("%d current sensors initialized",number_of_sensors);
}

void TLI4970::readCurrent(vector<float> &current){
  for(int i=0;i<number_of_sensors;i++){
    int16_t current_raw = IORD(base,i);
    current.push_back((current_raw/80.f));
  }
};
