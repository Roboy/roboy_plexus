#include "sensors/tle493d_w2b6.hpp"

TLE493D::TLE493D(int32_t *i2c_base, uint8_t tca_addr, int number_of_sensors):
  i2c_base(i2c_base),number_of_sensors(number_of_sensors),tca_addr(tca_addr){
    i2c = boost::shared_ptr<I2C>(new I2C(i2c_base));
    for(int i=0;i<number_of_sensors;i++){
      i2c->write(tca_addr,((0x1<<i)<<24),1);
      // configure to 1-byte protocol
      i2c->write(0x35, ((0x11 << 24) | 0x10 << 16), 2);
    }
    ROS_WARN_ONCE("tle493 uses left-handed coordinate system, we publish right-handed coordinates!");
}

void TLE493D::reset(){

}

TLE493D::~TLE493D() {

}

float TLE493D::convertToMilliTesla(uint8_t MSB, uint8_t LSB) {
    uint16_t data = (MSB<<4|(LSB&0xF));
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
    return val*0.13;
}

bool TLE493D::readMagneticData(vector<uint8_t> &sensor_id, vector<float> &fx, vector<float> &fy, vector<float> &fz){
    for(int i=0;i<number_of_sensors;i++){
      sensor_id.push_back(i);
      i2c->write(tca_addr,((0x1<<i)<<24),1);
      vector<uint8_t> data;
      i2c->read_continuous(0x35,7, data);
      fx.push_back(convertToMilliTesla(data[0], (uint8_t)(data[4]>>4)));
      fy.push_back(convertToMilliTesla(data[1], (uint8_t)(data[4]&0xF)));
      fz.push_back(-convertToMilliTesla(data[2], (uint8_t)(data[5]&0xF)));
    }
    i2c->write(tca_addr,0,1);
    return true;
}
