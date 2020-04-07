#include "sensors/tle493d_w2b6.hpp"

TLE493D::TLE493D(int32_t *i2c_base):i2c_base(i2c_base){
    i2c = boost::shared_ptr<I2C>(new I2C(i2c_base));
    // configure to 1-byte protocol
    i2c->write(0x35, ((0x11 << 24) | 0x10 << 16), 2);
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
    return val*0.098;
}

bool TLE493D::read(float &fx, float &fy, float &fz){
    vector<uint8_t> data;
    i2c->read_continuous(0x35,7, data);
    fx = convertToMilliTesla(data[0], (uint8_t)(data[4]>>4));
    fy = convertToMilliTesla(data[1], (uint8_t)(data[4]&0xF));
    fz = -convertToMilliTesla(data[2], (uint8_t)(data[5]&0xF));
    return true;
}
