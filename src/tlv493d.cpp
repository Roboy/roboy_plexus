#include "roboy_plexus/tlv493d.hpp"

TLV493D::TLV493D(int32_t *i2c_base):i2c_base(i2c_base){
    i2c = boost::shared_ptr<I2C>(new I2C(i2c_base));
//    reset();
}

void TLV493D::reset(){
    i2c->resetTLV();
    {
        vector<uint8_t> regdata;
        readAllRegisters(0x5e, regdata, false);
        // Static initial config for now
        uint32_t cfgdata = 0;
        cfgdata |= (0b0000011 | (regdata[7] & 0b0011000)) << 8;
        cfgdata |= (regdata[8] << 16);
        cfgdata |= (0b01000000 | (0b11111 & regdata[9])) << 24;
        if (!checkParity(cfgdata))
            cfgdata |= (0b10000000 << 8);
        i2c->write(0x5e, cfgdata, 4);
        vector<uint8_t> data;
        i2c->read_continuous(0x5e,7, data);
        frameCounter = data[3]>>2&0x3;
    }
    {
        vector<uint8_t> regdata;
        readAllRegisters(0x5e, regdata, false);
        // Static initial config for now
        uint32_t cfgdata = 0;
        cfgdata |= (0b0000011 | (regdata[7] & 0b0011000)) << 8;
        cfgdata |= (regdata[8] << 16);
        cfgdata |= (0b01000000 | (0b11111 & regdata[9])) << 24;
        if (!checkParity(cfgdata))
            cfgdata |= (0b10000000 << 8);
        i2c->write(0x5e, cfgdata, 4);
        vector<uint8_t> data;
        i2c->read_continuous(0x5e, 7, data);
        frameCounter = data[3] >> 2 & 0x3;
    }
    frameCounter++;
}

TLV493D::~TLV493D() {
    // deactivate all sensors
    IOWR(i2c_base, i2c->GPIO_CONTROL, 0);
}

bool TLV493D::checkParity(uint32_t v){
    v ^= v >> 1;
    v ^= v >> 2;
    v = (v & 0x11111111U) * 0x11111111U;
    return (v >> 28) & 1;
}

float TLV493D::convertToMilliTesla(uint8_t MSB, uint8_t LSB) {
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

float TLV493D::convertToMilliTesla(uint32_t data) {
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

void TLV493D::readAllRegisters(int deviceaddress, vector<uint8_t> &reg, bool print){
    i2c->read_continuous(deviceaddress, 10, reg);
    if(print) {
        ROS_INFO("register content:");
        uint i = 0;
        for (uint8_t val:reg) {
            printf("%d\t: " BYTE_TO_BINARY_PATTERN"\n", i++, BYTE_TO_BINARY(val));
        }
    }
}

bool TLV493D::read(float &fx, float &fy, float &fz){
    vector<uint8_t> data;
    i2c->read_continuous(0x5e,7, data);
    frameCounter++;
//    ROS_INFO("%d %d", (data[3]>>2)&0x3, frameCounter%4);
    bool success = true;
    if((((data[3]>>2)&0x3)!=(frameCounter%4))){
        reset();
        ROS_WARN_THROTTLE(5,"oh oh frame counter incorrect, attemptimg to reset tlv sensor");
        success = false;
    }
//    if((data[3]&0x3)!=0){
//        ROS_WARN_THROTTLE(1,"sensor values not ready yet");
//        success = false;
//    }
    if(!success)
        return false;
    fx = convertToMilliTesla(data[0], (uint8_t)(data[4]>>4));
    fy = convertToMilliTesla(data[1], (uint8_t)(data[4]&0xF));
    fz = convertToMilliTesla(data[2], (uint8_t)(data[5]&0xF));
    return true;
}

void TLV493D::updateData(){
    i2c->write(0x5e,0x00830060,4);
}

bool TLV493D::readData100Hz(float &fx, float &fy, float &fz){
    vector<uint8_t> data;
    i2c->read_continuous(0x5e,7, data);
    i2c->write(0x5e,0x00000020,4);
    fx = convertToMilliTesla(data[0], (uint8_t)(data[4]>>4));
    fy = convertToMilliTesla(data[1], (uint8_t)(data[4]&0xF));
    fz = convertToMilliTesla(data[2], (uint8_t)(data[5]&0xF));
}