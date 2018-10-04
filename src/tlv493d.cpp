#include "roboy_plexus/tlv493d.hpp"

TLV493D::TLV493D(void *i2c_base, vector<uint8_t> &deviceAddress, vector<int> &devicePin):i2c_base(i2c_base),deviceAddress(deviceAddress){
    i2c = boost::shared_ptr<I2C>(new I2C(i2c_base));
    vector<uint8_t> data;
    i2c->read(0x5e, 1, 1, data);
    if (i2c->ack_error()) {
        ROS_INFO("TLV sensor active at: %x", 0x5e);
    }else{
        ROS_ERROR("sensor does not respond on address: %x", 0x5e);
    }

    vector<uint8_t> regdata;
    ROS_INFO("before initialization");
    readAllRegisters(0x5e,regdata,true);

    // Begin config
    // Static initial config for now
    uint32_t cfgdata = 0;
    cfgdata |=  ((((0b010<<5)|(regdata[9]&0b11111))<<0)|(regdata[8]<<8)|(((0<<5)|((regdata[7]&0b00011000)|0b010))<<16));  // Last 3 bits: INT/FAST/LP
    ROS_DEBUG("config 0\t: " BYTE_TO_BINARY_PATTERN" \t%x\n",BYTE_TO_BINARY(cfgdata&0xff), cfgdata&0xff);
    ROS_DEBUG("config 1\t: " BYTE_TO_BINARY_PATTERN" \t%x\n",BYTE_TO_BINARY(((cfgdata>>8)&0xff)), (cfgdata>>8)&0xff);
    ROS_DEBUG("config 2\t: " BYTE_TO_BINARY_PATTERN" \t%x\n",BYTE_TO_BINARY(((cfgdata>>16)&0xff)), (cfgdata>>16)&0xff);
    ROS_DEBUG("config 3\t: " BYTE_TO_BINARY_PATTERN" \t%x\n",BYTE_TO_BINARY(((cfgdata>>24)&0xff)), (cfgdata>>24)&0xff);

//    // Write config
    ROS_DEBUG("Writing config now ...");
    i2c->write(0x5e, cfgdata, 4);

    ROS_INFO("Reading config now ...");
    regdata.clear();
    readAllRegisters(0x5e,regdata,true);
}

TLV493D::~TLV493D() {
    // deactivate all sensors
    IOWR(i2c_base, i2c->GPIO_CONTROL, 0);
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
    i2c->read_continuous(0x5e,6, data);
    if((data[3]&0x3)!=0)
        return false;

    fx = convertToMilliTesla(data[0], (uint8_t)(data[4]>>4));
    fy = convertToMilliTesla(data[1], (uint8_t)(data[4]&0xF));
    fz = convertToMilliTesla(data[2], (uint8_t)(data[5]&0xF));
//    fx = (uint16_t)(data[0]<<8|(uint8_t)(data[4]>>4));//convertToMilliTesla(data[0], (uint8_t)(data[4]>>4));
//    fy = (uint16_t)(data[1]<<8|(uint8_t)(data[4]&0xF));
//    fz = (uint16_t)(data[2]<<8|(uint8_t)(data[5]&0xF));
    return true;
}