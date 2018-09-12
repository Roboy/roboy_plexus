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
    i2c->read_continuous(0x5e, 10, regdata);
    if(true) {
        ROS_DEBUG("register content:");
        uint i = 0;
        for (uint8_t val:regdata) {
            printf("%d\t: " BYTE_TO_BINARY_PATTERN"\n", i++, BYTE_TO_BINARY(val));
        }
    }

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

    ROS_DEBUG("Reading config now ...");
    regdata.clear();
    i2c->read_continuous(0x5e, 10, regdata);
    if(false) {
        ROS_DEBUG("register content:");
        uint i = 0;
        for (uint8_t val:regdata) {
            printf("%d\t: " BYTE_TO_BINARY_PATTERN"\n", i++, BYTE_TO_BINARY(val));
        }
    }
}

TLV493D::~TLV493D() {
    // deactivate all sensors
    IOWR(i2c_base, i2c->GPIO_CONTROL, 0);
}

float TLV493D::convertToMilliTesla(uint8_t data) {
    float mTs = 0;
    uint8_t bitmask = 1;
    for(int i=0; i<7; i++){
        mTs += (data&bitmask);
        bitmask <<= 1;
    }
    mTs -= (data&bitmask);
    return (mTs*1.56f);
}

void TLV493D::readTLV_B_MSB(int deviceaddress, vector<uint8_t> &data) {
    // Read the first 3 registers only. Corresponding to the 8bit MSB values of the magnetic field
    i2c->read_continuous(deviceaddress,3, data);
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

void TLV493D::read(vector<float> &x, vector<float> &y, vector<float> &z){
    for(uint8_t device:deviceAddress){
        vector<uint8_t> data;
        readTLV_B_MSB(device,data);
        float fx = convertToMilliTesla(data[0]);
        float fy = convertToMilliTesla(data[1]);
        float fz = convertToMilliTesla(data[2]);
        x.push_back(((fabs(fx)-1.55999994)<0.000001?0:fx));
        y.push_back(((fabs(fy)-1.55999994)<0.000001?0:fy));
        z.push_back(((fabs(fz)-1.55999994)<0.000001?0:fz));
    }
}