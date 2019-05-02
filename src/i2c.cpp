#include "roboy_plexus/i2c.hpp"

I2C::I2C(void * baseAddr):h2p_lw_i2c_addr(baseAddr) {
//    vector<uint8_t> activeDevices;
//    checkAddressSpace(0,255,activeDevices);
//    stringstream str;
//    str << "active i2c devices on bus witch base address " << baseAddr << ": ";
//    for(int device:activeDevices){
//        str << hex << device << " ";
//    }
//    ROS_INFO_STREAM(str.str());
    IOWR(h2p_lw_i2c_addr, RESET_TLV, 0);
    IOWR(h2p_lw_i2c_addr, TLV_SCL, 1);
    IOWR(h2p_lw_i2c_addr, TLV_SDA, 1);
}

void I2C::write(uint8_t i2cAddr, uint32_t data, uint8_t number_of_bytes) {
    // we need this small pause!
    usleep(1);
    IOWR(h2p_lw_i2c_addr, READ_ONLY, 0);
    // Set slave address
    IOWR(h2p_lw_i2c_addr, ADDR, i2cAddr);
    // Set data to write
    IOWR(h2p_lw_i2c_addr, DATA, data);
    // Set the number of bytes to be written
    IOWR(h2p_lw_i2c_addr, NUMBER_OF_BYTES, number_of_bytes);
    // Set operation mode: write
    IOWR(h2p_lw_i2c_addr, RW, WRITE);
    // Start operation (enable = 1)
    IOWR(h2p_lw_i2c_addr, ENA, 1);
    IORD(h2p_lw_i2c_addr, DATA);// this blocks until transmission is done
}


void I2C::read(uint8_t i2cAddr, uint8_t reg, uint8_t number_of_bytes, vector<uint8_t> &data) {
    // we need this small pause!
    usleep(1);
    IOWR(h2p_lw_i2c_addr, READ_ONLY, 0);
    // Set slave address
    IOWR(h2p_lw_i2c_addr, ADDR, i2cAddr);
    // Set operation mode: read
    IOWR(h2p_lw_i2c_addr, RW, READ);
    // Set data to write
    IOWR(h2p_lw_i2c_addr, DATA, (reg<<24));
    // Set the number of bytes to be read + 1 for the register to read from
    IOWR(h2p_lw_i2c_addr, NUMBER_OF_BYTES, number_of_bytes+1);
    // Start operation (enable = 1)
    IOWR(h2p_lw_i2c_addr, ENA, 1);
    // read the fifo biatch
    uint32_t val;
    if(number_of_bytes>4) {
        for(uint i=0;i<number_of_bytes/4;i++){
            val = IORD(h2p_lw_i2c_addr, DATA);
            data.push_back((val>>24)&0xff);
            data.push_back((val>>16)&0xff);
            data.push_back((val>>8)&0xff);
            data.push_back((val>>0)&0xff);
        }

        if ((number_of_bytes % 4) > 0) {
            val = IORD(h2p_lw_i2c_addr, DATA);
            uint8_t rest_bytes = number_of_bytes % 4;
            for (uint i = 0; i < rest_bytes; i++) {
                data.push_back((val >> (3 - i) * 8) & 0xff);
            }
        }
    }else{
        val = IORD(h2p_lw_i2c_addr, DATA);
        for (uint i = 0; i < number_of_bytes; i++) {
            data.push_back((val >> (3 - i) * 8) & 0xff);
        }
    }
}

void I2C::read_continuous(uint8_t i2cAddr, uint8_t number_of_bytes, vector<uint8_t> &data) {
    IOWR(h2p_lw_i2c_addr, ENA, 0);
    IOWR(h2p_lw_i2c_addr, READ_ONLY, 1); // Set this register to enable reading from the I2C bus without having to write
    // the address of the register to read beforehand.
    // we need this small pause!
    usleep(1);
    // Set slave address
    IOWR(h2p_lw_i2c_addr, ADDR, i2cAddr);
    // Set operation mode: read
    IOWR(h2p_lw_i2c_addr, RW, READ);
    // Set the number of bytes to be read
    IOWR(h2p_lw_i2c_addr, NUMBER_OF_BYTES, number_of_bytes);
    // Start operation (enable = 1)
    IOWR(h2p_lw_i2c_addr, ENA, 1);

//    printf("fifo size: %d\n", IORD(h2p_lw_i2c_addr, FIFO_SIZE));

    // read the fifo biatch
    uint32_t reg;
    for(uint i=0;i<number_of_bytes/4;i++){
        reg = IORD(h2p_lw_i2c_addr, DATA);
        data.push_back((reg>>24)&0xff);
        data.push_back((reg>>16)&0xff);
        data.push_back((reg>>8)&0xff);
        data.push_back((reg>>0)&0xff);
    }
    if((number_of_bytes%4)>0){
        reg = IORD(h2p_lw_i2c_addr, DATA);
        uint8_t rest_bytes = number_of_bytes%4;
        for(uint i=0;i<rest_bytes;i++){
            data.push_back((reg>>(3-i)*8)&0xff);
        }
    }
}

bool I2C::ack_error() {
    return IORD(h2p_lw_i2c_addr, ACK_ERROR);
}

bool I2C::checkAddressSpace(uint8_t fromDeviceID, uint8_t toDeviceID, vector<uint8_t> &activeDevices){
    for(fromDeviceID;fromDeviceID<toDeviceID;fromDeviceID++){
        vector<uint8_t> data;
        read_continuous(fromDeviceID,1,data);
        if(!ack_error())
            activeDevices.push_back(fromDeviceID);
    }
    return !activeDevices.empty();
}

void I2C::resetTLV(){
    IOWR(h2p_lw_i2c_addr, RESET_TLV, true);
    IOWR(h2p_lw_i2c_addr, RESET_TLV, false);
}