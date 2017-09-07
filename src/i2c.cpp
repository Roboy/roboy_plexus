#include <roboy_plexus/i2c.hpp>

I2C::I2C(void * baseAddr):h2p_lw_i2c_addr(baseAddr) {
}

void I2C::write(uint8_t i2cAddr, uint32_t data, uint8_t number_of_bytes) {
    // we need this small pause!
    usleep(1);
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
    // this will block until transmission is done
    IORD(h2p_lw_i2c_addr, BUSY);
}


uint32_t I2C::read(uint8_t i2cAddr, uint8_t reg, uint8_t number_of_bytes) {
    // we need this small pause!
    usleep(1);
    // Set slave address
    IOWR(h2p_lw_i2c_addr, ADDR, i2cAddr);
    // Set operation mode: read
    IOWR(h2p_lw_i2c_addr, RW, READ);
    // Set data to write
    IOWR(h2p_lw_i2c_addr, DATA, reg<<24); // MSB is written first
    // Set the number of bytes to be read + 1 for the register to read from
    IOWR(h2p_lw_i2c_addr, NUMBER_OF_BYTES, number_of_bytes+1);
    // Start operation (enable = 1)
    IOWR(h2p_lw_i2c_addr, ENA, 1);
    // read and return data
    return IORD(h2p_lw_i2c_addr, DATA);
}

bool I2C::ack_error(){
    return IORD(h2p_lw_i2c_addr, ACK_ERROR);
}
