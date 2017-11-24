#pragma once

#include <stdio.h>
#include <unistd.h>
#include <cstdint>
#include <fcntl.h>
#include "hwlib.h"
#include "hps_0.h"
#include <vector>

using namespace std;

class I2C {

public:
	I2C(void * baseAddr);
	void * h2p_lw_i2c_addr;

	// registers: read or write
	const uint8_t ADDR = 0;
	const uint8_t DATA = 1;  // note that internally there are 2 registers for data: data_rd and data_wr
	const uint8_t RW = 2;
	const uint8_t ENA = 3;
	const uint8_t NUMBER_OF_BYTES = 4;

	// registers: only write
	const uint8_t GPIO_CONTROL = 5;
	const uint8_t READ_ONLY = 6;

	// registers: only read
	const uint8_t BUSY = 4;
	const uint8_t ACK_ERROR = 5;
	const uint8_t FIFO_SIZE = 6;

	// operations
	const uint8_t WRITE = 0;
	const uint8_t READ = 1;

    /**
     * Writes up to three bytes to an address
     * @param i2cAddr device address
     * @param data (reg<<24|data[0]<<16|data[1]<<8|data[2])
     * @param number_of_bytes length(data)+1 for register
     */
	void write(uint8_t i2cAddr, uint32_t data, uint8_t number_of_bytes);
	void read(uint8_t i2cAddr, uint8_t reg, uint8_t number_of_bytes, vector<uint8_t> &data);
	void read_continuous(uint8_t i2cAddr, uint8_t number_of_bytes, vector<uint8_t> &data);
    bool ack_error();
};

#define IORD(base,reg) (*(((volatile uint32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile uint32_t*)base)+reg)=data)
#define IORD_8DIRECT(base, offset) (*(((volatile uint8_t*)base)+offset))
#define IORD_16DIRECT(base, offset) (*(((volatile uint16_t*)base)+(offset>>1)))
#define IORD_32DIRECT(base, offset) (*(((volatile uint32_t*)base)+(offset>>2)))
#define IOWR_8DIRECT(base, offset, data) (*(((volatile uint8_t*)base)+offset)=data)
#define IOWR_16DIRECT(base, offset, data) (*(((volatile uint16_t*)base)+(offset>>1))=data)
#define IOWR_32DIRECT(base, offset, data) (*(((volatile uint32_t*)base)+(offset>>2))=data)