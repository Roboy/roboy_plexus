#pragma once

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "hwlib.h"
#include "roboy_plexus/hps_0.h"

class I2C {
private:
	void * h2p_lw_i2c_addr;

public:
	I2C(void * baseAddr);

	// registers: read or write
	const uint8_t ADDR = 0;
	const uint8_t DATA = 1;  // note that internally there are 2 registers for data: data_rd and data_wr
	const uint8_t RW = 2;
	const uint8_t ENA = 3;
	const uint8_t NUMBER_OF_BYTES = 4;

	// registers: only read
	const uint8_t BUSY = 4;
	const uint8_t ACK_ERROR = 5;

	// operations
	const uint8_t WRITE = 0;
	const uint8_t READ = 1;

	void write(uint8_t i2cAddr, uint32_t data, uint8_t number_of_bytes);
	uint32_t read(uint8_t i2cAddr, uint8_t reg, uint8_t number_of_bytes);
    bool ack_error();
};

#define IORD(base,reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile int32_t*)base)+reg)=data)
#define IORD_8DIRECT(base, offset) (*(((volatile int8_t*)base)+offset))
#define IORD_16DIRECT(base, offset) (*(((volatile int16_t*)base)+(offset>>1)))
#define IORD_32DIRECT(base, offset) (*(((volatile int32_t*)base)+(offset>>2)))
#define IOWR_8DIRECT(base, offset, data) (*(((volatile int8_t*)base)+offset)=data)
#define IOWR_16DIRECT(base, offset, data) (*(((volatile int16_t*)base)+(offset>>1))=data)
#define IOWR_32DIRECT(base, offset, data) (*(((volatile int32_t*)base)+(offset>>2))=data)
