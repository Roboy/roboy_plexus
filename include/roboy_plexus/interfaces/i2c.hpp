/*
    BSD 3-Clause License

    Copyright (c) 2017, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Simon Trendel ( simon.trendel@tum.de ), 2018
    description: i2c interface to fpga i2c core via lightweight axi bridge
*/

#pragma once

// #include <ros/ros.h>
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
	/**
	 * Constructor
	 * @param baseAddr i2c base address (cf hps_0.h)
	 */
	I2C(void * baseAddr);
	/**
     * Writes up to three bytes to an address
     * @param i2cAddr device address
     * @param data (reg<<24|data[0]<<16|data[1]<<8|data[2])
     * @param number_of_bytes length(data)+1 for register
     */
	void write(uint8_t i2cAddr, uint32_t data, uint8_t number_of_bytes);
	/**
	 * Reads up to number of bytes from i2c slave
	 * @param i2cAddr i2c slave address
	 * @param reg start register
	 * @param number_of_bytes hwo many bytes should be read
	 * @param data will be filled with the read data
	 */
	void read(uint8_t i2cAddr, uint8_t reg, uint8_t number_of_bytes, vector<uint8_t> &data);
	/**
	 * Reads up to number of bytes from i2c slave, without start register (behaviour depends on slave device)
	 * @param i2cAddr i2c slave address
	 * @param number_of_bytes hwo many bytes should be read
	 * @param data will be filled with the read data
	 */
	void read_continuous(uint8_t i2cAddr, uint8_t number_of_bytes, vector<uint8_t> &data);
	/**
	 * Queries for acknowledge error
	 * @return error/no error
	 */
	bool ack_error();
	/**
	 * Scans the provided address range for active devices
	 * @param fromDeviceID start address
	 * @param toDeviceID end address
	 * @param activeDevices will be filled with active device addresses
	 * @return true if at least one active device was found
	 */
	bool checkAddressSpace(uint8_t fromDeviceID, uint8_t toDeviceID, vector<uint8_t> &activeDevices);
	/**
	 * Runs a custom reset sequence for the tlv493 chip
	 */
	void resetTLV();
private:
	void * h2p_lw_i2c_addr;

public:
	// registers: read or write
	const uint8_t ADDR = 0;
	const uint8_t DATA = 1;  // note that internally there are 2 registers for data: data_rd and data_wr
	const uint8_t RW = 2;
	const uint8_t ENA = 3;
	const uint8_t NUMBER_OF_BYTES = 4;
	const uint8_t TLV_SDA = 8;
	const uint8_t TLV_SCL = 9;

	// registers: only write
	const uint8_t GPIO_CONTROL = 5;
	const uint8_t READ_ONLY = 6;
	const uint8_t RESET_TLV = 7;

	// registers: only read
	const uint8_t BUSY = 4;
	const uint8_t ACK_ERROR = 5;
	const uint8_t FIFO_SIZE = 6;

	// operations
	const uint8_t WRITE = 0;
	const uint8_t READ = 1;
};

#define IORD(base,reg) (*(((volatile uint32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile uint32_t*)base)+reg)=data)
#define IORD_8DIRECT(base, offset) (*(((volatile uint8_t*)base)+offset))
#define IORD_16DIRECT(base, offset) (*(((volatile uint16_t*)base)+(offset>>1)))
#define IORD_32DIRECT(base, offset) (*(((volatile uint32_t*)base)+(offset>>2)))
#define IOWR_8DIRECT(base, offset, data) (*(((volatile uint8_t*)base)+offset)=data)
#define IOWR_16DIRECT(base, offset, data) (*(((volatile uint16_t*)base)+(offset>>1))=data)
#define IOWR_32DIRECT(base, offset, data) (*(((volatile uint32_t*)base)+(offset>>2))=data)
