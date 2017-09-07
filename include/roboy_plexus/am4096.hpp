/* AM4096 - 12 bit angular magnetic encoder IC */
#pragma once

#include <bitset>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <roboy_plexus/i2c.hpp>

using namespace std;

class AM4096 {
private:
	void * h2p_lw_i2c_addr;

public:
	/**
	 * Constructor
	 * @param baseAddr fpga i2C base address
	 */
	AM4096(int32_t * baseAddr);
	/**
	 * Constructor
	 * @param baseAddr fpga i2C base address
	 * @param i2cAddrs vector of device ids
	 */
	AM4096(int32_t * baseAddr, vector<int> &i2cAddrs);
	~AM4096();
	/**
	 * Reads the absolute angles of all devices
	 * @param absAngles the angles
	 */
	void readAbsAngle(vector<uint32_t> &absAngles);
	/**
	 * Reads the ansolute angle of a device
	 * @param i2cAddr the device id
	 * @param absAngle the angle
	 * @return dataOK
	 */
	bool readAbsAngle(uint8_t i2cAddr, uint32_t &absAngle);
	/**
	 * Reads the relative angles of all devices
	 * @param relAngles the angles
	 */
	void readRelAngle(vector<uint32_t> &relAngles);
	/**
	 * Reads the relative angle of a device
	 * @param i2cAddr the device id
	 * @param relAngle the angle
	 * @return dataOK
	 */
	bool readRelAngle(uint8_t i2cAddr, uint32_t &relAngle);
	/**
	 * Reads the magnet status of all devices
	 * @param magnetTooFar
	 * @param magnetTooClose
	 */
	void readMagnetStatus(vector<bool> &magnetTooFar, vector<bool> &magnetTooClose);
	void readMagnetStatus(vector<unsigned char> &magnetTooFar, vector<unsigned char> &magnetTooClose);
	/**
	 * Reads the magnet status of a device
	 * @param i2cAddr the device id
	 * @param magnetTooFar
	 * @param magnetTooClose
	 */
	void readMagnetStatus(uint8_t i2cAddr, bool &magnetTooFar, bool &magnetTooClose);
	/**
	 * Reads AGC gain (magnet "power") of all devices
	 * @param agcGain the gains
	 */
	void readAgcGain(vector<uint8_t> &agcGain);
	/**
	 * Reads AGC gain (magnet "power")
	 * @param i2cAddr address
	 * @param agcGain the gain
	 */
	void readAgcGain(uint8_t i2cAddr, uint8_t &agcGain);
	/**
	 * Reads tacho of all devices
	 * @param tacho tacho values
	 */
	void readTacho(vector<uint32_t> &tacho);
	/**
 	 * Reads tacho, returns
	 * @param i2cAddr address
	 * @param tacho tacho value
	 * @return tacho overflow.
	 */
	bool readTacho(uint8_t i2cAddr, uint32_t &tacho);

	vector<int> i2cAddrs;
private:
	I2C *i2c;
};