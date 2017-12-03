#include <roboy_plexus/am4096.hpp>

AM4096::AM4096(int32_t * baseAddr){
	i2c = new I2C(baseAddr);
}

AM4096::AM4096(int32_t * baseAddr, vector<int> &i2cAddrs):i2cAddrs(i2cAddrs){
	i2c = new I2C(baseAddr);
}

AM4096::~AM4096(){
	delete i2c;
}

void AM4096::readAbsAngle(vector<uint32_t> &absAngles){
	for(int const addr:i2cAddrs){
		uint32_t val;
		bool dataOk = readAbsAngle(addr,val);
		absAngles.push_back(val);
//		if(!dataOk)
//			ROS_WARN("abs angle data of sensor %d NOT ok", addr);
	}
}

bool AM4096::readAbsAngle(uint8_t i2cAddr, uint32_t &absAngle){
	vector<uint8_t> data;
	i2c->read(i2cAddr, 33, 2, data);
	// reg = DataOK + 3x"0" + 12x bit abs angle + 16x"0"

	// TODO: check if this works with new I2C core
	bool dataOK = (bool)((data[0] >> 7) & 0b1);

	//std::bitset<32> bits(reg);
	//std::cout << "readAbsAngle " << bits << std::endl;

	absAngle = (uint32_t)(((data[0] & 0xf) << 8) | data[1]);
	return dataOK;
}

void AM4096::readRelAngle(vector<uint32_t> &relAngles){
	for(int const addr:i2cAddrs){
		uint32_t val;
		bool dataOk = readRelAngle(addr,val);
		relAngles.push_back(val);
		usleep(1000);
//		if(!dataOk)
//			ROS_WARN("rel angle data of sensor %d NOT ok", addr);
	}
}

bool AM4096::readRelAngle(uint8_t i2cAddr, uint32_t &relAngle){
	// TODO: check if this works with new I2C core
	vector<uint8_t> data;
	i2c->read(i2cAddr, 32, 2, data);
//	// reg = DataOK + 3x"0" + 12x bit rel angle + 16x"0"
//
	bool dataOK = (bool)((data[0] >> 7) & 0b1);
//
	relAngle = (uint32_t)(((data[0] & 0xf) << 8) | data[1]);
	return dataOK;
}

void AM4096::readMagnetStatus(vector<bool> &magnetTooFar, vector<bool> &magnetTooClose){
	for(int const addr:i2cAddrs){
		bool tooFar, tooClose;
		readMagnetStatus(addr, tooFar, tooClose);
		magnetTooFar.push_back(tooFar);
		magnetTooClose.push_back(tooClose);
	}
}

void AM4096::readMagnetStatus(vector<unsigned char> &magnetTooFar, vector<unsigned char> &magnetTooClose){
	for(int const addr:i2cAddrs){
		bool tooFar, tooClose;
		readMagnetStatus(addr, tooFar, tooClose);
		magnetTooFar.push_back((unsigned char)tooFar);
		magnetTooClose.push_back((unsigned char)tooClose);
	}
}

void AM4096::readMagnetStatus(uint8_t i2cAddr, bool &magnetTooFar, bool &magnetTooClose){
	// TODO: check if this works with new I2C core
	vector<uint8_t> data;
	i2c->read(i2cAddr, 34, 2, data);

	// reg = "?" + Weh + Wel + 29x"?"
	// Weh Magnet too far
	// Wel Magnet too close

	magnetTooFar = (bool)((data[0] >> 6) & 0x1);
	magnetTooClose = (bool)((data[0] >> 5) & 0x1);
}

void AM4096::readAgcGain(vector<uint8_t> &agcGain){
	for(int const addr:i2cAddrs){
		uint8_t gain;
		readAgcGain(addr, gain);
		agcGain.push_back(gain);
	}
}

void AM4096::readAgcGain(uint8_t i2cAddr, uint8_t &agcGain) {
	// TODO: check if this works with new I2C core
	vector<uint8_t> data;
	i2c->read(i2cAddr, 35, 2, data);
	// reg = 4x agcGain bits + "?" + Thof + 10x Tho bits + 16x "?"
	agcGain = (uint8_t)((data[0] >> 4) & 0x1);
}

void AM4096::readTacho(vector<uint32_t> &tacho){
	for(int const addr:i2cAddrs){
		uint32_t val;
		bool overFlow = readTacho(addr,val);
		tacho.push_back(val);
//		if(overFlow)
//			ROS_WARN("tach overflow for sensor %d", addr);
	}
}

bool AM4096::readTacho(uint8_t i2cAddr, uint32_t &tacho) {
	// TODO: check if this works with new I2C core
	vector<uint8_t> data;
	i2c->read(i2cAddr, 35, 2, data);
	// reg = 4x agcGain bits + "?" + Thof + 10x Tho bits + 16x "?"
	tacho = (uint32_t)(((data[0] & 0b11) << 8) | data[1]);
	bool overflow = (bool)((data[0] >> 2) & 0x1);
	return overflow;
}
