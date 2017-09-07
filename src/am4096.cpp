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
	uint32_t reg = i2c->read(i2cAddr, 33, 2);
	// reg = DataOK + 3x"0" + 12x bit abs angle + 16x"0"

	bool dataOK = (reg >> 31) & 0b1;

	//std::bitset<32> bits(reg);
	//std::cout << "readAbsAngle " << bits << std::endl;

	absAngle = (reg >> 16) & 0xFFF;
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
	uint32_t reg = i2c->read(i2cAddr, 32, 2);
	// reg = DataOK + 3x"0" + 12x bit rel angle + 16x"0"

	bool dataOK = (reg >> 31) & 0b1;

	relAngle = (reg >> 16) & 0xFFF;
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
	uint32_t reg = i2c->read(i2cAddr, 34, 2);

	// reg = "?" + Weh + Wel + 29x"?"
	// Weh Magnet too far
	// Wel Magnet too close

	magnetTooFar = (reg >> 30) & 0x1;
	magnetTooClose = (reg >> 29) & 0x1;
}

void AM4096::readAgcGain(vector<uint8_t> &agcGain){
	for(int const addr:i2cAddrs){
		uint8_t gain;
		readAgcGain(addr, gain);
		agcGain.push_back(gain);
	}
}

void AM4096::readAgcGain(uint8_t i2cAddr, uint8_t &agcGain) {
	uint32_t reg = i2c->read(i2cAddr, 35, 2);
	// reg = 4x agcGain bits + "?" + Thof + 10x Tho bits + 16x "?"
	agcGain = (reg >> 28) & 0xF;
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
	uint32_t reg = i2c->read(i2cAddr, 35, 2);
	// reg = 4x agcGain bits + "?" + Thof + 10x Tho bits + 16x "?"
	tacho = (reg >> 16) & 0x3FF;
	bool overflow = (reg >> 26) & 0x1;
	return overflow;
}
