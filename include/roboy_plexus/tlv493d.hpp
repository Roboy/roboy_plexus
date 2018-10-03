#pragma  once

#include <stdint.h>
#include "i2c.hpp"
#include <vector>
#include <ros/ros.h>

using namespace std;

#define bitRead(byte,pos) ((byte) & (1<<(pos)))
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')



class TLV493D{
public:
    TLV493D(void *i2c_base, vector<uint8_t> &deviceAddress, vector<int> &devicePin);
    ~TLV493D();
    bool initTLV(uint8_t &deviceaddress, int devicepin);
    float convertToMilliTesla(uint8_t MSB, uint8_t LSB) ;
    void readAllRegisters(int deviceaddress, vector<uint8_t> &reg, bool print=true);
    bool read(float &fx, float &fy, float &fz);
private:
    vector<uint8_t> deviceAddress;
    uint8_t gpioreg = 0;
public:
    boost::shared_ptr<I2C> i2c;
    void *i2c_base;
};
