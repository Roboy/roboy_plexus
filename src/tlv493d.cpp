#include "roboy_plexus/tlv493d.hpp"

TLV493D::TLV493D(void *i2c_base, vector<uint8_t> &deviceAddress, vector<int> &devicePin):i2c_base(i2c_base),deviceAddress(deviceAddress){
    i2c = boost::shared_ptr<I2C>(new I2C(i2c_base));

    IOWR(i2c_base, i2c->GPIO_CONTROL, 0);
//    gpioreg = IORD(i2c_base,i2c->GPIO_CONTROL); // Read previous pin status so as to not overwrite any values
//    uint32_t val = (4<<24)|(3<<16)|(2<<8)|1;
//    i2c->write(5,val,4);

    for(int i=0; i<deviceAddress.size();i++){
        initTLV(deviceAddress[i],devicePin[i]);
    }
}

TLV493D::~TLV493D() {
    // deactivate all sensors
    IOWR(i2c_base, i2c->GPIO_CONTROL, 0);
}

bool TLV493D::initTLV(uint8_t &deviceaddress, int devicepin) {
    bool ADDR_pin;
    uint8_t IICAddr;
    uint8_t setaddr;

    if (devicepin == 255){
        ADDR_pin = false;
        IICAddr  = 0;
        setaddr  = 0b1011110;
        ROS_INFO("No device power pin selected, asuming device is on and continuing with the default address: %x .\n"
                         "Triggering general reset to make sure device is configured correctly",setaddr);
        IOWR(i2c_base, i2c->GPIO_CONTROL, (gpioreg|0b10000)); // pull sda line high
        i2c->write(0,1,1);   // Clock in address 0 to reset 'ALL' devices. And write 1,
        // very important to keep SDA line up or else address changes to 0b00011111
    }else{
        ADDR_pin = bitRead(deviceaddress,6);
        IICAddr  = (!bitRead(deviceaddress,4)<<1)|(!bitRead(deviceaddress,2));
        setaddr  = (uint8_t)((ADDR_pin<<6)|(!bitRead(IICAddr,1)<<4)|(1<<3)|(!bitRead(IICAddr,0)<<2)|(1<<1)|(!ADDR_pin));
    }

    ROS_INFO("setaddr:      \t" BYTE_TO_BINARY_PATTERN,BYTE_TO_BINARY(setaddr));
//    ROS_INFO("deviceaddress:\t" BYTE_TO_BINARY_PATTERN,BYTE_TO_BINARY(deviceaddress));

    if (setaddr != deviceaddress){
        ROS_WARN("Configuring device on pin %d with address: %x (ADDR_pin = %x, IICAddr = %x)", devicepin, setaddr, ADDR_pin, IICAddr);
        ROS_ERROR("Invalid device address %x! setaddr is %x . Please check table 6 in the manual and try again", deviceaddress, setaddr);
        return false;
    }else{
        ROS_DEBUG("Configuring device on pin %d with address: %x (ADDR_pin = %x, IICAddr = %x)", devicepin, setaddr, ADDR_pin, IICAddr);
    }

    if (ADDR_pin == true){
        // take control of the SDA line
        gpioreg |= 1UL << 4;
        IOWR(i2c_base, i2c->GPIO_CONTROL, gpioreg);
        // Power on device while SDA low to set ADDR bit to 1
        usleep(100);
        gpioreg |=(1<<devicepin);
        IOWR(i2c_base, i2c->GPIO_CONTROL, gpioreg);
        usleep(2000);                     // At least during 200us
        ROS_INFO("Activating 'El cacharro' %d (SDA HIGH) "BYTE_TO_BINARY_PATTERN, devicepin, BYTE_TO_BINARY(gpioreg));
        // Release SDA line again
        gpioreg &= ~(1UL << 4);
        IOWR(i2c_base, i2c->GPIO_CONTROL, gpioreg);
    }else{
        // take control of the SDA line
        gpioreg |= 1UL << 3;
        IOWR(i2c_base, i2c->GPIO_CONTROL, gpioreg);
        // Power on device while SDA low to set ADDR bit to 0
        usleep(1);
        gpioreg|=(1<<devicepin);
        IOWR(i2c_base, i2c->GPIO_CONTROL, gpioreg);
        usleep(2000);                     // At least during 200us
        ROS_INFO("Activating 'El cacharro' %d (SDA LOW) "BYTE_TO_BINARY_PATTERN, devicepin, BYTE_TO_BINARY(gpioreg));
        // Release SDA line again
        gpioreg &= ~(1UL << 3);
        ROS_DEBUG("gpio: " BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(gpioreg));
        IOWR(i2c_base, i2c->GPIO_CONTROL, gpioreg);
    }

    ROS_DEBUG("Checking defaultaddr");
    vector<uint8_t> data;
    i2c->read(setaddr, 1, 1, data);
    if (!IORD(i2c_base, i2c->ACK_ERROR)) {
        ROS_INFO("TLV sensor active at: %x", setaddr);
    }else{
        ROS_ERROR("sensor does not respond on address: %x", setaddr);
    }

    vector<uint8_t> regdata;
    readAllRegisters(setaddr,regdata,false);
//
    // Begin config
    // Static initial config for now
    uint32_t cfgdata = 0;
    cfgdata |=  ((((0b010<<5)|(regdata[9]&0b11111))<<0)|(regdata[8]<<8)|(((IICAddr<<5)|((regdata[7]&0b00011000)|0b010))<<16));  // Last 3 bits: INT/FAST/LP
    ROS_DEBUG("config 0\t: " BYTE_TO_BINARY_PATTERN" \t%x\n",BYTE_TO_BINARY(cfgdata&0xff), cfgdata&0xff);
    ROS_DEBUG("config 1\t: " BYTE_TO_BINARY_PATTERN" \t%x\n",BYTE_TO_BINARY(((cfgdata>>8)&0xff)), (cfgdata>>8)&0xff);
    ROS_DEBUG("config 2\t: " BYTE_TO_BINARY_PATTERN" \t%x\n",BYTE_TO_BINARY(((cfgdata>>16)&0xff)), (cfgdata>>16)&0xff);
    ROS_DEBUG("config 3\t: " BYTE_TO_BINARY_PATTERN" \t%x\n",BYTE_TO_BINARY(((cfgdata>>24)&0xff)), (cfgdata>>24)&0xff);
//
////    // First 3 bits: Enable temp/Low power interval/Parity test
////
////    // Calculate parity bit     (well doesen't work for now so fuck it)
////    // bool parity = bitRead(cfgdata[0]+cfgdata[1]+cfgdata[2],0);
////    // printf("Setting parity bit to ");
////    // printfln(parity,BIN);
////    // bitWrite(cfgdata[0],7,parity);
////
//    // Write config
    ROS_DEBUG("Writing config now ...");
    i2c->write(setaddr, cfgdata, 4);

    ROS_DEBUG("Reading config now ...");
    regdata.clear();
    readAllRegisters(deviceaddress,regdata,false);

    return true;
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