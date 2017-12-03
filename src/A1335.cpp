#include "roboy_plexus/A1335.hpp"

A1335::A1335(int32_t* i2c_base, vector<int32_t> &deviceIDs):deviceIDs(deviceIDs){
    i2c = new I2C(i2c_base);
    for(auto device:deviceIDs){
        clearStatusRegisters(device);
    }
}

A1335::~A1335(){
    delete i2c;
};

bool A1335::readAngleData(vector<A1335State> &states){
    bool deviceActive = false;
    for(auto device:deviceIDs){
        A1335State state;
        if(readDeviceState(device, &state)){
            ROS_DEBUG_STREAM_THROTTLE(1,"motor sensor " << device << " is active");
            deviceActive = true; // at least one is active
        }else{
            ROS_WARN_STREAM_THROTTLE(1,"motor sensor " << device << " is NOT active, check cables (did you use a level shifter?!)");
        }
        states.push_back(state);
    }
    return deviceActive;
}

string A1335::decodeFlag(int type, uint16_t code){
    int i = 0;
    str.clear();
    switch(type){
        case ANGLES_FLAGS:
            for(auto flag:angle_flags){
                if((code >> i) & 1){
                    str.append(flag);
                }
            }
            break;
        case STATUS_FLAGS:
            for(auto flag:status_flags){
                if((code >> i) & 1){
                    str.append(flag);
                }
            }
            break;
        case ERROR_FLAGS:
            for(auto flag:error_flags){
                if((code >> i) & 1){
                    str.append(flag);
                }
            }
            break;
        case XERROR_FLAGS:
            for(auto flag:xerror_flags){
                if((code >> i) & 1){
                    str.append(flag);
                }
            }
            break;
        default: return str;
    }
    return str;
}

bool A1335::writeMemoryCheck(uint8_t deviceaddress, uint8_t eeaddress, uint8_t* wdata) {
    i2c->write(deviceaddress,(uint32_t)(eeaddress<<24|wdata[0]<<16|wdata[1]<<8),3);
    vector<uint8_t> rdata;
    i2c->read(deviceaddress,eeaddress,2,rdata);
    return (rdata[0] == wdata[0] && rdata[1] == wdata[1]);
}

bool A1335::clearStatusRegisters(uint8_t deviceaddress){
    uint8_t data[] = {
            0b00000111, // Clear STA, ERR & XERR
            0x46        // Keycode to activate this command
    };
    i2c->write(deviceaddress, (uint32_t)(0x1E<<24|data[0]<<16|data[1]<<8), 3);
    return !i2c->ack_error();
}

bool A1335::checkDefaultSettings(A1335State* state){
    for(uint8_t i = 0; i < 8; i++){
        if((state->rawData[i][0] & expected_registers_mask[i][0]) != expected_registers[i][0])
            return false;
        if((state->rawData[i][1] & expected_registers_mask[i][1]) != expected_registers[i][1])
            return false;
    }
    return true;
}


bool A1335::readDeviceState(uint8_t deviceaddress, A1335State* state){
    vector<uint8_t> data;
    i2c->read(deviceaddress, start_register, num_registers*2, data);
    if(i2c->ack_error())
        return false;
    for(uint8_t i = 0; i < num_registers; i++){
        state->rawData[i][0] = data[i*2];
        state->rawData[i][1] = data[i*2+1];
    }
    data.clear();
    i2c->read(deviceaddress, start_register2, num_registers*2, data);
    if(i2c->ack_error())
        return false;
    for(uint8_t i = 0; i < num_registers2; i++){
        state->rawData[num_registers+i][0] = data[i*2];
        state->rawData[num_registers+i][1] = data[i*2+1];
    }

    state->address = deviceaddress;

    state->isOK = checkDefaultSettings(state);

    state->angle = (float)((uint16_t)((state->rawData[0][0] & 0xf) << 8) | state->rawData[0][1]) * 360 / 4096 ;
    state->angle_flags = (state->rawData[0][0] >> 5) & 0b11;

    state->status_flags = state->rawData[1][0] & 0xf;

    state->err_flags = ((state->rawData[2][0] & 0xf) << 8) | state->rawData[2][1];
    state->xerr_flags = ((state->rawData[3][0] & 0xf) << 8) | state->rawData[3][1];

    state->temp =
            (float)((uint16_t)((state->rawData[4][0] & 0xf) << 8) | state->rawData[4][1])
            / 8.0 - 273.145; // 8th Kelvin to °C
    state->fieldStrength =
            (float)((uint16_t)((state->rawData[5][0] & 0xf) << 8) | state->rawData[5][1])
            / 10.0; // Gauss to milliTesla


    return true;
}

bool A1335::searchAddressSpace(){
    bool deviceActive = false;
    for(uint8_t i=0xC; i < 0xD; i++){ // search all possible addresses
        A1335State state;
        if(readDeviceState(i, &state)){
//            cout<< i << " is active" << endl;
            deviceActive = true;
        }
    }
    return deviceActive;
}