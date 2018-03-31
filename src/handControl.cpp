#include "roboy_plexus/handControl.hpp"

HandControl::HandControl(int32_t *i2c_base, vector<uint8_t> deviceIDs):deviceIDs(deviceIDs){
    i2c = new I2C(i2c_base);
    for(auto device:deviceIDs
            ){
        i2c->write(device,255,1);
        if(!i2c->ack_error())
            ROS_INFO("arm board with deviceID %x is active", device);
        else
            ROS_WARN("arm board with deviceID %x is not active", device);
    }
}

bool HandControl::command(vector<uint8_t> &setPoint){
    int j=0;
    vector<HandControl::CommandFrame> commands;
    for(int i=0;i<setPoint.size()/5;i++){
        HandControl::CommandFrame frame;
        frame.angleCommand[0] = setPoint[j];
        frame.angleCommand[1] = setPoint[j+1];
        frame.angleCommand[2] = setPoint[j+2];
        frame.angleCommand[3] = setPoint[j+3];
        frame.angleCommand[4] = setPoint[j+4];
        commands.push_back(frame);
        j+=5;
    }
    return write(commands);
}

bool HandControl::readSensorData(vector<SensorFrame> &sensor_data){
    bool ack_error = false;
    int i = 0;
    sensor_data.resize(deviceIDs.size());
    for(auto device:deviceIDs){
        vector<uint8_t> data;
        i2c->write(device,0,1);
        i2c->read(device,0,10,data);
        i2c->write(device,1,1);
        i2c->read(device,0,12,data);
        i2c->write(device,2,1);
        i2c->read(device,0,12,data);
        sensor_data[i].current[0] = (uint16_t)(data[1]<<8|data[0]);
        sensor_data[i].current[1] = (uint16_t)(data[3]<<8|data[2]);
        sensor_data[i].current[2] = (uint16_t)(data[5]<<8|data[4]);
        sensor_data[i].current[3] = (uint16_t)(data[7]<<8|data[6]);
        sensor_data[i].current[4] = (uint16_t)(data[9]<<8|data[8]);

        sensor_data[i].gyro[0] = ((int32_t)(data[13]<<24|data[12]<<16|data[11]<<8|data[10])* 250.0f) / 32768.0f;
        sensor_data[i].gyro[1] = ((int32_t)(data[17]<<24|data[16]<<16|data[15]<<8|data[14])* 250.0f) / 32768.0f;
        sensor_data[i].gyro[2] = ((int32_t)(data[21]<<24|data[20]<<16|data[19]<<8|data[18])* 250.0f) / 32768.0f;
        sensor_data[i].acc[0] = (int32_t)(data[25]<<24|data[24]<<16|data[23]<<8|data[22]);
        sensor_data[i].acc[1] = (int32_t)(data[29]<<24|data[28]<<16|data[27]<<8|data[26]);
        sensor_data[i].acc[2] = (int32_t)(data[33]<<24|data[32]<<16|data[31]<<8|data[30]);
        if(i2c->ack_error())
            ack_error = true;
    }
    return ack_error;
}

bool HandControl::write(vector<CommandFrame> &command){
    int i=0;
    bool ack_error = false;
    for(auto device:deviceIDs){
        i2c->write(device,(1<<24|command[i].data[2]<<16|command[i].data[1]<<8|command[i].data[0]),4);
        i2c->write(device,(2<<24|command[i].data[5]<<16|command[i].data[4]<<8|command[i].data[3]),4);
        i2c->write(device,(3<<24|command[i].data[8]<<16|command[i].data[7]<<8|command[i].data[6]),4);
        i2c->write(device,(4<<24|0<<16|command[i].data[10]<<8|command[i].data[9]),4);
        i++;
        if(i2c->ack_error())
            ack_error = true;
    }
    return ack_error;
}

void HandControl::test(){
    for(int j=0;j<3;j++) {
        for (uint8_t pos = 30; pos < 150; pos++) {
            vector<HandControl::SensorFrame> sensor_data;
            readSensorData(sensor_data);
            ROS_INFO("\ncurrent: %d\t%d\t%d\t%d\t%d\n gyro: %f\t%f\t%f\n acc: %f\t%f\t%f",sensor_data.back().current[0],sensor_data.back().current[1],
                     sensor_data.back().current[2],sensor_data.back().current[3],sensor_data.back().current[4],
                     sensor_data.back().gyro[0],sensor_data.back().gyro[1],sensor_data.back().gyro[2],
                     sensor_data.back().acc[0],sensor_data.back().acc[1],sensor_data.back().acc[2]);

            vector<uint8_t> setPoint = {pos, pos, pos, pos, pos};
            command(setPoint);

            usleep(10000);
        }

        for (uint8_t pos = 150; pos > 30; pos--) {
            vector<HandControl::SensorFrame> sensor_data;
            readSensorData(sensor_data);
            ROS_INFO("\ncurrent: %d\t%d\t%d\t%d\t%d\n gyro: %f\t%f\t%f\n acc: %f\t%f\t%f",sensor_data.back().current[0],sensor_data.back().current[1],
                     sensor_data.back().current[2],sensor_data.back().current[3],sensor_data.back().current[4],
                     sensor_data.back().gyro[0],sensor_data.back().gyro[1],sensor_data.back().gyro[2],
                     sensor_data.back().acc[0],sensor_data.back().acc[1],sensor_data.back().acc[2]);

            vector<uint8_t> setPoint = {pos, pos, pos, pos, pos};
            command(setPoint);
            usleep(10000);
        }
    }
}