#include "roboy_plexus/handControl.hpp"

HandControl::HandControl(int32_t *i2c_base, vector<uint8_t> deviceIDs):deviceIDs(deviceIDs){

    i2c = new I2C(i2c_base);
    int board = 0;
    for(auto device:deviceIDs
            ){
        i2c->write(device,255<<24,1);
        if(!i2c->ack_error()) {
            ROS_INFO("arm board with deviceID %x is active", device);
            vector<uint8_t> pos = {90,90,90,90,90};
            command(pos,board);
            board++;
        }else
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

bool HandControl::command(vector<uint8_t> &setPoint, int board){
    int j=0;
    HandControl::CommandFrame frame;
    frame.angleCommand[0] = setPoint[j];
    frame.angleCommand[1] = setPoint[j+1];
    frame.angleCommand[2] = setPoint[j+2];
    frame.angleCommand[3] = setPoint[j+3];
    frame.angleCommand[4] = setPoint[j+4];
    j+=5;
    return write(frame,board);
}

bool HandControl::readSensorData(vector<SensorFrame> &sensor_data){
    bool ack_error = false;
    int i = 0;
    sensor_data.resize(deviceIDs.size());
    for(auto device:deviceIDs){
        vector<uint8_t> data;
        i2c->read(device,0,12,data);
        i2c->read(device,1,12,data);
        i2c->read(device,2,12,data);
        sensor_data[i].current[0] = (uint16_t)(data[1]<<8|data[0]);
        sensor_data[i].current[1] = (uint16_t)(data[3]<<8|data[2]);
        sensor_data[i].current[2] = (uint16_t)(data[5]<<8|data[4]);
        sensor_data[i].current[3] = (uint16_t)(data[7]<<8|data[6]);
        sensor_data[i].current[4] = (uint16_t)(data[9]<<8|data[8]);
        sensor_data[i].current[5] = (uint16_t)(data[11]<<8|data[10]);

        sensor_data[i].gyro[0] = ((int32_t)(data[15]<<24|data[14]<<16|data[13]<<8|data[12])*250.0f) / 32768.0f;
        sensor_data[i].gyro[1] = ((int32_t)(data[19]<<24|data[18]<<16|data[17]<<8|data[16])*250.0f) / 32768.0f;
        sensor_data[i].gyro[2] = ((int32_t)(data[23]<<24|data[22]<<16|data[21]<<8|data[20])*250.0f) / 32768.0f;
        sensor_data[i].acc[0] = ((int32_t)(data[27]<<24|data[26]<<16|data[25]<<8|data[24])*2.0f) / 32768.0f;
        sensor_data[i].acc[1] = ((int32_t)(data[31]<<24|data[30]<<16|data[29]<<8|data[28])*2.0f) / 32768.0f;
        sensor_data[i].acc[2] = ((int32_t)(data[35]<<24|data[34]<<16|data[33]<<8|data[32])*2.0f) / 32768.0f;
        if(i2c->ack_error())
            ack_error = true;
        i++;
    }
    return !ack_error;
}

bool HandControl::readSensorData(SensorFrame &sensor_data, int board){
    bool ack_error = false;
    if(board>deviceIDs.size()||board<0) {
        ROS_ERROR("invalid board, only configured for %ld boards", deviceIDs.size());
        return false;
    }
    vector<uint8_t> data;
    i2c->read(deviceIDs[board],0,12,data);
    i2c->read(deviceIDs[board],1,12,data);
    i2c->read(deviceIDs[board],2,12,data);
    sensor_data.current[0] = (uint16_t)(data[1]<<8|data[0]);
    sensor_data.current[1] = (uint16_t)(data[3]<<8|data[2]);
    sensor_data.current[2] = (uint16_t)(data[5]<<8|data[4]);
    sensor_data.current[3] = (uint16_t)(data[7]<<8|data[6]);
    sensor_data.current[4] = (uint16_t)(data[9]<<8|data[8]);
    sensor_data.current[5] = (uint16_t)(data[11]<<8|data[10]);

    sensor_data.gyro[0] = ((int32_t)(data[15]<<24|data[14]<<16|data[13]<<8|data[12])*250.0f) / 32768.0f;
    sensor_data.gyro[1] = ((int32_t)(data[19]<<24|data[18]<<16|data[17]<<8|data[16])*250.0f) / 32768.0f;
    sensor_data.gyro[2] = ((int32_t)(data[23]<<24|data[22]<<16|data[21]<<8|data[20])*250.0f) / 32768.0f;
    sensor_data.acc[0] = ((int32_t)(data[27]<<24|data[26]<<16|data[25]<<8|data[24])*2.0f) / 32768.0f;
    sensor_data.acc[1] = ((int32_t)(data[31]<<24|data[30]<<16|data[29]<<8|data[28])*2.0f) / 32768.0f;
    sensor_data.acc[2] = ((int32_t)(data[35]<<24|data[34]<<16|data[33]<<8|data[32])*2.0f) / 32768.0f;
    if(i2c->ack_error())
        ack_error = true;
    return !ack_error;
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
    return !ack_error;
}

bool HandControl::write(CommandFrame &command, int board){
    int i=0;
    bool ack_error = false;
    if(board>deviceIDs.size()||board<0) {
        ROS_ERROR("invalid board, only configured for %ld boards", deviceIDs.size());
        return false;
    }
    i2c->write(deviceIDs[board],(1<<24|command.data[2]<<16|command.data[1]<<8|command.data[0]),4);
    i2c->write(deviceIDs[board],(2<<24|command.data[5]<<16|command.data[4]<<8|command.data[3]),4);
    i2c->write(deviceIDs[board],(3<<24|command.data[8]<<16|command.data[7]<<8|command.data[6]),4);
    i2c->write(deviceIDs[board],(4<<24|0<<16|command.data[10]<<8|command.data[9]),4);
    i++;
    if(i2c->ack_error())
        ack_error = true;
    return !ack_error;
}

void HandControl::test(){
    vector<bool> success(deviceIDs.size(),true);
    for(int board=0;board<deviceIDs.size();board++) {
        for(int motor = 0; motor<5; motor++) {
            for (uint8_t pos = 10; pos < 170; pos+=5) {
                vector<uint8_t> setPoint = {150, 150, 150, 150, 150};
                setPoint[motor] = pos;

                HandControl::SensorFrame sensor_data;
                success[board] = readSensorData(sensor_data, board) && command(setPoint, board);
                if (!success[board])
                    continue;
                else {
                    ROS_INFO("board %d motor %d current: %d", board, motor, sensor_data.current[motor]);
                    ROS_INFO_THROTTLE(1, "\ncurrent: %d\t%d\t%d\t%d\t%d\t%d\n gyro: %f\t%f\t%f\n acc: %f\t%f\t%f",
                                      sensor_data.current[0], sensor_data.current[1],
                                      sensor_data.current[2], sensor_data.current[3], sensor_data.current[4],
                                      sensor_data.current[5],
                                      sensor_data.gyro[0], sensor_data.gyro[1], sensor_data.gyro[2],
                                      sensor_data.acc[0], sensor_data.acc[1], sensor_data.acc[2]);
                }
                usleep(10000);
            }
        }
    }
    for(int board=0;board<deviceIDs.size();board++) {
        if(!success[board])
            ROS_ERROR("test FAILED for arm board %d with deviceID %x, check cableing!", board, deviceIDs[board]);
        else
            ROS_INFO("test SUCCESS for arm board %d with deviceID %x!", board, deviceIDs[board]);
    }
}