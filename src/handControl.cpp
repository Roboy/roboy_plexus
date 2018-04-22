#include "roboy_plexus/handControl.hpp"

HandControl::HandControl(int32_t *i2c_base, vector<uint8_t> deviceIDs, bool id):deviceIDs(deviceIDs), id(id){

    if(!id)
        hand = "left";
    else
        hand = "right";

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, ("handControl_"+hand).c_str());
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    i2c = new I2C(i2c_base);
    for(auto device:deviceIDs){
        i2c->write(device,255<<24,1);
        if(!i2c->ack_error()) {
            ROS_INFO("arm board with deviceID %x is active", device);
        }else
            ROS_WARN("arm board with deviceID %x is not active", device);
    }
    neutralHand();

    handCommand_sub = nh->subscribe("/roboy/middleware/HandCommand", 1, &HandControl::handCommandCB, this);
    handStatus_pub = nh->advertise<roboy_communication_middleware::HandStatus>("/roboy/middleware/HandStatus",1);
    fingerCommand_sub = nh->subscribe("/roboy/middleware/FingerCommand", 1, &HandControl::fingerCommandCB, this);
    setMode_srv = nh->advertiseService("/roboy/control/hand/" + hand,
                                       &HandControl::setHandModeService, this);
    handIMUThread = boost::shared_ptr<std::thread>(
            new std::thread(&HandControl::handStatusPublisher, this));
    handIMUThread->detach();
}

HandControl::~HandControl(){
    keep_publishing = false;
    if(handIMUThread->joinable()){
        ROS_INFO("waiting for IMU thread to terminate");
        handIMUThread->join();
    }
}

void HandControl::handCommandCB(const roboy_communication_middleware::HandCommand::ConstPtr &msg) {
    if(msg->id == id) {
        vector <uint8_t> setPoint;
        stringstream str;
        str << "hand " << (id?"right":"left") << " command:";
        if (msg->motorid.empty()) {
            for (auto s:msg->setPoint) {
                if (s < 20)
                    setPoint.push_back(20);
                if (s > 160)
                    setPoint.push_back(160);
                if (s >= 20 && s <= 160)
                    setPoint.push_back(s);
                str << (int) setPoint.back() << "\t";
                ROS_INFO_STREAM(str.str());

            }
            command(setPoint);
        }else{
            if(msg->motorid.size()==msg->setPoint.size()) {
                int i = 0;
                bool update[] = {false,false,false,false};
                for (auto m:msg->motorid) {
                    frame[m / 5].angleCommand[m % 5] = msg->setPoint[i];
                    str << "motor " << (int) m << ": " << (int) msg->setPoint[i] << "\t";
                    update[m / 5] = true;
                    i++;
                }
                ROS_INFO_STREAM(str.str());
                for(int i=0;i<4;i++)
                    if(update[i])
                        write(frame[i],i);
            }else{
                ROS_ERROR("motorid and setpoint must have the same size!!!");
            }
        }
    }
}

void HandControl::fingerCommandCB(const roboy_communication_middleware::FingerCommand::ConstPtr &msg){
    if(msg->id == id){
        if(msg->finger>3){
            ROS_ERROR("invalid finger, use THUMB(0), INDEXFINGER(1), MIDDLEFINGER(2), RINGLITTLE(3)");
            return;
        }
        fingerControl(msg->finger, msg->angles[0], msg->angles[1], msg->angles[2], msg->angles[3]);
    }
}

bool HandControl::setHandModeService(roboy_communication_control::SetModeRequest &req,
                            roboy_communication_control::SetModeResponse &res) {

    if (req.id!=id) {
        return 0;
    }
    switch (req.mode) {
        case CLOSE:
            closeHand();
            break;
        case OPEN:
            openHand();
            break;
        default:
            ROS_INFO_STREAM("Unknown id for hand mode");

    }

}

void HandControl::closeHand() {
    vector<uint8_t > order = {RINGLITTLEFINGER, MIDDLEFINGER, INDEXFINGER, THUMB};
    ros::Duration d(1);
    for (auto finger: order) {
        fingerControl(finger, 90, 110, 90, 90);
        d.sleep();
    }
}

void HandControl::openHand() {
    vector<uint8_t > order = {THUMB, INDEXFINGER, MIDDLEFINGER, RINGLITTLEFINGER};
    ros::Duration d(1);
    for (auto finger: order) {
        fingerControl(finger, 0, 0, 0, 90);
        d.sleep();
    }
}

void HandControl::handStatusPublisher(){
    ros::Rate rate(2);
    while (keep_publishing) {
        roboy_communication_middleware::HandStatus msg;
        msg.id = id;
        vector<HandControl::SensorFrame> sensor_data;
        readSensorData(sensor_data);
        for(int arm_board=0;arm_board<sensor_data.size();arm_board++){
            msg.current.push_back(sensor_data[arm_board].current[0]);
            msg.current.push_back(sensor_data[arm_board].current[1]);
            msg.current.push_back(sensor_data[arm_board].current[2]);
            msg.current.push_back(sensor_data[arm_board].current[3]);
            msg.current.push_back(sensor_data[arm_board].current[4]);
            msg.current.push_back(sensor_data[arm_board].current[5]);
            msg.gyro_x.push_back(sensor_data[arm_board].gyro[0]);
            msg.gyro_y.push_back(sensor_data[arm_board].gyro[1]);
            msg.gyro_z.push_back(sensor_data[arm_board].gyro[2]);
            msg.acc_x.push_back(sensor_data[arm_board].acc[0]);
            msg.acc_y.push_back(sensor_data[arm_board].acc[1]);
            msg.acc_z.push_back(sensor_data[arm_board].acc[2]);
        }
        handStatus_pub.publish(msg);
        rate.sleep();
    }
}

bool HandControl::fingerControl(uint8_t finger, uint8_t alpha, uint8_t beta, uint8_t gamma, uint8_t zeta){
    float r_a = 1.43f, r_b = 0.9f, r_c = 0.83f;
    uint8_t motor[4];
    motor[3] = (alpha * r_a) + (beta * r_b) + (gamma * r_c);
    motor[2] = (-beta * r_b) + (gamma * r_c);
    motor[1] = 160 - gamma * r_c;
    motor[0] = zeta;

    // limit check
    for(uint i=0;i<4;i++){
        if(motor[i]<20)
            motor[i] = 20;
        if(motor[i]>160)
            motor[i] = 160;
    }

    switch(finger){
        case THUMB:
            frame[1].angleCommand[3] = motor[3];
            frame[1].angleCommand[2] = motor[2];
            frame[1].angleCommand[1] = motor[1];
            frame[1].angleCommand[0] = motor[0];
            ROS_INFO("thumb command: %d %d %d %d", motor[0],motor[1],motor[2],motor[3]);
            return write(frame[1],1);
        case INDEXFINGER:
            frame[2].angleCommand[3] = motor[3];
            frame[2].angleCommand[2] = motor[2];
            frame[2].angleCommand[1] = motor[1];
            frame[2].angleCommand[0] = motor[0];
            ROS_INFO("index finger command: %d %d %d %d", motor[0],motor[1],motor[2],motor[3]);
            return write(frame[2],2);
        case MIDDLEFINGER:
            frame[3].angleCommand[3] = motor[3];
            frame[3].angleCommand[2] = motor[2];
            frame[3].angleCommand[1] = motor[1];
            frame[3].angleCommand[0] = motor[0];
            ROS_INFO("middle finger command: %d %d %d %d", motor[0],motor[1],motor[2],motor[3]);
            return write(frame[3],3);
        case RINGLITTLEFINGER:
            frame[0].angleCommand[3] = motor[3];
            frame[0].angleCommand[2] = motor[2];
            frame[0].angleCommand[1] = motor[1];
            frame[0].angleCommand[0] = motor[0];
            ROS_INFO("ring little finger command: %d %d %d %d", motor[0],motor[1],motor[2],motor[3]);
            return write(frame[0],0);
    }
}

void HandControl::neutralHand(){
    vector<uint8_t> pos = {90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90};
    command(pos);
}

bool HandControl::command(vector<uint8_t> &setPoint){
    int j=0;
    vector<HandControl::CommandFrame> commands;
    for(int i=0;i<setPoint.size()/5;i++){
        frame[i].angleCommand[0] = setPoint[j];
        frame[i].angleCommand[1] = setPoint[j+1];
        frame[i].angleCommand[2] = setPoint[j+2];
        frame[i].angleCommand[3] = setPoint[j+3];
        frame[i].angleCommand[4] = setPoint[j+4];
        commands.push_back(frame[i]);
        j+=5;
    }
    return write(commands);
}

bool HandControl::command(vector<uint8_t> &setPoint, int board){
    int j=0;
    frame[board].angleCommand[0] = setPoint[j];
    frame[board].angleCommand[1] = setPoint[j+1];
    frame[board].angleCommand[2] = setPoint[j+2];
    frame[board].angleCommand[3] = setPoint[j+3];
    frame[board].angleCommand[4] = setPoint[j+4];
    j+=5;
    return write(frame[board],board);
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