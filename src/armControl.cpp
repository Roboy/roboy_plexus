#include "roboy_plexus/armControl.hpp"

ArmControl::ArmControl(int32_t *myo_base, uint8_t elbowDeviceID, uint8_t wristDeviceID, vector<uint8_t> handDeviceIDs,
                         bool id, bool hand_control_active , bool elbow_joint_controller_active ,
                       bool wrist_joint_controller_active ) :
        myo_base(myo_base), handDeviceIDs(handDeviceIDs), wristDeviceID(wristDeviceID), id(id), hand_control_active(hand_control_active),
        elbow_joint_controller_active(elbow_joint_controller_active), wrist_joint_controller_active(wrist_joint_controller_active){

    if (!id) {
        hand = "left";
    } else {
        hand = "right";
    }
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, ("armControl_" + hand).c_str());
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    if (elbow_joint_controller_active) {
        MYO_WRITE_elbow_Kp_joint_angle(myo_base, 10);
        MYO_WRITE_elbow_Kd_joint_angle(myo_base, -5);
        MYO_WRITE_elbow_agonist(myo_base, elbow_agonist);
        MYO_WRITE_elbow_antagonist(myo_base, elbow_antagonist);
        MYO_WRITE_Kp(myo_base, elbow_agonist, 30);
        MYO_WRITE_outputDivider(myo_base, elbow_agonist, 100);
        MYO_WRITE_Kp(myo_base, elbow_antagonist, 30);
        MYO_WRITE_outputDivider(myo_base, elbow_antagonist, 100);
        MYO_WRITE_Kd(myo_base, elbow_agonist, 0);
        MYO_WRITE_Kd(myo_base, elbow_antagonist, 0);
        MYO_WRITE_elbow_joint_angle_offset(myo_base, -700);
        MYO_WRITE_elbow_joint_pretension(myo_base, 30);
        MYO_WRITE_elbow_joint_deadband(myo_base, 100);
        MYO_WRITE_elbow_joint_angle_setpoint(myo_base, 0);
        MYO_WRITE_elbow_joint_angle_device_id(myo_base, elbowDeviceID);
        MYO_WRITE_elbow_joint_control(myo_base, elbow_joint_controller_active);

        ROS_INFO("Configuring elbow joint with"
                         "\nKp %d\nKd %d\nagonist %d"
                         "\nantagonist %d\njoint_angle_offset %d"
                         "\njoint_angle_setpoint %d\njoint_angle_device_id %x",
                 MYO_READ_elbow_Kp_joint_angle(myo_base), MYO_READ_elbow_Kd_joint_angle(myo_base),
                 MYO_READ_elbow_antagonist(myo_base), MYO_READ_elbow_agonist(myo_base),
                 MYO_READ_elbow_joint_angle_offset(myo_base), MYO_READ_elbow_joint_angle_setpoint(myo_base),
                 MYO_READ_elbow_joint_angle_device_id(myo_base));
    }
    if (wrist_joint_controller_active) {
        MYO_WRITE_wrist_Kp_joint_angle(myo_base, 10);
        MYO_WRITE_wrist_Kd_joint_angle(myo_base, 5);
        MYO_WRITE_wrist_agonist(myo_base, wrist_agonist);
        MYO_WRITE_wrist_antagonist(myo_base, wrist_antagonist);
        MYO_WRITE_Kp(myo_base, wrist_agonist, 30);
        MYO_WRITE_outputDivider(myo_base, wrist_agonist, 100);
//        MYO_WRITE_outputPosMax(myo_base, wrist_agonist, 100);
//        MYO_WRITE_outputNegMax(myo_base, wrist_agonist, -100);
        MYO_WRITE_Kp(myo_base, wrist_antagonist, 30);
        MYO_WRITE_outputDivider(myo_base, wrist_antagonist, 100);
//        MYO_WRITE_outputPosMax(myo_base, wrist_antagonist, 100);
//        MYO_WRITE_outputNegMax(myo_base, wrist_antagonist, -100);
        MYO_WRITE_Kd(myo_base, wrist_agonist, 0);
        MYO_WRITE_Kd(myo_base, wrist_antagonist, 0);
        MYO_WRITE_wrist_joint_angle_offset(myo_base, -1274);
        MYO_WRITE_wrist_joint_pretension(myo_base, 0);
        MYO_WRITE_wrist_joint_deadband(myo_base, 100);
        MYO_WRITE_wrist_joint_angle_setpoint(myo_base, 0);
        MYO_WRITE_wrist_joint_angle_device_id(myo_base, wristDeviceID);
        MYO_WRITE_wrist_joint_control(myo_base, wrist_joint_controller_active);

        ROS_INFO("Configuring wrist joint with"
                         "\nKp %d\nKd %d\nagonist %d"
                         "\nantagonist %d\njoint_angle_offset %d"
                         "\njoint_angle_setpoint %d\njoint_angle_device_id %x",
                 MYO_READ_wrist_Kp_joint_angle(myo_base), MYO_READ_wrist_Kd_joint_angle(myo_base),
                 MYO_READ_wrist_antagonist(myo_base), MYO_READ_wrist_agonist(myo_base),
                 MYO_READ_wrist_joint_angle_offset(myo_base), MYO_READ_wrist_joint_angle_setpoint(myo_base),
                 MYO_READ_wrist_joint_angle_device_id(myo_base));
    }
    if (hand_control_active) {

        MYO_WRITE_arm_board_device_id(myo_base, 0, 0x50);
        MYO_WRITE_arm_board_device_id(myo_base, 1, 0x51);
        MYO_WRITE_arm_board_device_id(myo_base, 2, 0x52);
        MYO_WRITE_arm_board_device_id(myo_base, 3, 0x53);
        for (int board = 0; board < 4; board++) {
            MYO_WRITE_motor0(myo_base, board, 180);
            MYO_WRITE_motor1(myo_base, board, 180);
            MYO_WRITE_motor2(myo_base, board, 180);
            MYO_WRITE_motor3(myo_base, board, 180);
            MYO_WRITE_motor4(myo_base, board, 180);
        }
        MYO_WRITE_hand_control(myo_base, hand_control_active);

        ROS_INFO("Enableing hand control with"
                         "\nboard 0: %x setpoints %d %d %d %d %d"
                         "\nboard 1: %x setpoints %d %d %d %d %d"
                         "\nboard 2: %x setpoints %d %d %d %d %d"
                         "\nboard 3: %x setpoints %d %d %d %d %d",
                 MYO_READ_arm_board_device_id(myo_base, 0),
                 MYO_READ_motor0(myo_base, 0), MYO_READ_motor1(myo_base, 0), MYO_READ_motor2(myo_base, 0),
                 MYO_READ_motor3(myo_base, 0), MYO_READ_motor4(myo_base, 0),
                 MYO_READ_arm_board_device_id(myo_base, 1),
                 MYO_READ_motor0(myo_base, 1), MYO_READ_motor1(myo_base, 1), MYO_READ_motor2(myo_base, 1),
                 MYO_READ_motor3(myo_base, 1), MYO_READ_motor4(myo_base, 1),
                 MYO_READ_arm_board_device_id(myo_base, 2),
                 MYO_READ_motor0(myo_base, 2), MYO_READ_motor1(myo_base, 2), MYO_READ_motor2(myo_base, 2),
                 MYO_READ_motor3(myo_base, 2), MYO_READ_motor4(myo_base, 2),
                 MYO_READ_arm_board_device_id(myo_base, 3),
                 MYO_READ_motor0(myo_base, 3), MYO_READ_motor1(myo_base, 3), MYO_READ_motor2(myo_base, 3),
                 MYO_READ_motor3(myo_base, 3), MYO_READ_motor4(myo_base, 3)
        );

        bool hand_control_enabled = MYO_READ_hand_control(myo_base);
        if(test() == false) {
            MYO_WRITE_hand_control(myo_base, false);
            int board_active = MYO_READ_arm_board_ack_error(myo_base);
            ROS_INFO(
                    "board %s\n"
                            "board %s\n"
                            "board %s\n"
                            "board %s",
                    (((board_active >> 0) && hand_control_enabled) ? "offline" : "online"),
                    (((board_active >> 1) && hand_control_enabled) ? "offline" : "online"),
                    (((board_active >> 2) && hand_control_enabled) ? "offline" : "online"),
                    (((board_active >> 3) && hand_control_enabled) ? "offline" : "online")
            );
        }else{
            openHand();
        }
    }

    handCommand_sub = nh->subscribe("/roboy/middleware/HandCommand", 1, &ArmControl::handCommandCB, this);
    armStatus_pub = nh->advertise<roboy_communication_middleware::ArmStatus>("/roboy/middleware/ArmStatus", 1);
    if (!id) {
        elbowCommand_sub = nh->subscribe("/roboy/middleware/elbow_left/JointAngle", 1, &ArmControl::elbowCommandCB,
                                         this);
        wristCommand_sub = nh->subscribe("/roboy/middleware/wrist_left/JointAngle", 1, &ArmControl::wristCommandCB,
                                         this);
        jointController_srv = nh->advertiseService("/roboy/middleware/elbow_left/JointController",
                                                   &ArmControl::JointControllerService, this);
    } else {
        elbowCommand_sub = nh->subscribe("/roboy/middleware/elbow_right/JointAngle", 1, &ArmControl::elbowCommandCB,
                                         this);
        wristCommand_sub = nh->subscribe("/roboy/middleware/wrist_right/JointAngle", 1, &ArmControl::wristCommandCB,
                                         this);
        jointController_srv = nh->advertiseService("/roboy/middleware/elbow_right/JointController",
                                                   &ArmControl::JointControllerService, this);
    }

    fingerCommand_sub = nh->subscribe("/roboy/middleware/FingerCommand", 1, &ArmControl::fingerCommandCB, this);
    setMode_srv = nh->advertiseService("/roboy/control/hand/" + hand,
                                       &ArmControl::setHandModeService, this);
    armStatusThread = boost::shared_ptr<std::thread>(
            new std::thread(&ArmControl::armStatusPublisher, this));
    armStatusThread->detach();
}

ArmControl::~ArmControl() {
    MYO_WRITE_hand_control(myo_base, 0);
    MYO_WRITE_elbow_joint_control(myo_base, 0);
    MYO_WRITE_wrist_joint_control(myo_base, 0);
    keep_publishing = false;
    if (armStatusThread->joinable()) {
        ROS_INFO("waiting for arm status thread to terminate");
        armStatusThread->join();
    }
}

void ArmControl::handCommandCB(const roboy_communication_middleware::HandCommand::ConstPtr &msg) {
    if (msg->id == id) {
        vector<uint8_t> setPoint;
        stringstream str;
        str << "hand " << (id ? "right" : "left") << " command:";
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
            ROS_INFO("%s", (command(setPoint) ? "success" : "failed"));
        } else {
            if (msg->motorid.size() == msg->setPoint.size()) {
                int i = 0;
                for (auto m:msg->motorid) {
                    switch(m%5){
                        case 0:
                            MYO_WRITE_motor0(myo_base,m/5,msg->setPoint[i]);
                            break;
                        case 1:
                            MYO_WRITE_motor1(myo_base,m/5,msg->setPoint[i]);
                            break;
                        case 2:
                            MYO_WRITE_motor2(myo_base,m/5,msg->setPoint[i]);
                            break;
                        case 3:
                            MYO_WRITE_motor3(myo_base,m/5,msg->setPoint[i]);
                            break;
                        case 4:
                            MYO_WRITE_motor4(myo_base,m/5,msg->setPoint[i]);
                            break;
                    }
                    str << "motor " << (int) m << ": " << (int) msg->setPoint[i] << "\t";
                    i++;
                }
                ROS_INFO_STREAM(str.str());
            } else {
                ROS_ERROR("motorid and setpoint must have the same size!!!");
            }
        }
    }
}

void ArmControl::elbowCommandCB(const std_msgs::Float32::ConstPtr &msg) {
    // convert to joint angle ticks
    int32_t encoderTicks = (msg->data * 4096 / 360.0);
    MYO_WRITE_elbow_joint_angle_setpoint(myo_base, encoderTicks);
}

void ArmControl::wristCommandCB(const std_msgs::Float32::ConstPtr &msg) {
    // convert to joint angle ticks
    int32_t encoderTicks = -(msg->data * 4096 / 360.0); // negative because angle is flipped
    MYO_WRITE_wrist_joint_angle_setpoint(myo_base, encoderTicks);
}

void ArmControl::fingerCommandCB(const roboy_communication_middleware::FingerCommand::ConstPtr &msg) {
    if (msg->id == id) {
        if (msg->finger > 3) {
            ROS_ERROR("invalid finger, use THUMB(0), INDEXFINGER(1), MIDDLEFINGER(2), RINGLITTLE(3)");
            return;
        }
        fingerControl(msg->finger, msg->angles[0], msg->angles[1], msg->angles[2], msg->angles[3]);
    }
}

bool ArmControl::setHandModeService(roboy_communication_control::SetModeRequest &req,
                                     roboy_communication_control::SetModeResponse &res) {

    if (req.id != id) {
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

bool ArmControl::JointControllerService(roboy_communication_middleware::JointControllerRequest &req,
                                         roboy_communication_middleware::JointControllerResponse &res) {
    MYO_WRITE_elbow_joint_control(myo_base,req.elbow_control_enable);
    MYO_WRITE_elbow_Kp_joint_angle(myo_base, req.Kp_elbow_joint);
    MYO_WRITE_elbow_Kd_joint_angle(myo_base, req.Kd_elbow_joint);
    MYO_WRITE_Kp(myo_base, elbow_agonist, req.Kp_elbow_agonist);
    MYO_WRITE_Kp(myo_base, elbow_antagonist, req.Kp_elbow_antagonist);
    MYO_WRITE_Kd(myo_base, elbow_agonist, req.Kd_elbow_agonist);
    MYO_WRITE_Kd(myo_base, elbow_antagonist, req.Kd_elbow_agonist);
    MYO_WRITE_elbow_joint_pretension(myo_base, req.elbow_pretension);
    MYO_WRITE_elbow_joint_deadband(myo_base, req.elbow_deadband);
    MYO_WRITE_wrist_joint_control(myo_base,req.wrist_control_enable);
    MYO_WRITE_wrist_Kp_joint_angle(myo_base, req.Kp_wrist_joint);
    MYO_WRITE_wrist_Kd_joint_angle(myo_base, req.Kd_wrist_joint);
    MYO_WRITE_Kp(myo_base, wrist_agonist, req.Kp_wrist_agonist);
    MYO_WRITE_Kp(myo_base, wrist_antagonist, req.Kp_wrist_antagonist);
    MYO_WRITE_Kd(myo_base, wrist_agonist, req.Kd_wrist_agonist);
    MYO_WRITE_Kd(myo_base, wrist_antagonist, req.Kd_wrist_agonist);
    MYO_WRITE_wrist_joint_pretension(myo_base, req.wrist_pretension);
    MYO_WRITE_wrist_joint_deadband(myo_base, req.wrist_deadband);
}

void ArmControl::closeHand() {
    vector<uint8_t> order = {RINGLITTLEFINGER, MIDDLEFINGER, INDEXFINGER, THUMB};
    for (auto finger: order) {
        MYO_WRITE_motor0(myo_base,finger,10);
        MYO_WRITE_motor1(myo_base,finger,160);
        MYO_WRITE_motor2(myo_base,finger,160);
        MYO_WRITE_motor3(myo_base,finger,160);
        MYO_WRITE_motor4(myo_base,finger,160);
    }
}

void ArmControl::openHand() {
    vector<uint8_t> order = {THUMB, INDEXFINGER, MIDDLEFINGER, RINGLITTLEFINGER};
    for (auto finger: order) {
        MYO_WRITE_motor0(myo_base,finger,160);
        MYO_WRITE_motor1(myo_base,finger,10);
        MYO_WRITE_motor2(myo_base,finger,10);
        MYO_WRITE_motor3(myo_base,finger,10);
        MYO_WRITE_motor4(myo_base,finger,10);
    }
}

void ArmControl::armStatusPublisher() {
    ros::Rate rate(30);
    while (keep_publishing) {
        roboy_communication_middleware::ArmStatus msg;
        msg.id = id;
        vector<ArmControl::SensorFrame> sensor_data;
        readSensorData(sensor_data);
        for (int arm_board = 0; arm_board < sensor_data.size(); arm_board++) {
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

        msg.elbow_joint_angle = 360.0 * MYO_READ_elbow_joint_angle(myo_base) / 4096.0;
        msg.elbow_joint_angle_setpoint = 360.0 * MYO_READ_elbow_joint_angle_setpoint(myo_base) / 4096.0;
        msg.elbow_agonist_setpoint = MYO_READ_sp(myo_base, elbow_agonist);
        msg.elbow_antagonist_setpoint = MYO_READ_sp(myo_base, elbow_antagonist);

        msg.wrist_joint_angle = -360.0 * MYO_READ_wrist_joint_angle(myo_base) / 4096.0; // negative because angle is flipped
        msg.wrist_joint_angle_setpoint = -360.0 * MYO_READ_wrist_joint_angle_setpoint(myo_base) / 4096.0;
        msg.wrist_agonist_setpoint = MYO_READ_sp(myo_base, wrist_agonist);
        msg.wrist_antagonist_setpoint = MYO_READ_sp(myo_base, wrist_antagonist);

        armStatus_pub.publish(msg);

        rate.sleep();
    }
}

bool ArmControl::fingerControl(uint8_t finger, uint8_t alpha, uint8_t beta, uint8_t gamma, uint8_t zeta) {
    float r_a = 1.43f, r_b = 0.9f, r_c = 0.83f;
    uint8_t motor[4];
    motor[3] = (alpha * r_a) + (beta * r_b) + (gamma * r_c);
    motor[2] = (-beta * r_b) + (gamma * r_c);
    motor[1] = 160 - gamma * r_c;
    motor[0] = zeta;

    // limit check
    for (uint i = 0; i < 4; i++) {
        if (motor[i] < 20)
            motor[i] = 20;
        if (motor[i] > 160)
            motor[i] = 160;
    }

    switch (finger) {
        case THUMB:
            MYO_WRITE_motor1(myo_base,0,motor[0]);
            MYO_WRITE_motor2(myo_base,0,motor[1]);
            MYO_WRITE_motor3(myo_base,0,motor[2]);
            MYO_WRITE_motor4(myo_base,0,motor[3]);
            ROS_INFO("thumb command: %d %d %d %d", motor[0], motor[1], motor[2], motor[3]);
        case INDEXFINGER:
            MYO_WRITE_motor1(myo_base,1,motor[0]);
            MYO_WRITE_motor2(myo_base,1,motor[1]);
            MYO_WRITE_motor3(myo_base,1,motor[2]);
            MYO_WRITE_motor4(myo_base,1,motor[3]);
            ROS_INFO("index finger command: %d %d %d %d", motor[0], motor[1], motor[2], motor[3]);
        case MIDDLEFINGER:
            MYO_WRITE_motor1(myo_base,2,motor[0]);
            MYO_WRITE_motor2(myo_base,2,motor[1]);
            MYO_WRITE_motor3(myo_base,2,motor[2]);
            MYO_WRITE_motor4(myo_base,2,motor[3]);
            ROS_INFO("middle finger command: %d %d %d %d", motor[0], motor[1], motor[2], motor[3]);
        case RINGLITTLEFINGER:
            MYO_WRITE_motor1(myo_base,3,motor[0]);
            MYO_WRITE_motor2(myo_base,3,motor[1]);
            MYO_WRITE_motor3(myo_base,3,motor[2]);
            MYO_WRITE_motor4(myo_base,3,motor[3]);
            ROS_INFO("ring little finger command: %d %d %d %d", motor[0], motor[1], motor[2], motor[3]);
        default: return false;
    }
    return true;
}

void ArmControl::neutralHand() {
    vector<uint8_t> pos = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
    command(pos);
}

bool ArmControl::command(vector<uint8_t> &setPoint) {
    int j = 0;
    for (int i = 0; i < setPoint.size()/5; i++) {
        MYO_WRITE_motor0(myo_base,i,setPoint[j]);
        MYO_WRITE_motor1(myo_base,i,setPoint[j]);
        MYO_WRITE_motor2(myo_base,i,setPoint[j]);
        MYO_WRITE_motor3(myo_base,i,setPoint[j]);
        MYO_WRITE_motor4(myo_base,i,setPoint[j]);
        j += 5;
    }
}

bool ArmControl::command(vector<uint8_t> &setPoint, int board) {
    if(setPoint.size()!=5)
            return false;
    MYO_WRITE_motor0(myo_base,board,setPoint[0]);
    MYO_WRITE_motor1(myo_base,board,setPoint[1]);
    MYO_WRITE_motor2(myo_base,board,setPoint[2]);
    MYO_WRITE_motor3(myo_base,board,setPoint[3]);
    MYO_WRITE_motor4(myo_base,board,setPoint[4]);
    return true;
}

bool ArmControl::readSensorData(vector<SensorFrame> &sensor_data) {
//    bool ack_error = false;
//    int i = 0;
//    sensor_data.resize(deviceIDs.size());
//    for(auto device:deviceIDs){
//        vector<uint8_t> data;
//        i2c->read(device,0,12,data);
//        i2c->read(device,1,12,data);
//        i2c->read(device,2,12,data);
//        sensor_data[i].current[0] = (uint16_t)(data[1]<<8|data[0]);
//        sensor_data[i].current[1] = (uint16_t)(data[3]<<8|data[2]);
//        sensor_data[i].current[2] = (uint16_t)(data[5]<<8|data[4]);
//        sensor_data[i].current[3] = (uint16_t)(data[7]<<8|data[6]);
//        sensor_data[i].current[4] = (uint16_t)(data[9]<<8|data[8]);
//        sensor_data[i].current[5] = (uint16_t)(data[11]<<8|data[10]);
//
//        sensor_data[i].gyro[0] = ((int32_t)(data[15]<<24|data[14]<<16|data[13]<<8|data[12])*250.0f) / 32768.0f;
//        sensor_data[i].gyro[1] = ((int32_t)(data[19]<<24|data[18]<<16|data[17]<<8|data[16])*250.0f) / 32768.0f;
//        sensor_data[i].gyro[2] = ((int32_t)(data[23]<<24|data[22]<<16|data[21]<<8|data[20])*250.0f) / 32768.0f;
//        sensor_data[i].acc[0] = ((int32_t)(data[27]<<24|data[26]<<16|data[25]<<8|data[24])*2.0f) / 32768.0f;
//        sensor_data[i].acc[1] = ((int32_t)(data[31]<<24|data[30]<<16|data[29]<<8|data[28])*2.0f) / 32768.0f;
//        sensor_data[i].acc[2] = ((int32_t)(data[35]<<24|data[34]<<16|data[33]<<8|data[32])*2.0f) / 32768.0f;
//        if(i2c->ack_error())
//            ack_error = true;
//        i++;
//    }
//    return !ack_error;
}

bool ArmControl::readSensorData(SensorFrame &sensor_data, int board) {
//    bool ack_error = false;
//    if(board>deviceIDs.size()||board<0) {
//        ROS_ERROR("invalid board, only configured for %ld boards", deviceIDs.size());
//        return false;
//    }
//    vector<uint8_t> data;
//    i2c->read(deviceIDs[board],0,12,data);
//    i2c->read(deviceIDs[board],1,12,data);
//    i2c->read(deviceIDs[board],2,12,data);
//    sensor_data.current[0] = (uint16_t)(data[1]<<8|data[0]);
//    sensor_data.current[1] = (uint16_t)(data[3]<<8|data[2]);
//    sensor_data.current[2] = (uint16_t)(data[5]<<8|data[4]);
//    sensor_data.current[3] = (uint16_t)(data[7]<<8|data[6]);
//    sensor_data.current[4] = (uint16_t)(data[9]<<8|data[8]);
//    sensor_data.current[5] = (uint16_t)(data[11]<<8|data[10]);
//
//    sensor_data.gyro[0] = ((int32_t)(data[15]<<24|data[14]<<16|data[13]<<8|data[12])*250.0f) / 32768.0f;
//    sensor_data.gyro[1] = ((int32_t)(data[19]<<24|data[18]<<16|data[17]<<8|data[16])*250.0f) / 32768.0f;
//    sensor_data.gyro[2] = ((int32_t)(data[23]<<24|data[22]<<16|data[21]<<8|data[20])*250.0f) / 32768.0f;
//    sensor_data.acc[0] = ((int32_t)(data[27]<<24|data[26]<<16|data[25]<<8|data[24])*2.0f) / 32768.0f;
//    sensor_data.acc[1] = ((int32_t)(data[31]<<24|data[30]<<16|data[29]<<8|data[28])*2.0f) / 32768.0f;
//    sensor_data.acc[2] = ((int32_t)(data[35]<<24|data[34]<<16|data[33]<<8|data[32])*2.0f) / 32768.0f;
//    if(i2c->ack_error())
//        ack_error = true;
//    return !ack_error;
}

bool ArmControl::test() {
    vector<bool> success(handDeviceIDs.size(), true);
    bool allBoardsOK = true;
    for (int board = 0; board < handDeviceIDs.size(); board++) {
        for (uint8_t pos = 150; pos <= 180; pos += 5) {
            MYO_WRITE_motor0(myo_base, board, pos);
            uint8_t pos_0 = MYO_READ_motor0(myo_base, board);
            if(pos_0!=pos)
                success[board] = false;
            else
                ROS_INFO("board %d motor %d pos %d", board, 0, pos_0);
            MYO_WRITE_motor1(myo_base, board, pos);
            uint8_t pos_1 = MYO_READ_motor1(myo_base, board);
            if(pos_0!=pos)
                success[board] = false;
            else
                ROS_INFO("board %d motor %d pos %d", board, 1, pos_0);
            MYO_WRITE_motor2(myo_base, board, pos);
            uint8_t pos_2 = MYO_READ_motor1(myo_base, board);
            if(pos_0!=pos)
                success[board] = false;
            else
                ROS_INFO("board %d motor %d pos %d", board, 2, pos_0);
            MYO_WRITE_motor3(myo_base, board, pos);
            if(pos_0!=pos)
                success[board] = false;
            else
                ROS_INFO("board %d motor %d pos %d", board, 3, pos_0);
            uint8_t pos_3 = MYO_READ_motor1(myo_base, board);
            MYO_WRITE_motor4(myo_base, board, pos);
            if(pos_0!=pos)
                success[board] = false;
            else
                ROS_INFO("board %d motor %d pos %d", board, 4, pos_0);
            uint8_t pos_4 = MYO_READ_motor1(myo_base, board);
            usleep(100000);
        }
        if(!success[board])
            allBoardsOK = false;
    }
    if(!allBoardsOK){
        ROS_ERROR("test on hand boards failed");
        return false;
    }
    return true;
}