#include "roboy_plexus/armControl.hpp"

ArmControl::ArmControl(int32_t *myo_base, uint8_t elbowDeviceID, uint8_t wristDeviceID, vector<uint8_t> handDeviceIDs,
                         bool id, bool hand_control_active , bool elbow_joint_controller_active) :
        myo_base(myo_base), handDeviceIDs(handDeviceIDs), id(id), elbow_joint_controller_active(elbow_joint_controller_active){

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
    }

    handCommand_sub = nh->subscribe("/roboy/middleware/HandCommand", 1, &ArmControl::handCommandCB, this);
    armStatus_pub = nh->advertise<roboy_middleware_msgs::ArmStatus>("/roboy/middleware/ArmStatus", 1);
    if (!id) {
        elbowCommand_sub = nh->subscribe("/roboy/middleware/elbow_left/JointAngle", 1, &ArmControl::elbowCommandCB,
                                         this);
        jointController_srv = nh->advertiseService("/roboy/middleware/arm/left/JointControllerConfig",
                                                   &ArmControl::JointControllerService, this);
        elbow_joint_controller_srv = nh->advertiseService("/roboy/middleware/elbow_left/JointController",
                                                   &ArmControl::ElbowJointControllerService, this);
    } else {
        elbowCommand_sub = nh->subscribe("/roboy/middleware/elbow_right/JointAngle", 1, &ArmControl::elbowCommandCB,
                                         this);
        jointController_srv = nh->advertiseService("/roboy/middleware/arm/right/JointController",
                                                   &ArmControl::JointControllerService, this);
        elbow_joint_controller_srv = nh->advertiseService("/roboy/middleware/elbow_right/JointController",
                                                          &ArmControl::ElbowJointControllerService, this);
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

void ArmControl::handCommandCB(const roboy_middleware_msgs::HandCommand::ConstPtr &msg) {
    if (msg->id == id) {
        vector<uint8_t> setPoint;
        stringstream str;
        str << "hand " << (id ? "right" : "left") << " command:";
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

void ArmControl::fingerCommandCB(const roboy_middleware_msgs::FingerCommand::ConstPtr &msg) {
    if (msg->id == id) {
        if (msg->finger > 3) {
            ROS_ERROR("invalid finger, use THUMB(0), INDEXFINGER(1), MIDDLEFINGER(2), RINGLITTLE(3)");
            return;
        }
    }
}

bool ArmControl::setHandModeService(roboy_control_msgs::SetModeRequest &req,
                                     roboy_control_msgs::SetModeResponse &res) {
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

bool ArmControl::JointControllerService(roboy_middleware_msgs::JointControllerRequest &req,
                                         roboy_middleware_msgs::JointControllerResponse &res) {
    MYO_WRITE_elbow_joint_control(myo_base,req.elbow_control_enable);
    MYO_WRITE_elbow_Kp_joint_angle(myo_base, req.kp_elbow_joint);
    MYO_WRITE_elbow_Kd_joint_angle(myo_base, req.kd_elbow_joint);
    MYO_WRITE_Kp(myo_base, elbow_agonist, req.kp_elbow_agonist);
    MYO_WRITE_Kp(myo_base, elbow_antagonist, req.kp_elbow_antagonist);
    MYO_WRITE_Kd(myo_base, elbow_agonist, req.kd_elbow_agonist);
    MYO_WRITE_Kd(myo_base, elbow_antagonist, req.kd_elbow_agonist);
    MYO_WRITE_elbow_joint_pretension(myo_base, req.elbow_pretension);
    MYO_WRITE_elbow_joint_deadband(myo_base, req.elbow_deadband);
    MYO_WRITE_wrist_joint_control(myo_base,req.wrist_control_enable);
    MYO_WRITE_wrist_Kp_joint_angle(myo_base, req.kp_wrist_joint);
    MYO_WRITE_wrist_Kd_joint_angle(myo_base, req.kd_wrist_joint);
    MYO_WRITE_wrist_joint_pretension(myo_base, req.wrist_pretension);
    MYO_WRITE_wrist_joint_deadband(myo_base, req.wrist_deadband);
}

bool ArmControl::ElbowJointControllerService(std_srvs::SetBoolRequest &req,
                                 std_srvs::SetBoolResponse &res){
    ROS_INFO("%s elbow controller", (req.data?"enableing":"disabeling" ));
    MYO_WRITE_elbow_joint_control(myo_base,((bool)req.data));
    return true;
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
        roboy_middleware_msgs::ArmStatus msg;
        msg.id = id;

        msg.elbow_joint_angle = 360.0 * MYO_READ_elbow_joint_angle(myo_base) / 4096.0;
        msg.elbow_joint_angle_setpoint = 360.0 * MYO_READ_elbow_joint_angle_setpoint(myo_base) / 4096.0;
        msg.elbow_agonist_setpoint = MYO_READ_sp(myo_base, elbow_agonist);
        msg.elbow_antagonist_setpoint = MYO_READ_sp(myo_base, elbow_antagonist);

        msg.wrist_joint_angle = -360.0 * MYO_READ_wrist_joint_angle(myo_base) / 4096.0; // negative because angle is flipped
        msg.wrist_joint_angle_setpoint = -360.0 * MYO_READ_wrist_joint_angle_setpoint(myo_base) / 4096.0;

        armStatus_pub.publish(msg);

        rate.sleep();
    }
}