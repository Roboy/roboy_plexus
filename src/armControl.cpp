#include "roboy_plexus/armControl.hpp"

ArmControl::ArmControl(int32_t *myo_base, uint8_t hand_id) : myo_base(myo_base), hand_id(hand_id) {

    if (hand_id == HAND_LEFT) {
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

}

ArmControl::~ArmControl() {
}

void ArmControl::handCommandCB(const roboy_middleware_msgs::HandCommand::ConstPtr &msg) {
    if (msg->hand_id != hand_id) {
        return;
    }
    if (msg->command == COMMAND_OPEN) {
        ROS_INFO("Opening hand!");
        WRITE_MOTOR_DIRECTION(myo_base, hand_id, MOTOR_DIRECTION_OPEN);

    } else {
        WRITE_MOTOR_DIRECTION(myo_base, hand_id, MOTOR_DIRECTION_CLOSE);
        ROS_INFO("Closing hand!");
    }
    lockFingers(msg->lock_finger_ids);
    WRITE_MOTOR_PWM(myo_base, hand_id, DEFAULT_PWM);
}

void ArmControl::lockFingers(std::vector<uint8_t> finger_ids) {
    std::vector<uint8_t> locked_fingers(5, VAL_UNLOCK_FINGER);
    for (auto id : finger_ids) {
        locked_fingers[id] = VAL_LOCK_FINGER;
    }
}