#pragma once

#include <ros/ros.h>
#include <thread>
#include <vector>
#include <roboy_middleware_msgs/HandCommand.h>
#include <roboy_control_msgs/SetMode.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include "roboy_plexus/myoControl.hpp"
#include "roboy_plexus/hps_0.h"

#define HAND_LEFT 0
#define HAND_RIGHT 1

#define FINGER_THUMB 0
#define FINGER_INDEX 1
#define FINGER_MIDDLE 2
#define FINGER_RING 3
#define FINGER_LITTLE 4

#define COMMAND_OPEN 0
#define COMMAND_CLOSE 1

// TODO: test this values
#define VAL_LOCK_FINGER 1
#define VAL_UNLOCK_FINGER 0
#define MOTOR_DIRECTION_CLOSE 1
#define MOTOR_DIRECTION_OPEN 0
#define DEFAULT_PWM 2000
#define MOTOR_DIRECTION_LEFT HAND_LEFT
#define MOTOR_DIRECTION_RIGHT HAND_RIGHT

#define WRITE_LOCK_FINGER(base, hand_id, finger) IOWR(base, (1 << (finger + 4*hand_id)), LOCK_FIGHER_VAL)
#define WRITE_MOTOR_DIRECTION(base, hand_id, direction) IOWR(base, hand_id, direction)
#define WRITE_MOTOR_PWM(base, hand_id, pwm) IOWR(base, hand_id, pwm)


class ArmControl {
public:
    ArmControl(int32_t *myo_base, uint8_t hand_id);

    ~ArmControl();

    /**
     * Callback for hand command
     * @param msg hand command
     */
    void handCommandCB(const roboy_middleware_msgs::HandCommand::ConstPtr &msg);

private:

    void lockFingers(std::vector<uint8_t> finger_ids);

private:
    ros::NodeHandlePtr nh;
    ros::Subscriber handCommand_sub;
    int32_t *myo_base;
    uint8_t hand_id = 0;
    string hand;
};

typedef boost::shared_ptr<ArmControl> ArmControlPtr;