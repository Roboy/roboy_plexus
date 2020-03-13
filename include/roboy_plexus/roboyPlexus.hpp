/*
    BSD 3-Clause License

    Copyright (c) 2020, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Simon Trendel ( st@gi.ai ), 2020
    description: Roboys nervous system
*/

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "interfaces/icebusControl.hpp"
// #include "control/controlActions.hpp"
#include "control/fanControl.hpp"
#include <roboy_middleware_msgs/msg/ad_cvalue.hpp>
#include <roboy_middleware_msgs/msg/dark_room.hpp>
#include <roboy_middleware_msgs/msg/dark_room_ootx.hpp>
#include <roboy_middleware_msgs/msg/dark_room_sensor.hpp>
#include <roboy_middleware_msgs/msg/dark_room_status.hpp>
#include <roboy_middleware_msgs/srv/control_mode.hpp>
#include <roboy_middleware_msgs/msg/magnetic_sensor.hpp>
#include <roboy_middleware_msgs/msg/motor_angle.hpp>
#include <roboy_middleware_msgs/srv/motor_calibration_service.hpp>
#include <roboy_middleware_msgs/msg/motor_command.hpp>
#include <roboy_middleware_msgs/msg/motor_control.hpp>
#include <roboy_middleware_msgs/msg/motor_state.hpp>
#include <roboy_middleware_msgs/msg/motor_status.hpp>
#include <roboy_middleware_msgs/msg/motor_info.hpp>
#include <roboy_middleware_msgs/srv/motor_config_service.hpp>
#include <roboy_middleware_msgs/srv/emergency_stop.hpp>
#include <roboy_middleware_msgs/srv/myo_brick_calibration_service.hpp>
#include <roboy_middleware_msgs/msg/neopixel.hpp>
#include <roboy_middleware_msgs/srv/set_int16.hpp>
#include <roboy_control_msgs/msg/behavior.hpp>
#include <roboy_control_msgs/msg/start_record_trajectory.hpp>
#include <roboy_middleware_msgs/srv/system_check.hpp>
#include <roboy_control_msgs/srv/list_items.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <map>
#include <chrono>
#include <algorithm>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "hwlib.h"
#include "utility/half.hpp"
#include "utility/CRC32.h"
#include <bitset>
#include "sensors/A1335.hpp"
#include "sensors/BallJoint.hpp"
#include "sensors/tlv493d.hpp"
#include "sensors/tle493d_w2b6.hpp"
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <common_utilities/CommonDefinitions.h>

#pragma pack(1) // we need this, otherwise the ootx union will be padded and the checksum test fails

using namespace std;
using namespace chrono;
using half_float::half;

static vector<int32_t *> DEFAULT_POINTER_VECTOR;

/** @defgroup test The First Group
 *  This is the first group
 *  @{
 */
class RoboyPlexus {
public:
    RoboyPlexus(IcebusControlPtr icebusControl,
                vector<BallJointPtr> balljoints,
                vector<FanControlPtr> fanControls,
                vector<int32_t *> &i2c_base = DEFAULT_POINTER_VECTOR);

    ~RoboyPlexus();


private:
    /**
     * Service for changing the control mode of motors, perviously set PID parameters are restored
     * @param req control mode
     * @param res
     * @return success
     */
    void ControlModeService(const shared_ptr<roboy_middleware_msgs::srv::ControlMode::Request> req,
                            shared_ptr<roboy_middleware_msgs::srv::ControlMode::Response> res);


    void ElbowJointPublisher();

    /**
     * Emergency stop service, zeros all PID gains, causing all motors to stop, PID parameters and control mode are restored on release
     * @param req
     * @param res
     * @return
     */
    void EmergencyStopService(const shared_ptr<std_srvs::srv::SetBool::Request> req,
                              shared_ptr<std_srvs::srv::SetBool::Response> res);

    /**
     * Stops replaying the trajectory
     * @param msg
     */
    void EnablePlaybackCB(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * Service returns the list of actions in the requested behavior
     * @param req
     * @param res
     * @return
     */
    void ExpandBehaviorService(const shared_ptr<roboy_control_msgs::srv::ListItems::Request> req,
                               shared_ptr<roboy_control_msgs::srv::ListItems::Response> res);

    bool ExecuteAction(string actions);

    bool ExecuteActions(vector<string> actions);

    vector<string> ExpandBehavior(string name);

    /**
     * Service return a list of files in the requested folder
     * @param req
     * @param res
     * @return
     */
    void ListExistingItemsService(const shared_ptr<roboy_control_msgs::srv::ListItems::Request> req,
                                  shared_ptr<roboy_control_msgs::srv::ListItems::Response> res);

    /**
     * Publishes 3d magnetic information about joint
     */
    void MagneticJointPublisher();

    /**
     * Service for motor spring calibration, samples the spring space for a requested amount of time and estimates spring coefficients
     * @param req
     * @param res
     * @return
     */
    void MotorCalibrationService(const shared_ptr<roboy_middleware_msgs::srv::MotorCalibrationService::Request> req,
                                 shared_ptr<roboy_middleware_msgs::srv::MotorCalibrationService::Response> res);

    /**
     * Service for calibrating the motor angle sensor
     * @param req
     * @param res
     * @return
     */
    void MyoBrickCalibrationService(const shared_ptr<roboy_middleware_msgs::srv::MyoBrickCalibrationService::Request> req,
                                    shared_ptr<roboy_middleware_msgs::srv::MyoBrickCalibrationService::Response> res);

    /**
     * Callback for motor command
     * @param msg motor command
     */
    void MotorCommand(const roboy_middleware_msgs::msg::MotorCommand::SharedPtr msg);

    /**
     * Callback for motor control
     * @param msg motor control
     */
    void MotorControl(const roboy_middleware_msgs::msg::MotorControl::SharedPtr msg);

    /**
     * Service for changing motor PID parameters
     * @param req PID parameters
     * @param res success
     * @return success
     */
    void MotorConfigService(const shared_ptr<roboy_middleware_msgs::srv::MotorConfigService::Request> req,
                            shared_ptr<roboy_middleware_msgs::srv::MotorConfigService::Response> res);

    /**
     * Publishes information about motors
     */
    void MotorInfoPublisher();

    /**
     * Publishes state of motors
     */
    void MotorStatePublisher();

    /**
     * Publishes state of motors
     */
    [[deprecated("replaced with iCEbus")]]
    void MotorStatusPublisher();

    void Neopixel(const roboy_middleware_msgs::msg::Neopixel::SharedPtr msg);

    uint8_t reverseBits(uint8_t a);

    /**
    * Callback updates the displacement for recording trajectories
    * @param msg
    */
    void PredisplacementCB(const std_msgs::msg::Int32 &msg);

    /**
     * Callback saves behavior
     * @param msg
     */
    void SaveBehaviorCB(const roboy_control_msgs::msg::Behavior &msg);

    /**
     * Service sets displacement for all motors
     * @param req
     * @param res
     * @return
     */
    void SetDisplacementForAll(const shared_ptr<roboy_middleware_msgs::srv::SetInt16::Request> req,
                               shared_ptr<roboy_middleware_msgs::srv::SetInt16::Request> res);
    // /**
    //  * Motor setpoints trajectory recording callback.
    //  * @param msg
    //  * @return
    //  */
     void StartRecordTrajectoryCB(const roboy_control_msgs::msg::StartRecordTrajectory::SharedPtr msg);
    //
    // /**
    // * Callback stops recording the trajectory
    // * @param msg
    // * @return
    // */
     void StopRecordTrajectoryCB(const std_msgs::msg::Empty::SharedPtr msg);

    /**
     * Service for system check
     * @param req
     * @param res
     * @return
     */
    void SystemCheckService(const shared_ptr<roboy_middleware_msgs::srv::SystemCheck::Request> req,
                            shared_ptr<roboy_middleware_msgs::srv::SystemCheck::Response> res);

private:
    rclcpp::Node::SharedPtr nh;
    // std::shared_ptr<ros::AsyncSpinner> spinner;
    // rclcpp::Subscription motorCommand_sub, startRecordTrajectory_sub, stopRecordTrajectory_sub, saveBehavior_sub,
    //         enablePlayback_sub, predisplacement_sub, motorControl_sub, neopixel_sub;
    // rclcpp::publisher::Publisher motorState, motorInfo, motorStatus, darkroom, darkroom_ootx, darkroom_status, adc, gsensor,
    //         motorAngle, magneticSensor, jointState;
    // rclcpp::service::Service motorConfig_srv, controlMode_srv, emergencyStop_srv, motorCalibration_srv,
    //         replayTrajectory_srv, executeActions_srv, executeBehavior_srv, setDisplacementForAll_srv,
    //         listExistingTrajectories_srv, listExistingBehaviors_srv, expandBehavior_srv;

    rclcpp::Subscription<roboy_middleware_msgs::msg::MotorCommand>::SharedPtr motorCommand_sub;
    rclcpp::Subscription<roboy_middleware_msgs::msg::MotorControl>::SharedPtr motorControl_sub;
    rclcpp::Subscription<roboy_middleware_msgs::msg::Neopixel>::SharedPtr neopixel_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr predisplacement_sub;

    rclcpp::Publisher<roboy_middleware_msgs::msg::MotorState>::SharedPtr motorState;
    rclcpp::Publisher<roboy_middleware_msgs::msg::MotorInfo>::SharedPtr motorInfo;
    rclcpp::Publisher<roboy_middleware_msgs::msg::MotorStatus>::SharedPtr motorStatus;
    rclcpp::Publisher<roboy_middleware_msgs::msg::DarkRoom>::SharedPtr darkroom;
    rclcpp::Publisher<roboy_middleware_msgs::msg::DarkRoomOOTX>::SharedPtr darkroom_ootx;
    rclcpp::Publisher<roboy_middleware_msgs::msg::DarkRoomStatus>::SharedPtr darkroom_status;
    rclcpp::Publisher<roboy_middleware_msgs::msg::ADCvalue>::SharedPtr adc;
    // rclcpp::Publisher<roboy_middleware_msgs::msg::GS>::SharedPtr gsensor;
    rclcpp::Publisher<roboy_middleware_msgs::msg::MotorAngle>::SharedPtr motorAngle;
    rclcpp::Publisher<roboy_middleware_msgs::msg::MagneticSensor>::SharedPtr magneticSensor;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointState;

    rclcpp::Service<roboy_middleware_msgs::srv::MotorConfigService>::SharedPtr motorConfig_srv;
    rclcpp::Service<roboy_middleware_msgs::srv::ControlMode>::SharedPtr controlMode_srv;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergencyStop_srv;
    rclcpp::Service<roboy_middleware_msgs::srv::MotorCalibrationService>::SharedPtr motorCalibration_srv;
     rclcpp::Service<roboy_middleware_msgs::srv::SetInt16>::SharedPtr setDisplacementForAll_srv;
    map<int, map<int, control_Parameters_t>> control_params_backup;
    map<int, int> control_mode_backup,control_mode;
    vector<MotorControlPtr> motorControl;
    IcebusControlPtr icebusControl;
    vector<FanControlPtr> fanControls;
    vector<A1335Ptr> a1335;
    std::shared_ptr<std::thread> adcThread, motorStateThread, motorInfoThread, motorStatusThread,
            elbowJointAngleThread, magneticsThread;
    bool keep_publishing = true;
    int32_t *adc_base, *switches_base;
    vector<int32_t *> i2c_base;
    vector<BallJointPtr> balljoints;
    bool emergency_stop = false;
    int file;
    const char *filename = "/dev/i2c-0";
    uint8_t id = 0;
    const int mg_per_digi = 4;
    uint16_t szXYZ[3];
    int cnt = 0, max_cnt = 0;
    string ethaddr;

    bool external_led_control = false;

    string body_part;
    vector<string> body_parts;
};

/** @} */ // end of group1
