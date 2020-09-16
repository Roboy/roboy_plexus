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

#include <ros/ros.h>
#include <vector>
#include <thread>
#include <map>
#include <chrono>
#include <algorithm>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "hwlib.h"
#include <bitset>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <common_utilities/CommonDefinitions.h>
#include "interfaces/iCEbusControl.hpp"
#include "interfaces/fanControl.hpp"
#include "interfaces/myoControl.hpp"
#include "sensors/A1335.hpp"
#include "sensors/BallJoint.hpp"
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MagneticSensor.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorState.h>
#include <roboy_middleware_msgs/MotorInfo.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <roboy_middleware_msgs/Neopixel.h>
#include <roboy_middleware_msgs/RoboyState.h>
#include <roboy_middleware_msgs/SystemCheck.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include "sensors/tle493d_w2b6.hpp"
#include "sensors/tli4970.hpp"

using namespace std;
using namespace chrono;

static vector<int32_t *> DEFAULT_POINTER_VECTOR;

/** @defgroup test The First Group
 *  This is the first group
 *  @{
 */
class RoboyPlexus {
public:
    RoboyPlexus(IcebusControlPtr icebusControl,
                vector<TLE493DPtr> balljoints,
                vector<FanControlPtr> fanControls,
                int32_t *led,
                int32_t *switches,
                int32_t *power_control,
                int32_t *power_sense,
                vector<int32_t *> &i2c_base,
                int32_t * tli4970_base,
                vector<int> elbow_sensor_order,
                vector<int> elbow_sensor_sign,
                vector<float> elbow_sensor_offset,
                vector<int> knee_sensor_order,
                vector<int> knee_sensor_sign,
                vector<float> knee_sensor_offset,
                MyoControlPtr myoControl = nullptr);

    ~RoboyPlexus();


private:
    /**
     * Service for changing the control mode of motors, previously set PID parameters are restored
     * @param req control mode
     * @param res
     * @return success
     */
    bool ControlModeService(roboy_middleware_msgs::ControlMode::Request &req,
                            roboy_middleware_msgs::ControlMode::Response &res);

    /**
    * Publishes elbow joint angles to /external_joint_states
    */
    void ElbowJointPublisher();

    /**
    * Publishes knee joint angles to /external_joint_states
    */
    void KneeJointPublisher();

    /**
     * Emergency stop service, zeros all PID gains, causing all motors to stop, PID parameters and control mode are restored on release
     * @param req
     * @param res
     * @return
     */
    bool EmergencyStopService(std_srvs::SetBool::Request &req,
                              std_srvs::SetBool::Response &res);
    /**
    * Subscriber to /roboy/middleware/FanControl
    * controls the pwm percentage of the mosfets on the myo_shield_rev0.6
    * if the fans are connected to these mosfets, the speed of the fans can be regulated
    */
    void FanControl(std_msgs::Int32 duty);

    /**
    * Service for controlling the auto fan mode. The average current of the first
    * four icebuses is used to scale the pwm output of the mosfets connected to the
    * fans. The idea is that cooling via the fans is regulated automatically based
    * on the current consumption. This feature has not been tested yet
    * @param req enable/disable
    * @param res enabled/disabled
    * @return success
    */
    bool FanControlService(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);

    /**
     * Publishes 3d magnetic information about joint
     */
    void MagneticJointPublisher();

    /**
     * Callback for motor command
     * @param msg motor command
     */
    void MotorCommand(const roboy_middleware_msgs::MotorCommand::ConstPtr &msg);

    /**
     * Service for changing motor PID parameters
     * @param req PID parameters
     * @param res success
     * @return success
     */
    bool MotorConfigService(roboy_middleware_msgs::MotorConfigService::Request &req,
                            roboy_middleware_msgs::MotorConfigService::Response &res);

    /**
     * Publishes information about motors
     */
    void MotorInfoPublisher();

    /**
     * Publishes state of motors
     */
    void MotorStatePublisher();

    /**
    * Subscriber to /roboy/middleware/Neopixel
    * The neopixel color of each iceboard can be controlled with this topic
    */
    void Neopixel(const roboy_middleware_msgs::Neopixel::ConstPtr &msg);

    /**
     * Publishes state of roboy
     */
    void RoboyStatePublisher();

    /**
     * Service for system check
     * @param req
     * @param res
     * @return
     */
    bool SystemCheckService(roboy_middleware_msgs::SystemCheck::Request &req,
                            roboy_middleware_msgs::SystemCheck::Response &res);

    /**
     * Service for controlling the 5V line
     * @param req
     * @param res
     * @return
     */
    bool PowerService5V(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);

    /**
     * Service for controlling the 12V line
     * @param req
     * @param res
     * @return
     */
    bool PowerService12V(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);

private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber motorCommand_sub, neopixel_sub, fan_control_sub;
    ros::Publisher jointState, magneticSensor, motorInfo, motorState, roboyState;
    ros::ServiceServer motorConfig_srv, controlMode_srv, emergencyStop_srv,
      power5V_srv, power12V_srv, fan_control_srv, systemcheck_srv;
    map<int, map<int, control_Parameters_t>> control_params_backup;
    map<int, int> control_mode_backup,control_mode;
    vector<MotorControlPtr> motorControl;
    IcebusControlPtr icebusControl;
    MyoControlPtr myoControl;
    vector<FanControlPtr> fanControls;
    int32_t *power_control, *power_sense, *switches, *led;
    vector<A1335Ptr> a1335_elbow, a1335_knee;
    boost::shared_ptr<std::thread> elbowJointAngleThread, kneeJointAngleThread,
        magneticsThread, motorInfoThread, motorStateThread, roboyStateThread;
    bool keep_publishing = true;
    vector<TLE493DPtr> balljoints;
    bool emergency_stop = false;
    string ethaddr;

    bool external_led_control = false;
    bool power_5V_enabled = false, power_12V_enabled = false;
    string body_part;
    vector<string> body_parts;
    TLI4970Ptr tli4970;
    vector<int> elbow_sensor_order, elbow_sensor_sign, knee_sensor_order, knee_sensor_sign;
    vector<float> elbow_sensor_offset, knee_sensor_offset;
};

/** @} */ // end of group1
