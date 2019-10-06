/*
    BSD 3-Clause License

    Copyright (c) 2017, Roboy
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

    author: Simon Trendel ( st@gi.ai ), 2018
    description: Roboys nervous system
*/

#pragma once

#include <ros/ros.h>
#include <vector>
#include "interfaces/icebusControl.hpp"
#include "control/controlActions.hpp"
#include "sensors/A1335.hpp"
#include <roboy_middleware_msgs/ADCvalue.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MagneticSensor.h>
#include <roboy_middleware_msgs/MotorAngle.h>
#include <roboy_middleware_msgs/MotorCalibrationService.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorState.h>
#include <roboy_middleware_msgs/MotorInfo.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <roboy_middleware_msgs/MyoBrickCalibrationService.h>
#include <roboy_middleware_msgs/SetInt16.h>
#include <roboy_control_msgs/Behavior.h>
#include <roboy_control_msgs/StartRecordTrajectory.h>
#include <roboy_middleware_msgs/SystemCheck.h>
#include <roboy_control_msgs/ListItems.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
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
                vector<int32_t *> &i2c_base = DEFAULT_POINTER_VECTOR,
                int32_t *adc_base = nullptr, int32_t *switches_base = nullptr);

    ~RoboyPlexus();


private:
    /**
     * Service for changing the control mode of motors, perviously set PID parameters are restored
     * @param req control mode
     * @param res
     * @return success
     */
    bool ControlModeService(roboy_middleware_msgs::ControlMode::Request &req,
                            roboy_middleware_msgs::ControlMode::Response &res);

    /**
     * Emergency stop service, zeros all PID gains, causing all motors to stop, PID parameters and control mode are restored on release
     * @param req
     * @param res
     * @return
     */
    bool EmergencyStopService(std_srvs::SetBool::Request &req,
                              std_srvs::SetBool::Response &res);

    /**
     * Stops replaying the trajectory
     * @param msg
     */
    void EnablePlaybackCB(const std_msgs::Bool::ConstPtr &msg);

    /**
     * Service returns the list of actions in the requested behavior
     * @param req
     * @param res
     * @return
     */
    bool ExpandBehaviorService(roboy_control_msgs::ListItems::Request &req,
                               roboy_control_msgs::ListItems::Response &res);

    bool ExecuteAction(string actions);

    bool ExecuteActions(vector<string> actions);

    vector<string> ExpandBehavior(string name);

    /**
     * Service return a list of files in the requested folder
     * @param req
     * @param res
     * @return
     */
    bool ListExistingItemsService(roboy_control_msgs::ListItems::Request &req,
                                  roboy_control_msgs::ListItems::Response &res);

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
    bool MotorCalibrationService(roboy_middleware_msgs::MotorCalibrationService::Request &req,
                                 roboy_middleware_msgs::MotorCalibrationService::Response &res);

    /**
     * Service for calibrating the motor angle sensor
     * @param req
     * @param res
     * @return
     */
    bool MyoBrickCalibrationService(roboy_middleware_msgs::MyoBrickCalibrationService::Request &req,
                                    roboy_middleware_msgs::MyoBrickCalibrationService::Response &res);

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
    * Callback updates the displacement for recording trajectories
    * @param msg
    */
    void PredisplacementCB(const std_msgs::Int32 &msg);

    /**
     * Callback saves behavior
     * @param msg
     */
    void SaveBehaviorCB(const roboy_control_msgs::Behavior &msg);

    /**
     * Service sets displacement for all motors
     * @param req
     * @param res
     * @return
     */
    bool SetDisplacementForAll(roboy_middleware_msgs::SetInt16::Request &req,
                               roboy_middleware_msgs::SetInt16::Request &res);
    /**
     * Motor setpoints trajectory recording callback.
     * @param msg
     * @return
     */
    void StartRecordTrajectoryCB(const roboy_control_msgs::StartRecordTrajectory::ConstPtr &msg);

    /**
    * Callback stops recording the trajectory
    * @param msg
    * @return
    */
    void StopRecordTrajectoryCB(const std_msgs::Empty::ConstPtr &msg);

    /**
     * Service for system check
     * @param req
     * @param res
     * @return
     */
    bool SystemCheckService(roboy_middleware_msgs::SystemCheck::Request &req,
                            roboy_middleware_msgs::SystemCheck::Response &res);

private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber motorCommand_sub, startRecordTrajectory_sub, stopRecordTrajectory_sub, saveBehavior_sub,
            enablePlayback_sub, predisplacement_sub;
    ros::Publisher motorState, motorInfo, darkroom, darkroom_ootx, darkroom_status, adc, gsensor,
            motorAngle, magneticSensor;
    ros::ServiceServer motorConfig_srv, controlMode_srv, emergencyStop_srv, motorCalibration_srv,
            replayTrajectory_srv, executeActions_srv, executeBehavior_srv, setDisplacementForAll_srv,
            listExistingTrajectories_srv, listExistingBehaviors_srv, expandBehavior_srv;
    map<int, map<int, control_Parameters_t>> control_params_backup;
    map<int, int> control_mode, control_mode_backup;
    IcebusControlPtr icebusControl;
    A1335Ptr a1335;
    boost::shared_ptr<std::thread> adcThread, motorStateThread, motorInfoThread,
            jointAngleThread, magneticsThread;
    bool keep_publishing = true;
    int32_t *adc_base, *switches_base;
    vector<int32_t *> i2c_base;
    bool emergency_stop = false;
    int file;
    const char *filename = "/dev/i2c-0";
    uint8_t id = 0;
    const int mg_per_digi = 4;
    uint16_t szXYZ[3];
    int cnt = 0, max_cnt = 0;
    string ethaddr;

    int active_magnetic_sensors = 0;
    vector<TLE493DPtr> tle;

    string body_part;
    vector<string> body_parts;
};

/** @} */ // end of group1