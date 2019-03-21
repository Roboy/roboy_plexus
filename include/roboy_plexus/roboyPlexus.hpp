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
#include <roboy_plexus/myoControl.hpp>
#include "roboy_plexus/controlActions.hpp"
#include <roboy_plexus/am4096.hpp>
#include <roboy_plexus/Adafruit_LSM9DS1.hpp>
//#include <roboy_soli/roboySoli.hpp>
#include <roboy_middleware_msgs/ADCvalue.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/DarkRoom.h>
#include <roboy_middleware_msgs/DarkRoomOOTX.h>
#include <roboy_middleware_msgs/DarkRoomStatus.h>
#include <roboy_middleware_msgs/JointStatus.h>
#include <roboy_middleware_msgs/MagneticSensor.h>
#include <roboy_middleware_msgs/MotorAngle.h>
#include <roboy_middleware_msgs/MotorCalibrationService.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorStatus.h>
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
#include "std_msgs/Int32.h"
#include <sensor_msgs/Imu.h>
#include <thread>
#include <map>
#include <chrono>
#include <algorithm>
#include <roboy_plexus/ADXL345.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "hwlib.h"
#include "roboy_plexus/half.hpp"
#include "roboy_plexus/CRC32.h"
#include <bitset>
#include "roboy_plexus/A1335.hpp"
#include "roboy_plexus/tlv493d.hpp"
#include <sys/types.h>
#include <dirent.h>
//#include "roboy_plexus/armControl.hpp"
#include <sys/stat.h>
#include <common_utilities/CommonDefinitions.h>
#include <bondcpp/bond.h>


#define NUM_SENSORS 12
#define NUMBER_OF_LOADCELLS 8

#pragma pack(1) // we need this, otherwise the ootx union will be padded and the checksum test fails

using namespace std;
using namespace chrono;
using half_float::half;

static vector<int32_t *> DEFAULT_POINTER_VECTOR;
static vector<int32_t> DEFAULT_VECTOR;
static vector<vector<int32_t>> DEFAULT_VECTOR_VECTOR;

/** @defgroup test The First Group
 *  This is the first group
 *  @{
 */
class RoboyPlexus {
public:
    RoboyPlexus(MyoControlPtr myoControl, vector<int32_t *> &myo_base,
                vector<int32_t *> &i2c_base = DEFAULT_POINTER_VECTOR,
                int32_t *darkroom_base = nullptr, vector<int32_t *> &darkroom_ootx_addr = DEFAULT_POINTER_VECTOR,
                int32_t *adc_base = nullptr, int32_t *switches_base = nullptr);

    ~RoboyPlexus();

    string getBodyPart();

    /**
     * Publishes information about motors
     */
    void motorStatusPublisher();

private:
    /**
     * Publishes IMU data
     */
    void gsensorPublisher();

    /**
     * Publishes ADC values
     */
    void adcPublisher();

    /**
     * Publishes lighthouse sensor values
     */
    void darkRoomPublisher();

    /**
     * Publishes decoded lighthouse ootx data
     */
    void darkRoomOOTXPublisher();

    /**
     * Publishes motor angles
     */
    void motorAnglePublisher();

    /**
     * Publishes 3d magnetic information about shoulder joint
     */
    void magneticShoulderJointPublisher();

    /**
     * Callback for motor command
     * @param msg motor command
     */
    void motorCommandCB(const roboy_middleware_msgs::MotorCommand::ConstPtr &msg);

    /**
     * Motor Angle PID controller
     */
    void motorAnglePID();

    /**
     * Service for turning on/off hand
     * @param req
     * @param res
     * @return success
     */
    bool HandPower(std_srvs::SetBool::Request &req,
                   std_srvs::SetBool::Response &res);

    /**
     * Service for changing motor PID parameters
     * @param req PID parameters
     * @param res success
     * @return success
     */
    bool MotorConfigService(roboy_middleware_msgs::MotorConfigService::Request &req,
                            roboy_middleware_msgs::MotorConfigService::Response &res);

    /**
     * Service for changing the control mode of motors, perviously set PID parameters are restored
     * @param req control mode
     * @param res
     * @return success
     */
    bool ControlModeService(roboy_middleware_msgs::ControlMode::Request &req,
                            roboy_middleware_msgs::ControlMode::Response &res);

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
     * Emergency stop service, zeros all PID gains, causing all motors to stop, PID parameters and control mode are restored on release
     * @param req
     * @param res
     * @return
     */
    bool EmergencyStopService(std_srvs::SetBool::Request &req,
                              std_srvs::SetBool::Response &res);

    /**
     * Service for system check
     * @param req
     * @param res
     * @return
     */
    bool SystemCheckService(roboy_middleware_msgs::SystemCheck::Request &req,
                            roboy_middleware_msgs::SystemCheck::Response &res);

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
     * Callback saves behavior
     * @param msg
     */
    void SaveBehaviorCB(const roboy_control_msgs::Behavior &msg);

    /**
     * Callback updates the displacement for recording trajectories
     * @param msg
     */
    void PredisplacementCB(const std_msgs::Int32 &msg);

    /**
     * Stops replaying the trajectory
     * @param msg
     */
    void EnablePlaybackCB(const std_msgs::Bool::ConstPtr &msg);


    /**
     * Service return a list of files in the requested folder
     * @param req
     * @param res
     * @return
     */
    bool ListExistingItemsService(roboy_control_msgs::ListItems::Request &req,
                                  roboy_control_msgs::ListItems::Response &res);

    /**
     * Service returns the list of actions in the requested behavior
     * @param req
     * @param res
     * @return
     */
    bool ExpandBehaviorService(roboy_control_msgs::ListItems::Request &req,
                               roboy_control_msgs::ListItems::Response &res);


    /**
     * Service sets displacement for all motors
     * @param req
     * @param res
     * @return
     */
    bool SetDisplacementForAll(roboy_middleware_msgs::SetInt16::Request &req,
                               roboy_middleware_msgs::SetInt16::Request &res);

    /**
     * creates bond required by skill machine
     */
    void bondCreator();


private:
    /**
     * Reverses the bit order of a byte, eg. 0b00001011 -> 0b11010000
     * @param b input byte
     * @return reversed byte
     */
    inline uint8_t reverse(uint8_t b) {
        b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
        b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
        b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
        return b;
    }

    /**
     * Reverses the bit order of each byte in a 4 byte uint32_t
     * @param b input
     * @return reversed uint32_t
     */
    inline uint32_t reverse(uint32_t b) {
        uint32_t a = (uint32_t) ((uint8_t) reverse((uint8_t) (b >> 24 & 0xff)) << 24 |
                                 (uint8_t) reverse((uint8_t) (b >> 16 & 0xff)) << 16 |
                                 (uint8_t) reverse((uint8_t) (b >> 8 & 0xff)) << 8 |
                                 (uint8_t) reverse((uint8_t) (b & 0xff)));
        return a;
    }

    string node_name;

    // helper functions for reading the IMU
    bool ADXL345_REG_WRITE(int file, uint8_t address, uint8_t value);

    bool ADXL345_REG_READ(int file, uint8_t address, uint8_t *value);

    bool ADXL345_REG_MULTI_READ(int file, uint8_t readaddr, uint8_t readdata[], uint8_t len);

    bool ADXL345_Init(int file);

    bool ADXL345_IsDataReady(int file);

    bool ADXL345_XYZ_Read(int file, uint16_t szData16[3]);

    bool ADXL345_IdRead(int file, uint8_t *pId);

    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber motorCommand_sub, startRecordTrajectory_sub, stopRecordTrajectory_sub, saveBehavior_sub,
            enablePlayback_sub, predisplacement_sub;
    ros::Publisher motorStatus_pub, darkroom_pub, darkroom_ootx_pub, darkroom_status_pub, adc_pub, gsensor_pub,
            motorAngle_pub, magneticSensor_pub, jointStatus_pub;
    ros::ServiceServer motorConfig_srv, controlMode_srv, emergencyStop_srv, motorCalibration_srv,
            replayTrajectory_srv, executeActions_srv, executeBehavior_srv, handPower_srv,
            setDisplacementForAll_srv, listExistingTrajectories_srv, listExistingBehaviors_srv, expandBehavior_srv,
            soliGetData_srv, soli_srv, soliGetFrameFormat_srv, soliSetFrameFormat_srv, soliGetAdcSamplerate_srv,
            soliSetAdcSamplerate_srv, soliGetChirpDuration_srv, soliSetFMCWConfiguration_srv, soliGetFMCWConfiguration_srv,
            soliGetFrameInfo_srv, myobrick_calibration_srv;
    map<int, int> setPoint_backup;
    map<int, map<int, control_Parameters_t>> control_params_backup;
    map<int, int> control_mode, control_mode_backup;
    map<int, int> rotationCounter;
    map<int, float> motorAngles;
    boost::shared_ptr<MyoControl> myoControl;
//    ArmControlPtr armControl;
    boost::shared_ptr<std::thread> adcThread, darkRoomThread, darkRoomOOTXThread, motorStatusThread,
            gsensor_thread, motorAngleThread, jointAngleThread, magneticsShoulderThread, bondThread;
    bool keep_publishing = true;
    int32_t *darkroom_base, *adc_base, *switches_base;
    vector<int32_t *> myo_base, i2c_base, darkroom_ootx_addr;
    bool emergency_stop = false;
    vector<boost::shared_ptr<A1335>> motorAngle; // motor angle of new motor units
    vector<boost::shared_ptr<AM4096>> jointAngle;
    int file;
    const char *filename = "/dev/i2c-0";
    uint8_t id;
    bool bSuccess;
    const int mg_per_digi = 4;
    uint16_t szXYZ[3];
    int cnt = 0, max_cnt = 0;

    union {
        uint8_t data[33];
        struct {
            uint16_t fw_version;        // 0x00
            uint32_t ID;                // 0x02
            uint16_t fcal_0_phase;      // 0x06
            uint16_t fcal_1_phase;      // 0x08
            uint16_t fcal_0_tilt;       // 0x0A
            uint16_t fcal_1_tilt;       // 0x0C
            uint8_t unlock_count;       // 0x0E
            uint8_t hw_version;         // 0x0F
            uint16_t fcal_0_curve;      // 0x10
            uint16_t fcal_1_curve;      // 0x12
            int8_t accel_dir_x;         // 0x14
            int8_t accel_dir_y;         // 0x15
            int8_t accel_dir_z;         // 0x16
            uint16_t fcal_0_gibphase;   // 0x17
            uint16_t fcal_1_gibphase;   // 0x19
            uint16_t fcal_0_gibmag;     // 0x1B
            uint16_t fcal_1_gibmag;     // 0x1D
            uint8_t mode;               // 0x1F
            uint8_t faults;             // 0x20
        } frame;
    } ootx;

    uint32_t ootx_sensor_channel = 0;

    string ethaddr;

    vector<boost::shared_ptr<TLV493D>> tlv493D0;

    bool executeAction(string actions);

    bool executeActions(vector<string> actions);

    void jointStatusPublisher();

    vector<string> expandBehavior(string name);

    string body_part;

};

/** @} */ // end of group1