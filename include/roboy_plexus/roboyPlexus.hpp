#pragma once

#include <ros/ros.h>
#include <vector>
#include <roboy_plexus/myoControl.hpp>
#include "roboy_plexus/controlActions.hpp"
#include <roboy_plexus/am4096.hpp>
#include <roboy_plexus/Adafruit_LSM9DS1.hpp>
#include <roboy_communication_middleware/ADCvalue.h>
#include <roboy_communication_middleware/ControlMode.h>
#include <roboy_communication_middleware/DarkRoom.h>
#include <roboy_communication_middleware/DarkRoomOOTX.h>
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_communication_middleware/MagneticSensor.h>
#include <roboy_communication_middleware/MotorAngle.h>
#include <roboy_communication_middleware/MotorCalibrationService.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/MotorConfigService.h>
#include <roboy_communication_middleware/SetInt16.h>
#include <roboy_communication_control/Behavior.h>
#include <roboy_communication_control/StartRecordTrajectory.h>
//#include <roboy_communication_control/StopRecordTrajectory.h>
#include <roboy_communication_control/PerformMovement.h>
#include <roboy_communication_control/PerformBehavior.h>
#include <roboy_communication_control/PerformActions.h>
#include <roboy_communication_control/ListItems.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
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


#define NUM_SENSORS 32
#define NUMBER_OF_LOADCELLS 8

#pragma pack(1) // we need this, otherwise the ootx union will be padded and the checksum test fails

using namespace std;
using namespace chrono;
using half_float::half;

static vector<int32_t*> DEFAULT_POINTER_VECTOR;
static vector<int32_t> DEFAULT_VECTOR;
static vector<vector<int32_t>> DEFAULT_VECTOR_VECTOR;
/** @defgroup test The First Group
 *  This is the first group
 *  @{
 */
class RoboyPlexus{
public:
    RoboyPlexus(boost::shared_ptr<MyoControl> myoControl_, vector<int32_t*> &i2c_base = DEFAULT_POINTER_VECTOR,
                vector<vector<int32_t>> &deviceIDs = DEFAULT_VECTOR_VECTOR, int32_t* darkroom_base = nullptr,
                vector<int32_t *> &darkroom_ootx_addr = DEFAULT_POINTER_VECTOR,
                int32_t *adc_base = nullptr);
    ~RoboyPlexus();
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
     * Publishes joint angles
     */
    void jointStatusPublisher();
    /**
     * Publishes motor angles
     */
    void motorAnglePublisher();
    /**
     * Publishes information about motors
     */
    void motorStatusPublisher();
    /**
     * Publishes 3d magnetic information about shoulder joint
     */
    void magneticShoulderJointPublisher();
    /**
     * Callback for motor command
     * @param msg motor command
     */
    void motorCommandCB(const roboy_communication_middleware::MotorCommand::ConstPtr &msg);

    /**
     * Service for changing motor PID parameters
     * @param req PID parameters
     * @param res success
     * @return success
     */
    bool MotorConfigService(roboy_communication_middleware::MotorConfigService::Request  &req,
                            roboy_communication_middleware::MotorConfigService::Response &res);
    /**
     * Service for changing the control mode of motors, perviously set PID parameters are restored
     * @param req control mode
     * @param res
     * @return success
     */
    bool ControlModeService(roboy_communication_middleware::ControlMode::Request  &req,
                            roboy_communication_middleware::ControlMode::Response &res);
    /**
     * Service for motor spring calibration, samples the spring space for a requested amount of time and estimates spring coefficients
     * @param req
     * @param res
     * @return
     */
    bool MotorCalibrationService(roboy_communication_middleware::MotorCalibrationService::Request  &req,
                            roboy_communication_middleware::MotorCalibrationService::Response &res);
    /**
     * Emergency stop service, zeros all PID gains, causing all motors to stop, PID parameters and control mode are restored on release
     * @param req
     * @param res
     * @return
     */
    bool EmergencyStopService(std_srvs::SetBool::Request  &req,
                              std_srvs::SetBool::Response &res);
    /**
     * Motor setpoints trajectory recording callback.
     * @param msg
     * @return
     */
    void StartRecordTrajectoryCB(const roboy_communication_control::StartRecordTrajectory::ConstPtr &msg);
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
    void SaveBehaviorCB(const roboy_communication_control::Behavior &msg);

    /**
     * Service replays the trajectory
     * @param req
     * @param res
     * @return
     */
    bool ReplayTrajectoryService(roboy_communication_control::PerformMovement::Request &req,
                                     roboy_communication_control::PerformMovement::Response &res);

    /**
     * Service executes behavior (a set of trajectories possibly combined with pauses)
     * @param req
     * @param res
     * @return
     */
    bool ExecuteActionsService(roboy_communication_control::PerformActions::Request &req,
                                 roboy_communication_control::PerformActions::Response &res);

    /**
     * Service executes behavior (by name)
     * @param req
     * @param res
     * @return
     */
    bool ExecuteBehaviorService(roboy_communication_control::PerformBehavior::Request &req,
                                roboy_communication_control::PerformBehavior::Response &res);

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
    bool ListExistingItemsService(roboy_communication_control::ListItems::Request &req,
                                  roboy_communication_control::ListItems::Response &res);

    /**
     * Service returns the list of actions in the requested behavior
     * @param req
     * @param res
     * @return
     */
    bool ExpandBehaviorService(roboy_communication_control::ListItems::Request &req,
                                  roboy_communication_control::ListItems::Response &res);


    /**
     * Service sets displacement for all motors
     * @param req
     * @param res
     * @return
     */
    bool SetDisplacementForAll(roboy_communication_middleware::SetInt16::Request &req,
                               roboy_communication_middleware::SetInt16::Request &res);



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
    inline uint32_t reverse(uint32_t b){
        uint32_t a = (uint32_t)((uint8_t)reverse((uint8_t)(b>>24&0xff))<<24|(uint8_t)reverse((uint8_t)(b>>16&0xff))<<16|
                (uint8_t)reverse((uint8_t)(b>>8&0xff))<<8|(uint8_t)reverse((uint8_t)(b&0xff)));
        return a;
    }
    // helper functions for reading the IMU
    bool ADXL345_REG_WRITE(int file, uint8_t address, uint8_t value);
    bool ADXL345_REG_READ(int file, uint8_t address,uint8_t *value);
    bool ADXL345_REG_MULTI_READ(int file, uint8_t readaddr,uint8_t readdata[], uint8_t len);
    bool ADXL345_Init(int file);
    bool ADXL345_IsDataReady(int file);
    bool ADXL345_XYZ_Read(int file, uint16_t szData16[3]);
    bool ADXL345_IdRead(int file, uint8_t *pId);

    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber motorCommand_sub, startRecordTrajectory_sub, stopRecordTrajectory_sub, saveBehavior_sub,
            enablePlayback_sub;
    ros::Publisher motorStatus_pub, darkroom_pub, darkroom_ootx_pub, jointStatus_pub, adc_pub, gsensor_pub,
            motorAngle_pub, magneticSensor_pub;
    ros::ServiceServer motorConfig_srv, controlMode_srv, emergencyStop_srv, motorCalibration_srv,
            replayTrajectory_srv, executeActions_srv, executeBehavior_srv,
            setDisplacementForAll_srv, listExistingTrajectories_srv, listExistingBehaviors_srv, expandBehavior_srv;
    map<int, int> setPoint_backup;
    map<int,map<int,control_Parameters_t>> control_params_backup;
    map<int, int> control_mode, control_mode_backup;
    boost::shared_ptr<MyoControl> myoControl;
    boost::shared_ptr<std::thread> adcThread, darkRoomThread, darkRoomOOTXThread, jointStatusThread, motorStatusThread,
            gsensor_thread, motorAngleThread, magneticsShoulderThread;
    bool keep_publishing = true;
    int32_t *darkroom_base, *adc_base;
    vector<int32_t*> myo_base, i2c_base, darkroom_ootx_addr;
    vector<vector<int32_t>> deviceIDs;

    bool emergency_stop = false;
    vector<boost::shared_ptr<AM4096>> jointAngle; // joint angle sensors
    vector<boost::shared_ptr<A1335>> motorAngle; // motor angle of new motor units
    int file;
    const char *filename = "/dev/i2c-0";
    uint8_t id;
    bool bSuccess;
    const int mg_per_digi = 4;
    uint16_t szXYZ[3];
    int cnt=0, max_cnt=0;

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
        }frame;
    }ootx;

    uint32_t ootx_sensor_channel = 0;

    string ethaddr;

    boost::shared_ptr<TLV493D> tlv493D0[2];

    bool executeActions(vector<string> actions);
    vector<string> expandBehavior(string name);

};

/** @} */ // end of group1