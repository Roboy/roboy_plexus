#include <roboy_plexus/roboyPlexus.hpp>
#include <roboy_plexus/myoControl.hpp>

#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<roboy_communication_control::StartRecordTrajectoryAction> Server;

void execute(const roboy_communication_control::StartRecordTrajectoryGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
    // Do lots of awesome groundbreaking robot stuff here
    as->setSucceeded();
}

int main2(int argc, char** argv)
{
    ros::init(argc, argv, "do_dishes_server");
    ros::NodeHandle n;
    Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();
    return 0;
}

RoboyPlexus::RoboyPlexus(vector<int32_t *> &myo_base, vector<int32_t *> &i2c_base,
                         vector<vector<int32_t>> &deviceIDs, int32_t *darkroom_base,
                         vector<int32_t *> &darkroom_ootx_addr,
                         int32_t *adc_base) :
        myo_base(myo_base), i2c_base(i2c_base), deviceIDs(deviceIDs), darkroom_base(darkroom_base),
        darkroom_ootx_addr(darkroom_ootx_addr), adc_base(adc_base){
    ifstream ifile("/sys/class/net/eth0/address");
    ifile >> ethaddr;
    ifile.close();
    string node_name = "roboy_fpga_" + ethaddr;
    replace(node_name.begin(), node_name.end(), ':', '_');

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, node_name);
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(2));
    spinner->start();

    myoControl = boost::shared_ptr<MyoControl>(new MyoControl(myo_base, adc_base));

    motorCommand_sub = nh->subscribe("/roboy/middleware/MotorCommand", 1, &RoboyPlexus::motorCommandCB, this);
    motorConfig_srv = nh->advertiseService("/roboy/middleware/MotorConfig", &RoboyPlexus::MotorConfigService, this);
    controlMode_srv = nh->advertiseService("/roboy/middleware/ControlMode", &RoboyPlexus::ControlModeService, this);
    emergencyStop_srv = nh->advertiseService("/roboy/middleware/EmergencyStop", &RoboyPlexus::EmergencyStopService,
                                             this);
    motorCalibration_srv = nh->advertiseService("/roboy/middleware/MotorCalibration",
                                                &RoboyPlexus::MotorCalibrationService, this);
    setDisplacementForAll_srv = nh->advertiseService("/roboy/middleware/SetDisplacementForAll",
                                                     &RoboyPlexus::SetDisplacementForAll, this);
//    startRecordTrajectory_srv = nh->advertiseService("roboy/middleware/StartRecordTrajectory",
//                                                     &RoboyPlexus::StartRecordTrajectoryService, this);
//    stopRecordTrajectory_srv = nh->advertiseService("roboy/middleware/StopRecordTrajectory",
//                                                     &RoboyPlexus::StopRecordTrajectoryService, this);

//    actionlib::SimpleActionServer<roboy_communication_control::StartRecordTrajectoryAction> server(*nh, "do_dishes",
//                                            false);
//    boost::bind(&RoboyPlexus::StartRecordTrajectoryService, this)
//    server.start();

    replayTrajectory_srv = nh->advertiseService("roboy/middleware/ReplayTrajectory",
                                                    &RoboyPlexus::ReplayTrajectoryService, this);
    execureBehavior_srv = nh->advertiseService("roboy/middleware/ExecuteBehavior",
                                               &RoboyPlexus::ExecuteBehaviorService, this);
    listExistingTrajectories_srv = nh->advertiseService("roboy/middleware/ListExistingTrajectories",
                                                        &RoboyPlexus::ListExistingTrajectories, this);

    motorStatus_pub = nh->advertise<roboy_communication_middleware::MotorStatus>("/roboy/middleware/MotorStatus", 1);
    motorAngle_pub = nh->advertise<roboy_communication_middleware::MotorAngle>("/roboy/middleware/MotorAngle", 1);
    darkroom_pub = nh->advertise<roboy_communication_middleware::DarkRoom>("/roboy/middleware/DarkRoom/sensors", 1);
    darkroom_ootx_pub = nh->advertise<roboy_communication_middleware::DarkRoomOOTX>("/roboy/middleware/DarkRoom/ootx", 1);
    jointStatus_pub = nh->advertise<roboy_communication_middleware::JointStatus>("/roboy/middleware/JointStatus", 1);
    adc_pub = nh->advertise<roboy_communication_middleware::ADCvalue>("/roboy/middleware/LoadCells", 1);
    gsensor_pub = nh->advertise<sensor_msgs::Imu>("/roboy/middleware/imu0", 1);
    magneticSensor_pub = nh->advertise<roboy_communication_middleware::MagneticSensor>("/roboy/middleware/MagneticSensor",1, this);

    motorStatusThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::motorStatusPublisher, this));
    motorStatusThread->detach();

    // TODO: make the active interfaces ros parameter server variables
    if (darkroom_base != nullptr) {
        darkRoomThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::darkRoomPublisher, this));
        darkRoomThread->detach();
    }

    if (!darkroom_ootx_addr.empty()) {
        darkRoomOOTXThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::darkRoomOOTXPublisher, this));
        darkRoomOOTXThread->detach();
    }


    if (adc_base != nullptr) {
        adcThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::adcPublisher, this));
        adcThread->detach();
    }

//    if(!i2c_base.empty()) {
//        if(i2c_base.size()>0) {
//            motorAngle.push_back(boost::shared_ptr<A1335>(new A1335(i2c_base[0], deviceIDs[0])));
//            motorAngleThread = boost::shared_ptr<std::thread>(
//                    new std::thread(&RoboyPlexus::motorAnglePublisher, this));
//            motorAngleThread->detach();
//            if(i2c_base.size()==2){
//                jointAngle.push_back(boost::shared_ptr<AM4096>(new AM4096(i2c_base[1], deviceIDs[1])));
//                jointStatusThread = boost::shared_ptr<std::thread>(
//                        new std::thread(&RoboyPlexus::jointStatusPublisher, this));
//                jointStatusThread->detach();
//            }
//        }
//    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++)
        control_mode[motor] = DISPLACEMENT;

    myoControl->allToDisplacement(0);

//    // open i2c bus for gsensor
//    if ((file = open(filename, O_RDWR)) < 0) {
//        ROS_ERROR("Failed to open the i2c bus of gsensor");
//    }
//
//    // init
//    // gsensor i2c address: 101_0011
//    int addr = 0b01010011;
//    if (ioctl(file, I2C_SLAVE, addr) < 0) {
//        ROS_ERROR("Failed to acquire bus access and/or talk to slave");
//    }else{
//        // configure accelerometer as +-2g and start measure
//        bSuccess = ADXL345_Init(file);
//        if (bSuccess){
//            // dump chip id
//            bSuccess = ADXL345_IdRead(file, &id);
//            if (bSuccess){
//                ROS_INFO("gsensor chip_id=%02Xh", id);
//                gsensor_thread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::gsensorPublisher, this));
//                gsensor_thread->detach();
//            }
//        }
//    }

    // Look in the device's user manual for allowed addresses! (Table 6)
    vector<uint8_t> deviceaddress0 = {0b1001010, 0b1001110};//
    vector<uint8_t> deviceaddress1 = {0b1001010};//
    vector<int> devicepins0 = {0,1};
    vector<int> devicepins1 = {0};
    tlv493D0[0].reset(new TLV493D(i2c_base[0], deviceaddress0, devicepins0));
    tlv493D0[1].reset(new TLV493D(i2c_base[1], deviceaddress1, devicepins1));

    magneticsShoulderThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::magneticShoulderJointPublisher, this));
    magneticsShoulderThread->detach();

    ROS_INFO("roboy plexus initialized");
}

RoboyPlexus::~RoboyPlexus() {
    keep_publishing = false;

    if (motorStatusThread->joinable())
        motorStatusThread->join();
    if(!i2c_base.empty()) {
        if (jointStatusThread->joinable())
            jointStatusThread->join();
    }
    if(bSuccess) {
        if (gsensor_thread->joinable())
            gsensor_thread->join();
    }
    if (adc_base != nullptr) {
        if (adcThread->joinable())
            adcThread->join();
    }
    if (darkroom_base != nullptr) {
        if (darkRoomThread->joinable())
            darkRoomThread->join();
    }
    if (!darkroom_ootx_addr.empty()) {
        if (darkRoomOOTXThread->joinable())
            darkRoomOOTXThread->join();
    }

    if (file)
        close(file);
}

void RoboyPlexus::gsensorPublisher() {
    ros::Rate rate(100);
    while (keep_publishing) {
        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "calibrationCube";
        msg.orientation.w = 1;
        msg.linear_acceleration_covariance = {
                0.01, 0, 0,
                0, 0.01, 0,
                0, 0, 0.01
        };
        if (ADXL345_IsDataReady(file)){
            bSuccess = ADXL345_XYZ_Read(file, szXYZ);
            if (bSuccess){
                ROS_DEBUG_THROTTLE(1,"X=%d mg, Y=%d mg, Z=%d mg", (int16_t)szXYZ[0]*mg_per_digi, (int16_t)szXYZ[1]*mg_per_digi, (int16_t)szXYZ[2]*mg_per_digi);
                msg.linear_acceleration.x = (int16_t)szXYZ[0]*mg_per_digi;
                msg.linear_acceleration.y = (int16_t)szXYZ[1]*mg_per_digi;
                msg.linear_acceleration.z = (int16_t)szXYZ[2]*mg_per_digi;
                // convert to m/s^2
                msg.linear_acceleration.x *= 0.00981;
                msg.linear_acceleration.y *= 0.00981;
                msg.linear_acceleration.z *= 0.00981;
            }
            gsensor_pub.publish(msg);
        }
        rate.sleep();
    }
}

void RoboyPlexus::adcPublisher() {
    ros::Rate rate(100);
    while (keep_publishing) {
        roboy_communication_middleware::ADCvalue msg;
        for (uint i = 0; i < NUMBER_OF_LOADCELLS; i++) {
            uint32_t adcvalue;
            float val = myoControl->getWeight(i, adcvalue);
            msg.adc_value.push_back(adcvalue);
            msg.load.push_back(val);
        }
        adc_pub.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::darkRoomPublisher() {
    ros::Rate rate(240);
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while (keep_publishing) {
        uint active_sensors = 0;
        roboy_communication_middleware::DarkRoom msg;
        msg.objectID = ethaddr;
        for (uint i = 0; i < NUM_SENSORS; i++) {
            int32_t val = IORD(darkroom_base, i);
            high_resolution_clock::time_point t1 = high_resolution_clock::now();
            microseconds time_span = duration_cast<microseconds>(t1 - t0);
            msg.timestamp.push_back(time_span.count());
            msg.sensor_value.push_back(val);
            if ((val >> 29) & 0x1)//valid
                active_sensors++;
        }
        darkroom_pub.publish(msg);
        ROS_INFO_THROTTLE(10, "lighthouse sensors active %d/%d", active_sensors, NUM_SENSORS);
        rate.sleep();
    }
}

void RoboyPlexus::darkRoomOOTXPublisher() {
    // ootx frames are 17bits preamble + 271 bits payload + 32 bits checksum = 320 bits.
    // At 120Hz motor speed, this is ~2.6 seconds per frame. Lets set the update rate to 5 seconds then.
    ros::Rate rate(1/5.0);
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while (keep_publishing) {
        bool successfully_decoded_ootx = true;
        for(uint lighthouse=0;lighthouse<2;lighthouse++){
            for(uint decoder=0;decoder<darkroom_ootx_addr.size();decoder++){
                // TODO: remove reverse and have the fpga convert it directly
                ootx.frame.fw_version = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],(uint32_t)(18*lighthouse+0)))&0xFFFF);
                ootx.frame.ID = reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+1));
                ootx.frame.fcal_0_phase = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+2))&0xFFFF);
                ootx.frame.fcal_1_phase = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+3))&0xFFFF);
                ootx.frame.fcal_0_tilt = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+4))&0xFFFF);
                ootx.frame.fcal_1_tilt = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+5))&0xFFFF);
                ootx.frame.unlock_count = (uint8_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+6))&0xFF);
                ootx.frame.hw_version = (uint8_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+7))&0xFF);
                ootx.frame.fcal_0_curve = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+8))&0xFFFF);
                ootx.frame.fcal_1_curve = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+9))&0xFFFF);
                uint32_t acc = reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+10));
                ootx.frame.accel_dir_x = ((int8_t)(acc>>0&0xFF));
                ootx.frame.accel_dir_y = ((int8_t)(acc>>8&0xFF));
                ootx.frame.accel_dir_z = ((int8_t)(acc>>16&0xFF));
                ootx.frame.fcal_0_gibphase = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+11))&0xFFFF);
                ootx.frame.fcal_1_gibphase = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+12))&0xFFFF);
                ootx.frame.fcal_0_gibmag = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+13))&0xFFFF);
                ootx.frame.fcal_1_gibmag = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+14))&0xFFFF);
                ootx.frame.mode = (uint8_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+15))&0xFF);
                ootx.frame.faults = (uint8_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+16))&0xFF);

                uint32_t crc32checksum = reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+17));

                CRC32 crc;
                for(int i=0; i<33;i++) {
                    crc.update(ootx.data[i]);
                }

                uint32_t crc32checksumCalculated = crc.finalize();
                if(crc.finalize() == crc32checksum){ // kinda paranoid right...?!
                    roboy_communication_middleware::DarkRoomOOTX msg;
                    msg.lighthouse = lighthouse;
                    msg.fw_version = ootx.frame.fw_version;
                    msg.ID = ootx.frame.ID;
                    half fcal_0_phase, fcal_1_phase, fcal_0_tilt, fcal_1_tilt;
                    fcal_0_phase.data_ = ootx.frame.fcal_0_phase;
                    fcal_1_phase.data_ = ootx.frame.fcal_1_phase;
                    fcal_0_tilt.data_ = ootx.frame.fcal_0_tilt;
                    fcal_1_tilt.data_ = ootx.frame.fcal_1_tilt;
                    msg.fcal_0_phase = fcal_0_phase;
                    msg.fcal_1_phase = fcal_1_phase;
                    msg.fcal_0_tilt = fcal_0_tilt;
                    msg.fcal_1_tilt = fcal_1_tilt;
                    msg.unlock_count = fcal_1_tilt;
                    msg.hw_version = ootx.frame.hw_version;
                    half fcal_0_curve, fcal_1_curve;
                    fcal_0_curve.data_ = ootx.frame.fcal_0_curve;
                    fcal_1_curve.data_ = ootx.frame.fcal_1_curve;
                    msg.fcal_0_curve = fcal_0_curve;
                    msg.fcal_1_curve = fcal_1_curve;
                    // max/min value of acceleration is 127/-127, so we divide to get an orientation vector
                    msg.accel_dir_x = ootx.frame.accel_dir_x/127.0f;
                    msg.accel_dir_y = ootx.frame.accel_dir_y/127.0f;
                    msg.accel_dir_z = ootx.frame.accel_dir_z/127.0f;
                    half fcal_0_gibphase, fcal_1_gibphase, fcal_0_gibmag, fcal_1_gibmag;
                    fcal_0_gibphase.data_ = ootx.frame.fcal_0_gibphase;
                    fcal_1_gibphase.data_ = ootx.frame.fcal_1_gibphase;
                    fcal_0_gibmag.data_ = ootx.frame.fcal_0_gibmag;
                    fcal_1_gibmag.data_ = ootx.frame.fcal_1_gibmag;
                    msg.fcal_0_gibphase = fcal_0_gibphase;
                    msg.fcal_1_gibphase = fcal_1_gibphase;
                    msg.fcal_0_gibmag = fcal_0_gibmag;
                    msg.fcal_1_gibmag = fcal_1_gibmag;
                    msg.mode = ootx.frame.mode;
                    msg.faults = ootx.frame.faults;
                    msg.crc32 = crc32checksum;

                    darkroom_ootx_pub.publish(msg);
                }else{
                    successfully_decoded_ootx = false;

                    stringstream str;
                    str << "received ootx frame"  << " with crc " << crc32checksum
                        << " which does not match the calculated: " << crc32checksumCalculated << endl;
                    str << "fw_version:          " << ootx.frame.fw_version << endl;
                    str << "ID:                  " << ootx.frame.ID << endl;
                    str << "fcal_0_phase:        " << ootx.frame.fcal_0_phase << endl;
                    str << "fcal_1_phase:        " << ootx.frame.fcal_1_phase << endl;
                    str << "fcal_0_tilt:         " << ootx.frame.fcal_0_tilt << endl;
                    str << "fcal_1_tilt:         " << ootx.frame.fcal_1_tilt << endl;
                    str << "unlock_count:        " << (uint)ootx.frame.unlock_count << endl;
                    str << "hw_version:          " << (uint)ootx.frame.hw_version << endl;
                    str << "fcal_0_curve:        " << ootx.frame.fcal_0_curve << endl;
                    str << "fcal_1_curve:        " << ootx.frame.fcal_1_curve << endl;
                    str << "accel_dir_x:         " << (int)ootx.frame.accel_dir_x << endl;
                    str << "accel_dir_y:         " << (int)ootx.frame.accel_dir_y << endl;
                    str << "accel_dir_z:         " << (int)ootx.frame.accel_dir_z << endl;
                    str << "fcal_0_gibphase:     " << ootx.frame.fcal_0_gibphase << endl;
                    str << "fcal_1_gibphase:     " << ootx.frame.fcal_1_gibphase << endl;
                    str << "fcal_0_gibmag:       " << ootx.frame.fcal_0_gibmag << endl;
                    str << "fcal_1_gibmag:       " << ootx.frame.fcal_1_gibmag << endl;
                    str << "mode:                " << (uint)ootx.frame.mode << endl;
                    str << "faults:              " << (uint)ootx.frame.faults << endl;
                    ROS_DEBUG_STREAM( str.str() );
                }
            }
        }
        if(!successfully_decoded_ootx){
            // no valid ootx frame decoded, lets switch to another sensor channel
            ootx_sensor_channel++;
            if(ootx_sensor_channel>NUM_SENSORS)
                ootx_sensor_channel = 0;
            for(uint decoder=0;decoder<darkroom_ootx_addr.size();decoder++) {
                IOWR(darkroom_ootx_addr[decoder], 0, ootx_sensor_channel);
            }
        }
        rate.sleep();
    }
}


void RoboyPlexus::jointStatusPublisher() {
    ros::Rate rate(50);
    while (keep_publishing) {
        roboy_communication_middleware::JointStatus msg;
        for (uint i = 0; i < jointAngle.size(); i++) {
            jointAngle[i]->readAbsAngle(msg.absAngles);
            jointAngle[i]->readRelAngle(msg.relAngles);
            jointAngle[i]->readMagnetStatus(msg.tooFar, msg.tooClose);
            jointAngle[i]->readTacho(msg.tacho);
            jointAngle[i]->readAgcGain(msg.agcGain);
        }
        jointStatus_pub.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::motorAnglePublisher() {
    ros::Rate rate(50);
    while (keep_publishing) {
        roboy_communication_middleware::MotorAngle msg;
        vector<A1335State> state;
        motorAngle[0]->readAngleData(state);
        stringstream str;
        for(auto s:state){
            str << "Motor Angle Sensor on i2C address " << (int)s.address << " is " << (s.isOK?"ok":"not ok") << endl;
            str << "angle:         " << s.angle << endl;
            str << "angle_flags:   " << motorAngle[0]->decodeFlag(s.angle_flags,ANGLES_FLAGS) << endl;
            str << "err_flags:     " << motorAngle[0]->decodeFlag(s.err_flags,ERROR_FLAGS) << endl;
            str << "fieldStrength: " << s.fieldStrength << endl;
            str << "status_flags:  " << motorAngle[0]->decodeFlag(s.status_flags,STATUS_FLAGS) << endl;
            str << "xerr_flags:    " << motorAngle[0]->decodeFlag(s.xerr_flags,XERROR_FLAGS) << endl;
            msg.angles.push_back(s.angle);
            msg.magneticFieldStrength.push_back(s.fieldStrength);
//            msg.temperature.push_back(s.temp);
        }
        ROS_DEBUG_STREAM_THROTTLE(5,str.str());
        motorAngle_pub.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::motorStatusPublisher() {
    ros::Rate rate(200);
    while (keep_publishing) {
        roboy_communication_middleware::MotorStatus msg2;
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            msg2.pwmRef.push_back(myoControl->getPWM(motor));
            msg2.position.push_back(myoControl->getPosition(motor));
            msg2.velocity.push_back(myoControl->getVelocity(motor));
            msg2.displacement.push_back(myoControl->getDisplacement(motor));
            msg2.current.push_back(myoControl->getCurrent(motor));
        }
        motorStatus_pub.publish(msg2);
        rate.sleep();
    }
}

void RoboyPlexus::magneticShoulderJointPublisher(){
    ros::Rate rate(200);
    while (keep_publishing) {
        roboy_communication_middleware::MagneticSensor msg;
        tlv493D0[0]->read(msg.x,msg.y,msg.z);
        tlv493D0[1]->read(msg.x,msg.y,msg.z);
        magneticSensor_pub.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::motorCommandCB(const roboy_communication_middleware::MotorCommand::ConstPtr &msg) {
    uint i = 0;
    for (auto motor:msg->motors) {
        switch(control_mode[motor]){
            case POSITION:
                myoControl->setPosition(motor, msg->setPoints[i]);
                break;
            case VELOCITY:
                myoControl->setVelocity(motor, msg->setPoints[i]);
                break;
            case DISPLACEMENT:
                myoControl->setDisplacement(motor, msg->setPoints[i]);
                break;
            case FORCE:
                myoControl->setDisplacement(motor, msg->setPoints[i]);
                break;
        }

        i++;
    }
}

bool RoboyPlexus::MotorConfigService(roboy_communication_middleware::MotorConfigService::Request &req,
                                     roboy_communication_middleware::MotorConfigService::Response &res) {
    if (req.setPoints.size() != req.config.motors.size()) {
        ROS_ERROR("the number of setpoints do not match the number of motor configs");
        return false;
    }

    ROS_INFO("serving motor config service for %d motors", req.config.motors.size());
    control_Parameters_t params;
    uint i = 0;
    for (auto motor:req.config.motors) {
        if (req.config.control_mode[i] < POSITION || req.config.control_mode[i] > DISPLACEMENT) {
            ROS_ERROR("trying to set an invalid control mode %d, available control modes: "
                              "[0]Position [1]Velocity [2]Displacement", req.config.control_mode[i]);
            continue;
        }
        params.outputPosMax = req.config.outputPosMax[i];
        params.outputNegMax = req.config.outputNegMax[i];
        params.spPosMax = req.config.spPosMax[i];
        params.spNegMax = req.config.spNegMax[i];
        params.Kp = req.config.Kp[i];
        params.Ki = req.config.Ki[i];
        params.Kd = req.config.Kd[i];
        params.forwardGain = req.config.forwardGain[i];
        params.deadBand = req.config.deadBand[i];
        params.IntegralPosMax = req.config.IntegralPosMax[i];
        params.IntegralNegMax = req.config.IntegralNegMax[i];
        myoControl->changeControl(motor, req.config.control_mode[i], params, req.setPoints[i]);
        ROS_INFO("setting motor %d to control mode %d with setpoint %d", motor, req.config.control_mode[i],
                 req.setPoints[i]);
        control_mode[motor] = req.config.control_mode[i];
        i++;
    }
    return true;
}

bool RoboyPlexus::ControlModeService(roboy_communication_middleware::ControlMode::Request &req,
                                     roboy_communication_middleware::ControlMode::Response &res) {
    if (!emergency_stop) {
        uint i = 0;
        switch(req.control_mode){
            case POSITION:
                ROS_INFO("switch to POSITION control");
                for (auto &mode:control_mode)
                    mode.second = POSITION;
                myoControl->allToPosition(req.setPoint);
                break;
            case VELOCITY:
                ROS_INFO("switch to VELOCITY control");
                for (auto &mode:control_mode)
                    mode.second = VELOCITY;
                myoControl->allToVelocity(req.setPoint);
                break;
            case DISPLACEMENT:
                ROS_INFO("switch to DISPLACEMENT control");
                for (auto &mode:control_mode)
                    mode.second = DISPLACEMENT;
                myoControl->allToDisplacement(req.setPoint);
                break;
            default:
                return false;
        }
        return true;
    } else {
        ROS_WARN("emergency stop active, can NOT change control mode");
        return false;
    }
}

bool RoboyPlexus::MotorCalibrationService(roboy_communication_middleware::MotorCalibrationService::Request &req,
                                          roboy_communication_middleware::MotorCalibrationService::Response &res) {
    if (!emergency_stop) {
        ROS_INFO("serving motor calibration service for motor %d", req.motor);
        myoControl->estimateSpringParameters(req.motor, req.degree, res.estimated_spring_parameters,
                                             req.timeout, req.numberOfDataPoints, req.displacement_min,
                                             req.displacement_max, res.load, res.displacement);
        cout << "coefficients:\t";
        for(uint i=0; i<req.degree; i++){
            cout << res.estimated_spring_parameters[i] << "\t";
        }
        cout << endl;
        return true;
    } else {
        return false;
    }
}

bool RoboyPlexus::EmergencyStopService(std_srvs::SetBool::Request &req,
                                       std_srvs::SetBool::Response &res) {

    if (req.data == 1) {
        ROS_INFO("emergency stop service called");
        control_mode_backup = control_mode;
        control_params_backup = myoControl->control_params;
        control_Parameters_t params;
        params.Kp = 0;
        params.Ki = 0;
        params.Kd = 0;
        params.outputPosMax = 0;
        params.outputNegMax = 0;
        params.forwardGain = 0;
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            myoControl->changeControl(motor, DISPLACEMENT, params);
        }
        emergency_stop = true;
    } else {
        ROS_INFO("resuming normal operation");
        uint motor = 0;
        for (auto &params:control_params_backup) {
            myoControl->changeControl(motor, control_mode_backup[motor], params.second[control_mode_backup[motor]]);
            motor++;
        }
        emergency_stop = false;
    }
    return true;
}

bool RoboyPlexus::SetDisplacementForAll(roboy_communication_middleware::SetInt16::Request &req,
                                        roboy_communication_middleware::SetInt16::Request &res) {
    myoControl->allToDisplacement(req.setpoint);
    return true;
}

void RoboyPlexus::StartRecordTrajectoryService(const roboy_communication_control::StartRecordTrajectoryGoalConstPtr& goal,
                   actionlib::SimpleActionServer<roboy_communication_control::StartRecordTrajectoryAction>* actionServer ) {

    float samplingTime = 5; // 200 Hz is the fastest update rate for the motors
    string name = goal->name;
    vector<int> idList(begin(goal->idList), end(goal->idList));
    map<int, vector<float>> trajectories;

    myoControl->startRecordTrajectories(samplingTime, trajectories, idList, name);

//    std::ofstream outfile;
//    if (name.empty()) {
//        time_t rawtime;
//        struct tm *timeinfo;
//        time(&rawtime);
//        timeinfo = localtime(&rawtime);
//        char str[200];
//        sprintf(str, "recording_%s.log",
//                asctime(timeinfo));
//        name = str;
//    }
//
//    outfile.open(name);
//    if (outfile.is_open()) {
//        outfile << "<?xml version=\"1.0\" ?>"
//                << std::endl;
//        uint m = 0;
//        char motorname[10];
//        for (uint m = 0; m < idList.size(); m++) {
//            sprintf(motorname, "motor%d", idList[m]);
//            outfile << "<trajectory motorid=\"" << idList[m] << "\" controlmode=\""
//                    << POSITION << "\" samplingTime=\"" << samplingTime * 1000.0f << "\">"
//                    << std::endl;
//            outfile << "<waypointlist>" << std::endl;
//            for (uint i = 0; i < trajectories[idList[m]].size(); i++)
//                outfile << trajectories[idList[m]][i] << " ";
//            outfile << "</waypointlist>" << std::endl;
//            outfile << "</trajectory>" << std::endl;
//        }
//        outfile << "</roboybehavior>" << std::endl;
//        outfile.close();
//    }

//    res.trajectories.layout.dim[0].label = "motor_id";
//    res.trajectories.layout.dim[0].size = idList.size();
//    res.trajectories.layout.dim[0].stride = (trajectories[idList[0]].size()+1)*idList.size();
//    res.trajectories.layout.dim[1].label = "setpoint";
//    res.trajectories.layout.dim[1].size = trajectories[idList[0]].size()+1;
//    res.trajectories.layout.dim[1].stride = trajectories[idList[0]].size()+1;
//
//    for (auto const& motor : trajectories)
//    {
//        res.trajectories.data.push_back(motor.first); // motor id
//        res.trajectories.data.reserve(res.trajectories.data.size() + distance(motor.second.begin(),motor.second.end()));
//        res.trajectories.data.insert(res.trajectories.data.end(),motor.second.begin(),motor.second.end()); // setpoints
//    }

   actionServer->setSucceeded();

}

void RoboyPlexus::StopRecordTrajectoryService(const roboy_communication_control::StopRecordTrajectoryGoalConstPtr& goal,
                  actionlib::SimpleActionServer<roboy_communication_control::StopRecordTrajectoryAction>* actionServer ) {
    myoControl->stopRecordTrajectories();
    actionServer->setSucceeded();
}

bool RoboyPlexus::ReplayTrajectoryService(roboy_communication_control::PerformMovement::Request &req,
                             roboy_communication_control::PerformMovement::Response &res) {
    res.success = myoControl->playTrajectory(req.value.c_str());
    return res.success;
}

bool RoboyPlexus::ExecuteBehaviorService(roboy_communication_control::PerformBehavior::Request &req,
                                         roboy_communication_control::PerformBehavior::Response &res) {
    for (string actionName: req.actions) {
        if (actionName.find("pause") != std::string::npos) {
            string delimiter = "_";
            int pause = stoi(actionName.substr(0, actionName.find(delimiter)));
            ros::Duration(pause).sleep();
            res.success = true && res.success;
        }
        else {
            res.success = myoControl->playTrajectory(actionName.c_str()) && res.success;
        }
    }
}

bool RoboyPlexus::ListExistingTrajectories(roboy_communication_control::ListTrajectories::Request &req,
                              roboy_communication_control::ListTrajectories::Response &res) {
    // read filenames in the specified folder
    DIR* dirp = opendir(req.folder.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        res.trajectories.push_back(dp->d_name);
    }
    closedir(dirp);
    return true;
}

bool RoboyPlexus::ADXL345_REG_WRITE(int file, uint8_t address, uint8_t value){
    bool bSuccess = false;
    uint8_t szValue[2];
    // write to define register
    szValue[0] = address;
    szValue[1] = value;
    if (write(file, &szValue, sizeof(szValue)) == sizeof(szValue)){
        bSuccess = true;
    }
    return bSuccess;
}

bool RoboyPlexus::ADXL345_REG_READ(int file, uint8_t address,uint8_t *value){
    bool bSuccess = false;
    uint8_t Value;
    // write to define register
    if (write(file, &address, sizeof(address)) == sizeof(address)){

        // read back value
        if (read(file, &Value, sizeof(Value)) == sizeof(Value)){
            *value = Value;
            bSuccess = true;
        }
    }
    return bSuccess;
}

bool RoboyPlexus::ADXL345_REG_MULTI_READ(int file, uint8_t readaddr,uint8_t readdata[], uint8_t len){
    bool bSuccess = false;
    // write to define register
    if (write(file, &readaddr, sizeof(readaddr)) == sizeof(readaddr)){
        // read back value
        if (read(file, readdata, len) == len){
            bSuccess = true;
        }
    }
    return bSuccess;
}

bool RoboyPlexus::ADXL345_Init(int file){
    bool bSuccess;

    // +- 2g range, 10 bits
    bSuccess = ADXL345_REG_WRITE(file, ADXL345_REG_DATA_FORMAT, XL345_RANGE_2G | XL345_FULL_RESOLUTION);

    //Output Data Rate: 800Hz
    if (bSuccess){
        bSuccess = ADXL345_REG_WRITE(file, ADXL345_REG_BW_RATE, XL345_RATE_100);
    }

    //INT_Enable: Data Ready
    if (bSuccess){
        bSuccess = ADXL345_REG_WRITE(file, ADXL345_REG_INT_ENALBE, XL345_DATAREADY);
    }

    // stop measure
    if (bSuccess){
        bSuccess = ADXL345_REG_WRITE(file, ADXL345_REG_POWER_CTL, XL345_STANDBY);
    }

    // start measure
    if (bSuccess){
        bSuccess = ADXL345_REG_WRITE(file, ADXL345_REG_POWER_CTL, XL345_MEASURE);

    }


    return bSuccess;

}

bool RoboyPlexus::ADXL345_IsDataReady(int file){
    bool bReady = false;
    uint8_t data8;

    if (ADXL345_REG_READ(file, ADXL345_REG_INT_SOURCE,&data8)){
        if (data8 & XL345_DATAREADY)
            bReady = true;
    }

    return bReady;
}

bool RoboyPlexus::ADXL345_XYZ_Read(int file, uint16_t szData16[3]){
    bool bPass;
    uint8_t szData8[6];
    bPass = ADXL345_REG_MULTI_READ(file, 0x32, (uint8_t *)&szData8, sizeof(szData8));
    if (bPass){
        szData16[0] = (szData8[1] << 8) | szData8[0];
        szData16[1] = (szData8[3] << 8) | szData8[2];
        szData16[2] = (szData8[5] << 8) | szData8[4];
    }

    return bPass;
}

bool RoboyPlexus::ADXL345_IdRead(int file, uint8_t *pId){
    bool bPass;
    bPass = ADXL345_REG_READ(file, ADXL345_REG_DEVID, pId);

    return bPass;
}