#include <roboy_plexus/roboyPlexus.hpp>


RoboyPlexus::RoboyPlexus(MyoControlPtr myoControl, vector<int32_t *> &myo_base, vector<int32_t *> &i2c_base,
                         int32_t *darkroom_base,
                         vector<int32_t *> &darkroom_ootx_addr, int32_t *adc_base, int32_t *switches_base, int32_t *h2p_lw_bike_addr) :
        myo_base(myo_base), i2c_base(i2c_base), darkroom_base(darkroom_base), darkroom_ootx_addr(darkroom_ootx_addr),
        adc_base(adc_base), myoControl(myoControl), switches_base(switches_base){

    bike_addr = h2p_lw_bike_addr;
    id = IORD(switches_base, 0) & 0x7;
//    string body_part;
    switch (id) {
        case HEAD:
            body_part = "head";
            break;
        case SHOULDER_LEFT:
            body_part = "shoulder_left";
            break;
        case SPINE_LEFT:
            body_part = "spine_left";
            break;
        case SHOULDER_RIGHT:
            body_part = "shoulder_right";
            break;
        case SPINE_RIGHT:
            body_part = "spine_right";
            break;
        default:
            body_part = "unknown";
    }

    ROS_INFO("switches %d, I'm the %s", id, body_part.c_str());

    ifstream ifile("/sys/class/net/eth0/address");
    ifile >> ethaddr;
    ifile.close();
    string node_name = "roboy_fpga_" + body_part + "_" + ethaddr;
    replace(node_name.begin(), node_name.end(), ':', '_');

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    switch (id) {
        case HEAD: {
            motorAngle_pub = nh->advertise<roboy_middleware_msgs::MotorAngle>("/roboy/middleware/MotorAngle", 1);
            vector<uint8_t> deviceIDs = {0xC, 0xD, 0xF, 0xC, 0xE, 0xD};
            vector<int32_t> gearBoxRatio = {62, 62, 62, 62, 62, 62};
            vector<int32_t> encoderMultiplier = {4, 4, 4, 4, 4, 4};
            if (!myoControl->configureMyoBricks(myo_bricks[HEAD], deviceIDs, encoderMultiplier, gearBoxRatio)) {
                ROS_ERROR("could not configure myoBricks");
            }else {
                motorAngleThread = boost::shared_ptr<std::thread>(
                        new std::thread(&RoboyPlexus::motorAnglePublisher, this));
                motorAngleThread->detach();
                myobrick_calibration_srv = nh->advertiseService("/roboy/middleware/head/MyoBrickMotorCalibration",
                                                                &RoboyPlexus::MyoBrickCalibrationService, this);
            }
            break;
        }

        case SPINE_LEFT: {
            { // start motor angle publisher for three myoBricks
                motorAngle_pub = nh->advertise<roboy_middleware_msgs::MotorAngle>("/roboy/middleware/MotorAngle",
                                                                                           1);
                vector<uint8_t> deviceIDs = {0xE, 0xC};
//                vector<uint8_t> motorIDs = {0,1,2};
                vector<int32_t> gearBoxRatio = {62, 62};
                vector<int32_t> encoderMultiplier = {1, 1};
                if (!myoControl->configureMyoBricks(myo_bricks[SPINE_LEFT], deviceIDs, encoderMultiplier, gearBoxRatio)) {
                    ROS_ERROR("could not configure myoBricks, make sure the correct fpga image is used");
                }else {
                    motorAngleThread = boost::shared_ptr<std::thread>(
                            new std::thread(&RoboyPlexus::motorAnglePublisher, this));
                    motorAngleThread->detach();
                }
            }
            break;
        }
        case SPINE_RIGHT: {
            { // start motor angle publisher for three myoBricks
                motorAngle_pub = nh->advertise<roboy_middleware_msgs::MotorAngle>("/roboy/middleware/MotorAngle",
                                                                                           1);
                vector<uint8_t> deviceIDs = {0xC, 0xD, 0xF, 0xC, 0xE, 0xD};
                vector<int32_t> gearBoxRatio = {62, 62, 62, 62, 62, 62};
                vector<int32_t> encoderMultiplier = {4, 4, 4, 4, 4, 4};
                if (!myoControl->configureMyoBricks(myo_bricks[SPINE_RIGHT], deviceIDs, encoderMultiplier,gearBoxRatio)) {
                    ROS_ERROR("could not configure myoBricks, make sure the correct fpga image is used");
                }else {
                    motorAngleThread = boost::shared_ptr<std::thread>(
                            new std::thread(&RoboyPlexus::motorAnglePublisher, this));
                    motorAngleThread->detach();
                }
            }
//            soliInitSensor();
//
//            soliGetData_srv = nh->advertiseService("/roboy/middleware/leftHand/soliGetData", soliGetData);
//            soli_srv = nh->advertiseService("/roboy/middleware/leftHand/soli", soli);
//            soliGetFrameFormat_srv = nh->advertiseService("/roboy/middleware/leftHand/soliGetFrameFormat", soliGetFrameFormat);
//            soliSetFrameFormat_srv = nh->advertiseService("/roboy/middleware/leftHand/soliSetFrameFormat", soliSetFrameFormat);
//            soliGetAdcSamplerate_srv = nh->advertiseService("/roboy/middleware/leftHand/soliGetAdcSamplerate", soliGetAdcSamplerate);
//            soliSetAdcSamplerate_srv = nh->advertiseService("/roboy/middleware/leftHand/soliSetAdcSamplerate", soliSetAdcSamplerate);
//            soliGetChirpDuration_srv = nh->advertiseService("/roboy/middleware/leftHand/soliGetChirpDuration", soliGetChirpDuration);
//            soliSetFMCWConfiguration_srv = nh->advertiseService("/roboy/middleware/leftHand/soliSetFMCWConfiguration", soliSetFMCWConfiguration);
//            soliGetFMCWConfiguration_srv = nh->advertiseService("/roboy/middleware/leftHand/soliGetFMCWConfiguration", soliGetFMCWConfiguration);
//            soliGetFrameInfo_srv = nh->advertiseService("/roboy/middleware/leftHand/soliGetFrameInfo", soliGetFrameInfo);
            break;
        }
        case SHOULDER_LEFT: {
          StearingAngle_pub = nh->advertise<roboy_middleware_msgs::MotorAngle>("/roboy/middleware/StearingAngle",
                                                                                     1);
          stearingAngleThread = boost::shared_ptr<std::thread>(
                   new std::thread(&RoboyPlexus::StearingAnglePublisher, this));
           stearingAngleThread->detach();
//            {
//                motorAngle_pub = nh->advertise<roboy_middleware_msgs::MotorAngle>("/roboy/middleware/MotorAngle",
//                                                                                           1);
//                vector<uint8_t> deviceIDs = {0xF, 0xE};
//                vector<int32_t> gearBoxRatio = {62, 62};
//                vector<int32_t> encoderMultiplier = {1, 1};
//                if (!myoControl->configureMyoBricks(myo_bricks[SHOULDER_LEFT], deviceIDs, encoderMultiplier,
//                                                    gearBoxRatio)) {
//                    ROS_ERROR("could not configure myoBricks, make sure the correct fpga image is used");
//                } else {
//                    motorAngleThread = boost::shared_ptr<std::thread>(
//                            new std::thread(&RoboyPlexus::motorAnglePublisher, this));
//                    motorAngleThread->detach();
//                }
//            }
//            {
//                vector<uint8_t> deviceIDs = {0x50, 0x51, 0x52, 0x53};
//                armControl.reset(new ArmControl(myo_base[1], 0xF, 0xC, deviceIDs, false, false, false, true));
//            }
//            {
//                if(i2c_base.size()>=3) {
//                    vector<uint8_t> deviceIDs = {0x5e};
//                    vector<int> pins = {255};
//                    tlv493D0.push_back(boost::shared_ptr<TLV493D>(new TLV493D(i2c_base[0], deviceIDs, pins)));
//                    tlv493D0.push_back(boost::shared_ptr<TLV493D>(new TLV493D(i2c_base[1], deviceIDs, pins)));
//                    tlv493D0.push_back(boost::shared_ptr<TLV493D>(new TLV493D(i2c_base[2], deviceIDs, pins)));
//                    magneticsShoulderThread = boost::shared_ptr<std::thread>(
//                            new std::thread(&RoboyPlexus::magneticShoulderJointPublisher, this));
//                    magneticsShoulderThread->detach();
//                }
//            }

//            magneticSensor_pub = nh->advertise<roboy_middleware_msgs::MagneticSensor>(
//                    "/roboy/middleware/MagneticSensor", 1, this);
//
//            vector<uint8_t> deviceaddress = {0x5e};
//            vector<int> devicepins = {255};
//            tlv493D0.push_back(boost::shared_ptr<TLV493D>(new TLV493D(i2c_base[0], deviceaddress, devicepins)));
//            tlv493D0.push_back(boost::shared_ptr<TLV493D>(new TLV493D(i2c_base[1], deviceaddress, devicepins)));
//            tlv493D0.push_back(boost::shared_ptr<TLV493D>(new TLV493D(i2c_base[2], deviceaddress, devicepins)));
//
//            magneticsShoulderThread = boost::shared_ptr<std::thread>(
//                    new std::thread(&RoboyPlexus::magneticShoulderJointPublisher, this));
//            magneticsShoulderThread->detach();
            break;
        }
        case SHOULDER_RIGHT: {
            StearingAngle_pub = nh->advertise<roboy_middleware_msgs::MotorAngle>("/roboy/middleware/StearingAngle",
                                                                                 1);
            stearingAngleThread = boost::shared_ptr<std::thread>(
                    new std::thread(&RoboyPlexus::StearingAnglePublisher, this));
            stearingAngleThread->detach();
//            {
//                motorAngle_pub = nh->advertise<roboy_middleware_msgs::MotorAngle>("/roboy/middleware/MotorAngle",
//                                                                                           1);
//                vector<uint8_t> deviceIDs = {0xF, 0xE};
//                vector<int32_t> gearBoxRatio = {62, 62};
//                vector<int32_t> encoderMultiplier = {1, 1};
//                if (!myoControl->configureMyoBricks(myo_bricks[SHOULDER_RIGHT], deviceIDs, encoderMultiplier,
//                                                    gearBoxRatio)) {
//                    ROS_ERROR("could not configure myoBricks, make sure the correct fpga image is used");
//                } else {
//                    motorAngleThread = boost::shared_ptr<std::thread>(
//                            new std::thread(&RoboyPlexus::motorAnglePublisher, this));
//                    motorAngleThread->detach();
//                }
//            }
//            {
//                vector<uint8_t> deviceIDs = {0x50, 0x51, 0x52, 0x53};
//                armControl.reset(new ArmControl(myo_base[1], 0xF, 0xC, deviceIDs, true, false, false, false));
//            }
//            motorAngle_pub = nh->advertise<roboy_middleware_msgs::MotorAngle>("/roboy/middleware/MotorAngle",
//                                                                                       1);
//            jointStatus_pub = nh->advertise<roboy_middleware_msgs::JointStatus>(
//                    "/roboy/middleware/JointStatus", 1);
//            magneticSensor_pub = nh->advertise<roboy_middleware_msgs::MagneticSensor>(
//                    "/roboy/middleware/MagneticSensor", 1, this);
//            if(i2c_base[0]!=nullptr){ // start hand IMU publisher
//                vector<uint8_t> deviceIDs = {0x50, 0x51, 0x52, 0x53};
//                handControl.reset(new HandControl(i2c_base[0], deviceIDs, true));
//                handPower_srv = nh->advertiseService("/roboy/" + body_part + "/control/HandPower",
//                                                     &RoboyPlexus::HandPower, this);
//            }

//            soliInitSensor();
//
//            soliGetData_srv = nh->advertiseService("/roboy/middleware/rightHand/soliGetData", soliGetData);
//            soli_srv = nh->advertiseService("/roboy/middleware/rightHand/soli", soli);
//            soliGetFrameFormat_srv = nh->advertiseService("/roboy/middleware/rightHand/soliGetFrameFormat", soliGetFrameFormat);
//            soliSetFrameFormat_srv = nh->advertiseService("/roboy/middleware/rightHand/soliSetFrameFormat", soliSetFrameFormat);
//            soliGetAdcSamplerate_srv = nh->advertiseService("/roboy/middleware/rightHand/soliGetAdcSamplerate", soliGetAdcSamplerate);
//            soliSetAdcSamplerate_srv = nh->advertiseService("/roboy/middleware/rightHand/soliSetAdcSamplerate", soliSetAdcSamplerate);
//            soliGetChirpDuration_srv = nh->advertiseService("/roboy/middleware/rightHand/soliGetChirpDuration", soliGetChirpDuration);
//            soliSetFMCWConfiguration_srv = nh->advertiseService("/roboy/middleware/rightHand/soliSetFMCWConfiguration", soliSetFMCWConfiguration);
//            soliGetFMCWConfiguration_srv = nh->advertiseService("/roboy/middleware/rightHand/soliGetFMCWConfiguration", soliGetFMCWConfiguration);
//            soliGetFrameInfo_srv = nh->advertiseService("/roboy/middleware/rightHand/soliGetFrameInfo", soliGetFrameInfo);

//            // Look in the device's user manual for allowed addresses! (Table 6)
//            vector<uint8_t> deviceaddress0 = {0b1001010, 0b1001110};//
//            vector<uint8_t> deviceaddress1 = {0b1001010};//
//            vector<int> devicepins0 = {0, 1};
//            vector<int> devicepins1 = {0};
//            tlv493D0[0].reset(new TLV493D(i2c_base[0], deviceaddress0, devicepins0));
//            tlv493D0[1].reset(new TLV493D(i2c_base[1], deviceaddress1, devicepins1));
//
//            magneticsShoulderThread = boost::shared_ptr<std::thread>(
//                    new std::thread(&RoboyPlexus::magneticShoulderJointPublisher, this));
//            magneticsShoulderThread->detach();

//            vector<uint8_t> deviceaddress = {0b1001010};//
//            vector<int> devicepins = {255};
//            tlv493D0.push_back(boost::shared_ptr<TLV493D>(new TLV493D(i2c_base[1], deviceaddress, devicepins)));
//            tlv493D0.push_back(boost::shared_ptr<TLV493D>(new TLV493D(i2c_base[2], deviceaddress, devicepins)));
//            tlv493D0.push_back(boost::shared_ptr<TLV493D>(new TLV493D(i2c_base[3], deviceaddress, devicepins)));

//            magneticsShoulderThread = boost::shared_ptr<std::thread>(
//                    new std::thread(&RoboyPlexus::magneticShoulderJointPublisher, this));
//            magneticsShoulderThread->detach();
            break;
        }
        default: {
            motorCalibration_srv = nh->advertiseService("/roboy/middleware/MotorCalibration",
                                                        &RoboyPlexus::MotorCalibrationService, this);
            darkroom_pub = nh->advertise<roboy_middleware_msgs::DarkRoom>("/roboy/middleware/DarkRoom/sensors",
                                                                                   1);
            darkroom_ootx_pub = nh->advertise<roboy_middleware_msgs::DarkRoomOOTX>(
                    "/roboy/middleware/DarkRoom/ootx", 1);
            darkroom_status_pub = nh->advertise<roboy_middleware_msgs::DarkRoomStatus>(
                    "/roboy/middleware/DarkRoom/status", 1);
            adc_pub = nh->advertise<roboy_middleware_msgs::ADCvalue>("/roboy/middleware/LoadCells", 1);
            if (darkroom_base != nullptr) {
                darkRoomThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::darkRoomPublisher, this));
                darkRoomThread->detach();
            }
            if (!darkroom_ootx_addr.empty()) {
                darkRoomOOTXThread = boost::shared_ptr<std::thread>(
                        new std::thread(&RoboyPlexus::darkRoomOOTXPublisher, this));
                darkRoomOOTXThread->detach();
            }
            if (adc_base != nullptr) {
                adcThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::adcPublisher, this));
                adcThread->detach();
            }
        }
    }

    motorCommand_sub = nh->subscribe("/roboy/middleware/MotorCommand", 1, &RoboyPlexus::motorCommandCB, this);

    // TODO add body part to start record
    startRecordTrajectory_sub = nh->subscribe("/roboy/control/StartRecordTrajectory", 1,
                                              &RoboyPlexus::StartRecordTrajectoryCB, this);
    stopRecordTrajectory_sub = nh->subscribe("/roboy/control/StopRecordTrajectory", 1,
                                             &RoboyPlexus::StopRecordTrajectoryCB, this);
    setGPIO_sub = nh->subscribe("/roboy/control/GPIO", 1, &RoboyPlexus::SetGPIOCB, this);

    //AngleStatus_pub = nh->advertise<roboy_middleware_msgs::MotorStatus>("/roboy/middleware/MotorStatus", 1);
    saveBehavior_sub = nh->subscribe("/roboy/control/SaveBehavior", 1, &RoboyPlexus::SaveBehaviorCB, this);
    enablePlayback_sub = nh->subscribe("/roboy/control/EnablePlayback", 1, &RoboyPlexus::EnablePlaybackCB, this);
    predisplacement_sub = nh->subscribe("/roboy/middleware/PreDisplacement", 1, &RoboyPlexus::PredisplacementCB, this);

    motorConfig_srv = nh->advertiseService("/roboy/" + body_part + "/middleware/MotorConfig",
                                           &RoboyPlexus::MotorConfigService, this);
    controlMode_srv = nh->advertiseService("/roboy/" + body_part + "/middleware/ControlMode",
                                           &RoboyPlexus::ControlModeService, this);
    emergencyStop_srv = nh->advertiseService("/roboy/" + body_part + "/middleware/EmergencyStop",
                                             &RoboyPlexus::EmergencyStopService,
                                             this);

    setDisplacementForAll_srv = nh->advertiseService("/roboy/" + body_part + "/middleware/SetDisplacementForAll",
                                                     &RoboyPlexus::SetDisplacementForAll, this);
//    replayTrajectory_srv = nh->advertiseService("/roboy/" + body_part + "/control/ReplayTrajectory",
//                                                    &RoboyPlexus::ReplayTrajectoryService, this);
//    executeActions_srv = nh->advertiseService("/roboy/" + body_part + "/control/ExecuteActions",
//                                               &RoboyPlexus::ExecuteActionsService, this);
//    executeBehavior_srv = nh->advertiseService("/roboy/" + body_part + "/control/ExecuteBehavior",
//                                               &RoboyPlexus::ExecuteBehaviorService, this);
    listExistingTrajectories_srv = nh->advertiseService("/roboy/" + body_part + "/control/ListExistingTrajectories",
                                                        &RoboyPlexus::ListExistingItemsService, this);
    listExistingBehaviors_srv = nh->advertiseService("/roboy/" + body_part + "/control/ListExistingBehaviors",
                                                     &RoboyPlexus::ListExistingItemsService, this);
    expandBehavior_srv = nh->advertiseService("/roboy/" + body_part + "/control/ExpandBehavior",
                                              &RoboyPlexus::ExpandBehaviorService, this);

    motorStatus_pub = nh->advertise<roboy_middleware_msgs::MotorStatus>("/roboy/middleware/MotorStatus", 1);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
    spinner->start();

    motorStatusThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::motorStatusPublisher, this));
    motorStatusThread->detach();

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++)
        control_mode[motor] = VELOCITY;

    myoControl->allToVelocity(0);

//    // open i2c bus for gsensor
//    if ((file = open(filename, O_RDWR)) < 0) {
//        ROS_ERROR("Failed to open the i2c bus of gsensor");
//    }
//
//    // init
//    // gsensor i2c address: 101_0011
//    int addr = 0b01010011;
//    if (ioctl(file, I2C_SLAVE, addr) < 0) {
//        ROS_WARN("Failed to acquire bus access and/or talk to IMU slave, IMU data will not be published");
//    }else{
//        // configure accelerometer as +-2g and start measure
//        bSuccess = ADXL345_Init(file);
//        if (bSuccess){
//            // dump chip id
//            bSuccess = ADXL345_IdRead(file, &id);
//            if (bSuccess){
//                gsensor_pub = nh->advertise<sensor_msgs::Imu>("/roboy/middleware/IMU", 1);
//                ROS_INFO("gsensor chip_id=%02Xh", id);
//                gsensor_thread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::gsensorPublisher, this));
//                gsensor_thread->detach();
//            }
//        }
//    }

    ROS_INFO("roboy plexus initialized");
}

RoboyPlexus::~RoboyPlexus() {
    keep_publishing = false;

    if (motorStatusThread->joinable())
        motorStatusThread->join();
    if (bSuccess) {
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

string RoboyPlexus::getBodyPart() {
    return body_part;
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
        if (ADXL345_IsDataReady(file)) {
            bSuccess = ADXL345_XYZ_Read(file, szXYZ);
            if (bSuccess) {
                ROS_DEBUG_THROTTLE(1, "X=%d mg, Y=%d mg, Z=%d mg", (int16_t) szXYZ[0] * mg_per_digi,
                                   (int16_t) szXYZ[1] * mg_per_digi, (int16_t) szXYZ[2] * mg_per_digi);
                msg.linear_acceleration.x = (int16_t) szXYZ[0] * mg_per_digi;
                msg.linear_acceleration.y = (int16_t) szXYZ[1] * mg_per_digi;
                msg.linear_acceleration.z = (int16_t) szXYZ[2] * mg_per_digi;
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
        roboy_middleware_msgs::ADCvalue msg;
        msg.id = id;
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
        roboy_middleware_msgs::DarkRoom msg;
        msg.object_id = ethaddr;
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
        rate.sleep();
    }
}

void RoboyPlexus::darkRoomOOTXPublisher() {
    // ootx frames are 17bits preamble + 271 bits payload + 32 bits checksum = 320 bits.
    // At 120Hz motor speed, this is ~2.6 seconds per frame. Lets set the update rate to 5 seconds then.
    ros::Rate rate(1 / 5.0);
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while (keep_publishing) {
        bool successfully_decoded_ootx = true;
        for (uint lighthouse = 0; lighthouse < 2; lighthouse++) {
            for (uint decoder = 0; decoder < darkroom_ootx_addr.size(); decoder++) {
                // TODO: remove reverse and have the fpga convert it directly
                ootx.frame.fw_version = (uint16_t) (
                        reverse(IORD(darkroom_ootx_addr[decoder], (uint32_t) (18 * lighthouse + 0))) & 0xFFFF);
                ootx.frame.ID = reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 1));
                ootx.frame.fcal_0_phase = (uint16_t) (reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 2)) &
                                                      0xFFFF);
                ootx.frame.fcal_1_phase = (uint16_t) (reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 3)) &
                                                      0xFFFF);
                ootx.frame.fcal_0_tilt = (uint16_t) (reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 4)) &
                                                     0xFFFF);
                ootx.frame.fcal_1_tilt = (uint16_t) (reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 5)) &
                                                     0xFFFF);
                ootx.frame.unlock_count = (uint8_t) (reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 6)) &
                                                     0xFF);
                ootx.frame.hw_version = (uint8_t) (reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 7)) &
                                                   0xFF);
                ootx.frame.fcal_0_curve = (uint16_t) (reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 8)) &
                                                      0xFFFF);
                ootx.frame.fcal_1_curve = (uint16_t) (reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 9)) &
                                                      0xFFFF);
                uint32_t acc = reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 10));
                ootx.frame.accel_dir_x = ((int8_t) (acc >> 0 & 0xFF));
                ootx.frame.accel_dir_y = ((int8_t) (acc >> 8 & 0xFF));
                ootx.frame.accel_dir_z = ((int8_t) (acc >> 16 & 0xFF));
                ootx.frame.fcal_0_gibphase = (uint16_t) (
                        reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 11)) & 0xFFFF);
                ootx.frame.fcal_1_gibphase = (uint16_t) (
                        reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 12)) & 0xFFFF);
                ootx.frame.fcal_0_gibmag = (uint16_t) (
                        reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 13)) & 0xFFFF);
                ootx.frame.fcal_1_gibmag = (uint16_t) (
                        reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 14)) & 0xFFFF);
                ootx.frame.mode = (uint8_t) (reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 15)) & 0xFF);
                ootx.frame.faults = (uint8_t) (reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 16)) & 0xFF);

                uint32_t crc32checksum = reverse(IORD(darkroom_ootx_addr[decoder], 18 * lighthouse + 17));

                CRC32 crc;
                for (int i = 0; i < 33; i++) {
                    crc.update(ootx.data[i]);
                }

                uint32_t crc32checksumCalculated = crc.finalize();
                if (crc.finalize() == crc32checksum) { // kinda paranoid right...?!
                    roboy_middleware_msgs::DarkRoomOOTX msg;
                    msg.lighthouse = lighthouse;
                    msg.fw_version = ootx.frame.fw_version;
                    msg.id = ootx.frame.ID;
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
                    msg.accel_dir_x = ootx.frame.accel_dir_x / 127.0f;
                    msg.accel_dir_y = ootx.frame.accel_dir_y / 127.0f;
                    msg.accel_dir_z = ootx.frame.accel_dir_z / 127.0f;
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
                } else {
                    successfully_decoded_ootx = false;

                    stringstream str;
                    str << "received ootx frame" << " with crc " << crc32checksum
                        << " which does not match the calculated: " << crc32checksumCalculated << endl;
                    str << "fw_version:          " << ootx.frame.fw_version << endl;
                    str << "ID:                  " << ootx.frame.ID << endl;
                    str << "fcal_0_phase:        " << ootx.frame.fcal_0_phase << endl;
                    str << "fcal_1_phase:        " << ootx.frame.fcal_1_phase << endl;
                    str << "fcal_0_tilt:         " << ootx.frame.fcal_0_tilt << endl;
                    str << "fcal_1_tilt:         " << ootx.frame.fcal_1_tilt << endl;
                    str << "unlock_count:        " << (uint) ootx.frame.unlock_count << endl;
                    str << "hw_version:          " << (uint) ootx.frame.hw_version << endl;
                    str << "fcal_0_curve:        " << ootx.frame.fcal_0_curve << endl;
                    str << "fcal_1_curve:        " << ootx.frame.fcal_1_curve << endl;
                    str << "accel_dir_x:         " << (int) ootx.frame.accel_dir_x << endl;
                    str << "accel_dir_y:         " << (int) ootx.frame.accel_dir_y << endl;
                    str << "accel_dir_z:         " << (int) ootx.frame.accel_dir_z << endl;
                    str << "fcal_0_gibphase:     " << ootx.frame.fcal_0_gibphase << endl;
                    str << "fcal_1_gibphase:     " << ootx.frame.fcal_1_gibphase << endl;
                    str << "fcal_0_gibmag:       " << ootx.frame.fcal_0_gibmag << endl;
                    str << "fcal_1_gibmag:       " << ootx.frame.fcal_1_gibmag << endl;
                    str << "mode:                " << (uint) ootx.frame.mode << endl;
                    str << "faults:              " << (uint) ootx.frame.faults << endl;
                    ROS_DEBUG_STREAM(str.str());
                }
            }
        }
        if (!successfully_decoded_ootx) {
            // no valid ootx frame decoded, lets switch to another sensor channel
            ootx_sensor_channel++;
            if (ootx_sensor_channel > NUM_SENSORS)
                ootx_sensor_channel = 0;
            for (uint decoder = 0; decoder < darkroom_ootx_addr.size(); decoder++) {
                IOWR(darkroom_ootx_addr[decoder], 0, ootx_sensor_channel);
            }
        }

        roboy_middleware_msgs::DarkRoomStatus status_msg;
        status_msg.object_id = ethaddr;
        uint active_sensors = 0;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            int32_t val = IORD(darkroom_base, (1 << 8 | i));
            status_msg.sensor_state.push_back(val & 0xF);
            if (status_msg.sensor_state.back() == 1)
                active_sensors++;
        }
        ROS_INFO("lighthouse sensors active %d/%d", active_sensors, NUM_SENSORS);
        darkroom_status_pub.publish(status_msg);

        rate.sleep();
    }
}

void RoboyPlexus::motorAnglePublisher() {
    ros::Rate rate(60);
    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MotorAngle msg;
        msg.id = id;
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            if(find(myo_bricks[id].begin(),myo_bricks[id].end(),motor)!=myo_bricks[id].end()) {
                msg.angles.push_back((myoControl->getMotorAngle(motor) / 4096.0 * 360.0));
                msg.raw_angles.push_back(myoControl->getMotorAngle(motor));
                msg.raw_angles_prev.push_back(myoControl->getMotorAnglePrev(motor));
                msg.offset_angles.push_back(myoControl->getMotorAngleOffset(motor));
                msg.relative_angles.push_back(myoControl->getRelativeMotorAngle(motor));
                msg.rev_counter.push_back(myoControl->getRevolutionCounter(motor));
            }else {
                msg.angles.push_back(0);
                msg.raw_angles.push_back(0);
                msg.raw_angles_prev.push_back(0);
                msg.offset_angles.push_back(0);
                msg.relative_angles.push_back(0);
                msg.rev_counter.push_back(0);
            }
        }
        motorAngle_pub.publish(msg);
        rate.sleep();
    }
}
void RoboyPlexus::StearingAnglePublisher() {
    rickshaw_CTL rickshaw_CTL(bike_addr);
    ros::Rate rate(60);
    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MotorAngle msg;
        msg.id = id;
        msg.angles.push_back((rickshaw_CTL.readAngleSensor_raw() / 4096.0 * 360.0));
        msg.raw_angles.push_back(rickshaw_CTL.readAngleSensor_raw());
        msg.raw_angles_prev.push_back(rickshaw_CTL.readAngleSensor_velocity());
        msg.offset_angles.push_back(rickshaw_CTL.readAngleSensor_offset());
        msg.relative_angles.push_back(rickshaw_CTL.readAngleSensor_relative());
        //msg.rev_counter.push_back(rickshaw_CTL.readAngleSensor_counter());
        msg.rev_counter.push_back(rickshaw_CTL.readSpeedData());

        StearingAngle_pub.publish(msg);
        rate.sleep();
    }
}



void RoboyPlexus::motorStatusPublisher() {
    ros::Rate rate(200);
    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MotorStatus msg;
        msg.id = id;
        msg.power_sense = myoControl->getPowerSense();
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            msg.pwm_ref.push_back(myoControl->getPWM(motor));
            int16_t current = myoControl->getCurrent(motor);
            if (current >= 0) {
                msg.position.push_back(myoControl->getPosition(motor));
                msg.velocity.push_back(myoControl->getVelocity(motor));
                msg.displacement.push_back(myoControl->getDisplacement(motor));
                msg.current.push_back(current);
            } else {
                msg.position.push_back(0);
                msg.velocity.push_back(0);
                msg.displacement.push_back(0);
                msg.current.push_back(0);
            }
            msg.angle.push_back(myoControl->getMotorAngle(motor));
        }
        motorStatus_pub.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::magneticShoulderJointPublisher() {
    ros::Rate rate(60);
    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MagneticSensor msg;

        float fx,fy,fz;
        for(int i=0;i<tlv493D0.size();i++){
            ros::Time start_time = ros::Time::now();
            bool success = false;
            do{
                success = tlv493D0[i]->read(fx,fy,fz);
                if(success) {
                    msg.sensor_id.push_back(i);
                    msg.x.push_back(fx);
                    msg.y.push_back(fy);
                    msg.z.push_back(fz);
//                    ROS_INFO("sensor %d %.6f\t%.6f\t%.6f", i, fx, fy, fz);
                }
            }while(!success && (ros::Time::now()-start_time).toSec()<0.1);
        }
        if(msg.sensor_id.size()==tlv493D0.size())
            magneticSensor_pub.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::motorCommandCB(const roboy_middleware_msgs::MotorCommand::ConstPtr &msg) {
    if (msg->id == id) {
        uint i = 0;
        for (auto motor:msg->motors) {
            switch (control_mode[motor]) {
                case POSITION:
                    myoControl->setPosition(motor, msg->set_points[i]);
                    break;
                case VELOCITY:
                    myoControl->setVelocity(motor, msg->set_points[i]);
                    break;
                case DISPLACEMENT:
                    myoControl->setDisplacement(motor, msg->set_points[i]);
                    break;
                case FORCE:
                    myoControl->setDisplacement(motor, msg->set_points[i]);
                    break;
            }
            i++;
        }
    }
}

void RoboyPlexus::motorAnglePID() {
//    ros::Rate rate(100);
//    while (keep_publishing) {
//        roboy_middleware_msgs::MotorAngle msg;
//        msg.id = id;
//        vector<A1335State> state;
//        motorAngle[0]->readAngleData(state);
//        stringstream str;
//        int i=0;
//        for(auto s:state){
//            msg.angles.push_back(s.angle);
//            msg.magneticFieldStrength.push_back(s.fieldStrength);
////            msg.temperature.push_back(s.temp);
//            if(motorData[ANGLE].back()>340 && msg->angles[0] <20){ // increase rotation counter
//                rotationCounter[0]++;
//            }
//            if(motorData[ANGLE].back()<20 && msg->angles[0] > 340){ // decrease rotation counter
//                rotationCounter[0]--;
//            }
//        }
//        motorAngle_pub.publish(msg);
//
//
//
//        motorData[ANGLE].push_back(msg->angles[0]);
//        if (motorData[ANGLE].size() > samples_per_plot) {
//            motorData[ANGLE].pop_front();
//        }
//
//        motorData[ANGLEABSOLUT].push_back(msg->angles[0]+rotationCounter[0]*360.0f+offset[ANGLE]);
//        if (motorData[ANGLEABSOLUT].size() > samples_per_plot) {
//            motorData[ANGLEABSOLUT].pop_front();
//        }
//
//        motorData[SPRING].push_back(motorData[POSITIONABSOLUT].back()-motorData[ANGLEABSOLUT].back());
//        if (motorData[SPRING].size() > samples_per_plot) {
//            motorData[SPRING].pop_front();
//        }
//
//        if (timeMotorData[ANGLE].size() > samples_per_plot)
//            timeMotorData[ANGLE].pop_front();
//
//        rate.sleep();
//
//    }
}

bool RoboyPlexus::HandPower(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    myoControl->gpioControl(req.data);
    return true;
}

bool RoboyPlexus::MotorConfigService(roboy_middleware_msgs::MotorConfigService::Request &req,
                                     roboy_middleware_msgs::MotorConfigService::Response &res) {
    stringstream str;
    control_Parameters_t params;
    uint i = 0;
    for (int motor:req.config.motors) {
        if (req.config.control_mode[i] < POSITION || req.config.control_mode[i] > DISPLACEMENT) {
            ROS_ERROR("trying to change control to an invalid control mode %d, available control modes: "
                              "[0]Position [1]Velocity [2]Displacement", req.config.control_mode[i]);
            i++;
            continue;
        }
        if (req.config.control_mode[i] == POSITION)
            str << "\t" << (int) motor << ": POSITION";
        if (req.config.control_mode[i] == VELOCITY)
            str << "\t" << (int) motor << ": VELOCITY";
        if (req.config.control_mode[i] == DISPLACEMENT)
            str << "\t" << (int) motor << ": DISPLACEMENT";
        params.outputPosMax = req.config.output_pos_max[i];
        params.outputNegMax = req.config.output_neg_max[i];
        params.spPosMax = req.config.sp_pos_max[i];
        params.spNegMax = req.config.sp_neg_max[i];
        params.Kp = req.config.kp[i];
        params.Ki = req.config.ki[i];
        params.Kd = req.config.kd[i];
        params.forwardGain = req.config.forward_gain[i];
        params.deadBand = req.config.dead_band[i];
        params.IntegralPosMax = req.config.integral_pos_max[i];
        params.IntegralNegMax = req.config.integral_neg_max[i];
        params.control_mode = req.config.control_mode[i];
        params.outputDivider = req.config.output_divider[i];
        myoControl->changeControlParameters(motor, params);
        res.mode.push_back(params.control_mode);
        myoControl->changeControl(motor, req.config.control_mode[i], params, req.config.setpoint[i]);
        ROS_INFO("setting motor %d to control mode %d with setpoint %d", motor, req.config.control_mode[i], req.config.setpoint[i]);
        control_mode[motor] = req.config.control_mode[i];
        i++;
    }

    ROS_INFO("serving motor config service for %s control", str.str().c_str());
    return true;
}

bool RoboyPlexus::ControlModeService(roboy_middleware_msgs::ControlMode::Request &req,
                                     roboy_middleware_msgs::ControlMode::Response &res) {
    if (!emergency_stop) {
        if(req.motor_id.empty()) {
            switch (req.control_mode) {
                case POSITION:
                    ROS_INFO("switch to POSITION control");
                    for (auto &mode:control_mode)
                        mode.second = POSITION;
                    myoControl->allToPosition(req.set_point);
                    break;
                case VELOCITY:
                    ROS_INFO("switch to VELOCITY control");
                    for (auto &mode:control_mode)
                        mode.second = VELOCITY;
                    myoControl->allToVelocity(req.set_point);
                    break;
                case DISPLACEMENT:
                    ROS_INFO("switch to DISPLACEMENT control");
                    for (auto &mode:control_mode)
                        mode.second = DISPLACEMENT;
                    myoControl->allToDisplacement(req.set_point);
                    break;
                default:
                    return false;
            }
        }else{
            for(int motor:req.motor_id){
                myoControl->changeControl(motor,req.control_mode);
                control_mode[motor] = req.control_mode;
            }
        }
        return true;
    } else {
        ROS_WARN("emergency stop active, can NOT change control mode");
        return false;
    }
}

bool RoboyPlexus::MotorCalibrationService(roboy_middleware_msgs::MotorCalibrationService::Request &req,
                                          roboy_middleware_msgs::MotorCalibrationService::Response &res) {
    if (!emergency_stop) {
        ROS_INFO("serving motor calibration service for motor %d", req.motor);
        myoControl->estimateSpringParameters(req.motor, req.degree, res.estimated_spring_parameters,
                                             req.timeout, req.number_of_data_points, req.displacement_min,
                                             req.displacement_max, res.load, res.displacement);
        cout << "coefficients:\t";
        for (uint i = 0; i < req.degree; i++) {
            cout << res.estimated_spring_parameters[i] << "\t";
        }
        cout << endl;
        return true;
    } else {
        return false;
    }
}

bool RoboyPlexus::MyoBrickCalibrationService(roboy_middleware_msgs::MyoBrickCalibrationService::Request &req,
                                roboy_middleware_msgs::MyoBrickCalibrationService::Response &res){
    if (!emergency_stop) {
        ROS_INFO("serving myobrick calibration service for motor %d", req.motor);
        myoControl->estimateMotorAngleLinearisationParameters(req.motor, req.degree, res.estimated_spring_parameters,
                                             req.timeout, req.number_of_data_points, req.min_degree,
                                             req.max_degree, res.motor_angle, res.motor_encoder);
        cout << "coefficients:\t";
        for (uint i = 0; i < req.degree; i++) {
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
        // switch to displacement
        ros::Rate rate(100);
        for (int decrements = 99; decrements >= 0; decrements -= 1) {
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                int displacement = myoControl->getDisplacement(motor);
                if (displacement <= 0)
                    continue;
                else
                    myoControl->setDisplacement(motor, displacement * (decrements / 100.0));
            }
            rate.sleep();
        }

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

bool RoboyPlexus::SystemCheckService(roboy_middleware_msgs::SystemCheck::Request &req,
                                     roboy_middleware_msgs::SystemCheck::Response &res) {
    vector<uint8_t> motorIDs;
    if (req.motorids.empty()) {
        motorIDs.resize(NUMBER_OF_MOTORS_PER_FPGA);
        int i = 0;
        for (auto &m:motorIDs) {
            m = i;
            i++;
        }
    } else {
        motorIDs = req.motorids;
    }
    bool system_check_successful = true;
    for (auto &m:motorIDs) {
        if (myoControl->getCurrent(m) == 0) {
            system_check_successful = false;
            res.position.push_back(false);
            res.displacement.push_back(false);
        } else {
            // check position control
            bool position_control = true;
            int32_t current_pos = myoControl->getPosition(m);
            // run positive direction
            myoControl->setPosition(m, current_pos + 5000);
            ros::Duration wait_for_position(0.01);
            wait_for_position.sleep();
            if (abs(myoControl->getPosition(m) - (current_pos + 5000)) > 1000) {
                system_check_successful = false;
                position_control = false;
            }
            // run negative direction
            myoControl->setPosition(m, current_pos - 5000);
            wait_for_position.sleep();
            if (abs(myoControl->getPosition(m) - (current_pos - 5000)) > 1000) {
                system_check_successful = false;
                position_control = false;
            }
            // reset to start position
            myoControl->setPosition(m, current_pos);

            // check displacement control
            bool displacement_control = true;
            int32_t current_displacement = myoControl->getDisplacement(m);
            // run positive direction
            myoControl->setDisplacement(m, current_displacement + 10);
            ros::Duration wait_for_displacement(1);
            wait_for_displacement.sleep();
            if (abs(myoControl->getDisplacement(m) - (current_displacement + 10)) > 3) {
                system_check_successful = false;
                displacement_control = false;
            }
            // run negative direction
            myoControl->setDisplacement(m, current_displacement - 10);
            wait_for_displacement.sleep();
            if (abs(myoControl->getDisplacement(m) - (current_displacement + 10)) > 3) {
                system_check_successful = false;
                displacement_control = false;
            }
            // reset to start displacement
            myoControl->setDisplacement(m, current_displacement);
            res.position.push_back(position_control);
            res.displacement.push_back(displacement_control);
        }
    }
    return system_check_successful;
}

bool RoboyPlexus::SetDisplacementForAll(roboy_middleware_msgs::SetInt16::Request &req,
                                        roboy_middleware_msgs::SetInt16::Request &res) {
    if (id == HEAD) {
        myoControl->allToDisplacement(req.setpoint);
        ros::Duration d(5);
        d.sleep();
        int pos[4] = {0, 0, 0, 0};
        pos[0] = myoControl->getPosition(0);
        pos[1] = myoControl->getPosition(1);
        pos[2] = myoControl->getPosition(2);
        pos[3] = myoControl->getPosition(3);
        myoControl->allToPosition(0);
        myoControl->setPosition(0, pos[0]);
        myoControl->setPosition(1, pos[1]);
        myoControl->setPosition(2, pos[2]);
        myoControl->setPosition(3, pos[3]);
    }
    myoControl->allToDisplacement(req.setpoint);
    return true;
}

void RoboyPlexus::StartRecordTrajectoryCB(const roboy_control_msgs::StartRecordTrajectory::ConstPtr &msg) {

    if (std::find(msg->body_parts.begin(), msg->body_parts.end(), body_part) == msg->body_parts.end()) {
        ROS_INFO_STREAM("Not my call, not recording!");
        return;
    }
    ROS_INFO_STREAM("Recording a trajectory");
    float samplingTime = 5; // 200 Hz is the fastest update rate for the motors
    string name = body_part + "_" + msg->name;
    vector<int> idList(begin(msg->id_list), end(msg->id_list));
    map<int, vector<float>> trajectories;

    myoControl->startRecordTrajectories(samplingTime, trajectories, idList, name);

}

void RoboyPlexus::StopRecordTrajectoryCB(const std_msgs::Empty::ConstPtr &msg) {
    myoControl->stopRecordTrajectories();
}

void RoboyPlexus::SaveBehaviorCB(const roboy_control_msgs::Behavior &msg) {

    ofstream output_file(myoControl->behaviors_folder + msg.name);
    ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(msg.actions.begin(), msg.actions.end(), output_iterator);
}

//void RoboyPlexus::SetGPIOCB(const std_msgs::Bool::ConstPtr& msg) {
//void RoboyPlexus::motorStatusPublisher() {
//}
void RoboyPlexus::SetGPIOCB(const std_msgs::Bool::ConstPtr& msg) {
  rickshaw_CTL rickshaw_CTL(bike_addr);
  rickshaw_CTL.writeThrottle(0xffff & msg->data);
  ROS_INFO("New pint value: [%d]", (rickshaw_CTL.readThrottle()&0x0001));
  ROS_INFO("data form sensor, %d", rickshaw_CTL.readAngleSensor());
}

bool RoboyPlexus::executeActions(vector<string> actions) {

    bool success;
    for (string actionName: actions) {
        success = executeAction(actionName);
    }
    return success;
}

bool RoboyPlexus::executeAction(string actionName) {

    bool success;
    if (actionName.find("pause") != std::string::npos) {
        string delimiter = "_";
        int pause = stoi(actionName.substr(0, actionName.find(delimiter)));
        ros::Duration(pause).sleep();
        success = true && success;
    } else if (actionName.find("relax") != std::string::npos) {
        myoControl->allToDisplacement(0);
        success = true && success;
    } else {
        actionName = myoControl->trajectories_folder + actionName;
        success = myoControl->playTrajectory(actionName.c_str()) && success;
    }

    return success;
}

void RoboyPlexus::EnablePlaybackCB(const std_msgs::Bool::ConstPtr &msg) {
    myoControl->setReplay(msg->data);
}

void RoboyPlexus::PredisplacementCB(const std_msgs::Int32 &msg) {
    myoControl->setPredisplacement(msg.data);
}


bool RoboyPlexus::ListExistingItemsService(roboy_control_msgs::ListItems::Request &req,
                                           roboy_control_msgs::ListItems::Response &res) {

    // read filenames in the specified folder
    DIR *dirp = opendir(req.name.c_str());
    struct dirent *dp;
    if (!dirp) {
        mkdir(req.name.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
        ROS_INFO("created trajectories folder");
        dirp = opendir(req.name.c_str());
    }
    while ((dp = readdir(dirp)) != NULL) {
        if (dp->d_type != DT_DIR) {
            res.items.push_back(dp->d_name);
        }
    }
    closedir(dirp);
    return true;
}

bool RoboyPlexus::ExpandBehaviorService(roboy_control_msgs::ListItems::Request &req,
                                        roboy_control_msgs::ListItems::Response &res) {

    res.items = expandBehavior(req.name);
    return true;
}

vector<string> RoboyPlexus::expandBehavior(string name) {
    std::vector<string> actions;
    ifstream input_file(myoControl->behaviors_folder + name);
    std::copy(std::istream_iterator<std::string>(input_file),
              std::istream_iterator<std::string>(),
              std::back_inserter(actions));

    return actions;
}

bool RoboyPlexus::ADXL345_REG_WRITE(int file, uint8_t address, uint8_t value) {
    bool bSuccess = false;
    uint8_t szValue[2];
    // write to define register
    szValue[0] = address;
    szValue[1] = value;
    if (write(file, &szValue, sizeof(szValue)) == sizeof(szValue)) {
        bSuccess = true;
    }
    return bSuccess;
}

bool RoboyPlexus::ADXL345_REG_READ(int file, uint8_t address, uint8_t *value) {
    bool bSuccess = false;
    uint8_t Value;
    // write to define register
    if (write(file, &address, sizeof(address)) == sizeof(address)) {

        // read back value
        if (read(file, &Value, sizeof(Value)) == sizeof(Value)) {
            *value = Value;
            bSuccess = true;
        }
    }
    return bSuccess;
}

bool RoboyPlexus::ADXL345_REG_MULTI_READ(int file, uint8_t readaddr, uint8_t readdata[], uint8_t len) {
    bool bSuccess = false;
    // write to define register
    if (write(file, &readaddr, sizeof(readaddr)) == sizeof(readaddr)) {
        // read back value
        if (read(file, readdata, len) == len) {
            bSuccess = true;
        }
    }
    return bSuccess;
}

bool RoboyPlexus::ADXL345_Init(int file) {
    bool bSuccess;

    // +- 2g range, 10 bits
    bSuccess = ADXL345_REG_WRITE(file, ADXL345_REG_DATA_FORMAT, XL345_RANGE_2G | XL345_FULL_RESOLUTION);

    //Output Data Rate: 800Hz
    if (bSuccess) {
        bSuccess = ADXL345_REG_WRITE(file, ADXL345_REG_BW_RATE, XL345_RATE_100);
    }

    //INT_Enable: Data Ready
    if (bSuccess) {
        bSuccess = ADXL345_REG_WRITE(file, ADXL345_REG_INT_ENALBE, XL345_DATAREADY);
    }

    // stop measure
    if (bSuccess) {
        bSuccess = ADXL345_REG_WRITE(file, ADXL345_REG_POWER_CTL, XL345_STANDBY);
    }

    // start measure
    if (bSuccess) {
        bSuccess = ADXL345_REG_WRITE(file, ADXL345_REG_POWER_CTL, XL345_MEASURE);

    }


    return bSuccess;

}

bool RoboyPlexus::ADXL345_IsDataReady(int file) {
    bool bReady = false;
    uint8_t data8;

    if (ADXL345_REG_READ(file, ADXL345_REG_INT_SOURCE, &data8)) {
        if (data8 & XL345_DATAREADY)
            bReady = true;
    }

    return bReady;
}

bool RoboyPlexus::ADXL345_XYZ_Read(int file, uint16_t szData16[3]) {
    bool bPass;
    uint8_t szData8[6];
    bPass = ADXL345_REG_MULTI_READ(file, 0x32, (uint8_t *) &szData8, sizeof(szData8));
    if (bPass) {
        szData16[0] = (szData8[1] << 8) | szData8[0];
        szData16[1] = (szData8[3] << 8) | szData8[2];
        szData16[2] = (szData8[5] << 8) | szData8[4];
    }

    return bPass;
}

bool RoboyPlexus::ADXL345_IdRead(int file, uint8_t *pId) {
    bool bPass;
    bPass = ADXL345_REG_READ(file, ADXL345_REG_DEVID, pId);

    return bPass;
}
