#include <roboy_plexus/roboyPlexus.hpp>

RoboyPlexus::RoboyPlexus(MyoControlPtr myoControl, vector<int32_t *> &i2c_base,
                         int32_t *darkroom_base,
                         vector<int32_t *> &darkroom_ootx_addr, int32_t *adc_base, int32_t *switches_base) :
        i2c_base(i2c_base), darkroom_base(darkroom_base), darkroom_ootx_addr(darkroom_ootx_addr),
        adc_base(adc_base), myoControl(myoControl), switches_base(switches_base) {
//
//    id = IORD(switches_base, 0) & 0x7;
////    string body_part;
//    switch (id) {
//        case SHOULDER_LEFT:
//            body_part = "shoulder_left";
//            break;
//        case SHOULDER_RIGHT:
//            body_part = "shoulder_right";
//            break;
//        default:
//            body_part = "unknown";
//    }
//
    ROS_INFO("roboy3 plexus initializing");
    body_part = "roboy3";

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

    motorCommand_sub = nh->subscribe("/roboy/middleware/MotorCommand", 1, &RoboyPlexus::MotorCommand, this);

    // TODO add body part to start record
    startRecordTrajectory_sub = nh->subscribe("/roboy/control/StartRecordTrajectory", 1,
                                              &RoboyPlexus::StartRecordTrajectoryCB, this);
    stopRecordTrajectory_sub = nh->subscribe("/roboy/control/StopRecordTrajectory", 1,
                                             &RoboyPlexus::StopRecordTrajectoryCB, this);
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

    adc = nh->advertise<roboy_middleware_msgs::ADCvalue>("/roboy/middleware/LoadCells", 1);
    testbench = nh->advertise<std_msgs::Float32>("/roboy/middleware/TestRigPosition", 1);

    if (adc_base != nullptr) {
        adcThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::AdcPublisher, this));
        adcThread->detach();

        vector<uint8_t> deviceIds = {0xC};
        a1335.reset(new A1335(i2c_base[3], deviceIds));
        testbenchThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::testBenchPublisher, this));
        testbenchThread->detach();
    }

    setDisplacementForAll_srv = nh->advertiseService("/roboy/" + body_part + "/middleware/SetDisplacementForAll",
                                                     &RoboyPlexus::SetDisplacementForAll, this);
    listExistingTrajectories_srv = nh->advertiseService("/roboy/" + body_part + "/control/ListExistingTrajectories",
                                                        &RoboyPlexus::ListExistingItemsService, this);
    listExistingBehaviors_srv = nh->advertiseService("/roboy/" + body_part + "/control/ListExistingBehaviors",
                                                     &RoboyPlexus::ListExistingItemsService, this);
    expandBehavior_srv = nh->advertiseService("/roboy/" + body_part + "/control/ExpandBehavior",
                                              &RoboyPlexus::ExpandBehaviorService, this);

    motorState = nh->advertise<roboy_middleware_msgs::MotorState>("/roboy/middleware/MotorState", 1);
    motorInfo = nh->advertise<roboy_middleware_msgs::MotorInfo>("/roboy/middleware/MotorInfo", 1);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
    spinner->start();

    motorStateThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::MotorStatePublisher, this));
    motorStateThread->detach();

    motorInfoThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::MotorInfoPublisher, this));
    motorInfoThread->detach();

    for (uint motor = 0; motor < myoControl->motor_config->total_number_of_motors; motor++) {
        myoControl->SetGearBoxRatio(motor,53);
        myoControl->SetPoint(motor, myoControl->GetEncoderPosition(motor,ENCODER0));
        myoControl->ChangeControl(motor, POSITION);
        control_mode[motor] = POSITION;
    }

    vector<int> active_i2c_bus;
    for (int i = 0; i < i2c_base.size(); i++) {
        if (i2c_base[i] != nullptr) {
            tle.push_back(boost::shared_ptr<TLE493D>(new TLE493D(i2c_base[i])));
            active_magnetic_sensors++;
        }
    }
    if (active_magnetic_sensors > 0) {
        magneticSensor = nh->advertise<roboy_middleware_msgs::MagneticSensor>("/roboy/middleware/MagneticSensor", 1);
        magneticsThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::MagneticJointPublisher, this));
        magneticsThread->detach();
    } else {
        ROS_WARN("no active i2c buses, cannot read magnetic sensor data");
    }

    // open i2c bus for gsensor
    if ((file = open(filename, O_RDWR)) < 0) {
        ROS_ERROR("Failed to open the i2c bus of gsensor");
    }

    // init
    // gsensor i2c address: 101_0011
    int addr = 0b01010011;
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        ROS_WARN("Failed to acquire bus access and/or talk to IMU slave, IMU data will not be published");
    }else{
        // configure accelerometer as +-2g and start measure
        bSuccess = ADXL345_Init(file);
        if (bSuccess){
            // dump chip id
            bSuccess = ADXL345_IdRead(file, &id);
            if (bSuccess){
                gsensor = nh->advertise<sensor_msgs::Imu>("/roboy/middleware/IMU", 1);
                ROS_INFO("gsensor chip_id=%02Xh", id);
                gsensor_thread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::GsensorPublisher, this));
                gsensor_thread->detach();
            }
        }
    }

    ROS_INFO("roboy plexus initialized");
}

RoboyPlexus::~RoboyPlexus() {
    keep_publishing = false;
    if (motorStateThread->joinable())
        motorStateThread->join();
    if (motorInfoThread->joinable())
        motorInfoThread->join();
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

void RoboyPlexus::GsensorPublisher() {
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
            gsensor.publish(msg);
        }
        rate.sleep();
    }
}

void RoboyPlexus::AdcPublisher() {
    ros::Rate rate(100);
    while (keep_publishing) {
        roboy_middleware_msgs::ADCvalue msg;
        msg.id = id;
        for (uint i = 0; i < NUMBER_OF_LOADCELLS; i++) {
            uint32_t adcvalue;
            float val = myoControl->GetWeight(i, adcvalue);
            msg.adc_value.push_back(adcvalue);
            msg.load.push_back(val);
        }
        adc.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::testBenchPublisher() {
    ros::Rate rate(100);
    while (keep_publishing) {
        std_msgs::Float32 msg;
        vector<A1335State> state;
        a1335->readAngleData(state);
        msg.data = state[0].angle;
        testbench.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::DarkRoomPublisher() {
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
        darkroom.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::DarkRoomOOTXPublisher() {
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

                    darkroom_ootx.publish(msg);
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
        darkroom_status.publish(status_msg);

        rate.sleep();
    }
}

void RoboyPlexus::MotorStatePublisher() {
    ros::Rate rate(200);
    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MotorState msg;
        for (uint motor = 0; motor < myoControl->motor_config->total_number_of_motors; motor++) {
            msg.encoder0_pos.push_back(myoControl->GetEncoderPosition(motor,ENCODER0));
            msg.encoder1_pos.push_back(myoControl->GetEncoderPosition(motor,ENCODER1));
            msg.displacement.push_back(myoControl->GetDisplacement(motor));
//            msg.encoder1_pos.push_back(myoControl->GetEncoderPosition(motor,ENCODER1));
//            msg.encoder1_vel.push_back(myoControl->GetEncoderVelocity(motor,ENCODER1));
        }
        motorState.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::MotorInfoPublisher() {
    ros::Rate rate(100);
    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MotorInfo msg;
        for (uint motor = 0; motor < myoControl->motor_config->total_number_of_motors; motor++) {
            int32_t Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit;
            myoControl->GetControllerParameter(motor, Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit);
            msg.control_mode.push_back(myoControl->GetControlMode(motor));
            msg.Kp.push_back(Kp);
            msg.Ki.push_back(Ki);
            msg.Kd.push_back(Kd);
            msg.deadband.push_back(deadband);
            msg.IntegralLimit.push_back(IntegralLimit);
            msg.PWMLimit.push_back(PWMLimit);
            int32_t communication_quality = myoControl->GetCommunicationQuality(motor);
            msg.communication_quality.push_back(communication_quality);
            msg.gearBoxRatio.push_back(myoControl->GetGearBoxRatio(motor));
            if(communication_quality>0) {
                msg.setpoint.push_back(myoControl->GetSetPoint(motor));
                msg.pwm.push_back(myoControl->GetPWM(motor));
//                msg.current_phase1.push_back(myoControl->GetCurrent(motor, 1));
//                msg.current_phase2.push_back(myoControl->GetCurrent(motor, 2));
//                msg.current_phase3.push_back(myoControl->GetCurrent(motor, 3));
            }else{
                msg.setpoint.push_back(0);
                msg.pwm.push_back(0);
//                msg.current_phase1.push_back(0);
//                msg.current_phase2.push_back(0);
//                msg.current_phase3.push_back(0);
            }
        }
        motorInfo.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::MagneticJointPublisher() {
    ros::Rate rate(100);
    roboy_middleware_msgs::MagneticSensor msg;
    msg.id = id;
    for(int i=0;i<tle.size();i++){
        msg.sensor_id.push_back(i);
        msg.x.push_back(0);
        msg.y.push_back(0);
        msg.z.push_back(0);
    }

    float fx = 0, fy = 0, fz = 0;
    while (ros::ok()) {
        rate.sleep();
        for (int i = 0; i < tle.size(); i++) {
            tle[i]->read(fx, fy, fz);
            msg.x[i] = fx;
            msg.y[i] = fy;
            msg.z[i] = fz;
        }
        magneticSensor.publish(msg);
    }
}

void RoboyPlexus::MotorCommand(const roboy_middleware_msgs::MotorCommand::ConstPtr &msg) {
    uint i = 0;
    for (auto motor:msg->motor) {
        switch (control_mode[motor]) {
            case POSITION:
                myoControl->SetPoint(motor, msg->setpoint[i]);
                break;
            case VELOCITY:
                myoControl->SetPoint(motor, msg->setpoint[i]);
                break;
            case DISPLACEMENT:
                myoControl->SetPoint(motor, msg->setpoint[i]);
                break;
            case FORCE:
                myoControl->SetPoint(motor, msg->setpoint[i]);
                break;
            case DIRECT_PWM:
                if(fabsf(msg->setpoint[i])>128) {
                    ROS_WARN_THROTTLE(1,"setpoints exceeding sane direct pwm values, "
                                      "what the heck are you publishing?!");
                    break;
                }
                myoControl->SetPoint(motor, msg->setpoint[i]);
                break;
        }
        i++;
    }
}

bool RoboyPlexus::MotorConfigService(roboy_middleware_msgs::MotorConfigService::Request &req,
                                     roboy_middleware_msgs::MotorConfigService::Response &res) {
    stringstream str;
    control_Parameters_t params;
    uint i = 0;
    for (int motor:req.config.motor) {
        if (req.config.control_mode[i] == POSITION)
            str << "\t" << (int) motor << ": POSITION";
        if (req.config.control_mode[i] == VELOCITY)
            str << "\t" << (int) motor << ": VELOCITY";
        if (req.config.control_mode[i] == DISPLACEMENT)
            str << "\t" << (int) motor << ": DISPLACEMENT";
        if (req.config.control_mode[i] == DIRECT_PWM)
            str << "\t" << (int) motor << ": DIRECT_PWM";
        params.PWMLimit = req.config.PWMLimit[i];
        params.Kp = req.config.Kp[i];
        params.Ki = req.config.Ki[i];
        params.Kd = req.config.Kd[i];
        params.deadband = req.config.deadband[i];
        params.IntegralLimit = req.config.IntegralLimit[i];
        params.control_mode = req.config.control_mode[i];
        myoControl->ChangeControlParameters(motor, params);
        res.mode.push_back(params.control_mode);
        myoControl->ChangeControl(motor, req.config.control_mode[i], params, req.config.setpoint[i]);
        myoControl->SetMotorUpdateFrequency(motor,req.config.update_frequency[i]);
        myoControl->SetGearBoxRatio(motor,req.config.gearBoxRatio[i]);
        ROS_INFO("setting motor %d to control mode %d with setpoint %d", motor, req.config.control_mode[i],
                 req.config.setpoint[i]);
        control_mode[motor] = req.config.control_mode[i];
        i++;
    }

    ROS_INFO("serving motor config service for %s control", str.str().c_str());
    return true;
}

bool RoboyPlexus::ControlModeService(roboy_middleware_msgs::ControlMode::Request &req,
                                     roboy_middleware_msgs::ControlMode::Response &res) {
    if (!emergency_stop) {
        if (req.motor_id.empty()) {
            switch (req.control_mode) {
                case POSITION:
                    ROS_INFO("switch to POSITION control");
                    for (auto &mode:control_mode)
                        mode.second = POSITION;
                    myoControl->SetAllToPosition(req.set_point);
                    break;
                case VELOCITY:
                    ROS_INFO("switch to VELOCITY control");
                    for (auto &mode:control_mode)
                        mode.second = VELOCITY;
                    myoControl->SetAllToVelocity(req.set_point);
                    break;
                case DISPLACEMENT:
                    ROS_INFO("switch to DISPLACEMENT control");
                    for (auto &mode:control_mode)
                        mode.second = DISPLACEMENT;
                    myoControl->SetAllToDisplacement(req.set_point);
                    break;
                case DIRECT_PWM:
                    ROS_INFO("switch to DIRECT_PWM control");
                    for (auto &mode:control_mode)
                        mode.second = DIRECT_PWM;
                    myoControl->SetAllToDirectPWM(req.set_point);
                    break;
                default:
                    ROS_ERROR(
                            "invalid control mode requested, available: POSITION, VELOCITY, DISPLACEMENT, FORCE, DIRECT_PWM");
                    return false;
            }
        } else {
            for (int motor:req.motor_id) {
                myoControl->ChangeControl(motor, req.control_mode);
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
        myoControl->EstimateSpringParameters(req.motor, req.degree, res.estimated_spring_parameters,
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
                                             roboy_middleware_msgs::MyoBrickCalibrationService::Response &res) {
    if (!emergency_stop) {
        ROS_INFO("serving myobrick calibration service for motor %d", req.motor);
        myoControl->EstimateMotorAngleLinearisationParameters(req.motor, req.degree, res.estimated_spring_parameters,
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
            for (uint motor = 0; motor < myoControl->motor_config->total_number_of_motors; motor++) {
                int displacement = myoControl->GetEncoderPosition(motor,ENCODER1);
                if (displacement <= 0)
                    continue;
                else
                    myoControl->SetPoint(motor, displacement * (decrements / 100.0));
            }
            rate.sleep();
        }

        control_mode_backup = control_mode;
        control_params_backup = myoControl->control_params;
        control_Parameters_t params;
        params.Kp = 0;
        params.Ki = 0;
        params.Kd = 0;
        params.PWMLimit = 0;
        for (uint motor = 0; motor < myoControl->motor_config->total_number_of_motors; motor++) {
            myoControl->ChangeControl(motor, DISPLACEMENT, params);
        }
        emergency_stop = true;
    } else {
        ROS_INFO("resuming normal operation");
        uint motor = 0;
        for (auto &params:control_params_backup) {
            myoControl->ChangeControl(motor, control_mode_backup[motor], params.second[control_mode_backup[motor]]);
            motor++;
        }
        emergency_stop = false;
    }
    return true;
}

bool RoboyPlexus::SystemCheckService(roboy_middleware_msgs::SystemCheck::Request &req,
                                     roboy_middleware_msgs::SystemCheck::Response &res) {
//    vector<uint8_t> motorIDs;
//    if (req.motorids.empty()) {
//        motorIDs.resize(myoControl->motor_config->total_number_of_motors);
//        int i = 0;
//        for (auto &m:motorIDs) {
//            m = i;
//            i++;
//        }
//    } else {
//        motorIDs = req.motorids;
//    }
//    bool system_check_successful = true;
//    for (auto &m:motorIDs) {
//        if (myoControl->GetCurrent(m,0) == 0) {
//            system_check_successful = false;
//            res.position.push_back(false);
//            res.displacement.push_back(false);
//        } else {
//            // check position control
//            bool position_control = true;
//            int32_t current_pos = myoControl->GetEncoderPosition(m,ENCODER0);
//            // run positive direction
//            myoControl->SetPoint(m, current_pos + 5000);
//            ros::Duration wait_for_position(0.01);
//            wait_for_position.sleep();
//            if (abs(myoControl->GetEncoderPosition(m,ENCODER0) - (current_pos + 5000)) > 1000) {
//                system_check_successful = false;
//                position_control = false;
//            }
//            // run negative direction
//            myoControl->SetPoint(m, current_pos - 5000);
//            wait_for_position.sleep();
//            if (abs(myoControl->GetEncoderPosition(m,ENCODER0) - (current_pos - 5000)) > 1000) {
//                system_check_successful = false;
//                position_control = false;
//            }
//            // reset to start position
//            myoControl->SetPoint(m, current_pos);
//
//            // check displacement control
//            bool displacement_control = true;
//            int32_t current_displacement = myoControl->GetEncoderPosition(m,ENCODER1);
//            // run positive direction
//            myoControl->SetPoint(m, current_displacement + 10);
//            ros::Duration wait_for_displacement(1);
//            wait_for_displacement.sleep();
//            if (abs(myoControl->GetEncoderPosition(m,ENCODER1) - (current_displacement + 10)) > 3) {
//                system_check_successful = false;
//                displacement_control = false;
//            }
//            // run negative direction
//            myoControl->SetPoint(m, current_displacement - 10);
//            wait_for_displacement.sleep();
//            if (abs(myoControl->GetEncoderPosition(m,ENCODER1) - (current_displacement + 10)) > 3) {
//                system_check_successful = false;
//                displacement_control = false;
//            }
//            // reset to start displacement
//            myoControl->SetPoint(m, current_displacement);
//            res.position.push_back(position_control);
//            res.displacement.push_back(displacement_control);
//        }
//    }
//    return system_check_successful;
}

bool RoboyPlexus::SetDisplacementForAll(roboy_middleware_msgs::SetInt16::Request &req,
                                        roboy_middleware_msgs::SetInt16::Request &res) {
    ROS_INFO("all to displacement %d called", req.setpoint);
    for (auto motor:req.motors) {
        myoControl->ChangeControl(motor, DISPLACEMENT);
        myoControl->SetPoint(motor, req.setpoint);
        control_mode[motor] = DISPLACEMENT;
    }
    return true;
}

void RoboyPlexus::StartRecordTrajectoryCB(const roboy_control_msgs::StartRecordTrajectory::ConstPtr &msg) {
    bool its_for_me = false;
    stringstream str;
    for (auto b:body_parts) {
        if (std::find(msg->body_parts.begin(), msg->body_parts.end(), b) != msg->body_parts.end()) {
            str << body_part << " recording " << b << " with " << msg->id_list.size() << " motors" << endl;
            its_for_me = true;
        }
    }
    if (!its_for_me)
        return;
    for (int i:msg->id_list)
        str << i << "\t";
    ROS_INFO_STREAM(str.str());
    float samplingTime = 5; // 200 Hz is the fastest update rate for the motors
    string name = body_part + "_" + msg->name;
    vector<int> idList(begin(msg->id_list), end(msg->id_list));
    map<int, vector<float>> trajectories;

    myoControl->StartRecordTrajectories(samplingTime, trajectories, idList, name);

}

void RoboyPlexus::StopRecordTrajectoryCB(const std_msgs::Empty::ConstPtr &msg) {
    myoControl->StopRecordTrajectories();
}

void RoboyPlexus::SaveBehaviorCB(const roboy_control_msgs::Behavior &msg) {

    ofstream output_file(myoControl->behaviors_folder + msg.name);
    ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(msg.actions.begin(), msg.actions.end(), output_iterator);
}

bool RoboyPlexus::ExecuteActions(vector<string> actions) {

    bool success;
    for (string actionName: actions) {
        success = ExecuteAction(actionName);
    }
    return success;
}

bool RoboyPlexus::ExecuteAction(string actionName) {

    bool success;
    if (actionName.find("pause") != std::string::npos) {
        string delimiter = "_";
        int pause = stoi(actionName.substr(0, actionName.find(delimiter)));
        ros::Duration(pause).sleep();
        success = true && success;
    } else if (actionName.find("relax") != std::string::npos) {
        myoControl->SetAllToDisplacement(0);
        success = true && success;
    } else {
        actionName = myoControl->trajectories_folder + actionName;
        success = myoControl->PlayTrajectory(actionName.c_str()) && success;
    }

    return success;
}

void RoboyPlexus::EnablePlaybackCB(const std_msgs::Bool::ConstPtr &msg) {
    myoControl->SetReplay(msg->data);
}

void RoboyPlexus::PredisplacementCB(const std_msgs::Int32 &msg) {
    myoControl->SetPredisplacement(msg.data);
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

    res.items = ExpandBehavior(req.name);
    return true;
}

vector<string> RoboyPlexus::ExpandBehavior(string name) {
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