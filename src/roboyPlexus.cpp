#include <roboy_plexus/roboyPlexus.hpp>
#include <roboy_plexus/myoControl.hpp>

RoboyPlexus::RoboyPlexus(vector<int32_t *> &myo_base, vector<int32_t *> &i2c_base,
                         vector<int32_t> &deviceIDs, int32_t *darkroom_base,
                         vector<int32_t *> &darkroom_ootx_addr,
                         int32_t *adc_base) :
        myo_base(myo_base), i2c_base(i2c_base), deviceIDs(deviceIDs), darkroom_base(darkroom_base),
        darkroom_ootx_addr(darkroom_ootx_addr), adc_base(adc_base){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy_fpga_interface");
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    myoControl = boost::shared_ptr<MyoControl>(new MyoControl(myo_base, adc_base));

    motorCommand_sub = nh->subscribe("/roboy/middleware/MotorCommand", 1, &RoboyPlexus::motorCommandCB, this);
    motorConfig_srv = nh->advertiseService("/roboy/middleware/MotorConfig", &RoboyPlexus::MotorConfigService, this);
    controlMode_srv = nh->advertiseService("/roboy/middleware/ControlMode", &RoboyPlexus::ControlModeService, this);
    emergencyStop_srv = nh->advertiseService("/roboy/middleware/EmergencyStop", &RoboyPlexus::EmergencyStopService,
                                             this);
    motorCalibration_srv = nh->advertiseService("/roboy/middleware/MotorCalibration",
                                                &RoboyPlexus::MotorCalibrationService, this);

    motorStatus_pub = nh->advertise<roboy_communication_middleware::MotorStatus>("/roboy/middleware/MotorStatus", 1);
    jointStatus_pub = nh->advertise<roboy_communication_middleware::JointStatus>("/roboy/middleware/JointStatus", 1);
    darkroom_pub = nh->advertise<roboy_communication_middleware::DarkRoom>("/roboy/middleware/DarkRoom/sensors", 1);
    darkroom_ootx_pub = nh->advertise<roboy_communication_middleware::DarkRoomOOTX>("/roboy/middleware/DarkRoom/ootx", 1);
    adc_pub = nh->advertise<roboy_communication_middleware::ADCvalue>("/roboy/middleware/LoadCells", 1);
    gsensor_pub = nh->advertise<sensor_msgs::Imu>("/roboy/middleware/imu0", 1);

    motorStatusThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::motorStatusPublisher, this));
    motorStatusThread->detach();

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

    if(!i2c_base.empty()) {
        for (uint i2c_bus = 0; i2c_bus < i2c_base.size(); i2c_bus++)
            jointAngle.push_back(boost::shared_ptr<AM4096>(new AM4096(i2c_base[i2c_bus], deviceIDs)));
        jointStatusThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::jointStatusPublisher, this));
        jointStatusThread->detach();
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++)
        control_mode[motor] = DISPLACEMENT;

    myoControl->allToDisplacement(0);

    // open i2c bus for gsensor
    if ((file = open(filename, O_RDWR)) < 0) {
        ROS_ERROR("Failed to open the i2c bus of gsensor");
    }

    // init
    // gsensor i2c address: 101_0011
    int addr = 0b01010011;
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        ROS_ERROR("Failed to acquire bus access and/or talk to slave");
    }
    // configure accelerometer as +-2g and start measure
    bSuccess = ADXL345_Init(file);
    if (bSuccess){
        // dump chip id
        bSuccess = ADXL345_IdRead(file, &id);
        if (bSuccess){
            ROS_INFO("gsensor chip_id=%02Xh\r\n", id);
            gsensor_thread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::gsensorPublisher, this));
            gsensor_thread->detach();
        }
    }
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
        ROS_INFO_THROTTLE(1, "lighthouse sensors active %d/%d", active_sensors, NUM_SENSORS);
    }
}

void RoboyPlexus::darkRoomOOTXPublisher() {
    ros::Rate rate(1);
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while (keep_publishing) {
        for(uint lighthouse=0;lighthouse<2;lighthouse++){
            for(uint decoder=0;decoder<darkroom_ootx_addr.size();decoder++){
                roboy_communication_middleware::DarkRoomOOTX msg;
                // TODO: remove reverse and have the fpga convert it directly
                uint16_t fw_version = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],(uint32_t)(18*lighthouse+0)))&0xFFFF);
                uint32_t ID = reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+1));
                uint32_t crc32checksum = reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+17));
                uint32_t test = (uint16_t)(IORD(darkroom_ootx_addr[decoder],18)&0xFFFF);
                bitset<32> bits(crc32checksum);
                cout << bits << endl;


                msg.lighthouse = lighthouse;
                msg.fw_version = fw_version;
                msg.ID = ID;
                half fcal_0_phase, fcal_1_phase, fcal_0_tilt, fcal_1_tilt;
                fcal_0_phase.data_ = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+2))&0xFFFF);
                fcal_1_phase.data_ = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+3))&0xFFFF);
                fcal_0_tilt.data_ = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+4))&0xFFFF);
                fcal_1_tilt.data_ = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+5))&0xFFFF);
                msg.fcal_0_phase = fcal_0_phase;
                msg.fcal_1_phase = fcal_1_phase;
                msg.fcal_0_tilt = fcal_0_tilt;
                msg.fcal_1_tilt = fcal_1_tilt;
                msg.unlock_count = reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+6))&0xFF;
                msg.hw_version = reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+7));
                half fcal_0_curve, fcal_1_curve;
                fcal_0_curve.data_ = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+8))&0xFFFF);
                fcal_1_curve.data_ = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+9))&0xFFFF);
                msg.fcal_0_curve = fcal_0_curve;
                msg.fcal_1_curve = fcal_1_curve;
                uint32_t acc = reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+10));
                msg.accel_dir_x = ((int8_t)(acc>>0&0xff));
                msg.accel_dir_y = ((int8_t)(acc>>8&0xff));
                msg.accel_dir_z = ((int8_t)(acc>>16&0xff));
                half fcal_0_gibphase, fcal_1_gibphase, fcal_0_gibmag, fcal_1_gibmag;
                fcal_0_gibphase.data_ = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+11))&0xFFFF);
                fcal_1_gibphase.data_ = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+12))&0xFFFF);
                fcal_0_gibmag.data_ = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+13))&0xFFFF);
                fcal_1_gibmag.data_ = (uint16_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+14))&0xFFFF);
                msg.fcal_0_gibphase = fcal_0_gibphase;
                msg.fcal_1_gibphase = fcal_1_gibphase;
                msg.fcal_0_gibmag = fcal_0_gibmag;
                msg.fcal_1_gibmag = fcal_1_gibmag;
                msg.mode = (uint8_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+15))&0xFF);
                msg.faults = (uint8_t)(reverse(IORD(darkroom_ootx_addr[decoder],18*lighthouse+16))&0xFF);
                msg.crc32 = crc32checksum;

                cout << "received ootx frame"  << " with crc " << crc32checksum << endl;
                cout << "fw_version:          " ; printf("%x\n",msg.fw_version);
                cout << "ID:                  " << msg.ID << endl;
                cout << "fcal_0_phase:        " << msg.fcal_0_phase << endl;
                cout << "fcal_1_phase:        " << msg.fcal_1_phase << endl;
                cout << "fcal_0_tilt:         " << msg.fcal_0_tilt << endl;
                cout << "fcal_1_tilt:         " << msg.fcal_1_tilt << endl;
                cout << "unlock_count:        " << (uint)msg.unlock_count << endl;
                cout << "hw_version:          " << (uint)msg.hw_version << endl;
                cout << "fcal_0_curve:        " << msg.fcal_0_curve << endl;
                cout << "fcal_1_curve:        " << msg.fcal_1_curve << endl;
                cout << "accel_dir_x:         " << msg.accel_dir_x << endl;
                cout << "accel_dir_y:         " << msg.accel_dir_y << endl;
                cout << "accel_dir_z:         " << msg.accel_dir_z << endl;
                cout << "fcal_0_gibphase:     " << msg.fcal_0_gibphase << endl;
                cout << "fcal_1_gibphase:     " << msg.fcal_1_gibphase << endl;
                cout << "fcal_0_gibmag:       " << msg.fcal_0_gibmag << endl;
                cout << "fcal_1_gibmag:       " << msg.fcal_1_gibmag << endl;
                cout << "mode:                " << (uint)msg.mode << endl;
                cout << "faults:              " << (uint)msg.faults << endl;

                darkroom_ootx_pub.publish(msg);
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

void RoboyPlexus::motorCommandCB(const roboy_communication_middleware::MotorCommand::ConstPtr &msg) {
    uint i = 0;
    for (auto motor:msg->motors) {
        myoControl->changeSetpoint(motor, msg->setPoints[i]);
        i++;
    }
}

bool RoboyPlexus::MotorConfigService(roboy_communication_middleware::MotorConfigService::Request &req,
                                     roboy_communication_middleware::MotorConfigService::Response &res) {
    if (req.setPoints.size() != req.config.motors.size()) {
        ROS_ERROR("the number of setpoints do not match the number of motor configs");
        return false;
    }

    ROS_INFO("serving motor config service");
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
        ROS_INFO("serving control mode service");
        uint i = 0;
        switch(req.control_mode){
            case POSITION:
                for (auto &mode:control_mode)
                    mode.second = POSITION;
                myoControl->allToPosition(req.setPoint);
                break;
            case VELOCITY:
                for (auto &mode:control_mode)
                    mode.second = VELOCITY;
                myoControl->allToVelocity(req.setPoint);
                break;
            case DISPLACEMENT:
                for (auto &mode:control_mode)
                    mode.second = DISPLACEMENT;
                myoControl->allToDisplacement(req.setPoint);
                break;
            default:
                return false;
        }
        return true;
    } else {
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