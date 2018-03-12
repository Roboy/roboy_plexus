#include <roboy_plexus/roboyPlexus.hpp>
#include <roboy_plexus/myoControl.hpp>

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
    startRecordTrajectory_srv = nh->advertiseService("roboy/middleware/StartRecordTrajectory",
                                                     &RoboyPlexus::StartRecordTrajectoryService, this);
    stopRecordTrajectory_srv = nh->advertiseService("roboy/middleware/StopRecordTrajectory",
                                                     &RoboyPlexus::StopRecordTrajectoryService, this);

    motorStatus_pub = nh->advertise<roboy_communication_middleware::MotorStatus>("/roboy/middleware/MotorStatus", 1);
    motorAngle_pub = nh->advertise<roboy_communication_middleware::MotorAngle>("/roboy/middleware/MotorAngle", 1);
    jointStatus_pub = nh->advertise<roboy_communication_middleware::JointStatus>("/roboy/middleware/JointStatus", 1);
    adc_pub = nh->advertise<roboy_communication_middleware::ADCvalue>("/roboy/middleware/LoadCells", 1);
    gsensor_pub = nh->advertise<sensor_msgs::Imu>("/roboy/middleware/imu0", 1);
    magneticSensor_pub = nh->advertise<roboy_communication_middleware::MagneticSensor>("/roboy/middleware/MagneticSensor",1, this);

    motorStatusThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::motorStatusPublisher, this));
    motorStatusThread->detach();


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

bool RoboyPlexus::StartRecordTrajectoryService(roboy_communication_control::StartRecordTrajectory::Request &req,
                                  roboy_communication_control::StartRecordTrajectory::Response &res) {

    float samplingTime = 1/200; // 200 Hz is the fastest update rate for the motors
    string name = req.name;
    vector<int> idList(begin(req.idList), end(req.idList));
    map<int, vector<float>> trajectories;

    myoControl->startRecordTrajectories(samplingTime, trajectories, idList, name);

    std::ofstream outfile;
    if (name.empty()) {
        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char str[200];
        sprintf(str, "recording_%s.log",
                asctime(timeinfo));
        name = str;
    }

    outfile.open(name);
    if (outfile.is_open()) {
        outfile << "<?xml version=\"1.0\" ?>"
                << std::endl;
        uint m = 0;
        char motorname[10];
        for (uint m = 0; m < idList.size(); m++) {
            sprintf(motorname, "motor%d", idList[m]);
            outfile << "<trajectory motorid=\"" << idList[m] << "\" controlmode=\""
                    << POSITION << "\" samplingTime=\"" << samplingTime * 1000.0f << "\">"
                    << std::endl;
            outfile << "<waypointlist>" << std::endl;
            for (uint i = 0; i < trajectories[idList[m]].size(); i++)
                outfile << trajectories[idList[m]][i] << " ";
            outfile << "</waypointlist>" << std::endl;
            outfile << "</trajectory>" << std::endl;
        }
        outfile << "</roboybehavior>" << std::endl;
        outfile.close();
    }

    res.trajectories.layout.dim[0].label = "motor_id";
    res.trajectories.layout.dim[0].size = idList.size();
    res.trajectories.layout.dim[0].stride = (trajectories[idList[0]].size()+1)*idList.size();
    res.trajectories.layout.dim[1].label = "setpoint";
    res.trajectories.layout.dim[1].size = trajectories[idList[0]].size()+1;
    res.trajectories.layout.dim[1].stride = trajectories[idList[0]].size()+1;

    for (auto const& motor : trajectories)
    {
        res.trajectories.data.push_back(motor.first); // motor id
        res.trajectories.data.reserve(res.trajectories.data.size() + distance(motor.second.begin(),motor.second.end()));
        res.trajectories.data.insert(res.trajectories.data.end(),motor.second.begin(),motor.second.end()); // setpoints
    }

    res.success = true;

    return true;

}

bool RoboyPlexus::StopRecordTrajectoryService(roboy_communication_control::StopRecordTrajectory::Request &req,
                                   roboy_communication_control::StopRecordTrajectory::Response &res) {
    myoControl->stopRecordTrajectories();
    res.success = true;
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