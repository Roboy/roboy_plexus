#include <roboy_plexus/roboyPlexus.hpp>
#include <roboy_plexus/myoControl.hpp>

RoboyPlexus::RoboyPlexus(vector<int32_t *> &myo_base, vector<int32_t *> &i2c_base,
                         vector<int> &deviceIDs, int32_t *darkroom_base, uint32_t *adc_base) :
        darkroom_base(darkroom_base) {
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
    adc_pub = nh->advertise<roboy_communication_middleware::ADCvalue>("/roboy/middleware/LoadCells", 1);

    motorStatusThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::motorStatusPublisher, this));
    motorStatusThread->detach();

    if (darkroom_base != nullptr) {
        darkRoomThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::darkRoomPublisher, this));
        darkRoomThread->detach();
    }

    if (adc_base != nullptr) {
        adcThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::adcPublisher, this));
        adcThread->detach();
    }

    jointStatusThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::jointStatusPublisher, this));
    jointStatusThread->detach();

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++)
        control_mode[motor] = DISPLACEMENT;

    myoControl->allToDisplacement(0);

    for (uint i2c_bus = 0; i2c_bus < i2c_base.size(); i2c_bus++)
        jointAngle.push_back(boost::shared_ptr<AM4096>(new AM4096(i2c_base[i2c_bus], deviceIDs)));;
}

RoboyPlexus::~RoboyPlexus() {
    keep_publishing = false;
    if (motorStatusThread->joinable())
        motorStatusThread->join();
    if (jointStatusThread->joinable())
        jointStatusThread->join();
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
    while (keep_publishing) {
        uint active_sensors = 0;
        roboy_communication_middleware::DarkRoom msg;
        for (uint i = 0; i < NUM_SENSORS; i++) {
            int32_t val = IORD(darkroom_base, i);
            msg.sensor_value.push_back(val);
            if ((val >> 29) & 0x1)//valid
                active_sensors++;
        }
        darkroom_pub.publish(msg);
        ROS_INFO_THROTTLE(1, "lighthouse sensors active %d/%d", active_sensors, NUM_SENSORS);
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
    ros::Rate rate(100);
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
//        myoControl->changeControl(motor, req.config.control_mode[i], params, req.setPoints[i]);
//        ROS_INFO("setting motor %d to control mode %d with setpoint %d", motor, req.config.control_mode[i],
//                 req.setPoints[i]);
//        control_mode[motor] = req.config.control_mode[i];
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