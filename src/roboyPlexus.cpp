#include <roboy_plexus/roboyPlexus.hpp>

RoboyPlexus::RoboyPlexus(vector<int32_t *> &myo_base, int32_t* darkroom_base):darkroom_base(darkroom_base){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "interface");
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    myoControl = boost::shared_ptr<MyoControl>(new MyoControl(myo_base));

    motorCommand_sub = nh->subscribe("/roboy/middleware/MotorCommand", 1, &RoboyPlexus::motorCommandCB, this);
    motorConfig_srv = nh->advertiseService("/roboy/middleware/MotorConfig", &RoboyPlexus::MotorConfigService, this);
    controlMode_srv = nh->advertiseService("/roboy/middleware/ControlMode", &RoboyPlexus::ControlModeService, this);

    motorStatus_pub = nh->advertise<roboy_communication_middleware::MotorStatus>("/roboy/middleware/MotorStatus", 1);
    darkroom_pub = nh->advertise<roboy_communication_middleware::DarkRoom>("/roboy/middleware/DarkRoom/sensors", 1);

    motorStatusThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::motorStatusPublisher, this));
    motorStatusThread->detach();

    myoControl->allToDisplacement(0);
}

RoboyPlexus::~RoboyPlexus(){
    keep_publishing_motor_status = false;
    if(motorStatusThread->joinable())
        motorStatusThread->join();
}

void RoboyPlexus::motorStatusPublisher(){
    ros::Rate rate(240);
    while(keep_publishing_motor_status){
        uint active_sensors = 0;
        roboy_communication_middleware::DarkRoom msg;
        for(uint i=0;i<NUM_SENSORS;i++){
            int32_t val = IORD(darkroom_base,i);
            msg.sensor_value.push_back(val);
            if((val >> 29)&0x1)//valid
                active_sensors ++;
        }
        darkroom_pub.publish(msg);

        ROS_INFO_THROTTLE(1,"lighthouse sensors active %d/%d", active_sensors, NUM_SENSORS);

        roboy_communication_middleware::MotorStatus msg2;
        for (uint motor = 0; motor < 14; motor++) {
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

void RoboyPlexus::motorCommandCB(const roboy_communication_middleware::MotorCommand::ConstPtr &msg){
    uint i = 0;
    for(auto motor:msg->motors){
        myoControl->changeSetpoint(motor,msg->setPoints[i]);
    }
}

bool RoboyPlexus::MotorConfigService(roboy_communication_middleware::MotorConfigService::Request  &req,
                                  roboy_communication_middleware::MotorConfigService::Response &res){
    if(req.setPoints.size()!=req.config.motors.size()){
        ROS_ERROR("the number of setpoints do not match the number of motor configs");
        return false;
    }

    ROS_INFO("serving motor config service");
    control_Parameters_t params;
    uint i = 0;
    for(auto motor:req.config.motors){
        if(req.config.control_mode[i]<POSITION || req.config.control_mode[i]>DISPLACEMENT){
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
        ROS_INFO("setting motor %d to control mode %d with setpoint %d", motor, req.config.control_mode[i], req.setPoints[i]);
        i++;
    }
    return true;
}

bool RoboyPlexus::ControlModeService(roboy_communication_middleware::ControlMode::Request  &req,
                                     roboy_communication_middleware::ControlMode::Response &res){
    ROS_INFO("serving control mode service");
    uint i = 0;
    if(req.control_mode == POSITION)
        myoControl->allToPosition(req.setPoint);
    else if (req.control_mode = VELOCITY)
        myoControl->allToVelocity(req.setPoint);
    else if(req.control_mode == DISPLACEMENT)
        myoControl->allToDisplacement(req.setPoint);
    return true;
}