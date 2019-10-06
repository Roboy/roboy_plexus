#include "roboyPlexus.hpp"

RoboyPlexus::RoboyPlexus(IcebusControlPtr icebusControl, vector<int32_t *> &i2c_base,
                            int32_t *adc_base, int32_t *switches_base) :
        i2c_base(i2c_base),  adc_base(adc_base), icebusControl(icebusControl), switches_base(switches_base) {
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

    for (uint motor = 0; motor < icebusControl->motor_config->total_number_of_motors; motor++) {
        icebusControl->SetGearBoxRatio(motor,53);
        icebusControl->SetPoint(motor, icebusControl->GetEncoderPosition(motor,ENCODER0));
        icebusControl->ChangeControl(motor, POSITION);
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

    ROS_INFO("roboy plexus initialized");
}

RoboyPlexus::~RoboyPlexus() {
    if (motorStateThread->joinable())
        motorStateThread->join();
    if (motorInfoThread->joinable())
        motorInfoThread->join();
    if (file)
        close(file);
}

void RoboyPlexus::MotorStatePublisher() {
    ros::Rate rate(200);
    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MotorState msg;
        for (uint motor = 0; motor < icebusControl->motor_config->total_number_of_motors; motor++) {
            msg.encoder0_pos.push_back(icebusControl->GetEncoderPosition(motor,ENCODER0));
            msg.encoder1_pos.push_back(icebusControl->GetEncoderPosition(motor,ENCODER1));
            msg.displacement.push_back(icebusControl->GetDisplacement(motor));
        }
        motorState.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::MotorInfoPublisher() {
    ros::Rate rate(100);
    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MotorInfo msg;
        for (uint motor = 0; motor < icebusControl->motor_config->total_number_of_motors; motor++) {
            int32_t Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit;
            icebusControl->GetControllerParameter(motor, Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit);
            msg.control_mode.push_back(icebusControl->GetControlMode(motor));
            msg.Kp.push_back(Kp);
            msg.Ki.push_back(Ki);
            msg.Kd.push_back(Kd);
            msg.deadband.push_back(deadband);
            msg.IntegralLimit.push_back(IntegralLimit);
            msg.PWMLimit.push_back(PWMLimit);
            int32_t communication_quality = icebusControl->GetCommunicationQuality(motor);
            msg.communication_quality.push_back(communication_quality);
            msg.gearBoxRatio.push_back(icebusControl->GetGearBoxRatio(motor));
            msg.setpoint.push_back(icebusControl->GetSetPoint(motor));
            msg.pwm.push_back(icebusControl->GetPWM(motor));
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
                icebusControl->SetPoint(motor, msg->setpoint[i]);
                break;
            case VELOCITY:
                icebusControl->SetPoint(motor, msg->setpoint[i]);
                break;
            case DISPLACEMENT:
                icebusControl->SetPoint(motor, msg->setpoint[i]);
                break;
            case FORCE:
                icebusControl->SetPoint(motor, msg->setpoint[i]);
                break;
            case DIRECT_PWM:
                bool direct_pwm_override;
                nh->getParam("direct_pwm_override",direct_pwm_override);
                if(fabsf(msg->setpoint[i])>128 && !direct_pwm_override) {
                    ROS_WARN_THROTTLE(1,"setpoints exceeding sane direct pwm values, "
                                      "what the heck are you publishing?!");
                    break;
                }
                icebusControl->SetPoint(motor, msg->setpoint[i]);
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
        icebusControl->ChangeControlParameters(motor, params);
        res.mode.push_back(params.control_mode);
        icebusControl->ChangeControl(motor, req.config.control_mode[i], params, req.config.setpoint[i]);
        icebusControl->SetMotorUpdateFrequency(motor,req.config.update_frequency[i]);
        icebusControl->SetGearBoxRatio(motor,req.config.gearBoxRatio[i]);
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
                    icebusControl->SetAllToPosition(req.set_point);
                    break;
                case VELOCITY:
                    ROS_INFO("switch to VELOCITY control");
                    for (auto &mode:control_mode)
                        mode.second = VELOCITY;
                    icebusControl->SetAllToVelocity(req.set_point);
                    break;
                case DISPLACEMENT:
                    ROS_INFO("switch to DISPLACEMENT control");
                    for (auto &mode:control_mode)
                        mode.second = DISPLACEMENT;
                    icebusControl->SetAllToDisplacement(req.set_point);
                    break;
                case DIRECT_PWM:
                    ROS_INFO("switch to DIRECT_PWM control");
                    for (auto &mode:control_mode)
                        mode.second = DIRECT_PWM;
                    icebusControl->SetAllToDirectPWM(req.set_point);
                    break;
                default:
                    ROS_ERROR(
                            "invalid control mode requested, available: POSITION, VELOCITY, DISPLACEMENT, FORCE, DIRECT_PWM");
                    return false;
            }
        } else {
            for (int motor:req.motor_id) {
                icebusControl->ChangeControl(motor, req.control_mode);
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
        icebusControl->EstimateSpringParameters(req.motor, req.degree, res.estimated_spring_parameters,
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
        icebusControl->EstimateMotorAngleLinearisationParameters(req.motor, req.degree, res.estimated_spring_parameters,
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
            for (uint motor = 0; motor < icebusControl->motor_config->total_number_of_motors; motor++) {
                int displacement = icebusControl->GetEncoderPosition(motor,ENCODER1);
                if (displacement <= 0)
                    continue;
                else
                    icebusControl->SetPoint(motor, displacement * (decrements / 100.0));
            }
            rate.sleep();
        }

        control_mode_backup = control_mode;
        control_params_backup = icebusControl->control_params;
        control_Parameters_t params;
        params.Kp = 0;
        params.Ki = 0;
        params.Kd = 0;
        params.PWMLimit = 0;
        for (uint motor = 0; motor < icebusControl->motor_config->total_number_of_motors; motor++) {
            icebusControl->ChangeControl(motor, DISPLACEMENT, params);
        }
        emergency_stop = true;
    } else {
        ROS_INFO("resuming normal operation");
        uint motor = 0;
        for (auto &params:control_params_backup) {
            icebusControl->ChangeControl(motor, control_mode_backup[motor], params.second[control_mode_backup[motor]]);
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
//        motorIDs.resize(icebusControl->motor_config->total_number_of_motors);
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
//        if (icebusControl->GetCurrent(m,0) == 0) {
//            system_check_successful = false;
//            res.position.push_back(false);
//            res.displacement.push_back(false);
//        } else {
//            // check position control
//            bool position_control = true;
//            int32_t current_pos = icebusControl->GetEncoderPosition(m,ENCODER0);
//            // run positive direction
//            icebusControl->SetPoint(m, current_pos + 5000);
//            ros::Duration wait_for_position(0.01);
//            wait_for_position.sleep();
//            if (abs(icebusControl->GetEncoderPosition(m,ENCODER0) - (current_pos + 5000)) > 1000) {
//                system_check_successful = false;
//                position_control = false;
//            }
//            // run negative direction
//            icebusControl->SetPoint(m, current_pos - 5000);
//            wait_for_position.sleep();
//            if (abs(icebusControl->GetEncoderPosition(m,ENCODER0) - (current_pos - 5000)) > 1000) {
//                system_check_successful = false;
//                position_control = false;
//            }
//            // reset to start position
//            icebusControl->SetPoint(m, current_pos);
//
//            // check displacement control
//            bool displacement_control = true;
//            int32_t current_displacement = icebusControl->GetEncoderPosition(m,ENCODER1);
//            // run positive direction
//            icebusControl->SetPoint(m, current_displacement + 10);
//            ros::Duration wait_for_displacement(1);
//            wait_for_displacement.sleep();
//            if (abs(icebusControl->GetEncoderPosition(m,ENCODER1) - (current_displacement + 10)) > 3) {
//                system_check_successful = false;
//                displacement_control = false;
//            }
//            // run negative direction
//            icebusControl->SetPoint(m, current_displacement - 10);
//            wait_for_displacement.sleep();
//            if (abs(icebusControl->GetEncoderPosition(m,ENCODER1) - (current_displacement + 10)) > 3) {
//                system_check_successful = false;
//                displacement_control = false;
//            }
//            // reset to start displacement
//            icebusControl->SetPoint(m, current_displacement);
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
        icebusControl->ChangeControl(motor, DISPLACEMENT);
        icebusControl->SetPoint(motor, req.setpoint);
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

    icebusControl->StartRecordTrajectories(samplingTime, trajectories, idList, name);

}

void RoboyPlexus::StopRecordTrajectoryCB(const std_msgs::Empty::ConstPtr &msg) {
    icebusControl->StopRecordTrajectories();
}

void RoboyPlexus::SaveBehaviorCB(const roboy_control_msgs::Behavior &msg) {

    ofstream output_file(icebusControl->behaviors_folder + msg.name);
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
        icebusControl->SetAllToDisplacement(0);
        success = true && success;
    } else {
        actionName = icebusControl->trajectories_folder + actionName;
        success = icebusControl->PlayTrajectory(actionName.c_str()) && success;
    }

    return success;
}

void RoboyPlexus::EnablePlaybackCB(const std_msgs::Bool::ConstPtr &msg) {
    icebusControl->SetReplay(msg->data);
}

void RoboyPlexus::PredisplacementCB(const std_msgs::Int32 &msg) {
    icebusControl->SetPredisplacement(msg.data);
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
    ifstream input_file(icebusControl->behaviors_folder + name);
    std::copy(std::istream_iterator<std::string>(input_file),
              std::istream_iterator<std::string>(),
              std::back_inserter(actions));

    return actions;
}