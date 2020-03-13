#include "roboyPlexus.hpp"
#include <rcutils/logging_macros.h>
#include <rcutils/time.h>

RoboyPlexus::RoboyPlexus(IcebusControlPtr icebusControl,
        vector<BallJointPtr> balljoints,
        vector<FanControlPtr> fanControls,
        vector<int32_t *> &i2c_base) :
        icebusControl(icebusControl), fanControls(fanControls), balljoints(balljoints), i2c_base(i2c_base){
    RCLCPP_INFO(nh->get_logger(), "roboy3 plexus initializing");

    ifstream ifile("/sys/class/net/eth0/address");
    ifile >> ethaddr;
    ifile.close();
    string node_name = "roboy_fpga_" + ethaddr;
    replace(node_name.begin(), node_name.end(), ':', '_');

    if (!rclcpp::is_initialized()) {
        int argc = 0;
        char **argv = NULL;
        rclcpp::init(argc, argv); //, node_name, ros::init_options::NoSigintHandler);
    }

    nh = rclcpp::Node::make_shared(node_name);

    if (!balljoints.empty()) {
        magneticSensor = nh->create_publisher<roboy_middleware_msgs::msg::MagneticSensor>("/roboy/middleware/MagneticSensor",1);

        magneticsThread = shared_ptr<std::thread>(
                new std::thread(&RoboyPlexus::MagneticJointPublisher, this));
        magneticsThread->detach();
    } else {
        RCLCPP_WARN(nh->get_logger(), "no active ball joints");
    }
    RCLCPP_INFO(nh->get_logger(), "initializing elbow joints");
    if(i2c_base.size()>4){
      {
        vector <uint8_t> ids = {0xC};
        a1335.push_back(A1335Ptr(new A1335(i2c_base[4],ids)));
      }
      {
        vector <uint8_t> ids = {0xD};
        a1335.push_back(A1335Ptr(new A1335(i2c_base[5],ids)));
      }

    }

    // TODO wait for juri
    jointState = nh->create_publisher<sensor_msgs::msg::JointState>("external_joint_states",1);
    elbowJointAngleThread = shared_ptr<std::thread>(
            new std::thread(&RoboyPlexus::ElbowJointPublisher, this));
    elbowJointAngleThread->detach();

    motorState = nh->create_publisher<roboy_middleware_msgs::msg::MotorState>("/roboy/middleware/MotorState", 1);
    motorInfo = nh->create_publisher<roboy_middleware_msgs::msg::MotorInfo>("/roboy/middleware/MotorInfo", 1);

    motorStateThread = shared_ptr<std::thread>(new std::thread(&RoboyPlexus::MotorStatePublisher, this));
    motorStateThread->detach();

    motorInfoThread = shared_ptr<std::thread>(new std::thread(&RoboyPlexus::MotorInfoPublisher, this));
    motorInfoThread->detach();

    neopixel_sub = nh->create_subscription<roboy_middleware_msgs::msg::Neopixel>("/roboy/middleware/Neopixel", 1, bind(&RoboyPlexus::Neopixel, this, placeholders::_1));

    for (auto &m:icebusControl->motor_config->motor) {
        icebusControl->SetNeopixelColor(m.second->motor_id_global, 0xF00000);
        if (icebusControl->GetCommunicationQuality(m.second->motor_id_global) != 0)
            icebusControl->SetPoint(m.second->motor_id_global, icebusControl->GetEncoderPosition(m.second->motor_id_global, ENCODER0));
        else
            icebusControl->SetPoint(m.second->motor_id_global, 0);
        control_Parameters_t params;
        icebusControl->GetDefaultControlParams(&params, 3);
        icebusControl->SetControlMode(m.second->motor_id_global, 3, params);
        icebusControl->SetCurrentLimit(m.second->motor_id_global, 2.0);
    }
    motorControl.push_back(icebusControl);

    motorConfig_srv = nh->create_service<roboy_middleware_msgs::srv::MotorConfigService>("/roboy/middleware/MotorConfig",
            bind(&RoboyPlexus::MotorConfigService, this, placeholders::_1,placeholders::_2));
    controlMode_srv = nh->create_service<roboy_middleware_msgs::srv::ControlMode>("/roboy/middleware/ControlMode",
            bind(&RoboyPlexus::ControlModeService, this, placeholders::_1,placeholders::_2));
    emergencyStop_srv = nh->create_service<std_srvs::srv::SetBool>("/roboy/middleware/EmergencyStop",
            bind(&RoboyPlexus::EmergencyStopService, this, placeholders::_1,placeholders::_2));

    motorControl_sub = nh->create_subscription<roboy_middleware_msgs::msg::MotorControl>("/roboy/middleware/MotorControl", 1, bind(&RoboyPlexus::MotorControl, this, placeholders::_1));
    motorCommand_sub = nh->create_subscription<roboy_middleware_msgs::msg::MotorCommand>("/roboy/middleware/MotorCommand", 1,bind(&RoboyPlexus::MotorCommand, this, placeholders::_1));

//    using rclcpp::executors::MultiThreadedExecutor;
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);
    
//    spinner = shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
//    spinner->start();

    if(!fanControls.empty()){
      for(auto fan:fanControls){
        fan->SetAutoFan(true);
        RCLCPP_INFO(nh->get_logger(), "auto fan is %s", (fan->GetAutoFan()?"on":"off"));
        fan->SetSensitivity(1);
        RCLCPP_INFO(nh->get_logger(), "fan pwm freq %d, duty %d",fan->GetPWMFrequency(), fan->GetDuty());
      }
      // rclcpp::Rate rate(10);
      //
      // while(rclcpp::ok()){
      //   for(auto fan:fanControls){
      //     RCLCPP_INFO(nh->get_logger(), "duty %d, average current %d",fan->GetDuty(), fan->GetCurrentAverage());
      //     rate.sleep();
      //   }
      // }
    }
    RCLCPP_INFO(nh->get_logger(), "roboy plexus initialized");
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
    rclcpp::Rate rate(200);
    roboy_middleware_msgs::msg::MotorState msg;
    for (auto &m:icebusControl->motor_config->motor) {
      msg.global_id.push_back(m.second->motor_id_global);
    }
    msg.setpoint.resize(icebusControl->motor_config->motor.size());
    msg.encoder0_pos.resize(icebusControl->motor_config->motor.size());
    msg.encoder1_pos.resize(icebusControl->motor_config->motor.size());
    msg.displacement.resize(icebusControl->motor_config->motor.size());
    msg.current.resize(icebusControl->motor_config->motor.size());
    while (keep_publishing && rclcpp::ok()) {
        int i = 0;
        for (auto &m:icebusControl->motor_config->motor) {
            msg.setpoint[i] = icebusControl->GetSetPoint(m.second->motor_id_global);
            msg.encoder0_pos[i] = icebusControl->GetEncoderPosition(m.second->motor_id_global,ENCODER0_POSITION);
            msg.encoder1_pos[i] = icebusControl->GetEncoderPosition(m.second->motor_id_global,ENCODER1_POSITION);
            msg.displacement[i] = icebusControl->GetDisplacement(m.second->motor_id_global);
            msg.current[i] = icebusControl->GetCurrent(m.second->motor_id_global); // running mean filter the current
            // ROS_INFO_THROTTLE(1,"%x",msg.current[i]);
            i++;
        }
        motorState->publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::MotorInfoPublisher() {
    rclcpp::Rate rate(10);
    int32_t light_up_motor = 0;
    bool dir = true;
    while (keep_publishing && rclcpp::ok()) {
        roboy_middleware_msgs::msg::MotorInfo msg;
        int motor = 0;
        for (auto &m:icebusControl->motor_config->motor) {
            int32_t Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit;
            icebusControl->GetControllerParameter(m.second->motor_id_global, Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit);
            msg.control_mode.push_back(icebusControl->GetControlMode(m.second->motor_id_global));
            msg.kp.push_back(Kp);
            msg.ki.push_back(Ki);
            msg.kd.push_back(Kd);
            msg.deadband.push_back(deadband);
            msg.intergral_limit.push_back(IntegralLimit);
            msg.pwm_limit.push_back(PWMLimit);
            msg.current_limit.push_back(icebusControl->GetCurrentLimit(m.second->motor_id_global));
            int32_t communication_quality = icebusControl->GetCommunicationQuality(m.second->motor_id_global);
            string error_code = icebusControl->GetErrorCode(m.second->motor_id_global);
            if(communication_quality==0 && error_code!="timeout")
              error_code = "---";
            msg.communication_quality.push_back(communication_quality);
            msg.error_code.push_back(error_code);
            msg.neopixel_color.push_back(icebusControl->GetNeopixelColor(m.second->motor_id_global));
            msg.setpoint.push_back(icebusControl->GetSetPoint(m.second->motor_id_global));
            msg.pwm.push_back(icebusControl->GetPWM(m.second->motor_id_global));
            if(!external_led_control){
              if(light_up_motor==motor)
                  icebusControl->SetNeopixelColor(m.second->motor_id_global,0x00000F);
              else
                  icebusControl->SetNeopixelColor(m.second->motor_id_global,0);
            }
            motor++;
        }
        if(!external_led_control){
          if(dir)
              light_up_motor++;
          else
              light_up_motor--;
          if(light_up_motor>=icebusControl->motor_config->total_number_of_motors && dir)
              dir = false;
          if(light_up_motor<0 && !dir)
              dir = true;
        }
        
        motorInfo->publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::Neopixel(const roboy_middleware_msgs::msg::Neopixel::SharedPtr msg){
  uint i = 0;
  external_led_control = true;
  for (auto motor:msg->motor) {
      for(auto &bus:motorControl){
        if(bus->MyMotor(motor)){
          int32_t color = int32_t(msg->g<<16|msg->r<<8|msg->b);
          bus->SetNeopixelColor(motor, color);
        }
      }
      i++;
  }
}

uint8_t RoboyPlexus::reverseBits(uint8_t v){
  uint8_t r = v; // r will be reversed bits of v; first get LSB of v
  int s = sizeof(v) * CHAR_BIT - 1; // extra shift needed at end

  for (v >>= 1; v; v >>= 1)
  {
    r <<= 1;
    r |= v & 1;
    s--;
  }
  r <<= s; // shift when v's highest bits are zero
  return r;
}

void RoboyPlexus::MagneticJointPublisher() {
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
      int i=0;
      for(auto ball:balljoints){
        roboy_middleware_msgs::msg::MagneticSensor msg;
        msg.id = i;
        ball->readMagneticData(msg.x,msg.y,msg.z);
        magneticSensor->publish(msg);
        i++;
      }
      rate.sleep();
    }
}

void RoboyPlexus::MotorCommand(const roboy_middleware_msgs::msg::MotorCommand::SharedPtr msg) {
    uint i = 0;
    for (auto motor:msg->motor) {
      for(auto &bus:motorControl){
        if(bus->MyMotor(motor)){
          if(control_mode[motor]!=3){
            bus->SetPoint(motor, msg->setpoint[i]);
          }else{
            bool direct_pwm_override;
            nh->get_parameter("direct_pwm_override",direct_pwm_override);
            if(fabsf(msg->setpoint[i])>128 && !direct_pwm_override) {
                RCUTILS_LOG_WARN_THROTTLE(rcutils_steady_time_now,1,"setpoints exceeding sane direct pwm values (>128), "
                                    "what the heck are you publishing?!");
            }else {
                bus->SetPoint(motor, msg->setpoint[i]);
            }
          }
        }
      }
      i++;
    }
}

void RoboyPlexus::MotorControl(const roboy_middleware_msgs::msg::MotorControl::SharedPtr msg) {
    uint i = 0;
    for (auto motor:msg->motor) {
        for(auto &bus:motorControl){
          if(bus->MyMotor(motor)){
            bus->SetPoint(motor, msg->setpoint[i]);
          }
        }
        i++;
    }
}

void RoboyPlexus::MotorConfigService(const shared_ptr<roboy_middleware_msgs::srv::MotorConfigService::Request> req,
                                    shared_ptr<roboy_middleware_msgs::srv::MotorConfigService::Response> res) {
    stringstream str;
    uint i = 0;
    for (int motor:req->config.motor) {
        control_Parameters_t params;
        icebusControl->GetDefaultControlParams(&params, req->config.control_mode[i]);
        params.control_mode = req->config.control_mode[i];
        if (req->config.control_mode[i] == 0)
            str << "\t" << (int) motor << ": ENCODER0";
        if (req->config.control_mode[i] == 1)
            str << "\t" << (int) motor << ": ENCODER1";
        if (req->config.control_mode[i] == 2)
            str << "\t" << (int) motor << ": DISPLACEMENT";
        if (req->config.control_mode[i] == 3)
            str << "\t" << (int) motor << ": DIRECT_PWM";
        if(i<req->config.pwm_limit.size())
            params.PWMLimit = req->config.pwm_limit[i];
        if(i<req->config.kp.size())
            params.Kp = req->config.kp[i];
        if(i<req->config.ki.size())
            params.Ki = req->config.ki[i];
        if(i<req->config.kd.size())
            params.Kd = req->config.kd[i];
        if(i<req->config.deadband.size())
            params.deadband = req->config.deadband[i];
        if(i<req->config.integral_limit.size())
            params.IntegralLimit = req->config.integral_limit[i];
        if(i<req->config.update_frequency.size())
            icebusControl->SetMotorUpdateFrequency(motor, req->config.update_frequency[i]);
        params.control_mode = req->config.control_mode[i];
        icebusControl->SetControlMode(motor, req->config.control_mode[i], params);
        res->mode.push_back(params.control_mode);
        icebusControl->SetControlMode(motor, req->config.control_mode[i], params, req->config.setpoint[i]);

        RCLCPP_INFO(nh->get_logger(), "setting motor %d to control mode %d with setpoint %d", motor, req->config.control_mode[i],
                 req->config.setpoint[i]);
        control_mode[motor] = req->config.control_mode[i];
        i++;
    }

    RCLCPP_INFO(nh->get_logger(), "serving motor config service for %s control", str.str().c_str());
//    return true;
}

void RoboyPlexus::ControlModeService(const shared_ptr<roboy_middleware_msgs::srv::ControlMode::Request> req,
                                     shared_ptr<roboy_middleware_msgs::srv::ControlMode::Response> res) {
    if (!emergency_stop) {
        if (req->motor_id.empty()) {
            RCLCPP_ERROR(nh->get_logger(), "no motor ids defined, cannot change control mode");
//            return false;
        } else {
            int i=0;
            for (int motor:req->motor_id) {
                for(auto &bus:motorControl){
                  if(bus->MyMotor(motor)){
                    bus->SetControlMode(motor, req->control_mode);
                    control_mode[motor] = req->control_mode;
                    if(i<req->set_points.size()){
                      bus->SetPoint(motor, req->set_points[i]);
                    }
                    RCLCPP_INFO(nh->get_logger(), "changing control mode of motor %d on %s to %d", motor, bus->whoami().c_str(), req->control_mode);
                  }
                }
                i++;
            }
        }
//        return true;
    } else {
        RCLCPP_WARN(nh->get_logger(), "emergency stop active, can NOT change control mode");
//        return false;
    }
}

void RoboyPlexus::ElbowJointPublisher(){
    sensor_msgs::msg::JointState msg;
    msg.name = {"elbow_left_axis0","elbow_left_axis1"};
    msg.position = {0,0};
    msg.velocity = {0,0};
    msg.effort = {0,0};
    rclcpp::Rate rate(30);
    vector<float> offsets = {94, -17.3};
    vector<float> angles = {0,0}, angles_prev = {0,0};
    vector<int> overflow_counter = {0,0};
    vector<int> sign = {-1,1};
    vector<int> order = {1,0};

    while(rclcpp::ok()){
        int k = 0;
        for(int j=0;j<a1335.size();j++){
          vector<A1335State> state;
          a1335[j]->readAngleData(state);
          for(int i=0;i<state.size();i++){
              if(angles_prev[k]>340 && state[i].angle < 20)
                overflow_counter[k]++;
              if(angles_prev[k]<20 && state[i].angle > 340)
                overflow_counter[k]--;
              angles[k] = 0.2f*(state[i].angle + overflow_counter[k]*360 - offsets[k]) + 0.8f*angles[k];
              angles_prev[k] = state[i].angle;
              msg.position[order[k]] = angles[k]*M_PI/180.0f*sign[k];
              k++;
          }
        }
        jointState->publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::MotorCalibrationService(const shared_ptr<roboy_middleware_msgs::srv::MotorCalibrationService::Request> req,
                                          shared_ptr<roboy_middleware_msgs::srv::MotorCalibrationService::Response> res) {
    if (!emergency_stop) {
        RCLCPP_INFO(nh->get_logger(), "serving motor calibration service for motor %d", req->motor);
        icebusControl->EstimateSpringParameters(req->motor, req->degree, res->estimated_spring_parameters,
                                             req->timeout, req->number_of_data_points, req->displacement_min,
                                             req->displacement_max, res->load, res->displacement);
        cout << "coefficients:\t";
        for (uint i = 0; i < req->degree; i++) {
            cout << res->estimated_spring_parameters[i] << "\t";
        }
        cout << endl;
//        return true;
    }
//    else {
//        return false;
//    }
}

void RoboyPlexus::MyoBrickCalibrationService(const shared_ptr<roboy_middleware_msgs::srv::MyoBrickCalibrationService::Request> req,
                                             shared_ptr<roboy_middleware_msgs::srv::MyoBrickCalibrationService::Response> res) {
    if (!emergency_stop) {
        RCLCPP_INFO(nh->get_logger(), "serving myobrick calibration service for motor %d", req->motor);
        icebusControl->EstimateMotorAngleLinearisationParameters(req->motor, req->degree, res->estimated_spring_parameters,
                                                              req->timeout, req->number_of_data_points, req->min_degree,
                                                              req->max_degree, res->motor_angle, res->motor_encoder);
        cout << "coefficients:\t";
        for (uint i = 0; i < req->degree; i++) {
            cout << res->estimated_spring_parameters[i] << "\t";
        }
        cout << endl;
//        return true;
    }
//    else {
//        return false;
//    }
}

void RoboyPlexus::EmergencyStopService(const shared_ptr<std_srvs::srv::SetBool::Request> req,
                                       shared_ptr<std_srvs::srv::SetBool::Response> res) {

    if (req->data == 1) {
        RCLCPP_INFO(nh->get_logger(), "emergency stop service called");
        // switch to displacement
        rclcpp::Rate rate(100);
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
            icebusControl->SetControlMode(motor, DISPLACEMENT, params);
        }
        emergency_stop = true;
    } else {
        RCLCPP_INFO(nh->get_logger(), "resuming normal operation");
        uint motor = 0;
        for (auto &params:control_params_backup) {
            icebusControl->SetControlMode(motor, control_mode_backup[motor], params.second[control_mode_backup[motor]]);
            motor++;
        }
        emergency_stop = false;
    }
}

void RoboyPlexus::SystemCheckService(const shared_ptr<roboy_middleware_msgs::srv::SystemCheck::Request> req,
                                    shared_ptr< roboy_middleware_msgs::srv::SystemCheck::Response> res) {
//    vector<uint8_t> motorIDs;
//    if (req->motorids.empty()) {
//        motorIDs.resize(icebusControl->motor_config->total_number_of_motors);
//        int i = 0;
//        for (auto &m:motorIDs) {
//            m = i;
//            i++;
//        }
//    } else {
//        motorIDs = req->motorids;
//    }
//    bool system_check_successful = true;
//    for (auto &m:motorIDs) {
//        if (icebusControl->GetCurrent(m,0) == 0) {
//            system_check_successful = false;
//            res->position.push_back(false);
//            res->displacement.push_back(false);
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
//            res->position.push_back(position_control);
//            res->displacement.push_back(displacement_control);
//        }
//    }
//    return system_check_successful;
}

void RoboyPlexus::SetDisplacementForAll(const shared_ptr<roboy_middleware_msgs::srv::SetInt16::Request> req,
                                        shared_ptr<roboy_middleware_msgs::srv::SetInt16::Request> res) {
    RCLCPP_INFO(nh->get_logger(), "all to displacement %d called", req->setpoint);
    for (auto motor:req->motors) {
        icebusControl->SetControlMode(motor, DISPLACEMENT);
        icebusControl->SetPoint(motor, req->setpoint);
        control_mode[motor] = DISPLACEMENT;
    }
}

void RoboyPlexus::StartRecordTrajectoryCB(const roboy_control_msgs::msg::StartRecordTrajectory::SharedPtr msg) {
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
    RCLCPP_INFO(nh->get_logger(), str.str());
    float samplingTime = 5; // 200 Hz is the fastest update rate for the motors
    string name = body_part + "_" + msg->name;
    vector<int> idList(begin(msg->id_list), end(msg->id_list));
    map<int, vector<float>> trajectories;

    icebusControl->StartRecordTrajectories(samplingTime, trajectories, idList, name);

}

void RoboyPlexus::StopRecordTrajectoryCB(const std_msgs::msg::Empty::SharedPtr msg) {
    icebusControl->StopRecordTrajectories();
}

void RoboyPlexus::SaveBehaviorCB(const roboy_control_msgs::msg::Behavior &msg) {

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
//        rclcpp::Duration(pause).sleep(1);
        //TODO: ROS2 sleep
        success = true && success;
    } else if (actionName.find("relax") != std::string::npos) {
//        icebusControl->SetAllToDisplacement(0);
        success = true && success;
    } else {
        actionName = icebusControl->trajectories_folder + actionName;
        success = icebusControl->PlayTrajectory(actionName.c_str()) && success;
    }

    return success;
}

void RoboyPlexus::EnablePlaybackCB(const std_msgs::msg::Bool::SharedPtr msg) {
    icebusControl->SetReplay(msg->data);
}

void RoboyPlexus::PredisplacementCB(const std_msgs::msg::Int32 &msg) {
    icebusControl->SetPredisplacement(msg.data);
}


void RoboyPlexus::ListExistingItemsService(const shared_ptr<roboy_control_msgs::srv::ListItems::Request> req,
                                           shared_ptr<roboy_control_msgs::srv::ListItems::Response> res) {

    // read filenames in the specified folder
    DIR *dirp = opendir(req->name.c_str());
    struct dirent *dp;
    if (!dirp) {
        mkdir(req->name.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
        RCLCPP_INFO(nh->get_logger(), "created trajectories folder");
        dirp = opendir(req->name.c_str());
    }
    while ((dp = readdir(dirp)) != NULL) {
        if (dp->d_type != DT_DIR) {
            res->items.push_back(dp->d_name);
        }
    }
    closedir(dirp);
//    return true;
}

void RoboyPlexus::ExpandBehaviorService(const shared_ptr<roboy_control_msgs::srv::ListItems::Request> req,
                                        shared_ptr<roboy_control_msgs::srv::ListItems::Response> res) {

    res->items = ExpandBehavior(req->name);
//    return true;
}

vector<string> RoboyPlexus::ExpandBehavior(string name) {
    std::vector<string> actions;
    ifstream input_file(icebusControl->behaviors_folder + name);
    std::copy(std::istream_iterator<std::string>(input_file),
              std::istream_iterator<std::string>(),
              std::back_inserter(actions));

    return actions;
}
