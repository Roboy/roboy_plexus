#include "roboyPlexus.hpp"

RoboyPlexus::RoboyPlexus(IcebusControlPtr icebusControl,
        vector<BallJointPtr> balljoints,
        vector<FanControlPtr> fanControls,
        int32_t *led,
        int32_t *switches,
        int32_t *power_control,
        int32_t *power_sense,
        vector<int32_t *> &i2c_base) :
        icebusControl(icebusControl), fanControls(fanControls), balljoints(balljoints),
        power_control(power_control), power_sense(power_sense), switches(switches), led(led),
        i2c_base(i2c_base){
    ROS_INFO("roboy3 plexus initializing");

    ifstream ifile("/sys/class/net/eth0/address");
    ifile >> ethaddr;
    ifile.close();
    string node_name = "roboy_fpga_" + ethaddr;
    replace(node_name.begin(), node_name.end(), ':', '_');

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    if (!balljoints.empty()) {
        magneticSensor = nh->advertise<roboy_middleware_msgs::MagneticSensor>("/roboy/middleware/MagneticSensor",
                                                                              1);
        magneticsThread = boost::shared_ptr<std::thread>(
                new std::thread(&RoboyPlexus::MagneticJointPublisher, this));
        magneticsThread->detach();
    } else {
        ROS_WARN("no active ball joints");
    }
    ROS_INFO("initializing elbow joints");

    if(i2c_base.size()>4){
      {
        vector <uint8_t> ids = {0xC};
        a1335.push_back(A1335Ptr(new A1335(i2c_base[4],ids)));
      }
      {
        vector <uint8_t> ids = {0xD};
        a1335.push_back(A1335Ptr(new A1335(i2c_base[5],ids)));
      }
      jointState = nh->advertise<sensor_msgs::JointState>("external_joint_states",1);
      elbowJointAngleThread = boost::shared_ptr<std::thread>(
              new std::thread(&RoboyPlexus::ElbowJointPublisher, this));
      elbowJointAngleThread->detach();
    }

    motorState = nh->advertise<roboy_middleware_msgs::MotorState>("/roboy/middleware/MotorState", 1);
    motorInfo = nh->advertise<roboy_middleware_msgs::MotorInfo>("/roboy/middleware/MotorInfo", 1);
    roboyState = nh->advertise<roboy_middleware_msgs::RoboyState>("/roboy/middleware/RoboyState", 1);

    motorStateThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::MotorStatePublisher, this));
    motorStateThread->detach();

    motorInfoThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::MotorInfoPublisher, this));
    motorInfoThread->detach();

    roboyStateThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::RoboyStatePublisher, this));
    roboyStateThread->detach();

    neopixel_sub = nh->subscribe("/roboy/middleware/Neopixel", 1, &RoboyPlexus::Neopixel, this);
    fan_control_sub = nh->subscribe("/roboy/middleware/FanControl", 1, &RoboyPlexus::FanControl, this);

    ROS_INFO("initializing icebus");
    for (auto &m:icebusControl->motor_config->motor) {
        icebusControl->SetMotorUpdateFrequency(m.second->motor_id_global,50);
        icebusControl->SetBaudrate(m.second->motor_id_global,19200);
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

    fan_control_srv = nh->advertiseService("/roboy/middleware/FanControl",
                          &RoboyPlexus::FanControlService,
                          this);

    motorConfig_srv = nh->advertiseService("/roboy/middleware/MotorConfig",
                                           &RoboyPlexus::MotorConfigService, this);
    controlMode_srv = nh->advertiseService("/roboy/middleware/ControlMode",
                                           &RoboyPlexus::ControlModeService, this);
    emergencyStop_srv = nh->advertiseService("/roboy/middleware/EmergencyStop",
                                             &RoboyPlexus::EmergencyStopService,
                                             this);
    power5V_srv = nh->advertiseService("/roboy/middleware/PowerService5V",
                           &RoboyPlexus::PowerService5V,
                           this);
    power12V_srv = nh->advertiseService("/roboy/middleware/PowerService12V",
                          &RoboyPlexus::PowerService12V,
                          this);
    motorCommand_sub = nh->subscribe("/roboy/middleware/MotorCommand", 1, &RoboyPlexus::MotorCommand, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
    spinner->start();

    if(!fanControls.empty()){
      for(auto fan:fanControls){
        fan->SetAutoFan(true);
        ROS_INFO("auto fan is %s", (fan->GetAutoFan()?"on":"off"));
        fan->SetSensitivity(1);
        ROS_INFO("fan pwm freq %d, duty %d",fan->GetPWMFrequency(), fan->GetDuty());
      }
      // ros::Rate rate(10);
      //
      // while(ros::ok()){
      //   for(auto fan:fanControls){
      //     ROS_INFO("duty %d, average current %d",fan->GetDuty(), fan->GetCurrentAverage());
      //     rate.sleep();
      //   }
      // }
    }
    ROS_INFO("roboy plexus initialized");
}

RoboyPlexus::~RoboyPlexus() {
    if (motorStateThread->joinable())
        motorStateThread->join();
    if (motorInfoThread->joinable())
        motorInfoThread->join();
}

void RoboyPlexus::MotorStatePublisher() {
    ros::Rate rate(100);
    roboy_middleware_msgs::MotorState msg;
    for (auto &m:icebusControl->motor_config->motor) {
      msg.global_id.push_back(m.second->motor_id_global);
    }
    msg.setpoint.resize(icebusControl->motor_config->motor.size());
    msg.encoder0_pos.resize(icebusControl->motor_config->motor.size());
    msg.encoder1_pos.resize(icebusControl->motor_config->motor.size());
    msg.displacement.resize(icebusControl->motor_config->motor.size());
    msg.current.resize(icebusControl->motor_config->motor.size());
    while (keep_publishing && ros::ok()) {
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
        motorState.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::MotorInfoPublisher() {
    ros::Rate rate(10);
    int32_t light_up_motor = 0;
    bool dir = true;
    while (keep_publishing && ros::ok()) {
        roboy_middleware_msgs::MotorInfo msg;
        int motor = 0;
        for (auto &m:icebusControl->motor_config->motor) {
            int32_t Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit;
            icebusControl->GetControllerParameter(m.second->motor_id_global, Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit);
            msg.control_mode.push_back(icebusControl->GetControlMode(m.second->motor_id_global));
            msg.Kp.push_back(Kp);
            msg.Ki.push_back(Ki);
            msg.Kd.push_back(Kd);
            msg.deadband.push_back(deadband);
            msg.IntegralLimit.push_back(IntegralLimit);
            msg.PWMLimit.push_back(PWMLimit);
            msg.current_limit.push_back(icebusControl->GetCurrentLimit(m.second->motor_id_global));
            int32_t communication_quality = icebusControl->GetCommunicationQuality(m.second->motor_id_global);
            string error_code = icebusControl->GetErrorCode(m.second->motor_id_global);
            // if(communication_quality==0 && error_code!="timeout")
            //   error_code = "---";
            msg.communication_quality.push_back(communication_quality);
            msg.error_code.push_back(error_code);
            msg.neopixelColor.push_back(icebusControl->GetNeopixelColor(m.second->motor_id_global));
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
        motorInfo.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::RoboyStatePublisher(){
  roboy_middleware_msgs::RoboyState msg;
  msg.power_sense.resize(6);
  ros::Rate rate(2);
  while(ros::ok()){
    int32_t state = *power_sense;
    int j = msg.power_sense.size()-1;
    for(int i=0;i<msg.power_sense.size();i++){
      msg.power_sense[j--] = !((state>>i)&0x1);
    }
    msg.power_5V_enabled = power_5V_enabled;
    msg.power_12V_enabled = power_12V_enabled;
    roboyState.publish(msg);
    rate.sleep();
  }
}

void RoboyPlexus::Neopixel(const roboy_middleware_msgs::Neopixel::ConstPtr &msg){
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
    ros::Rate rate(100);
    while (ros::ok()) {
      int i=0;
      for(auto ball:balljoints){
        roboy_middleware_msgs::MagneticSensor msg;
        msg.id = i;
        ball->readMagneticData(msg.x,msg.y,msg.z);
        magneticSensor.publish(msg);
        i++;
      }
      rate.sleep();
    }
}

void RoboyPlexus::MotorCommand(const roboy_middleware_msgs::MotorCommand::ConstPtr &msg) {
    uint i = 0;
    for (auto motor:msg->motor) {
      for(auto &bus:motorControl){
        if(bus->MyMotor(motor)){
          if(control_mode[motor]!=3){
            bus->SetPoint(motor, msg->setpoint[i]);
          }else{
            bool direct_pwm_override;
            nh->getParam("direct_pwm_override",direct_pwm_override);
            if(fabsf(msg->setpoint[i])>128 && !direct_pwm_override) {
                ROS_WARN_THROTTLE(1,"setpoints exceeding sane direct pwm values (>128), "
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

bool RoboyPlexus::MotorConfigService(roboy_middleware_msgs::MotorConfigService::Request &req,
                                     roboy_middleware_msgs::MotorConfigService::Response &res) {
    stringstream str;
    uint i = 0;
    for (int motor:req.config.motor) {
        control_Parameters_t params;
        icebusControl->GetDefaultControlParams(&params, req.config.control_mode[i]);
        params.control_mode = req.config.control_mode[i];
        if (req.config.control_mode[i] == 0)
            str << "\t" << (int) motor << ": ENCODER0";
        if (req.config.control_mode[i] == 1)
            str << "\t" << (int) motor << ": ENCODER1";
        if (req.config.control_mode[i] == 2)
            str << "\t" << (int) motor << ": DISPLACEMENT";
        if (req.config.control_mode[i] == 3)
            str << "\t" << (int) motor << ": DIRECT_PWM";
        if(i<req.config.PWMLimit.size())
            params.PWMLimit = req.config.PWMLimit[i];
        if(i<req.config.Kp.size())
            params.Kp = req.config.Kp[i];
        if(i<req.config.Ki.size())
            params.Ki = req.config.Ki[i];
        if(i<req.config.Kd.size())
            params.Kd = req.config.Kd[i];
        if(i<req.config.deadband.size())
            params.deadband = req.config.deadband[i];
        if(i<req.config.IntegralLimit.size())
            params.IntegralLimit = req.config.IntegralLimit[i];
        if(i<req.config.update_frequency.size())
            icebusControl->SetMotorUpdateFrequency(motor, req.config.update_frequency[i]);
        params.control_mode = req.config.control_mode[i];
        icebusControl->SetControlMode(motor, req.config.control_mode[i], params);
        res.mode.push_back(params.control_mode);
        icebusControl->SetControlMode(motor, req.config.control_mode[i], params, req.config.setpoint[i]);

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
            ROS_ERROR("no motor ids defined, cannot change control mode");
            return false;
        } else {
            int i=0;
            for (int motor:req.motor_id) {
                for(auto &bus:motorControl){
                  if(bus->MyMotor(motor)){
                    bus->SetControlMode(motor, req.control_mode);
                    control_mode[motor] = req.control_mode;
                    if(i<req.set_points.size()){
                      bus->SetPoint(motor, req.set_points[i]);
                    }
                    ROS_INFO("changing control mode of motor %d on %s to %d", motor, bus->whoami().c_str(), req.control_mode);
                  }
                }
                i++;
            }
        }
        return true;
    } else {
        ROS_WARN("emergency stop active, can NOT change control mode");
        return false;
    }
}

void RoboyPlexus::ElbowJointPublisher(){
    sensor_msgs::JointState msg;
    msg.name = {"elbow_left_axis0","elbow_left_axis1"};
    msg.position = {0,0};
    msg.velocity = {0,0};
    msg.effort = {0,0};
    ros::Rate rate(30);
    vector<float> offsets = {94, -17.3};
    vector<float> angles = {0,0}, angles_prev = {0,0};
    vector<int> overflow_counter = {0,0};
    vector<int> sign = {-1,1};
    vector<int> order = {1,0};

    while(ros::ok()){
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
        jointState.publish(msg);
        rate.sleep();
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
            icebusControl->SetControlMode(motor, DISPLACEMENT, params);
        }
        emergency_stop = true;
    } else {
        ROS_INFO("resuming normal operation");
        uint motor = 0;
        for (auto &params:control_params_backup) {
            icebusControl->SetControlMode(motor, control_mode_backup[motor], params.second[control_mode_backup[motor]]);
            motor++;
        }
        emergency_stop = false;
    }
    return true;
}

void RoboyPlexus::FanControl(std_msgs::Int32 duty){
  for(auto fan:fanControls){
    if(!fan->GetAutoFan()){
      fan->SetDuty(duty.data);
    }
  }
}

bool RoboyPlexus::FanControlService(std_srvs::SetBool::Request &req,
                  std_srvs::SetBool::Response &res){
    for(auto fan:fanControls){
      fan->SetAutoFan(req.data);
    }
    if(req.data){
      res.message = "auto fan enabled";
    }else{
      res.message = "auto fan disabled";
    }
    res.success = true;
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

bool RoboyPlexus::PowerService5V(std_srvs::SetBool::Request &req,
                    std_srvs::SetBool::Response &res){
                      power_5V_enabled = req.data;
                      *power_control = (!power_5V_enabled<<1|!power_12V_enabled);
                      res.success = true;
                      if(power_5V_enabled)
                        res.message = "5V enabled";
                      else
                        res.message = "5V disabled";
                      return true;
}

bool RoboyPlexus::PowerService12V(std_srvs::SetBool::Request &req,
                    std_srvs::SetBool::Response &res){
                      power_12V_enabled = req.data;
                      *power_control = (!power_5V_enabled<<1|!power_12V_enabled);
                      res.success = true;
                      if(power_12V_enabled)
                        res.message = "12V enabled";
                      else
                        res.message = "12V disabled";
                      return true;
                    }
