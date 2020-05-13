#include "roboyPlexus.hpp"

RoboyPlexus::RoboyPlexus(IcebusControlPtr icebusControl,
        vector<TLE493DPtr> balljoints,
        vector<FanControlPtr> fanControls,
        int32_t *led,
        int32_t *switches,
        int32_t *power_control,
        int32_t *power_sense,
        vector<int32_t *> &i2c_base,
        int32_t * tli4970_base,
        MyoControlPtr myoControl) :
        icebusControl(icebusControl), fanControls(fanControls), balljoints(balljoints),
        power_control(power_control), power_sense(power_sense), switches(switches), led(led),
        myoControl(myoControl){
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

    // turn on 5V and 12V lines
    *power_control = 0x0;
    power_5V_enabled = true;
    power_12V_enabled = true;

    motorControl.push_back(icebusControl);
    if(myoControl!=nullptr)
      motorControl.push_back(myoControl);

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

    if(i2c_base.size()>=2){
      {
        vector <uint8_t> ids = {0xC,0xD};
        a1335.push_back(A1335Ptr(new A1335(i2c_base[0],ids)));
      }
      // {
      //   vector <uint8_t> ids = {0xC,0xD};
      //   a1335.push_back(A1335Ptr(new A1335(i2c_base[1],ids)));
      // }
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

    if(tli4970_base!=nullptr){
      tli4970.reset(new TLI4970(tli4970_base));
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
  for (auto &m:motorControl[0]->motor_config->motor) {
    msg.global_id.push_back(m.first);
  }
  msg.setpoint.resize(motorControl[0]->motor_config->total_number_of_motors);
  msg.encoder0_pos.resize(motorControl[0]->motor_config->total_number_of_motors);
  msg.encoder1_pos.resize(motorControl[0]->motor_config->total_number_of_motors);
  msg.displacement.resize(motorControl[0]->motor_config->total_number_of_motors);
  msg.current.resize(motorControl[0]->motor_config->total_number_of_motors);

  while (keep_publishing && ros::ok()) {
      int i = 0;
      for(auto &bus:motorControl){
        for(auto m:bus->motor_config->motor){
          if(bus->MyMotor(m.first)){
            switch(control_mode[m.first]){
              case ENCODER0_POSITION: msg.setpoint[i] = bus->GetSetPoint(m.first)*m.second->encoder0_conversion_factor; break;
              case ENCODER1_POSITION: msg.setpoint[i] = bus->GetSetPoint(m.first)*m.second->encoder1_conversion_factor; break;
              default: msg.setpoint[i] = bus->GetSetPoint(m.first);
            }
            msg.encoder0_pos[i] = bus->GetEncoderPosition(m.first,ENCODER0_POSITION)*m.second->encoder0_conversion_factor;
            msg.encoder1_pos[i] = bus->GetEncoderPosition(m.first,ENCODER1_POSITION)*m.second->encoder1_conversion_factor;
            msg.displacement[i] = bus->GetDisplacement(m.first);
            msg.current[i] = bus->GetCurrent(m.first);
            i++;
          }
        }
      }
      motorState.publish(msg);
      rate.sleep();
  }
}

void RoboyPlexus::MotorInfoPublisher() {
    ros::Rate rate(10);
    int32_t light_up_motor = 0;
    bool dir = true;

    roboy_middleware_msgs::MotorInfo msg;
    for (auto &m:motorControl[0]->motor_config->motor) {
      msg.global_id.push_back(m.first);
    }
    msg.setpoint.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.control_mode.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.Kp.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.Ki.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.Kd.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.deadband.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.setpoint.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.IntegralLimit.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.PWMLimit.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.pwm.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.current_limit.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.communication_quality.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.error_code.resize(motorControl[0]->motor_config->total_number_of_motors);
    msg.neopixelColor.resize(motorControl[0]->motor_config->total_number_of_motors);

    while (keep_publishing && ros::ok()) {
        int i = 0;
        for(auto &bus:motorControl){
          for(auto m:bus->motor_config->motor){
            if(bus->MyMotor(m.first)){
              int32_t Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit;
              bus->GetControllerParameter(m.first, Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit);
              msg.control_mode[i] = bus->GetControlMode(m.first);
              msg.Kp[i] = Kp;
              msg.Ki[i] = Ki;
              msg.Kd[i] = Kd;
              msg.deadband[i] = deadband;
              msg.IntegralLimit[i] = IntegralLimit;
              msg.PWMLimit[i] = PWMLimit;
              msg.current_limit[i] = bus->GetCurrentLimit(m.first);
              int32_t communication_quality = bus->GetCommunicationQuality(m.first);
              string error_code = bus->GetErrorCode(m.first);
              // if(communication_quality==0 && error_code!="timeout")
              //   error_code = "---";
              msg.communication_quality[i] = communication_quality;
              msg.error_code[i] = error_code;
              msg.neopixelColor[i] = bus->GetNeopixelColor(m.first);
              switch(control_mode[m.first]){
                case ENCODER0_POSITION: msg.setpoint[i] = bus->GetSetPoint(m.first)*m.second->encoder0_conversion_factor; break;
                case ENCODER1_POSITION: msg.setpoint[i] = bus->GetSetPoint(m.first)*m.second->encoder1_conversion_factor; break;
                default: msg.setpoint[i] = bus->GetSetPoint(m.first);
              }
              msg.pwm[i] = bus->GetPWM(m.first);
              if(!external_led_control){
                if(light_up_motor==m.first)
                    bus->SetNeopixelColor(m.first,0x00000F);
                else
                    bus->SetNeopixelColor(m.first,0);
              }
              i++;
            }
          }

          if(!external_led_control){
            if(dir)
                light_up_motor++;
            else
                light_up_motor--;
            if(light_up_motor>=bus->motor_config->total_number_of_motors && dir)
                dir = false;
            if(light_up_motor<0 && !dir)
                dir = true;
          }
        }
        motorInfo.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::RoboyStatePublisher(){
  ros::Rate rate(2);
  while(ros::ok()){
    int32_t state = *power_sense;
    roboy_middleware_msgs::RoboyState msg;
    msg.power_sense.resize(6);
    int j = msg.power_sense.size()-1;
    for(int i=0;i<msg.power_sense.size();i++){
      msg.power_sense[j--] = !((state>>i)&0x1);
    }
    msg.power_5V_enabled = power_5V_enabled;
    msg.power_12V_enabled = power_12V_enabled;
    if(tli4970!=nullptr)
      tli4970->readCurrent(msg.current);
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
    ROS_INFO("MagneticJointPublisher started");
    ros::Rate rate(100);
    while (ros::ok()) {
      int i=0;
      for(auto ball:balljoints){
        roboy_middleware_msgs::MagneticSensor msg;
        msg.id = i;
        ball->readMagneticData(msg.sensor_id,msg.x,msg.y,msg.z);
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
          switch(control_mode[motor]){
            case ENCODER0_POSITION: {
              int setpoint = msg->setpoint[i]/bus->motor_config->motor[motor]->encoder0_conversion_factor;
              bus->SetPoint(motor, setpoint);
              break;
            }
            case ENCODER1_POSITION: {
              int setpoint = msg->setpoint[i]/bus->motor_config->motor[motor]->encoder1_conversion_factor;
              bus->SetPoint(motor, setpoint);
              break;
            }
            case DISPLACEMENT: {
              bus->SetPoint(motor, msg->setpoint[i]);
              break;
            }
            case DIRECT_PWM: {
              bool direct_pwm_override;
              nh->getParam("direct_pwm_override",direct_pwm_override);
              if(fabsf(msg->setpoint[i])>200 && !direct_pwm_override) {
                  ROS_WARN_THROTTLE(1,"setpoints exceeding sane direct pwm values (>200), "
                                      "what the heck are you publishing?!, "
                                      "you can enable/disable this check by setting the ros parameter direct_pwm_override, "
                                      "execute from the commandline:\n"
                                      "rosparam set direct_pwm_override true");
              }else {
                  bus->SetPoint(motor, msg->setpoint[i]);
              }
              break;
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
      for(auto &bus:motorControl){
        if(bus->MyMotor(motor)){
          control_Parameters_t params;
          bus->GetDefaultControlParams(&params, req.config.control_mode[i]);
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
              bus->SetMotorUpdateFrequency(motor, req.config.update_frequency[i]);
          if(req.config.setpoint.size()==req.config.control_mode.size())
            bus->SetControlMode(motor, req.config.control_mode[i], params, req.config.setpoint[i]);
          else
            bus->SetControlMode(motor, req.config.control_mode[i], params);
          params.control_mode = req.config.control_mode[i];
          res.mode.push_back(params.control_mode);

          ROS_INFO("setting motor %d to control mode %d with setpoint %d", motor, req.config.control_mode[i],
                   req.config.setpoint[i]);
          control_mode[motor] = req.config.control_mode[i];
          i++;
        }
      }
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
                      switch(control_mode[motor]){
                        case ENCODER0_POSITION: {
                          int setpoint = req.set_points[i]/bus->motor_config->motor[motor]->encoder0_conversion_factor;
                          bus->SetPoint(motor, setpoint);
                          break;
                        }
                        case ENCODER1_POSITION: {
                          int setpoint = req.set_points[i]/bus->motor_config->motor[motor]->encoder1_conversion_factor;
                          bus->SetPoint(motor, setpoint);
                          break;
                        }
                        case DISPLACEMENT: {
                          bus->SetPoint(motor, req.set_points[i]);
                          break;
                        }
                        case DIRECT_PWM: {
                          bool direct_pwm_override;
                          nh->getParam("direct_pwm_override",direct_pwm_override);
                          if(fabsf(req.set_points[i])>200 && !direct_pwm_override) {
                              ROS_WARN_THROTTLE(1,"setpoints exceeding sane direct pwm values (>200), "
                                                  "what the heck are you publishing?!, "
                                                  "you can enable/disable this check by setting the ros parameter direct_pwm_override, "
                                                  "execute from the commandline:\n"
                                                  "rosparam set direct_pwm_override true");
                          }else {
                              bus->SetPoint(motor, req.set_points[i]);
                          }
                          break;
                        }
                      }
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
