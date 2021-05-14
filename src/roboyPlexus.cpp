#include "roboyPlexus.hpp"

RoboyPlexus::RoboyPlexus(string robot_name, IcebusControlPtr icebusControl,
        vector<TLE493DPtr> balljoints,
        vector<FanControlPtr> fanControls,
        int32_t *led,
        int32_t *switches,
        int32_t *power_control,
        int32_t *power_sense,
        vector<int32_t *> &i2c_base,
        int32_t * tli4970_base,
        vector<int> elbow_sensor_order,
        vector<int> elbow_sensor_sign,
        vector<float> elbow_sensor_offset,
        vector<int> knee_sensor_order,
        vector<int> knee_sensor_sign,
        vector<float> knee_sensor_offset,
        MyoControlPtr myoControl) :
        robot_name(robot_name),
        icebusControl(icebusControl), fanControls(fanControls), balljoints(balljoints),
        power_control(power_control), power_sense(power_sense), switches(switches), led(led),
        elbow_sensor_order(elbow_sensor_order),elbow_sensor_sign(elbow_sensor_sign),elbow_sensor_offset(elbow_sensor_offset),
        knee_sensor_order(knee_sensor_order),knee_sensor_sign(knee_sensor_sign),knee_sensor_offset(knee_sensor_offset),
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

    if(i2c_base.size()>=2){
      ROS_INFO("initializing elbow joints");
      vector <uint8_t> ids = {0xC,0xD};
      ROS_INFO("right elbow:");
      a1335_elbow.push_back(A1335Ptr(new A1335(i2c_base[0],ids)));
      ROS_INFO("left elbow:");
      a1335_elbow.push_back(A1335Ptr(new A1335(i2c_base[1],ids)));

      jointState = nh->advertise<sensor_msgs::JointState>("external_joint_states",1);
      elbowJointAngleThread = boost::shared_ptr<std::thread>(
              new std::thread(&RoboyPlexus::ElbowJointPublisher, this));
      elbowJointAngleThread->detach();
    }
    if(i2c_base.size()>=4){
      ROS_INFO("initializing knee joints");
      vector <uint8_t> ids = {0xC,0xD};
      ROS_INFO("right knee:");
      a1335_knee.push_back(A1335Ptr(new A1335(i2c_base[2],ids)));
      ROS_INFO("left knee:");
      a1335_knee.push_back(A1335Ptr(new A1335(i2c_base[3],ids)));

      kneeJointAngleThread = boost::shared_ptr<std::thread>(
              new std::thread(&RoboyPlexus::KneeJointPublisher, this));
      kneeJointAngleThread->detach();
    }

    if (!balljoints.empty()) {
      magneticSensor = nh->advertise<roboy_middleware_msgs::MagneticSensor>(robot_name + "middleware/MagneticSensor",
                                                                            1);
      magneticsThread = boost::shared_ptr<std::thread>(
              new std::thread(&RoboyPlexus::MagneticJointPublisher, this));
      magneticsThread->detach();
    } else {
        ROS_WARN("no active ball joints");
    }

    motorState = nh->advertise<roboy_middleware_msgs::MotorState>(robot_name + "middleware/MotorState", 1);
    motorInfo = nh->advertise<roboy_middleware_msgs::MotorInfo>(robot_name + "middleware/MotorInfo", 1);
    roboyState = nh->advertise<roboy_middleware_msgs::RoboyState>(robot_name + "middleware/RoboyState", 1);

    motorStateThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::MotorStatePublisher, this));
    motorStateThread->detach();

    motorInfoThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::MotorInfoPublisher, this));
    motorInfoThread->detach();

    roboyStateThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::RoboyStatePublisher, this));
    roboyStateThread->detach();


    neopixel_sub = nh->subscribe(robot_name + "middleware/Neopixel", 1, &RoboyPlexus::Neopixel, this);
    emergencyStop_srv = nh->advertiseService(robot_name + "middleware/EmergencyStop",
                                             &RoboyPlexus::EmergencyStopService,
                                             this);
    power5V_srv = nh->advertiseService(robot_name + "middleware/PowerService5V",
                           &RoboyPlexus::PowerService5V,
                           this);
    power12V_srv = nh->advertiseService(robot_name + "middleware/PowerService12V",
                          &RoboyPlexus::PowerService12V,
                          this);
    systemcheck_srv = nh->advertiseService(robot_name + "middleware/SystemCheck",
                          &RoboyPlexus::SystemCheckService,
                          this);

    motorConfig_srv = nh->advertiseService(robot_name + "middleware/MotorConfig",
                                           &RoboyPlexus::MotorConfigService, this);
    controlMode_srv = nh->advertiseService(robot_name + "middleware/ControlMode",
                                           &RoboyPlexus::ControlModeService, this);
    motorCommand_sub = nh->subscribe(robot_name + "middleware/MotorCommand", 1, &RoboyPlexus::MotorCommand, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(0));
    spinner->start();

    if(!fanControls.empty()){
      for(auto fan:fanControls){
        fan->SetPWMFrequency(20000);
        fan->SetAutoFan(false);
        ROS_INFO("auto fan is %s", (fan->GetAutoFan()?"on":"off"));
        fan->SetSensitivity(1);
        ROS_INFO("fan pwm freq %d, duty %d",fan->GetPWMFrequency(), fan->GetDuty());
        fan->SetDuty(100);
      }
      fan_control_sub = nh->subscribe(robot_name + "middleware/FanControl", 1, &RoboyPlexus::FanControl, this);

      fan_control_srv = nh->advertiseService(robot_name + "middleware/FanControl",
                            &RoboyPlexus::FanControlService,
                            this);
    }

    if(tli4970_base!=nullptr){
      tli4970.reset(new TLI4970(tli4970_base));
    }

    powerTrackerThread = boost::shared_ptr<std::thread>(new std::thread(&RoboyPlexus::MotorPowerTracker, this));
    powerTrackerThread->detach();

    ROS_INFO("roboy plexus initialized");
}

RoboyPlexus::~RoboyPlexus() {
    if (motorStateThread->joinable())
        motorStateThread->join();
    if (motorInfoThread->joinable())
        motorInfoThread->join();
    if (powerTrackerThread->joinable())
        powerTrackerThread->join();
}

void RoboyPlexus::MotorStatePublisher() {
  ros::Rate rate(100);

  while (keep_publishing && ros::ok()) {
      roboy_middleware_msgs::MotorState msg;
      for(auto &bus:motorControl){
        for(auto m:bus->motor_config->motor){
          if(bus->MyMotor(m.first)){
            msg.header.stamp = ros::Time::now();
            msg.global_id.push_back(m.first);
            switch(control_mode[m.first]){
              case ENCODER0_POSITION: msg.setpoint.push_back(bus->GetSetPoint(m.first)*m.second->encoder0_conversion_factor); break;
              case ENCODER1_POSITION: msg.setpoint.push_back(bus->GetSetPoint(m.first)*m.second->encoder1_conversion_factor); break;
              case DIRECT_PWM: msg.setpoint.push_back(bus->GetSetPoint(m.first)*m.second->direction);
              default: msg.setpoint.push_back(bus->GetSetPoint(m.first));
            }
            msg.encoder0_pos.push_back(bus->GetEncoderPosition(m.first,ENCODER0_POSITION)*m.second->encoder0_conversion_factor);//*m.second->direction);
            msg.encoder1_pos.push_back(bus->GetEncoderPosition(m.first,ENCODER1_POSITION)*m.second->encoder1_conversion_factor);
            msg.displacement.push_back(bus->GetDisplacement(m.first));
            // msg.displacement.push_back(msg.encoder0_pos.back()-msg.encoder1_pos.back());
            auto current = bus->GetCurrent(m.first);
            msg.current.push_back(current);

          }
        }
      }
      motorState.publish(msg);
      rate.sleep();
  }
}

void RoboyPlexus::MotorInfoPublisher() {
    ros::Rate rate(10);

    bool blink = true;
    ros::Time t0 = ros::Time::now();

    while (keep_publishing && ros::ok()) {
        int bus_number = 0;
        roboy_middleware_msgs::MotorInfo msg;
        for(auto &bus:motorControl){
          for(auto m:bus->motor_config->motor){
            if(bus->MyMotor(m.first)){
              msg.global_id.push_back(m.first);
              int32_t Kp, Ki, Kd, deadband, IntegralLimit;
              float PWMLimit;
              bus->GetControllerParameter(m.first, Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit);
              msg.control_mode.push_back(bus->GetControlMode(m.first));
              msg.Kp.push_back(Kp);
              msg.Ki.push_back(Ki);
              msg.Kd.push_back(Kd);
              msg.deadband.push_back(deadband);
              msg.IntegralLimit.push_back(IntegralLimit);
              msg.PWMLimit.push_back(PWMLimit); // 1600 is the max 32MHz/20kHz=1600, shows in percent
              msg.current_limit.push_back(bus->GetCurrentLimit(m.first));
              int32_t communication_quality = bus->GetCommunicationQuality(m.first);
              string error_code = bus->GetErrorCode(m.first);
              // if(communication_quality==0 && error_code!="timeout")
              //   error_code = "---";
              msg.communication_quality.push_back(communication_quality);
              msg.error_code.push_back(error_code);
              msg.neopixelColor.push_back(bus->GetNeopixelColor(m.first));
              switch(control_mode[m.first]){
                case ENCODER0_POSITION: msg.setpoint.push_back(bus->GetSetPoint(m.first)*m.second->encoder0_conversion_factor); break;
                case ENCODER1_POSITION: msg.setpoint.push_back(bus->GetSetPoint(m.first)*m.second->encoder1_conversion_factor); break;
                default: msg.setpoint.push_back(bus->GetSetPoint(m.first));
              }
              msg.pwm.push_back(bus->GetPWM(m.first));
              if(!external_led_control){
                if(blink)
                    bus->SetNeopixelColor(m.first,int32_t(0<<16|0<<8|50));
                else
                    bus->SetNeopixelColor(m.first,0);

                if((ros::Time::now()-t0).toSec()>1){
                  t0 = ros::Time::now();
                  blink = !blink;
                }
              }
            }
          }
          bus_number++;
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

void RoboyPlexus::MotorPowerTracker(){
    ros::Rate rate(200);
    // TODO read values from the config
    vector<int> hand_motors = {38,39,40,41,42,43,44,45};
    while(ros::ok()) {

        for(auto &bus:motorControl){
            for(auto motor:bus->motor_config->motor) {
                if(bus->MyMotor(motor.first)) {
                    if (motor.second->muscleType != "openBionics" ) {


                        //                    // ignore hand motors
                        //                    //TODO proper implementation
                        //                    if(std::find(hand_motors.begin(), hand_motors.end(), motor.first) != hand_motors.end()) {
                        //                        continue;
                        //                    }
                        if (bus->GetCommunicationQuality(motor.first) > 0) {
                            motor.second->is_on = true;
                        } else { // as long as no communication - setpoint 0
                            bus->SetPoint(motor.first, 0);
                            motor.second->is_on = false;
                            ROS_INFO_THROTTLE(3, "motor %d on %s is off. Will ignore setpoints arriving.",
                                              motor.second->motor_id_global, bus->whoami().c_str());
                        }
                    }
                }
            }
        }
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

void RoboyPlexus::MagneticJointPublisher() {
    ROS_INFO("MagneticJointPublisher started");
    ros::Rate rate(100);
    while (ros::ok()) {
      int i=0;
      for(auto ball:balljoints){
        roboy_middleware_msgs::MagneticSensor msg;
        msg.header.stamp = ros::Time::now();
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
    for (auto motor:msg->global_id) {
      for(auto &bus:motorControl){
        if(bus->MyMotor(motor)){
          switch(control_mode[motor]){
            case ENCODER0_POSITION: {
              int setpoint = msg->setpoint[i]/bus->motor_config->motor[motor]->encoder0_conversion_factor*bus->motor_config->motor[motor]->direction;
              bus->SetPoint(motor, setpoint);
              break;
            }
            case ENCODER1_POSITION: {
              // TODO:  if (bus->motor_type == 'M3')
              //          int setpoint = msg->setpoint[i]/bus->motor_config->motor[motor]->encoder1_conversion_factor*bus->motor_config->motor[motor]->direction;
              //        else
              //          ....
              int setpoint = msg->setpoint[i]/bus->motor_config->motor[motor]->encoder1_conversion_factor;
              bus->SetPoint(motor, setpoint);
              break;
            }
            case DISPLACEMENT: {
              // TODO:  if (bus->motor_type == 'M3')
              //          bus->SetPoint(motor, msg->setpoint[i]*bus->motor_config->motor[motor]->direction;);
              //        else
              //          ....
              bus->SetPoint(motor, msg->setpoint[i]);
              break;
            }
            case DIRECT_PWM: {
              if(fabsf(msg->setpoint[i])>100.0f){
                ROS_WARN("you are sending motor commands in direct_pwm mode bigger than 100%, that doesn't make sense");
              }else{
                // TODO remove magic numbers
                auto sp = msg->setpoint[i]/100.0f*1600; // 1600 is the max pwm 32Mhz/20kHz=1600
                sp *= bus->motor_config->motor[motor]->direction;
                bus->SetPoint(motor, sp); // 1600 is the max pwm 32Mhz/20kHz=1600
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
    for (int motor:req.config.global_id) {
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
        if (req.global_id.empty()) {
            ROS_ERROR("no motor ids defined, cannot change control mode");
            return false;
        } else {
            int i=0;
            for (int motor:req.global_id) {
                for(auto &bus:motorControl){
                  if(bus->MyMotor(motor)){
                    bus->SetControlMode(motor, req.control_mode);
                    control_mode[motor] = req.control_mode;
                    if(i<req.set_points.size()){
                      switch(control_mode[motor]){
                        case ENCODER0_POSITION: {
                          int setpoint = req.set_points[i]/bus->motor_config->motor[motor]->encoder0_conversion_factor*bus->motor_config->motor[motor]->direction;
                          bus->SetPoint(motor, setpoint);
                          break;
                        }
                        case ENCODER1_POSITION: {
                          // TODO:  if (bus->motor_type == 'M3')
                          //          int setpoint = req.set_points[i]/bus->motor_config->motor[motor]->encoder1_conversion_factor*bus->motor_config->motor[motor]->direction;
                          //        else
                          //          ....
                          int setpoint = req.set_points[i]/bus->motor_config->motor[motor]->encoder1_conversion_factor;
                          bus->SetPoint(motor, setpoint);
                          break;
                        }
                        case DISPLACEMENT: {
                          // TODO:  if (bus->motor_type == 'M3')
                          //          bus->SetPoint(motor, msg->setpoint[i]*bus->motor_config->motor[motor]->direction;);
                          //        else
                          //          ....
                          bus->SetPoint(motor, req.set_points[i]);
                          break;
                        }
                        case DIRECT_PWM: {
                          if(fabsf(req.set_points[i])>100.0f){
                            ROS_WARN("you are sending motor commands in direct_pwm mode bigger than 100%, that doesn't make sense");
                          }else{
                            bus->SetPoint(motor, req.set_points[i]/100.0f*1600.0f*bus->motor_config->motor[motor]->direction);
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
    msg.name = {"elbow_right_axis0","elbow_right_axis1","elbow_left_axis0","elbow_left_axis1"};
    msg.position = {0,0,0,0};
    msg.velocity = {0,0,0,0};
    msg.effort = {0,0,0,0};
    ros::Rate rate(30);
    vector<float> angles = {0,0,0,0}, angles_prev = {0,0,0,0};
    vector<int> overflow_counter = {0,0,0,0};

    while(ros::ok()){
        int k = 0;
        for(int j=0;j<a1335_elbow.size();j++){
          vector<A1335State> state;
          a1335_elbow[j]->readAngleData(state);
          for(int i=0;i<state.size();i++){
              if(angles_prev[k]>340 && state[i].angle < 20)
                overflow_counter[k]++;
              if(angles_prev[k]<20 && state[i].angle > 340)
                overflow_counter[k]--;
              angles[k] = 0.2f*((state[i].angle + overflow_counter[k]*360)*M_PI/180.0f-elbow_sensor_offset[k]) + 0.8f*angles[k];
              angles_prev[k] = state[i].angle;
              msg.position[elbow_sensor_order[k]] = angles[k]*elbow_sensor_sign[k];
              k++;
          }
        }
        jointState.publish(msg);
        rate.sleep();
    }
}

void RoboyPlexus::KneeJointPublisher(){
    sensor_msgs::JointState msg;
    msg.name = {"knee_right_axis0","knee_right_axis1","knee_left_axis0","knee_left_axis1"};
    msg.position = {0,0,0,0};
    msg.velocity = {0,0,0,0};
    msg.effort = {0,0,0,0};
    ros::Rate rate(30);
    vector<float> angles = {0,0,0,0}, angles_prev = {0,0,0,0};
    vector<int> overflow_counter = {0,0,0,0};

    while(ros::ok()){
        int k = 0;
        for(int j=0;j<a1335_knee.size();j++){
          vector<A1335State> state;
          a1335_knee[j]->readAngleData(state);
          for(int i=0;i<state.size();i++){
              if(angles_prev[k]>340 && state[i].angle < 20)
                overflow_counter[k]++;
              if(angles_prev[k]<20 && state[i].angle > 340)
                overflow_counter[k]--;
              angles[k] = 0.2f*((state[i].angle + overflow_counter[k]*360)*M_PI/180.0f-knee_sensor_offset[k]) + 0.8f*angles[k];
              angles_prev[k] = state[i].angle;
              msg.position[knee_sensor_order[k]] = angles[k]*knee_sensor_sign[k];
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
   vector<uint8_t> motorIDs;
   if (req.motorIDs.empty()) {
      motorIDs.resize((*motorControl.begin())->motor_config->total_number_of_motors);
       ROS_INFO("performing overall system check of %ld motors",motorIDs.size());
       int i = 0;
       for (auto &m:motorIDs) {
           m = i;
           i++;
       }
   } else {
       motorIDs = req.motorIDs;
   }

   res.encoder0_pos.resize(motorIDs.size());
   res.encoder1_pos.resize(motorIDs.size());
   res.communication_quality.resize(motorIDs.size());
   res.motorIDs = motorIDs;

   stringstream str;
   // note the current encoder positions
   map<int,float> encoder0_pos, encoder1_pos, displacement;
   str << endl << "motor  |  encoder0_pos  |  encoder1_pos  | displacement" << endl;
   for (auto &motor:motorIDs) {
     for(auto &bus:motorControl){
       if(bus->MyMotor(motor)){
         encoder0_pos[motor] = bus->GetEncoderPosition(motor,0);
         encoder1_pos[motor] = bus->GetEncoderPosition(motor,1);
         displacement[motor] = bus->GetDisplacement(motor);
         str << to_string(motor) << "\t" << encoder0_pos[motor] << "\t" << encoder1_pos[motor] << "\t" << displacement[motor] << endl;
       }
     }
   }
   ROS_INFO_STREAM(str.str());

   for (auto &motor:motorIDs) {
     for(auto &bus:motorControl){
       if(bus->MyMotor(motor)){
         encoder0_pos[motor] = bus->GetEncoderPosition(motor,0);
         encoder1_pos[motor] = bus->GetEncoderPosition(motor,1);
         displacement[motor] = bus->GetDisplacement(motor);
         str << to_string(motor) << "\t" << encoder0_pos[motor] << "\t" << encoder1_pos[motor] << "\t" << displacement[motor] << endl;
       }
     }
   }

   ROS_INFO("disabling motorConfig and controlMode services and motorCommand subrscribers");
   motorConfig_srv.shutdown();
   controlMode_srv.shutdown();
   motorCommand_sub.shutdown();

   ROS_INFO("changing to DIRECT_PWM mode with setpoint -3");
   for (auto &motor:motorIDs) {
     for(auto &bus:motorControl){
       if(bus->MyMotor(motor)){
         if(bus->GetMuscleType(motor)=="openBionics"){
           bus->SetPoint(motor,4000);
         }else if(bus->GetMuscleType(motor)=="m3"){
           bus->SetPoint(motor,-30);
         }else{
           bus->SetControlMode(motor,DIRECT_PWM);
           bus->SetPoint(motor,-0.03*1600.0f);
         }
       }
     }
   }
   ROS_INFO("waiting...");
   ros::Duration wait_for_position(2);
   wait_for_position.sleep();

   // note the current encoder positions
   map<int,float> encoder0_pos_t0, encoder1_pos_t0, displacement_t0;
   str.clear();
   str << endl << "motor  |  encoder0_pos  |  encoder1_pos  | displacement" << endl;
   for (auto &motor:motorIDs) {
     for(auto &bus:motorControl){
       if(bus->MyMotor(motor)){
         encoder0_pos_t0[motor] = bus->GetEncoderPosition(motor,0);
         encoder1_pos_t0[motor] = bus->GetEncoderPosition(motor,1);
         displacement_t0[motor] = bus->GetDisplacement(motor);
         str << to_string(motor) << "\t" << encoder0_pos_t0[motor] << "\t" << encoder1_pos_t0[motor] << "\t" << displacement_t0[motor] << endl;
       }
     }
   }
   ROS_INFO_STREAM(str.str());

   ROS_INFO("changing to DIRECT_PWM mode with setpoint 3");
   for (auto &motor:motorIDs) {
     for(auto &bus:motorControl){
       if(bus->MyMotor(motor)){
         if(bus->GetMuscleType(motor)=="openBionics"){
           bus->SetPoint(motor,0);
         }else if(bus->GetMuscleType(motor)=="m3"){
           bus->SetPoint(motor,30);
         }else{
           bus->SetControlMode(motor,DIRECT_PWM);
           bus->SetPoint(motor,0.03*1600.0f);
         }
       }
     }
   }
   ROS_INFO("waiting...");
   wait_for_position.sleep();

   // note the current encoder positions
   map<int,float> encoder0_pos_t1, encoder1_pos_t1, displacement_t1;
   str.clear();
   str << endl << "motor  |  encoder0_pos  |  encoder1_pos  | displacement" << endl;
   for (auto &motor:motorIDs) {
     for(auto &bus:motorControl){
       if(bus->MyMotor(motor)){
         encoder0_pos_t1[motor] = bus->GetEncoderPosition(motor,0);
         encoder1_pos_t1[motor] = bus->GetEncoderPosition(motor,1);
         displacement_t1[motor] = bus->GetDisplacement(motor);
         str << to_string(motor) << "\t" << encoder0_pos_t1[motor] << "\t" << encoder1_pos_t1[motor] << "\t" << displacement_t1[motor] << endl;
       }
     }
   }
   ROS_INFO_STREAM(str.str());
   ROS_INFO("disableing motors");
   for (auto &motor:motorIDs) {
     for(auto &bus:motorControl){
       if(bus->MyMotor(motor)){
         bus->SetControlMode(motor,DIRECT_PWM);
         bus->SetPoint(motor,0);
       }
     }
   }

   str.clear();
   str << "system check results" << endl;
   str << "motor  |  encoder0  | encoder1  | communication_quality" << endl;
   int i = 0;
   for (auto &motor:motorIDs) {
     for(auto &bus:motorControl){
       if(bus->MyMotor(motor)){
         if(bus->GetMuscleType(motor)=="myoBrick"){
           // check if encoder0 was rotating in the correct direction
           if(encoder0_pos_t0[motor]>=encoder0_pos[motor] || encoder0_pos_t1[motor]<=encoder0_pos_t0[motor])
            res.encoder0_pos[i] = false;
           else
            res.encoder0_pos[i] = true;
           // check if encoder1 was rotating in the correct direction
           if(encoder1_pos_t0[motor]>=encoder1_pos[motor] || encoder1_pos_t1[motor]<=encoder1_pos_t0[motor])
            res.encoder1_pos[i] = false;
           else
            res.encoder1_pos[i] = true;
           // check if we are able to talk to the motor at all
           if(bus->GetCommunicationQuality(motor)>0)
            res.communication_quality[i] = true;
           else
            res.communication_quality[i] = false;
          }else if(bus->GetMuscleType(motor)=="m3"){
            // check if encoder0 was rotating in the correct direction
            if(encoder0_pos_t0[motor]<=encoder0_pos[motor] || encoder0_pos_t1[motor]>=encoder0_pos_t0[motor])
             res.encoder0_pos[i] = false;
            else
             res.encoder0_pos[i] = true;
            res.encoder1_pos[i] = true;
            // check if we are able to talk to the motor at all
            if(bus->GetCommunicationQuality(motor)>0)
             res.communication_quality[i] = true;
            else
             res.communication_quality[i] = false;
           }else{ // no feedback from openbionics hand
             res.encoder0_pos[i] = true;
             res.encoder1_pos[i] = true;
             res.communication_quality[i] = true;
           }
           str << to_string(res.motorIDs[i])
              << "\t" << (res.encoder0_pos[i]?"true":"false")
              << "\t" << (res.encoder1_pos[i]?"true":"false")
              << "\t" << (res.communication_quality[i]?"true":"false") << endl;
       }
     }
     i++;
   }

   ROS_INFO("enabling motorConfig and controlMode services and motorCommand subrscriber");
   motorConfig_srv = nh->advertiseService(robot_name + "middleware/MotorConfig",
                                          &RoboyPlexus::MotorConfigService, this);
   controlMode_srv = nh->advertiseService(robot_name + "middleware/ControlMode",
                                          &RoboyPlexus::ControlModeService, this);
   motorCommand_sub = nh->subscribe(robot_name + "middleware/MotorCommand", 1, &RoboyPlexus::MotorCommand, this);

   ROS_INFO_STREAM(str.str());

   return true;
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
