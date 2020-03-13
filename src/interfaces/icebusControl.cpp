#include "interfaces/icebusControl.hpp"

IcebusControl::IcebusControl(string motor_config_filepath, vector<int32_t *> &mb, int32_t *adc_base, NeoPixelPtr neopixel)
                        : adc_base(adc_base), neopixel(neopixel) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"initializing icebusControl for %d icebuses with motor config file %s", mb.size(), motor_config_filepath.c_str());
    motor_config = MotorConfigPtr(new MotorConfig);
    motor_config->readConfig(motor_config_filepath);
    icebus_base = mb;
    for (uint i = 0; i < icebus_base.size(); i++) {
        ICEBUS_CONTROL_WRITE_update_frequency_Hz(icebus_base[i], 500);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"icebus %d motor update frequency %d", i, ICEBUS_CONTROL_READ_update_frequency_Hz(icebus_base[i]));
    }

    for(auto &bus:motor_config->icebus){
        int j = 0;
        for(auto &m:bus.second){
            if(!SetID(m->motor_id,m->bus_id)){
                RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),"something went wrong writing the bus_ids, check your roboy3.yaml file");
            }else{
              SetBaudrate(m->motor_id,2000000);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"icebus motor has baudrate %d",GetBaudrate(m->motor_id));
            }
        }
    }
}

IcebusControl::~IcebusControl() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"shutting down icebus control");
}

bool IcebusControl::SetControlMode(int motor, int mode, control_Parameters_t &params, int32_t setPoint) {
    if(!SetControlMode(motor, mode, params))
        return false;
    ICEBUS_CONTROL_WRITE_sp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id, setPoint);
    return true;
}

bool IcebusControl::SetID(int motor, int id){
    if(id==0) // must not be 0
        return false;
    ICEBUS_CONTROL_WRITE_id(icebus_base[motor_config->motor[motor]->bus],motor,id);
    if(ICEBUS_CONTROL_READ_id(icebus_base[motor_config->motor[motor]->bus],motor)==id){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"motor %d now has bus_id %d", motor, id);
        motor_config->motor[motor]->bus_id = id;
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"unable to change bus_id of motor %d from id %d -> %d",
                motor,ICEBUS_CONTROL_READ_id(icebus_base[motor_config->motor[motor]->bus],motor),id );
        return false;
    }
    return true;
}

bool IcebusControl::SetControlMode(int motor, int mode, control_Parameters_t &params) {
    if(mode>=ENCODER0_POSITION && mode<=DIRECT_PWM) {
        ICEBUS_CONTROL_WRITE_control_mode(icebus_base[motor_config->motor[motor]->bus],
                                          motor_config->motor[motor]->motor_id, mode);
        ICEBUS_CONTROL_WRITE_Kp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                params.Kp);
        ICEBUS_CONTROL_WRITE_Kd(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                params.Kd);
        ICEBUS_CONTROL_WRITE_Ki(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                params.Ki);
        ICEBUS_CONTROL_WRITE_deadband(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                      params.deadband);
        ICEBUS_CONTROL_WRITE_IntegralLimit(icebus_base[motor_config->motor[motor]->bus],
                                           motor_config->motor[motor]->motor_id, params.IntegralLimit);
        ICEBUS_CONTROL_WRITE_PWMLimit(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                      params.PWMLimit);
        if ((mode == ENCODER0_POSITION || mode == ENCODER1_POSITION) && GetCommunicationQuality(motor)!=0) {
            int32_t current_position = ICEBUS_CONTROL_READ_encoder0_position(icebus_base[myo_base_of_motor[motor]],
                                                                             motor_config->motor[motor]->motor_id);
            ICEBUS_CONTROL_WRITE_sp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                    current_position);
        } else {
            ICEBUS_CONTROL_WRITE_sp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                    0);
        }
        return true;
    }else{
        return false;
    }
}

bool IcebusControl::SetControlMode(int motor, int mode) {
    if(mode>=ENCODER0_POSITION && mode<=DIRECT_PWM) {
//        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"motor_id %d", motor_config->motor[motor]->motor_id);
        ICEBUS_CONTROL_WRITE_Kp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                control_params[motor][mode].Kp);
        ICEBUS_CONTROL_WRITE_Kd(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                control_params[motor][mode].Kd);
        ICEBUS_CONTROL_WRITE_Ki(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                control_params[motor][mode].Ki);
        ICEBUS_CONTROL_WRITE_deadband(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                      (control_params[motor][mode].deadband));
        ICEBUS_CONTROL_WRITE_IntegralLimit(icebus_base[motor_config->motor[motor]->bus],
                                           motor_config->motor[motor]->motor_id,
                                           control_params[motor][mode].IntegralLimit);
        ICEBUS_CONTROL_WRITE_PWMLimit(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                      control_params[motor][mode].PWMLimit);
        ICEBUS_CONTROL_WRITE_control_mode(icebus_base[motor_config->motor[motor]->bus],
                                          motor_config->motor[motor]->motor_id, mode);
        if ((mode == ENCODER0_POSITION || mode == ENCODER1_POSITION) && GetCommunicationQuality(motor)!=0) {
            int32_t current_position = ICEBUS_CONTROL_READ_encoder0_position(icebus_base[myo_base_of_motor[motor]],
                                                                             motor_config->motor[motor]->motor_id);
            ICEBUS_CONTROL_WRITE_sp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                    current_position);
        } else {
            ICEBUS_CONTROL_WRITE_sp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                    0);
        }
        return true;
    }else{
        return false;
    }
}

bool IcebusControl::SetControlMode(int mode) {
    if(mode>=ENCODER0_POSITION && mode<=DIRECT_PWM) {
        for (uint motor = 0; motor < numberOfMotors; motor++) {
            ICEBUS_CONTROL_WRITE_Kp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                    control_params[motor][mode].Kp);
            ICEBUS_CONTROL_WRITE_Kd(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                    control_params[motor][mode].Kd);
            ICEBUS_CONTROL_WRITE_Ki(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,
                                    control_params[motor][mode].Ki);
            ICEBUS_CONTROL_WRITE_deadband(icebus_base[motor_config->motor[motor]->bus],
                                          motor_config->motor[motor]->motor_id,
                                          (control_params[motor][mode].deadband));
            ICEBUS_CONTROL_WRITE_IntegralLimit(icebus_base[motor_config->motor[motor]->bus],
                                               motor_config->motor[motor]->motor_id,
                                               control_params[motor][mode].IntegralLimit);
            ICEBUS_CONTROL_WRITE_PWMLimit(icebus_base[motor_config->motor[motor]->bus],
                                          motor_config->motor[motor]->motor_id,
                                          control_params[motor][mode].PWMLimit);
            ICEBUS_CONTROL_WRITE_control_mode(icebus_base[motor_config->motor[motor]->bus],
                                              motor_config->motor[motor]->motor_id, mode);
            if ((mode == ENCODER0_POSITION || mode == ENCODER1_POSITION) && GetCommunicationQuality(motor)!=0) {
                int32_t current_position = ICEBUS_CONTROL_READ_encoder0_position(icebus_base[myo_base_of_motor[motor]],
                                                                                 motor_config->motor[motor]->motor_id);
                ICEBUS_CONTROL_WRITE_sp(icebus_base[motor_config->motor[motor]->bus],
                                        motor_config->motor[motor]->motor_id, current_position);
            } else {
                ICEBUS_CONTROL_WRITE_sp(icebus_base[motor_config->motor[motor]->bus],
                                        motor_config->motor[motor]->motor_id, 0);
            }
        }
        return true;
    }else{
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"control_mode %d invalid, ignoring...");
        return false;
    }
}

int32_t IcebusControl::GetBaudrate(int motor){
    return ICEBUS_CONTROL_READ_baudrate(icebus_base[motor_config->motor[motor]->bus]);
}

int32_t IcebusControl::GetCommunicationQuality(int motor){
    return ICEBUS_CONTROL_READ_communication_quality(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
}

string IcebusControl::GetErrorCode(int motor){
    switch(ICEBUS_CONTROL_READ_error_code(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id)){
      case 0: return "ok";
      case 0xDEADBEAF: return "timeout";
      case 0xBAADC0DE: return "crc";
      default: return "ok";
    }
}

void IcebusControl::GetControllerParameter(int motor, int32_t &Kp, int32_t &Ki, int32_t &Kd,
                            int32_t &deadband, int32_t &IntegralLimit, int32_t &PWMLimit){
    Kp = ICEBUS_CONTROL_READ_Kp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
    Ki = ICEBUS_CONTROL_READ_Ki(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
    Kd = ICEBUS_CONTROL_READ_Kd(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
    IntegralLimit = ICEBUS_CONTROL_READ_IntegralLimit(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
    deadband = ICEBUS_CONTROL_READ_deadband(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
    PWMLimit = ICEBUS_CONTROL_READ_PWMLimit(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
}

uint8_t IcebusControl::GetControlMode(int motor) {

    return ICEBUS_CONTROL_READ_control_mode(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
}

float IcebusControl::GetCurrent(int motor) {
    // ROS_INFO_THROTTLE(1,"%x",ICEBUS_CONTROL_READ_current(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id));
    return ICEBUS_CONTROL_READ_current(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
}

int IcebusControl::GetCurrentAverage() {
    // return ICEBUS_CONTROL_READ_current_average(icebus_base[motor_config->motor[motor]->bus]);
    return 0;
}

float IcebusControl::GetCurrentLimit(int motor) {
    int16_t current_limit = ICEBUS_CONTROL_READ_current_limit(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
    return current_limit/80.f;
}

void IcebusControl::GetDefaultControlParams(control_Parameters_t *params, int control_mode) {
    switch (control_mode) {
        case ENCODER0_POSITION:
            params->IntegralLimit = 25;
            params->Kp = 1;
            params->Ki = 1;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 500;
            break;
        case ENCODER1_POSITION:
            params->IntegralLimit = 25;
            params->Kp = 1;
            params->Ki = 1;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 500;
            break;
//        case ENCODER0_VELOCITY: //TODO: velocity control not implemented yet
//            params->IntegralLimit = 0;
//            params->Kp = 0;
//            params->Ki = 0;
//            params->Kd = 0;
//            params->deadband = 0;
//            params->PWMLimit = 0;
//            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"velocity control not available yet, disabling controller");
//            break;
//        case ENCODER1_VELOCITY: //TODO: velocity control not implemented yet
//            params->IntegralLimit = 0;
//            params->Kp = 0;
//            params->Ki = 0;
//            params->Kd = 0;
//            params->deadband = 0;
//            params->PWMLimit = 0;
//            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"velocity control not available yet, disabling controller");
//            break;
        case DIRECT_PWM:
            params->IntegralLimit = 25;
            params->Kp = 1;
            params->Ki = 1;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 500;
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"unknown control mode %d", control_mode);
            break;
    }

}

int16_t IcebusControl::GetDisplacement(int motor) {
    return ICEBUS_CONTROL_READ_displacement(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
}

int32_t IcebusControl::GetEncoderPosition(int motor, int encoder) {
    if(encoder==0)
        return ICEBUS_CONTROL_READ_encoder0_position(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
    else if(encoder==1)
        return ICEBUS_CONTROL_READ_encoder1_position(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
    else
        return -1;
}

int32_t IcebusControl::GetEncoderVelocity(int motor, int encoder) {
    return 0;
}

int32_t IcebusControl::GetNeopixelColor(int motor) {
    return ICEBUS_CONTROL_READ_neopxl_color(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
}

int32_t IcebusControl::GetSetPoint(int motor){
    return ICEBUS_CONTROL_READ_sp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
}

int32_t IcebusControl::GetPWM(int motor){
    return ICEBUS_CONTROL_READ_pwm(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id);
}

bool IcebusControl::SetCurrentLimit(int motor, float limit) {
    int16_t current_limit = limit*80;
    ICEBUS_CONTROL_WRITE_current_limit(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id,current_limit);
    return (ICEBUS_CONTROL_READ_current_limit(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id)==current_limit);
}

void IcebusControl::SetNeopixelColor(int motor, int32_t color){
    ICEBUS_CONTROL_WRITE_neopxl_color(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id, color);
}

void IcebusControl::SetPoint(int motor, int32_t setPoint) {
    ICEBUS_CONTROL_WRITE_sp(icebus_base[motor_config->motor[motor]->bus], motor_config->motor[motor]->motor_id, (int32_t) setPoint);
}

bool IcebusControl::AllToSetpoint(int control_mode, int32_t setpoint) {
    if(!SetControlMode(control_mode))
        return false;
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        SetPoint(motor, setpoint);
    }
    return true;
}

void IcebusControl::SetBaudrate(int motor, int baudrate){
  ICEBUS_CONTROL_WRITE_baudrate(icebus_base[motor_config->motor[motor]->bus],baudrate);
}

void IcebusControl::SetMotorUpdateFrequency(int motor, int32_t freq) {
    ICEBUS_CONTROL_WRITE_update_frequency_Hz(icebus_base[motor_config->motor[motor]->bus],freq);
}

void IcebusControl::ZeroWeight(int load_cell) {
    uint32_t adc_value = 0;
    weight_offset = -GetWeight(load_cell, adc_value);
}

uint32_t IcebusControl::ReadADC(int load_cell = 0) {
    // start measure
    IOWR(adc_base, 0x00, (load_cell << 1) | 0x00);
    IOWR(adc_base, 0x00, (load_cell << 1) | 0x01);
    IOWR(adc_base, 0x00, (load_cell << 1) | 0x00);
    usleep(1);
    // wait measure done
    while ((IORD(adc_base, 0x00) & 0x01) == 0x00);
    // read adc value
    uint32_t adc_value = 0;
    uint sample = 0;
    while (sample < NUMBER_OF_ADC_SAMPLES) {
        uint32_t val = IORD(adc_base, 0x01);
        if (val > 0) {
            sample++;
            adc_value += val;
//            printf("CH%d=%.3fV (0x%04x)\r\n", load_cell, (float)adc_value/1000.0, adc_value);
        } else {
            // start measure
            IOWR(adc_base, 0x00, (load_cell << 1) | 0x00);
            IOWR(adc_base, 0x00, (load_cell << 1) | 0x01);
            IOWR(adc_base, 0x00, (load_cell << 1) | 0x00);
            usleep(1);
            // wait measure done
            while ((IORD(adc_base, 0x00) & 0x01) == 0x00);
        }
    }

    adc_value /= NUMBER_OF_ADC_SAMPLES;

    return adc_value;
}

float IcebusControl::GetWeight(int load_cell) {
    uint32_t adc_value = ReadADC(load_cell);
    float weight = (adc_weight_parameters[0] + weight_offset + adc_weight_parameters[1] * adc_value) * 9.81f;
    return weight;
}

float IcebusControl::GetWeight(int load_cell, uint32_t &adc_value) {
    adc_value = ReadADC(load_cell);
    float weight = (adc_weight_parameters[0] + weight_offset + adc_weight_parameters[1] * adc_value) * 9.81f;
    return weight;
}

bool IcebusControl::MyMotor(int m){
  return motor_config->motor.find(m) != motor_config->motor.end();
}

float IcebusControl::RecordTrajectories(
        float samplingTime, float recordTime,
        map<int, vector<float>> &trajectories, vector<int> &idList,
        vector<int> &controlmode, string name) {

//    ROS_INFO_STREAM("Started recording a trajectory " + name);
    string filepath = trajectories_folder + name;
    // this will be filled with the trajectories
    AllToSetpoint(DISPLACEMENT, predisplacement);

    // samplingTime milli -> seconds
    samplingTime /= 1000.0f;

    double elapsedTime = 0.0, dt;
    long sample = 0;

    // start recording
    timer.start();
    do {
        dt = elapsedTime;
//        for (uint motor = 0; motor < idList.size(); motor++) {
        for (auto it = idList.begin(); it != idList.end(); it++) {
            if (controlmode[*it] == ENCODER0_POSITION)
                trajectories[idList[*it]].push_back(GetEncoderPosition(*it,ENCODER0_POSITION));
            else if (controlmode[*it] == ENCODER1_POSITION)
                trajectories[idList[*it]].push_back(GetEncoderPosition(*it,ENCODER1_POSITION));
        }
        sample++;
        elapsedTime = timer.elapsedTime();
        dt = elapsedTime - dt;
        // if faster than sampling time sleep for difference
        if (dt < samplingTime) {
            usleep((samplingTime - dt) * 1000000.0);
            elapsedTime = timer.elapsedTime();
        }
    } while (elapsedTime < recordTime);

    // set force to zero
    AllToSetpoint(DISPLACEMENT,0);

    // done recording
    if (filepath.empty()) {
        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char str[200];
        sprintf(str, "recording_%s.log",
                asctime(timeinfo));
        filepath = str;
    }

    std::ofstream outfile(filepath, ofstream::binary);
    stringstream ss;
    if (outfile.is_open()) {
        ss << "<?xml version=\"1.0\" ?>"
           << std::endl;
        uint m = 0;
        char motorname[10];
        for (uint m = 0; m < idList.size(); m++) {
            sprintf(motorname, "motor%d", idList[m]);
            ss << "<trajectory motorid=\"" << idList[m] << "\" controlmode=\""
               << controlmode[m] << "\" samplingTime=\"" << samplingTime * 1000.0f << "\">"
               << std::endl;
            ss << "<waypointlist>" << std::endl;
            for (uint i = 0; i < trajectories[idList[m]].size(); i++)
                ss << trajectories[idList[m]][i] << " ";
            ss << "</waypointlist>" << std::endl;
            ss << "</trajectory>" << std::endl;
        }
        ss << "</roboybehavior>" << std::endl;
        outfile << ss.rdbuf();
        outfile.close();
    }

//    ROS_INFO_STREAM("Saved trajectory " + name);

    // return average sampling time in milliseconds
    return elapsedTime / (double) sample * 1000.0f;
}

float IcebusControl::StartRecordTrajectories(
        float samplingTime, map<int, vector<float>> &trajectories,
        vector<int> &idList, string name) {
    string filepath = trajectories_folder + name;
    recording = true;
//    ROS_INFO_STREAM("Started recording a trajectory " + name);
    // this will be filled with the trajectories
    for(auto motor:idList) {
        SetControlMode(motor,DISPLACEMENT);
        SetPoint(motor,predisplacement);
    }

    // samplingTime milli -> seconds
    samplingTime /= 1000.0f;

    double elapsedTime = 0.0, dt;
    long sample = 0;
    rclcpp::Rate rate(1.0 / samplingTime);
//    ROS_INFO_STREAM(1.0/samplingTime);
    // start recording
    do {
        dt = elapsedTime;
        for (auto it:  idList)//.begin(); it != idList.end(); it++ ) {
        {
            trajectories[it].push_back(GetEncoderPosition(it,ENCODER0_POSITION));
        }
        sample++;
        rate.sleep();
    } while (recording);


    for(auto motor:idList) {
        SetControlMode(motor,DISPLACEMENT);
        SetPoint(motor,10);
    }

    // done recording

    if (filepath.empty()) {
        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char str[200];
        sprintf(str, "recording_%s.log",
                asctime(timeinfo));
        filepath = str;
    }

    std::ofstream outfile(filepath, ofstream::binary);
    stringstream ss;
    if (outfile.is_open()) {
        ss << "<?xml version=\"1.0\" ?>"
           << std::endl;
        uint m = 0;
        char motorname[10];
        ss << "<behavior>" << std::endl;
        for (uint m = 0; m < idList.size(); m++) {
            sprintf(motorname, "motor%d", idList[m]);
            ss << "<trajectory motorid=\"" << idList[m] << "\" controlmode=\""
               << ENCODER0_POSITION << "\" samplingTime=\"" << samplingTime * 1000.0f << "\">"
               << std::endl;
            ss << "<waypointlist>" << std::endl;
            for (uint i = 0; i < trajectories[idList[m]].size(); i++)
                ss << trajectories[idList[m]][i] << " ";
            ss << "</waypointlist>" << std::endl;
            ss << "</trajectory>" << std::endl;
        }
        ss << "</behavior>" << std::endl;
//        outfile.write(buffer, buffer.size());
        outfile << ss.rdbuf();
        outfile.close();
    }

//    ROS_INFO_STREAM("Saved trajectory " + name);

    // return average sampling time in milliseconds
    return elapsedTime / (double) sample * 1000.0f;
}

void IcebusControl::StopRecordTrajectories() {
    recording = false;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Stopped recording a trajectory");
}

void IcebusControl::SetReplay(bool status) {
    replay = status;
    if (replay) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Replaying trajectories enabled");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Replaying trajectories disabled");
    }
}

void IcebusControl::PrintStatus(int motor_id_global){
//        printf(
//                "control_mode  %d\n"
//                "sp            %d\n"
//               "encoder0_pos  %d\n"
//               "encoder1_pos  %d\n"
//               "encoder0_vel  %d\n"
//               "encoder1_vel  %d\n"
//               "current_phase1 %d\n"
//               "current_phase2 %d\n"
//               "current_phase3 %d\n"
//               "Kp            %d\n"
//               "Ki            %d\n"
//               "Kd            %d\n"
//               "PWMLimit      %d\n"
//               "IntegralLimit %d\n"
//               "deadband      %d\n"
//
//               "suf           %d\n"
//               "error_code    %x\n"
//               "crc           %x\n"
//               "com quality   %d\n"
//               "-------------------------------------------------\n",
//                ICEBUS_CONTROL_READ_control_mode(h2p_lw_myo_addr[0],0),
//                ICEBUS_CONTROL_READ_sp(h2p_lw_myo_addr[0],0),
//
//               ICEBUS_CONTROL_READ_encoder0_position(h2p_lw_myo_addr[0],0),
//               ICEBUS_CONTROL_READ_encoder1_position(h2p_lw_myo_addr[0],0),
//               ICEBUS_CONTROL_READ_encoder0_velocity(h2p_lw_myo_addr[0],0),
//               ICEBUS_CONTROL_READ_encoder1_velocity(h2p_lw_myo_addr[0],0),
//                ICEBUS_CONTROL_READ_current_phase1(h2p_lw_myo_addr[0],0),
//                ICEBUS_CONTROL_READ_current_phase2(h2p_lw_myo_addr[0],0),
//                ICEBUS_CONTROL_READ_current_phase3(h2p_lw_myo_addr[0],0),
//                ICEBUS_CONTROL_READ_Kp(h2p_lw_myo_addr[0],0),
//                ICEBUS_CONTROL_READ_Ki(h2p_lw_myo_addr[0],0),
//                ICEBUS_CONTROL_READ_Kd(h2p_lw_myo_addr[0],0),
//               ICEBUS_CONTROL_READ_PWMLimit(h2p_lw_myo_addr[0],0),
//               ICEBUS_CONTROL_READ_IntegralLimit(h2p_lw_myo_addr[0],0),
//               ICEBUS_CONTROL_READ_deadband(h2p_lw_myo_addr[0],0),
//               ICEBUS_CONTROL_READ_update_frequency_Hz(h2p_lw_myo_addr[0]),
//               ICEBUS_CONTROL_READ_error_code(h2p_lw_myo_addr[0],0),
//               ICEBUS_CONTROL_READ_crc_checksum(h2p_lw_myo_addr[0],0),
//                ICEBUS_CONTROL_READ_communication_quality(h2p_lw_myo_addr[0],0)
//               );
//        usleep(100000);
}

bool IcebusControl::PlayTrajectory(const char *file) {

    TiXmlDocument doc(file);
    if (!doc.LoadFile()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"could not load xml trajectory %s", file);
        return false;
    }

    TiXmlElement *root = doc.FirstChildElement("behavior");

    map<int, vector<float>> trajectories;
    int samplingTime, numberOfSamples;

    // Constructs the myoMuscles by parsing custom xml.

//    ROS_INFO_STREAM("Found trajectory " + string(file));

    TiXmlElement *trajectory_it = NULL;

    for (trajectory_it = root->FirstChildElement("trajectory"); trajectory_it;


         trajectory_it = trajectory_it->NextSiblingElement("trajectory")) {

        if (trajectory_it->QueryIntAttribute("samplingTime", &samplingTime) == TIXML_SUCCESS) {
            int motor;
            if (trajectory_it->QueryIntAttribute("motorid", &motor) != TIXML_SUCCESS) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"no motorid found");
                return false;
            }
            TiXmlElement *waypointlist_it = trajectory_it->FirstChildElement("waypointlist");
            stringstream stream(waypointlist_it->GetText());
            while (1) {
                int n;
                stream >> n;
                trajectories[motor].push_back(n);
                if (!stream) {
                    numberOfSamples = trajectories[motor].size();
                    break;
                }
            }
        } else {
            return false;
        }
    }

//    allToDisplacement(0);
//    ROS_INFO_STREAM("Replaying trajectory " + string(file));
    timer.start();
    double elapsedTime = 0.0, dt;
    int sample = 0;

    samplingTime;
    rclcpp::Rate rate(1.0 / (samplingTime / 1000.0f));
//    ROS_INFO_STREAM(1.0/(samplingTime/1000.0f));
    do {
        dt = elapsedTime;
        for (auto &motor : trajectories) {
            if (sample == 0) {
                SetControlMode(motor.first, ENCODER0_POSITION);
            }

            SetPoint(motor.first, motor.second[sample]);
        }
        sample++;
        rate.sleep();
    } while (sample < numberOfSamples && replay);

    return true;
}

void IcebusControl::SetPredisplacement(int value) {
    predisplacement = value;
//    RCLCPP_INFO("Now recording with displacement" + predisplacement);
}

void IcebusControl::EstimateSpringParameters(int motor, int degree, vector<float> &coeffs, int timeout,
                                          uint numberOfDataPoints, float displacement_min,
                                          float displacement_max, vector<double> &load, vector<double> &displacement) {
    SetPoint(motor, 0);
    SetControlMode(motor, DISPLACEMENT);
    milliseconds ms_start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()), ms_stop, t0, t1;
    ofstream outfile;
    char str[100];
    sprintf(str, "springParameters_calibration_motor%d.csv", motor);
    outfile.open(str);
    if (!outfile.is_open()) {
        cout << "could not open file " << str << " for writing, aborting!" << endl;
        return;
    }
    outfile << "displacement[ticks], load[N]" << endl;
    do {
        float f = (rand() / (float) RAND_MAX) * (displacement_max - displacement_min) + displacement_min;
        SetPoint(motor, f);
        t0 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        do {// wait a bit until force is applied
            // update control
            t1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        } while ((t1 - t0).count() < 1000);

        // note the weight
        load.push_back(GetWeight(0)); // TODO: use a different load_cell for each motor
        // note the force
        displacement.push_back(GetEncoderPosition(motor,ENCODER1_POSITION));
        outfile << displacement.back() << ", " << load.back() << endl;
        ms_stop = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        cout << "setPoint: \t" << f << "\tdisplacement:\t" << displacement.back() << "\tload:\t" <<
             load.back() << endl;
    } while ((ms_stop - ms_start).count() < timeout && load.size() < numberOfDataPoints);
    SetPoint(motor, 0);
    PolynomialRegression(degree, displacement, load, coeffs);
    outfile << "regression coefficients for polynomial of " << degree << " degree:" << endl;
    for (float coef:coeffs) {
        outfile << coef << "\t";
    }
    outfile << endl;
//	polyPar[motor] = coeffs;
    outfile.close();
}

void IcebusControl::EstimateMotorAngleLinearisationParameters(int motor, int degree, vector<float> &coeffs, int timeout,
                                                           uint numberOfDataPoints, float delta_revolution_negative,
                                                           float delta_revolution_positive, vector<double> &motor_angle,
                                                           vector<double> &motor_encoder) {
//    auto it = find(myo_bricks.begin(), myo_bricks.end(), motor);
//    if (it == myo_bricks.end()) {
//        cerr << "motor " << motor << " is not configured as a myobrick, aborting..." << endl;
//        return;
//    }
//
//    ptrdiff_t id = distance(myo_bricks.begin(), it);
//
//    changeControl(motor, POSITION);
//    setPoint(motor, 0);
//    while (abs(getEncoderPosition(motor,ENCODER0)) > 1000) {
//        cout << "waiting for motor " << motor << " to go to zero position" << endl;
//        usleep(1000000);
//    }
//    changeControl(motor, VELOCITY);
//    milliseconds ms_start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()), ms_stop;
//    ofstream outfile;
//    char str[100];
//    sprintf(str, "motorAngleLinearisation_calibration_motor%d.csv", motor);
//    outfile.open(str);
//    if (!outfile.is_open()) {
//        cout << "could not open file " << str << " for writing, aborting!" << endl;
//        return;
//    }
//    outfile << "motor_angle[ticks], motor_optical_encoder[ticks]" << endl;
//
//    int32_t initial_motor_pos = getEncoderPosition(motor,ENCODER0);
//    int32_t initial_motor_angle = abs(getEncoderPosition(motor,ENCODER1)) % 4096;
//    setPoint(motor, -30000);
//    bool go_backward = true;
//    int back_and_forth = 2;
//
//    int pos_min = initial_motor_pos + ((delta_revolution_negative / 360) * 1024 * myo_bricks_gearbox_ratio[id]);
//    int pos_max = initial_motor_pos + ((delta_revolution_positive / 360) * 1024 * myo_bricks_gearbox_ratio[id]);
//
//    cout << "position min\t" << pos_min << endl;
//    cout << "position max\t" << pos_max << endl;
//
//    int sample = 0;
//
//    do {
//        if (go_backward) {
//            if (getEncoderPosition(motor,ENCODER0) < pos_min) {
//                setPoint(motor, 30000);
//                go_backward = false;
//                cout << "going forward" << endl;
//            }
//        } else {
//            if (getEncoderPosition(motor,ENCODER0) > pos_max) {
//                setPoint(motor, -30000);
//                go_backward = true;
//                cout << "going backward" << endl;
//                back_and_forth--;
//                if(back_and_forth<=0)
//                    break;
//            }
//        }
//
//        // note the motor angle
//        motor_angle.push_back(abs(getEncoderPosition(motor,ENCODER1)));
//        // note the motor encoder
//        motor_encoder.push_back(
//                abs(getEncoderPosition(motor,ENCODER0)/ myo_bricks_gearbox_ratio[id] * myo_bricks_encoder_multiplier[id] -
//                    initial_motor_angle) % 4096);
//        outfile << motor_angle.back() << ", " << motor_encoder.back() << endl;
//        if(sample%100==0)
//            printf("sample %d motor angle %lf \t motor encoder %lf\n", sample, motor_angle.back(), motor_encoder.back());
//        ms_stop = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
//        sample++;
//        usleep(10000);
//    } while ((ms_stop - ms_start).count() < timeout && motor_angle.size() < numberOfDataPoints);
//    changeControl(motor, POSITION);
//    setPoint(motor, initial_motor_pos);
//    polynomialRegression(degree, motor_angle, motor_encoder, coeffs);
//    outfile << "regression coefficients for polynomial of " << degree << " degree:" << endl;
//    for (float coef:coeffs) {
//        outfile << coef << "\t";
//    }
//    outfile << endl;
////	polyPar[motor] = coeffs;
//    outfile.close();
}

void IcebusControl::PolynomialRegression(int degree, vector<double> &x, vector<double> &y,
                                      vector<float> &coeffs) {
    int N = x.size(), i, j, k;
    double X[2 * degree +
             1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    for (i = 0; i < 2 * degree + 1; i++) {
        X[i] = 0;
        for (j = 0; j < N; j++)
            X[i] = X[i] + pow(x[j],
                              i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
    double B[degree + 1][degree + 2], a[degree +
                                        1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    for (i = 0; i <= degree; i++)
        for (j = 0; j <= degree; j++)
            B[i][j] = X[i +
                        j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    double Y[degree +
             1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^degree*yi)
    for (i = 0; i < degree + 1; i++) {
        Y[i] = 0;
        for (j = 0; j < N; j++)
            Y[i] = Y[i] + pow(x[j], i) *
                          y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i = 0; i <= degree; i++)
        B[i][degree +
             1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    degree = degree +
             1;                //degree is made degree+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
    for (i = 0; i <
                degree; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k = i + 1; k < degree; k++)
            if (B[i][i] < B[k][i])
                for (j = 0; j <= degree; j++) {
                    double temp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = temp;
                }

    for (i = 0; i < degree - 1; i++)            //loop to perform the gauss elimination
        for (k = i + 1; k < degree; k++) {
            double t = B[k][i] / B[i][i];
            for (j = 0; j <= degree; j++)
                B[k][j] = B[k][j] - t *
                                    B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
        }
    for (i = degree - 1; i >= 0; i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        a[i] = B[i][degree];                //make the variable to be calculated equal to the rhs of the last equation
        for (j = 0; j < degree; j++)
            if (j !=
                i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i] = a[i] - B[i][j] * a[j];
        a[i] = a[i] /
               B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }
    for (i = 0; i < degree; i++)
        coeffs.push_back(a[i]);    //the values of x^0,x^1,x^2,x^3,....
}
