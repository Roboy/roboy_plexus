#include "interfaces/myoControl.hpp"

MyoControl::MyoControl(string motor_config_filepath, vector<int32_t *> &mb, int32_t *adc_base, NeoPixelPtr neopixel)
    : adc_base(adc_base), neopixel(neopixel) {
    ROS_INFO("initializing myoControl for %d myobuses with motor config file %s", mb.size(), motor_config_filepath.c_str());
    motor_config = MotorConfigPtr(new MotorConfig);
    motor_config->readConfig(motor_config_filepath);
    myo_base = mb;
    for (uint i = 0; i < myo_base.size(); i++) {
        MYO_WRITE_update_frequency(myo_base[i], MOTOR_BOARD_COMMUNICATION_FREQUENCY);
        MYO_WRITE_spi_activated(myo_base[i], true);
        usleep(10000);
        ROS_INFO("myobus %d motor update frequency %d", i, MYO_READ_update_frequency(myo_base[i]));
    }

    SetControlMode(ENCODER0_POSITION);

    for (uint i = 0; i < myo_base.size(); i++) {
//        MYO_WRITE_update_frequency(myo_base[i], 0); // as fast as possible
        MYO_WRITE_update_frequency(myo_base[i], MOTOR_BOARD_COMMUNICATION_FREQUENCY);
        MYO_WRITE_spi_activated(myo_base[i], true);
        usleep(10000);
        ROS_INFO("bus %d motor update frequency %d", i, MYO_READ_update_frequency(myo_base[i]));
    }
    reset();

    if(adc_base!= nullptr) {
        // set measure number for ADC convert
        IOWR(adc_base, 0x01, NUMBER_OF_ADC_SAMPLES);

        // start measure
        for (uint channel = 0; channel < 8; channel++) {
            IOWR(adc_base, 0x00, (channel << 1) | 0x00);
            IOWR(adc_base, 0x00, (channel << 1) | 0x01);
            IOWR(adc_base, 0x00, (channel << 1) | 0x00);
            usleep(1);
        }
    }
    if(neopixel!=nullptr)
        neopixel->setColorAll(NeoPixelColorRGB::green);
}

MyoControl::~MyoControl() {
    ROS_INFO("shutting down myoControl");
}

bool MyoControl::SetControlMode(int motor, int mode, control_Parameters_legacy &params, int32_t setPoint) {
    if(!SetControlMode(motor, mode, params))
        return false;
    int32_t *bus = myo_base[motor_config->motor[motor]->bus];
    int bus_id = motor_config->motor[motor]->bus_id;
    motor_config->motor[motor]->control_mode = mode;
    MYO_WRITE_sp(bus, bus_id, setPoint);
    int kp, ki, kd, fg, db, sp, od;
    getPIDcontrollerParams(kp, ki, kd, fg, db, sp, od, motor);
    ROS_INFO("change control motor %d (Kp: %d, Kd %d, Ki %d, ForwardGain %d, deadband %d, "
             "OutputDivider %d, setPoint %d)", motor, kp, ki, kd, fg, db, od, sp);
    return true;
}

bool MyoControl::SetControlMode(int motor, int mode, control_Parameters_legacy &params) {
    if(mode>=ENCODER0_POSITION && mode<=DIRECT_PWM) {
        int32_t *bus = myo_base[motor_config->motor[motor]->bus];
        int bus_id = motor_config->motor[motor]->bus_id;
        motor_config->motor[motor]->control_mode = mode;
        MYO_WRITE_control(bus, bus_id, mode);
        MYO_WRITE_reset_controller(bus, bus_id);
        MYO_WRITE_Kp(bus, bus_id, params.Kp);
        MYO_WRITE_Kd(bus, bus_id, params.Kd);
        MYO_WRITE_Ki(bus, bus_id, params.Ki);
        MYO_WRITE_forwardGain(bus, bus_id,
                              params.forwardGain);
        MYO_WRITE_deadBand(bus, bus_id,
                           params.deadBand);
        MYO_WRITE_IntegralPosMax(bus, bus_id,
                                 params.IntegralPosMax);
        MYO_WRITE_IntegralNegMax(bus, bus_id,
                                 params.IntegralNegMax);
        MYO_WRITE_outputPosMax(bus, bus_id,
                               params.outputPosMax);
        MYO_WRITE_outputNegMax(bus, bus_id,
                               params.outputNegMax);
        MYO_WRITE_outputDivider(bus, bus_id,
                                params.outputDivider);
        if (mode == ENCODER0_POSITION) {
            int32_t current_position = MYO_READ_position(bus, bus_id);
            MYO_WRITE_sp(bus, bus_id,
                         current_position);
        } else {
            MYO_WRITE_sp(bus, bus_id, 0);
        }
        return true;
    }else{
        return false;
    }
}

bool MyoControl::SetControlMode(int motor, int mode) {
    if(mode>=ENCODER0_POSITION && mode<=DIRECT_PWM) {
        int32_t *bus = myo_base[motor_config->motor[motor]->bus];
        int bus_id = motor_config->motor[motor]->bus_id;
        motor_config->motor[motor]->control_mode = mode;
        MYO_WRITE_reset_controller(bus, bus_id);
        MYO_WRITE_Kp(bus, bus_id,
                     control_params[motor][mode].Kp);
        MYO_WRITE_Kd(bus, bus_id,
                     control_params[motor][mode].Kd);
        MYO_WRITE_Ki(bus, bus_id,
                     control_params[motor][mode].Ki);
        MYO_WRITE_forwardGain(bus, bus_id,
                              control_params[motor][mode].forwardGain);
        MYO_WRITE_deadBand(bus, bus_id,
                           (control_params[motor][mode].deadBand));
        MYO_WRITE_IntegralPosMax(bus, bus_id,
                                 control_params[motor][mode].IntegralPosMax);
        MYO_WRITE_IntegralNegMax(bus, bus_id,
                                 control_params[motor][mode].IntegralNegMax);
        MYO_WRITE_outputPosMax(bus, bus_id,
                               control_params[motor][mode].outputPosMax);
        MYO_WRITE_outputNegMax(bus, bus_id,
                               control_params[motor][mode].outputNegMax);
        MYO_WRITE_control(bus, bus_id, mode);
        MYO_WRITE_outputDivider(bus, bus_id,
                                control_params[motor][mode].outputDivider);
        if (mode == ENCODER0_POSITION) {
            int32_t current_position = MYO_READ_position(bus, bus_id);
            MYO_WRITE_sp(bus, bus_id,
                         current_position);
        } else {
            MYO_WRITE_sp(bus, bus_id, 0);
        }
    }else{
        return false;
    }
}

bool MyoControl::SetControlMode(int mode) {
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        int32_t *bus = myo_base[motor_config->motor[motor]->bus];
        int bus_id = motor_config->motor[motor]->bus_id;
        motor_config->motor[motor]->control_mode = mode;
        MYO_WRITE_reset_controller(bus, bus_id);
        MYO_WRITE_Kp(bus, bus_id, control_params[motor][mode].Kp);
        MYO_WRITE_Kd(bus, bus_id, control_params[motor][mode].Kd);
        MYO_WRITE_Ki(bus, bus_id, control_params[motor][mode].Ki);
        MYO_WRITE_forwardGain(bus, bus_id,
                              control_params[motor][mode].forwardGain);
        MYO_WRITE_deadBand(bus, bus_id,
                           (control_params[motor][mode].deadBand));
        MYO_WRITE_IntegralPosMax(bus, bus_id,
                                 control_params[motor][mode].IntegralPosMax);
        MYO_WRITE_IntegralNegMax(bus, bus_id,
                                 control_params[motor][mode].IntegralNegMax);
        MYO_WRITE_outputPosMax(bus, bus_id,
                               control_params[motor][mode].outputPosMax);
        MYO_WRITE_outputNegMax(bus, bus_id,
                               control_params[motor][mode].outputNegMax);
        MYO_WRITE_control(bus, bus_id, mode);
        MYO_WRITE_outputDivider(bus, bus_id,
                                control_params[motor][mode].outputDivider);
        if (mode == ENCODER0_POSITION) {
            int32_t current_position = MYO_READ_position(bus, bus_id);
            MYO_WRITE_sp(bus, bus_id, current_position);
        } else {
            MYO_WRITE_sp(bus, bus_id, 0);
        }
    }
}

void MyoControl::reset() {
    for (uint i = 0; i < myo_base.size(); i++) {
        MYO_WRITE_reset_myo_control(myo_base[i], true);
        MYO_WRITE_reset_myo_control(myo_base[i], false);
    }
}

bool MyoControl::setSPIactive(int motor, bool active) {
    MYO_WRITE_spi_activated(myo_base[motor_config->motor[motor]->bus], active);
}

void MyoControl::getPIDcontrollerParams(int &Pgain, int &Igain, int &Dgain, int &forwardGain, int &deadband,
                                        int &setPoint, int &outputDivider, int motor) {
    int32_t *bus = myo_base[motor_config->motor[motor]->bus];
    int bus_id = motor_config->motor[motor]->bus_id;
    Pgain = MYO_READ_Kp(bus, bus_id);
    Igain = MYO_READ_Ki(bus, bus_id);
    Dgain = MYO_READ_Kd(bus, bus_id);
    forwardGain = MYO_READ_forwardGain(bus, bus_id);
    deadband = MYO_READ_deadBand(bus, bus_id);
    setPoint = MYO_READ_sp(bus, bus_id);
    outputDivider = MYO_READ_outputDivider(bus, bus_id);
}

void MyoControl::setPIDcontrollerParams(uint16_t Pgain, uint16_t Igain, uint16_t Dgain, uint16_t forwardGain,
                                        uint16_t deadband, int motor, int mode) {
    control_params[motor][mode].Kp = Pgain;
    control_params[motor][mode].Ki = Igain;
    control_params[motor][mode].Kd = Dgain;
    control_params[motor][mode].forwardGain = forwardGain;
    control_params[motor][mode].deadBand = deadband;
}

bool MyoControl::GetPowerSense() {
    return (bool) MYO_READ_power_sense(myo_base[0]);
}

int32_t MyoControl::GetPWM(int motor) {
    int32_t *bus = myo_base[motor_config->motor[motor]->bus];
    int bus_id = motor_config->motor[motor]->bus_id;
    return MYO_READ_pwmRef(bus, bus_id);
}

int32_t MyoControl::GetEncoderPosition(int motor, int encoder) {
    int32_t *bus = myo_base[motor_config->motor[motor]->bus];
    int bus_id = motor_config->motor[motor]->bus_id;
    return MYO_READ_position(bus, bus_id);
}

int32_t MyoControl::GetEncoderVelocity(int motor, int encoder) {
    int32_t *bus = myo_base[motor_config->motor[motor]->bus];
    int bus_id = motor_config->motor[motor]->bus_id;
    int16_t vel = MYO_READ_velocity(bus, bus_id);
    return ((int32_t) vel) * MOTOR_BOARD_COMMUNICATION_FREQUENCY;
}

int32_t MyoControl::GetDisplacement(int motor) {
    int32_t *bus = myo_base[motor_config->motor[motor]->bus];
    int bus_id = motor_config->motor[motor]->bus_id;
    return (int32_t)MYO_READ_displacement(bus, bus_id);
}

void MyoControl::SetPoint(int motor, int32_t setPoint) {
    int32_t *bus = myo_base[motor_config->motor[motor]->bus];
    int bus_id = motor_config->motor[motor]->bus_id;
    MYO_WRITE_sp(bus, bus_id, (int32_t) setPoint);
}

bool MyoControl::configureMyoBricks(vector<int32_t> &motorIDs,
                                    vector<int32_t> &encoderMultiplier,
                                    vector<int32_t> &gearBoxRatio) {

    myo_bricks = motorIDs;
    myo_bricks_gearbox_ratio = gearBoxRatio;
    myo_bricks_encoder_multiplier = encoderMultiplier;
    uint32_t myo_brick = 0;
    uint i = 0;
    stringstream str;
    str << "configuring myoBricks\nmotor ID | gear box ratio | encoder multiplier" << endl;
    for (auto motor_id:motorIDs) {
        int32_t *bus = myo_base[motor_config->motor[motor_id]->bus];
        int bus_id = motor_config->motor[motor_id]->bus_id;
        ROS_INFO("myoBrick %d of myoControl %d", bus_id, motor_config->motor[motor_id]->bus);
        myo_brick |= (1 << bus_id);
        MYO_WRITE_myo_brick_gear_box_ratio(bus,bus_id, gearBoxRatio[i]);
        MYO_WRITE_myo_brick_encoder_multiplier(bus,bus_id, encoderMultiplier[i]);
        int ratio = MYO_READ_myo_brick_gear_box_ratio(bus,bus_id);
        int multiplier = MYO_READ_myo_brick_encoder_multiplier(bus,bus_id);
        if (ratio != gearBoxRatio[i] ||
            multiplier != encoderMultiplier[i]) { // if the value was not written correctly, we abort!
            ROS_FATAL("id %d, ratio %d, multiplier %d", motor_id, ratio, multiplier);
            return false;
        }
        str << (int) motor_id << "\t\t| " << ratio << "\t\t| " << multiplier << endl;
        MYO_WRITE_myo_brick(bus, myo_brick);
        i++;
    }
    ROS_INFO_STREAM(str.str());
    return true;
}

int16_t MyoControl::GetCurrent(int motor) {
    int32_t *bus = myo_base[motor_config->motor[motor]->bus];
    int bus_id = motor_config->motor[motor]->bus_id;
    return MYO_READ_current(bus, bus_id);
}

void MyoControl::getDefaultControlParams(control_Parameters_legacy *params, int control_mode) {
    params->outputPosMax = 500;
    params->outputNegMax = -500;

    params->radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);

    switch (control_mode) {
        case 0:
//            params->outputPosMax = 4000;
//            params->outputNegMax = -4000;
            params->spPosMax = 10000000;
            params->spNegMax = -10000000;
            params->Kp = 1;
            params->Ki = 0;
            params->Kd = 0;
            params->forwardGain = 0;
            params->deadBand = 0;
            params->IntegralPosMax = 100;
            params->IntegralNegMax = -100;
            params->outputDivider = 5;
            break;
        case 1:
            params->spPosMax = 100000;
            params->spNegMax = -100000;
            params->Kp = 30;
            params->Ki = 0;
            params->Kd = 0;
            params->forwardGain = 0;
            params->deadBand = 0;
            params->IntegralPosMax = 100;
            params->IntegralNegMax = -100;
            params->outputDivider = 0;
            break;
        case 2:
            params->spPosMax = 100000;
            params->spNegMax = 0;
            params->Kp = 100;
            params->Ki = 0;
            params->Kd = 0;
            params->forwardGain = 0;
            params->deadBand = 1;
            params->IntegralPosMax = 100;
            params->IntegralNegMax = 0;
            params->outputDivider = 0;
            break;
        case 3:
            params->spPosMax = 256;
            params->spNegMax = -256;
            params->Kp = 1;
            params->Ki = 0;
            params->Kd = 0;
            params->forwardGain = 0;
            params->deadBand = 0;
            params->IntegralPosMax = 100;
            params->IntegralNegMax = 0;
            params->outputDivider = 0;
            break;
        default:
            ROS_ERROR("unknown control mode %d", control_mode);
            break;
    }

}

float MyoControl::recordTrajectories(
        float samplingTime, float recordTime,
        map<int, vector<float>> &trajectories, vector<int> &idList,
        vector<int> &controlmode, string name) {

    ROS_INFO_STREAM("Started recording a trajectory " + name);
    string filepath = trajectories_folder + name;
    // this will be filled with the trajectories
    allToDisplacement(predisplacement);

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
                trajectories[idList[*it]].push_back(getPosition(*it));
            else if (controlmode[*it] == ENCODER0_VELOCITY)
                trajectories[idList[*it]].push_back(getVelocity(*it));
            else if (controlmode[*it] == FORCE)
                trajectories[idList[*it]].push_back(getDisplacement(*it));
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
    allToDisplacement(0);

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

    ROS_INFO_STREAM("Saved trajectory " + name);

    // return average sampling time in milliseconds
    return elapsedTime / (double) sample * 1000.0f;
}

float MyoControl::startRecordTrajectories(
        float samplingTime, map<int, vector<float>> &trajectories,
        vector<int> &idList, string name) {
    string filepath = trajectories_folder + name;
    recording = true;
    ROS_INFO_STREAM("Started recording a trajectory " + name);
    // this will be filled with the trajectories
    for(auto motor:idList) {
        SetControlMode(motor,DISPLACEMENT);
        setDisplacement(motor,predisplacement);
    }

    // samplingTime milli -> seconds
    samplingTime /= 1000.0f;

    double elapsedTime = 0.0, dt;
    long sample = 0;
    ros::Rate rate(1.0 / samplingTime);
//    ROS_INFO_STREAM(1.0/samplingTime);
    // start recording
    do {
        dt = elapsedTime;
        for (auto it:  idList)//.begin(); it != idList.end(); it++ ) {
        {
            trajectories[it].push_back(getPosition(it));
        }
        sample++;
        rate.sleep();
    } while (recording);


    for(auto motor:idList) {
        SetControlMode(motor,DISPLACEMENT);
        setDisplacement(motor,10);
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

    ROS_INFO_STREAM("Saved trajectory " + name);

    // return average sampling time in milliseconds
    return elapsedTime / (double) sample * 1000.0f;
}

void MyoControl::stopRecordTrajectories() {
    recording = false;
    ROS_INFO("Stopped recording a trajectory");
}

void MyoControl::setReplay(bool status) {
    replay = status;
    if (replay) {
        ROS_INFO("Replaying trajectories enabled");
    } else {
        ROS_INFO("Replaying trajectories disabled");
    }
}

bool MyoControl::playTrajectory(const char *file) {

    TiXmlDocument doc(file);
    if (!doc.LoadFile()) {
        ROS_ERROR("could not load xml trajectory %s", file);
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
                ROS_ERROR("no motorid found");
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
    ROS_INFO_STREAM("Replaying trajectory " + string(file));
    timer.start();
    double elapsedTime = 0.0, dt;
    int sample = 0;

    samplingTime;
    ros::Rate rate(1.0 / (samplingTime / 1000.0f));
//    ROS_INFO_STREAM(1.0/(samplingTime/1000.0f));
    do {
        dt = elapsedTime;
        for (auto &motor : trajectories) {
            if (sample == 0) {
                SetControlMode(motor.first, ENCODER0_POSITION);
            }

            setPosition(motor.first, motor.second[sample]);
        }
        sample++;
        rate.sleep();
    } while (sample < numberOfSamples && replay);

    return true;
}

void MyoControl::setPredisplacement(int value) {
    predisplacement = value;
    ROS_INFO_STREAM("Now recording with displacement" + predisplacement);
}

void MyoControl::estimateSpringParameters(int motor, int degree, vector<float> &coeffs, int timeout,
                                          uint numberOfDataPoints, float displacement_min,
                                          float displacement_max, vector<double> &load, vector<double> &displacement) {
    setDisplacement(motor, 0);
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
        setDisplacement(motor, f);
        t0 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        do {// wait a bit until force is applied
            // update control
            t1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        } while ((t1 - t0).count() < 1000);

        // note the weight
        load.push_back(getWeight(0)); // TODO: use a different load_cell for each motor
        // note the force
        displacement.push_back(getDisplacement(motor));
        outfile << displacement.back() << ", " << load.back() << endl;
        ms_stop = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        cout << "setPoint: \t" << f << "\tdisplacement:\t" << displacement.back() << "\tload:\t" <<
             load.back() << endl;
    } while ((ms_stop - ms_start).count() < timeout && load.size() < numberOfDataPoints);
    setDisplacement(motor, 0);
    polynomialRegression(degree, displacement, load, coeffs);
    outfile << "regression coefficients for polynomial of " << degree << " degree:" << endl;
    for (float coef:coeffs) {
        outfile << coef << "\t";
    }
    outfile << endl;
//	polyPar[motor] = coeffs;
    outfile.close();
}

void MyoControl::polynomialRegression(int degree, vector<double> &x, vector<double> &y,
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

void MyoControl::gpioControl(bool power) {
    MYO_WRITE_gpio(myo_base[0], power);
}