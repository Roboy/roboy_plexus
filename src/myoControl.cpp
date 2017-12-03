#include "roboy_plexus/myoControl.hpp"

MyoControl::MyoControl(vector<int32_t *> &myo_base) : myo_base(myo_base) {
    // initialize control mode
    numberOfMotors = myo_base.size() * MOTORS_PER_MYOCONTROL;
    // initialize all controllers with default values
    control_Parameters_t params;
    getDefaultControlParams(&params, POSITION);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        control_params[motor][POSITION] = params;
    }
    getDefaultControlParams(&params, VELOCITY);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        control_params[motor][VELOCITY] = params;
    }
    getDefaultControlParams(&params, DISPLACEMENT);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        control_params[motor][DISPLACEMENT] = params;
    }

    changeControl(DISPLACEMENT);

    for (uint i = 0; i < myo_base.size(); i++) {
        MYO_WRITE_spi_activated(myo_base[i], true);
    }
    reset();
}

MyoControl::MyoControl(vector<int32_t *> &myo_base, int32_t *adc_base) : myo_base(myo_base),
                                                                          adc_base(adc_base) {
    // initialize control mode
    numberOfMotors = myo_base.size() * MOTORS_PER_MYOCONTROL;
    // initialize all controllers with default values
    control_Parameters_t params;
    getDefaultControlParams(&params, POSITION);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        control_params[motor][POSITION] = params;
    }
    getDefaultControlParams(&params, VELOCITY);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        control_params[motor][VELOCITY] = params;
    }
    getDefaultControlParams(&params, DISPLACEMENT);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        control_params[motor][DISPLACEMENT] = params;
    }

    changeControl(DISPLACEMENT);

    for (uint i = 0; i < myo_base.size(); i++) {
        MYO_WRITE_spi_activated(myo_base[i], true);
    }
    reset();

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

MyoControl::~MyoControl() {
    cout << "shutting down myoControl" << endl;
}

void MyoControl::changeControl(int motor, int mode, control_Parameters_t &params, int32_t setPoint) {
    changeControl(motor, mode, params);
    MYO_WRITE_sp(myo_base[motor / MOTORS_PER_MYOCONTROL],
                 motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0), setPoint);
}

void MyoControl::changeControl(int motor, int mode, control_Parameters_t &params) {
    int motorOffset = motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0;
    MYO_WRITE_control(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, mode);
    MYO_WRITE_reset_controller(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset);
    MYO_WRITE_Kp(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, params.Kp);
    MYO_WRITE_Kd(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, params.Kd);
    MYO_WRITE_Ki(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, params.Ki);
    MYO_WRITE_forwardGain(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, params.forwardGain);
    MYO_WRITE_deadBand(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, params.deadBand);
    MYO_WRITE_IntegralPosMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, params.IntegralPosMax);
    MYO_WRITE_IntegralNegMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, params.IntegralNegMax);
    MYO_WRITE_outputPosMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, params.outputPosMax);
    MYO_WRITE_outputNegMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, params.outputNegMax);
}

void MyoControl::changeControl(int motor, int mode) {
    int motorOffset = motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0;
    MYO_WRITE_control(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, mode);
    MYO_WRITE_reset_controller(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset);
    MYO_WRITE_Kp(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, control_params[motor][mode].Kp);
    MYO_WRITE_Kd(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, control_params[motor][mode].Kd);
    MYO_WRITE_Ki(myo_base[motor / MOTORS_PER_MYOCONTROL], motor, control_params[motor][mode].Ki);
    MYO_WRITE_forwardGain(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                          control_params[motor][mode].forwardGain);
    MYO_WRITE_deadBand(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                       (control_params[motor][mode].deadBand));
    MYO_WRITE_IntegralPosMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                             control_params[motor][mode].IntegralPosMax);
    MYO_WRITE_IntegralNegMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                             control_params[motor][mode].IntegralNegMax);
    MYO_WRITE_outputPosMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                           control_params[motor][mode].outputPosMax);
    MYO_WRITE_outputNegMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                           control_params[motor][mode].outputNegMax);
    MYO_WRITE_control(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, mode);
}

void MyoControl::changeControl(int mode) {
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        int motorOffset = motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0;
        MYO_WRITE_control(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, mode);
        MYO_WRITE_reset_controller(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset);
        MYO_WRITE_Kp(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, control_params[motor][mode].Kp);
        MYO_WRITE_Kd(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, control_params[motor][mode].Kd);
        MYO_WRITE_Ki(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, control_params[motor][mode].Ki);
        MYO_WRITE_forwardGain(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                              control_params[motor][mode].forwardGain);
        MYO_WRITE_deadBand(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                           (control_params[motor][mode].deadBand));
        MYO_WRITE_IntegralPosMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                                 control_params[motor][mode].IntegralPosMax);
        MYO_WRITE_IntegralNegMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                                 control_params[motor][mode].IntegralNegMax);
        MYO_WRITE_outputPosMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                               control_params[motor][mode].outputPosMax);
        MYO_WRITE_outputNegMax(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset,
                               control_params[motor][mode].outputNegMax);
        MYO_WRITE_control(myo_base[motor / MOTORS_PER_MYOCONTROL], motor - motorOffset, mode);
    }
}

bool MyoControl::toggleSPI() {
    bool spi_active;
    for (uint i = 0; i < myo_base.size(); i++) {
        spi_active = MYO_READ_spi_activated(myo_base[i]);
        MYO_WRITE_spi_activated(myo_base[i], !spi_active);
    }

    return !spi_active;
}

void MyoControl::reset() {
    for (uint i = 0; i < myo_base.size(); i++) {
        MYO_WRITE_reset_myo_control(myo_base[i], true);
        MYO_WRITE_reset_myo_control(myo_base[i], false);
    }
}

void MyoControl::changeSetpoint(int motor, int32_t position) {
    MYO_WRITE_sp(myo_base[motor / MOTORS_PER_MYOCONTROL],
                 motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0), position);
}

bool MyoControl::setSPIactive(int motor, bool active) {
    MYO_WRITE_spi_activated(myo_base[motor / MOTORS_PER_MYOCONTROL], active);
}

void MyoControl::getPIDcontrollerParams(int &Pgain, int &Igain, int &Dgain, int &forwardGain, int &deadband,
                                        int &setPoint, int &setPointMin, int &setPointMax, int motor) {
    Pgain = MYO_READ_Kp(myo_base[motor / MOTORS_PER_MYOCONTROL],
                        motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
    Igain = MYO_READ_Ki(myo_base[motor / MOTORS_PER_MYOCONTROL],
                        motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
    Dgain = MYO_READ_Kd(myo_base[motor / MOTORS_PER_MYOCONTROL],
                        motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
    forwardGain = MYO_READ_forwardGain(myo_base[motor / MOTORS_PER_MYOCONTROL],
                                       motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
    deadband = MYO_READ_deadBand(myo_base[motor / MOTORS_PER_MYOCONTROL],
                                 motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
    setPoint = MYO_READ_sp(myo_base[motor / MOTORS_PER_MYOCONTROL],
                           motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
    setPointMin = 0;
    setPointMax = 0;
}

void MyoControl::setPIDcontrollerParams(uint16_t Pgain, uint16_t Igain, uint16_t Dgain, uint16_t forwardGain,
                                        uint16_t deadband, int motor, int mode) {
    control_params[motor][mode].Kp = Pgain;
    control_params[motor][mode].Ki = Igain;
    control_params[motor][mode].Kd = Dgain;
    control_params[motor][mode].forwardGain = forwardGain;
    control_params[motor][mode].deadBand = deadband;
}

uint16_t MyoControl::getControlMode(int motor) {
    return MYO_READ_control(myo_base[motor / MOTORS_PER_MYOCONTROL],
                            motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
}

int16_t MyoControl::getPWM(int motor) {
    return MYO_READ_pwmRef(myo_base[motor / MOTORS_PER_MYOCONTROL],
                           motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
}

int32_t MyoControl::getPosition(int motor) {
    return MYO_READ_position(myo_base[motor / MOTORS_PER_MYOCONTROL],
                             motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
}

int32_t MyoControl::getVelocity(int motor) {
    int16_t vel = MYO_READ_velocity(myo_base[motor / MOTORS_PER_MYOCONTROL],
                                    motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
    return  ((int32_t) vel) * MOTOR_BOARD_COMMUNICATION_FREQUENCY;
}

int16_t MyoControl::getDisplacement(int motor) {
    return MYO_READ_displacement(myo_base[motor / MOTORS_PER_MYOCONTROL],
                                 motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
}

int16_t MyoControl::getCurrent(int motor) {
    return MYO_READ_current(myo_base[motor / MOTORS_PER_MYOCONTROL],
                            motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0));
}

bool MyoControl::getSPIactive(int motor) {
    return MYO_READ_spi_activated(myo_base[motor / MOTORS_PER_MYOCONTROL]);
}

void MyoControl::getDefaultControlParams(control_Parameters_t *params, int control_mode) {
    params->outputPosMax = 1000;
    params->outputNegMax = -1000;

    params->radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);

    switch (control_mode) {
        case POSITION:
            params->spPosMax = 10000000;
            params->spNegMax = -10000000;
            params->Kp = 1;
            params->Ki = 0;
            params->Kd = 0;
            params->forwardGain = 0;
            params->deadBand = 0;
            params->IntegralPosMax = 100;
            params->IntegralNegMax = -100;
            break;
        case VELOCITY:
            params->spPosMax = 100;
            params->spNegMax = -100;
            params->Kp = 1;
            params->Ki = 0;
            params->Kd = 0;
            params->forwardGain = 0;
            params->deadBand = 0;
            params->IntegralPosMax = 100;
            params->IntegralNegMax = -100;
            break;
        case DISPLACEMENT:
            params->spPosMax = 2000;
            params->spNegMax = 0;
            params->Kp = 100;
            params->Ki = 0;
            params->Kd = 0;
            params->forwardGain = 0;
            params->deadBand = 0;
            params->IntegralPosMax = 100;
            params->IntegralNegMax = 0;
            break;
        default:
            cout << "unknown control mode" << endl;
            break;
    }

}

void MyoControl::allToPosition(int32_t pos) {
    changeControl(POSITION);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        MYO_WRITE_sp(myo_base[motor / MOTORS_PER_MYOCONTROL],
                     motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0), pos);
    }
}

void MyoControl::allToVelocity(int16_t vel) {
    changeControl(VELOCITY);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        MYO_WRITE_sp(myo_base[motor / MOTORS_PER_MYOCONTROL],
                     motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0), vel);
    }
}

void MyoControl::allToDisplacement(int16_t displacement) {
    changeControl(DISPLACEMENT);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        MYO_WRITE_sp(myo_base[motor / MOTORS_PER_MYOCONTROL],
                     motor - (motor >= MOTORS_PER_MYOCONTROL ? MOTORS_PER_MYOCONTROL : 0), displacement);
    }
}

void MyoControl::zeroWeight(int load_cell) {
    uint32_t adc_value = 0;
    weight_offset = -getWeight(load_cell, adc_value);
}

uint32_t MyoControl::readADC(int load_cell = 0) {
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
    while(sample<NUMBER_OF_ADC_SAMPLES){
        uint32_t val = IORD(adc_base, 0x01);
        if(val>0){
            sample ++;
            adc_value += val;
//            printf("CH%d=%.3fV (0x%04x)\r\n", load_cell, (float)adc_value/1000.0, adc_value);
        }else{
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

float MyoControl::getWeight(int load_cell) {
    uint32_t adc_value = readADC(load_cell);
    float weight = (adc_weight_parameters[0] + weight_offset + adc_weight_parameters[1] * adc_value)*9.81f;
    return weight;
}

float MyoControl::getWeight(int load_cell, uint32_t &adc_value) {
    adc_value = readADC(load_cell);
    float weight = (adc_weight_parameters[0] + weight_offset + adc_weight_parameters[1] * adc_value)*9.81f;
    return weight;
}

float MyoControl::recordTrajectories(
        float samplingTime, float recordTime,
        map<int, vector<float>> &trajectories, vector<int> &idList,
        vector<int> &controlmode, string name) {
    // this will be filled with the trajectories
    allToDisplacement(200);

    // samplingTime milli -> seconds
    samplingTime /= 1000.0f;

    double elapsedTime = 0.0, dt;
    long sample = 0;

    // start recording
    timer.start();
    do {
        dt = elapsedTime;
        for (uint motor = 0; motor < idList.size(); motor++) {
            if (controlmode[motor] == POSITION)
                trajectories[idList[motor]].push_back(getPosition(motor));
            else if (controlmode[motor] == VELOCITY)
                trajectories[idList[motor]].push_back(getVelocity(motor));
            else if (controlmode[motor] == FORCE)
                trajectories[idList[motor]].push_back(getDisplacement(motor));
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
                    << controlmode[m] << "\" samplingTime=\"" << samplingTime * 1000.0f << "\">"
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

    // return average sampling time in milliseconds
    return elapsedTime / (double) sample * 1000.0f;
}

bool MyoControl::playTrajectory(const char *file) {
    // initialize TiXmlDocument doc with a string
    TiXmlDocument doc(file);
    if (!doc.LoadFile()) {
        return false;
    }

    TiXmlElement *root = doc.RootElement();

    map<int, vector<float>> trajectories;
    int samplingTime, numberOfSamples;

    // Constructs the myoMuscles by parsing custom xml.
    TiXmlElement *trajectory_it = NULL;
    for (trajectory_it = root->FirstChildElement("trajectory"); trajectory_it;
         trajectory_it = trajectory_it->NextSiblingElement("trajectory")) {
        if (trajectory_it->Attribute("motorid") && trajectory_it->QueryIntAttribute("samplingTime", &samplingTime)) {
            int motor;
            if (trajectory_it->QueryIntAttribute("motorid", &motor) != TIXML_SUCCESS) {
                return false;
            }
            TiXmlElement *waypointlist_it = trajectory_it->FirstChildElement("waypointlist");
            stringstream stream(waypointlist_it->GetText());
            while (1) {
                int n;
                stream >> n;
                trajectories[motor].push_back(n);
                if (!stream) {
                    numberOfSamples = trajectories.size();
                    break;
                }
            }
        }
    }
    allToDisplacement(0);
    timer.start();
    double elapsedTime = 0.0, dt;
    int sample = 0;
    samplingTime /= 1000.0f;
    do {
        dt = elapsedTime;
        for (auto &motor : trajectories) {
            changeSetpoint(motor.first, motor.second[sample]);
        }
        sample++;

        elapsedTime = timer.elapsedTime();
        dt = elapsedTime - dt;
        // if faster than sampling time sleep for difference
        if (dt < samplingTime) {
            usleep((samplingTime - dt) * 1000000.0);
            elapsedTime = timer.elapsedTime();
        }
    } while (timer.elapsedTime() < (numberOfSamples * samplingTime / 1000.0f));

    return true;
}

void MyoControl::estimateSpringParameters(int motor, int degree, vector<float> &coeffs, int timeout,
                                          uint numberOfDataPoints, float displacement_min,
                                          float displacement_max, vector<double> &load, vector<double> &displacement) {
    changeSetpoint(motor, 0);
    changeControl(motor, DISPLACEMENT);
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
        changeSetpoint(motor, f);
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
    changeSetpoint(motor, 0);
    polynomialRegression(degree, displacement, load, coeffs);
    outfile << "regression coefficients for polynomial of "<< degree << " degree:" << endl;
    for(float coef:coeffs){
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
