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
    for(int mode = POSITION;mode<=DIRECT_PWM;mode++) {
        for (int i = 0; i < 8; i++) {
            getDefaultControlParams(&control_params[i][mode],mode);
        }
    }

    SetControlMode(POSITION);

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
    if(mode>=POSITION && mode<=DIRECT_PWM) {
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
        if (mode == POSITION) {
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
    if(mode>=POSITION && mode<=DIRECT_PWM) {
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
        if (mode == POSITION) {
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
        if (mode == POSITION) {
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

bool MyoControl::MyMotor(int motor){
 return motor_config->motor.find(motor) != motor_config->motor.end();
}

bool MyoControl::setSPIactive(int motor, bool active) {
    MYO_WRITE_spi_activated(myo_base[motor_config->motor[motor]->bus], active);
}

void MyoControl::SetNeopixelColor(int motor, int32_t color){
  ROS_WARN_ONCE("you are trying to set the neopixel color of a legacy motorboard...thou shall not pass");
};

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

uint8_t MyoControl::GetControlMode(int motor){
    return MYO_READ_control(myo_base[0],motor);
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

float MyoControl::GetCurrentLimit(int motor){
  ROS_WARN_ONCE("not implemented");
  return -1;
}

bool MyoControl::SetCurrentLimit(int motor, float limit){
  ROS_WARN_ONCE("not implemented");
  return false;
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

void MyoControl::changeControlParameters(int motor, control_Parameters_legacy &params){
    ROS_ERROR("not implemented");
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

float MyoControl::GetCurrent(int motor) {
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
