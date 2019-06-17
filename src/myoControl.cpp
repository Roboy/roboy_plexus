#include "roboy_plexus/myoControl.hpp"

MyoControl::MyoControl(vector<int32_t *> &myo_base) : myo_base(myo_base) {
    if (myo_base.size() > 3)
        ROS_FATAL("a maximum of THREE myoControls is currently supported");
    // initialize control mode
    numberOfMotors = (myo_base.size() == 0 ? NUMBER_OF_MOTORS_MYOCONTROL_0 :
                      (myo_base.size() == 1 ? NUMBER_OF_MOTORS_MYOCONTROL_0 + NUMBER_OF_MOTORS_MYOCONTROL_1 :
                       NUMBER_OF_MOTORS_MYOCONTROL_0 + NUMBER_OF_MOTORS_MYOCONTROL_1 + NUMBER_OF_MOTORS_MYOCONTROL_2));
    ROS_INFO("initializing myoControl for %d spi buses with %d motors in total", myo_base.size(), numberOfMotors);
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
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        if (motor < NUMBER_OF_MOTORS_MYOCONTROL_0) {
            myo_base_of_motor[motor] = 0;
            motor_offset[motor] = 0;
        } else if (motor < NUMBER_OF_MOTORS_MYOCONTROL_0 + NUMBER_OF_MOTORS_MYOCONTROL_1) {
            myo_base_of_motor[motor] = 1;
            motor_offset[motor] = NUMBER_OF_MOTORS_MYOCONTROL_0;
        } else {
            myo_base_of_motor[motor] = 2;
            motor_offset[motor] = NUMBER_OF_MOTORS_MYOCONTROL_0 + NUMBER_OF_MOTORS_MYOCONTROL_1;
        }
    }

    changeControl(VELOCITY);

    for (uint i = 0; i < myo_base.size(); i++) {
//        MYO_WRITE_update_frequency(myo_base[i], 0); // as fast as possible
        MYO_WRITE_update_frequency(myo_base[i], MOTOR_BOARD_COMMUNICATION_FREQUENCY);
        MYO_WRITE_spi_activated(myo_base[i], true);
        usleep(10000);
        ROS_INFO("motor update frequency %d", MYO_READ_update_frequency(myo_base[i]));
//        for(uint motor=0; motor<7; motor++){
//            printf(        "Kp             %d\n"
//                           "Ki             %d\n"
//                           "Kd             %d\n"
//                           "sp             %d\n"
//                           "forwardGain    %d\n"
//                           "outputPosMax   %d\n"
//                           "outputNegMax   %d\n"
//                           "IntegralPosMax %d\n"
//                           "IntegralNegMax %d\n"
//                           "deadBand       %d\n"
//                           "control        %d\n"
//                           "position       %d\n"
//                           "velocity       %d\n"
//                           "current        %d\n"
//                           "displacement   %d\n"
//                           "pwmRef         %d\n"
//                           "update_frequency %d\n",
//            MYO_READ_Kp(myo_base[i],motor),
//            MYO_READ_Ki(myo_base[i],motor),
//            MYO_READ_Kd(myo_base[i],motor),
//            MYO_READ_sp(myo_base[i],motor),
//            MYO_READ_forwardGain(myo_base[i],motor),
//                           (int16_t)MYO_READ_outputPosMax(myo_base[i],motor),
//                           (int16_t)MYO_READ_outputNegMax(myo_base[i],motor),
//            MYO_READ_IntegralPosMax(myo_base[i],motor),
//            MYO_READ_IntegralNegMax(myo_base[i],motor),
//            MYO_READ_deadBand(myo_base[i],motor),
//            MYO_READ_control(myo_base[i],motor),
//            MYO_READ_position(myo_base[i],motor),
//            MYO_READ_velocity(myo_base[i],motor),
//            MYO_READ_current(myo_base[i],motor),
//            MYO_READ_displacement(myo_base[i],motor),
//            MYO_READ_pwmRef(myo_base[i],motor),
//            MYO_READ_update_frequency(myo_base[i]));
//        }

    }
    reset();
}

MyoControl::MyoControl(vector<int32_t *> &myo_base, int32_t *adc_base, NeoPixelPtr neopixel) : myo_base(myo_base),
                                                                         adc_base(adc_base), neopixel(neopixel) {
    if (myo_base.size() > 3)
        ROS_FATAL("a maximum of THREE myoControls is currently supported");
    // initialize control mode
    numberOfMotors = (myo_base.size() == 0 ? NUMBER_OF_MOTORS_MYOCONTROL_0 :
                      (myo_base.size() == 1 ? NUMBER_OF_MOTORS_MYOCONTROL_0 + NUMBER_OF_MOTORS_MYOCONTROL_1 :
                       NUMBER_OF_MOTORS_MYOCONTROL_0 + NUMBER_OF_MOTORS_MYOCONTROL_1 + NUMBER_OF_MOTORS_MYOCONTROL_2));
    ROS_INFO("initializing myoControl for %d spi buses with %d motors in total", myo_base.size(), numberOfMotors);
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

    for (uint motor = 0; motor < numberOfMotors; motor++) {
        if (motor < NUMBER_OF_MOTORS_MYOCONTROL_0) {
            myo_base_of_motor[motor] = 0;
            motor_offset[motor] = 0;
        } else if (motor < NUMBER_OF_MOTORS_MYOCONTROL_0 + NUMBER_OF_MOTORS_MYOCONTROL_1) {
            myo_base_of_motor[motor] = 1;
            motor_offset[motor] = NUMBER_OF_MOTORS_MYOCONTROL_0;
        } else {
            myo_base_of_motor[motor] = 2;
            motor_offset[motor] = NUMBER_OF_MOTORS_MYOCONTROL_0 + NUMBER_OF_MOTORS_MYOCONTROL_1;
        }
    }

    changeControl(VELOCITY);

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

void MyoControl::changeControl(int motor, int mode, control_Parameters_t &params, int32_t setPoint) {
    changeControl(motor, mode, params);
    MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], setPoint);
    int kp, ki, kd, fg, db, sp, od;
    getPIDcontrollerParams(kp, ki, kd, fg, db, sp, od, motor);
    ROS_INFO("change control motor %d (Kp: %d, Kd %d, Ki %d, ForwardGain %d, deadband %d, "
                     "OutputDivider %d, setPoint %d)", motor, kp, ki, kd, fg, db, od, sp);
}

void MyoControl::changeControl(int motor, int mode, control_Parameters_t &params) {
    MYO_WRITE_control(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], mode);
    MYO_WRITE_reset_controller(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
    MYO_WRITE_Kp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], params.Kp);
    MYO_WRITE_Kd(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], params.Kd);
    MYO_WRITE_Ki(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], params.Ki);
    MYO_WRITE_forwardGain(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], params.forwardGain);
    MYO_WRITE_deadBand(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], params.deadBand);
    MYO_WRITE_IntegralPosMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], params.IntegralPosMax);
    MYO_WRITE_IntegralNegMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], params.IntegralNegMax);
    MYO_WRITE_outputPosMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], params.outputPosMax);
    MYO_WRITE_outputNegMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], params.outputNegMax);
    MYO_WRITE_outputDivider(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], params.outputDivider);
    if (mode == POSITION) {
        int32_t current_position = MYO_READ_position(myo_base[myo_base_of_motor[motor]],
                                                     motor - motor_offset[motor]);
        MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], current_position);
    } else {
        MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], 0);
    }
}

void MyoControl::changeControl(int motor, int mode) {
    MYO_WRITE_reset_controller(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
    MYO_WRITE_Kp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], control_params[motor][mode].Kp);
    MYO_WRITE_Kd(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], control_params[motor][mode].Kd);
    MYO_WRITE_Ki(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], control_params[motor][mode].Ki);
    MYO_WRITE_forwardGain(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                          control_params[motor][mode].forwardGain);
    MYO_WRITE_deadBand(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                       (control_params[motor][mode].deadBand));
    MYO_WRITE_IntegralPosMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                             control_params[motor][mode].IntegralPosMax);
    MYO_WRITE_IntegralNegMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                             control_params[motor][mode].IntegralNegMax);
    MYO_WRITE_outputPosMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                           control_params[motor][mode].outputPosMax);
    MYO_WRITE_outputNegMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                           control_params[motor][mode].outputNegMax);
    MYO_WRITE_control(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], mode);
    MYO_WRITE_outputDivider(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                            control_params[motor][mode].outputDivider);
    if (mode == POSITION) {
        int32_t current_position = MYO_READ_position(myo_base[myo_base_of_motor[motor]],
                                                     motor - motor_offset[motor]);
        MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], current_position);
    } else {
        MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], 0);
    }
}

void MyoControl::changeControl(int mode) {
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        MYO_WRITE_reset_controller(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
        MYO_WRITE_Kp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], control_params[motor][mode].Kp);
        MYO_WRITE_Kd(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], control_params[motor][mode].Kd);
        MYO_WRITE_Ki(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], control_params[motor][mode].Ki);
        MYO_WRITE_forwardGain(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                              control_params[motor][mode].forwardGain);
        MYO_WRITE_deadBand(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                           (control_params[motor][mode].deadBand));
        MYO_WRITE_IntegralPosMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                                 control_params[motor][mode].IntegralPosMax);
        MYO_WRITE_IntegralNegMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                                 control_params[motor][mode].IntegralNegMax);
        MYO_WRITE_outputPosMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                               control_params[motor][mode].outputPosMax);
        MYO_WRITE_outputNegMax(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                               control_params[motor][mode].outputNegMax);
        MYO_WRITE_control(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], mode);
        MYO_WRITE_outputDivider(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                                control_params[motor][mode].outputDivider);
        if (mode == POSITION) {
            int32_t current_position = MYO_READ_position(myo_base[myo_base_of_motor[motor]],
                                                         motor - motor_offset[motor]);
            MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], current_position);
        } else {
            MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], 0);
        }
    }
}

void MyoControl::changeControlParameters(int motor, control_Parameters_t &params) {
    control_params[motor][params.control_mode] = params;
}

void MyoControl::reset() {
    for (uint i = 0; i < myo_base.size(); i++) {
        MYO_WRITE_reset_myo_control(myo_base[i], true);
        MYO_WRITE_reset_myo_control(myo_base[i], false);
    }
}

bool MyoControl::setSPIactive(int motor, bool active) {
    int myo_base_nr = (motor < NUMBER_OF_MOTORS_MYOCONTROL_0 ? 0 :
                       (motor < NUMBER_OF_MOTORS_MYOCONTROL_0 + NUMBER_OF_MOTORS_MYOCONTROL_1 ? 1 :
                        2));
    MYO_WRITE_spi_activated(myo_base[myo_base_nr], active);
}

void MyoControl::getPIDcontrollerParams(int &Pgain, int &Igain, int &Dgain, int &forwardGain, int &deadband,
                                        int &setPoint, int &outputDivider, int motor) {
    Pgain = MYO_READ_Kp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
    Igain = MYO_READ_Ki(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
    Dgain = MYO_READ_Kd(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
    forwardGain = MYO_READ_forwardGain(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
    deadband = MYO_READ_deadBand(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
    setPoint = MYO_READ_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
    outputDivider = MYO_READ_outputDivider(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
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

    return MYO_READ_control(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

int32_t MyoControl::getMotorAngle(int motor) {
    return MYO_READ_myo_brick_motor_angle(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

int32_t MyoControl::getMotorAnglePrev(int motor) {
    return MYO_READ_myo_brick_motor_raw_angle_prev(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

int32_t MyoControl::getRawMotorAngle(int motor) {
    return MYO_READ_myo_brick_motor_raw_angle(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

int32_t MyoControl::getRelativeMotorAngle(int motor) {
    return MYO_READ_myo_brick_motor_relative_angle(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

int32_t MyoControl::getMotorAngleOffset(int motor) {
    return MYO_READ_myo_brick_motor_offset_angle(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

int32_t MyoControl::getRevolutionCounter(int motor) {
    return MYO_READ_myo_brick_motor_angle_revolution_counter(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

bool MyoControl::getPowerSense() {
    return (bool) MYO_READ_power_sense(myo_base[0]);
}

int16_t MyoControl::getPWM(int motor) {
    return MYO_READ_pwmRef(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

int32_t MyoControl::getPosition(int motor) {
    return MYO_READ_position(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

int32_t MyoControl::getVelocity(int motor) {
    int16_t vel = MYO_READ_velocity(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
    return ((int32_t) vel) * MOTOR_BOARD_COMMUNICATION_FREQUENCY;
}

int32_t MyoControl::getDisplacement(int motor) {
    return (int32_t)MYO_READ_displacement(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

void MyoControl::setPosition(int motor, int32_t setPoint) {
    MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], (int32_t) setPoint);
}

void MyoControl::setVelocity(int motor, int32_t setPoint) {
    MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor],
                 (int32_t) (setPoint / MOTOR_BOARD_COMMUNICATION_FREQUENCY));
}

void MyoControl::setDisplacement(int motor, int32_t setPoint) {
    MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], setPoint);
}

void MyoControl::setPWM(int motor, int32_t setPoint) {
    MYO_WRITE_sp(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor], setPoint);
}

bool MyoControl::configureMyoBricks(vector<uint8_t> &motorIDs,
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
        int myoBaseOfMotor = myo_base_of_motor[motor_id];
        int myoControlMotor = motor_id - motor_offset[motor_id];
        ROS_INFO("myoBrick %d of myoControl %d", myoControlMotor, myoBaseOfMotor);
        myo_brick |= (1 << myoControlMotor);
        MYO_WRITE_myo_brick_gear_box_ratio(myo_base[myoBaseOfMotor], myoControlMotor, gearBoxRatio[i]);
        MYO_WRITE_myo_brick_encoder_multiplier(myo_base[myoBaseOfMotor], myoControlMotor, encoderMultiplier[i]);
        int ratio = MYO_READ_myo_brick_gear_box_ratio(myo_base[myoBaseOfMotor], myoControlMotor);
        int multiplier = MYO_READ_myo_brick_encoder_multiplier(myo_base[myoBaseOfMotor], myoControlMotor);
        if (ratio != gearBoxRatio[i] ||
            multiplier != encoderMultiplier[i]) { // if the value was not written correctly, we abort!
            ROS_FATAL("id %d, ratio %d, multiplier %d", motor_id, ratio, multiplier);
            return false;
        }
        str << (int) motor_id << "\t\t| " << ratio << "\t\t| " << multiplier << endl;
        MYO_WRITE_myo_brick(myo_base[myoBaseOfMotor], myo_brick);
        i++;
    }
    ROS_INFO_STREAM(str.str());
    return true;
}

int16_t MyoControl::getCurrent(int motor) {
    return MYO_READ_current(myo_base[myo_base_of_motor[motor]], motor - motor_offset[motor]);
}

void MyoControl::getDefaultControlParams(control_Parameters_t *params, int control_mode) {
    params->outputPosMax = 500;
    params->outputNegMax = -500;

    params->radPerEncoderCount = 2 * 3.14159265359 / (2000.0 * 53.0);

    switch (control_mode) {
        case POSITION:
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
        case VELOCITY:
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
        case DISPLACEMENT:
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
        default:
            ROS_ERROR("unknown control mode");
            break;
    }

}

void MyoControl::allToPosition(int32_t pos) {
    changeControl(POSITION);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        setPosition(motor, pos);
    }
}

void MyoControl::allToVelocity(int32_t vel) {
    changeControl(VELOCITY);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        setVelocity(motor, vel);
    }
}

void MyoControl::allToDisplacement(int32_t displacement) {
    changeControl(DISPLACEMENT);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        setDisplacement(motor, displacement);
    }
}

void MyoControl::allToDirectPWM(int32_t pwm) {
    changeControl(DIRECT_PWM);
    for (uint motor = 0; motor < numberOfMotors; motor++) {
        setPWM(motor, pwm);
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

float MyoControl::getWeight(int load_cell) {
    uint32_t adc_value = readADC(load_cell);
    float weight = (adc_weight_parameters[0] + weight_offset + adc_weight_parameters[1] * adc_value) * 9.81f;
    return weight;
}

float MyoControl::getWeight(int load_cell, uint32_t &adc_value) {
    adc_value = readADC(load_cell);
    float weight = (adc_weight_parameters[0] + weight_offset + adc_weight_parameters[1] * adc_value) * 9.81f;
    return weight;
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
            if (controlmode[*it] == POSITION)
                trajectories[idList[*it]].push_back(getPosition(*it));
            else if (controlmode[*it] == VELOCITY)
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
        changeControl(motor,DISPLACEMENT);
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
        changeControl(motor,DISPLACEMENT);
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
               << POSITION << "\" samplingTime=\"" << samplingTime * 1000.0f << "\">"
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
                changeControl(motor.first, POSITION);
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

void MyoControl::estimateMotorAngleLinearisationParameters(int motor, int degree, vector<float> &coeffs, int timeout,
                                                           uint numberOfDataPoints, float delta_revolution_negative,
                                                           float delta_revolution_positive, vector<double> &motor_angle,
                                                           vector<double> &motor_encoder) {
    auto it = find(myo_bricks.begin(), myo_bricks.end(), motor);
    if (it == myo_bricks.end()) {
        cerr << "motor " << motor << " is not configured as a myobrick, aborting..." << endl;
        return;
    }

    ptrdiff_t id = distance(myo_bricks.begin(), it);

    changeControl(motor, POSITION);
    setPosition(motor, 0);
    while (abs(getPosition(motor)) > 1000) {
        cout << "waiting for motor " << motor << " to go to zero position" << endl;
        usleep(1000000);
    }
    changeControl(motor, VELOCITY);
    milliseconds ms_start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()), ms_stop;
    ofstream outfile;
    char str[100];
    sprintf(str, "motorAngleLinearisation_calibration_motor%d.csv", motor);
    outfile.open(str);
    if (!outfile.is_open()) {
        cout << "could not open file " << str << " for writing, aborting!" << endl;
        return;
    }
    outfile << "motor_angle[ticks], motor_optical_encoder[ticks]" << endl;

    int32_t initial_motor_pos = getPosition(motor);
    int32_t initial_motor_angle = abs(getMotorAngle(motor)) % 4096;
    setVelocity(motor, -30000);
    bool go_backward = true;
    int back_and_forth = 2;

    int pos_min = initial_motor_pos + ((delta_revolution_negative / 360) * 1024 * myo_bricks_gearbox_ratio[id]);
    int pos_max = initial_motor_pos + ((delta_revolution_positive / 360) * 1024 * myo_bricks_gearbox_ratio[id]);

    cout << "position min\t" << pos_min << endl;
    cout << "position max\t" << pos_max << endl;

    int sample = 0;

    do {
        if (go_backward) {
            if (getPosition(motor) < pos_min) {
                setVelocity(motor, 30000);
                go_backward = false;
                cout << "going forward" << endl;
            }
        } else {
            if (getPosition(motor) > pos_max) {
                setVelocity(motor, -30000);
                go_backward = true;
                cout << "going backward" << endl;
                back_and_forth--;
                if(back_and_forth<=0)
                    break;
            }
        }

        // note the motor angle
        motor_angle.push_back(abs(getMotorAngle(motor)) % 4096);
        // note the motor encoder
        motor_encoder.push_back(
                abs(getPosition(motor) / myo_bricks_gearbox_ratio[id] * myo_bricks_encoder_multiplier[id] -
                    initial_motor_angle) % 4096);
        outfile << motor_angle.back() << ", " << motor_encoder.back() << endl;
        if(sample%100==0)
            printf("sample %d motor angle %lf \t motor encoder %lf\n", sample, motor_angle.back(), motor_encoder.back());
        ms_stop = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        sample++;
        usleep(10000);
    } while ((ms_stop - ms_start).count() < timeout && motor_angle.size() < numberOfDataPoints);
    changeControl(motor, POSITION);
    setPosition(motor, initial_motor_pos);
    polynomialRegression(degree, motor_angle, motor_encoder, coeffs);
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
