#include "interfaces/canBusControl.hpp"

// TODO add internal battery power reading

CanBusControl::CanBusControl(MotorConfigPtr motor_config)
{
    this->motor_config = motor_config;
    // iterate over all can buses
    for (auto &bus : motor_config->canbus)
    {
        tuple<string, int> busInfo = {bus.second[0]->bus_socket_name, bus.second[0]->update_frequency};
        // check if can bus is already used
        if (std::find(can_bus_names.begin(), can_bus_names.end(), busInfo) != can_bus_names.end())
        {
            ROS_FATAL("Multiple Can buses want to use the same socket.");
        };
        // if not append to the can bus names
        can_bus_names.push_back(busInfo);
        // init interim map for orering of buses
        map<int, CanMotorPtr> motorOnBus;
        for (auto &m : bus.second)
        {
            // check if can id has already been used
            if (motorOnBus.find(m->bus_id) != motorOnBus.end())
            {
                ROS_FATAL("motor with can id %d and global id %d already defined on bus, check you motor config yaml", m->bus_id, m->motor_id_global);
            }
            // valid motor
            CanMotorPtr motor_ = CanMotorPtr(new CanMotor(m->bus_id, m->motor_id_global, m->bus_socket_name, m->muscleType));
            motorOnBus[m->bus_id] = motor_;
            canMotor[m->motor_id_global] = motor_;
        }
        // append motors to vector
        canMotorOnSocket.push_back(motorOnBus);
    }
    // create all CanSockets
    CreateSockets();
    SetSocketsForMotor();
    // get all default control parameters
    for (auto &motor : canMotor)
    {
        for (int mode = 0; mode <= 3; mode++)
        {
            control_Parameters_t params;
            GetDefaultControlParams(&params, mode, motor.second->muscleType, motor.second->motor_global_id);
            control_params[motor.second->motor_global_id][mode] = params;
        }
        SetControlMode(motor.second->motor_global_id, 0);
        //SetPoint(motor.second->motor_global_id, 0);
    }
}

CanBusControl::~CanBusControl()
{
    // join all the request threads and stop all motors on deletion of the object
    ROS_INFO("shutting down can bus control");
    for (auto &tread : statusRequestThreads)
    {
        if (tread->joinable())
            tread->join();
    }
    for (auto &motor : canMotor)
    {
        OffMotor(motor.second->motor_global_id);
    }
}

void CanBusControl::CreateSockets()
{
    // iterate over each canbus name to create the socket verify over ROS stream
    for (auto &socket_name : can_bus_names)
    {
        ROS_INFO("Create socket with the name %s", get<0>(socket_name));
        CanSocketPtr socket_ = CanSocketPtr(new CanSocket(socket_name));
        canSockets.push_back(socket_);
    }
    ROS_INFO("All sockets have been succesfully created.");
}

void CanBusControl::SetSocketsForMotor()
{
    // iterater over each motor and add thier socket to their parameter
    for (auto &motor : canMotor)
    {
        string socketName = motor.second->bus_socket_name;
        for (auto &socket : canSockets)
        {
            if (socket->name == socketName)
            {
                // virfy over ROs stream that it has each motor has been added successfuly
                ROS_INFO("Motor with id: %d has the socket with the name %s", motor.second->motor_global_id, socketName);
                motor.second->socket_ = socket;
                break;
            }
        }
    }
    ROS_INFO("All motors been set");
}

void CanBusControl::StartStatusRequestThreads()
{
    // create a thread for each socket so each socket is always fully used
    boost::shared_ptr<std::thread> inter_thread_pointer;
    for (int i = 0; i < canMotorOnSocket.size(); i++)
    {
        inter_thread_pointer = boost::shared_ptr<std::thread>(new std::thread(&CanBusControl::StatusRequestLoop, this));
        inter_thread_pointer->detach();
        statusRequestThreads.push_back(inter_thread_pointer);
    }
    // set that each thread has been created so the threads can now find out their id from the vector as well as their corresponding socket
    finished_init_threads = true;
    ROS_INFO("all threads have been started");
}

void CanBusControl::StatusRequestLoop()
{
    // make sure the thread is in the vector
    while (!finished_init_threads)
    {
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // get the right vector for iteration
    int id;
    for (int i = 0; i < statusRequestThreads.size(); i++)
    {
        if (statusRequestThreads[i]->get_id() == std::this_thread::get_id())
        {
            id = i;
            break;
        }
    }
    // get the motors that have to be iterated over
    std::map<int, CanMotorPtr> busMotors = canMotorOnSocket[id];
    // get the socket
    CanSocketPtr socket = canSockets[id];
    int update_frequen = socket->update_frequency;
    ROS_INFO("The thread with id: %d is started.", id);
    // run while ros is ok
    ros::Rate rate(update_frequen);
    int recievRet = 0;
    while (ros::ok())
    {
        for (std::pair<int, CanMotorPtr> m : busMotors)
        {
            // by applying the peterson algorithm it is always safe that the socket is only used by one resource
            while (socket->in_use_by != -1)
            {
                this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            socket->in_use_by = CAN_CONTROLLER_USED_STATUS;
            this_thread::sleep_for(std::chrono::milliseconds(3));
            if (socket->in_use_by == CAN_CONTROLLER_USED_STATUS)
            {
                // TODO add different motors
                recievRet = 0;
                // get the motor status
                uint8_t status[8] = RMD_X6_STATUS_REQUEST_FRAME;
                socket->canTransmit(m.second->bus_id, status);\
                sleep(0.001);
                recievRet = socket->canRensieve(status);
                //socket->canRensieve(status);
                if (recievRet == 0 && status[0] == RMD_X6_STATUS_RESPONSE_ID)
                {
                    // write to motor parameters
                    m.second->speed = status[5] << 8 | status[4];
                    m.second->torque = status[3] << 8 | status[2];
                    m.second->temperature = status[1];
                    /*
                    if(status[1] >= SAFTY_TEMP_OFF && m.second->is_on == true){
                        m.second->is_on = false;
                        uint8_t stop[8] RMD_X6_MOTOR_STOP_FRAME;
                        socket->canTransmit(m.second->bus_id, stop);
                        sleep(0.0001);
                        recievRet = socket->canRensieve(stop);
                        //BUG check if it works
                    }else{
                        m.second->is_on = true;
                    }
                    */
                }
                // delay for next asking
                sleep(0.001);
                // get the exact position
                uint8_t pos[8] = RMD_X6_READ_MULTITURN_ANGLE;
                socket->canTransmit(m.second->bus_id, pos);
                sleep(0.001);
                recievRet = 0;
                recievRet = socket->canRensieve(pos);
                //socket->canRensieve(pos);
                if (recievRet == 0 && pos[0] == RMD_X6_READ_MULTITURN_ANGLE_ID)
                {
                    // write to motor parameter
                    m.second->encoderPosition = pos[7] << 24 | pos[6] << 16 | pos[5] << 8 | pos[4];
                    // also keep the direction in mind
                    if (pos[7] == 1)
                    {
                        m.second->encoderPosition = m.second->encoderPosition * -1;
                    }
                }
                // free the socket for diffrent use
                socket->in_use_by = -1;
            };
        }
        rate.sleep();
    }
}

void CanBusControl::UseSocket(int can_id, uint8_t *data, CanSocketPtr socket)
{
    // apply the pertson algorithm for on socket and send and recieve the data on a socket
    while (socket->in_use_by != -1)
    {
        //string s(data, data+8);
        sleep(0.0005);
    }
    socket->in_use_by = OUTSIDE_CAN_COMMAND;
    sleep(0.001);
    if (socket->in_use_by == OUTSIDE_CAN_COMMAND)
    {
        socket->canTransmit(can_id, data);
        sleep(0.001);
        socket->canRensieve(data);
        socket->in_use_by = -1;
    }
}

void CanBusControl::StartMotor(int motor)
{
    //FIXME remove
    //uint8_t data[8] = RMD_X6_MOTOR_START_FRAME;
    //UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    ROS_INFO("Motor %d is started", motor);
    canMotor[motor]->is_on = true;
}

void CanBusControl::StopMotor(int motor)
{
    uint8_t data[8] = RMD_X6_MOTOR_STOP_FRAME;
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    ROS_INFO("Motor %d is stoped", motor);
    canMotor[motor]->is_on = false;
}

void CanBusControl::OffMotor(int motor)
{
    uint8_t data[8] = RMD_X6_MOTOR_OFF_FRAME;
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    ROS_INFO("Motor %d is stoped", motor);
    canMotor[motor]->is_on = false;
}

void CanBusControl::GetPIDValues(control_Parameters_t *params, int motor, int mode)
{
    uint8_t data[8] = RMD_X6_READ_PID_VALUE;
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    if (data[0] == RMD_X6_READ_KP_TORQUE_ID)
    {
        int pidKp = 0;
        int pidKi = 0;
        switch (mode)
        {
        case ENCODER0_POSITION:
            pidKp = data[6];
            pidKi = data[7];
            break;
        case ENCODER1_POSITION:
            ROS_WARN("The Can bus does not hase a second encoder");
            break;
        case DISPLACEMENT:
            pidKp = data[4];
            pidKi = data[5];
            break;
        case DIRECT_PWM:
            pidKp = data[2];
            pidKi = data[3];
            break;
        default:
            ROS_WARN("There is no default can protocol");
            break;
        }
    }
}

void CanBusControl::GetKpTorqueControl(control_Parameters_t *params, int motor)
{
    uint8_t data[8] = RMD_X6_READ_KP_TORQUE_FRAME;
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    if (data[0] == RMD_X6_READ_KP_TORQUE_ID)
    {
        int pidKp = data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4];
        // can be converted by deviding by 16777216 but is not done
        params->Kp = pidKp;
    }
}
void CanBusControl::GetKiTorqueControl(control_Parameters_t *params, int motor)
{
    uint8_t data[8] = RMD_X6_READ_KI_TORQUE_FRAME;
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    if (data[0] == RMD_X6_READ_KI_TORQUE_ID)
    {
        int pidKi = data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4];
        // can be converted by deviding by 16777216 but is not done
        params->Ki = pidKi;
    }
}
void CanBusControl::GetKpSpeedControl(control_Parameters_t *params, int motor)
{
    uint8_t data[8] = RMD_X6_READ_KP_SPEED_FRAME;
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    if (data[0] == RMD_X6_READ_KP_SPEED_ID)
    {
        int pidKp = data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4];
        // can be converted by deviding by 16777216 but is not done
        params->Kp = pidKp;
    }
}
void CanBusControl::GetKiSpeedControl(control_Parameters_t *params, int motor)
{
    uint8_t data[8] = RMD_X6_READ_KI_SPEED_FRAME;
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    if (data[0] == RMD_X6_READ_KI_SPEED_ID)
    {
        int pidKi = data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4];
        // can be converted by deviding by 16777216 but is not done
        params->Ki = pidKi;
    }
}
void CanBusControl::GetKpPositionControl(control_Parameters_t *params, int motor)
{
    uint8_t data[8] = RMD_X6_READ_KP_POSITION_FRAME;
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    if (data[0] == RMD_X6_READ_KP_POSITION_ID)
    {
        int pidKp = data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4];
        // can be converted by deviding by 16777216 but is not done
        params->Kp = pidKp;
    }
}
void CanBusControl::GetKiPositionControl(control_Parameters_t *params, int motor)
{
    uint8_t data[8] = RMD_X6_READ_KI_POSITION_FRAME;
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    if (data[0] == RMD_X6_READ_KI_POSITION_ID)
    {
        int pidKi = data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4];
        // can be converted by deviding by 16777216 but is not done
        params->Ki = pidKi;
    }
}

void CanBusControl::SetKpTorqueControlRam(control_Parameters_t *params, int motor, int mode)
{
    int val = params->Kp;
    uint8_t data[8] = {RMD_X6_WRITE_KP_TORQUE_RAM, 0x00, 0x00, 0x00, (val & 0xFF), ((val >> 8) & 0xFF), ((val >> 16) & 0xFF), ((val >> 24) & 0xFF)};
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    control_params[motor][mode].Kp = val;
}
void CanBusControl::SetKiTorqueControlRam(control_Parameters_t *params, int motor, int mode)
{
    int val = params->Ki;
    uint8_t data[8] = {RMD_X6_WRITE_KI_TORQUE_RAM, 0x00, 0x00, 0x00, (val & 0xFF), ((val >> 8) & 0xFF), ((val >> 16) & 0xFF), ((val >> 24) & 0xFF)};
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    control_params[motor][mode].Ki = val;
}
void CanBusControl::SetKpSpeedControlRam(control_Parameters_t *params, int motor, int mode)
{
    int val = params->Kp;
    uint8_t data[8] = {RMD_X6_WRITE_KP_SPEED_RAM, 0x00, 0x00, 0x00, (val & 0xFF), ((val >> 8) & 0xFF), ((val >> 16) & 0xFF), ((val >> 24) & 0xFF)};
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    control_params[motor][mode].Kp = val;
}
void CanBusControl::SetKiSpeedControlRam(control_Parameters_t *params, int motor, int mode)
{
    int val = params->Ki;
    uint8_t data[8] = {RMD_X6_WRITE_KI_SPEED_RAM, 0x00, 0x00, 0x00, (val & 0xFF), ((val >> 8) & 0xFF), ((val >> 16) & 0xFF), ((val >> 24) & 0xFF)};
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    control_params[motor][mode].Ki = val;
}
void CanBusControl::SetKpPositionControlRam(control_Parameters_t *params, int motor, int mode)
{
    int val = params->Kp;
    uint8_t data[8] = {RMD_X6_WRITE_KP_POSITION_RAM, 0x00, 0x00, 0x00, (val & 0xFF), ((val >> 8) & 0xFF), ((val >> 16) & 0xFF), ((val >> 24) & 0xFF)};
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    control_params[motor][mode].Kp = val;
}
void CanBusControl::SetKiPositionControlRam(control_Parameters_t *params, int motor, int mode)
{
    int val = params->Ki;
    uint8_t data[8] = {RMD_X6_WRITE_KI_POSITION_RAM, 0x00, 0x00, 0x00, (val & 0xFF), ((val >> 8) & 0xFF), ((val >> 16) & 0xFF), ((val >> 24) & 0xFF)};
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    control_params[motor][mode].Ki = val;
}

void CanBusControl::SetKpParamRam(control_Parameters_t *params, int motor, int mode)
{
    switch (mode)
    {
    case ENCODER0_POSITION:
        if (canMotor[motor]->muscleType == "rmdX6")
        {
            SetKpPositionControlRam(params, motor, mode);
        }
        else
        {
            ROS_WARN("There is no default can protocol");
        }
        break;
    case ENCODER1_POSITION:
        ROS_WARN("The Can bus does not hase a second encoder");
        break;
    case DISPLACEMENT:
        if (canMotor[motor]->muscleType == "rmdX6")
        {
            SetKpTorqueControlRam(params, motor, mode);
        }
        else
        {
            ROS_WARN("There is no default can protocol");
        }
        break;
    case DIRECT_PWM:
        if (canMotor[motor]->muscleType == "rmdX6")
        {
            SetKpSpeedControlRam(params, motor, mode);
        }
        else
        {
            ROS_WARN("There is no default can protocol");
        }
        break;
    default:
        ROS_WARN("There is no default can protocol");
        break;
    }
}

void CanBusControl::SetKiParamRam(control_Parameters_t *params, int motor, int mode)
{
    switch (mode)
    {
    case ENCODER0_POSITION:
        if (canMotor[motor]->muscleType == "rmdX6")
        {
            SetKiPositionControlRam(params, motor, mode);
        }
        else
        {
            ROS_WARN("There is no default can protocol");
        }
        break;
    case ENCODER1_POSITION:
        ROS_WARN("The Can bus does not hase a second encoder");
        break;
    case DISPLACEMENT:
        if (canMotor[motor]->muscleType == "rmdX6")
        {
            SetKiTorqueControlRam(params, motor, mode);
        }
        else
        {
            ROS_WARN("There is no default can protocol");
        }
        break;
    case DIRECT_PWM:
        if (canMotor[motor]->muscleType == "rmdX6")
        {
            SetKiSpeedControlRam(params, motor, mode);
        }
        else
        {
            ROS_WARN("There is no default can protocol");
        }
        break;
    default:
        ROS_WARN("There is no default can protocol");
        break;
    }
}

void CanBusControl::SetPIDParamRam(control_Parameters_t *params, int motor, int mode)
{
    control_params[motor][mode].Kp =params->Kp;
    control_params[motor][mode].Ki =params->Ki;
    uint8_t data[8] = {RMD_X6_WRITE_PID_VALUE_ID, 0x00, (control_params[motor][DISPLACEMENT].Kp & 0xFF), (control_params[motor][DISPLACEMENT].Ki & 0xFF),
    (control_params[motor][DIRECT_PWM].Kp & 0xFF), (control_params[motor][DIRECT_PWM].Ki & 0xFF),
    (control_params[motor][ENCODER0_POSITION].Kp & 0xFF), (control_params[motor][ENCODER0_POSITION].Ki & 0xFF)};
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
}

bool CanBusControl::SetControlMode(int motor, int mode, control_Parameters_t &params, float setPoint)
{
    if (!SetControlMode(motor, mode, params))
    {
        return false;
    }
    SetPoint(motor, setPoint);
    return true;
}

bool CanBusControl::SetID(int motor, int id)
{
    int respondedCanId = -1;
    // check if can id does not exists on the socket
    uint8_t data[8] = {RMD_X6_SET_CAN_ID, 0x00, 0x01, 0x00,
                       0x00, 0x00, 0x00, 0x00};
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    if (data[0] == RMD_X6_SET_CAN_ID)
    {
        respondedCanId = data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4];
    }
    else
    {
        ROS_ERROR("The motor did not respond.");
        return false;
    }
    if (respondedCanId != -1)
    {
        ROS_ERROR("The can id aready exists.");
        return false;
    }
    // set can id
    // data[2] is read write flag 0x00 is write
    data[0] = RMD_X6_SET_CAN_ID;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = (id & 0xFF);
    data[5] = ((id >> 8) & 0xFF);
    data[6] = ((id >> 16) & 0xFF);
    data[7] = ((id >> 24) & 0xFF);
    UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    if (data[0] == RMD_X6_SET_CAN_ID)
    {
        respondedCanId = data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4];
        // add the canId offset
        respondedCanId += 0x140;
        if (id == respondedCanId)
        {
            return true;
        }
        return false;
    }
    return false;
}

bool CanBusControl::SetControlMode(int motor, int mode, control_Parameters_t &params)
{
    if (mode >= ENCODER0_POSITION && mode <= DIRECT_PWM)
    {
        if (params.Kp != control_params[motor][mode].Kp || params.Ki != control_params[motor][mode].Ki)
        {
            SetPIDParamRam(&params, motor, mode);
        }
        control_params[motor][mode].deadband = params.deadband;
        control_params[motor][mode].IntegralLimit = params.IntegralLimit;
        control_params[motor][mode].PWMLimit = params.PWMLimit;
        control_params[motor][mode].Kd = params.Kd;
        if (canMotor[motor]->is_on)
        {
            if (canMotor[motor]->control_mode == DIRECT_PWM)
            {
                ROS_WARN("It is not safe to switch between Speed control and something else.");
            }
        }
        canMotor[motor]->control_mode = mode;
        return true;
    }
    else
    {
        return false;
    }
}

bool CanBusControl::SetControlMode(int motor, int mode)
{
    if (mode >= ENCODER0_POSITION && mode <= DIRECT_PWM)
    {
        if (canMotor[motor]->is_on)
        {
            if (canMotor[motor]->control_mode == DIRECT_PWM)
            {
                ROS_WARN("It is not safe to switch between Speed control and something else.");
            }
        }
        canMotor[motor]->control_mode = mode;
        return true;
    }
    else
    {
        return false;
    }
}

bool CanBusControl::SetControlMode(int mode)
{
    if (mode >= ENCODER0_POSITION && mode <= DIRECT_PWM)
    {
        for (auto &motor : canMotor)
        {
            if (motor.second->is_on)
            {
                if (motor.second->control_mode == DIRECT_PWM)
                {
                    ROS_WARN("It is not safe to switch between Speed control and something else.");
                }
            }
            motor.second->control_mode = mode;
        }
        return true;
    }
    else
    {
        return false;
    }
}

int32_t CanBusControl::GetBaudrate(int motor)
{
    return motor_config->motor[motor]->baudrate;
}

void CanBusControl::GetControllerParameter(int motor, int32_t &Kp, int32_t &Ki, int32_t &Kd,
                                           int32_t &deadband, int32_t &IntegralLimit, float &PWMLimit)
{
    control_Parameters_t params = control_params[motor][canMotor[motor]->control_mode];
    Kp = params.Kp;
    Ki = params.Ki;
    Kd = params.Kd;
    IntegralLimit = params.IntegralLimit;
    deadband = params.deadband;
    PWMLimit = params.PWMLimit;
}

uint8_t CanBusControl::GetControlMode(int motor)
{
    return canMotor[motor]->control_mode;
}

float CanBusControl::GetCurrent(int motor)
{
    float power = -1;
    uint8_t data[8] = RMD_X6_POWER_AQUISITION_FRAME;
    //UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
    if (data[0] == RMD_X6_POWER_AQUISITION_ID)
    {
        // is in the format 0.1 w / LSB
        int powerInt = data[7] << 8 | data[6];
        power = powerInt / 10;
    }
    return power;
}

int CanBusControl::GetCurrentAverage()
{
    // FIXME not even implemented in icebusControl
    return 0;
}

float CanBusControl::GetCurrentLimit(int motor)
{
    // hard coded depending on the motor
    if (canMotor[motor]->muscleType == "rmdX6")
    {
        return 70.0;
    }
    else
    {
        return -1;
    }
}

void CanBusControl::GetDefaultControlParams(control_Parameters_t *params, int control_mode, string muscleType, int motor_id)
{
    switch (control_mode)
    {
    case ENCODER0_POSITION:
        if (muscleType == "rmdX6")
        {
            params->IntegralLimit = 25;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 360; // 30% of max pwm
            // ask the motor for its default paramerter because those can be changed
             GetPIDValues(params,motor_id,control_mode);
        }
        else
        {
            params->IntegralLimit = 0;
            params->Kp = 0;
            params->Ki = 0;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 0;
        }
        break;
    case ENCODER1_POSITION:
        ROS_WARN("For CAN bus there is no second encoder");
        if (muscleType == "rmdX6")
        {
            params->IntegralLimit = 25;
            params->Kp = 16;
            params->Ki = 1;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 360; // 10% of max pwm
        }
        else
        {
            params->IntegralLimit = 0;
            params->Kp = 0;
            params->Ki = 0;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 0;
        }
        break;
    case DISPLACEMENT:
        if (muscleType == "rmdX6")
        {
            params->IntegralLimit = 0;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 10; // 10% of max pwm
            // ask the motor for its default paramerter because those can be changed
            GetPIDValues(params,motor_id,control_mode);
        }
        else
        {
            params->IntegralLimit = 0;
            params->Kp = 0;
            params->Ki = 0;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 0;
        }
        break;
    case DIRECT_PWM:
        if (muscleType == "rmdX6")
        {
            // FIXME This is not pwm but speed
            params->IntegralLimit = 0;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 10; // 10% of max pwm
            // ask the motor for its default paramerter because those can be changed
             GetPIDValues(params,motor_id,control_mode);
        }
        else
        {
            params->IntegralLimit = 0;
            params->Kp = 0;
            params->Ki = 0;
            params->Kd = 0;
            params->deadband = 0;
            params->PWMLimit = 0;
        }
        break;
    default:
        ROS_ERROR("unknown control mode %d, available control modes:\n"
                  "ENCODER0:     0\n"
                  "ENCODER1:     1\n"
                  "DISPLACEMENT: 2\n"
                  "DIRECT_PWM:   3\n",
                  control_mode);
        break;
    }
}

int32_t CanBusControl::GetDisplacement(int motor)
{
    return canMotor[motor]->torque;
}

int32_t CanBusControl::GetEncoderPosition(int motor, int encoder)
{
    if (encoder == ENCODER0)
    {
        return canMotor[motor]->encoderPosition;
    }
    if (encoder == ENCODER1)
    {
        //BUG mis use encoder1 as temperature value
        return canMotor[motor]->temperature;
    }
    else
    {
        // BUG i removed the waring because otherwise it would always show up in the current system
        //  ROS_WARN("The can motors do not have multiple encoders.");
        return -1;
    }
}

string CanBusControl::GetMuscleType(int motor)
{
    return canMotor[motor]->muscleType;
}

int32_t CanBusControl::GetEncoderVelocity(int motor, int encoder)
{
    // FIXME why does this exist
    return 0;
}

int32_t CanBusControl::GetMotorUpdateFrequency(int motor)
{
    return canMotor[motor]->socket_->update_frequency;
}

int32_t CanBusControl::GetNeopixelColor(int motor)
{
    // FIXME does not exist in new motor
    return 0;
}

float CanBusControl::GetSetPoint(int motor)
{
    return canMotor[motor]->setpoint;
}

float CanBusControl::GetPWM(int motor)
{
    // FIXME PWM does not exist it now is speed
    return canMotor[motor]->speed;
}

bool CanBusControl::SetCurrentLimit(int motor, float limit)
{
    // FIXME should this be deleted
    ROS_WARN("Can't be set for Can Bus motors");
    return true;
}

void CanBusControl::SetNeopixelColor(int motor, int32_t color)
{
    // FIXME does not exist for CAN Motor
}

void CanBusControl::SetPoint(int motor, float setPoint)
{
    // FIXME rename PWM Limit is used as speed Limit
    int setpoint = (int32_t)setPoint;
    // check if motor is on / not overheated
    //if(canMotor[motor]->is_on==false){
      //  return;
    //}
    switch (canMotor[motor]->control_mode)
    {
    case ENCODER0_POSITION:
        if (canMotor[motor]->muscleType == "rmdX6")
        {
            // create the right communication frame
            uint8_t data[8] = {RMD_X6_POSITION_CONTROL_2_ID, 0x00, ((int32_t)control_params[motor][ENCODER0_POSITION].PWMLimit & 0xFF), (((int32_t)control_params[motor][ENCODER0_POSITION].PWMLimit >> 8) & 0xFF),
                               (setpoint & 0xFF), ((setpoint >> 8) & 0xFF), ((setpoint >> 16) & 0xFF), ((setpoint >> 24) & 0xFF)};
            if (!canMotor[motor]->is_on)
            {
                StartMotor(motor);
            }
            UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
            if (data[0] == RMD_X6_POSITION_CONTROL_2_ID)
            {
                // update the motor values so no useless communication
                canMotor[motor]->speed = data[5] << 8 | data[4];
                canMotor[motor]->torque = data[3] << 8 | data[2];
                canMotor[motor]->temperature = data[1];
            }
            canMotor[motor]->setpoint = setpoint;
        }
        else
        {
            ROS_WARN("There is no default can protocol");
        }
        break;
    case ENCODER1_POSITION:
        ROS_WARN("The Can bus does not hase a second encoder");
        break;
    case DISPLACEMENT:
        if (canMotor[motor]->muscleType == "rmdX6")
        {
            uint8_t data[8] = {RMD_X6_TORQUE_CONTROL_ID, 0x00, 0x00, 0x00,
                               (setpoint & 0xFF), ((setpoint >> 8) & 0xFF), 0x00, 0x00};
            if (!canMotor[motor]->is_on)
            {
                StartMotor(motor);
            }
            UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
            if (data[0] == RMD_X6_TORQUE_CONTROL_ID)
            {
                canMotor[motor]->speed = data[5] << 8 | data[4];
                canMotor[motor]->torque = data[3] << 8 | data[2];
                canMotor[motor]->temperature = data[1];
            }
            canMotor[motor]->setpoint = setpoint;
        }
        else
        {
            ROS_WARN("There is no default can protocol");
        }
        break;
    case DIRECT_PWM:
        if (canMotor[motor]->muscleType == "rmdX6")
        {
            uint8_t data[8] = {RMD_X6_SPEED_CONTROL_ID, 0x00, 0x00, 0x00,
                               (setpoint & 0xFF), ((setpoint >> 8) & 0xFF), ((setpoint >> 16) & 0xFF), ((setpoint >> 24) & 0xFF)};
            if (!canMotor[motor]->is_on)
            {
                StartMotor(motor);
            }
            UseSocket(canMotor[motor]->bus_id, data, canMotor[motor]->socket_);
            if (data[0] == RMD_X6_POSITION_CONTROL_2_ID)
            {
                canMotor[motor]->speed = data[5] << 8 | data[4];
                canMotor[motor]->torque = data[3] << 8 | data[2];
                canMotor[motor]->temperature = data[1];
            }
            canMotor[motor]->setpoint = setpoint;
        }
        else
        {
            ROS_WARN("There is no default can protocol");
        }
        break;
    default:
        ROS_WARN("There is no default can protocol");
        break;
    }
}

bool CanBusControl::AllToSetpoint(int control_mode, int32_t setpoint)
{
    if (!SetControlMode(control_mode))
        return false;
    for (auto m : motor_config->motor)
    {
        SetPoint(m.first, setpoint);
    }
    return true;
}

void CanBusControl::SetBaudrate(int motor, int baudrate)
{
    // FIXME can not be set on the Can bus
    ROS_WARN("Nothing happend through setting the Baudrate.");
}

void CanBusControl::SetMotorUpdateFrequency(int motor, int32_t freq)
{
    // FIXME unchangeable
    ROS_WARN("The update frequency can only be changed on start up.");
}

bool CanBusControl::MyMotor(int motor)
{
    for (auto bus : motor_config->canbus)
    {
        for (auto m : bus.second)
        {
            if (m->motor_id_global == motor)
            {
                return true;
            }
        }
    }
    return false;
}

float CanBusControl::RecordTrajectories(
    float samplingTime, float recordTime,
    map<int, vector<float>> &trajectories, vector<int> &idList,
    vector<int> &controlmode, string name)
{

    ROS_INFO_STREAM("Started recording a trajectory " + name);
    string filepath = trajectories_folder + name;
    // this will be filled with the trajectories
    AllToSetpoint(DISPLACEMENT, predisplacement);

    // samplingTime milli -> seconds
    samplingTime /= 1000.0f;

    double elapsedTime = 0.0, dt;
    long sample = 0;

    // start recording
    timer.start();
    do
    {
        dt = elapsedTime;
        //        for (uint motor = 0; motor < idList.size(); motor++) {
        for (auto it = idList.begin(); it != idList.end(); it++)
        {
            if (controlmode[*it] == ENCODER0_POSITION)
                trajectories[idList[*it]].push_back(GetEncoderPosition(*it, ENCODER0_POSITION));
            else if (controlmode[*it] == ENCODER1_POSITION)
                trajectories[idList[*it]].push_back(GetEncoderPosition(*it, ENCODER1_POSITION));
        }
        sample++;
        elapsedTime = timer.elapsedTime();
        dt = elapsedTime - dt;
        // if faster than sampling time sleep for difference
        if (dt < samplingTime)
        {
            usleep((samplingTime - dt) * 1000000.0);
            elapsedTime = timer.elapsedTime();
        }
    } while (elapsedTime < recordTime);

    // set force to zero
    AllToSetpoint(DISPLACEMENT, 0);

    // done recording
    if (filepath.empty())
    {
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
    if (outfile.is_open())
    {
        ss << "<?xml version=\"1.0\" ?>"
           << std::endl;
        uint m = 0;
        char motorname[10];
        for (uint m = 0; m < idList.size(); m++)
        {
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
    return elapsedTime / (double)sample * 1000.0f;
}

float CanBusControl::StartRecordTrajectories(
    float samplingTime, map<int, vector<float>> &trajectories,
    vector<int> &idList, string name)
{
    string filepath = trajectories_folder + name;
    recording = true;
    ROS_INFO_STREAM("Started recording a trajectory " + name);
    // this will be filled with the trajectories
    for (auto motor : idList)
    {
        SetControlMode(motor, DISPLACEMENT);
        SetPoint(motor, predisplacement);
    }

    // samplingTime milli -> seconds
    samplingTime /= 1000.0f;

    double elapsedTime = 0.0, dt;
    long sample = 0;
    ros::Rate rate(1.0 / samplingTime);
    //    ROS_INFO_STREAM(1.0/samplingTime);
    // start recording
    do
    {
        dt = elapsedTime;
        for (auto it : idList) //.begin(); it != idList.end(); it++ ) {
        {
            trajectories[it].push_back(GetEncoderPosition(it, ENCODER0_POSITION));
        }
        sample++;
        rate.sleep();
    } while (recording);

    for (auto motor : idList)
    {
        SetControlMode(motor, DISPLACEMENT);
        SetPoint(motor, 10);
    }

    // done recording

    if (filepath.empty())
    {
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
    if (outfile.is_open())
    {
        ss << "<?xml version=\"1.0\" ?>"
           << std::endl;
        uint m = 0;
        char motorname[10];
        ss << "<behavior>" << std::endl;
        for (uint m = 0; m < idList.size(); m++)
        {
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
    return elapsedTime / (double)sample * 1000.0f;
}

void CanBusControl::StopRecordTrajectories()
{
    recording = false;
    ROS_INFO("Stopped recording a trajectory");
}

void CanBusControl::SetReplay(bool status)
{
    replay = status;
    if (replay)
    {
        ROS_INFO("Replaying trajectories enabled");
    }
    else
    {
        ROS_INFO("Replaying trajectories disabled");
    }
}

bool CanBusControl::PlayTrajectory(const char *file)
{

    TiXmlDocument doc(file);
    if (!doc.LoadFile())
    {
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

         trajectory_it = trajectory_it->NextSiblingElement("trajectory"))
    {

        if (trajectory_it->QueryIntAttribute("samplingTime", &samplingTime) == TIXML_SUCCESS)
        {
            int motor;
            if (trajectory_it->QueryIntAttribute("motorid", &motor) != TIXML_SUCCESS)
            {
                ROS_ERROR("no motorid found");
                return false;
            }
            TiXmlElement *waypointlist_it = trajectory_it->FirstChildElement("waypointlist");
            stringstream stream(waypointlist_it->GetText());
            while (1)
            {
                int n;
                stream >> n;
                trajectories[motor].push_back(n);
                if (!stream)
                {
                    numberOfSamples = trajectories[motor].size();
                    break;
                }
            }
        }
        else
        {
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
    do
    {
        dt = elapsedTime;
        for (auto &motor : trajectories)
        {
            if (sample == 0)
            {
                SetControlMode(motor.first, ENCODER0_POSITION);
            }

            SetPoint(motor.first, motor.second[sample]);
        }
        sample++;
        rate.sleep();
    } while (sample < numberOfSamples && replay);

    return true;
}

void CanBusControl::SetPredisplacement(int value)
{
    predisplacement = value;
    ROS_INFO_STREAM("Now recording with displacement" + predisplacement);
}

void CanBusControl::EstimateSpringParameters(int motor, int degree, vector<float> &coeffs, int timeout,
                                             uint numberOfDataPoints, float displacement_min,
                                             float displacement_max, vector<double> &load, vector<double> &displacement)
{
    SetPoint(motor, 0);
    SetControlMode(motor, DISPLACEMENT);
    milliseconds ms_start = duration_cast<milliseconds>(system_clock::now().time_since_epoch()), ms_stop, t0, t1;
    ofstream outfile;
    char str[100];
    sprintf(str, "springParameters_calibration_motor%d.csv", motor);
    outfile.open(str);
    if (!outfile.is_open())
    {
        cout << "could not open file " << str << " for writing, aborting!" << endl;
        return;
    }
    outfile << "displacement[ticks], load[N]" << endl;
    do
    {
        float f = (rand() / (float)RAND_MAX) * (displacement_max - displacement_min) + displacement_min;
        SetPoint(motor, f);
        t0 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        do
        { // wait a bit until force is applied
            // update control
            t1 = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        } while ((t1 - t0).count() < 1000);

        // note the weight
        // load.push_back(GetWeight(0)); // TODO: use a different load_cell for each motor
        // note the force
        displacement.push_back(GetEncoderPosition(motor, ENCODER1_POSITION));
        outfile << displacement.back() << ", " << load.back() << endl;
        ms_stop = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        cout << "setPoint: \t" << f << "\tdisplacement:\t" << displacement.back() << "\tload:\t" << load.back() << endl;
    } while ((ms_stop - ms_start).count() < timeout && load.size() < numberOfDataPoints);
    SetPoint(motor, 0);
    PolynomialRegression(degree, displacement, load, coeffs);
    outfile << "regression coefficients for polynomial of " << degree << " degree:" << endl;
    for (float coef : coeffs)
    {
        outfile << coef << "\t";
    }
    outfile << endl;
    //	polyPar[motor] = coeffs;
    outfile.close();
}

void CanBusControl::PolynomialRegression(int degree, vector<double> &x, vector<double> &y,
                                         vector<float> &coeffs)
{
    int N = x.size(), i, j, k;
    double X[2 * degree +
             1]; // Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    for (i = 0; i < 2 * degree + 1; i++)
    {
        X[i] = 0;
        for (j = 0; j < N; j++)
            X[i] = X[i] + pow(x[j],
                              i); // consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
    double B[degree + 1][degree + 2], a[degree +
                                        1]; // B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    for (i = 0; i <= degree; i++)
        for (j = 0; j <= degree; j++)
            B[i][j] = X[i +
                        j]; // Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    double Y[degree +
             1]; // Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^degree*yi)
    for (i = 0; i < degree + 1; i++)
    {
        Y[i] = 0;
        for (j = 0; j < N; j++)
            Y[i] = Y[i] + pow(x[j], i) *
                              y[j]; // consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i = 0; i <= degree; i++)
        B[i][degree +
             1] = Y[i]; // load the values of Y as the last column of B(Normal Matrix but augmented)
    degree = degree +
             1; // degree is made degree+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
    for (i = 0; i <
                degree;
         i++) // From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k = i + 1; k < degree; k++)
            if (B[i][i] < B[k][i])
                for (j = 0; j <= degree; j++)
                {
                    double temp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = temp;
                }

    for (i = 0; i < degree - 1; i++) // loop to perform the gauss elimination
        for (k = i + 1; k < degree; k++)
        {
            double t = B[k][i] / B[i][i];
            for (j = 0; j <= degree; j++)
                B[k][j] = B[k][j] - t *
                                        B[i][j]; // make the elements below the pivot elements equal to zero or elimnate the variables
        }
    for (i = degree - 1; i >= 0; i--) // back-substitution
    {                                 // x is an array whose values correspond to the values of x,y,z..
        a[i] = B[i][degree];          // make the variable to be calculated equal to the rhs of the last equation
        for (j = 0; j < degree; j++)
            if (j !=
                i) // then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i] = a[i] - B[i][j] * a[j];
        a[i] = a[i] /
               B[i][i]; // now finally divide the rhs by the coefficient of the variable to be calculated
    }
    for (i = 0; i < degree; i++)
        coeffs.push_back(a[i]); // the values of x^0,x^1,x^2,x^3,....
}
