#include "roboy_plexus/xl320.hpp"

XL320::XL320(int32_t* xl320_base):xl320_base(xl320_base){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "xl320_control");
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    motor_command = nh->subscribe("/roboy/middleware/MotorCommand", 1, &XL320::MotorCommandCB, this);
    xl320_srv = nh->advertiseService("/xl320",&XL320::Service, this);
    for(int i=0;i<8;i++)
        write(i,Address::GOAL_POSITION,512);

    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();
}


void XL320::write(uint8_t motor, Address address, int16_t value){
    XL320_write(xl320_base,motor,(int16_t)address,value);
    XL320_write(xl320_base,motor,(int16_t)address,value);
}

void XL320::MotorCommandCB(const roboy_communication_middleware::MotorCommandConstPtr &msg){
    if(msg->id==id){
        for(int i=0;i<msg->motors.size();i++){
            write((uint8_t)msg->motors[i],Address::GOAL_POSITION,(int16_t)msg->setPoints[i]);
        }
    }
}

bool XL320::Service(roboy_communication_middleware::XL320::Request &req,
                   roboy_communication_middleware::XL320::Response &res){
    if(req.type==0){ // READ
        res.value = XL320_read(xl320_base,req.motor,req.address);
    }else{ //WRITE
        XL320_write(xl320_base,req.motor,req.address,req.value);
    }
    return true;
}