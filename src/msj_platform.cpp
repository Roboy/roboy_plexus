/*
    BSD 3-Clause License

    Copyright (c) 2017, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Simon Trendel ( simon.trendel@tum.de ), 2018
    description: msj platform main
*/

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdlib.h>
#include "hwlib.h"
#include "socal/hps.h"
#include "roboy_plexus/msj_platform_hps.h"
#include "roboy_plexus/msj_platform.hpp"

using namespace std;

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

MSJPlatform::MSJPlatform(int32_t *msj_platform_base, int32_t *switch_base, vector<int32_t *> i2c_base):
        msj_platform_base(msj_platform_base), switch_base(switch_base), i2c_base(i2c_base){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "msj_platform_fpga");
        ros::start();
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    motor_status = nh->advertise<roboy_middleware_msgs::MotorStatus>("/roboy/middleware/MotorStatus",1);
    magnet_status = nh->advertise<roboy_middleware_msgs::MagneticSensor>("/roboy/middleware/MagneticSensor",1);
    motor_command = nh->subscribe("/roboy/middleware/MotorCommand", 1, &MSJPlatform::MotorCommand, this);

    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();

    for(int i=0;i<NUMBER_OF_MOTORS;i++){
        MSJ_WRITE_zero_speed(msj_platform_base,i,zero_speed[i]);
        MSJ_WRITE_outputPosMax(msj_platform_base,i,(zero_speed[i]+30));
        MSJ_WRITE_outputNegMax(msj_platform_base,i,(zero_speed[i]-30));
        if(i==7){
            MSJ_WRITE_Kp(msj_platform_base,i,-20);
            MSJ_WRITE_Kd(msj_platform_base,i,-10);
        }else{
            MSJ_WRITE_Kp(msj_platform_base,i,20);
            MSJ_WRITE_Kd(msj_platform_base,i,10);
        }

        MSJ_WRITE_outputDivider(msj_platform_base,i,6);
        MSJ_WRITE_deadBand(msj_platform_base,i,0);
        MSJ_WRITE_control_mode(msj_platform_base,i,0);
        MSJ_WRITE_sp(msj_platform_base,i,0);
    }
    MSJ_WRITE_reset_control(msj_platform_base,true);

    status_thread = boost::shared_ptr<std::thread>( new std::thread(&MSJPlatform::publishStatus, this));
    status_thread->detach();

    for(int i=0;i<i2c_base.size();i++){
        boost::shared_ptr<TLV493D> t;
        t.reset(new TLV493D(i2c_base[i]));
        tlv.push_back(t);
    }
    if(!i2c_base.empty()){
        tlv_thread = boost::shared_ptr<std::thread>( new std::thread(&MSJPlatform::publishTLV, this));
        tlv_thread->detach();
    }
}

void MSJPlatform::publishStatus(){
    ros::Rate r(100);
    while(ros::ok()){
        roboy_middleware_msgs::MotorStatus msg;
        msg.id = 0;
        msg.power_sense = !(IORD(switch_base,0)&0x1);
        for(int i=0; i<NUMBER_OF_MOTORS; i++){
            int32_t pwm = MSJ_READ_dutys(msj_platform_base,i);
            int32_t angle = MSJ_READ_sensor_angle_absolute(msj_platform_base,i);
            int32_t vel = MSJ_READ_sensor_angle_velocity(msj_platform_base,i);

            msg.pwmRef.push_back(pwm);
            msg.position.push_back(angle);
            msg.velocity.push_back(vel);
            msg.current.push_back(0);
            msg.displacement.push_back(0);
            msg.angle.push_back(angle);
        }
        motor_status.publish(msg);
        r.sleep();
//        stringstream str;
//        str << "angle           : " << MSJ_READ_sensor_angle(msj_platform_base,5) << endl;
//        str << "angle absolute  : " << MSJ_READ_sensor_angle_absolute(msj_platform_base,5) << endl;
//        str << "angle offset    : " << MSJ_READ_sensor_angle_offset(msj_platform_base,5) << endl;
//        str << "angle relative  : " << MSJ_READ_sensor_angle_relative(msj_platform_base,5) << endl;
//        str << "angle velocity  : " << MSJ_READ_sensor_angle_velocity(msj_platform_base,5) << endl;
//        str << "angle revolution: " << MSJ_READ_sensor_revolution_counter(msj_platform_base,5) << endl;
//        str << "dutys           : " << MSJ_READ_dutys(msj_platform_base,5) << endl;
//        str << "sp              : " << MSJ_READ_sp(msj_platform_base,5) << endl;
//        str << "control mode    : " << MSJ_READ_control_mode(msj_platform_base,5) << endl;
//
//        ROS_INFO_STREAM_THROTTLE(1,str.str());
    }
}

void MSJPlatform::publishTLV(){
    ros::Rate rate(60);
    while (ros::ok()) {
        roboy_middleware_msgs::MagneticSensor msg;

        float fx,fy,fz;
        for(int i=0;i<tlv.size();i++){
            ros::Time start_time = ros::Time::now();
            bool success = false;
            do{
                success = tlv[i]->read(fx,fy,fz);
                if(success) {
                    msg.sensor_id.push_back(i);
                    msg.x.push_back(fx);
                    msg.y.push_back(fy);
                    msg.z.push_back(fz);
//                    ROS_INFO("sensor %d %.6f\t%.6f\t%.6f", i, fx, fy, fz);
                }
            }while(!success && (ros::Time::now()-start_time).toSec()<0.1);
        }
        if(msg.sensor_id.size()==tlv.size())
            magnet_status.publish(msg);
        rate.sleep();
    }
}

void MSJPlatform::MotorCommand(const roboy_middleware_msgs::MotorCommandConstPtr &msg){
    if(msg->id!=5) // not for me
        return;
    for(int i=0;i<msg->motors.size();i++) {
//        MSJ_WRITE_control_mode(msj_platform_base, msg->motors[i], 2);
        MSJ_WRITE_sp(msj_platform_base, msg->motors[i], (int)msg->setPoints[i]);
    }
}

int32_t *h2p_lw_led_addr;
int32_t *h2p_lw_adc_addr;
int32_t *h2p_lw_switches_addr;
int32_t *h2p_lw_msj_platform;
vector<int32_t*> h2p_lw_i2c;

void SigintHandler(int sig)
{
    cout << "shutting down" << endl;

    if(h2p_lw_led_addr!=nullptr)
        *h2p_lw_led_addr = 0;

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
    *h2p_lw_led_addr = 0x00;
    system("fortune");
}

int main(int argc, char *argv[]) {
    void *virtual_base;
    int fd;

//     map the address space for all registers into user space so we can interact with them.
//     we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span
    if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
        printf( "ERROR: could not open \"/dev/mem\"...\n" );
        return( 1 );
    }

    virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

    if( virtual_base == MAP_FAILED ) {
        printf( "ERROR: mmap() failed...\n" );
        close( fd );
        return( 1 );
    }
#ifdef LED_BASE
    h2p_lw_led_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_led_addr = nullptr;
#endif
#ifdef SWITCHES_BASE
    h2p_lw_switches_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SWITCHES_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_switches_addr = nullptr;
#endif
#ifdef MSJPLATFORMCONTROLLER_0_BASE
    h2p_lw_msj_platform = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MSJPLATFORMCONTROLLER_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_msj_platform = nullptr;
#endif
#ifdef I2C_0_BASE
    h2p_lw_i2c.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c.push_back(nullptr);
#endif
#ifdef I2C_1_BASE
    h2p_lw_i2c.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c.push_back(nullptr);
#endif
#ifdef I2C_2_BASE
    h2p_lw_i2c.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c.push_back(nullptr);
#endif

    MSJPlatform msjPlatform(h2p_lw_msj_platform, h2p_lw_switches_addr, h2p_lw_i2c);

    uint8_t mask = 0x1;
    ros::Rate rate(30);
    bool dir = 1;
    while(ros::ok()){
        if(dir)
            mask<<=1;
        else
            mask>>=1;
        *h2p_lw_led_addr = mask;
        rate.sleep();
        if(mask==0x80)
            dir = 0;
        if(mask==0x1)
            dir = 1;
    }

    // clean up our memory mapping and exit
    if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
        printf( "ERROR: munmap() failed...\n" );
        close( fd );
        return( 1 );
    }

    close( fd );

    return( 0 );
}