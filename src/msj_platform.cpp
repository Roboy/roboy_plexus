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

MSJPlatform::MSJPlatform(int32_t *msj_platform_base, int32_t *switch_base, vector<int32_t*> i2c_base, int32_t*darkroom_base, int32_t*darkroom_ootx_base):
        msj_platform_base(msj_platform_base),switch_base(switch_base),i2c_base(i2c_base),darkroom_base(darkroom_base),darkroom_ootx_base(darkroom_ootx_base){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "msj_platform_fpga");
        ros::start();
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    motor_status = nh->advertise<roboy_middleware_msgs::MotorStatus>("/roboy/middleware/MotorStatus",1);
    magnetic_sensor = nh->advertise<roboy_middleware_msgs::MagneticSensor>("/roboy/middleware/MagneticSensor",1);
    motor_command = nh->subscribe("/roboy/middleware/MotorCommand", 10, &MSJPlatform::MotorCommand, this);
    emergency_stop = nh->advertiseService("/msj_platform/emergency_stop",&MSJPlatform::EmergencyStop, this);
    zero = nh->advertiseService("/msj_platform/zero",&MSJPlatform::Zero, this);

    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();

    for(int i=0;i<NUMBER_OF_MOTORS;i++){
        MSJ_WRITE_zero_speed(msj_platform_base,i,zero_speed[i]);
        MSJ_WRITE_outputPosMax(msj_platform_base,i,(zero_speed[i]+30));
        MSJ_WRITE_outputNegMax(msj_platform_base,i,(zero_speed[i]-30));

        MSJ_WRITE_Kp(msj_platform_base,i,10);
        MSJ_WRITE_Kd(msj_platform_base,i,0);

        MSJ_WRITE_outputDivider(msj_platform_base,i,1);
        MSJ_WRITE_deadBand(msj_platform_base,i,0);
        MSJ_WRITE_control_mode(msj_platform_base,i,0);
        MSJ_WRITE_sp(msj_platform_base,i,0);
    }
    MSJ_WRITE_pwm_mute(msj_platform_base,false);

    status_thread = boost::shared_ptr<std::thread>( new std::thread(&MSJPlatform::publishStatus, this));
    status_thread->detach();

    for(int i=0;i<i2c_base.size();i++){
        tlv.push_back(boost::shared_ptr<TLV493D>(new TLV493D(i2c_base[i])));
    }

    magnetic_thread = boost::shared_ptr<std::thread>( new std::thread(&MSJPlatform::publishMagneticSensors, this));
    magnetic_thread->detach();

//    darkroom_pub = nh->advertise<roboy_middleware_msgs::DarkRoom>("/roboy/middleware/DarkRoom/sensors",
//                                                                  1);
//    darkroom_ootx_pub = nh->advertise<roboy_middleware_msgs::DarkRoomOOTX>(
//            "/roboy/middleware/DarkRoom/ootx", 1);
//    darkroom_status_pub = nh->advertise<roboy_middleware_msgs::DarkRoomStatus>(
//            "/roboy/middleware/DarkRoom/status", 1);
//    darkroom_thread = boost::shared_ptr<std::thread>(new std::thread(&MSJPlatform::darkRoomPublisher, this));
//    darkroom_thread->detach();
//    darkroom_ootx_thread = boost::shared_ptr<std::thread>(
//            new std::thread(&MSJPlatform::darkRoomOOTXPublisher, this));
//    darkroom_ootx_thread->detach();
}

void MSJPlatform::publishStatus(){
    ros::Rate r(100);
    while(ros::ok()){
        roboy_middleware_msgs::MotorStatus msg;
        msg.id = 5;
        msg.power_sense = !(IORD(switch_base,0)&0x1);
        for(int i=0; i<NUMBER_OF_MOTORS; i++){
            int32_t pwm = MSJ_READ_dutys(msj_platform_base,i);
            int32_t angle = MSJ_READ_sensor_angle_absolute(msj_platform_base,i);
            int32_t vel = MSJ_READ_sensor_angle_velocity(msj_platform_base,i);

            msg.pwm_ref.push_back(pwm);
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

void MSJPlatform::publishMagneticSensors() {
    ros::Rate rate(200);
    while (ros::ok()) {
        roboy_middleware_msgs::MagneticSensor msg;
        float fx,fy,fz;
        for(int i=0;i<tlv.size();i++){
            bool success = tlv[i]->read(fx,fy,fz);
            if(success) {
                ROS_WARN_THROTTLE(5,"oh oh, magnetic sensor values invalid");
            }
            msg.sensor_id.push_back(i);
            msg.x.push_back(fx);
            msg.y.push_back(fy);
            msg.z.push_back(fz);
        }
        magnetic_sensor.publish(msg);
        rate.sleep();
    }
}

void MSJPlatform::MotorCommand(const roboy_middleware_msgs::MotorCommandConstPtr &msg){
    if(msg->id!=5) // not for me
        return;
    ROS_INFO("receiving motor commands");
    for(int i=0;i<msg->motors.size();i++) {
//        MSJ_WRITE_control_mode(msj_platform_base, msg->motors[i], 2);
        MSJ_WRITE_sp(msj_platform_base, msg->motors[i], (int)msg->set_points[i]);
        ROS_INFO_STREAM("motor " << msg->motors[i] << " " << msg->set_points[i]);
    }
}

bool MSJPlatform::EmergencyStop(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    if(req.data==1){
        MSJ_WRITE_pwm_mute(msj_platform_base,true);
    }else{
        MSJ_WRITE_pwm_mute(msj_platform_base,false);
    }
    return true;
}

bool MSJPlatform::Zero(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    MSJ_WRITE_reset_control(msj_platform_base,true);
    return true;
}

void MSJPlatform::darkRoomPublisher() {
    ros::Rate rate(240);
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while (ros::ok()) {
        uint active_sensors = 0;
        roboy_middleware_msgs::DarkRoom msg;
        msg.object_id = ethaddr;
        for (uint i = 0; i < NUM_SENSORS; i++) {
            int32_t val = IORD(darkroom_base, i);
            high_resolution_clock::time_point t1 = high_resolution_clock::now();
            microseconds time_span = duration_cast<microseconds>(t1 - t0);
            msg.timestamp.push_back(time_span.count());
            msg.sensor_value.push_back(val);
            if ((val >> 29) & 0x1)//valid
                active_sensors++;
        }
        ROS_INFO_THROTTLE(5,"lighthouse sensors active %d/%d", active_sensors, NUM_SENSORS);
        darkroom_pub.publish(msg);
        rate.sleep();
    }
}

void MSJPlatform::darkRoomOOTXPublisher() {
    // ootx frames are 17bits preamble + 271 bits payload + 32 bits checksum = 320 bits.
    // At 120Hz motor speed, this is ~2.6 seconds per frame. Lets set the update rate to 5 seconds then.
    ros::Rate rate(1 / 5.0);
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while (ros::ok()) {
        bool successfully_decoded_ootx = true;
        for (uint lighthouse = 0; lighthouse < 2; lighthouse++) {
            // TODO: remove reverse and have the fpga convert it directly
            ootx.frame.fw_version = (uint16_t) (
                    reverse(IORD(darkroom_ootx_base, (uint32_t) (18 * lighthouse + 0))) & 0xFFFF);
            ootx.frame.ID = reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 1));
            ootx.frame.fcal_0_phase = (uint16_t) (reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 2)) &
                                                  0xFFFF);
            ootx.frame.fcal_1_phase = (uint16_t) (reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 3)) &
                                                  0xFFFF);
            ootx.frame.fcal_0_tilt = (uint16_t) (reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 4)) &
                                                 0xFFFF);
            ootx.frame.fcal_1_tilt = (uint16_t) (reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 5)) &
                                                 0xFFFF);
            ootx.frame.unlock_count = (uint8_t) (reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 6)) &
                                                 0xFF);
            ootx.frame.hw_version = (uint8_t) (reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 7)) &
                                               0xFF);
            ootx.frame.fcal_0_curve = (uint16_t) (reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 8)) &
                                                  0xFFFF);
            ootx.frame.fcal_1_curve = (uint16_t) (reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 9)) &
                                                  0xFFFF);
            uint32_t acc = reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 10));
            ootx.frame.accel_dir_x = ((int8_t) (acc >> 0 & 0xFF));
            ootx.frame.accel_dir_y = ((int8_t) (acc >> 8 & 0xFF));
            ootx.frame.accel_dir_z = ((int8_t) (acc >> 16 & 0xFF));
            ootx.frame.fcal_0_gibphase = (uint16_t) (
                    reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 11)) & 0xFFFF);
            ootx.frame.fcal_1_gibphase = (uint16_t) (
                    reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 12)) & 0xFFFF);
            ootx.frame.fcal_0_gibmag = (uint16_t) (
                    reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 13)) & 0xFFFF);
            ootx.frame.fcal_1_gibmag = (uint16_t) (
                    reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 14)) & 0xFFFF);
            ootx.frame.mode = (uint8_t) (reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 15)) & 0xFF);
            ootx.frame.faults = (uint8_t) (reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 16)) & 0xFF);

            uint32_t crc32checksum = reverse(IORD(darkroom_ootx_base, 18 * lighthouse + 17));

            CRC32 crc;
            for (int i = 0; i < 33; i++) {
                crc.update(ootx.data[i]);
            }

            uint32_t crc32checksumCalculated = crc.finalize();
            if (crc.finalize() == crc32checksum) { // kinda paranoid right...?!
                roboy_middleware_msgs::DarkRoomOOTX msg;
                msg.lighthouse = lighthouse;
                msg.fw_version = ootx.frame.fw_version;
                msg.id = ootx.frame.ID;
                half fcal_0_phase, fcal_1_phase, fcal_0_tilt, fcal_1_tilt;
                fcal_0_phase.data_ = ootx.frame.fcal_0_phase;
                fcal_1_phase.data_ = ootx.frame.fcal_1_phase;
                fcal_0_tilt.data_ = ootx.frame.fcal_0_tilt;
                fcal_1_tilt.data_ = ootx.frame.fcal_1_tilt;
                msg.fcal_0_phase = fcal_0_phase;
                msg.fcal_1_phase = fcal_1_phase;
                msg.fcal_0_tilt = fcal_0_tilt;
                msg.fcal_1_tilt = fcal_1_tilt;
                msg.unlock_count = fcal_1_tilt;
                msg.hw_version = ootx.frame.hw_version;
                half fcal_0_curve, fcal_1_curve;
                fcal_0_curve.data_ = ootx.frame.fcal_0_curve;
                fcal_1_curve.data_ = ootx.frame.fcal_1_curve;
                msg.fcal_0_curve = fcal_0_curve;
                msg.fcal_1_curve = fcal_1_curve;
                // max/min value of acceleration is 127/-127, so we divide to get an orientation vector
                msg.accel_dir_x = ootx.frame.accel_dir_x / 127.0f;
                msg.accel_dir_y = ootx.frame.accel_dir_y / 127.0f;
                msg.accel_dir_z = ootx.frame.accel_dir_z / 127.0f;
                half fcal_0_gibphase, fcal_1_gibphase, fcal_0_gibmag, fcal_1_gibmag;
                fcal_0_gibphase.data_ = ootx.frame.fcal_0_gibphase;
                fcal_1_gibphase.data_ = ootx.frame.fcal_1_gibphase;
                fcal_0_gibmag.data_ = ootx.frame.fcal_0_gibmag;
                fcal_1_gibmag.data_ = ootx.frame.fcal_1_gibmag;
                msg.fcal_0_gibphase = fcal_0_gibphase;
                msg.fcal_1_gibphase = fcal_1_gibphase;
                msg.fcal_0_gibmag = fcal_0_gibmag;
                msg.fcal_1_gibmag = fcal_1_gibmag;
                msg.mode = ootx.frame.mode;
                msg.faults = ootx.frame.faults;
                msg.crc32 = crc32checksum;

                darkroom_ootx_pub.publish(msg);
            } else {
                successfully_decoded_ootx = false;

                stringstream str;
                str << "received ootx frame" << " with crc " << crc32checksum
                    << " which does not match the calculated: " << crc32checksumCalculated << endl;
                str << "fw_version:          " << ootx.frame.fw_version << endl;
                str << "ID:                  " << ootx.frame.ID << endl;
                str << "fcal_0_phase:        " << ootx.frame.fcal_0_phase << endl;
                str << "fcal_1_phase:        " << ootx.frame.fcal_1_phase << endl;
                str << "fcal_0_tilt:         " << ootx.frame.fcal_0_tilt << endl;
                str << "fcal_1_tilt:         " << ootx.frame.fcal_1_tilt << endl;
                str << "unlock_count:        " << (uint) ootx.frame.unlock_count << endl;
                str << "hw_version:          " << (uint) ootx.frame.hw_version << endl;
                str << "fcal_0_curve:        " << ootx.frame.fcal_0_curve << endl;
                str << "fcal_1_curve:        " << ootx.frame.fcal_1_curve << endl;
                str << "accel_dir_x:         " << (int) ootx.frame.accel_dir_x << endl;
                str << "accel_dir_y:         " << (int) ootx.frame.accel_dir_y << endl;
                str << "accel_dir_z:         " << (int) ootx.frame.accel_dir_z << endl;
                str << "fcal_0_gibphase:     " << ootx.frame.fcal_0_gibphase << endl;
                str << "fcal_1_gibphase:     " << ootx.frame.fcal_1_gibphase << endl;
                str << "fcal_0_gibmag:       " << ootx.frame.fcal_0_gibmag << endl;
                str << "fcal_1_gibmag:       " << ootx.frame.fcal_1_gibmag << endl;
                str << "mode:                " << (uint) ootx.frame.mode << endl;
                str << "faults:              " << (uint) ootx.frame.faults << endl;
                ROS_DEBUG_STREAM(str.str());
            }
        }
        if (!successfully_decoded_ootx) {
            // no valid ootx frame decoded, lets switch to another sensor channel
            ootx_sensor_channel++;
            if (ootx_sensor_channel > NUM_SENSORS)
                ootx_sensor_channel = 0;
                IOWR(darkroom_ootx_base, 0, ootx_sensor_channel);
        }

        roboy_middleware_msgs::DarkRoomStatus status_msg;
        status_msg.object_id = ethaddr;
        uint active_sensors = 0;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            int32_t val = IORD(darkroom_base, (1 << 8 | i));
            status_msg.sensor_state.push_back(val & 0xF);
            if (status_msg.sensor_state.back() == 1)
                active_sensors++;
        }
        ROS_INFO("lighthouse sensors active %d/%d", active_sensors, NUM_SENSORS);
        darkroom_status_pub.publish(status_msg);

        rate.sleep();
    }
}

int32_t *h2p_lw_led_addr;
int32_t *h2p_lw_adc_addr;
int32_t *h2p_lw_switches_addr;
int32_t *h2p_lw_msj_platform;
vector<int32_t*> h2p_lw_i2c;
int32_t *h2p_lw_darkroom;
int32_t *h2p_lw_darkroom_ootx;

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
    h2p_lw_msj_platform = nullptr;
#endif
#ifdef I2C_1_BASE
    h2p_lw_i2c.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_msj_platform = nullptr;
#endif
#ifdef I2C_2_BASE
    h2p_lw_i2c.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_msj_platform = nullptr;
#endif
#ifdef DARKROOM_0_BASE
    h2p_lw_darkroom = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DARKROOM_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_darkroom = nullptr;
#endif
#ifdef DARKROOMOOTXDECODER_0_BASE
    h2p_lw_darkroom_ootx = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DARKROOMOOTXDECODER_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_darkroom_ootx = nullptr;
#endif

    MSJPlatform msjPlatform(h2p_lw_msj_platform, h2p_lw_switches_addr, h2p_lw_i2c, h2p_lw_darkroom, h2p_lw_darkroom_ootx);


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