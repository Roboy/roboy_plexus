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

    author: Simon Trendel ( simon.trendel@tum.de ), 2019
    description: bldc_testbed
*/

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdlib.h>
#include "hwlib.h"
#include "socal/hps.h"
#include "roboy_plexus/hps_0.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <roboy_plexus/A1335.hpp>
#include <std_srvs/Trigger.h>
#include <iostream>
#include <vector>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort

using namespace std;
#define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)
#define NUMBER_OF_ADC_SAMPLES 50

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int32_t *h2p_lw_sysid_addr;
int32_t *h2p_lw_adc_addr;
int32_t *h2p_lw_pwm_addr;
int32_t *h2p_lw_led_addr;
int32_t *h2p_lw_switches_addr;
vector<int32_t *> h2p_lw_i2c_addr;
vector<int32_t *> h2p_lw_tli_addr;

int commute_state = 0;
const int INHA = 0, INLA = 1, INHB = 2, INLB = 3, INHC = 4, INLC = 5;
int motor_position = 0, motor_position_prev = 0, motor_pos_absolute = 0, motor_offset = 0, rev_counter = 0, pwm = 3000;
float current[3] = {0,0};
vector<size_t> commute_state_transition = {0,1,2,3,4,5}, cw_transition = {0,1,2,3,4,5}, ccw_transition = {5,4,3,2,1,0};

const int TLI4970_CURRENT_OFFSET = 4096;
const int TLI4970_CURRENT_DIVIDER = 80;

void SigintHandler(int sig)
{
    cout << "shutting down" << endl;

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
    *h2p_lw_led_addr = 0x00;
    system("fortune");
}

template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

    // initialize original index locations
    vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});

    return idx;
}

void control_all_mosfet(int pwm){
    for(int i=0;i<6;i++)
        IOWR(h2p_lw_pwm_addr, i, pwm);
}

void control_mosfet(int number, int pwm){
    IOWR(h2p_lw_pwm_addr, number, pwm);
}

void commute_state_cb(const std_msgs::Int32ConstPtr &msg){
    ROS_INFO("changing commute state %d -> %d", commute_state, msg->data);
    commute_state = msg->data;

}

bool auto_phase_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    ROS_INFO("-----------------------------\n\nauto_phase called\n\n-----------------------------");
    ros::Time t0, t1;
    vector<int> pos;
    pos.resize(6);
    for(int i=0;i<6;i++){
        t0 = ros::Time::now();
        commute_state = i;
        ROS_INFO("running commute_state %d",i);
        ros::Duration delay(0.5);
        delay.sleep();
        pos[i] = motor_pos_absolute;
        while((ros::Time::now()-t0).toSec()<1){
            pos[i] = pos[i]*0.9+0.1*motor_pos_absolute;
        }
    }

    auto commute_state_transition = sort_indexes<int>(pos);
    motor_offset = pos[commute_state_transition[0]];
    stringstream str;
    str << "commute_state_transition: ";
    int phase_increment = 0;
    int j = 0;
    for(auto v:commute_state_transition) {
        if(j<5)
            phase_increment += (pos[v+1]-pos[v]);
        str << v << " " << phase_increment << " ";
        j++;
    }
    phase_increment/=6;
    str << endl;
    str << "average motor pos:" << endl;
    for(auto v:pos) {
        str << v - motor_offset << "\n";
    }

    str << "phase increment " << phase_increment;
    if(phase_increment<0){
        ccw_transition = commute_state_transition;
        for(int i=0;i<6;i++){
            cw_transition[i] = commute_state_transition[5-i];
        }
    }else{
        cw_transition = commute_state_transition;
        for(int i=0;i<6;i++){
            ccw_transition[i] = commute_state_transition[5-i];
        }
    }


    ROS_INFO_STREAM(str.str());
    ROS_INFO("running on full revolution now...");
    int start_pos =  motor_pos_absolute;
    int i = 0;
    ros::Duration delay(0.1);
    t0 = ros::Time::now();
    do{
        commute_state = cw_transition[i++];
//        ROS_INFO("running commute_state %d",i);
        delay.sleep();
        if(i>5)
            i = 0;
    }while((motor_pos_absolute-start_pos)<4096 && ((ros::Time::now()-t0).toSec()<10));
    ROS_INFO("done");
    return true;
}

uint32_t readADC(int load_cell = 0) {
    // start measure
    IOWR(h2p_lw_adc_addr, 0x00, (load_cell << 1) | 0x00);
    IOWR(h2p_lw_adc_addr, 0x00, (load_cell << 1) | 0x01);
    IOWR(h2p_lw_adc_addr, 0x00, (load_cell << 1) | 0x00);
    usleep(1);
    // wait measure done
    while ((IORD(h2p_lw_adc_addr, 0x00) & 0x01) == 0x00);
    // read adc value
    uint32_t adc_value = 0;
    uint sample = 0;
    while (sample < NUMBER_OF_ADC_SAMPLES) {
        uint32_t val = IORD(h2p_lw_adc_addr, 0x01);
        if (val > 0) {
            sample++;
            adc_value += val;
//            printf("CH%d=%.3fV (0x%04x)\r\n", load_cell, (float)adc_value/1000.0, adc_value);
        } else {
            // start measure
            IOWR(h2p_lw_adc_addr, 0x00, (load_cell << 1) | 0x00);
            IOWR(h2p_lw_adc_addr, 0x00, (load_cell << 1) | 0x01);
            IOWR(h2p_lw_adc_addr, 0x00, (load_cell << 1) | 0x00);
            usleep(1);
            // wait measure done
            while ((IORD(h2p_lw_adc_addr, 0x00) & 0x01) == 0x00);
        }
    }

    adc_value /= NUMBER_OF_ADC_SAMPLES;

    return adc_value;
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

    h2p_lw_sysid_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SYSID_QSYS_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    if(*h2p_lw_sysid_addr!=0x0001beef){ // if the system id does not match, we abort
        ROS_ERROR("system id %x does not match this version of plexus %x, make sure you loaded the correct fpga image",*h2p_lw_sysid_addr, 0x0001beef);
        // clean up our memory mapping and exit
        if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
            printf( "ERROR: munmap() failed...\n" );
            close( fd );
            return( 1 );
        }
        close( fd );
        return -1;
    }

    h2p_lw_switches_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SWITCHES_BASE ) & ( unsigned long)( HW_REGS_MASK )) );

    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));

    h2p_lw_pwm_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );

    h2p_lw_adc_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_LTC2308_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );

    h2p_lw_tli_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + TLI_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_tli_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + TLI_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_tli_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + TLI_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));


    control_all_mosfet(0);

    ros::init(argc, argv, "bldc_testbed", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

//    ros::Publisher adc_pub = nh.advertise<std_msgs::Int32>("adc",1);
    ros::Publisher motor_pos_pub = nh.advertise<std_msgs::Float32>("pos",1);
    ros::Publisher current_0_pub = nh.advertise<std_msgs::Float32>("current_0",1);
    ros::Publisher current_1_pub = nh.advertise<std_msgs::Float32>("current_1",1);
    ros::Publisher current_2_pub = nh.advertise<std_msgs::Float32>("current_2",1);
    ros::Subscriber commute_sub = nh.subscribe("commute_state",1,commute_state_cb);
    ros::ServiceServer auto_phase = nh.advertiseService("auto_phase",auto_phase_cb);

    signal(SIGINT, SigintHandler);

    vector<uint8_t> deviceIDs = {0xD};

    A1335 pos(h2p_lw_i2c_addr[0],deviceIDs);

    ros::Rate rate(10);
    std_msgs::Float32 msg;

    int commute_state_prev = 0;

    while(ros::ok()){

//        int32_t adc_value = readADC(0);
//        msg.data = adc_value;
//        adc_pub.publish(msg);
        vector<A1335State> state;
        pos.readAngleData(state);
        motor_position = state[0].angle_raw;
        if(motor_position_prev<1000 && motor_position > 3096)
            rev_counter--;
        if(motor_position_prev>3096 && motor_position < 1000)
            rev_counter++;

        motor_pos_absolute = rev_counter*4096 + motor_position - motor_offset;
        motor_position_prev = motor_position;

        msg.data = motor_pos_absolute;
        motor_pos_pub.publish(msg);
        current[0] = (IORD(h2p_lw_tli_addr[0],0)-4096)/80;
        msg.data = current[0];
        current_0_pub.publish(msg);
        current[1] = (IORD(h2p_lw_tli_addr[1],0)-4096)/80;
        msg.data = current[1];
        current_1_pub.publish(msg);
        current[2] = (IORD(h2p_lw_tli_addr[2],0)-4096)/80;
        msg.data = current[2];
        current_2_pub.publish(msg);

//        rate.sleep();
        if(commute_state_prev!=commute_state){
            commute_state_prev = commute_state;
            control_all_mosfet(0);
            // dead time, to make sure that mosfets are off
            ros::Duration delay(0.1);
        }

        switch(commute_state){
            case 0: // B -> C
                control_mosfet(INHA,0);
                control_mosfet(INLA,0);
                control_mosfet(INHB,pwm);
                control_mosfet(INLB,0);
                control_mosfet(INHC,0);
                control_mosfet(INLC,32767);
                break;
            case 1: // A -> C
                control_mosfet(INHA,pwm);
                control_mosfet(INLA,0);
                control_mosfet(INHB,0);
                control_mosfet(INLB,0);
                control_mosfet(INHC,0);
                control_mosfet(INLC,32767);
                break;
            case 2: // A -> B
                control_mosfet(INHA,pwm);
                control_mosfet(INLA,0);
                control_mosfet(INHB,0);
                control_mosfet(INLB,32767);
                control_mosfet(INHC,0);
                control_mosfet(INLC,0);
                break;
            case 3: // C -> B
                control_mosfet(INHA,0);
                control_mosfet(INLA,0);
                control_mosfet(INHB,0);
                control_mosfet(INLB,32767);
                control_mosfet(INHC,pwm);
                control_mosfet(INLC,0);
                break;
            case 4: // C -> A
                control_mosfet(INHA,0);
                control_mosfet(INLA,32767);
                control_mosfet(INHB,0);
                control_mosfet(INLB,0);
                control_mosfet(INHC,pwm);
                control_mosfet(INLC,0);
                break;
            case 5: // B -> A
                control_mosfet(INHA,0);
                control_mosfet(INLA,32767);
                control_mosfet(INHB,pwm);
                control_mosfet(INLB,0);
                control_mosfet(INHC,0);
                control_mosfet(INLC,0);
                break;
            default:
                control_all_mosfet(0);
        }

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