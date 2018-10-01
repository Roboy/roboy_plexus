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
    description: main
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
#include "roboy_plexus/xl320.hpp"
#include "roboy_plexus/tlv493d.hpp"
#include <roboy_communication_middleware/MagneticSensor.h>

using namespace std;

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int32_t *xl320_addr, *h2p_lw_led_addr, *h2p_lw_switch_addr, *h2p_lw_i2c0_addr, *h2p_lw_i2c1_addr, *h2p_lw_i2c2_addr;


void SigintHandler(int sig)
{
    cout << "shutting down" << endl;


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

    xl320_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + XL320_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_led_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_i2c0_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_i2c1_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_i2c2_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_switch_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SWITCHES_BASE ) & ( unsigned long)( HW_REGS_MASK )) );

    signal(SIGINT, SigintHandler);

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "magnetic_shoulder", ros::init_options::NoSigintHandler);
    }

    ros::NodeHandle nh;

    XL320 xl320(xl320_addr);

    vector<boost::shared_ptr<TLV493D>> tlv;
    vector<uint8_t> deviceIDs = {0x5e};
    vector<int> pins = {255};
    boost::shared_ptr<TLV493D> t0 = boost::shared_ptr<TLV493D>(new TLV493D(h2p_lw_i2c0_addr,deviceIDs,pins));
    boost::shared_ptr<TLV493D> t1 = boost::shared_ptr<TLV493D>(new TLV493D(h2p_lw_i2c1_addr,deviceIDs,pins));
    boost::shared_ptr<TLV493D> t2 = boost::shared_ptr<TLV493D>(new TLV493D(h2p_lw_i2c2_addr,deviceIDs,pins));
    tlv.push_back(t0);
    tlv.push_back(t1);
    tlv.push_back(t2);

    ros::Publisher magnetic_sensor_pub;
    magnetic_sensor_pub = nh.advertise<roboy_communication_middleware::MagneticSensor>("/roboy/middleware/MagneticSensor",1);

    ros::Rate r(100);
    while(ros::ok()){
        vector<float> x,y,z;
        for(auto t:tlv){
            t->read(x,y,z);
        }
        roboy_communication_middleware::MagneticSensor msg;
        msg.x = x;
        msg.y = y;
        msg.z = z;
        magnetic_sensor_pub.publish(msg);
        ROS_INFO("\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f", x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
        r.sleep();
    }

//    I2C i2c0(h2p_lw_i2c0_addr), i2c1(h2p_lw_i2c1_addr), i2c2(h2p_lw_i2c2_addr);
//    ros::Duration d(0.1);
//    while(ros::ok()) {
////        vector<uint8_t> data;
////        i2c2.read(0x5e,0,1,data);
////        if(i2c0.ack_error())
////            ROS_ERROR("ack");
////        else
////            ROS_INFO("yea");
//        {
//            vector <uint8_t> active_devices;
//            i2c0.checkAddressSpace(0, 127, active_devices);
//            ROS_INFO("i2c 0 found %ld active devices", active_devices.size());
//            for (auto device:active_devices)
//                printf("%x\t", device);
//            cout << endl;
//        }
//        {
//            vector <uint8_t> active_devices;
//            i2c1.checkAddressSpace(0, 127, active_devices);
//            ROS_INFO("i2c 1 found %ld active devices", active_devices.size());
//            for (auto device:active_devices)
//                printf("%x\t", device);
//            cout << endl;
//        }
//        {
//            vector <uint8_t> active_devices;
//            i2c2.checkAddressSpace(0, 127, active_devices);
//            ROS_INFO("i2c 2 found %ld active devices", active_devices.size());
//            for (auto device:active_devices)
//                printf("%x\t", device);
//            cout << endl;
//        }
//        d.sleep();
//    }

    uint8_t mask = 0x1;
    ros::Rate rate(100);
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