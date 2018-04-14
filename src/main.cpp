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
#include "hwlib.h"
#include "socal/hps.h"
#include "roboy_plexus/hps_0.h"
#include "roboy_plexus/roboyPlexus.hpp"

using namespace std;

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int main(int argc, char *argv[]) {
    void *virtual_base;
    int fd;
    int32_t *h2p_lw_led_addr;
    int32_t *h2p_lw_adc_addr;
    int32_t *h2p_lw_switches_addr;
    int32_t *h2p_lw_darkroom_addr;
    vector<int32_t*> h2p_lw_darkroom_ootx_addr;
    vector<int32_t*> h2p_lw_myo_addr;
    vector<int32_t*> h2p_lw_i2c_addr;

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

    h2p_lw_led_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_switches_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SWITCHES_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_3_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_4_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_darkroom_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DARKROOM_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_darkroom_ootx_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DARKROOMOOTXDECODER_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_adc_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_LTC2308_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy_plexus");
        ros::start();
    }
//
//    I2C i2c(h2p_lw_i2c_addr[4]);
//    vector<uint8_t> active_devices;
//    i2c.checkAddressSpace(0,127,active_devices);
//    ROS_INFO("found %ld active devices", active_devices.size());
//    for(auto device:active_devices)
//        printf("%x\t",device);
//    cout << endl;
//
//    vector<uint8_t> deviceIDs = {0xC,0xD,0xE,0xF};
//    A1335 motorAngle(h2p_lw_i2c_addr[4],deviceIDs);
//    while(ros::ok()) {
//        vector<A1335State> state;
//        motorAngle.readAngleData(state);
//        stringstream str;
//        str<<endl;
//        for (auto s:state) {
//            str << "Motor Angle Sensor on i2C address " << (int) s.address << " is " << (s.isOK ? "ok" : "not ok")
//                << endl;
//            str << "angle:         " << s.angle << endl;
//            str << "angle_flags:   " << motorAngle.decodeFlag(s.angle_flags, ANGLES_FLAGS) << endl;
//            str << "err_flags:     " << motorAngle.decodeFlag(s.err_flags, ERROR_FLAGS) << endl;
//            str << "fieldStrength: " << s.fieldStrength << endl;
//            str << "status_flags:  " << motorAngle.decodeFlag(s.status_flags, STATUS_FLAGS) << endl;
//            str << "xerr_flags:    " << motorAngle.decodeFlag(s.xerr_flags, XERROR_FLAGS) << endl;
////            msg.temperature.push_back(s.temp);
//        }
//        ROS_INFO_STREAM_THROTTLE(1,str.str());
//    }




////
//    vector<uint8_t> deviceIDs = {0x50, 0x51, 0x52, 0x53};
//    HandControl handControl(h2p_lw_i2c_addr[4],deviceIDs);
//    handControl.test();

    MyoControlPtr myoControl = MyoControlPtr(new MyoControl(h2p_lw_myo_addr,h2p_lw_adc_addr));
////
    RoboyPlexus roboyPlexus(myoControl, h2p_lw_myo_addr, h2p_lw_i2c_addr, h2p_lw_darkroom_addr,
                            h2p_lw_darkroom_ootx_addr, h2p_lw_adc_addr, h2p_lw_switches_addr);
//
    PerformMovementAction performMovementAction(myoControl, roboyPlexus.getBodyPart() + "_movement_server");
    PerformMovementsAction performMovementsAction(myoControl, roboyPlexus.getBodyPart() + "_movements_server");

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



////    I2C i2c(h2p_lw_i2c_addr[0]);
////    vector<uint8_t> data;
////    i2c.read(LSM9DS1_ADDRESS_ACCELGYRO,0x80 | 0x18,6,data);
////    for(auto d:data)
////        printf("%x\t",d);
////    printf("\n");
////
////
////    Adafruit_LSM9DS1 lsm(0,h2p_lw_i2c_addr[0]);
////    while(!lsm.begin()){
////        usleep(100);
////        ROS_WARN_THROTTLE(1, "no lsm detected");
////    }
////
////    // 1.) Set the accelerometer range
////    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
////    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
////    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
////    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
////
////    // 2.) Set the magnetometer sensitivity
////    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
////    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
////    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
////    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
////
////    // 3.) Setup the gyroscope
//////    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
////    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
////    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
////
////    ros::Rate rate(10);
////    while(ros::ok()){
////        lsm.read();  /* ask it to read in the data */
////
////        /* Get a new sensor event */
////        sensors_event_t a, m, g, temp;
////
////        lsm.getEvent(&a, &m, &g, &temp);
////
////        ROS_INFO("%f\t%f\t%f\t%f\t%f\t%f", a.acceleration.x, a.acceleration.y, a.acceleration.z,
////                 g.gyro.x, g.gyro.y, g.gyro.z, temp.temperature);
////        rate.sleep();
////    }

    // clean up our memory mapping and exit
    if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
        printf( "ERROR: munmap() failed...\n" );
        close( fd );
        return( 1 );
    }

    close( fd );

    return( 0 );
}