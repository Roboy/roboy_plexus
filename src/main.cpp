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
#include "roboy_plexus/roboyPlexus.hpp"

using namespace std;

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int32_t *h2p_lw_led_addr;
int32_t *h2p_lw_adc_addr;
int32_t *h2p_lw_switches_addr;
int32_t *h2p_lw_darkroom_addr;
vector<int32_t*> h2p_lw_darkroom_ootx_addr;
vector<int32_t*> h2p_lw_myo_addr;
vector<int32_t*> h2p_lw_i2c_addr;

MyoControlPtr myoControl;

void SigintHandler(int sig)
{
    cout << "shutting down" << endl;
    if(h2p_lw_myo_addr[0]!=nullptr) {
        MYO_WRITE_elbow_joint_control(h2p_lw_myo_addr[0], 0);
        MYO_WRITE_hand_control(h2p_lw_myo_addr[0], 0);
    }
    if(h2p_lw_myo_addr[1]!=nullptr) {
        MYO_WRITE_elbow_joint_control(h2p_lw_myo_addr[1], 0);
        MYO_WRITE_hand_control(h2p_lw_myo_addr[1], 0);
    }
    if(myoControl!= nullptr){
        // switch to displacement
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            myoControl->changeControl(motor, DISPLACEMENT);
        }
        // relax the springs
        ros::Rate rate(100);
        bool toggle = true;
        for(int decrements = 99; decrements>=0; decrements-=1){
            if(toggle)
                *h2p_lw_led_addr = 0xFF;
            else
                *h2p_lw_led_addr = 0x00;

            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                int displacement = myoControl->getDisplacement(motor);
                if(displacement<=0)
                    continue;
                else
                    myoControl->setDisplacement(motor,displacement*(decrements/100.0));
            }
            rate.sleep();
            if(decrements%15==0) {
                toggle = !toggle;
                cout << "." << endl;
            }
        }
    }
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        myoControl->setDisplacement(motor,0);
    }

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
#ifdef MYOCONTROL_0_BASE
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_myo_addr.push_back(nullptr);
#endif
#ifdef MYOCONTROL_1_BASE
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_myo_addr.push_back(nullptr);
#endif
#ifdef MYOCONTROL_2_BASE
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_myo_addr.push_back(nullptr);
#endif
#ifdef I2C_0_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c_addr.push_back(nullptr);
#endif
#ifdef I2C_1_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c_addr.push_back(nullptr);
#endif
#ifdef I2C_2_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c_addr.push_back(nullptr);
#endif
#ifdef I2C_3_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_3_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_i2c_addr.push_back(nullptr);
#endif
#ifdef DARKROOM_0_BASE
    h2p_lw_darkroom_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DARKROOM_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_darkroom_addr = nullptr;
#endif
#ifdef DARKROOMOOTXDECODER_0_BASE
    h2p_lw_darkroom_ootx_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DARKROOMOOTXDECODER_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
    h2p_lw_darkroom_ootx_addr.push_back(nullptr);
#endif
#ifdef ADC_LTC2308_0_BASE
    h2p_lw_adc_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_LTC2308_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_adc_addr = nullptr;
#endif

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy_plexus");
        ros::start();
    }
//
//    I2C i2c0(h2p_lw_i2c_addr[0]), i2c1(h2p_lw_i2c_addr[1]), i2c(h2p_lw_i2c_addr[2]);
//    ros::Duration d(0.1);
//    while(ros::ok()) {
//        vector<uint8_t> data;
//        i2c2.read(0x5e,0,1,data);
//        if(i2c0.ack_error())
//            ROS_ERROR("ack");
//        else
//            ROS_INFO("yea");
////        {
////            vector <uint8_t> active_devices;
////            i2c0.checkAddressSpace(0, 127, active_devices);
////            ROS_INFO("i2c 0 found %ld active devices", active_devices.size());
////            for (auto device:active_devices)
////                printf("%x\t", device);
////            cout << endl;
////        }
////        {
////            vector <uint8_t> active_devices;
////            i2c1.checkAddressSpace(0, 127, active_devices);
////            ROS_INFO("i2c 1 found %ld active devices", active_devices.size());
////            for (auto device:active_devices)
////                printf("%x\t", device);
////            cout << endl;
////        }
////        {
////            vector <uint8_t> active_devices;
////            i2c2.checkAddressSpace(0, 127, active_devices);
////            ROS_INFO("i2c 2 found %ld active devices", active_devices.size());
////            for (auto device:active_devices)
////                printf("%x\t", device);
////            cout << endl;
////        }
//        d.sleep();
//    }

//    vector<uint8_t > deviceIDs = {0x5e};
//    vector<int> devicePin = {255};
//    TLV493D tlv493D(h2p_lw_i2c_addr[2],deviceIDs,devicePin);
//    while(ros::ok()){
//        vector<float> x,y,z;
//        tlv493D.read(x,y,z);
//        ROS_INFO_THROTTLE(1,"%f\t%f\t%f", x[0],y[0],z[0]);
//    }
//    return 0;

//    vector<uint8_t> deviceIDs = {0xF};
//    vector<uint8_t> motorids = {0};
//    A1335 motorAngle(h2p_lw_i2c_addr[0],motorids,deviceIDs);
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
//    u_int8_t id = IORD(h2p_lw_switches_addr,0)&0x7;
//    vector<uint8_t> deviceIDs = {0x50, 0x51, 0x52, 0x53};
//    HandControl handControl(h2p_lw_i2c_addr[0], deviceIDs, id);
//    handControl.test();

    myoControl = MyoControlPtr(new MyoControl(h2p_lw_myo_addr,h2p_lw_adc_addr));
    RoboyPlexus roboyPlexus(myoControl, h2p_lw_myo_addr, h2p_lw_i2c_addr, h2p_lw_darkroom_addr,
                            h2p_lw_darkroom_ootx_addr, h2p_lw_adc_addr, h2p_lw_switches_addr);
    PerformMovementAction performMovementAction(myoControl, roboyPlexus.getBodyPart() + "_movement_server");
    PerformMovementsAction performMovementsAction(myoControl, roboyPlexus.getBodyPart() + "_movements_server");

    signal(SIGINT, SigintHandler);

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