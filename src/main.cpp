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

    author: Simon Trendel ( simon.trendel@tum.de ), 2020
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
#include "hps_0.h"
#include "roboyPlexus.hpp"

using namespace std;

#define SYSTEM_ID 0xb16b00b5

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int32_t *h2p_lw_sysid_addr;
int32_t *h2p_lw_power_sense_addr;
int32_t *h2p_lw_power_control_addr;
int32_t *h2p_lw_led_addr;
int32_t *h2p_lw_switches_addr;
vector<int32_t*> h2p_lw_armbus_addr;
vector<int32_t*> h2p_lw_icebus_addr;
vector<int32_t*> h2p_lw_myo_addr;
vector<int32_t*> h2p_lw_auxilliary_i2c_addr;
vector<int32_t*> h2p_lw_ball_joint_addr;
vector<int32_t*> h2p_lw_fan_control_addr;
vector<int32_t*> i2c_base;

IcebusControlPtr icebusControl;
MyoControlPtr myoControl;

void SigintHandler(int sig)
{
    cout << "shutting down" << endl;
    // turn off 5V and 12V power
    *h2p_lw_power_control_addr = 0x3;
    // system("sl -ae&");
    ros::Rate rate(10);
    *h2p_lw_led_addr = 0;
    for(int i=0;i<10;i++){
      *h2p_lw_led_addr = ~(*h2p_lw_led_addr);
      rate.sleep();
    }

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
    *h2p_lw_led_addr = 0x00;

    system("fortune | cowsay");
}

int main(int argc, char *argv[]) {
    if(argc!=2) {
        printf("ERROR: please provide motor config file path\n"
               "USAGE: ./roboy_plexus path/to/motor_config.yaml\n");
        return -1;
    }
    string motor_config_file_path(argv[1]);

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
    if(*h2p_lw_sysid_addr!=SYSTEM_ID){ // if the system id does not match, we abort
        ROS_ERROR("system id %x does not match this version of plexus %x, make sure you loaded the correct fpga image",*h2p_lw_sysid_addr, SYSTEM_ID);
        // clean up our memory mapping and exit
        if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
            printf( "ERROR: munmap() failed...\n" );
            close( fd );
            return( 1 );
        }
        close( fd );
        return -1;
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

#ifdef POWER_SENSE_BASE
    h2p_lw_power_sense_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + POWER_SENSE_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_power_sense_addr = nullptr;
#endif

#ifdef POWER_CONTROL_BASE
    h2p_lw_power_control_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + POWER_CONTROL_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_power_control_addr = nullptr;
#endif

#ifdef ICEBUSCONTROL_0_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBUSCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBUSCONTROL_1_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBUSCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBUSCONTROL_2_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBUSCONTROL_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBUSCONTROL_3_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBUSCONTROL_3_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBUSCONTROL_4_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBUSCONTROL_4_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBUSCONTROL_5_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBUSCONTROL_5_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBUSCONTROL_6_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBUSCONTROL_6_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBUSCONTROL_7_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBUSCONTROL_7_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif

#ifdef AUXILLIARY_I2C_0_BASE
    h2p_lw_auxilliary_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + AUXILLIARY_I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef AUXILLIARY_I2C_1_BASE
    h2p_lw_auxilliary_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + AUXILLIARY_I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef AUXILLIARY_I2C_2_BASE
    h2p_lw_auxilliary_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + AUXILLIARY_I2C_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef AUXILLIARY_I2C_3_BASE
    h2p_lw_auxilliary_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + AUXILLIARY_I2C_3_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif

#ifdef BALLJOINT_0_BASE
    h2p_lw_ball_joint_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + BALLJOINT_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef BALLJOINT_1_BASE
    h2p_lw_ball_joint_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + BALLJOINT_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef BALLJOINT_2_BASE
    h2p_lw_ball_joint_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + BALLJOINT_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef BALLJOINT_3_BASE
    h2p_lw_ball_joint_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + BALLJOINT_3_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef BALLJOINT_4_BASE
    h2p_lw_ball_joint_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + BALLJOINT_4_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif

#ifdef FANCONTROL_0_BASE
    h2p_lw_fan_control_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FANCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef FANCONTROL_1_BASE
    h2p_lw_fan_control_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FANCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef FANCONTROL_2_BASE
    h2p_lw_fan_control_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FANCONTROL_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef FANCONTROL_3_BASE
    h2p_lw_fan_control_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FANCONTROL_3_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef FANCONTROL_4_BASE
    h2p_lw_fan_control_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FANCONTROL_4_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef FANCONTROL_5_BASE
    h2p_lw_fan_control_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FANCONTROL_5_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif

#ifdef MYOCONTROL_0_BASE
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef MYOCONTROL_1_BASE
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef MYOCONTROL_2_BASE
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif

    MotorConfigPtr motor_config = MotorConfigPtr(new MotorConfig);
    motor_config->readConfig(motor_config_file_path);

    myoControl = MyoControlPtr(new MyoControl(motor_config,h2p_lw_myo_addr));
    icebusControl = IcebusControlPtr(new IcebusControl(motor_config,h2p_lw_icebus_addr));
    vector<TLE493DPtr> balljoints;
    for(auto addr:h2p_lw_ball_joint_addr)
      balljoints.push_back(TLE493DPtr(new TLE493D(addr)));
    vector<FanControlPtr> fanControls;
    for(auto addr:h2p_lw_fan_control_addr)
      fanControls.push_back(FanControlPtr(new FanControl(addr)));
    RoboyPlexus roboyPlexus(icebusControl,balljoints,fanControls,
        h2p_lw_led_addr,h2p_lw_switches_addr,h2p_lw_power_control_addr,h2p_lw_power_sense_addr,h2p_lw_auxilliary_i2c_addr,myoControl);
////    PerformMovementAction performMovementAction(myoControl, roboyPlexus.getBodyPart() + "_movement_server");
////    PerformMovementsAction performMovementsAction(myoControl, roboyPlexus.getBodyPart() + "_movements_server");
////
    signal(SIGINT, SigintHandler);
// //

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "magnetic_sensor_test", ros::init_options::NoSigintHandler);
    }

    // ros::NodeHandle nh;
    // ros::AsyncSpinner spinner = ros::AsyncSpinner(0);
    // spinner.start();
//
    // ros::Publisher magneticSensor = nh.advertise<roboy_middleware_msgs::MagneticSensor>("/roboy/middleware/MagneticSensor",
    //                                                                       1);
    //
    // vector<TLE493DPtr> sensor;
    // for(auto base:i2c_base){
    //   sensor.push_back(TLE493DPtr(new TLE493D(base)));
    // }
    ros::Rate rate(10);

    while(ros::ok()){

        // roboy_middleware_msgs::MagneticSensor msg;
        // int i=0;
        // for(auto s:sensor){
        //   s->read(msg.x,msg.y,msg.z);
        // }
        // magneticSensor.publish(msg);

        rate.sleep();
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
