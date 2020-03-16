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
#include "interfaces/NeoPixel.hpp"

using namespace std;

#define SYSTEM_ID 0xb16b00b5

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int32_t *h2p_lw_sysid_addr;
int32_t *h2p_lw_led_addr;
int32_t *h2p_lw_neopixel_addr;
int32_t *h2p_lw_adc_addr;
int32_t *h2p_lw_switches_addr;
int32_t *h2p_lw_a1339_addr;
vector<int32_t*> h2p_lw_darkroom_ootx_addr;
vector<int32_t*> h2p_lw_icebus_addr;
vector<int32_t*> h2p_lw_myo_addr;
vector<int32_t*> h2p_lw_auxilliary_i2c_addr;
vector<int32_t*> h2p_lw_ball_joint_addr;
vector<int32_t*> h2p_lw_fan_control_addr;

IcebusControlPtr icebusControl;
NeoPixelPtr neoPixel;

void SigintHandler(int sig)
{
    cout << "shutting down" << endl;

    // All the default sigint handler does is call shutdown()
    rclcpp::shutdown();
    *h2p_lw_led_addr = 0x00;
    system("sl");
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
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"system id %x does not match this version of plexus %x, make sure you loaded the correct fpga image",*h2p_lw_sysid_addr, SYSTEM_ID);
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

#ifdef ICEBOARDCONTROL_0_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBOARDCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBOARDCONTROL_1_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBOARDCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBOARDCONTROL_2_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBOARDCONTROL_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
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
#ifdef FANCONTROL_0_BASE
    h2p_lw_fan_control_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FANCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "icebus init");
    icebusControl = IcebusControlPtr(new IcebusControl(motor_config_file_path,h2p_lw_icebus_addr,h2p_lw_adc_addr,neoPixel));
    vector<BallJointPtr> balljoints;
    for(auto addr:h2p_lw_ball_joint_addr)
      balljoints.push_back(BallJointPtr(new BallJoint(addr)));
    vector<FanControlPtr> fanControls;
    for(auto addr:h2p_lw_fan_control_addr)
      fanControls.push_back(FanControlPtr(new FanControl(addr)));
    // balljoints.push_back(BallJointPtr(new BallJoint(h2p_lw_sensor1_i2c_addr)));
    // balljoints.push_back(BallJointPtr(new BallJoint(h2p_lw_sensor2_i2c_addr)));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "roboy plexus init");
    RoboyPlexus roboyPlexus(icebusControl,balljoints,fanControls,h2p_lw_auxilliary_i2c_addr);
////    PerformMovementAction performMovementAction(myoControl, roboyPlexus.getBodyPart() + "_movement_server");
////    PerformMovementsAction performMovementsAction(myoControl, roboyPlexus.getBodyPart() + "_movements_server");
////
//    signal(SIGINT, SigintHandler);
//


    // TODO: fix rate
    rclcpp::Rate rate(30);
    if(h2p_lw_neopixel_addr!=nullptr){
//        auto pattern = neoPixel->getPattern("nightrider",NeoPixelColorRGB::blue);
        while(rclcpp::ok()){
//            neoPixel->runPattern(pattern,rate);
            rate.sleep();
        }
    }else{
        while(rclcpp::ok()){
            rate.sleep();
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
