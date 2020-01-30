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
#include "hps_0.h"
#include "roboyPlexus.hpp"
#include "interfaces/NeoPixel.hpp"
#include "interfaces/hand.hpp"

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
int32_t *h2p_lw_uart_addr;
vector<int32_t*> h2p_lw_darkroom_ootx_addr;
vector<int32_t*> h2p_lw_icebus_addr;
vector<int32_t*> h2p_lw_myo_addr;
vector<int32_t*> h2p_lw_i2c_addr;

IcebusControlPtr icebusControl;
MyoControlPtr myoControl;
NeoPixelPtr neoPixel;

void SigintHandler(int sig)
{
    cout << "shutting down" << endl;

    if(h2p_lw_led_addr!=nullptr)
        *h2p_lw_led_addr = 0;
    neoPixel->abort = true;
    neoPixel->setColorAll(NeoPixelColorRGB::black);
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
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
#ifdef NEOPIXEL_0_BASE
    h2p_lw_neopixel_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + NEOPIXEL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    neoPixel.reset(new NeoPixel(h2p_lw_neopixel_addr,10));
#else
    h2p_lw_neopixel_addr = nullptr;
#endif
#ifdef SWITCHES_BASE
    h2p_lw_switches_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SWITCHES_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_switches_addr = nullptr;
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
#ifdef ICEBOARDCONTROL_0_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBOARDCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBOARDCONTROL_1_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBOARDCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBOARDCONTROL_2_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBOARDCONTROL_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBOARDCONTROL_3_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBOARDCONTROL_3_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBOARDCONTROL_4_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBOARDCONTROL_4_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef ICEBOARDCONTROL_5_BASE
    h2p_lw_icebus_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ICEBOARDCONTROL_5_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef I2C_0_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef I2C_1_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef I2C_2_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef I2C_3_BASE
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_3_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#endif
#ifdef UART_HAND_BASE
    h2p_lw_uart_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + UART_HAND_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#endif



    myoControl = MyoControlPtr(new MyoControl(motor_config_file_path,h2p_lw_myo_addr,h2p_lw_adc_addr,neoPixel));
    icebusControl = IcebusControlPtr(new IcebusControl(motor_config_file_path,h2p_lw_icebus_addr,h2p_lw_adc_addr,neoPixel));
    RoboyPlexus roboyPlexus(icebusControl,myoControl,h2p_lw_i2c_addr, h2p_lw_adc_addr, h2p_lw_switches_addr);
    UART hand_uart(h2p_lw_uart_addr,9600);
////    PerformMovementAction performMovementAction(myoControl, roboyPlexus.getBodyPart() + "_movement_server");
////    PerformMovementsAction performMovementsAction(myoControl, roboyPlexus.getBodyPart() + "_movements_server");
////
//    signal(SIGINT, SigintHandler);
//


    ros::Rate rate(30);
    if(h2p_lw_neopixel_addr!=nullptr){
        auto pattern = neoPixel->getPattern("nightrider",NeoPixelColorRGB::blue);
        while(ros::ok()){
            neoPixel->runPattern(pattern,rate);
            rate.sleep();
        }
    }else{
        while(ros::ok()){
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
