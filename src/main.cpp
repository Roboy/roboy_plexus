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
#include "roboy_plexus/NeoPixel.hpp"

using namespace std;

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int32_t *h2p_lw_sysid_addr;
int32_t *h2p_lw_led_addr;
int32_t *h2p_lw_neopixel_addr;
int32_t *h2p_lw_adc_addr;
int32_t *h2p_lw_switches_addr;
int32_t *h2p_lw_darkroom_addr;
int32_t *h2p_lw_a1339_addr;
vector<int32_t*> h2p_lw_darkroom_ootx_addr;
vector<int32_t*> h2p_lw_myo_addr;
vector<int32_t*> h2p_lw_i2c_addr;

MyoControlPtr myoControl;
NeoPixelPtr neoPixel;

void SigintHandler(int sig)
{
    cout << "shutting down" << endl;
    if(myoControl!= nullptr){
        // switch to displacement
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            if(find(myoControl->myo_bricks.begin(), myoControl->myo_bricks.end(), motor)!=myoControl->myo_bricks.end())
                myoControl->changeControl(motor, VELOCITY);
            else
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
                if(find(myoControl->myo_bricks.begin(), myoControl->myo_bricks.end(), motor)!=myoControl->myo_bricks.end())
                    continue;
                int displacement = myoControl->getDisplacement(motor);
                if(displacement<=0)
                    continue;
                else
                    myoControl->setDisplacement(motor,displacement*(decrements/100.0));
            }
            rate.sleep();
            if(decrements%15==0) {
                if(!toggle)
                    neoPixel->setColorAll(0xF00000);
                else
                    neoPixel->setColorAll(NeoPixelColorRGB::black);
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
    neoPixel->abort = true;
    neoPixel->setColorAll(NeoPixelColorRGB::black);
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

#ifdef LED_BASE
    h2p_lw_led_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_led_addr = nullptr;
#endif
#ifdef NEOPIXEL_0_BASE
    h2p_lw_neopixel_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + NEOPIXEL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_neopixel_addr = nullptr;
#endif
#ifdef SWITCHES_BASE
    h2p_lw_switches_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SWITCHES_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_switches_addr = nullptr;
#endif
#ifdef A1339_0_BASE
    h2p_lw_a1339_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + A1339_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
#else
    h2p_lw_a1339_addr = nullptr;
#endif
#ifdef MYOCONTROL_0_BASE
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
//    h2p_lw_myo_addr.push_back(nullptr);
#endif
#ifdef MYOCONTROL_1_BASE
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
//    h2p_lw_myo_addr.push_back(nullptr);
#endif
#ifdef MYOCONTROL_2_BASE
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_2_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
#else
//    h2p_lw_myo_addr.push_back(nullptr);
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

    neoPixel.reset(new NeoPixel(h2p_lw_neopixel_addr,10));

    myoControl = MyoControlPtr(new MyoControl(h2p_lw_myo_addr,h2p_lw_adc_addr,neoPixel));
    RoboyPlexus roboyPlexus(myoControl, h2p_lw_myo_addr, h2p_lw_i2c_addr, h2p_lw_darkroom_addr,
                            h2p_lw_darkroom_ootx_addr, h2p_lw_adc_addr, h2p_lw_switches_addr);
    PerformMovementAction performMovementAction(myoControl, roboyPlexus.getBodyPart() + "_movement_server");
    PerformMovementsAction performMovementsAction(myoControl, roboyPlexus.getBodyPart() + "_movements_server");

    signal(SIGINT, SigintHandler);

    ros::Rate rate(30);
    auto pattern = neoPixel->getPattern("nightrider",NeoPixelColorRGB::blue);
    while(ros::ok()){
        neoPixel->runPattern(pattern,rate);
//        neoPixel->setColor(1,0x80);
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