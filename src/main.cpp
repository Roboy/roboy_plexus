#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "roboy_plexus/hps_0.h"
#include "roboy_plexus/roboyPlexus.hpp"
#include <vector>

using namespace std;

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

//#define ADC_MEASUREMENT

int main(int argc, char *argv[]) {

    void *virtual_base;
    int fd;
    void *h2p_lw_led_addr;
    uint32_t *h2p_lw_adc_addr;
    int32_t *h2p_lw_darkroom_addr;
    vector<int32_t*> h2p_lw_myo_addr;
    vector<int32_t*> h2p_lw_i2c_addr;

//     map the address space for the LED registers into user space so we can interact with them.
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

    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));

    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));

    h2p_lw_darkroom_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DARKROOM_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );

    h2p_lw_adc_addr = (uint32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_LTC2308_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );

    vector<int> deviceIDs = {0,1};

    RoboyPlexus roboyPlexus(h2p_lw_myo_addr, h2p_lw_i2c_addr, deviceIDs, h2p_lw_darkroom_addr, h2p_lw_adc_addr);

    while(ros::ok()){
        uint32_t crc32 = IORD(h2p_lw_darkroom_addr, 32);
        uint32_t fw_version = IORD(h2p_lw_darkroom_addr, 33);
        uint32_t ID = IORD(h2p_lw_darkroom_addr, 34);
        uint32_t phase0 = IORD(h2p_lw_darkroom_addr, 35);
        uint32_t phase1 = IORD(h2p_lw_darkroom_addr, 36);
        uint32_t tilt0 = IORD(h2p_lw_darkroom_addr, 37);
        uint32_t tilt1 = IORD(h2p_lw_darkroom_addr, 38);
        uint32_t unlock_count = IORD(h2p_lw_darkroom_addr, 39);
        uint32_t hw_version = IORD(h2p_lw_darkroom_addr, 40);
        uint32_t curve0 = IORD(h2p_lw_darkroom_addr, 41);
        uint32_t curve1 = IORD(h2p_lw_darkroom_addr, 42);
        uint32_t acc_x = IORD(h2p_lw_darkroom_addr, 43);
        uint32_t acc_y = IORD(h2p_lw_darkroom_addr, 44);
        uint32_t acc_z = IORD(h2p_lw_darkroom_addr, 45);
        uint32_t gibphase0 = IORD(h2p_lw_darkroom_addr, 46);
        uint32_t gibphase1 = IORD(h2p_lw_darkroom_addr, 47);
        uint32_t gibmag0 = IORD(h2p_lw_darkroom_addr, 48);
        uint32_t gibmag1 = IORD(h2p_lw_darkroom_addr, 49);
        uint32_t mode = IORD(h2p_lw_darkroom_addr, 50);
        uint32_t faults = IORD(h2p_lw_darkroom_addr, 51);
        ROS_INFO_THROTTLE(1,
            "crc32:       %d\n"
            "fw_version:  %d\n"
            "ID:          %d\n"
            "phase0:      %d\n"
            "phase1:      %d\n"
            "tilt0:       %d\n"
            "tilt1:       %d\n"
            "unlock_count:%d\n"
            "hw_version:  %d\n"
            "curve0:      %d\n"
            "curve1:      %d\n"
            "acc_x:       %d\n"
            "acc_y:       %d\n"
            "acc_z:       %d\n"
            "gibphase0:   %d\n"
            "gibphase1:   %d\n"
            "gibmag0:     %d\n"
            "gibmag0:     %d\n"
            "mode:        %d\n"
            "faults:      %d\n", crc32, fw_version,ID,phase0,phase1,tilt0,tilt1,unlock_count,hw_version,
                          curve0,curve1,acc_x,acc_y,acc_z, gibphase0, gibphase1, gibmag0, gibmag1, mode, faults
        );
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
