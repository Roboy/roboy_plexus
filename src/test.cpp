#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdlib.h>
#include "hwlib.h"
#include "socal/hps.h"
#include <vector>
#include "roboyPlexus.hpp"

// #include "common_utilities/NeoPixel.hpp"

// #define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define SYSID_QSYS_BASE 0x1000
#define SYSTEM_ID 0xb16b00b5

using namespace std;

int32_t *h2p_lw_sysid_addr;
int32_t *h2p_lw_led_addr;
int32_t *h2p_lw_neopixel_addr;
int32_t *h2p_lw_adc_addr;
int32_t *h2p_lw_switches_addr;
int32_t *h2p_lw_darkroom_addr;
int32_t *h2p_lw_a1339_addr;
int32_t *h2p_lw_myoquad_addr;
vector<int32_t*> h2p_lw_darkroom_ootx_addr;
vector<int32_t*> h2p_lw_myo_addr;
vector<int32_t*> h2p_lw_i2c_addr;

// NeoPixelPtr neoPixel;

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
  if(*h2p_lw_sysid_addr!=SYSTEM_ID){ // if the system id does not match, we abort
      printf("system id %x does not match this version of plexus %x, make sure you loaded the correct fpga image",*h2p_lw_sysid_addr, SYSTEM_ID);
      // clean up our memory mapping and exit
      if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
          printf( "ERROR: munmap() failed...\n" );
          close( fd );
          return( 1 );
      }
      close( fd );
      return -1;
  }
  printf("yay!\n");

  // clean up our memory mapping and exit
if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
    printf( "ERROR: munmap() failed...\n" );
    close( fd );
    return( 1 );
}

close( fd );


  // h2p_lw_neopixel_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + NEOPIXEL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
  //
  // neoPixel.reset(new NeoPixel(h2p_lw_neopixel_addr,10));
  // ros::Rate rate(30);
  //   auto pattern = neoPixel->getPattern("nightrider",NeoPixelColorRGB::blue);
  //
  //   while(ros::ok()){
  //       neoPixel->runPattern(pattern,rate);
  //       rate.sleep();
  //   }
  return 0;
}
