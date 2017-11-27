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

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

//#define ADC_MEASUREMENT

int main(int argc, char *argv[]) {

//    int file;
//    const char *filename = "/dev/i2c-1";
//    uint8_t id;
//    bool bSuccess;
//    const int mg_per_digi = 4;
//    uint16_t szXYZ[3];
//    int cnt=0, max_cnt=0;
//
//    printf("===== gsensor test =====\r\n");
//
//    if (argc == 2){
//        max_cnt = atoi(argv[1]);
//    }
//
//    // open bus
//    if ((file = open(filename, O_RDWR)) < 0) {
//        /* ERROR HANDLING: you can check errno to see what went wrong */
//        perror("Failed to open the i2c bus of gsensor");
//        exit(1);
//    }
//
//    bSuccess = false;
//    uint8_t readdata[5];
//    // write to define register
//    uint8_t readaddr = 0;
//    int i = 0;
//    while(i<100) {
//        if (write(file, &readaddr, sizeof(readaddr)) == sizeof(readaddr)) {
//            // read back value
//            if (read(file, readdata, 5) == 5) {
//                bSuccess = true;
//            }
//        }
//        usleep(10000);
//        i++;
//    }
//
//    return bSuccess;

    void *virtual_base;
    int fd;
    uint32_t *h2p_lw_led_addr;
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

    h2p_lw_led_addr = (uint32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_BASE ) & ( unsigned long)( HW_REGS_MASK )) );

    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
//
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
    h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
//
    h2p_lw_darkroom_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DARKROOM_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
//
    h2p_lw_adc_addr = (uint32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_LTC2308_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
//
    vector<int> deviceIDs = {0,1};

    RoboyPlexus roboyPlexus(h2p_lw_myo_addr, h2p_lw_i2c_addr, deviceIDs, h2p_lw_darkroom_addr, h2p_lw_adc_addr);
//
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy_fpga_interface");
    }
    ros::NodeHandle nh;

    uint32_t mask = 0;
    high_resolution_clock::time_point t0 = high_resolution_clock::now(), t1;
    while(ros::ok()){
        t1 = high_resolution_clock::now();
        milliseconds time_span = duration_cast<milliseconds>(t1-t0);
        if(time_span.count()%500==0){
            mask ++;
            if(mask==255)
                mask =0;
            *h2p_lw_led_addr = mask;
        }
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