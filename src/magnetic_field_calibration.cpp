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
#include <std_msgs/Float32.h>

using namespace std;

#define SYSTEM_ID 0xb16b00b5

#define ALT_LWFPGASLVS_OFST 0xFF200000
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int32_t *h2p_lw_sysid_addr;
int32_t* h2p_lw_ball_joint_addr;
int32_t* h2p_lw_a1335_addr;
int32_t* h2p_lw_pwm_addr;

float target = 100.0, Kp = 1, Ki = 0.1, integral = 0, integralLimit = 5,
  PWMLimit = 50, error = 0.0, result = 0.0, zero_speed = 310, position_prev = 0, position = 0;
int overflow_counter = 0;

void MotorCommand(const std_msgs::Float32::ConstPtr &msg){
  target = msg->data;
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


    h2p_lw_ball_joint_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + BALLJOINT_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_a1335_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + AUXILLIARY_I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );
    h2p_lw_pwm_addr = (int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) );

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "magnetic_field_calibration", ros::init_options::NoSigintHandler);
    }

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner = ros::AsyncSpinner(0);
    spinner.start();
//
    ros::Publisher magneticSensor = nh.advertise<roboy_middleware_msgs::MagneticSensor>("/roboy/middleware/MagneticSensor",1);
    ros::Publisher motor_position = nh.advertise<std_msgs::Float32>("/motor_position",1);
    ros::Subscriber motor_target = nh.subscribe("/motor_target", 1, &MotorCommand);

    int tca_addr[] = {0x70,0x71,0x72,0x73,0x74,0x75,0x76};
    int number_of_sensors[] = {4,4,4,2,4,4,4};
    vector<TLE493DPtr> sensor;
    for(int i=0;i<7;i++){
      sensor.push_back(TLE493DPtr(new TLE493D(h2p_lw_ball_joint_addr,tca_addr[i],number_of_sensors[i])));
    }

    vector <uint8_t> ids = {0xC};
    A1335 a1335(h2p_lw_a1335_addr,ids);

    ros::Rate rate(100);
    int16_t pwm = 0;

    while(ros::ok()){

        roboy_middleware_msgs::MagneticSensor msg;
        int i=0;
        for(auto s:sensor){
          s->readMagneticData(msg.sensor_id,msg.x,msg.y,msg.z);
        }
        magneticSensor.publish(msg);

        vector<A1335State> state;
        a1335.readAngleData(state);
        // ROS_INFO_STREAM_THROTTLE(1,state[0].angle);

        if(position_prev>340 && state[0].angle < 20)
          overflow_counter++;
        if(position_prev<20 && state[0].angle > 340)
          overflow_counter--;
        position = state[0].angle + overflow_counter*360;
        std_msgs::Float32 msg2;
        msg2.data = position;
        motor_position.publish(msg2);  

        position_prev = state[0].angle;

        error = target - position;

        integral += Ki*error;
        if(integral>integralLimit)
          integral = integralLimit;
        if(integral<-integralLimit)
          integral = -integralLimit;

        result = Kp*error+integral;
        if(result>PWMLimit)
          result = PWMLimit;
        if(result<-PWMLimit)
          result = -PWMLimit;

        ROS_INFO_THROTTLE(1, "position %f, error %f, integral %f, result %f", position, error, integral, result);

        *h2p_lw_pwm_addr=int(zero_speed+result);

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
