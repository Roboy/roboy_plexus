#include <ros/ros.h>
#include <common_utilities/UDPSocket.hpp>

const char* key = "The path of the righteous man is beset on all sides by the inequities of the "
        "selfish and the tyranny of evil men. Blessed is he who, in the name of "
        "charity and good will, shepherds the weak through the valley of the darkness. "
        "For he is truly his brother's keeper and the finder of lost children. And I "
        "will strike down upon thee with great vengeance and furious anger those who "
        "attempt to poison and destroy my brothers. And you will know I am the Lord "
        "when I lay my vengeance upon you\0";

int main(int argc, char *argv[]){
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "ROS_MASTER_URI_receiver");
    }
    // create the IP broadcast Socket
    UDPSocketPtr receiver_socket = UDPSocketPtr(new UDPSocket(BROADCAST_PORT, false));
    uint32_t status = 0;
    while(status!=3 && !ros::master::check()){
        usleep(1000000);
        printf("listening for HOST IP\n");
        uint32_t host_IP;
        status = receiver_socket->receiveHostIP(key,host_IP);
        if(status==3){
            char IP[16];
            receiver_socket->convertByte2Text(host_IP,IP);
            ROS_INFO("Received HOST IP: %s",IP);
            char ros_master_uri[100];
            sprintf(ros_master_uri, "ROS_MASTER_URI=http://%s:11311",IP);
            printf("%s\n",ros_master_uri);
            putenv(ros_master_uri);
        }
    }

    system (argv[1]);
}