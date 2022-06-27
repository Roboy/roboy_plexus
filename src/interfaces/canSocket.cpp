#include "interfaces/canSocket.hpp"

int CanSocket::initInterface(string interface)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    /* open socket and check if it worked*/
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        ROS_FATAL("Cannot create CAN socket err: %d", s);
        return -1;
    }

    strcpy(ifr.ifr_name, interface.c_str());
    /* find the right socket name depending on the interface string" */
    ioctl(s, SIOCGIFINDEX, &ifr);
    // verify that the socket exists
    if(!ifr.ifr_ifindex){
        ROS_FATAL("Cannot find index with socket name %s", interface);
        return -1;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // bind the socket index to the socket s and verify if it worked
    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0){
        ROS_FATAL("Cannot bind socket!");
        return -1;
    }

    ROS_INFO("Initilzation of Can socket succesfull.");
    return 0;
}

void CanSocket::canTransmit(int can_id, uint8_t *data){
    struct can_frame tx;
    tx.can_dlc = 8;
    tx.can_id = can_id;
    memcpy(tx.data, data, 8);
    int nbytes = write(s, &tx, sizeof(struct can_frame));
}

int CanSocket::canRensieve(uint8_t *data)
{
    struct can_frame frame;

    int nbytes = read(s, &frame, sizeof(struct can_frame));

    if (nbytes < 0)
    {
        perror("can raw socket read");
        return -1;
    }

    /* paranoid check ... */
    if (nbytes < sizeof(struct can_frame))
    {
        fprintf(stderr, "read: incomplete CAN frame\n");
        return -1;
    }

    memcpy(data, frame.data, frame.can_dlc);

    return 0;
}

CanSocket::~CanSocket() {
    // close the socket
    close(s);
}