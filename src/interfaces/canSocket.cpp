#include "interfaces/canSocket.hpp"

CanSocket::CanSocket() {}

int CanSocket::initInterface(std::string interface)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    /* open socket and check if it worked*/
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        ROS_ERROR("Cannot create CAN socket err: %d", s);
        return -1;
    }

    strcpy(ifr.ifr_name, interface.c_str());
    /* find the right socket name depending on the interface string" */
    ioctl(s, SIOCGIFINDEX, &ifr);
    // verify that the socket exists
    if(!ifr.ifr_ifindex){
        ROS_ERROR("Cannot find index with socket name %s", interface);
        return -1;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // bind the socket index to the socket s and verify if it worked
    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0){
        ROS_ERROR("Cannot bind socket!");
        return -1;
    }

    ROS_INFO("Initilzation of Can socket succesfull.");
    return 0;
}

void CanSocket::canTransmit(const roboy_middleware_msgs::CanFrame::ConstPtr &msg)
{
    struct can_frame tx;
    tx.can_dlc = msg->data_length;
    tx.can_id = msg->can_id;
    memcpy(tx.data, &msg->data[0], msg->data_length);
    ROS_INFO("tx: %d, %d, %d, %d, %d, %d, %d, %d;", tx.data[0], tx.data[1], tx.data[2], tx.data[3], tx.data[4], tx.data[5], tx.data[6], tx.data[7]);
    int nbytes = write(s, &tx, sizeof(struct can_frame));
    ROS_INFO("Send return %d", nbytes);
    ROS_INFO("Errno %d", errno);
}

void CanSocket::canTransmit(roboy_middleware_msgs::CanFrame msg)
{
    struct can_frame tx;
    tx.can_dlc = msg.data_length;
    tx.can_id = msg.can_id;
    memcpy(tx.data, &msg.data[0], msg.data_length);
    int nbytes = write(s, &tx, sizeof(struct can_frame));
}

int CanSocket::canRensieve(roboy_middleware_msgs::CanFrame* msg)
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

    msg->can_id = frame.can_id;
    msg->data_length = frame.can_dlc;
    memcpy(&msg->data[0], frame.data, frame.can_dlc);

    return 0;
}

CanSocket::~CanSocket() {
    // close the socket
    close(s);
}