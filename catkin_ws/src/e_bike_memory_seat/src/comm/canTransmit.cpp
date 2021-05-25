#include "canTransmit.h"

void writeMessageToCan(std::string height, char direction) {
    int s; 
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    ROS_INFO("CAN Sockets Demo");

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        ROS_ERROR("Socket setting failed!");
        return;
    }

    strcpy(ifr.ifr_name, "can0" );
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("Binding socket failed!");
        return;
    }

    frame.can_id = 0x555;
    frame.can_dlc = 5;

    // First, write the goal hieght to the buffer, then append the character 'r'
    // to raise or 'l' to lower.
    int iterator = sprintf((char *)frame.data, "%s", height.c_str());
    iterator += sprintf((char *)frame.data+iterator, "%c", direction);

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        ROS_ERROR("Writting frame to CAN socket failed!");
        return;
    }

    if (close(s) < 0) {
        ROS_ERROR("Closing socket failed!");
        return;
    }
}