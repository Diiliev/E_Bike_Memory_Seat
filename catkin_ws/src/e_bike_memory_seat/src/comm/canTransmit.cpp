#include "canTransmit.h"

/**
 * Write a message to the CAN bus containing data: height and can_id: canIdToSend.
 * - Returns true on success.
 * - Returns false on failure.
 */
bool writeMessageToCan(int height, int canIdToSend) {
    int s; 
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    ROS_INFO("CAN Sockets Write Demo");

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        ROS_ERROR("Socket setting failed!");
        return false;
    }

    strcpy(ifr.ifr_name, "can0" );
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("Binding socket failed!");
        return false;
    }

    frame.can_id = canIdToSend;   // id of the sender
    frame.can_dlc = CAN_MAX_DLEN;      // data length code 

    // write the goal height to the buffer
    // sprintf((char *)frame.data, "%d", height);
    frame.data[0] = height;
    ROS_INFO("Attempting to publish to CAN bus the following height: %d", height);

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        ROS_ERROR("Writting frame to CAN socket failed!");
        return false;
    }

    if (close(s) < 0) {
        ROS_ERROR("Closing socket failed!");
        return false;
    }

    return true;
}

/**
 * Read a message from the CAN bus with can_id: canIdToRead.
 * - Returns current seat height in millimeters on success.
 * - Returns -1 on failure.
 */
int readMessageFromCanWithId(int canIdToRead) {
    int s, i; 
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;

	ROS_INFO("CAN Sockets Receive Demo\r\n");

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		ROS_ERROR("Socket setting failed!");
		return -1;
	}

	strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		ROS_ERROR("Bindding socket failed!");
		return -1;
	}

    // ToDo check if there is a timeout option on the read function.
    // http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient
	nbytes = read(s, &frame, sizeof(struct can_frame));

 	if (nbytes < 0) {
		ROS_ERROR("Reading from CAN socket failed!");
		return -1;
	}

	ROS_INFO("Read message with id: 0x%03X, length: [%d] ", frame.can_id, frame.can_dlc);

    // ignore messages which are not published by canIdToRead
    // TODO we're returning -2 here because -1 will abort the action instead of retrying
    if (frame.can_id != canIdToRead) {
        ROS_WARN("Ignoring message from sender with id: 0x%X", frame.can_id);
        return -2;
    }

    for (int i = 0; i < frame.can_dlc; i++) {
        ROS_INFO("hex data: %X ", frame.data[i]);
    }
    
    // the feedback height is the first byte of the message
    // convert it from hex data to int which represents millimeters
    // i.e HEX 64 = INT 100, this means 100mm high
    int feedbackHeight = (int)frame.data[0]; 

	ROS_INFO("\r\n");

	if (close(s) < 0) {
		ROS_ERROR("Closing socket failed!");
		return -1;
	}

    return feedbackHeight;
}