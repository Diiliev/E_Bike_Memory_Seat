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

    // write the goal height in the first byte of the data array
    // and zeroes in the remaining bytes of the data array
    frame.data[0] = height;
    for (int i = 1; i < CAN_MAX_DLEN; i++) {
        frame.data[i] = 0;
    }

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
 * - Returns -1 on failure to read.
 * - If the feedback value is 255 that means the actuator has stalled
 *   and the operation has failed.
 */
int readMessageFromCanWithId(int canIdToRead) {

    // FD is short for File Descriptor
    int canSocketFD, selectResult, readResult;
    fd_set readFDSet;
    struct timeval timeout;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;

    // create a socket and save the returned file descriptor
	if ((canSocketFD = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		ROS_ERROR("Socket setting failed!");
		return -1;
	}

    // Get the interface index for "can0"
	strcpy(ifr.ifr_name, "can0" );
	ioctl(canSocketFD, SIOCGIFINDEX, &ifr);

    // Bind the socket to the interface 
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(canSocketFD, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		ROS_ERROR("Bindding socket failed!");
		return -1;
	}

    // Filter out messages with id not equal to the id of the MCU
    struct can_filter readFilter[1];

    readFilter[0].can_id = canIdToRead;
    readFilter[0].can_mask = 0xFFF;

    setsockopt(canSocketFD, SOL_CAN_RAW, CAN_RAW_FILTER, &readFilter, sizeof(readFilter));

    // initialize the timeout timeval
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;

    FD_ZERO(&readFDSet); // clear the set
    FD_SET(canSocketFD, &readFDSet); // add the file descriptor to the set

    // Read a can message if the socket is ready
    selectResult = select(canSocketFD + 1, &readFDSet, NULL, NULL, &timeout);

 	if (selectResult == -1) {
		ROS_ERROR("Select from CAN socket failed!");
		return -1;
	}
    else if (selectResult == 0) {
        ROS_ERROR("Timeout while reading from CAN socket!");
		return -1;
    }
    else {
        readResult = read(canSocketFD, &frame, sizeof(struct can_frame));

        if (readResult < 0) {
            ROS_ERROR("Reading from CAN socket failed!");
            return -1;
        }
    }

	ROS_INFO("Read message with id: 0x%03X, length: [%d] ", frame.can_id, frame.can_dlc);

    // ignore messages which are not published by canIdToRead
    // TODO use a can filter instead. See https://www.beyondlogic.org/example-c-socketcan-code/
    if (frame.can_id != canIdToRead) {
        ROS_WARN("Ignoring message from sender with id: 0x%X", frame.can_id);
        return -2;
    }
    
    // the feedback height is the first byte of the message
    // convert it from hex data to int which represents millimeters
    // i.e HEX 64 = INT 100, this means 100mm high
    int feedbackHeight = (int)frame.data[0]; 

	ROS_INFO("Current height is %dmm", feedbackHeight);

	if (close(canSocketFD) < 0) {
		ROS_ERROR("Closing socket failed!");
		return -1;
	}

    return feedbackHeight;
}