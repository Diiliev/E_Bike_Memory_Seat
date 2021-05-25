#ifndef CANTRANSMIT_H
#define CANTRANSMIT_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <ros/ros.h>

void writeMessageToCan(std::string height, char direction);


#endif