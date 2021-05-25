#!/bin/sh

sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
sudo ifconfig can0 txqueuelen 1000
#sudo roscore
#sudo rosrun e_bike_memory_seat ebmsActionServer
#sudo roslaunch rosbridge_server rosbridge_websocket.launch
