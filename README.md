This is the "E-Bike Memory Seat" project repository. The seperate modules of the project are combined inside a catkin workspace. The main modules are the web client, ROS server and microcontroller.

Credit where credit is due, making this project would not have been possible without the help of these tutorials:
- Example C SocketCAN Code by Craig Peacock, https://www.beyondlogic.org/example-c-socketcan-code/
- CAN recieve tutorial by loovee, 2014-6-13, for the Arduino.
- ROS tutorials, http://wiki.ros.org/ROS/Tutorials

# How to build the project

To build the project change directory to catkin_ws and use:
$ catkin_make 

If you add a new package to the already compiled workspace use:
$ catkin_make --force-cmake

For more information visit: http://wiki.ros.org/catkin/Tutorials/using_a_workspace


# How to run the project

$ sudo ip link set can0 down
$ sudo ip link set can0 up type can bitrate 500000
$ sudo ifconfig can0 txqueuelen 1000
$ roscore
(Then in a separate terminal)
$ rosrun e_bike_memory_seat ebmsActionServer
(Then in a separate terminal)
$ roslaunch rosbridge_server rosbridge_websocket.launch

Next open catkin_ws/src/e_bike_memory_seat/src/client/EbmsLoginPage.html
and start using the application
