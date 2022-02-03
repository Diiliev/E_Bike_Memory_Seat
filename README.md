# E-Bike Memory Seat
This is the "E-Bike Memory Seat" project repository. This project demonstrates the possibility of using an E-Bike rental service where each bike can electronically adjust its seat height to the optimal position for any given user. This would allow the user to start riding immediatelly after ordering a bike without the need to manually adjust their seat beforehand. It would also allow the user to reposition their seat while riding with just the press of a button. 
The main modules of the project are the web client, ROS server and microcontroller, located inside a catkin workspace.
This project was developed as part of my Master's Thesis in the [Technical University of Sofia](https://tu-sofia.bg/) at the Faculty of Electronic Engineering and Technology and in collaboration with [Otto-von-Guericke-Universität Magdeburg](https://www.ovgu.de/) and the team behind ["AuRa - Autonomes Rad"](https://www.aura.ovgu.de/).

![Testbench Front](/assets/EN/Testbench_Front.jpg)
![Testbench Back](/assets/EN/Testbench_Back.jpg)

## Credit where credit is due
Making this project would not have been possible without the help of these tutorials:
- [Example C SocketCAN Code by Craig Peacock](https://www.beyondlogic.org/example-c-socketcan-code/);
- CAN recieve tutorial by loovee, 2014-6-13, for the Arduino;
- [ROS tutorials](http://wiki.ros.org/ROS/Tutorials);
and these programs:
- Schematic and Structural Diagrams made with [XCircuit v3.8 rev78](http://opencircuitdesign.com/xcircuit/download.html);
- Graphic design made with [Inkscape 0.92.3](https://inkscape.org/);
- Sequence diagram made with [UMLet](https://www.umlet.com/);
- Microcontroller embedded software written with [Arduino IDE](https://www.arduino.cc/);
- Web and ROS software written with [Visual Studio Code v1.63.2](https://code.visualstudio.com/);

## How does it work?
Here is a simplified sequence diagram presenting the project workflow for a registered client:
![Sequence Diagram](/assets/EN/Sequence_Diagram.png)

When the client requests an E-Bike from the web application, they can specify their preferred seat height or let the system calculate the optimal seat height for them. Whichever option they choose will result in the system automatically adjusting the seat height of the ordered E-Bike, allowing the user to start riding comfortably and safely immediatelly.

The structure of the project is shown in the diagram bellow:
![Structure Diagram](/assets/EN/Structural_Diagram.png)

When a user orders an E-Bike through the web client application, their request is sent to the backend of the web server. If the user chooses a specific seat height, that becomes the "wanted seat height". Otherwise, if the user has no preference, the server will calculate the optimal seat height by using the LeMond method. This method states that the optimal seat height is 88.3% of the inseam length, which can be taken from the user's database profile.
Once the wanted seat height is defined, the server sends a JSON message to the on-board computer or NUC of the E-Bike. A Robot Operating System (ROS) action server is running on the on-board computer. It intercepts the message and sets the "wanted seat height" as the "goal" value of a new action. This goal is published to the CAN bus of the vehicle.
The microcontroller, responsible for adjusting the seat height, is also connected to the CAN bus and reads the new message. Depending on the current seat height, it determines whether to raise or lower the seat in order to reach the goal height. The linear actuator, used to reposition the seat, has a feedback output which we use to determine the current seat height. This feedback is read by the microcontroller and sent via the ROS server all the way back to the web server, in order to notify the client of the status of their request.
Here is a very outdated block diagram of the code for the web app, ROS server and microcontroller, which attempts to visually explain what was previously written:
![Block Diagram](/assets/EN/Code_Block_Diagrams.png)

In order to keep the project more self contained and portable for educational purposes, the web client, web server and ROS server are all located on one laptop computer. This can be seen as Bl.9 on the following schematic diagram:
![Schematic Diagram](/assets/EN/Schematic_Diagram.png)

Instead of a seperate relational database management system, we use IndexedDB. "IndexedDB is a low-level API for client-side storage of significant amounts of structured data, including files/blobs."[src](https://developer.mozilla.org/en-US/docs/Web/API/IndexedDB_API), so it's a database built into most modern browsers. One user was hardcoded into the database of this project and his information can be used to login to the personalised [EbmsWebClient](catkin_ws/src/e_bike_memory_seat/src/client/EbmsWebClient.html) page. The registration page was not completed because it was out of the scope of this project. 

Once the user has logged in, they can choose via radio button one of two options: Automatically or Manually adjusted seat height. The first option calculates the optimal seat height using the LeMond method we mentioned earlier, whereas the second option allows the user to manually enter a number between 0 and 150mm, which is the movement range of the linear actuator. Once the user clicks on the "Order E-Bike" button, their "wanted seat height" is sent as a JSON message from the [web server](catkin_ws/src/e_bike_memory_seat/src/client/js/EbmsWebClient.js) to the [ROS action server](catkin_ws/src/e_bike_memory_seat/src/server/EbmsActionServer.cpp).

The ROS action server extracts the wanted seat height from the JSON message and writes it into a new CAN message before publishing it to the CAN bus. It then starts listening for feedback from the microcontroller on the CAN bus for the current seat height until it is equal to the wanted seat height. The ROS server sends the current seat height to the web server, where it is visualised for the user in real time. This allows the user to keep track of the status of their request.

The Microcontroller has several important functions. It was already established that it reads messages from the CAN bus, drives the linear actuator acordingly and publish feedback to the CAN bus, however it has a few safety features as well. It protects the linear actuator from overheating by keeping track of how long it has been powered on and ensuring enough time is spent powered off. This time is calculated given the 25% duty cycle of the actuator, which means that for every second it spends powered on, 4 seconds are spent powered off. Another requirement of the actuator is the maximum duty operational time, which is 1 minute at nominal dynamic load. The microcontroller keeps track of that as well. Finally, the microcontroller also provides stall protection by reading the positional feedback of the actuator. If it does not move for more than 26ms while it is powered on then the microcontroller cuts the power.


## How to build the project
To build the project change directory to catkin_ws and use:
```
$ catkin_make 
```
If you add a new package to the already compiled workspace use:
```
$ catkin_make --force-cmake
```
For more information visit the ros wiki catkin tutorial [here](http://wiki.ros.org/catkin/Tutorials/using_a_workspace).


## How to run the project

### Prerequisites:
- Ubuntu 18.04.6 LTS
- ROS Melodic
#### 1. Setup your SocketCan interface
```
$ sudo ip link set can0 down
$ sudo ip link set can0 up type can bitrate 500000
$ sudo ifconfig can0 txqueuelen 1000
```
#### 2. Setup your environment:
```
$ source catkin_ws/devel/setup.bash
```
setup.bash -- Environment setup file for Bash shell<br />
setup.sh   -- Environment setup file for Bourne shell<br />
setup.zsh  -- Environment setup file for zshell<br />
For more information visit the ros wiki [here](http://wiki.ros.org/catkin/workspaces#Source_Space).

#### 3. Start the ROS core, action server and rosbridge websocket:
```
$ roscore
```
(Then in a separate terminal make sure to setup your environment and execute the following line)
```
$ rosrun e_bike_memory_seat ebmsActionServer
```
(Then in a separate terminal make sure to setup your environment and execute the following line)
```
$ roslaunch rosbridge_server rosbridge_websocket.launch
```
By the way if you don't want to execute setup.bash in every new terminal you should change your ~/.bashrc by adding "source {path to your project}/catkin_ws/devel/setup.bash" to the end of the file. Just make sure to change it when you start working on a different project.

#### 4. Application is ready to be used:
Open the [EbmsLoginPage](catkin_ws/src/e_bike_memory_seat/src/client/EbmsLoginPage.html) with a browser that supports IndexedDB and start using the application with the sample user data found in the [DatabaseFunctions](catkin_ws/src/e_bike_memory_seat/src/client/js/DatabaseFunctions.js) js file.

## How to set up the arduino microcontroller
The code for the microcontroller is located in [this folder](catkin_ws/src/e_bike_memory_seat/src/microcontroller/EbmsMicrocontroller/EbmsMicrocontroller.ino).
Verify and Upload it to your Arduino Uno microcontroller and proceed to the next section [## How to set up the test bench](#how-to-set-up-the-test-bench) for wiring instructions.

## How to set up the test bench
Most of the components, excluding the power supply, are conveniently attached to the test bench. For wiring instructions one can refer to the detailed schematic diagram in the section [## How does it work?](#how-does-it-work) or the simplified schematic diagram bellow:
![Simplified Schematic Diagram](/assets/EN/Simplified_Schematic_Diagram.png)

The microcontroller can be powered externally, or it can be powered via USB connection to a computer. The second option allows us to reprogram the microcontroller as well, which makes it the better option for lab experiments. The computer marked as U1 can be replaced by a different brand and model, the most significant requirement is that it runs Ubuntu 18.04.6 LTS and ROS Melodic. Compatability with other versions of ROS and Ubuntu are not guarenteed.

## How to use the test bench
Afer the components have been wired up and the programs have been started, open the [EbmsLoginPage](catkin_ws/src/e_bike_memory_seat/src/client/EbmsLoginPage.html) with a browser which supports IndexedDB. Enter the email address of the user which has been hardcoded in the [DatabaseFunctions.js](catkin_ws/src/e_bike_memory_seat/src/client/js/DatabaseFunctions.js) file. Then you will be redirected to the [EbmsWebClient](catkin_ws/src/e_bike_memory_seat/src/client/EbmsWebClient.html) page where you can choose automatic or manual seat height adjustment. Experiment with both options to see how they work. When you are ready click the "Order E-Bike" button. This will send your request to the backend web server which will forward it to the ROS action server until your desired seat height reaches the microcontroller. You can follow the transmission of messages through the browser console, the ROS topics and the microcontroller serial monitor. The list of ROS topics can be seen by typing:
```
rostopic list
```
in the terminal of the computer running the ROS action server. One can then use:
```
rostopic echo /topic_name
```
to view the messages on that topic.
The messages exchanged between the ROS action server and microcontroller over the CAN bus can be observed by using the tool "candump" from can-utils. Proceed to the next section for more information.

## How to test the application
You can use can-utils to view and publish messages to the CAN bus. By publishing a message with a specific ID one can simulate a command sent from the server of feedback sent from the microcontroller. This allows the user to test different parts of the project without having to make all the connections.
```
$ candump can0 //read all published messages on can0
$ cansend can0 555#57.00.00.00.00.00.00.00 //simulate the server sending a goal height.
$ cansend can0 111#57.00.00.00.00.00.00.00 //simulate the microcontroller sending feedback.
```
For more information visit the "mbedded blog" [here](https://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-the-command-line-in-linux/).

## Possible future improvements and educational tasks
- Electronics
  - Add a display which provides useful information to the client such as:
    - Current-seat-height / goal-seat-height;
    - Actuator cooldown period;
  - LEDs denoting the direction of travel for the actuator, e.g. green when it's raising the seat, red when it's lowering the seat.
  - Blinking LEDs when the actuator is moving as a safety feature.

- Computer Science
  - Replace IndexedDB with a relational database management system (RDBMS).
  - Seperate the client side from the server side. Setup an online web server which can be accessed by remote devices such as computers, smartphones etc.
  - Implement a working registration page.