#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <e_bike_memory_seat/adjustSeatHeightAction.h>
#include <string>
#include "canTransmit.h"

class SeatHeightAdjuster {

    protected:
    ros::NodeHandle rosNode;
    actionlib::SimpleActionServer<e_bike_memory_seat::adjustSeatHeightAction> actionServer;
    std::string actionName;
    e_bike_memory_seat::adjustSeatHeightFeedback feedback;
    e_bike_memory_seat::adjustSeatHeightResult result;

    public:
    SeatHeightAdjuster(std::string name) :
        actionServer(rosNode, name, boost::bind(&SeatHeightAdjuster::executeCB, this, _1), false),
        actionName(name)
        {
            actionServer.start();
            feedback.currentHeight = 0;
        }
    
    ~SeatHeightAdjuster(void) { }

    void executeCB(const e_bike_memory_seat::adjustSeatHeightGoalConstPtr &goal) {
        ros::Rate rate(10);
        bool operationSuccessful = true;

        // TODO get the current seat height by querying the arduino mcu
        // ...

        ROS_INFO("Executing %s, current seat height is %i, wanted height is %i",
            actionName.c_str(),
            feedback.currentHeight,
            goal->wantedHeight
        );

        // if the current height is already equal to the goal height,
        // set the action status to succeeded.
        if (feedback.currentHeight == goal->wantedHeight) {
            actionServer.publishFeedback(feedback);
            result.finalHeight = feedback.currentHeight;
            ROS_INFO("%s Succeeded :)", actionName.c_str());
            actionServer.setSucceeded(result);
        }

        // If the current seat height is lower than the goal,
        // lift the seat to the goal height.
        else if (feedback.currentHeight < goal->wantedHeight) {

            // Send raise seat command to the arduino mcu
            writeMessageToCan(std::to_string(goal->wantedHeight), 'r');
            
            while (feedback.currentHeight < goal->wantedHeight) {
                if (actionServer.isPreemptRequested() || !ros::ok()) {
                    ROS_WARN("%s was preempted", actionName.c_str());
                    actionServer.setPreempted();
                    operationSuccessful = false;
                    break;
                }

                // TODO read current height from the arduino
                feedback.currentHeight++;

                actionServer.publishFeedback(feedback);
                rate.sleep();

                // TODO if the arduino returns error during operation,
                // set the action server status to aborted
            }
        }

        // If the current seat height is higher than the goal,
        // lower the seat to the goal height.
        else {

            // Send lower seat command to the arduino mcu
            writeMessageToCan(std::to_string(goal->wantedHeight), 'l');

            while (feedback.currentHeight > goal->wantedHeight) {
                if (actionServer.isPreemptRequested() || !ros::ok()) {
                    ROS_WARN("%s was preempted", actionName.c_str());
                    actionServer.setPreempted();
                    operationSuccessful = false;
                    break;
                }

                // TODO read current height from the arduino
                feedback.currentHeight--;

                actionServer.publishFeedback(feedback);
                rate.sleep();

                // TODO if the arduino returns error during operation,
                // set the action server status to aborted
            }
        }

        if (operationSuccessful) {
            result.finalHeight = feedback.currentHeight;
            ROS_INFO("%s Succeeded :)", actionName.c_str());
            actionServer.setSucceeded(result);

            // dirty hack to stop the motor by sending 's' to MCU
            writeMessageToCan(std::to_string(goal->wantedHeight), 's');
        }        
    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "ebmsRosNode");

    SeatHeightAdjuster seatHeightAdjuster("ebmsRosNode");
    ROS_INFO("EBMS Action Server has started");
    ros::spin();

    return 0;
}