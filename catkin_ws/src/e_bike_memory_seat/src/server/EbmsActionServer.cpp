#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <e_bike_memory_seat/adjustSeatHeightAction.h>
#include <string>
#include "canTransmit.h"

// 0x555 is the hex id of the ROS server
// 0x111 is the hex id of the Arduino microcontroller
#define CAN_ID_TO_SEND 0x555
#define CAN_ID_TO_READ 0x111

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
        }
    
    ~SeatHeightAdjuster(void) { }

    void executeCB(const e_bike_memory_seat::adjustSeatHeightGoalConstPtr &goal) {
        ros::Rate rate(10);

        ROS_INFO("Executing %s, wanted height is %i",
            actionName.c_str(),
            goal->wantedHeight
        );

        // send the goal height to the microcontroller by publishing it to the CAN bus
        if(!writeMessageToCan(goal->wantedHeight, CAN_ID_TO_SEND)) {
            ROS_ERROR("There was an error writting message to CAN bus.");
            actionServer.setAborted();
            return;
        }

        // read the current height 
        feedback.currentHeight = readMessageFromCanWithId(CAN_ID_TO_READ);

        // if there was an error reading message from the CAN bus, abort the action
        // ToDo what happens when reading is successful but the IDs don't match?
        if (feedback.currentHeight == -1) {
            ROS_ERROR("There was an error reading message from CAN bus.");
            actionServer.setAborted();
            return;
        }

        // keep reading and publishing the feedback messages sent from the microcontroller
        // until the feedback height is equal to the goal wanted height
        // or until the action is preempted
        // or until the microcontroller returns error with -1
        // TODO add timer condition
        // TODO what if feedback overshoots goal and has to go in the other direction?
        while (feedback.currentHeight != goal->wantedHeight) {
            if (actionServer.isPreemptRequested()
                || !ros::ok()
                || feedback.currentHeight == -1) {

                ROS_WARN("%s was preempted", actionName.c_str());
                actionServer.setPreempted();
                break;
            }
            feedback.currentHeight = readMessageFromCanWithId(CAN_ID_TO_READ);
            actionServer.publishFeedback(feedback);
            rate.sleep();

            // TODO what happens if the arduino returns error during operation?
            // maybe include status info in the messages.
            // TODO what happens when the arduino is "resting" ?
        }

        // if the current height is equal to the goal height,
        // set the action status to succeeded.
        if (feedback.currentHeight == goal->wantedHeight) {
            result.finalHeight = feedback.currentHeight;
            ROS_INFO("%s Succeeded :)", actionName.c_str());
            actionServer.setSucceeded(result);
        }     
        else {
            ROS_WARN("%s not completed", actionName.c_str());
            result.finalHeight = feedback.currentHeight; // TODO not sure if I should set result if the action is aborted. Test this out.
            actionServer.setAborted();
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