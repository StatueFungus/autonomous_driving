#ifndef ROSCONTROLLER_H
#define ROSCONTROLLER_H

#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
using namespace ros;


/**
 * @brief The RosController class.
 * General class for everything about ros.
 * Here we initalize our ros node and our nodehandler object for handling all subscribers and publishers, callbacks etc. in our project.
 * If you create additional classes which should communicate over ros, pass an instance of RosController in its constructor.
 * IMPORTANT: Check out the following links and read wiki.ros.org.
 * http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers.
 * http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning.
 */
class RosController{
public:

    /**
     * The spinner triggers the callback functions.
     * IMPORTANT: Check out the documentation at wiki.ros.org.
     * http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
     * http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
     */
    ros::AsyncSpinner* spinner;

    /**
     * RosController constructor.
     */
    RosController(){}

    /**
     * RosController destructor.
     */
    ~RosController()
    {
        if(spinner) delete spinner;
        if(nodeHandle) delete nodeHandle;
    }

    /**
     * The method init initializes the ROS node, the NodeHandle is like a controller for everything in ROS (e.g. subscriber and publisher etc.)
     * The AsyncSpinner is a multithreaded variant to trigger all of your callback functions. It's necessary to get the latest value out of the callback queues.
     * For detailed explanation check out hda drone manual and wiki.ros.org.
     * http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
     * http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
     * @param argc number of arguments
     * @param argv here you can pass arguments to ROS initialization
     * @param nodeName here you have to pass a name for the ROS Node
     */
    void init(int argc, char** argv, std::string nodeName){
        ros::init(argc, argv, nodeName);
        nodeHandle = new ros::NodeHandle();
        spinner = new AsyncSpinner(3);
    }

    /**
     * Starts the spinner.
     */
    void start(){
        spinner->start();
    }

    /**
     * Stops the spinner.
     */
    void stop(){
        spinner->stop();
    }

    /**
     * Getter for the NodeHandle.
     */
    NodeHandle* getNodeHandle(){
        return nodeHandle;
    }
private:

    /** Private NodeHandle
     * The NodeHandle is like a controller for everything in ROS (e.g. subscriber and publisher etc.)
     */
    NodeHandle* nodeHandle;
};
#endif
