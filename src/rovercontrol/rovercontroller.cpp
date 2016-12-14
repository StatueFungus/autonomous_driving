#include "rovercontroller.h"

using namespace std;


RoverController::RoverController(RosController* rosController)
{
    // Subscriber for throttle and yaw
    subRoverControl = rosController->getNodeHandle()->subscribe("rovercontrol/rover_control", 1, &RoverController::onIncomingRoverControl, this);
}

 RoverController::~RoverController()
{
}

void RoverController::calculateYT()
{
    // Implemented in the python nodes
}

void RoverController::onIncomingRoverControl( mavros_msgs::ActuatorControl::ConstPtr msg)
{
    desiredSteering = msg->controls[0];
    desiredThrottle = msg->controls[3];

    this->signal();
}
