#include "rovercontroller.h"

using namespace std;


RoverController::RoverController(RosController* rosController)
{
    // Subscriber for throttle and yaw
    subSteering = rosController->getNodeHandle()->subscribe("rovercontrol/steering", 1, &RoverController::onIncomingSteering, this);
    subThrottle = rosController->getNodeHandle()->subscribe("rovercontrol/throttle", 1, &RoverController::onIncomingThrottle, this);

    desiredSteering = 0.0;
    desiredThrottle = 0.0;
}

 RoverController::~RoverController()
{
}

void RoverController::calculateYT()
{
    // Implemented in the python nodes
}

void RoverController::onIncomingSteering(std_msgs::Float64::ConstPtr msg)
{
    desiredSteering = msg->data;

    this->signal();
}

void RoverController::onIncomingThrottle(std_msgs::Float64::ConstPtr msg)
{
    desiredThrottle = msg->data;

    this->signal();
}
