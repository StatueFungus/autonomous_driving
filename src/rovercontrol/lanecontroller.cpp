#include "lanecontroller.h"

using namespace std;


LaneController::LaneController(RosController* rosController)
{
    // Subscriber for throttle and yaw
    subSteering = rosController->getNodeHandle()->subscribe("lanecontroller/steering", 1, &LaneController::onIncomingSteering, this);
    subThrottle = rosController->getNodeHandle()->subscribe("lanecontroller/throttle", 1, &LaneController::onIncomingThrottle, this);
}

void LaneController::calculateYT()
{
    // Implemented in the python nodes
}

void LaneController::onIncomingSteering(std_msgs::Float64::ConstPtr msg)
{
    setSteering(msg->data);
    this->signal();
}

void LaneController::onIncomingThrottle(std_msgs::Float64::ConstPtr msg)
{
    setThrottle(msg->data);
    this->signal();
}
