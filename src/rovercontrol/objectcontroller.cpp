#include "objectcontroller.h"

using namespace std;


ObjectController::ObjectController(RosController* rosController)
{
    // Subscriber for throttle and yaw
    subSteering = rosController->getNodeHandle()->subscribe("objectcontroller/steering", 1, &ObjectController::onIncomingSteering, this);
    subThrottle = rosController->getNodeHandle()->subscribe("objectcontroller/throttle", 1, &ObjectController::onIncomingThrottle, this);
}

void ObjectController::calculateYT()
{
    // Implemented in the python nodes
}

void ObjectController::onIncomingSteering(std_msgs::Float64::ConstPtr msg)
{
    setSteering(msg->data);
    this->signal();
}

void ObjectController::onIncomingThrottle(std_msgs::Float64::ConstPtr msg)
{
    setThrottle(msg->data);
    this->signal();
}
