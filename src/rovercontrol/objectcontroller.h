#ifndef OBJECTCONTROLLER_H
#define OBJECTCONTROLLER_H

#include "controller.h"
#include <std_msgs/Float64.h>

class ObjectController : public Controller
{
public:
    ObjectController(RosController* rosController);

    void calculateYT();

private:
    ros::Subscriber subSteering;
    ros::Subscriber subThrottle;

    void onIncomingSteering(std_msgs::Float64::ConstPtr msg);
    void onIncomingThrottle(std_msgs::Float64::ConstPtr msg);
};

#endif // OBJECTCONTROLLER_H
