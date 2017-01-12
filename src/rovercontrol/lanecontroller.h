#ifndef LANECONTROLLER_H
#define LANECONTROLLER_H

#include "controller.h"
#include <std_msgs/Float64.h>

class LaneController : public Controller
{
public:
    LaneController(RosController* rosController);

    void calculateYT();

private:
    ros::Subscriber subSteering;
    ros::Subscriber subThrottle;

    void onIncomingSteering(std_msgs::Float64::ConstPtr msg);
    void onIncomingThrottle(std_msgs::Float64::ConstPtr msg);
};

#endif // LANECONTROLLER_H
