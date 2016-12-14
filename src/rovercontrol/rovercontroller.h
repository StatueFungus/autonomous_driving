#ifndef ROVERCONTROLLER_H
#define ROVERCONTROLLER_H

#include "controller.h"
#include <std_msgs/Float64.h>

class RoverController : public Controller
{
public:
    RoverController(RosController* rosController);
    ~RoverController();

    void calculateYT();

private:
    ros::Subscriber subSteering;
    ros::Subscriber subThrottle;

    void onIncomingSteering(std_msgs::Float64::ConstPtr msg);
    void onIncomingThrottle(std_msgs::Float64::ConstPtr msg);
};

#endif // ROVERCONTROLLER_H
