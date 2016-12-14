#ifndef ROVERCONTROLLER_H
#define ROVERCONTROLLER_H

#include "controller.h"
#include <mavros_msgs/ActuatorControl.h>

class RoverController : public Controller
{
public:
    RoverController(RosController* rosController);
    ~RoverController();

    void calculateYT();

private:
    ros::Subscriber subRoverControl;
    void onIncomingRoverControl(mavros_msgs::ActuatorControl::ConstPtr msg);
};

#endif // ROVERCONTROLLER_H
