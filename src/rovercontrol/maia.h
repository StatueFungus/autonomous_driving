#ifndef MAIA_H
#define MAIA_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include "controller.h"
#include "flightcontroller.h"

class Maia
{
public:
    Maia(RosController* ros);
    ~Maia();

    void runMAIA();

private:
    std::mutex communicationLock;
    std::condition_variable inputAvailable;

    //Communication Objects
    Flightcontroller *pixhawk;
    Controller *lanecontroller;
    Controller *objectcontroller;

    ros::Duration laneDisableDuration;
    ros::Timer timerLaneDisableSteering;
    void laneDisableSteeringCallback(const ros::TimerEvent& e);
    bool laneSteeringDisabled;

    ros::Timer timerLaneDisableThrottle;
    void laneDisableThrottleCallback(const ros::TimerEvent& e);
    bool laneThrottleDisabled;
};

#endif // MAIA_H
