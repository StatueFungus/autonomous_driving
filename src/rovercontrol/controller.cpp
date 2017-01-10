#include "controller.h"
#include <thread>
using namespace std;

Controller::Controller()
    : desiredSteering(0.0), desiredThrottle(0.0), updatedSteering(false), updatedThrottle(false)
{
}

float Controller::fetchSteering(bool setUpdatedFalse)
{
    if(setUpdatedFalse) updatedSteering = false;
    return desiredSteering;
}
float Controller::fetchThrottle(bool setUpdatedFalse)
{
    if(setUpdatedFalse) updatedThrottle = false;
    return desiredThrottle;
}

bool Controller::bUpdatedSteering() const
{
    return updatedSteering;
}

bool Controller::bUpdatedThrottle() const
{
     return updatedThrottle;
}

void Controller::setSteering(float steering)
{
    desiredSteering = steering;
    updatedSteering = true;
}

void Controller::setThrottle(float throttle)
{
    desiredThrottle = throttle;
    updatedThrottle = true;
}
