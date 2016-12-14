#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "communication.h"

class Controller : public Communication {
public:
    Controller() {}
    virtual ~Controller() {}

    virtual void calculateYT() = 0; // Berechne Yaw Throttle async. mit getActuatorControl

    float getSteering() const { return desiredSteering;}
    float getThrottle() const { return desiredThrottle;}

protected:
    float desiredSteering;
    float desiredThrottle;
};

#endif // CONTROLLER_H
