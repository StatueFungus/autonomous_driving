#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "communication.h"

class Controller : public Communication {
public:
    Controller();
    virtual ~Controller() {}

    virtual void calculateYT() = 0; // Berechne Yaw Throttle.

    float fetchSteering(bool setUpdatedFalse = true);
    float fetchThrottle(bool setUpdatedFalse = true);

    bool bUpdatedSteering() const;
    bool bUpdatedThrottle() const;

protected:
    bool updatedSteering;
    bool updatedThrottle;

    void setSteering(float steering);
    void setThrottle(float throttle);

private:
    float desiredSteering;
    float desiredThrottle;
};

#endif // CONTROLLER_H
