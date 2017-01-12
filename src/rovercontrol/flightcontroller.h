#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "communication.h"

class Flightcontroller : public Communication
{
public:
    Flightcontroller() : enabled(false){}
    virtual ~Flightcontroller() {}

    virtual void setOutput(double roll, double pitch, double yaw, double throttle, bool overwrite = false) = 0;
    virtual void setThrottle(double throttle, bool overwrite = false) {};
    virtual void setSteering(double steering, bool overwrite = false) {};
    virtual void setOutputRate(int rateHz) {}

    virtual bool isReadyToDrive() const { return false; }

    virtual void enable() {enabled = true;}
    virtual void disable() {enabled = false;}
    bool isEnabled() const {return enabled;}

    virtual void update() = 0;
    
protected:
    bool enabled;

};
#endif // FLIGHTCONTROLLER_H
