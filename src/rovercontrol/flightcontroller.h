#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "communication.h"

class Flightcontroller : public Communication
{
public:
    Flightcontroller() : enabled_(false){}
    virtual ~Flightcontroller() {}

    virtual void setOutput(double roll, double pitch, double yaw, double throttle, bool overwrite = false) = 0;
    virtual void setOutputRate(int rateHz) {}

    virtual bool isReadyToDrive() const { return false; }

    virtual void enable() {enabled_ = true;}
    virtual void disable() {enabled_ = false;}
    bool isEnabled() const {return enabled_;}

    virtual void update() = 0;
    
protected:
    bool enabled_;

};
#endif // FLIGHTCONTROLLER_H
