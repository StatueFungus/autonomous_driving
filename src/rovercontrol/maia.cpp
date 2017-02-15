#include "maia.h"
#include "lanecontroller.h"
#include "objectcontroller.h"
#include "pixhawk.h"

Maia::Maia(RosController* ros)
    : laneSteeringDisabled(false), laneThrottleDisabled(false), laneDisableDuration(ros::Duration(0.4))
{
    pixhawk = new Pixhawk(ros);
    lanecontroller = new LaneController(ros);
    objectcontroller = new ObjectController(ros);

    timerLaneDisableSteering = ros->getNodeHandle()->createTimer(laneDisableDuration,
                                                                 &Maia::laneDisableSteeringCallback,this,true,false);
    timerLaneDisableThrottle = ros->getNodeHandle()->createTimer(laneDisableDuration,
                                                                 &Maia::laneDisableThrottleCallback,this,true,false);
}

Maia::~Maia()
{
    delete pixhawk;
    delete lanecontroller;
    delete objectcontroller;
}

void Maia::laneDisableSteeringCallback(const ros::TimerEvent& e)
{
    laneSteeringDisabled = false;
}

void Maia::laneDisableThrottleCallback(const ros::TimerEvent& e)
{
    laneThrottleDisabled = false;
}

void Maia::runMAIA()
{
    pixhawk->setSignal(&communicationLock, &inputAvailable);
    lanecontroller->setSignal(&communicationLock, &inputAvailable);
    objectcontroller->setSignal(&communicationLock, &inputAvailable);

    std::unique_lock<std::mutex> lock(communicationLock);

    while(ros::ok()) {
        // temporarily unlock mutex:
        bool timeOut = std::cv_status::timeout == inputAvailable.wait_for(lock, std::chrono::milliseconds(500));

        if (timeOut) {
            std::cout << "No ROS-Input: Emergency shutdown initiated!" << std::endl;
            if(pixhawk->isEnabled())
            {
                pixhawk->disable();
                pixhawk->update();
            }
        }

        bool updFlightcontroller = pixhawk->bUpdated();
        bool updLaneController = lanecontroller->bUpdated();
        bool updObjectController = objectcontroller->bUpdated();

        if (updLaneController) {
            if(lanecontroller->bUpdatedSteering() && !laneSteeringDisabled) {
                pixhawk->setSteering(lanecontroller->fetchSteering());
            }
            if(lanecontroller->bUpdatedThrottle() && !laneThrottleDisabled) {
                pixhawk->setThrottle(lanecontroller->fetchThrottle());
            }
        }
        // Object Controller
        if (updObjectController) {
            if(objectcontroller->bUpdatedSteering()) {
                pixhawk->setSteering(objectcontroller->fetchSteering());
                laneSteeringDisabled = true;
                timerLaneDisableSteering.setPeriod(laneDisableDuration, true);
                timerLaneDisableSteering.start();
            }
            if(objectcontroller->bUpdatedThrottle()) {
                pixhawk->setThrottle(objectcontroller->fetchThrottle());
                laneThrottleDisabled = true;
                timerLaneDisableThrottle.setPeriod(laneDisableDuration, true);
                timerLaneDisableThrottle.start();
            }
        }

        if(updLaneController || updObjectController) {
            if(!pixhawk->isEnabled()) {
                pixhawk->enable();
                pixhawk->update();
            }
        }

        if (updFlightcontroller) {
            pixhawk->update();
        }
    }

    // Shutdown
    std::cout << "Shutdown: disable flightcontroller" << std::endl;
    pixhawk->disable();
    pixhawk->update();
}
