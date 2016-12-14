#include "maia.h"

Maia::Maia(RosController* ros)
    : flightcontroller_(nullptr),
      controller_(nullptr)
{
    flightcontroller_ = new PHwrap(ros);
    controller_ = new RoverController(ros);
}

Maia::~Maia()
{
    delete flightcontroller_;
    delete controller_;
}

void Maia::runMAIA()
{
    flightcontroller_->setSignal(&communicationLock_, &inputAvailable_);
    controller_->setSignal(&communicationLock_, &inputAvailable_);

    std::unique_lock<std::mutex> lock(communicationLock_);

    while(ros::ok()) {
        // temporarily unlock mutex:
        bool timeOut = std::cv_status::timeout == inputAvailable_.wait_for(lock, std::chrono::milliseconds(500));

        if (timeOut) {
            std::cout << "No ROS-Input: Emergency shutdown initiated!" << std::endl;
            if(flightcontroller_->isEnabled())
            {
                flightcontroller_->disable();
                flightcontroller_->update();
            }
        }

        bool updFlightcontroller = flightcontroller_->updated();
        bool updController = controller_->updated();

        if (updController) {
            // Set Throttle and Steering directly
            flightcontroller_->setOutput(controller_->getSteering(), 0.0, 0.0, controller_->getThrottle());

            // Enable Flightcontroller
            if(!flightcontroller_->isEnabled())
            {
                flightcontroller_->enable();
                flightcontroller_->update();
            }
        }

        if (updFlightcontroller) {
            flightcontroller_->update();
        }
    }

    // Shutdown
    std::cout << "Shutdown: disable flightcontroller" << std::endl;
    flightcontroller_->disable();
    flightcontroller_->update();
}
