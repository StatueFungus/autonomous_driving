#ifndef PHWRAP_H
#define PHWRAP_H

#include <iostream>

#include <ros/timer.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>


#include "flightcontroller.h"

class Pixhawk : public Flightcontroller
{
public:
    Pixhawk(RosController* rosController);

    void setOutput(double roll, double pitch, double yaw, double throttle, bool overwrite);
    void setThrottle(double throttle, bool overwrite);
    void setSteering(double steering, bool overwrite);
    void setOutputRate(int rateHz);

    virtual bool isReadyToDrive() const;

    virtual void update();
    virtual void disable();

private:

    enum class PixhawkState {
        Off,
        Connecting,
        Arming,
        SettingMode,
        Driving
    };

    void onIncomingMavrosState(mavros_msgs::State::ConstPtr msg);
    void timerCallback(const ros::TimerEvent& e);

    ros::ServiceClient armingClient;
    ros::ServiceClient modeClient;
    ros::Subscriber subMavState;
    ros::Publisher actuator_controls_pub;

    mavros_msgs::ActuatorControl actuator_control_msg;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::SetMode offb_set_mode;

    ros::Timer timer;
    ros::Time lastRequest;
    std::mutex actuatorSending;

    PixhawkState currentState;
    bool isArmed;
    bool isInOffboard;
    bool isConnected;
};

#endif // PHWRAP_H
