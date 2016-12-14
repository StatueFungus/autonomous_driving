#ifndef PHWRAP_H
#define PHWRAP_H

#include <iostream>

#include <ros/timer.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>


#include "flightcontroller.h"

class PHwrap : public Flightcontroller
{
public:
    PHwrap(RosController* rosController);
    ~PHwrap();

    void setOutput(double roll, double pitch, double yaw, double throttle, bool overwrite);
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

    ros::ServiceClient armingClient_;
    ros::ServiceClient modeClient_;
    ros::Subscriber subMavState_;
    ros::Publisher actuator_controls_pub;

    mavros_msgs::ActuatorControl actuator_control_msg;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::SetMode offb_set_mode;

    ros::Timer timer_;
    ros::Time lastRequest_;
    std::mutex actuatorSending_;

    PixhawkState currentState_;
    bool isArmed_;
    bool isInOffboard_;
    bool isConnected_;
};

#endif // PHWRAP_H
