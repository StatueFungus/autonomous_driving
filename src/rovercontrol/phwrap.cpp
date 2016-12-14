#include "phwrap.h"

PHwrap::PHwrap(RosController *rosController) :
    currentState_(PixhawkState::Off)
    , isArmed_(false)
    , isInOffboard_(false)
    , isConnected_(false)
{
    lastRequest_ = ros::Time::now();
    armingClient_ = rosController->getNodeHandle()->serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    modeClient_ = rosController->getNodeHandle()->serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
    subMavState_ = rosController->getNodeHandle()->subscribe("mavros/state", 1, &PHwrap::onIncomingMavrosState, this);
    actuator_controls_pub = rosController->getNodeHandle()->advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);

    actuator_control_msg.group_mix = 0;
    actuator_control_msg.controls[0] = 0.0;
    actuator_control_msg.controls[1] = 0.0;
    actuator_control_msg.controls[2] = 0.0;
    actuator_control_msg.controls[3] = 0.0;
    actuator_control_msg.controls[4] = 0.0;
    actuator_control_msg.controls[5] = 0.0;
    actuator_control_msg.controls[6] = 0.0;
    actuator_control_msg.controls[7] = 0.0;

    timer_ = rosController->getNodeHandle()->createTimer(ros::Rate(20), &PHwrap::timerCallback,this,false);
    timer_.start();

     std::cout << "Pixhawk_Wrapper State: Off" << std::endl;
}

PHwrap::~PHwrap()
{

}

void PHwrap::disable()
{
    enabled_ = false;
    std::cout << "Pixhawk_Wrapper State: Off" << std::endl;

    offb_set_mode.request.custom_mode = "MANUAL";
    if( modeClient_.call(offb_set_mode) && offb_set_mode.response.success){
        ROS_INFO("Manual enabled");
    }

    setOutput(0,0,0,0,true);
}

void PHwrap::setOutput(double roll, double pitch, double yaw, double throttle, bool overwrite)
{
    if(currentState_ == PixhawkState::Driving ||
       overwrite)
    {
        std::lock_guard<std::mutex> guard(actuatorSending_);

        actuator_control_msg.controls[0] = roll;
        actuator_control_msg.controls[1] = pitch;
        actuator_control_msg.controls[2] = yaw;
        actuator_control_msg.controls[3] = throttle;
    }
}

void PHwrap::update()
{
    if (!this->enabled_) {
        currentState_ = PixhawkState::Off;
    }

    switch(currentState_) {
    case PixhawkState::Off:
        setOutput(0,0,0,0,true);

        if(this->isEnabled())
        {
            currentState_ = PixhawkState::Connecting;
            std::cout << "Pixhawk_Wrapper State: Connecting" << std::endl;
        }
    break;
    case PixhawkState::Connecting:
        if(isConnected_) {
            currentState_ = PixhawkState::SettingMode;
            std::cout << "Pixhawk_Wrapper State: SettingMode" << std::endl;
        }
        break;

    case PixhawkState::SettingMode:
        if(isInOffboard_) {
            currentState_ = PixhawkState::Arming;
            std::cout << "Pixhawk_Wrapper State: Arming" << std::endl;
        }
        else if(ros::Time::now() - lastRequest_ > ros::Duration(5.0)) {
            lastRequest_ = ros::Time::now();
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if( modeClient_.call(offb_set_mode) && offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
        }
        break;
    case PixhawkState::Arming:
        if(isArmed_) {
            // Go to driving
            currentState_ = PixhawkState::Driving;
            std::cout << "Pixhawk_Wrapper State: Driving" << std::endl;
        }
        else if(ros::Time::now() - lastRequest_ > ros::Duration(5.0)) {
            lastRequest_ = ros::Time::now();
            arm_cmd.request.value = true; // should be true when in automatic mode
            if( armingClient_.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
            }
        }
        break;
    case PixhawkState::Driving:
        // do nothing, let controller work!
        if(!isReadyToDrive()) {
                    currentState_ = PixhawkState::Off;
                    std::cout << "Pixhawk_Wrapper State: Off" << std::endl;
        }
        break;
    }
}

bool PHwrap::isReadyToDrive() const
{
    bool ready = false;
    if(isInOffboard_ && isArmed_) {
        ready = true;
    }
    return ready;
}

void PHwrap::setOutputRate(int rateHz)
{
    timer_.setPeriod(ros::Duration(ros::Rate(rateHz)));
}

void PHwrap::onIncomingMavrosState(mavros_msgs::State::ConstPtr msg)
{
    std::string offboardMode("OFFBOARD");
    bool newMode = (offboardMode.compare(msg->mode) == 0);
    if (!isInOffboard_ || newMode != isInOffboard_) {
        std::cout << "Flightcontroller State: " << msg->mode << std::endl;
    }
    if (enabled_) {
        isArmed_ = msg->armed;
        isInOffboard_ = newMode;
        isConnected_ = msg->connected;
    } else {
        isArmed_ = false;
        isInOffboard_ = false;
        isConnected_ = false;
    }

    this->signal();
}

void PHwrap::timerCallback(const TimerEvent &e)
{
    std::lock_guard<std::mutex> guard(actuatorSending_);
    actuator_controls_pub.publish(actuator_control_msg);

    // Pixhawk start-up phase
    if (currentState_ == PixhawkState::Off ||
        currentState_ == PixhawkState::Connecting ||
        currentState_ == PixhawkState::SettingMode ||
        currentState_ == PixhawkState::Arming)
    {
            this->signal();
    }
}
