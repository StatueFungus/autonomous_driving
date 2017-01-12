#include "pixhawk.h"

Pixhawk::Pixhawk(RosController *rosController) :
    currentState(PixhawkState::Off)
    , isArmed(false)
    , isInOffboard(false)
    , isConnected(false)
{
    lastRequest = ros::Time::now();
    armingClient = rosController->getNodeHandle()->serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    modeClient = rosController->getNodeHandle()->serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
    subMavState = rosController->getNodeHandle()->subscribe("mavros/state", 1, &Pixhawk::onIncomingMavrosState, this);
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

    timer = rosController->getNodeHandle()->createTimer(ros::Rate(30), &Pixhawk::timerCallback,this,false);
    timer.start();

     std::cout << "Pixhawk_Wrapper State: Off" << std::endl;
     std::cout << "Flightcontroller State: MANUAL" << std::endl;
}

void Pixhawk::disable()
{
    enabled = false;
    std::cout << "Pixhawk_Wrapper State: Off" << std::endl;

    offb_set_mode.request.custom_mode = "MANUAL";
    if( modeClient.call(offb_set_mode) && offb_set_mode.response.success){
        ROS_INFO("Manual enabled");
    }

    setOutput(0,0,0,0,true);
}

void Pixhawk::setOutput(double roll, double pitch, double yaw, double throttle, bool overwrite)
{
    if(currentState == PixhawkState::Driving ||
       overwrite)
    {
        std::lock_guard<std::mutex> guard(actuatorSending);

        actuator_control_msg.controls[0] = roll;
        actuator_control_msg.controls[1] = pitch;
        actuator_control_msg.controls[2] = yaw;
        actuator_control_msg.controls[3] = throttle;
    }
}

void Pixhawk::setThrottle(double throttle, bool overwrite)
{
    if(currentState == PixhawkState::Driving ||
       overwrite)
    {
        std::lock_guard<std::mutex> guard(actuatorSending);
        actuator_control_msg.controls[3] = throttle;
    }
}

void Pixhawk::setSteering(double steering, bool overwrite)
{
    if(currentState == PixhawkState::Driving ||
       overwrite)
    {
        std::lock_guard<std::mutex> guard(actuatorSending);
        actuator_control_msg.controls[0] = steering;
    }
}

void Pixhawk::update()
{
    if (!this->enabled) {
        currentState = PixhawkState::Off;
    }

    switch(currentState) {
    case PixhawkState::Off:
        setOutput(0,0,0,0,true);

        if(this->isEnabled())
        {
            currentState = PixhawkState::Connecting;
            std::cout << "Pixhawk_Wrapper State: Connecting" << std::endl;
        }
    break;
    case PixhawkState::Connecting:
        if(isConnected) {
            currentState = PixhawkState::SettingMode;
            std::cout << "Pixhawk_Wrapper State: SettingMode" << std::endl;
        }
        break;

    case PixhawkState::SettingMode:
        if(isInOffboard) {
            currentState = PixhawkState::Arming;
            std::cout << "Pixhawk_Wrapper State: Arming" << std::endl;
        }
        else if(ros::Time::now() - lastRequest > ros::Duration(5.0)) {
            lastRequest = ros::Time::now();
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if( modeClient.call(offb_set_mode) && offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
        }
        break;
    case PixhawkState::Arming:
        if(isArmed) {
            // Go to driving
            currentState = PixhawkState::Driving;
            std::cout << "Pixhawk_Wrapper State: Driving" << std::endl;
        }
        else if(ros::Time::now() - lastRequest > ros::Duration(5.0)) {
            lastRequest = ros::Time::now();
            arm_cmd.request.value = true; // should be true when in automatic mode
            if( armingClient.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
            }
        }
        break;
    case PixhawkState::Driving:
        // do nothing, let controller work!
        if(!isReadyToDrive()) {
                    currentState = PixhawkState::Off;
                    std::cout << "Pixhawk_Wrapper State: Off" << std::endl;
        }
        break;
    }
}

bool Pixhawk::isReadyToDrive() const
{
    bool ready = false;
    if(isInOffboard && isArmed) {
        ready = true;
    }
    return ready;
}

void Pixhawk::setOutputRate(int rateHz)
{
    timer.setPeriod(ros::Duration(ros::Rate(rateHz)));
}

void Pixhawk::onIncomingMavrosState(mavros_msgs::State::ConstPtr msg)
{
    std::string offboardMode("OFFBOARD");
    bool newMode = (offboardMode.compare(msg->mode) == 0);
    if ((isInOffboard == false || newMode != isInOffboard) && enabled) {
        std::cout << "Flightcontroller State: " << msg->mode << std::endl;
    }

    if (enabled) {
        isArmed = msg->armed;
        isInOffboard = newMode;
        isConnected = msg->connected;
    } else {
        isArmed = false;
        isInOffboard = false;
        isConnected = false;
    }
}

void Pixhawk::timerCallback(const TimerEvent &e)
{
    std::lock_guard<std::mutex> guard(actuatorSending);
    actuator_controls_pub.publish(actuator_control_msg);

    // Pixhawk start-up phase
    if (currentState == PixhawkState::Off ||
        currentState == PixhawkState::Connecting ||
        currentState == PixhawkState::SettingMode ||
        currentState == PixhawkState::Arming)
    {
            this->signal();
    }
}
