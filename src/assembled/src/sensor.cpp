#include "sensor.h"
#include "assembled.h"

bumper::bumper(ros::NodeHandle& nh, std::string name, float x, float y, float z, Assembled& assembled): 
bumper_sub(nh.subscribe("/" + name, 1, &bumper::bumperCallback, this)),
bumper_state(released),
Sensor(nh, name, x, y, z, assembled){}

void bumper::bumperCallback(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data == true)
    {
        if (bumper_state == released){
        ROS_WARN("bumperCallback: Bumper %s is pressed, changing state from released to pressed", name.c_str());
        assembled.performRecoveryRoutine(*this);
        }
        bumper_state = pressed;
    }
    else
    {
        if (bumper_state == pressed){
            ROS_WARN("bumperCallback: Bumper %s is released", name.c_str());
            assembled.changeState(Assembled::State::IDLE);
        }
        bumper_state = released;
    }
}

bool bumper::isTriggered(){
    if (bumper_state == pressed)
    {
        return true;
    }
    else
    {
        return false;
    }
}
std::tuple <float, float, float> bumper::getCoordinates() 
{
    return std::make_tuple(x, y, z);
}

std::string bumper::getName()
{
    return Sensor::name;
}