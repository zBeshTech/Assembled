#include "sensor.h"

bumper::bumper(ros::NodeHandle& nh, std::string name, float x, float y, float z): 
bumper_sub(nh.subscribe("/" + name, 1, &bumper::bumperCallback, this)),
bumper_state(released),
Sensor(nh, name, x, y, z){}

void bumper::bumperCallback(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data == true)
    {
        ROS_WARN("bumperCallback: Bumper %s is pressed", name.c_str());
        bumper_state = pressed;
    }
    else
    {
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
    return std::make_tuple(x, y, z);;
}

std::string bumper::getName()
{
    return Sensor::name;
}