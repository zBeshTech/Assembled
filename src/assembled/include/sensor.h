#ifndef SENSOR_H
#define SENSOR_H

#include <string>
#include <tuple>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
class Sensor
{
protected:
    ros::NodeHandle& nh;
    std::string name;
    float x, y, z;
public:
    Sensor(ros::NodeHandle& nh, std::string name, float x, float y, float z):
    nh(nh),
    name(name),
    x(x),
    y(y),
    z(z)
    {};
    virtual std::string getName() = 0;
    virtual std::tuple <float, float, float> getCoordinates() = 0;
    ~Sensor() = default;
    virtual bool isTriggered() = 0;
};
class bumper : public Sensor
{
    enum state {pressed, released};
    state bumper_state;
    ros::Subscriber bumper_sub;
    // TODO make signals and notify when triggered

    void bumperCallback(const std_msgs::Bool::ConstPtr& msg);

public:
    bumper(ros::NodeHandle& nh, std::string name, float x, float y, float z);
    ~bumper()=default;
    bool isTriggered();
    std::tuple <float, float, float>  getCoordinates() override;
    std::string getName() override;
};


#endif // SENSORS_H
